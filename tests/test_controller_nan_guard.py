"""Checks the NaN/Inf guard in left_hand_finger_ratio_node_safe.finger_callback.

No hardware: dynamixel_sdk is stubbed and the callback runs on a fake node
object that records what would have been written to the motors.
"""

import json
import sys
import types
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
from std_msgs.msg import Float32MultiArray  # noqa: E402


ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture
def controller(monkeypatch):
    stub = types.ModuleType("dynamixel_sdk")
    for name in ["PortHandler", "PacketHandler", "GroupSyncWrite"]:
        setattr(stub, name, object)
    stub.COMM_SUCCESS = 0
    for name in ["DXL_LOBYTE", "DXL_HIBYTE", "DXL_LOWORD", "DXL_HIWORD"]:
        setattr(stub, name, lambda value: value)
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", stub)
    monkeypatch.delitem(sys.modules, "left_hand_finger_ratio_node_safe", raising=False)

    import left_hand_finger_ratio_node_safe

    return left_hand_finger_ratio_node_safe


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def info(self, text):
        pass

    def warn(self, text):
        self.warnings.append(text)

    def error(self, text):
        raise AssertionError(f"unexpected error log: {text}")


def make_fake_node(controller, armed=True):
    logger = FakeLogger()
    sent = []
    node = types.SimpleNamespace(
        armed=armed,
        last_ratios={"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0},
        motor_map=json.loads((ROOT / "left_hand_motor_map.json").read_text()),
        get_logger=lambda: logger,
        sync_write_goal_deg=sent.append,
    )
    return node, logger, sent


def call(controller, node, data):
    msg = Float32MultiArray()
    msg.data = data
    controller.LeftHandSafeFingerRatioNode.finger_callback(node, msg)


def test_clamp_itself_turns_nan_into_full_fist(controller):
    # The reason for the guard: without it NaN would command a closed hand.
    assert controller.clamp(float("nan")) == 1.0


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
@pytest.mark.parametrize("position", [0, 1, 2, 3])
def test_non_finite_ratio_is_ignored(controller, bad, position):
    node, logger, sent = make_fake_node(controller)
    data = [0.2, 0.2, 0.2, 0.2]
    data[position] = bad

    call(controller, node, data)

    assert sent == []
    assert node.last_ratios == {"thumb": 0.0, "index": 0.0, "middle": 0.0, "ring": 0.0}
    assert any("NaN/Inf" in w for w in logger.warnings)


def test_valid_ratio_is_still_sent(controller):
    node, _, sent = make_fake_node(controller)

    call(controller, node, [0.0, 0.5, 0.5, 1.0])

    assert len(sent) == 1
    assert sent[0] == controller.make_target_pose(
        node.motor_map, {"thumb": 0.0, "index": 0.5, "middle": 0.5, "ring": 1.0}
    )
