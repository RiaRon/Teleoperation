"""Fake glove -> SenseGlove ratio pipeline -> right controller -> motor targets.

Everything runs in-process on a simulated 15 Hz clock with DryRunBus; no ROS
graph, no serial port. The ratio side uses the existing, unchanged
senseglove_finger_ratio module exactly as senseglove_finger_ratio_pub.py does.
"""

import json
from pathlib import Path

import pytest

from right_hand_controller_core import ALL_IDS, DryRunBus, RightHandControllerCore, load_hand_config
from senseglove_finger_ratio import SenseGloveRatioPipeline, load_calibration, ratios_to_list


ROOT = Path(__file__).resolve().parent.parent
POSES = json.loads((ROOT / "tests" / "fixtures" / "senseglove_poses.json").read_text())["poses"]
PERIOD = 1.0 / 15.0


def glove_message(pose):
    joints = {}
    for finger in ["thumb", "index", "middle", "ring"]:
        for key, value in zip(["mcp", "pip", "dip"], pose[finger]):
            joints[f"r_{finger}_{key}"] = value
    for extra in ["r_pinky_mcp", "r_pinky_pip", "r_pinky_dip", "r_palm_strap"]:
        joints[extra] = 0.0
    names = sorted(joints, reverse=True)
    return names, [joints[n] for n in names]


@pytest.fixture
def system():
    config = load_hand_config(ROOT / "tests" / "fixtures" / "right_hand_synthetic_test.json")
    glove = SenseGloveRatioPipeline(
        load_calibration(ROOT / "senseglove_calibration_anatomical.json"),
        stale_timeout=0.3,
        max_ratio=0.7,
        max_step=0.05,
    )
    bus = DryRunBus(config["open_pose_deg"])
    controller = RightHandControllerCore(config, bus, watchdog_timeout=0.5, max_step_deg=3.0)
    return config, glove, controller, bus


def run(glove, controller, pose, start, seconds, publish=True):
    """Glove at 60 Hz (pose=None means disconnected), ratio node + controller at 15 Hz."""
    now = start
    command = None
    for _ in range(int(seconds / PERIOD)):
        if pose is not None:
            for k in range(4):
                glove.on_joint_state(*glove_message(pose), now=now + k / 60.0)
        now += PERIOD
        output = ratios_to_list(glove.step(now))
        if publish:
            command = controller.on_ratios(output, now) or command
        controller.tick(now)
    return now, output, command


def test_fist_moves_right_motors_toward_close_within_limits(system):
    config, glove, controller, bus = system

    now, _, _ = run(glove, controller, POSES["open"], 0.0, 1.0)
    controller.arm(now)
    now, output, command = run(glove, controller, POSES["fist"], now, 4.0)

    # max_ratio 0.7 caps the glove output; the fist sample gives thumb 0.55.
    assert output == pytest.approx([0.55, 0.7, 0.7, 0.7], abs=0.01)
    for i in config["finger_ids"]["index"]:
        o, c = config["open_pose_deg"][str(i)], config["close_pose_deg"][str(i)]
        assert command[i] == pytest.approx(o + 0.7 * (c - o), abs=0.5)
    for i in ALL_IDS:
        low, high = config["limits_deg"][str(i)]
        assert low <= command[i] <= high

    steps = [goal for kind, *goal in bus.calls if kind == "goal"]
    assert len(steps) > 10


def test_glove_disconnect_ramps_right_hand_back_to_open(system):
    config, glove, controller, bus = system

    controller.arm(0.0)
    now, _, _ = run(glove, controller, POSES["fist"], 0.0, 4.0)
    now, output, command = run(glove, controller, None, now, 3.0)

    assert output == [0.0] * 4
    assert controller.armed  # ratio node keeps publishing, so the controller stays armed
    assert command == pytest.approx({i: config["open_pose_deg"][str(i)] for i in ALL_IDS}, abs=0.01)


def test_ratio_node_death_disarms_right_controller(system):
    config, glove, controller, bus = system

    controller.arm(0.0)
    now, _, _ = run(glove, controller, POSES["fist"], 0.0, 2.0)
    now, _, _ = run(glove, controller, POSES["fist"], now, 1.0, publish=False)

    assert not controller.armed
    assert bus.torque == {i: False for i in ALL_IDS}
    assert "watchdog" in controller.last_disarm_reason


def test_nothing_is_written_until_arm(system):
    _, glove, controller, bus = system
    run(glove, controller, POSES["fist"], 0.0, 2.0)
    assert bus.calls == []
