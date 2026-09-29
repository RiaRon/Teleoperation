import copy
import json
import math
import sys
import types
from pathlib import Path

import pytest

from right_hand_controller_core import (
    ALL_IDS,
    FINGER_NAMES,
    ArmRefused,
    DryRunBus,
    InvalidConfig,
    InvalidRatios,
    RightHandControllerCore,
    compute_targets_deg,
    deg_to_tick,
    hardware_blockers,
    load_hand_config,
    target_blockers,
    tick_to_deg,
    validate_hand_config,
    validate_ratios,
)


ROOT = Path(__file__).resolve().parent.parent
REAL_CONFIG = ROOT / "config" / "right_hand.json"
SYNTHETIC_CONFIG = ROOT / "tests" / "fixtures" / "right_hand_synthetic_test.json"
LEFT_MAP = ROOT / "left_hand_motor_map.json"

OPEN = 0.0
CLOSED = 1.0


@pytest.fixture
def config():
    return load_hand_config(SYNTHETIC_CONFIG)


def open_deg(config, dxl_id):
    return config["open_pose_deg"][str(dxl_id)]


def close_deg(config, dxl_id):
    return config["close_pose_deg"][str(dxl_id)]


def make_core(config, initial=None, watchdog_timeout=0.5, max_step_deg=3.0):
    bus = DryRunBus(initial or config["open_pose_deg"])
    return RightHandControllerCore(config, bus, watchdog_timeout, max_step_deg), bus


def goal_writes(bus):
    return [c[1] for c in bus.calls if c[0] == "goal"]


def ratios(thumb=0.0, index=0.0, middle=0.0, ring=0.0):
    return {"thumb": thumb, "index": index, "middle": middle, "ring": ring}


# 1-3. interpolation --------------------------------------------------------------

def test_ratio_0_gives_right_open_pose(config):
    targets = compute_targets_deg(config, ratios())
    assert targets == {i: open_deg(config, i) for i in ALL_IDS}


def test_ratio_1_gives_right_close_pose(config):
    targets = compute_targets_deg(config, ratios(1, 1, 1, 1))
    assert targets == {i: close_deg(config, i) for i in ALL_IDS}


def test_ratio_half_gives_midpoint(config):
    targets = compute_targets_deg(config, ratios(0.5, 0.5, 0.5, 0.5))
    for i in ALL_IDS:
        assert targets[i] == pytest.approx((open_deg(config, i) + close_deg(config, i)) / 2)


# 4-7. input validation -------------------------------------------------------------

@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
@pytest.mark.parametrize("position", range(4))
def test_non_finite_ratio_is_rejected(config, bad, position):
    core, bus = make_core(config)
    core.arm(0.0)
    calls_before = len(bus.calls)
    data = [0.2] * 4
    data[position] = bad

    with pytest.raises(InvalidRatios, match="not finite"):
        core.on_ratios(data, 0.1)

    assert len(bus.calls) == calls_before
    assert core.last_valid_time is None


@pytest.mark.parametrize("data", [[], [0.1, 0.2, 0.3], [0.1] * 5, ["a", 0, 0, 0], [None, 0, 0, 0]])
def test_malformed_ratio_message_is_rejected(data):
    with pytest.raises(InvalidRatios):
        validate_ratios(data)


def test_ratio_is_clamped_after_finite_check(config):
    assert validate_ratios([-0.5, 1.7, 0.3, 1.0]) == ratios(0.0, 1.0, 0.3, 1.0)

    targets = compute_targets_deg(config, validate_ratios([-3.0, 9.0, 0.0, 0.0]))
    for i in config["finger_ids"]["thumb"]:
        assert targets[i] == open_deg(config, i)
    for i in config["finger_ids"]["index"]:
        assert targets[i] == close_deg(config, i)


# 8. limits ------------------------------------------------------------------------

def test_config_with_pose_outside_limit_is_rejected(config):
    bad = copy.deepcopy(config)
    bad["close_pose_deg"]["2"] = bad["limits_deg"]["2"][1] + 1.0

    with pytest.raises(InvalidConfig, match="outside limits"):
        validate_hand_config(bad)


def test_targets_are_clamped_to_limits(config):
    narrow = copy.deepcopy(config)
    narrow["limits_deg"]["2"] = [170.0, 200.0]  # tighter than close 250 after validation
    targets = compute_targets_deg(narrow, ratios(index=1.0))
    assert targets[2] == 200.0


def test_arm_hold_goal_is_clamped_into_limits(config):
    present = {str(i): open_deg(config, i) for i in ALL_IDS}
    low = config["limits_deg"]["0"][0]
    present["0"] = low - 3.0  # within the 5 deg arm tolerance
    core, bus = make_core(config, initial=present)

    core.arm(0.0)

    assert tick_to_deg(goal_writes(bus)[0][0]) == pytest.approx(low, abs=0.1)


def test_arm_refused_when_present_far_outside_limits(config):
    present = {str(i): open_deg(config, i) for i in ALL_IDS}
    present["13"] = 20.0  # e.g. wrong ID map or remounted horn
    core, bus = make_core(config, initial=present)

    with pytest.raises(ArmRefused, match="ID 13"):
        core.arm(0.0)

    assert not core.armed
    assert all(c[0] == "read_present" for c in bus.calls)


def test_limits_must_stay_inside_one_turn(config):
    bad = copy.deepcopy(config)
    bad["limits_deg"]["5"] = [-10.0, 200.0]
    with pytest.raises(InvalidConfig, match="0 < min < max < 360"):
        validate_hand_config(bad)


# 9. finger -> motor mapping ------------------------------------------------------------

@pytest.mark.parametrize("finger", FINGER_NAMES)
def test_each_ratio_moves_only_its_own_motors(config, finger):
    targets = compute_targets_deg(config, ratios(**{finger: 1.0}))
    moved = {i for i in ALL_IDS if targets[i] != open_deg(config, i)}
    expected = {i for i in config["finger_ids"][finger] if close_deg(config, i) != open_deg(config, i)}

    assert moved == expected
    assert moved <= set(config["finger_ids"][finger])


def test_right_mapping_follows_official_v1_table():
    config = load_hand_config(REAL_CONFIG)
    assert config["finger_ids"] == {
        "thumb": [12, 13, 14, 15],
        "index": [0, 1, 2, 3],
        "middle": [4, 5, 6, 7],
        "ring": [8, 9, 10, 11],
    }
    assert config["joint_names"]["0"] == "index_mcp_side"
    assert config["joint_names"]["9"] == "ring_mcp_forward"


# 10. left/right separation ---------------------------------------------------------------

def test_left_motor_map_is_rejected_by_right_loader():
    with pytest.raises(InvalidConfig, match="hand must be 'right'"):
        load_hand_config(LEFT_MAP)


def test_right_config_does_not_copy_left_values():
    right = json.loads(REAL_CONFIG.read_text())
    left = json.loads(LEFT_MAP.read_text())

    assert right["finger_ids"] != left["ids"]
    assert right["open_pose_deg"] != left["open_pose_deg"]
    assert all(v is None for v in right["close_pose_deg"].values())
    assert right["baudrate"] is None


def test_right_and_left_topics_are_separate():
    import right_hand_finger_ratio_controller as right
    source = (ROOT / "left_hand_finger_ratio_node_safe.py").read_text()

    assert right.FINGER_TOPIC == "/right_hand/finger_ratios"
    assert right.ARM_TOPIC == "/right_hand/arm"
    assert 'FINGER_TOPIC = "/left_hand/finger_ratios"' in source
    assert 'ARM_TOPIC = "/left_hand/arm"' in source


# 11. watchdog ------------------------------------------------------------------------------

def test_no_motor_command_before_arm(config):
    core, bus = make_core(config)
    for k in range(10):
        core.on_ratios([1, 1, 1, 1], k * 0.066)
        core.tick(k * 0.066)

    assert bus.calls == []
    assert core.target_deg is not None


def test_watchdog_turns_torque_off_when_ratios_stop(config):
    core, bus = make_core(config, watchdog_timeout=0.5)
    core.arm(0.0)
    for k in range(1, 10):
        core.on_ratios([0.2] * 4, k / 15.0)
        assert core.tick(k / 15.0) is False

    last = 9 / 15.0
    assert core.tick(last + 0.45) is False
    assert core.armed
    assert core.tick(last + 0.55) is True

    assert not core.armed
    assert bus.calls[-1] == ("torque_off",)
    assert "watchdog" in core.last_disarm_reason


def test_watchdog_counts_nan_stream_as_silence(config):
    core, bus = make_core(config, watchdog_timeout=0.5)
    core.arm(0.0)
    core.on_ratios([0.1] * 4, 0.05)

    for k in range(2, 12):
        with pytest.raises(InvalidRatios):
            core.on_ratios([float("nan")] * 4, k * 0.066)

    assert core.tick(0.6) is True
    assert bus.torque == {i: False for i in ALL_IDS}


def test_watchdog_disarms_if_no_ratio_arrives_after_arm(config):
    core, _ = make_core(config, watchdog_timeout=0.5)
    core.on_ratios([0.0] * 4, 0.0)  # before ARM, must not count as fresh
    core.arm(10.0)

    assert core.tick(10.4) is False
    assert core.tick(10.6) is True


def test_watchdog_is_idle_while_disarmed(config):
    core, bus = make_core(config)
    assert core.tick(100.0) is False
    assert bus.calls == []


# startup jump protection ------------------------------------------------------------------------

def test_arm_holds_present_pose_then_ramps(config):
    core, bus = make_core(config, max_step_deg=3.0)
    core.arm(0.0)

    hold = goal_writes(bus)[0]
    assert hold == {i: deg_to_tick(open_deg(config, i)) for i in ALL_IDS}
    assert ("torque_on",) in bus.calls and ("profile_velocity", 8) in bus.calls

    first = core.on_ratios([1, 1, 1, 1], 0.07)
    for i in ALL_IDS:
        assert abs(first[i] - open_deg(config, i)) <= 3.0 + 1e-9

    for k in range(2, 40):
        command = core.on_ratios([1, 1, 1, 1], k * 0.066)
    assert command == pytest.approx({i: close_deg(config, i) for i in ALL_IDS})


def test_arm_refused_while_close_pose_is_tbd():
    config = load_hand_config(REAL_CONFIG)
    core, bus = make_core(config)

    with pytest.raises(ArmRefused, match="close_pose_deg TBD"):
        core.arm(0.0)
    assert core.on_ratios([0.5] * 4, 0.1) is None
    assert bus.calls == []


# config blockers ------------------------------------------------------------------------------

def test_real_config_blocks_hardware_until_verified():
    config = load_hand_config(REAL_CONFIG)
    blockers = " | ".join(hardware_blockers(config))

    for expected in ["close_pose_deg TBD", "port TBD", "baudrate TBD", "not verified"]:
        assert expected in blockers
    assert target_blockers(config) == ["close_pose_deg TBD for IDs 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15"]


def test_synthetic_config_can_never_drive_hardware(config):
    assert hardware_blockers(config) == ["config_kind is 'synthetic_test', not 'hardware'"]


def test_complete_verified_config_has_no_blockers(config):
    ready = copy.deepcopy(config)
    ready["config_kind"] = "hardware"
    assert hardware_blockers(ready) == []

    ready["hardware_verified"]["direction"] = False
    assert hardware_blockers(ready) == ["not verified on the real right hand: direction"]


def test_duplicate_or_missing_ids_are_rejected(config):
    bad = copy.deepcopy(config)
    bad["finger_ids"]["ring"] = [8, 9, 10, 0]
    with pytest.raises(InvalidConfig, match="exactly once"):
        validate_hand_config(bad)


def test_tick_conversion_stays_in_one_turn():
    assert deg_to_tick(180.0) == 2048
    assert tick_to_deg(2048) == 180.0
    with pytest.raises(ValueError):
        deg_to_tick(360.0)
    with pytest.raises(ValueError):
        deg_to_tick(-1.0)


# 12. shutdown ---------------------------------------------------------------------------------

def test_shutdown_turns_torque_off_and_closes(config):
    core, bus = make_core(config)
    core.arm(0.0)
    core.shutdown()

    assert bus.calls[-2:] == [("torque_off",), ("close",)]
    assert not core.armed and not bus.opened


def test_shutdown_closes_port_even_if_torque_off_fails(config):
    core, bus = make_core(config)
    core.arm(0.0)

    def fail():
        raise RuntimeError("bus error")

    bus.torque_off = fail
    with pytest.raises(RuntimeError):
        core.shutdown()
    assert bus.calls[-1] == ("close",)


def test_disarm_always_turns_torque_off(config):
    core, bus = make_core(config)
    core.disarm("DISARM requested")
    assert bus.calls == [("torque_off",)]


def test_arm_failure_turns_torque_off(config):
    core, bus = make_core(config)

    def fail(_goal):
        raise RuntimeError("sync write failed")

    bus.write_goal_ticks = fail
    with pytest.raises(RuntimeError):
        core.arm(0.0)
    assert not core.armed
    assert bus.calls[-1] == ("torque_off",)


# real bus layer with a fake SDK (no serial port) -----------------------------------------------

class FakePacket:
    def __init__(self, fail_ids=()):
        self.fail_ids = set(fail_ids)
        self.writes = []

    def write1ByteTxRx(self, port, dxl_id, address, value):
        self.writes.append((dxl_id, address, value))
        return (1, 0) if dxl_id in self.fail_ids else (0, 0)

    def getTxRxResult(self, result):
        return "fake comm error"

    def getRxPacketError(self, error):
        return "fake packet error"


@pytest.fixture
def fake_sdk(monkeypatch):
    sdk = types.ModuleType("dynamixel_sdk")
    sdk.COMM_SUCCESS = 0
    sdk.PortHandler = lambda port: types.SimpleNamespace(closePort=lambda: None)
    sdk.PacketHandler = lambda version: FakePacket()
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", sdk)
    return sdk


def test_bus_torque_off_tries_every_motor_even_after_failure(fake_sdk):
    from right_hand_dynamixel_bus import ADDR_TORQUE_ENABLE, DynamixelBus

    bus = DynamixelBus("/dev/fake", 57600, ALL_IDS)
    bus.packet = FakePacket(fail_ids={3})

    with pytest.raises(RuntimeError, match="ID 3"):
        bus.torque_off()
    assert [w[0] for w in bus.packet.writes] == ALL_IDS
    assert all(w[1:] == (ADDR_TORQUE_ENABLE, 0) for w in bus.packet.writes)


def test_present_position_is_read_as_signed_32bit():
    from right_hand_dynamixel_bus import to_signed32

    assert to_signed32(2048) == 2048
    assert to_signed32(0xFFFFFFFF) == -1
    assert to_signed32(4096 + 2048) == 6144  # multi-turn stays visible, not wrapped


def test_hardware_mode_refuses_before_touching_the_port(monkeypatch, capsys):
    pytest.importorskip("rclpy")
    import right_hand_finger_ratio_controller as node

    monkeypatch.setitem(sys.modules, "dynamixel_sdk", None)  # any import would fail
    monkeypatch.setattr(sys, "argv", ["x", "--mode", "hardware", "--config", str(REAL_CONFIG)])

    assert node.main() == 2
    out = capsys.readouterr().out
    assert "hardware mode refused, port not opened" in out
    assert "close_pose_deg TBD" in out
