import copy
import json
import math
import random
from pathlib import Path

import pytest

from senseglove_finger_ratio import (
    FINGER_NAMES,
    JOINT_KEYS,
    STATE_ACTIVE,
    STATE_STALE,
    STATE_WAITING,
    InvalidCalibration,
    InvalidJointState,
    SenseGloveRatioPipeline,
    build_calibration,
    compute_finger_ratios,
    extract_joint_values,
    load_calibration,
    ratios_to_list,
    summarize_samples,
    validate_calibration,
)


ROOT = Path(__file__).resolve().parent.parent
FIXTURES = json.loads((ROOT / "tests" / "fixtures" / "senseglove_poses.json").read_text())["poses"]
ANATOMICAL = ROOT / "senseglove_calibration_anatomical.json"

EXTRA_JOINTS = [
    "r_thumb_brake", "r_index_brake", "r_middle_brake", "r_ring_brake",
    "r_pinky_brake", "r_pinky_mcp", "r_pinky_pip", "r_pinky_dip",
    "r_palm_index", "r_palm_pinky", "r_palm_strap",
]


def pose_joint_map(pose):
    joints = {}
    for finger in FINGER_NAMES:
        for key, value in zip(JOINT_KEYS, pose[finger]):
            joints[f"r_{finger}_{key}"] = float(value)
    for name in EXTRA_JOINTS:
        joints[name] = 0.0
    return joints


def to_msg(joints, order=None):
    names = list(order) if order is not None else sorted(joints)
    return names, [joints[n] for n in names]


def ratios_for(joints, calibration, order=None):
    names, positions = to_msg(joints, order)
    values = extract_joint_values(names, positions, calibration)
    ratios, _ = compute_finger_ratios(values, calibration)
    return ratios


@pytest.fixture
def anatomical():
    return load_calibration(ANATOMICAL)


@pytest.fixture
def recorded(anatomical):
    """A calibration built the way the recorder builds it: from many noisy
    frames around open and fist, not from a single hard-coded snapshot."""
    rng = random.Random(1)

    def noisy_frames(pose, count=60):
        frames = []
        for _ in range(count):
            joints = {k: v + rng.gauss(0.0, 0.01) for k, v in pose_joint_map(pose).items()}
            frames.append(to_msg(joints))
        return frames

    open_summary = summarize_samples(noisy_frames(FIXTURES["open"]), anatomical)
    fist_summary = summarize_samples(noisy_frames(FIXTURES["fist"]), anatomical)
    return build_calibration(anatomical, open_summary, fist_summary, min_span=0.1)


def make_pipeline(calibration, stale_timeout=0.3, max_ratio=1.0, max_step=1.0):
    return SenseGloveRatioPipeline(calibration, stale_timeout, max_ratio, max_step)


# 1. joint name order ---------------------------------------------------------

def test_joint_order_does_not_change_result(anatomical):
    joints = pose_joint_map(FIXTURES["pinch"])
    reference = ratios_for(joints, anatomical)

    rng = random.Random(0)
    for _ in range(20):
        order = list(joints)
        rng.shuffle(order)
        assert ratios_for(joints, anatomical, order) == reference


# 2-3. open / fist direction --------------------------------------------------

def test_open_pose_gives_small_ratio_uncalibrated(anatomical):
    ratios = ratios_for(pose_joint_map(FIXTURES["open"]), anatomical)
    assert all(r < 0.25 for r in ratios.values()), ratios


def test_fist_pose_gives_large_ratio_uncalibrated(anatomical):
    ratios = ratios_for(pose_joint_map(FIXTURES["fist"]), anatomical)
    assert ratios["index"] > 0.8 and ratios["middle"] > 0.8 and ratios["ring"] > 0.8, ratios
    assert ratios["thumb"] > 0.5, ratios


def test_recorded_calibration_maps_open_near_0_and_fist_near_1(recorded):
    open_ratios = ratios_for(pose_joint_map(FIXTURES["open"]), recorded)
    fist_ratios = ratios_for(pose_joint_map(FIXTURES["fist"]), recorded)

    assert all(r < 0.05 for r in open_ratios.values()), open_ratios
    assert all(r > 0.95 for r in fist_ratios.values()), fist_ratios


def test_partial_poses_fall_between_open_and_fist(recorded):
    for name in ("index_bend", "pinch"):
        ratios = ratios_for(pose_joint_map(FIXTURES[name]), recorded)
        assert all(0.0 <= r <= 1.0 for r in ratios.values())
        assert 0.1 < ratios["index"] < 1.0, (name, ratios)


# 4. output bounds ------------------------------------------------------------

@pytest.mark.parametrize("calibration_name", ["anatomical", "recorded"])
def test_ratios_always_within_0_and_1(calibration_name, request):
    calibration = request.getfixturevalue(calibration_name)
    rng = random.Random(2)
    base = pose_joint_map(FIXTURES["open"])

    for _ in range(2000):
        joints = {k: rng.uniform(-math.pi, math.pi) for k in base}
        for r in ratios_for(joints, calibration).values():
            assert 0.0 <= r <= 1.0


def test_pipeline_output_respects_max_ratio_and_step(anatomical):
    pipeline = make_pipeline(anatomical, max_ratio=0.7, max_step=0.05)
    names, positions = to_msg(pose_joint_map(FIXTURES["fist"]))

    previous = [0.0] * 4
    for tick in range(40):
        now = tick / 15.0
        pipeline.on_joint_state(names, positions, now)
        output = ratios_to_list(pipeline.step(now))

        for prev, cur in zip(previous, output):
            assert 0.0 <= cur <= 0.7 + 1e-9
            assert abs(cur - prev) <= 0.05 + 1e-9
        previous = output

    assert max(previous) == pytest.approx(0.7)


# 5. missing / malformed messages ----------------------------------------------

def test_missing_required_joint_is_rejected(anatomical):
    joints = pose_joint_map(FIXTURES["open"])
    del joints["r_middle_pip"]
    names, positions = to_msg(joints)

    with pytest.raises(InvalidJointState, match="r_middle_pip"):
        extract_joint_values(names, positions, anatomical)


def test_name_position_length_mismatch_is_rejected(anatomical):
    names, positions = to_msg(pose_joint_map(FIXTURES["open"]))

    with pytest.raises(InvalidJointState, match="length mismatch"):
        extract_joint_values(names, positions[:-1], anatomical)


def test_duplicate_joint_name_is_rejected(anatomical):
    names, positions = to_msg(pose_joint_map(FIXTURES["open"]))
    names[0] = "r_index_mcp"

    with pytest.raises(InvalidJointState):
        extract_joint_values(names, positions, anatomical)


def test_left_hand_message_is_rejected_by_right_hand_calibration(anatomical):
    joints = {k.replace("r_", "l_", 1): v for k, v in pose_joint_map(FIXTURES["open"]).items()}
    names, positions = to_msg(joints)

    with pytest.raises(InvalidJointState, match="missing joints"):
        extract_joint_values(names, positions, anatomical)


def test_rejected_message_does_not_change_latest_ratios(anatomical):
    pipeline = make_pipeline(anatomical)
    pipeline.on_joint_state(*to_msg(pose_joint_map(FIXTURES["open"])), now=0.0)
    before = dict(pipeline.latest_ratios)

    joints = pose_joint_map(FIXTURES["fist"])
    del joints["r_thumb_dip"]
    with pytest.raises(InvalidJointState):
        pipeline.on_joint_state(*to_msg(joints), now=0.1)

    assert pipeline.latest_ratios == before


# 6. NaN / Inf / implausible values ---------------------------------------------

@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
@pytest.mark.parametrize("joint", ["r_thumb_mcp", "r_index_pip", "r_ring_dip"])
def test_non_finite_value_is_rejected(anatomical, joint, bad):
    joints = pose_joint_map(FIXTURES["open"])
    joints[joint] = bad

    with pytest.raises(InvalidJointState, match="non-finite"):
        extract_joint_values(*to_msg(joints), anatomical)


def test_implausible_value_is_rejected(anatomical):
    joints = pose_joint_map(FIXTURES["open"])
    joints["r_index_mcp"] = 10.0

    with pytest.raises(InvalidJointState, match="implausible"):
        extract_joint_values(*to_msg(joints), anatomical)


def test_unused_joints_are_not_validated(anatomical):
    # Pinky, brake and palm entries are not part of the LEAP V1 mapping.
    joints = pose_joint_map(FIXTURES["open"])
    joints["r_pinky_pip"] = float("nan")
    joints["r_palm_strap"] = 99.0

    extract_joint_values(*to_msg(joints), anatomical)


# 7. stale watchdog --------------------------------------------------------------

def test_waiting_before_first_message_outputs_open(anatomical):
    pipeline = make_pipeline(anatomical)
    assert ratios_to_list(pipeline.step(0.0)) == [0.0] * 4
    assert pipeline.state == STATE_WAITING


def test_stale_input_ramps_output_to_open(anatomical):
    pipeline = make_pipeline(anatomical, stale_timeout=0.3, max_ratio=0.7, max_step=0.05)
    fist = to_msg(pose_joint_map(FIXTURES["fist"]))

    now = 0.0
    for _ in range(30):
        pipeline.on_joint_state(*fist, now=now)
        pipeline.step(now)
        now += 1.0 / 15.0
    assert pipeline.state == STATE_ACTIVE
    held = ratios_to_list(pipeline.output)
    assert max(held) > 0.6

    # Glove stops. Within the timeout the last value is still used...
    last_msg_time = now - 1.0 / 15.0
    output = pipeline.step(last_msg_time + 0.25)
    assert pipeline.state == STATE_ACTIVE
    assert ratios_to_list(output) == held

    # ...after it the output ramps to open one max_step at a time, never jumping.
    now = last_msg_time + 0.35
    previous = held
    for _ in range(30):
        output = ratios_to_list(pipeline.step(now))
        assert pipeline.state == STATE_STALE
        for prev, cur in zip(previous, output):
            assert cur <= prev
            assert prev - cur <= 0.05 + 1e-9
        previous = output
        now += 1.0 / 15.0

    assert previous == [0.0] * 4


def test_holding_the_same_pose_stays_active(anatomical):
    # The watchdog tracks message arrival, not value changes: a hand held
    # still for 3 s at 60 Hz must keep its pose, not drift to open.
    pipeline = make_pipeline(anatomical, stale_timeout=0.3, max_ratio=0.7, max_step=0.05)
    fist = to_msg(pose_joint_map(FIXTURES["fist"]))

    for i in range(180):
        now = i / 60.0
        pipeline.on_joint_state(*fist, now=now)
        if i % 4 == 0:
            output = pipeline.step(now)
            assert pipeline.state == STATE_ACTIVE

    assert ratios_to_list(output) == pytest.approx([0.55, 0.7, 0.7, 0.7], abs=0.01)


def test_stream_of_invalid_messages_counts_as_stale(anatomical):
    pipeline = make_pipeline(anatomical, stale_timeout=0.3)
    pipeline.on_joint_state(*to_msg(pose_joint_map(FIXTURES["fist"])), now=0.0)

    bad = pose_joint_map(FIXTURES["fist"])
    bad["r_index_pip"] = float("nan")
    for i in range(1, 10):
        with pytest.raises(InvalidJointState):
            pipeline.on_joint_state(*to_msg(bad), now=i * 0.05)

    pipeline.step(0.45)
    assert pipeline.state == STATE_STALE


def test_valid_message_after_stale_resumes(anatomical):
    pipeline = make_pipeline(anatomical, stale_timeout=0.3)
    pipeline.on_joint_state(*to_msg(pose_joint_map(FIXTURES["fist"])), now=0.0)
    pipeline.step(1.0)
    assert pipeline.state == STATE_STALE

    pipeline.on_joint_state(*to_msg(pose_joint_map(FIXTURES["fist"])), now=1.1)
    pipeline.step(1.1)
    assert pipeline.state == STATE_ACTIVE


# 8. output order -----------------------------------------------------------------

@pytest.mark.parametrize("finger_index, finger", list(enumerate(FINGER_NAMES)))
def test_output_order_is_thumb_index_middle_ring(recorded, finger_index, finger):
    joints = pose_joint_map(FIXTURES["open"])
    for key, value in zip(JOINT_KEYS, FIXTURES["fist"][finger]):
        joints[f"r_{finger}_{key}"] = value

    output = ratios_to_list(ratios_for(joints, recorded))

    assert output[finger_index] > 0.9
    assert all(v < 0.1 for i, v in enumerate(output) if i != finger_index), output


def test_index_bend_pose_flexes_index_most(recorded):
    output = ratios_to_list(ratios_for(pose_joint_map(FIXTURES["index_bend"]), recorded))
    assert output[1] == max(output)


# calibration handling --------------------------------------------------------------

def test_weights_are_applied(anatomical):
    calibration = copy.deepcopy(anatomical)
    calibration["fingers"]["index"]["weights"] = {"mcp": 1.0, "pip": 0.0, "dip": 0.0}

    joints = pose_joint_map(FIXTURES["open"])
    joints["r_index_mcp"] = 0.7854  # half of the 1.5708 closed value
    joints["r_index_pip"] = 1.7453
    joints["r_index_dip"] = 1.5708

    assert ratios_for(joints, calibration)["index"] == pytest.approx(0.5, abs=1e-3)


def test_reversed_calibration_direction_is_supported(anatomical):
    calibration = copy.deepcopy(anatomical)
    for key in JOINT_KEYS:
        cfg = calibration["fingers"]["ring"]["joints"][key]
        cfg["open"], cfg["closed"] = -cfg["open"], -cfg["closed"]

    joints = pose_joint_map(FIXTURES["open"])
    for key, value in zip(JOINT_KEYS, FIXTURES["fist"]["ring"]):
        joints[f"r_ring_{key}"] = -value

    assert ratios_for(joints, calibration)["ring"] > 0.99


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda c: c["fingers"]["index"]["joints"]["pip"].update(closed=0.0), "too close"),
        (lambda c: c["fingers"].pop("ring"), "missing finger"),
        (lambda c: c["fingers"]["thumb"]["weights"].update(mcp=-1.0), "weights"),
        (lambda c: c["fingers"]["middle"]["joints"]["dip"].update(open=float("nan")), "finite"),
        (lambda c: c.update(max_abs_joint_rad=0.0), "max_abs_joint_rad"),
    ],
)
def test_invalid_calibration_is_rejected(anatomical, mutate, message):
    calibration = copy.deepcopy(anatomical)
    mutate(calibration)

    with pytest.raises(InvalidCalibration, match=message):
        validate_calibration(calibration)


def test_recorder_rejects_joint_that_did_not_move(anatomical):
    frames = [to_msg(pose_joint_map(FIXTURES["open"]))] * 10
    summary = summarize_samples(frames, anatomical)

    with pytest.raises(InvalidCalibration, match="span"):
        build_calibration(anatomical, summary, summary, min_span=0.1)


def test_recorder_median_ignores_single_glitch_frame(anatomical):
    frames = [to_msg(pose_joint_map(FIXTURES["open"]))] * 20
    glitch = pose_joint_map(FIXTURES["open"])
    glitch["r_index_mcp"] = 2.5
    frames.append(to_msg(glitch))

    summary = summarize_samples(frames, anatomical)
    assert summary["index"]["mcp"]["median"] == pytest.approx(-0.166)


def test_recorder_rejects_invalid_frames(anatomical):
    bad = pose_joint_map(FIXTURES["open"])
    bad["r_index_mcp"] = float("nan")

    with pytest.raises(InvalidJointState):
        summarize_samples([to_msg(bad)], anatomical)
