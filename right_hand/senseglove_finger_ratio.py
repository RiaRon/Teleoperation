"""SenseGlove JointState -> LEAP finger ratios, without any ROS dependency.

Output order and meaning match the LEAP finger-ratio topics
(/right_hand/finger_ratios for right_hand_finger_ratio_controller.py):
[thumb, index, middle, ring], 0.0 = open, 1.0 = fully flexed.
"""

import json
import math
import statistics


FINGER_NAMES = ["thumb", "index", "middle", "ring"]
JOINT_KEYS = ["mcp", "pip", "dip"]

STATE_WAITING = "WAITING"
STATE_ACTIVE = "ACTIVE"
STATE_STALE = "STALE"

# Guard against divide-by-(almost)-zero when open and closed are the same.
MIN_CALIBRATION_SPAN_RAD = 1e-3


class InvalidJointState(ValueError):
    pass


class InvalidCalibration(ValueError):
    pass


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, float(value)))


def make_zero_ratios():
    return {name: 0.0 for name in FINGER_NAMES}


def ratios_to_list(ratios):
    return [float(ratios[name]) for name in FINGER_NAMES]


def joint_name(prefix, finger, key):
    return f"{prefix}{finger}_{key}"


def required_joint_names(calibration):
    prefix = calibration["joint_prefix"]
    return [
        joint_name(prefix, finger, key)
        for finger in FINGER_NAMES
        for key in JOINT_KEYS
    ]


def validate_calibration(calibration):
    if not isinstance(calibration.get("joint_prefix"), str):
        raise InvalidCalibration("joint_prefix must be a string")

    max_abs = calibration.get("max_abs_joint_rad")
    if not isinstance(max_abs, (int, float)) or not math.isfinite(max_abs) or max_abs <= 0.0:
        raise InvalidCalibration("max_abs_joint_rad must be a positive finite number")

    fingers = calibration.get("fingers", {})

    for finger in FINGER_NAMES:
        if finger not in fingers:
            raise InvalidCalibration(f"missing finger calibration: {finger}")

        finger_cfg = fingers[finger]
        weight_sum = 0.0

        for key in JOINT_KEYS:
            joint_cfg = finger_cfg.get("joints", {}).get(key)
            if joint_cfg is None:
                raise InvalidCalibration(f"missing joint calibration: {finger}.{key}")

            open_rad = joint_cfg.get("open")
            closed_rad = joint_cfg.get("closed")
            for label, value in (("open", open_rad), ("closed", closed_rad)):
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise InvalidCalibration(f"{finger}.{key}.{label} must be a finite number")

            if abs(closed_rad - open_rad) < MIN_CALIBRATION_SPAN_RAD:
                raise InvalidCalibration(f"{finger}.{key}: open and closed are too close")

            weight = finger_cfg.get("weights", {}).get(key, 1.0)
            if not isinstance(weight, (int, float)) or not math.isfinite(weight) or weight < 0.0:
                raise InvalidCalibration(f"{finger}.weights.{key} must be a finite number >= 0")
            weight_sum += weight

        if weight_sum <= 0.0:
            raise InvalidCalibration(f"{finger}: at least one weight must be > 0")

    return calibration


def load_calibration(path):
    with open(path, "r", encoding="utf-8") as f:
        return validate_calibration(json.load(f))


def extract_joint_values(names, positions, calibration):
    """Look up MCP/PIP/DIP by joint name; never by array index.

    Raises InvalidJointState instead of returning partial data, so a bad
    message can never turn into a command.
    """
    names = list(names)
    positions = list(positions)

    if len(names) != len(positions):
        raise InvalidJointState(
            f"name/position length mismatch: {len(names)} names, {len(positions)} positions"
        )

    if len(set(names)) != len(names):
        raise InvalidJointState("duplicate joint names")

    joint_map = dict(zip(names, positions))
    required = required_joint_names(calibration)

    missing = [name for name in required if name not in joint_map]
    if missing:
        raise InvalidJointState(f"missing joints: {', '.join(missing)}")

    max_abs = float(calibration["max_abs_joint_rad"])
    prefix = calibration["joint_prefix"]
    values = {}

    for finger in FINGER_NAMES:
        values[finger] = {}

        for key in JOINT_KEYS:
            name = joint_name(prefix, finger, key)
            value = float(joint_map[name])

            if not math.isfinite(value):
                raise InvalidJointState(f"non-finite value: {name}={value}")

            if abs(value) > max_abs:
                raise InvalidJointState(
                    f"implausible value: {name}={value:.3f} rad (limit +/-{max_abs:.3f})"
                )

            values[finger][key] = value

    return values


def normalize(value, open_rad, closed_rad):
    # Works for either sign of (closed - open).
    return clamp((value - open_rad) / (closed_rad - open_rad))


def compute_finger_ratios(joint_values, calibration):
    """Return (ratios, details).

    Each joint is normalized to 0..1 on its own open/closed range first, so a
    joint with a large range does not dominate one with a small range.

    The joints are then combined with a weighted average, equal weights by
    default. Equal weights keep this simple and close to the camera input:
    the camera ratio came from the PIP angle alone, and on the Nova 2 the
    finger DIP follows PIP (~0.9 x PIP in the recorded samples), so an equal
    MCP/PIP/DIP average is roughly 1/3 MCP + 2/3 PIP. The thumb uses the same
    formula but its own calibration entries, because its joints (CMC/MCP/IP
    on the glove, named mcp/pip/dip) move very differently from the fingers.
    """
    ratios = {}
    details = {}

    for finger in FINGER_NAMES:
        finger_cfg = calibration["fingers"][finger]
        weights = finger_cfg.get("weights", {})
        normalized = {}
        weighted_sum = 0.0
        weight_sum = 0.0

        for key in JOINT_KEYS:
            joint_cfg = finger_cfg["joints"][key]
            n = normalize(joint_values[finger][key], joint_cfg["open"], joint_cfg["closed"])
            w = float(weights.get(key, 1.0))
            normalized[key] = n
            weighted_sum += w * n
            weight_sum += w

        ratios[finger] = clamp(weighted_sum / weight_sum)
        details[finger] = {
            "raw": dict(joint_values[finger]),
            "normalized": normalized,
            "ratio": ratios[finger],
        }

    return ratios, details


def apply_max_ratio(ratios, max_ratio):
    return {name: clamp(ratios[name], 0.0, max_ratio) for name in FINGER_NAMES}


def step_limit(prev_output, target, max_step):
    # Same per-publish rate limit as left_hand_camera_finger_pub_stable.py.
    output = {}

    for name in FINGER_NAMES:
        diff = target[name] - prev_output[name]
        diff = max(-max_step, min(max_step, diff))
        output[name] = clamp(prev_output[name] + diff)

    return output


class SenseGloveRatioPipeline:
    """Glove message -> validated ratios -> watchdog -> rate-limited output.

    Time is passed in explicitly (monotonic seconds) so the watchdog can be
    tested without ROS or sleeping.
    """

    def __init__(self, calibration, stale_timeout, max_ratio, max_step):
        if stale_timeout <= 0.0:
            raise ValueError("stale_timeout must be > 0")
        if not 0.0 <= max_ratio <= 1.0:
            raise ValueError("max_ratio must be within 0..1")
        if max_step <= 0.0:
            raise ValueError("max_step must be > 0")

        self.calibration = validate_calibration(calibration)
        self.stale_timeout = float(stale_timeout)
        self.max_ratio = float(max_ratio)
        self.max_step = float(max_step)

        self.latest_ratios = None
        self.latest_details = None
        self.last_valid_time = None
        self.output = make_zero_ratios()
        self.state = STATE_WAITING

    def on_joint_state(self, names, positions, now):
        """Accept a message or raise InvalidJointState.

        A rejected message does not refresh the watchdog, so a stream of bad
        messages ends up STALE just like no messages at all.
        """
        joint_values = extract_joint_values(names, positions, self.calibration)
        ratios, details = compute_finger_ratios(joint_values, self.calibration)

        self.latest_ratios = ratios
        self.latest_details = details
        self.last_valid_time = float(now)

        return ratios

    def is_fresh(self, now):
        return (
            self.last_valid_time is not None
            and float(now) - self.last_valid_time <= self.stale_timeout
        )

    def step(self, now):
        """Advance one publish tick and return the output ratios.

        Without fresh glove data the target becomes open, and the output
        ramps there through the same step limit, like the camera publisher
        does when it loses the hand.
        """
        if self.is_fresh(now):
            self.state = STATE_ACTIVE
            target = apply_max_ratio(self.latest_ratios, self.max_ratio)
        else:
            self.state = STATE_WAITING if self.last_valid_time is None else STATE_STALE
            target = make_zero_ratios()

        self.output = step_limit(self.output, target, self.max_step)
        return dict(self.output)


def summarize_samples(samples, calibration):
    """Median of each required joint over a list of (names, positions).

    Every sample is validated like a live message; the median keeps a single
    glitchy frame from becoming a calibration constant.
    """
    if not samples:
        raise InvalidCalibration("no samples recorded")

    per_joint = {finger: {key: [] for key in JOINT_KEYS} for finger in FINGER_NAMES}

    for names, positions in samples:
        values = extract_joint_values(names, positions, calibration)
        for finger in FINGER_NAMES:
            for key in JOINT_KEYS:
                per_joint[finger][key].append(values[finger][key])

    summary = {}
    for finger in FINGER_NAMES:
        summary[finger] = {}
        for key in JOINT_KEYS:
            data = per_joint[finger][key]
            summary[finger][key] = {
                "median": statistics.median(data),
                "stdev": statistics.pstdev(data),
                "count": len(data),
            }

    return summary


def build_calibration(template, open_summary, closed_summary, min_span, metadata=None):
    """Create a calibration dict from recorded open/closed summaries.

    Weights, joint prefix and validation limits are copied from template.
    Joints whose open/closed medians are closer than min_span are rejected,
    because their normalized value would be mostly sensor noise.
    """
    too_small = []
    fingers = {}

    for finger in FINGER_NAMES:
        joints = {}
        for key in JOINT_KEYS:
            open_rad = open_summary[finger][key]["median"]
            closed_rad = closed_summary[finger][key]["median"]

            if abs(closed_rad - open_rad) < min_span:
                too_small.append(f"{finger}.{key} ({closed_rad - open_rad:+.3f} rad)")

            joints[key] = {"open": round(open_rad, 4), "closed": round(closed_rad, 4)}

        fingers[finger] = {
            "joints": joints,
            "weights": dict(template["fingers"][finger].get("weights", {})),
        }

    if too_small:
        raise InvalidCalibration(
            f"open/closed span below {min_span} rad: {', '.join(too_small)}"
        )

    calibration = {
        "source": "recorded",
        "joint_prefix": template["joint_prefix"],
        "max_abs_joint_rad": template["max_abs_joint_rad"],
        "fingers": fingers,
    }
    if metadata:
        calibration["metadata"] = metadata

    return validate_calibration(calibration)


def format_details(details):
    lines = []
    for finger in FINGER_NAMES:
        d = details[finger]
        lines.append(
            f"{finger:<6} raw MCP/PIP/DIP = "
            f"{d['raw']['mcp']:+.3f} / {d['raw']['pip']:+.3f} / {d['raw']['dip']:+.3f} rad | "
            f"normalized = {d['normalized']['mcp']:.2f} / "
            f"{d['normalized']['pip']:.2f} / {d['normalized']['dip']:.2f} | "
            f"ratio = {d['ratio']:.2f}"
        )
    return lines
