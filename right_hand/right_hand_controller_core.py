"""Right LEAP Hand V1 controller logic, without ROS or dynamixel_sdk.

Input is the same /x_hand/finger_ratios contract as the left controller:
[thumb, index, middle, ring], 0.0 = open, 1.0 = closed. This module knows
nothing about the sensor that produced the ratios.

Motor I/O goes through a bus object (DryRunBus here, DynamixelBus in
right_hand_dynamixel_bus.py), so every path can be tested without hardware.
"""

import json
import math


FINGER_NAMES = ["thumb", "index", "middle", "ring"]
ALL_IDS = list(range(16))
TICKS_PER_REV = 4096
VERIFIED_KEYS = ["port", "baudrate", "motor_ids", "open_pose", "close_pose", "direction", "limits"]


class InvalidConfig(ValueError):
    pass


class InvalidRatios(ValueError):
    pass


class ArmRefused(RuntimeError):
    pass


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def load_hand_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return validate_hand_config(json.load(f))


def validate_hand_config(config):
    """Structural checks. TBD (null) values are allowed here and reported by
    target_blockers() / hardware_blockers() instead."""
    if config.get("hand") != "right":
        raise InvalidConfig(f"hand must be 'right', got {config.get('hand')!r}")

    finger_ids = config.get("finger_ids")
    if not isinstance(finger_ids, dict) or set(finger_ids) != set(FINGER_NAMES):
        raise InvalidConfig(f"finger_ids must have exactly {FINGER_NAMES}")

    ids = []
    for finger in FINGER_NAMES:
        finger_list = finger_ids[finger]
        if not isinstance(finger_list, list) or len(finger_list) != 4:
            raise InvalidConfig(f"finger_ids.{finger} must list 4 motor IDs")
        ids.extend(finger_list)

    if sorted(ids) != ALL_IDS:
        raise InvalidConfig(f"finger_ids must use each ID 0-15 exactly once, got {sorted(ids)}")

    for section in ("open_pose_deg", "close_pose_deg", "limits_deg"):
        table = config.get(section)
        if not isinstance(table, dict) or set(table) != {str(i) for i in ALL_IDS}:
            raise InvalidConfig(f"{section} must have keys '0'..'15'")

    for key in map(str, ALL_IDS):
        limit = config["limits_deg"][key]
        if limit is None:
            continue
        if (not isinstance(limit, list) or len(limit) != 2
                or not all(_finite_number(v) for v in limit)):
            raise InvalidConfig(f"limits_deg.{key} must be [min, max] or null")
        low, high = limit
        # Staying inside one turn keeps tick conversion free of wrap-around.
        if not 0.0 < low < high < 360.0:
            raise InvalidConfig(f"limits_deg.{key} must satisfy 0 < min < max < 360")

        for section in ("open_pose_deg", "close_pose_deg"):
            value = config[section][key]
            if value is None:
                continue
            if not _finite_number(value):
                raise InvalidConfig(f"{section}.{key} must be a finite number or null")
            if not low <= value <= high:
                raise InvalidConfig(
                    f"{section}.{key}={value} is outside limits_deg.{key}={limit}"
                )

    return config


def target_blockers(config):
    """What prevents computing motor targets at all (dry-run included)."""
    blockers = []
    for section in ("open_pose_deg", "close_pose_deg", "limits_deg"):
        missing = [k for k in map(str, ALL_IDS) if config[section][k] is None]
        if missing:
            blockers.append(f"{section} TBD for IDs {','.join(missing)}")
    return blockers


def hardware_blockers(config):
    """What prevents talking to real motors. Empty list = hardware allowed."""
    blockers = list(target_blockers(config))

    if config.get("config_kind") != "hardware":
        blockers.append(f"config_kind is {config.get('config_kind')!r}, not 'hardware'")
    if not config.get("port"):
        blockers.append("port TBD")
    if not isinstance(config.get("baudrate"), int) or config.get("baudrate", 0) <= 0:
        blockers.append("baudrate TBD")

    verified = config.get("hardware_verified", {})
    unverified = [k for k in VERIFIED_KEYS if verified.get(k) is not True]
    if unverified:
        blockers.append(f"not verified on the real right hand: {', '.join(unverified)}")

    return blockers


def validate_ratios(data):
    """Finite check first, then clamp: clamp(NaN) must never happen."""
    values = list(data)

    if len(values) != len(FINGER_NAMES):
        raise InvalidRatios(f"expected 4 values [thumb, index, middle, ring], got {len(values)}")

    ratios = {}
    for name, value in zip(FINGER_NAMES, values):
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise InvalidRatios(f"{name} ratio is not a number: {value!r}")
        if not math.isfinite(value):
            raise InvalidRatios(f"{name} ratio is not finite: {value}")
        ratios[name] = max(0.0, min(1.0, value))

    return ratios


def clamp_to_limits(config, key, deg):
    low, high = config["limits_deg"][key]
    return max(low, min(high, deg))


def compute_targets_deg(config, ratios):
    """target = open + ratio * (close - open), then clamped to the joint limit."""
    targets = {}

    for finger in FINGER_NAMES:
        ratio = ratios[finger]
        for dxl_id in config["finger_ids"][finger]:
            key = str(dxl_id)
            open_deg = float(config["open_pose_deg"][key])
            close_deg = float(config["close_pose_deg"][key])
            targets[dxl_id] = clamp_to_limits(config, key, open_deg + ratio * (close_deg - open_deg))

    return targets


def deg_to_tick(deg):
    tick = int(round(deg / 360.0 * TICKS_PER_REV))
    if not 0 <= tick < TICKS_PER_REV:
        raise ValueError(f"{deg} deg is outside one turn")
    return tick


def tick_to_deg(tick):
    return tick * 360.0 / TICKS_PER_REV


def slew(current, target, max_step_deg):
    return {
        dxl_id: current[dxl_id] + max(-max_step_deg, min(max_step_deg, target[dxl_id] - current[dxl_id]))
        for dxl_id in target
    }


class DryRunBus:
    """Stands in for the Dynamixel bus: no port, no writes, records calls.

    Present positions start at the given pose (open pose by default) and
    follow goal writes instantly.
    """

    def __init__(self, initial_deg):
        self.present_deg = {int(k): float(v) for k, v in initial_deg.items()}
        self.torque = {dxl_id: False for dxl_id in self.present_deg}
        self.calls = []
        self.opened = True

    def read_present_deg(self):
        self.calls.append(("read_present",))
        return dict(self.present_deg)

    def set_profile_velocity(self, value):
        self.calls.append(("profile_velocity", value))

    def torque_on(self):
        self.calls.append(("torque_on",))
        self.torque = {k: True for k in self.torque}

    def torque_off(self):
        self.calls.append(("torque_off",))
        self.torque = {k: False for k in self.torque}

    def write_goal_ticks(self, goal_ticks):
        self.calls.append(("goal", dict(goal_ticks)))
        for dxl_id, tick in goal_ticks.items():
            self.present_deg[dxl_id] = tick_to_deg(tick)

    def close(self):
        self.calls.append(("close",))
        self.opened = False


class RightHandControllerCore:
    """ARM gate, ratio -> target, startup ramp, watchdog, shutdown.

    Nothing is written to the bus before arm(); after disarm() or the
    watchdog, torque is off until the next explicit arm().
    """

    def __init__(self, config, bus, watchdog_timeout, max_step_deg):
        if watchdog_timeout <= 0.0:
            raise ValueError("watchdog_timeout must be > 0")
        if max_step_deg <= 0.0:
            raise ValueError("max_step_deg must be > 0")

        self.config = validate_hand_config(config)
        self.bus = bus
        self.watchdog_timeout = float(watchdog_timeout)
        self.max_step_deg = float(max_step_deg)

        self.blockers = target_blockers(self.config)
        self.armed = False
        self.arm_time = None
        self.last_valid_time = None
        self.latest_ratios = None
        self.target_deg = None
        self.command_deg = None
        self.last_disarm_reason = None

    def on_ratios(self, data, now):
        """Validate and, only when armed, send one rate-limited goal.

        Raises InvalidRatios; a rejected message does not feed the watchdog.
        Returns the commanded degrees, or None when nothing was sent.
        """
        ratios = validate_ratios(data)
        self.latest_ratios = ratios
        self.last_valid_time = float(now)

        if self.blockers:
            return None

        self.target_deg = compute_targets_deg(self.config, ratios)

        if not self.armed:
            return None

        command = slew(self.command_deg, self.target_deg, self.max_step_deg)
        command = {i: clamp_to_limits(self.config, str(i), d) for i, d in command.items()}
        self.bus.write_goal_ticks({i: deg_to_tick(d) for i, d in command.items()})
        self.command_deg = command
        return command

    def arm(self, now):
        if self.armed:
            return
        if self.blockers:
            raise ArmRefused("; ".join(self.blockers))

        present = self.bus.read_present_deg()
        tolerance = float(self.config["arm_position_tolerance_deg"])
        outside = []
        for dxl_id in ALL_IDS:
            low, high = self.config["limits_deg"][str(dxl_id)]
            deg = present[dxl_id]
            if not low - tolerance <= deg <= high + tolerance:
                outside.append(f"ID {dxl_id}={deg:.1f} not in [{low}, {high}]")
        if outside:
            # Usually a wrong ID map, wrong horn mounting or wrong config.
            raise ArmRefused("present position outside limits: " + "; ".join(outside))

        try:
            self.bus.set_profile_velocity(int(self.config["profile_velocity"]))
            self.bus.torque_on()
            hold = {i: clamp_to_limits(self.config, str(i), present[i]) for i in ALL_IDS}
            self.bus.write_goal_ticks({i: deg_to_tick(d) for i, d in hold.items()})
        except Exception:
            self.bus.torque_off()
            raise

        # The ramp starts from where the hand really is, not from the target.
        self.command_deg = hold
        self.armed = True
        self.arm_time = float(now)
        self.last_disarm_reason = None

    def disarm(self, reason):
        # Torque off even if already disarmed: it is the safe direction.
        self.bus.torque_off()
        self.armed = False
        self.last_disarm_reason = reason

    def watchdog_expired(self, now):
        if not self.armed:
            return False
        last = self.arm_time
        if self.last_valid_time is not None and self.last_valid_time > last:
            last = self.last_valid_time
        return float(now) - last > self.watchdog_timeout

    def tick(self, now):
        """Call periodically. Returns True when the watchdog disarmed the hand."""
        if self.watchdog_expired(now):
            self.disarm(f"watchdog: no valid ratio for > {self.watchdog_timeout} s")
            return True
        return False

    def shutdown(self):
        try:
            if getattr(self.bus, "opened", True):
                self.bus.torque_off()
        finally:
            self.armed = False
            self.bus.close()


def format_targets(config, ratios, targets):
    lines = [
        "ratios: " + ", ".join(f"{n}={ratios[n]:.2f}" for n in FINGER_NAMES),
        "motor targets:",
    ]
    for finger in FINGER_NAMES:
        for dxl_id in config["finger_ids"][finger]:
            deg = targets[dxl_id]
            lines.append(
                f"  ID {dxl_id:>2} {config['joint_names'][str(dxl_id)]:<18} -> "
                f"{deg:7.2f} deg (tick {deg_to_tick(deg)})"
            )
    return lines
