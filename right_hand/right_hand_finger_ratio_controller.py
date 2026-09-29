import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32MultiArray

from right_hand_controller_core import (
    ALL_IDS,
    ArmRefused,
    DryRunBus,
    InvalidRatios,
    RightHandControllerCore,
    format_targets,
    hardware_blockers,
    load_hand_config,
)


FINGER_TOPIC = "/right_hand/finger_ratios"
ARM_TOPIC = "/right_hand/arm"


def open_hardware_bus(config, logger):
    """Open the port and run read-only checks. Only called in hardware mode."""
    from right_hand_dynamixel_bus import DynamixelBus

    bus = DynamixelBus(config["port"], config["baudrate"], ALL_IDS, config["protocol_version"])
    bus.open()

    try:
        models = bus.ping_all()
        logger.info(f"ping OK for IDs {sorted(models)} (models {sorted(set(models.values()))})")

        errors = {i: e for i, e in bus.read_hardware_errors().items() if e}
        if errors:
            raise RuntimeError(f"hardware error status set: {errors}")

        modes = bus.read_operating_modes()
        bad = {i: m for i, m in modes.items() if m not in config["allowed_operating_modes"]}
        if bad:
            raise RuntimeError(
                f"operating mode not in {config['allowed_operating_modes']}: {bad}"
            )

        torque = bus.read_torque()
        if any(torque.values()):
            # The only write before ARM, and it is the safe direction.
            logger.warn(f"torque was ON at startup for IDs {[i for i, t in torque.items() if t]}; turning OFF")
            bus.torque_off()

    except Exception:
        bus.close()
        raise

    return bus


class RightHandFingerRatioController(Node):
    def __init__(self, args, config, bus):
        super().__init__("right_hand_finger_ratio_controller")

        self.args = args
        self.config = config
        self.hardware = args.mode == "hardware"
        self.core = RightHandControllerCore(
            config,
            bus,
            watchdog_timeout=args.watchdog_timeout,
            max_step_deg=args.max_step_deg,
        )

        self.create_subscription(Float32MultiArray, args.ratio_topic, self.finger_callback, 10)
        self.create_subscription(Bool, args.arm_topic, self.arm_callback, 10)
        self.create_timer(0.05, self.watchdog_tick)

        self.rejected_count = 0
        self.last_reject_log = 0.0
        self.last_log = 0.0

        tag = "HARDWARE" if self.hardware else "DRY-RUN (no port, no motor writes)"
        self.get_logger().info(f"Right LEAP Hand controller started: {tag}")
        self.get_logger().info(f"config: {args.config}")
        self.get_logger().info(f"subscribed: {args.ratio_topic} [thumb, index, middle, ring]")
        self.get_logger().info(f"subscribed: {args.arm_topic} (std_msgs/Bool)")
        self.get_logger().info(
            f"watchdog_timeout={args.watchdog_timeout} s (then torque OFF), "
            f"max_step_deg={args.max_step_deg} per message, profile_velocity={config['profile_velocity']}"
        )
        self.get_logger().info("Initial state: DISARMED")

        for blocker in self.core.blockers:
            self.get_logger().warn(f"motor targets blocked: {blocker}")

    def finger_callback(self, msg):
        now = time.monotonic()
        try:
            command = self.core.on_ratios(msg.data, now)
        except InvalidRatios as e:
            self.rejected_count += 1
            if now - self.last_reject_log >= self.args.log_period:
                self.last_reject_log = now
                self.get_logger().warn(f"rejected ratio message ({self.rejected_count} total): {e}")
            return
        except Exception as e:
            self.get_logger().error(f"motor write failed, disarming: {e}")
            self.safe_disarm("write failure")
            return

        if now - self.last_log >= self.args.log_period:
            self.last_log = now
            self.log_state(command)

    def log_state(self, command):
        state = "ARMED" if self.core.armed else "DISARMED"
        tag = "Right LEAP HARDWARE" if self.hardware else "Right LEAP dry-run"
        self.get_logger().info(f"[{tag}] {state}")

        if self.core.target_deg is None:
            return

        for line in format_targets(self.config, self.core.latest_ratios, self.core.target_deg):
            self.get_logger().info(f"  {line}")

        if command is not None:
            sent = ", ".join(f"{i}:{command[i]:.1f}" for i in ALL_IDS)
            self.get_logger().info(f"  commanded after ramp: {sent}")

    def arm_callback(self, msg):
        now = time.monotonic()
        if not msg.data:
            self.safe_disarm("DISARM requested")
            return

        try:
            self.core.arm(now)
            self.get_logger().warn("ARMED: torque ON, holding present pose; targets ramp from here")
        except ArmRefused as e:
            self.get_logger().error(f"ARM refused: {e}")
        except Exception as e:
            self.get_logger().error(f"ARM failed: {e}")
            self.safe_disarm("ARM failure")

    def watchdog_tick(self):
        try:
            if self.core.tick(time.monotonic()):
                self.get_logger().error(f"DISARMED by {self.core.last_disarm_reason}")
        except Exception as e:
            self.get_logger().error(f"watchdog torque off failed: {e}")

    def safe_disarm(self, reason):
        try:
            self.core.disarm(reason)
            self.get_logger().warn(f"DISARMED ({reason}): torque OFF")
        except Exception as e:
            self.get_logger().error(f"DISARM torque off failed: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/right_hand.json")
    parser.add_argument(
        "--mode",
        choices=["dry-run", "hardware"],
        default="dry-run",
        help="dry-run never opens the serial port",
    )
    parser.add_argument("--ratio-topic", default=FINGER_TOPIC)
    parser.add_argument("--arm-topic", default=ARM_TOPIC)
    parser.add_argument(
        "--watchdog-timeout",
        type=float,
        default=0.5,
        help="seconds without a valid ratio while ARMED before torque OFF",
    )
    parser.add_argument(
        "--max-step-deg",
        type=float,
        default=3.0,
        help="largest goal change per motor per ratio message",
    )
    parser.add_argument("--log-period", type=float, default=1.0)

    args, ros_args = parser.parse_known_args()

    config = load_hand_config(args.config)

    if args.mode == "hardware":
        blockers = hardware_blockers(config)
        if blockers:
            print("[ERROR] hardware mode refused, port not opened:")
            for blocker in blockers:
                print(f"  - {blocker}")
            return 2

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    logger = rclpy.logging.get_logger("right_hand_finger_ratio_controller")

    if args.mode == "hardware":
        try:
            bus = open_hardware_bus(config, logger)
        except Exception as e:
            logger.error(f"hardware startup failed: {e}")
            rclpy.shutdown()
            return 1
    else:
        # Simulated present pose = open pose; unused while targets are blocked.
        bus = DryRunBus({k: v for k, v in config["open_pose_deg"].items() if v is not None})

    node = None
    try:
        node = RightHandFingerRatioController(args, config, bus)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.core.shutdown()
            else:
                bus.torque_off()
                bus.close()
            print("[INFO] shutdown: torque OFF, port closed" if args.mode == "hardware"
                  else "[INFO] shutdown (dry-run)")
        except Exception as e:
            print(f"[ERROR] shutdown torque off failed: {e}")
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
