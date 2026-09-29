import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from senseglove_finger_ratio import (
    STATE_ACTIVE,
    InvalidJointState,
    SenseGloveRatioPipeline,
    format_details,
    load_calibration,
    make_zero_ratios,
    ratios_to_list,
)


INPUT_TOPIC = "/senseglove/glove00782/rh/joint_states"
OUTPUT_TOPIC = "/left_hand/finger_ratios"


def format_ratios(ratios):
    return ", ".join(f"{value:.2f}" for value in ratios_to_list(ratios))


class SenseGloveFingerRatioPublisher(Node):
    def __init__(self, args):
        super().__init__("senseglove_finger_ratio_publisher")

        self.args = args
        self.calibration = load_calibration(args.calibration)

        self.pipeline = SenseGloveRatioPipeline(
            self.calibration,
            stale_timeout=args.stale_timeout,
            max_ratio=args.max_ratio,
            max_step=args.max_step,
        )

        self.publish_enabled = args.publish
        self.publisher = None

        # Dry-run never creates the publisher, so it cannot command the hand.
        if self.publish_enabled:
            self.publisher = self.create_publisher(Float32MultiArray, args.output_topic, 10)

        # Best-effort QoS subscribes to both reliable and best-effort publishers.
        self.subscription = self.create_subscription(
            JointState,
            args.input_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(1.0 / args.publish_rate, self.tick)

        self.received_count = 0
        self.rejected_count = 0
        self.last_reject_reason = None
        self.last_log_time = 0.0
        self.last_reject_log_time = 0.0
        self.last_state = None

        mode = f"PUBLISH -> {args.output_topic}" if self.publish_enabled else "DRY-RUN (no publisher)"
        self.get_logger().info("SenseGlove Finger Ratio Publisher started")
        self.get_logger().info(f"mode: {mode}")
        self.get_logger().info(f"input: {args.input_topic}")
        self.get_logger().info(
            f"calibration: {args.calibration} (source={self.calibration.get('source', 'unknown')})"
        )
        self.get_logger().info("data order: [thumb, index, middle, ring]")
        self.get_logger().info(
            f"publish_rate={args.publish_rate} Hz, stale_timeout={args.stale_timeout} s, "
            f"max_ratio={args.max_ratio}, max_step={args.max_step}"
        )

        if self.calibration.get("source") != "recorded":
            self.get_logger().warn(
                "calibration is not a recorded user calibration; ratios are only approximate"
            )

    def joint_state_callback(self, msg):
        now = time.monotonic()
        self.received_count += 1

        try:
            self.pipeline.on_joint_state(msg.name, msg.position, now)
        except InvalidJointState as e:
            self.rejected_count += 1
            self.last_reject_reason = str(e)

            if now - self.last_reject_log_time >= self.args.log_period:
                self.last_reject_log_time = now
                self.get_logger().warn(
                    f"rejected glove message ({self.rejected_count} total): {e}"
                )

    def tick(self):
        now = time.monotonic()
        output = self.pipeline.step(now)
        state = self.pipeline.state

        if state != self.last_state:
            # rclpy forbids mixing severities on one call site, so keep two calls.
            text = f"glove input state: {self.last_state} -> {state}"
            if state == STATE_ACTIVE:
                self.get_logger().info(text)
            else:
                self.get_logger().warn(text)
            self.last_state = state

        if self.publisher is not None:
            msg = Float32MultiArray()
            msg.data = ratios_to_list(output)
            self.publisher.publish(msg)

        if now - self.last_log_time >= self.args.log_period:
            self.last_log_time = now
            self.log_status(output)

    def log_status(self, output):
        tag = "PUBLISH" if self.publisher is not None else "DRY-RUN"
        latest = self.pipeline.latest_ratios

        self.get_logger().info(
            f"[{tag}] state={self.pipeline.state} "
            f"glove=[{format_ratios(latest) if latest else '-'}] "
            f"output=[{format_ratios(output)}] "
            f"msgs={self.received_count} rejected={self.rejected_count}"
        )

        if self.args.debug_joints and self.pipeline.latest_details is not None:
            for line in format_details(self.pipeline.latest_details):
                self.get_logger().info(f"  {line}")

    def publish_open_once(self):
        # Same as the camera publisher on exit: leave the hand commanded open.
        if self.publisher is None:
            return

        msg = Float32MultiArray()
        msg.data = ratios_to_list(make_zero_ratios())
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--input-topic", default=INPUT_TOPIC)
    parser.add_argument("--output-topic", default=OUTPUT_TOPIC)
    parser.add_argument("--calibration", default="senseglove_calibration.json")

    parser.add_argument(
        "--publish",
        action="store_true",
        help="actually publish ratios. Without this flag the node only logs (dry-run).",
    )

    parser.add_argument(
        "--publish-rate",
        type=float,
        default=15.0,
        help="output rate. The controller does one Dynamixel sync write per message.",
    )

    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.3,
        help="seconds without a valid glove message before ramping to open",
    )

    parser.add_argument(
        "--max-ratio",
        type=float,
        default=0.7,
        help="maximum output ratio (same default as the camera publisher)",
    )

    parser.add_argument(
        "--max-step",
        type=float,
        default=0.05,
        help="maximum ratio change per publish (same default as the camera publisher)",
    )

    parser.add_argument("--log-period", type=float, default=1.0)
    parser.add_argument(
        "--debug-joints",
        action="store_true",
        help="also log raw and normalized MCP/PIP/DIP per finger",
    )

    args, ros_args = parser.parse_known_args()

    if args.publish_rate <= 0.0:
        parser.error("--publish-rate must be > 0")

    # Keep the ROS context alive on Ctrl+C so the final open command can go out.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)

    node = None

    try:
        node = SenseGloveFingerRatioPublisher(args)
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.publish_open_once()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
