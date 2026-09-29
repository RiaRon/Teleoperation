import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from senseglove_finger_ratio import (
    FINGER_NAMES,
    JOINT_KEYS,
    InvalidCalibration,
    InvalidJointState,
    build_calibration,
    load_calibration,
    summarize_samples,
)


INPUT_TOPIC = "/senseglove/glove00782/rh/joint_states"


class JointStateRecorder(Node):
    """Only reads the glove topic; publishes nothing."""

    def __init__(self, input_topic):
        super().__init__("senseglove_calibration_recorder")
        self.recording = False
        self.samples = []
        self.create_subscription(
            JointState,
            input_topic,
            self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, msg):
        if self.recording:
            self.samples.append((list(msg.name), list(msg.position)))

    def record(self, duration):
        self.samples = []
        self.recording = True
        end = time.monotonic() + duration

        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.recording = False
        return list(self.samples)


def print_summary(label, summary):
    print(f"\n[{label}] median (stdev) rad")
    for finger in FINGER_NAMES:
        cells = [
            f"{key.upper()}={summary[finger][key]['median']:+.3f} ({summary[finger][key]['stdev']:.3f})"
            for key in JOINT_KEYS
        ]
        print(f"  {finger:<6} " + "  ".join(cells))


def record_pose(node, label, instruction, duration, min_samples, template):
    input(f"\n{instruction}\nHold the pose and press Enter to record {duration:.1f} s ...")
    samples = node.record(duration)

    if len(samples) < min_samples:
        raise RuntimeError(
            f"{label}: only {len(samples)} messages in {duration:.1f} s "
            f"(need {min_samples}). Is the glove topic running?"
        )

    summary = summarize_samples(samples, template)
    print_summary(label, summary)
    return summary, len(samples)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--input-topic", default=INPUT_TOPIC)
    parser.add_argument(
        "--template",
        default="senseglove_calibration_anatomical.json",
        help="joint prefix, weights and validation limits are copied from here",
    )
    parser.add_argument("--output", default="senseglove_calibration.json")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument(
        "--min-span",
        type=float,
        default=0.1,
        help="minimum |closed - open| per joint in rad",
    )
    parser.add_argument("--overwrite", action="store_true")

    args, ros_args = parser.parse_known_args()

    output = Path(args.output)
    if output.exists() and not args.overwrite:
        parser.error(f"{output} exists. Use --overwrite to replace it.")

    template = load_calibration(args.template)

    rclpy.init(args=ros_args)
    node = JointStateRecorder(args.input_topic)

    try:
        open_summary, open_count = record_pose(
            node,
            "OPEN",
            "Step 1/2: open the hand flat, fingers straight and together.",
            args.duration,
            args.min_samples,
            template,
        )
        closed_summary, closed_count = record_pose(
            node,
            "FIST",
            "Step 2/2: make a firm fist with the thumb wrapped over the fingers.",
            args.duration,
            args.min_samples,
            template,
        )

        calibration = build_calibration(
            template,
            open_summary,
            closed_summary,
            args.min_span,
            metadata={
                "recorded_at": datetime.now().isoformat(timespec="seconds"),
                "input_topic": args.input_topic,
                "duration_s": args.duration,
                "open_samples": open_count,
                "closed_samples": closed_count,
            },
        )

    except (InvalidJointState, InvalidCalibration, RuntimeError) as e:
        print(f"\n[ERROR] calibration not saved: {e}")
        return 1

    except KeyboardInterrupt:
        print("\n[INFO] cancelled, calibration not saved")
        return 1

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    with open(output, "w", encoding="utf-8") as f:
        json.dump(calibration, f, indent=2)
        f.write("\n")

    print(f"\n[INFO] saved {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
