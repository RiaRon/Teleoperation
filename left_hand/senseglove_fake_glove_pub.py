"""Replay recorded Nova 2 poses as a fake glove, for dry-runs without the glove.

Publishes sensor_msgs/JointState with the same 23 joint names as
/senseglove/glove00795/lh/joint_states (l_ prefix), in a shuffled order, so
the name-based lookup is exercised too. The poses were recorded on the RIGHT
glove #00782; the SenseGlove SDK gives identical flexion signs and limits for
both hands, so they stand in for left-glove data until it is recorded. Optional faults: stop publishing
(glove disconnect) and inject NaN.
"""

import argparse
import json
import random
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


OUTPUT_TOPIC = "/senseglove/glove00795/lh/joint_states"
JOINT_PREFIX = "l_"
FIXTURE = Path(__file__).parent / "tests" / "fixtures" / "senseglove_poses.json"

FINGERS = ["thumb", "index", "middle", "ring"]
KEYS = ["mcp", "pip", "dip"]


def load_poses(path=FIXTURE):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)["poses"]


def pose_to_joint_map(pose, prefix=JOINT_PREFIX):
    joints = {}

    for finger in FINGERS:
        for key, value in zip(KEYS, pose[finger]):
            joints[f"{prefix}{finger}_{key}"] = float(value)

    # Pinky mirrors ring on the real glove; brake/palm entries are unused.
    for key, value in zip(KEYS, pose["ring"]):
        joints[f"{prefix}pinky_{key}"] = float(value)

    for finger in FINGERS + ["pinky"]:
        joints[f"{prefix}{finger}_brake"] = 0.0

    for extra in ["palm_index", "palm_pinky", "palm_strap"]:
        joints[f"{prefix}{extra}"] = 0.0

    return joints


def blend(pose_a, pose_b, t):
    return {
        finger: [a + (b - a) * t for a, b in zip(pose_a[finger], pose_b[finger])]
        for finger in FINGERS
    }


class FakeGlovePublisher(Node):
    def __init__(self, args):
        super().__init__("senseglove_fake_glove_publisher")
        self.args = args
        self.poses = load_poses()
        self.sequence = args.sequence.split(",")
        for name in self.sequence:
            if name not in self.poses:
                raise ValueError(f"unknown pose {name}; choose from {sorted(self.poses)}")

        self.publisher = self.create_publisher(JointState, args.topic, 10)
        self.start = time.monotonic()
        self.rng = random.Random(0)
        self.stopped_logged = False
        self.create_timer(1.0 / args.rate, self.tick)

        self.get_logger().info(f"fake glove -> {args.topic} at {args.rate} Hz")
        self.get_logger().info(f"sequence: {self.sequence}, {args.hold} s per pose")
        if args.stop_after > 0:
            self.get_logger().info(f"will stop publishing after {args.stop_after} s")
        if args.nan_after > 0:
            self.get_logger().info(f"will send NaN in r_index_pip after {args.nan_after} s")

    def current_pose(self, elapsed):
        # Hold each pose, then blend to the next over the last 30 % of the hold.
        slot = elapsed / self.args.hold
        i = int(slot) % len(self.sequence)
        frac = slot - int(slot)
        a = self.poses[self.sequence[i]]
        b = self.poses[self.sequence[(i + 1) % len(self.sequence)]]
        t = max(0.0, (frac - 0.7) / 0.3)
        return blend(a, b, t)

    def tick(self):
        elapsed = time.monotonic() - self.start

        if self.args.stop_after > 0 and elapsed >= self.args.stop_after:
            if not self.stopped_logged:
                self.get_logger().warn("simulated glove disconnect: publishing stopped")
                self.stopped_logged = True
            return

        joints = pose_to_joint_map(self.current_pose(elapsed))

        if self.args.nan_after > 0 and elapsed >= self.args.nan_after:
            joints["r_index_pip"] = float("nan")

        names = list(joints)
        self.rng.shuffle(names)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [joints[n] for n in names]
        msg.velocity = [0.0] * len(names)
        msg.effort = [0.0] * len(names)
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default=OUTPUT_TOPIC)
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--sequence", default="open,fist,index_bend,pinch")
    parser.add_argument("--hold", type=float, default=3.0, help="seconds per pose")
    parser.add_argument("--stop-after", type=float, default=0.0, help="0 = never stop")
    parser.add_argument("--nan-after", type=float, default=0.0, help="0 = never")

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = FakeGlovePublisher(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
