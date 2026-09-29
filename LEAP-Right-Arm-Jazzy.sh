#!/usr/bin/env bash
# ARM the RIGHT LEAP controller (right_hand_finger_ratio_controller.py) via /right_hand/arm.
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once --max-wait-time-secs 5 /right_hand/arm std_msgs/msg/Bool "{data: true}"
