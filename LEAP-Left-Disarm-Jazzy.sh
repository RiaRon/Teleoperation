#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once --max-wait-time-secs 5 /left_hand/arm std_msgs/msg/Bool "{data: false}"
