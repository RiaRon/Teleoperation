#!/usr/bin/env bash
# DISARM the RIGHT LEAP hand (torque OFF).
# 1) ask the right controller over /right_hand/arm;
# 2) if no right controller is listening, send torque OFF directly on the serial port.
cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash

if ros2 topic pub --once --max-wait-time-secs 3 /right_hand/arm std_msgs/msg/Bool "{data: false}"; then
    echo "[OK] DISARM sent to the right controller (it turns torque OFF)."
    exit 0
fi

echo "[WARN] no right controller on /right_hand/arm; trying direct torque OFF."
python3 right_hand_torque_off.py
