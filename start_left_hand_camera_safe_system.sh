#!/bin/bash

cd ~/left_hand_project

gnome-terminal --title="T1 SAFE REAL HAND NODE" -- bash -c '
cd ~/left_hand_project
source /opt/ros/humble/setup.bash
source .venv/bin/activate
sudo chmod a+rw /dev/ttyUSB0
python3 left_hand_finger_ratio_node_safe.py --port /dev/ttyUSB0 --profile-velocity 8
exec bash
'

sleep 1

gnome-terminal --title="T2 FINGER JOINT STATE PUB" -- bash -c '
cd ~/left_hand_project
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 left_hand_finger_joint_state_pub.py
exec bash
'

sleep 1

gnome-terminal --title="T3 ROBOT STATE PUBLISHER" -- bash -c '
cd ~/left_hand_project
source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher left_hand_simple.urdf
exec bash
'

sleep 1

gnome-terminal --title="T4 RVIZ" -- bash -c '
cd ~/left_hand_project
source /opt/ros/humble/setup.bash
rviz2
exec bash
'

sleep 1

gnome-terminal --title="T5 STABLE CAMERA PUBLISHER" -- bash -c '
cd ~/left_hand_project
source /opt/ros/humble/setup.bash
source .venv/bin/activate
python3 left_hand_camera_finger_pub_stable.py --camera 0 --mirror --max-ratio 0.7 --alpha 0.12 --max-step 0.025 --deadzone 0.10 --lost-timeout 0.7
exec bash
'

sleep 1

gnome-terminal --title="T6 COMMAND TERMINAL" -- bash -c '
cd ~/left_hand_project
source /opt/ros/humble/setup.bash
source .venv/bin/activate
echo "Command terminal ready."
echo "Arm:"
echo "ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool \"{data: true}\""
echo ""
echo "Disarm:"
echo "ros2 topic pub --once /left_hand/arm std_msgs/msg/Bool \"{data: false}\""
exec bash
'
