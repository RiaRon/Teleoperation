#!/usr/bin/env bash
# Left Nova 2 (#00795) -> LEFT LEAP Hand, ROS 2 Jazzy (SenseGlove version of start_left_hand_camera_safe_system.sh).
#
#   ./start_left_hand_senseglove_system.sh           dryrun : ratio node logs only, no publish, no motors
#   ./start_left_hand_senseglove_system.sh preview   preview: publish ratios + RViz model, no motors
#   ./start_left_hand_senseglove_system.sh hand      hand   : preview + LEAP controller (starts DISARMED)
#
# SenseCom and the senseglove bringup must already be running.

MODE="${1:-dryrun}"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${PORT:-/dev/ttyUSB0}"
PROFILE_VELOCITY="8"
MAX_RATIO="0.5"
MAX_STEP="0.05"
GLOVE_TOPIC="/senseglove/glove00795/lh/joint_states"
CALIBRATION="senseglove_calibration.json"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

cd "$PROJECT_DIR" || exit 1
source "$ROS_SETUP"

case "$MODE" in
    dryrun|preview|hand) ;;
    *)
        echo "[ERROR] unknown mode: $MODE (use dryrun, preview or hand)"
        exit 1
        ;;
esac

if [ ! -f "$CALIBRATION" ]; then
    if [ "$MODE" != "dryrun" ]; then
        echo "[ERROR] $CALIBRATION not found. Record it first:"
        echo "  python3 senseglove_record_calibration.py"
        exit 1
    fi
    echo "[WARN] $CALIBRATION not found. dryrun uses senseglove_calibration_anatomical.json"
    CALIBRATION="senseglove_calibration_anatomical.json"
fi

if ! timeout 5 ros2 topic info "$GLOVE_TOPIC" > /dev/null 2>&1; then
    echo "[WARN] $GLOVE_TOPIC not visible yet. Is SenseCom + senseglove bringup running?"
fi

if [ "$MODE" = "hand" ]; then
    if [ ! -e "$PORT" ]; then
        echo "[ERROR] $PORT not found."
        echo "Check U2D2 USB connection."
        exit 1
    fi

    sudo chmod a+rw "$PORT"
fi

echo "Starting LEAP Left Hand SenseGlove system"
echo "Mode: $MODE"
echo "Calibration: $CALIBRATION"
echo "Max ratio: $MAX_RATIO"

if [ "$MODE" = "hand" ]; then
    gnome-terminal --title="T1 SAFE REAL HAND NODE" -- bash -c "
    cd $PROJECT_DIR
    source $ROS_SETUP
    python3 left_hand_finger_ratio_node_safe.py --port $PORT --profile-velocity $PROFILE_VELOCITY
    exec bash
    "

    sleep 2
fi

if [ "$MODE" != "dryrun" ]; then
    gnome-terminal --title="T2 FINGER JOINT STATE PUB" -- bash -c "
    cd $PROJECT_DIR
    source $ROS_SETUP
    python3 left_hand_finger_joint_state_pub.py
    exec bash
    "

    sleep 1

    gnome-terminal --title="T3 ROBOT STATE PUBLISHER" -- bash -c "
    cd $PROJECT_DIR
    source $ROS_SETUP
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(cat left_hand_simple.urdf)\"
    exec bash
    "

    sleep 1

    gnome-terminal --title="T4 RVIZ" -- bash -c "
    cd $PROJECT_DIR
    source $ROS_SETUP
    rviz2
    exec bash
    "

    sleep 1

    PUBLISH_FLAG="--publish"
else
    PUBLISH_FLAG=""
fi

gnome-terminal --title="T5 SENSEGLOVE FINGER RATIO ($MODE)" -- bash -c "
cd $PROJECT_DIR
source $ROS_SETUP
python3 senseglove_finger_ratio_pub.py $PUBLISH_FLAG --input-topic $GLOVE_TOPIC --calibration $CALIBRATION --max-ratio $MAX_RATIO --max-step $MAX_STEP
exec bash
"

if [ "$MODE" = "hand" ]; then
    sleep 1

    gnome-terminal --title="T6 COMMAND TERMINAL" -- bash -c "
    cd $PROJECT_DIR
    source $ROS_SETUP
    echo 'Command terminal ready.'
    echo 'Controller starts DISARMED (torque OFF).'
    echo 'Open the gloved hand, check T5 output is near 0, then:'
    echo ''
    echo 'Arm:    ./LEAP-Left-Arm-Jazzy.sh'
    echo 'Disarm: ./LEAP-Left-Disarm-Jazzy.sh'
    echo ''
    echo 'Do not run the camera publisher or LEAP-Left-Fist/VSign at the same time.'
    exec bash
    "
fi

echo "Done. Terminals opened for mode: $MODE"
