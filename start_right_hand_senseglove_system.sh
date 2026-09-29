#!/usr/bin/env bash
# Right Nova 2 (#00782) -> RIGHT LEAP Hand V1, ROS 2 Jazzy.
#
#   ./start_right_hand_senseglove_system.sh           dryrun : glove -> ratio logs only, nothing published
#   ./start_right_hand_senseglove_system.sh preview   preview: ratios on /right_hand/finger_ratios + right controller
#                                                              in dry-run (motor targets logged, port never opened)
#   ./start_right_hand_senseglove_system.sh hand      hand   : ratios + right controller in hardware mode (starts DISARMED)
#
# CONFIG=... overrides config/right_hand.json (e.g. the synthetic test config for a preview demo;
# hardware mode refuses anything that is not a verified config_kind "hardware").
# SenseCom and the senseglove bringup must already be running. The left system is not touched.

MODE="${1:-dryrun}"

PROJECT_DIR="$HOME/left_hand_project"
CONFIG="${CONFIG:-config/right_hand.json}"
MAX_RATIO="0.5"
MAX_STEP="0.05"
GLOVE_TOPIC="/senseglove/glove00782/rh/joint_states"
RATIO_TOPIC="/right_hand/finger_ratios"
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

if [ "$MODE" = "hand" ]; then
    # Refuse before opening any terminal if the right config is not hardware-ready.
    PORT=$(python3 - "$CONFIG" <<'EOF'
import sys
from right_hand_controller_core import hardware_blockers, load_hand_config
config = load_hand_config(sys.argv[1])
blockers = hardware_blockers(config)
if blockers:
    print("[ERROR] right hand config is not hardware-ready:", file=sys.stderr)
    for b in blockers:
        print(f"  - {b}", file=sys.stderr)
    sys.exit(1)
print(config["port"])
EOF
    ) || exit 1

    if [ ! -e "$PORT" ]; then
        echo "[ERROR] $PORT not found. Check the right hand U2D2 USB connection."
        exit 1
    fi
    sudo chmod a+rw "$PORT"
fi

if ! timeout 5 ros2 topic info "$GLOVE_TOPIC" > /dev/null 2>&1; then
    echo "[WARN] $GLOVE_TOPIC not visible yet. Is SenseCom + senseglove bringup running?"
fi

echo "Starting RIGHT LEAP Hand SenseGlove system"
echo "Mode: $MODE"
echo "Config: $CONFIG"
echo "Calibration: $CALIBRATION"

if [ "$MODE" = "dryrun" ]; then
    gnome-terminal --title="R1 SENSEGLOVE RATIO (dry-run)" -- bash -c "
    cd $PROJECT_DIR
    source $ROS_SETUP
    python3 senseglove_finger_ratio_pub.py --input-topic $GLOVE_TOPIC --calibration $CALIBRATION --max-ratio $MAX_RATIO --max-step $MAX_STEP --debug-joints
    exec bash
    "
    echo "Done. dryrun: nothing is published."
    exit 0
fi

if [ "$MODE" = "hand" ]; then
    CONTROLLER_MODE="hardware"
    CONTROLLER_TITLE="R1 RIGHT LEAP CONTROLLER (HARDWARE)"
else
    CONTROLLER_MODE="dry-run"
    CONTROLLER_TITLE="R1 RIGHT LEAP CONTROLLER (dry-run)"
fi

gnome-terminal --title="$CONTROLLER_TITLE" -- bash -c "
cd $PROJECT_DIR
source $ROS_SETUP
python3 right_hand_finger_ratio_controller.py --mode $CONTROLLER_MODE --config $CONFIG --ratio-topic $RATIO_TOPIC
exec bash
"

sleep 2

gnome-terminal --title="R2 SENSEGLOVE RATIO -> $RATIO_TOPIC" -- bash -c "
cd $PROJECT_DIR
source $ROS_SETUP
python3 senseglove_finger_ratio_pub.py --publish --output-topic $RATIO_TOPIC --input-topic $GLOVE_TOPIC --calibration $CALIBRATION --max-ratio $MAX_RATIO --max-step $MAX_STEP
exec bash
"

sleep 1

gnome-terminal --title="R3 RIGHT COMMAND TERMINAL" -- bash -c "
cd $PROJECT_DIR
source $ROS_SETUP
echo 'Right controller mode: $CONTROLLER_MODE (starts DISARMED).'
echo 'Open the gloved hand, check R2 output is near 0, then:'
echo ''
echo 'Arm:    ./LEAP-Right-Arm-Jazzy.sh'
echo 'Disarm: ./LEAP-Right-Disarm-Jazzy.sh'
echo ''
echo 'The right controller turns torque OFF if no valid ratio arrives for 0.5 s while ARMED.'
exec bash
"

echo "Done. Terminals opened for mode: $MODE"
