#!/bin/bash
#
# Step 2: Convert a single rosbag to GR00T LeRobot v2 format (test / one episode).
#
# Use this to verify topic names and conversion output before running the full
# batch. Check the parquet and videos before committing to a full batch run.
#
# Usage:
#   ./02_convert_single_bag.sh /path/to/rosbag_folder /path/to/output [repo_id]
#
# Example:
#   ./02_convert_single_bag.sh \
#     /mnt/sandisk/data/tidyverse-hand-2026-01-19/rosbag2_2026_01_18-21_14_59 \
#     /tmp/lerobot_test tidyverse_hand_test

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONVERT_SCRIPT="${SCRIPT_DIR}/convert_rosbag_to_groot_lerobot.py"

if [[ ! -f "$CONVERT_SCRIPT" ]]; then
    echo "ERROR: Converter not found: $CONVERT_SCRIPT"
    exit 1
fi

if [[ -z "$1" ]] || [[ -z "$2" ]]; then
    echo "Usage: $0 /path/to/rosbag_folder /path/to/output [repo_id]"
    echo ""
    echo "Converts one rosbag to GR00T LeRobot v2 for inspection."
    echo "Run ./01_inspect.sh first to confirm topic names."
    exit 1
fi

INPUT_BAG="$1"
OUTPUT_DIR="$2"
REPO_ID="${3:-local/hand_teleop_test}"

# Source ROS2 workspace for custom message types
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/tetheria/aero-open-ros2/install/setup.bash" ]]; then
    source "$HOME/tetheria/aero-open-ros2/install/setup.bash"
fi

echo "=============================================="
echo "Single-bag GR00T LeRobot v2 conversion"
echo "=============================================="
echo "  Input:   $INPUT_BAG"
echo "  Output:  $OUTPUT_DIR"
echo "  Repo ID: $REPO_ID"
echo ""

python3 "$CONVERT_SCRIPT" \
    --input-dir "$INPUT_BAG" \
    --output-root "$OUTPUT_DIR" \
    --repo-id "$REPO_ID" \
    --robot-type "aero_hand" \
    --fps 30 \
    --task "hand teleop demonstration" \
    --use-video \
    --image-h 480 \
    --image-w 640 \
    --topic-cmd-vel "/spacemouse/cmd_vel" \
    --topic-joint-states "/joint_states" \
    --topic-hand-control "/right/joint_control" \
    --topic-actuator-states "/right/actuator_states" \
    --topic-manus "/manus_glove_0" \
    --arm-joint-names "joint1,joint2,joint3,joint4,joint5,joint6,joint7" \
    --arm-joint-dim 7 \
    --hand-joint-dim 16 \
    --num-actuators 7 \
    --manus-dim 20 \
    --camera-head "/camera_0/color" \
    --camera-wrist "/camera_4/color" \
    --camera-base "/logitech_base/color" \
    --overwrite \
    --debug-topics

echo ""
echo "Done! Output: $OUTPUT_DIR"
echo ""
echo "Next: inspect the output, then run ./03_batch_convert.sh for all bags."
echo "Or validate directly: python3 $SCRIPT_DIR/validate_lerobot_dataset.py --dataset $OUTPUT_DIR"
