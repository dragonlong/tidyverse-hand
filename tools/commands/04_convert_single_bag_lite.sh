#!/bin/bash
# LIGHTWEIGHT: Convert a single rosbag to LeRobot format WITHOUT video
# Use this on low-memory systems (< 16GB RAM)
#
# Usage: ./04_convert_single_bag_lite.sh /path/to/rosbag_folder /path/to/output [repo_id]
#
# This skips video encoding which is the main memory hog.
# You can add video later on a more powerful machine.

set -e

# Activate tidybot2 conda environment
source ~/miniforge3/etc/profile.d/conda.sh 2>/dev/null || source ~/mambaforge/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
mamba activate tidybot2 || conda activate tidybot2

# Source ROS2 workspace for custom messages
cd ~/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: $0 /path/to/rosbag_folder /path/to/output [repo_id]"
    echo ""
    echo "LIGHTWEIGHT version - no video encoding (saves memory)"
    echo ""
    echo "Example:"
    echo "  $0 ~/tetheria/tidyverse-hand/data/rosbag2_xxx ~/tetheria/tidyverse-hand/data/lerobot_format/ep1"
    exit 1
fi

INPUT_BAG="$1"
OUTPUT_DIR="$2"
REPO_ID="${3:-local/hand_teleop}"

echo "Converting rosbag to LeRobot format (LITE - no video)"
echo "  Input: $INPUT_BAG"
echo "  Output: $OUTPUT_DIR"
echo "  Repo ID: $REPO_ID"
echo ""

# Run with nice to lower priority
# NO --use-video flag = skip video encoding
nice -n 10 python3 ~/tetheria/tidyverse-hand/tools/convert_rosbag_to_lerobot.py \
    --input-dir "$INPUT_BAG" \
    --output-root "$OUTPUT_DIR" \
    --repo-id "$REPO_ID" \
    --robot-type "aero_hand" \
    --fps 30 \
    --task "hand teleop demonstration" \
    --topic-cmd-vel "/spacemouse/cmd_vel" \
    --topic-joint-states "/joint_states" \
    --topic-hand-control "/right/joint_control" \
    --topic-actuator-states "/right/actuator_states" \
    --topic-manus "/manus_glove_0" \
    --arm-joint-dim 7 \
    --hand-joint-dim 16 \
    --num-actuators 7 \
    --manus-dim 20 \
    --sync-mode sample \
    --episode-segmentation none \
    --overwrite \
    --debug-topics

echo ""
echo "Done! Dataset saved to: $OUTPUT_DIR"
echo ""
echo "NOTE: Video was skipped. To add video later on a powerful machine:"
echo "  - Re-run with --use-video --camera-head /camera_0/color"
