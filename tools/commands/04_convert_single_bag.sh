#!/bin/bash
# Convert a single rosbag to LeRobot format (lighter weight)
#
# Usage: ./04_convert_single_bag.sh /path/to/rosbag_folder /path/to/output [repo_id]
#
# This script processes one bag at a time to reduce memory/CPU load

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
    echo "Example:"
    echo "  $0 ~/tetheria/aero-open-ros2/rosbag2_2026_01_16-17_09_38 ~/tidybot2/data/ep1"
    exit 1
fi

INPUT_BAG="$1"
OUTPUT_DIR="$2"
REPO_ID="${3:-local/hand_teleop}"

echo "Converting single rosbag to LeRobot format"
echo "  Input: $INPUT_BAG"
echo "  Output: $OUTPUT_DIR"
echo "  Repo ID: $REPO_ID"
echo ""

# Run with nice to lower priority and avoid overwhelming the system
nice -n 10 python ~/tetheria/tidyverse-hand/tools/convert_rosbag_to_lerobot.py \
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
    --camera-head "/camera_0/color" \
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
