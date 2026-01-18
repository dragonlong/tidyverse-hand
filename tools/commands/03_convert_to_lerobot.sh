#!/bin/bash
# Convert rosbag2 recordings to LeRobot dataset format
#
# Usage: ./03_convert_to_lerobot.sh /path/to/rosbags_dir /path/to/output_dataset [repo_id] [sync_mode]
#
# Sync modes for handling high-frequency (100Hz) control data:
#   - sample:  Take latest value before each frame (zero-order hold) - DEFAULT
#   - average: Average all values in frame interval (smoothing)
#   - sum:     Sum all delta commands in frame interval (accumulate deltas)
#
# For delta velocity control at 100Hz downsampled to 30fps:
#   - Use "sample" if each command is an absolute target
#   - Use "average" if you want smoothed velocities
#   - Use "sum" if each command is a delta that should accumulate

set -e

# Activate tidybot2 conda environment
source ~/miniforge3/etc/profile.d/conda.sh 2>/dev/null || source ~/mambaforge/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
mamba activate tidybot2 || conda activate tidybot2

# Source ROS2 workspace for custom messages
cd ~/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: $0 /path/to/rosbags_dir /path/to/output_dataset [repo_id] [sync_mode]"
    echo ""
    echo "Arguments:"
    echo "  rosbags_dir      Directory containing rosbag2 folders"
    echo "  output_dataset   Output LeRobot dataset directory"
    echo "  repo_id          Optional: Dataset repo ID (default: local/hand_teleop)"
    echo "  sync_mode        Optional: sample|average|sum (default: sample)"
    echo ""
    echo "Sync modes:"
    echo "  sample  - Take latest value before each frame (zero-order hold)"
    echo "  average - Average all values in frame interval"
    echo "  sum     - Sum all delta commands in frame interval"
    echo ""
    echo "Example:"
    echo "  $0 ~/tetheria/tidyverse-hand/data ~/tetheria/tidyverse-hand/data/lerobot_format/combined"
    echo "  $0 ~/tetheria/tidyverse-hand/data/rosbag_ep1 ~/tetheria/tidyverse-hand/data/lerobot_format/ep1 local/ep1"
    exit 1
fi

INPUT_DIR="$1"
OUTPUT_DIR="$2"
REPO_ID="${3:-local/hand_teleop}"
SYNC_MODE="${4:-sample}"

echo "Converting rosbags to LeRobot format"
echo "  Input: $INPUT_DIR"
echo "  Output: $OUTPUT_DIR"
echo "  Repo ID: $REPO_ID"
echo "  Sync mode: $SYNC_MODE"
echo ""

python3 ~/tetheria/tidyverse-hand/tools/convert_rosbag_to_lerobot.py \
    --input-dir "$INPUT_DIR" \
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
    --sync-mode "$SYNC_MODE" \
    --overwrite \
    --debug-topics

# To add more cameras, use:
#    --camera-wrist "/camera_4/color" \
#    --camera-base "/logitech_base/color" \

echo ""
echo "Done! Dataset saved to: $OUTPUT_DIR"
