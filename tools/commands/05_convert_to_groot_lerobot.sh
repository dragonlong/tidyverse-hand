#!/bin/bash
# Convert rosbag to GR00T LeRobot v2 format (for fine-tuning GR00T N1 models)
#
# Usage: ./05_convert_to_groot_lerobot.sh /path/to/rosbag /path/to/output [repo_id] [episode_segmentation]
#
# Episode segmentation methods:
#   - none:      Treat entire bag as single episode (default)
#   - time_gap:  Segment by time gaps (pauses > 2s)
#   - joint_home: Segment when arm returns to home position
#   - audio:     Segment by audio commands [placeholder]
#
# Output structure:
#   ├─meta/
#   │ ├─episodes.jsonl
#   │ ├─modality.json (GR00T specific)
#   │ ├─info.json
#   │ └─tasks.jsonl
#   ├─videos/chunk-000/observation.images.<view>/
#   │ └─episode_XXXXXX.mp4
#   └─data/chunk-000/
#     └─episode_XXXXXX.parquet

set -e

# Activate tidybot2 conda environment
source ~/miniforge3/etc/profile.d/conda.sh 2>/dev/null || source ~/mambaforge/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
mamba activate tidybot2 || conda activate tidybot2

# Source ROS2 workspace for custom messages
cd ~/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: $0 /path/to/rosbag_folder /path/to/output [repo_id] [episode_segmentation]"
    echo ""
    echo "Converts rosbag to GR00T LeRobot v2 format for fine-tuning GR00T N1 models."
    echo ""
    echo "Episode segmentation methods:"
    echo "  none      - Treat entire bag as single episode (default)"
    echo "  time_gap  - Segment by time gaps (pauses > 2s)"
    echo "  joint_home - Segment when arm returns to home"
    echo "  audio     - Segment by audio commands [placeholder]"
    echo ""
    echo "Example:"
    echo "  $0 ~/tetheria/tidyverse-hand/data/rosbag2_xxx ~/tetheria/tidyverse-hand/data/lerobot_v2/ep1"
    echo "  $0 ~/tetheria/tidyverse-hand/data/rosbag2_xxx ~/tetheria/tidyverse-hand/data/lerobot_v2/ep1 local/teleop time_gap"
    exit 1
fi

INPUT_BAG="$1"
OUTPUT_DIR="$2"
REPO_ID="${3:-local/hand_teleop}"
EPISODE_SEG="${4:-none}"

echo "Converting rosbag to GR00T LeRobot v2 format"
echo "  Input: $INPUT_BAG"
echo "  Output: $OUTPUT_DIR"
echo "  Repo ID: $REPO_ID"
echo "  Episode segmentation: $EPISODE_SEG"
echo ""

python3 ~/tetheria/tidyverse-hand/tools/convert_rosbag_to_groot_lerobot.py \
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
    --episode-segmentation "$EPISODE_SEG" \
    --overwrite \
    --debug-topics 2>&1

echo ""
echo "Done! GR00T LeRobot v2 dataset saved to: $OUTPUT_DIR"
