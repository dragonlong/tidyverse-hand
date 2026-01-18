#!/bin/bash
# Inspect a rosbag2 recording to see topics, message counts, and duration
#
# Usage: ./01_inspect_rosbag.sh /path/to/rosbag_folder

set -e

# Activate tidybot2 conda environment
source ~/miniforge3/etc/profile.d/conda.sh 2>/dev/null || source ~/mambaforge/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
mamba activate tidybot2 || conda activate tidybot2

# Source ROS2 workspace for custom messages
cd ~/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

if [ -z "$1" ]; then
    echo "Usage: $0 /path/to/rosbag_folder"
    echo ""
    echo "Available rosbags in ~/tetheria/aero-open-ros2:"
    ls -d ~/tetheria/aero-open-ros2/rosbag2_* 2>/dev/null || echo "  (none found)"
    exit 1
fi

ROSBAG_PATH="$1"

echo "=========================================="
echo "Rosbag Info: $ROSBAG_PATH"
echo "=========================================="
ros2 bag info "$ROSBAG_PATH"

echo ""
echo "=========================================="
echo "Sample messages (first 3 of each key topic)"
echo "=========================================="

# Play bag slowly and echo topics (run for a short time)
echo ""
echo "To inspect messages interactively, run in two terminals:"
echo "  Terminal 1: ros2 bag play $ROSBAG_PATH --rate 0.1"
echo "  Terminal 2: ros2 topic echo /right/joint_control"
echo ""
echo "Or use the Python inspection tool:"
echo "  python ~/tetheria/tidyverse-hand/tools/inspect_rosbag.py --input $ROSBAG_PATH"
