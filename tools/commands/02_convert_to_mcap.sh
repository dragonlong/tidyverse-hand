#!/bin/bash
# Convert rosbag2 (db3/sqlite3) to MCAP format
#
# Usage: ./02_convert_to_mcap.sh /path/to/rosbag_folder [/path/to/output.mcap]

set -e

# Activate tidybot2 conda environment
source ~/miniforge3/etc/profile.d/conda.sh 2>/dev/null || source ~/mambaforge/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
mamba activate tidybot2 || conda activate tidybot2

# Source ROS2 workspace for custom messages
cd ~/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

if [ -z "$1" ]; then
    echo "Usage: $0 /path/to/rosbag_folder [/path/to/output.mcap]"
    echo ""
    echo "If output path is not specified, will create <rosbag_name>.mcap in current directory"
    exit 1
fi

INPUT_PATH="$1"
ROSBAG_NAME=$(basename "$INPUT_PATH")

if [ -z "$2" ]; then
    OUTPUT_PATH="./${ROSBAG_NAME}.mcap"
else
    OUTPUT_PATH="$2"
fi

echo "Converting: $INPUT_PATH -> $OUTPUT_PATH"

# Method 1: Using ros2 bag convert (if mcap plugin is available)
# ros2 bag convert -i "$INPUT_PATH" -o "${OUTPUT_PATH%.*}" -s mcap

# Method 2: Using Python script
python3 ~/tetheria/tidyverse-hand/tools/convert_db3_to_mcap.py \
    --input "$INPUT_PATH" \
    --output "$OUTPUT_PATH" \
    --overwrite

echo "Done! Output: $OUTPUT_PATH"
