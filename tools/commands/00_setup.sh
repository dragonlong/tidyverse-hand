#!/bin/bash
# Setup script - install required Python dependencies in tidybot2 environment
#
# Usage: ./00_setup.sh

set -e

# Activate tidybot2 conda environment
echo "Activating tidybot2 environment..."
source ~/miniforge3/etc/profile.d/conda.sh 2>/dev/null || source ~/mambaforge/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
mamba activate tidybot2 || conda activate tidybot2

echo "Installing Python dependencies for rosbag conversion tools..."

# rosbags - for reading ROS2 rosbag files (db3 and mcap)
pip install rosbags

# lerobot - for LeRobot dataset format (if not already installed)
pip install lerobot 2>/dev/null || echo "lerobot may already be installed or requires special setup"

# Other useful packages
pip install matplotlib pillow numpy

# Optional: mcap support
pip install mcap mcap-ros2-support 2>/dev/null || echo "mcap-ros2-support not available, using rosbags instead"

echo ""
echo "Setup complete!"
echo ""
echo "To use these tools:"
echo "  1. Activate conda environment: mamba activate tidybot2"
echo "  2. Source ROS2 workspace:"
echo "     cd ~/tetheria/aero-open-ros2"
echo "     source /opt/ros/humble/setup.bash"
echo "     source install/setup.bash"
