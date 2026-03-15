#!/bin/bash
# ─── ARM TELEOPERATION — INPUT SIDE ───────────────────────────────────────────
# Gello → /joint_states  (sensor_msgs/JointState, 30 Hz)
#
# Reads the Gello replica arm (6-DOF Dynamixel) and publishes joint positions
# to /joint_states.  The YAM arm controller (03_yam_arm.sh) subscribes on the
# mobile PC to mirror these positions onto the real arm.
#
# Run on: LOCAL PC (Gello USB-serial plugged in here)
# Prerequisite: pip install dynamixel-sdk evdev pyudev
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# Drop conda env if active — use system Python for ROS 2
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniforge3 | tr '\n' ':' | sed 's/:$//')
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE CONDA_SHLVL 2>/dev/null || true

cd /home/dragonx/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/gello_controller/gello_controller/gello_publisher.py
