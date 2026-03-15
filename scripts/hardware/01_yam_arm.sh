#!/bin/bash
# ─── ARM TELEOPERATION — HARDWARE SIDE ────────────────────────────────────────
# /joint_states → YAM Arm (CAN bus)
#
# Subscribes to /joint_states published by 02_gello_arm.sh (local PC) and
# forwards joint position commands to the YAM arm over CAN.
#
# Run on: MOBILE PC (YAM arm CAN interface connected here, e.g. can0)
# Prerequisite: i2rt SDK, CAN interface up (sudo ip link set can0 up type can bitrate 1000000)
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# Drop conda env if active — use system Python for ROS 2
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v miniforge3 | tr '\n' ':' | sed 's/:$//')
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_EXE CONDA_PYTHON_EXE CONDA_SHLVL 2>/dev/null || true

cd /home/dragonx/tetheria/aero-open-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/gello_controller/gello_controller/yam_gello_controller.py
