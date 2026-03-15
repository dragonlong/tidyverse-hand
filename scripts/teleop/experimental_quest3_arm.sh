#!/bin/bash
# ─── EXPERIMENTAL: Quest 3 Wrist → YAM Arm ────────────────────────────────────
# Quest 3 wrist pose → IK (pyroki) → /joint_states → YAM Arm
#
# Replaces Gello (02_gello_arm.sh) for arm control using Quest 3 wrist tracking.
# Still requires 03_yam_arm.sh on the mobile PC.
#
# WARNING: Experimental — workspace limits and IK stability not fully validated.
# For data collection use 02_gello_arm.sh (Gello) instead.
#
# Run on: LOCAL PC (same machine as 04_quest3_hand.sh)
# Note: hand_tracking_bridge is already launched by 04_quest3_hand.sh —
#       do NOT run this alongside 04_quest3_hand.sh (duplicate bridge conflict).
#       Use this as a standalone arm-only replacement for Gello.
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

source /opt/ros/humble/setup.bash
cd /home/dragonx/tetheria/aero-open-ros2
source install/setup.bash

# Launch bridge + Quest 3 wrist → YAM arm IK node
ros2 launch yam_teleop quest3_yam_teleop.launch.py
