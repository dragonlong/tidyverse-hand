#!/bin/bash
# ─── HAND TELEOPERATION ────────────────────────────────────────────────────────
# Quest 3 → /right/joint_control  (aero_hand_open_msgs/JointControl, 16 DOF)
#
# Starts two nodes:
#   1. hand_tracking_bridge  — receives Quest 3 UDP hand-landmark stream
#   2. quest3_retargeting    — maps 21-point skeleton to 16 Aero Hand DOFs
#
# The aero_hand_node on the mobile PC subscribes to /right/joint_control over
# the shared ROS_DOMAIN_ID.
#
# Run on: LOCAL PC (receives UDP from Quest 3 headset over Wi-Fi/LAN)
# Quest 3 app: hand-tracking-streamer (streams to port 5555 by default)
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

source /opt/ros/humble/setup.bash
cd /home/dragonx/tetheria/aero-open-ros2
source install/setup.bash

ros2 launch yam_teleop quest3_full_teleop.launch.py
