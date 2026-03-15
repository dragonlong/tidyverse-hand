#!/bin/bash
# ─── BASE CONTROL ─────────────────────────────────────────────────────────────
# SpaceMouse → spacemouse/cmd_vel  (geometry_msgs/Twist, 100 Hz)
#
# Publishes linear.x / linear.y / angular.z from the 3Dconnexion SpaceMouse.
# The mobile base subscribes to spacemouse/cmd_vel.
#
# Run on: LOCAL PC (USB SpaceMouse plugged in here)
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

cd /home/dragonx/tetheria/aero-open-ros2
python3 src/aero_hand_open_teleop/aero_hand_open_teleop/spacemouse.py \
  --ros-args \
  -p rate_hz:=100.0 \
  -p deadzone:=0.05 \
  -p scale_x:=1.0 \
  -p scale_y:=1.0 \
  -p scale_theta:=1.0
