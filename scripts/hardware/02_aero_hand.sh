#!/bin/bash
# ─── HAND HARDWARE DRIVER ─────────────────────────────────────────────────────
# /right/joint_control → Aero Hand (Serial UART @ 921600 baud)
#
# aero_hand_node subscribes to /right/joint_control (16-DOF finger targets in
# radians) and drives the Aero Hand via serial.  Also publishes actuator
# feedback on /right/actuator_states at 100 Hz.
#
# Run on: MOBILE PC (Aero Hand serial port connected here)
#
# Set RIGHT_PORT to the correct /dev/serial/by-id/... path before running.
# Find it with: ls /dev/serial/by-id/ | grep FTDI
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

RIGHT_PORT="${RIGHT_PORT:-/dev/serial/by-id/usb-FTDI_USB_Serial_Cable_XXXXXXXX-if00-port0}"
BAUD=921600
FEEDBACK_HZ=100.0

source /opt/ros/humble/setup.bash
cd /home/dragonx/tetheria/aero-open-ros2
source install/setup.bash

echo "Starting aero_hand_node on port: $RIGHT_PORT"

ros2 run aero_hand_open aero_hand_node \
  --ros-args \
  -p right_port:="$RIGHT_PORT" \
  -p baudrate:=$BAUD \
  -p feedback_frequency:=$FEEDBACK_HZ \
  -p control_space:=joint
