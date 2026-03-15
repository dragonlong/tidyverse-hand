#!/bin/bash
# ─── HARDWARE CONTROL NODES — EDGE DEVICE ─────────────────────────────────────
# Starts all hardware control nodes in a single tmux session.
#
# Layout (2 panes, horizontal):
#   ┌──────────────────────────────┬──────────────────────────────┐
#   │ 0: YAM Arm Controller        │ 1: Aero Hand Node            │
#   │ /joint_states → CAN          │ /right/joint_control → serial│
#   └──────────────────────────────┴──────────────────────────────┘
#
# These nodes subscribe to topics published by the local PC teleop publishers.
# Ensure ROS_DOMAIN_ID matches on both machines.
#
# Set RIGHT_PORT before running:
#   export RIGHT_PORT=/dev/serial/by-id/usb-FTDI_...
#   bash hardware/launch_all.sh
#
# Stop:  bash utils/kill_teleop.sh
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

HW="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="teleop"

if [ -z "${RIGHT_PORT:-}" ]; then
    echo "ERROR: RIGHT_PORT is not set."
    echo "  export RIGHT_PORT=/dev/serial/by-id/usb-FTDI_..."
    echo "  Find it with: ls /dev/serial/by-id/ | grep FTDI"
    exit 1
fi

tmux kill-session -t $SESSION 2>/dev/null || true
tmux new-session  -d -s $SESSION -n main

tmux split-window -h -t $SESSION:main
tmux select-layout -t $SESSION:main even-horizontal

tmux send-keys -t $SESSION:main.0 "bash $HW/01_yam_arm.sh"    C-m
tmux send-keys -t $SESSION:main.1 "bash $HW/02_aero_hand.sh"  C-m

tmux select-pane -t $SESSION:main.0
tmux attach-session -t $SESSION
