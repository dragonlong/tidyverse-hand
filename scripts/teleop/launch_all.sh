#!/bin/bash
# ─── TELEOP PUBLISHERS — LOCAL PC ─────────────────────────────────────────────
# Starts all input-device publisher nodes in a single tmux session.
#
# Layout (3 panes, horizontal):
#   ┌──────────────────┬──────────────────┬───────────────────────┐
#   │ 0: SpaceMouse    │ 1: Gello Arm     │ 2: Quest 3 Hand       │
#   │ /spacemouse/     │ /joint_states    │ /right/joint_control  │
#   │ cmd_vel          │ (30 Hz)          │ (bridge + retarget)   │
#   └──────────────────┴──────────────────┴───────────────────────┘
#
# These nodes publish ROS topics consumed by the edge device hardware nodes.
# Run the edge device side separately:
#   ssh <edge-device> "bash ~/tetheria/tidyverse-hand/scripts/hardware/launch_all.sh"
#
# Navigation:  Ctrl+b + ←/→  move panes  |  Ctrl+b + z  zoom  |  Ctrl+b + d  detach
# Stop:        bash utils/kill_teleop.sh
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

TELEOP="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="teleop"

tmux kill-session -t $SESSION 2>/dev/null || true
tmux new-session  -d -s $SESSION -n main

tmux split-window -h -t $SESSION:main
tmux split-window -h -t $SESSION:main
tmux select-layout -t $SESSION:main even-horizontal

tmux send-keys -t $SESSION:main.0 "bash $TELEOP/01_spacemouse.sh"    C-m
tmux send-keys -t $SESSION:main.1 "bash $TELEOP/02_gello_arm.sh"     C-m
tmux send-keys -t $SESSION:main.2 "bash $TELEOP/03_quest3_hand.sh"   C-m

tmux select-pane -t $SESSION:main.0
tmux attach-session -t $SESSION
