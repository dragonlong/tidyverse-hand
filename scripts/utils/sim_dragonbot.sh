#!/bin/bash
# ─── SIM: MuJoCo Full Dragonbot Viewer ────────────────────────────────────────
# Visualizes all three Dragonbot input streams in a single MuJoCo window:
#   /spacemouse/cmd_vel   → mobile base (x, y, θ) — velocity integrated live
#   /joint_states         → YAM arm (joint1–joint6)
#   /right/joint_control  → Aero Hand (16 joints)
#
# The Aero Hand is attached to link_6 of the YAM arm, which sits on the
# TidyBot mobile base — the full robot kinematic chain is visible.
#
# Typical workflow:
#   Terminal 1:  bash scripts/teleop/01_spacemouse.sh    (SpaceMouse → base)
#   Terminal 2:  bash scripts/teleop/02_gello_arm.sh     (Gello → arm)
#   Terminal 3:  bash scripts/teleop/03_quest3_hand.sh   (Quest 3 → hand)
#   Terminal 4:  bash scripts/utils/sim_dragonbot.sh     (this — full viewer)
#
# Arm + hand only (skip spacemouse):
#   Just omit Terminal 1 — the base stays at origin.
#
# Override scene:
#   SCENE=/path/to/scene.xml bash scripts/utils/sim_dragonbot.sh
#
# Prerequisites:
#   pip install mujoco
#   cd ~/tetheria/aero-open-ros2 && colcon build --packages-select aero_hand_open_teleop
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCENE="${SCENE:-/home/dragonx/tetheria/tidyverse-hand/models/dragonbot/scene.xml}"

source /opt/ros/humble/setup.bash
cd /home/dragonx/tetheria/aero-open-ros2
source install/setup.bash

echo "Starting full Dragonbot MuJoCo viewer"
echo "Scene: $SCENE"
echo "Subscribing to /spacemouse/cmd_vel, /joint_states, /right/joint_control"

ros2 run aero_hand_open_teleop mujoco_dragonbot_viewer \
  --ros-args -p scene_xml:="$SCENE"
