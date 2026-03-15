#!/bin/bash
# ─── SIM: MuJoCo Aero Hand Viewer ─────────────────────────────────────────────
# Visualizes /right/joint_control in MuJoCo — use to verify hand retargeting
# without connecting physical hardware.
#
# Typical workflow (no hardware):
#   Terminal 1:  bash 04_quest3_hand.sh   (bridge + retargeting)
#   Terminal 2:  bash sim_mujoco_hand.sh  (this — visualization only)
#
# Prerequisite: pip install mujoco
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCENE="${SCENE:-/home/dragonx/tetheria/mujoco_menagerie/tetheria_aero_hand_open/scene_right.xml}"

source /opt/ros/humble/setup.bash
cd /home/dragonx/tetheria/aero-open-ros2
source install/setup.bash

echo "Subscribing to /right/joint_control"
echo "Scene: $SCENE"

ros2 run aero_hand_open_teleop mujoco_hand_viewer \
  --ros-args -p scene_xml:="$SCENE"
