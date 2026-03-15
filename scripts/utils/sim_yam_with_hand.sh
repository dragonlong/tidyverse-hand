#!/bin/bash
# ─── SIM: MuJoCo YAM Arm + Aero Hand Combined Viewer ─────────────────────────
# Visualizes /joint_states (6 arm joints) and /right/joint_control (16 hand
# joints) in a single MuJoCo window — the Aero Hand is attached to link_6 of
# the YAM arm so arm and hand move together.
#
# Typical workflow (no hardware):
#   Terminal 1:  bash scripts/teleop/02_gello_arm.sh     (Gello → /joint_states)
#   Terminal 2:  bash scripts/teleop/03_quest3_hand.sh   (Quest 3 → /right/joint_control)
#   Terminal 3:  bash scripts/utils/sim_yam_with_hand.sh (this — visualization)
#
# Override scene:
#   SCENE=/path/to/scene.xml bash scripts/utils/sim_yam_with_hand.sh
#
# Prerequisite: pip install mujoco
#   Build: cd ~/tetheria/aero-open-ros2 && colcon build --packages-select aero_hand_open_teleop
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCENE="${SCENE:-/home/dragonx/tetheria/tidyverse-hand/models/yam_with_hand/scene_yam_with_hand.xml}"

source /opt/ros/humble/setup.bash
cd /home/dragonx/tetheria/aero-open-ros2
source install/setup.bash

echo "Starting combined YAM arm + Aero Hand MuJoCo viewer"
echo "Scene: $SCENE"
echo "Subscribing to /joint_states and /right/joint_control"

ros2 run aero_hand_open_teleop mujoco_yam_hand_viewer \
  --ros-args -p scene_xml:="$SCENE"
