#!/bin/bash
# Inspect a rosbag2 recording to see topics, message counts, and duration.
# Use this to find the correct topic names for conversion (arm, hand, etc.).
#
# Usage: ./01_inspect_rosbag.sh /path/to/rosbag_folder

set -e

# Activate tidybot2 conda environment (for rosbags + custom msg types)
source ~/miniforge3/etc/profile.d/conda.sh 2>/dev/null || source ~/mambaforge/etc/profile.d/conda.sh 2>/dev/null || source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null
mamba activate tidybot2 2>/dev/null || conda activate tidybot2 2>/dev/null || true

if [ -z "$1" ]; then
    echo "Usage: $0 /path/to/rosbag_folder"
    echo ""
    echo "Available rosbags in ~/tetheria/aero-open-ros2:"
    ls -d ~/tetheria/aero-open-ros2/rosbag2_* 2>/dev/null || echo "  (none found)"
    exit 1
fi

ROSBAG_PATH="$1"
TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSPECT_PY="${TOOLS_DIR}/inspect_rosbag.py"

if [ ! -d "$ROSBAG_PATH" ] || [ ! -f "${ROSBAG_PATH}/metadata.yaml" ]; then
    echo "ERROR: Not a rosbag directory: $ROSBAG_PATH"
    exit 1
fi

echo "=========================================="
echo "Rosbag Info (ros2 bag info)"
echo "=========================================="
# Source ROS2 for ros2 bag info (optional; if not available we still run Python inspector)
(cd ~/tetheria/aero-open-ros2 2>/dev/null && source /opt/ros/humble/setup.bash 2>/dev/null && source install/setup.bash 2>/dev/null && ros2 bag info "$ROSBAG_PATH") 2>/dev/null || echo "(ros2 bag info skipped - source ROS2 to enable)"

echo ""
echo "=========================================="
echo "Topics in bag (name, type, count) — set 02_batch_convert topic vars to match"
echo "=========================================="
if [ -f "$INSPECT_PY" ]; then
    python3 "$INSPECT_PY" --input "$ROSBAG_PATH" 2>/dev/null || true
else
    echo "Inspect script not found: $INSPECT_PY"
fi

echo ""
echo "=========================================="
echo "Topics needed for GR00T LeRobot conversion"
echo "=========================================="
echo "  Arm (JointState):     /joint_states  or  /right/gello_js"
echo "  Hand target (JointControl):  /right/joint_control"
echo "  Hand state (ActuatorStates): /right/actuator_states"
echo "  Base/teleop (Twist):  /spacemouse/cmd_vel"
echo "  Glove (ManusGlove):   /manus_glove_0"
echo ""
echo "If your bag uses different names, edit 02_batch_convert_2026_01_18.sh"
echo "and set TOPIC_* and ARM_JOINT_STATES_TOPIC at the top."
echo ""
echo "To sample a topic: python3 $INSPECT_PY --input $ROSBAG_PATH --topic /topic/name --sample 3"
echo "To plot hand joints: python3 $INSPECT_PY --input $ROSBAG_PATH --plot-joints"
