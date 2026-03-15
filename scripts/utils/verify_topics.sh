#!/bin/bash
# ─── VERIFY TELEOP TOPICS ─────────────────────────────────────────────────────
# Check all expected topics are publishing at the right rate.
# Run this after starting the full teleop stack to confirm everything is live.
# ──────────────────────────────────────────────────────────────────────────────

source /opt/ros/humble/setup.bash
cd /home/dragonx/tetheria/aero-open-ros2
source install/setup.bash

TIMEOUT=4

check_topic() {
    local topic=$1
    local expected_hz=$2
    echo -n "  $topic ... "
    hz=$(timeout $TIMEOUT ros2 topic hz "$topic" 2>/dev/null | grep -oP 'average rate: \K[\d.]+' | head -1)
    if [ -z "$hz" ]; then
        echo "NOT PUBLISHING ✗"
    else
        echo "${hz} Hz ✓  (expected ~${expected_hz} Hz)"
    fi
}

echo ""
echo "=== Base ==="
check_topic "/spacemouse/cmd_vel"         100

echo ""
echo "=== Arm ==="
check_topic "/joint_states"               30

echo ""
echo "=== Hand (input) ==="
check_topic "/hands/right/landmarks"      70
check_topic "/hands/right/wrist_pose"     70

echo ""
echo "=== Hand (output) ==="
check_topic "/right/joint_control"        30
check_topic "/right/actuator_states"      100

echo ""
echo "=== Active nodes ==="
ros2 node list

echo ""
echo "=== All topics ==="
ros2 topic list
