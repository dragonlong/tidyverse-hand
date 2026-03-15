#!/bin/bash
#
# Step 6: Plot arm joint ACTION vs STATE curves for one rosbag episode.
#
# Saves PNGs to disk for visual verification of the conversion and arm tracking.
# Use this to sanity-check that joint_states are being parsed correctly and that
# the arm follows the commanded trajectory.
#
# Usage:
#   cd ~/tetheria/tidyverse-hand/tools/commands
#   ./06_plot_joints.sh                              # first matching bag, full length
#   ./06_plot_joints.sh /path/to/rosbag2_xxx         # specific bag
#   MAX_DURATION_SEC=30 ./06_plot_joints.sh /path/to/bag   # first 30 s only
#
# Environment overrides:
#   INPUT_DIR        Root dir to scan for bags (default: /mnt/sandisk/data/tidyverse-hand-2026-01-19)
#   PATTERN          Glob pattern for bags to scan (default: rosbag2_2026_01_18*)
#   OUTPUT_DIR       Where to save PNGs (default: tools/plots_arm_action_state)
#   ACTION_TOPIC     ROS topic for commanded joint angles (default: /joint_states)
#   STATE_TOPIC      ROS topic for actual joint angles   (default: /joint_states)
#   MAX_DURATION_SEC Crop to first N seconds (default: full bag)

set -e

INPUT_DIR="${INPUT_DIR:-/mnt/sandisk/data/tidyverse-hand-2026-01-19}"
PATTERN="${PATTERN:-rosbag2_2026_01_18*}"
OUTPUT_DIR="${OUTPUT_DIR:-$(dirname "$0")/../plots_arm_action_state}"
ACTION_TOPIC="${ACTION_TOPIC:-/joint_states}"
STATE_TOPIC="${STATE_TOPIC:-/joint_states}"
MAX_DURATION_SEC="${MAX_DURATION_SEC:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLOT_SCRIPT="${SCRIPT_DIR}/plot_arm_action_state.py"

if [[ ! -f "$PLOT_SCRIPT" ]]; then
    echo "ERROR: Plot script not found: $PLOT_SCRIPT"
    exit 1
fi

# Resolve bag path
if [[ -n "$1" ]]; then
    BAG_PATH="$1"
    if [[ ! -d "$BAG_PATH" ]] || [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
        echo "ERROR: Not a rosbag directory: $BAG_PATH"
        exit 1
    fi
else
    BAG_PATH=""
    while IFS= read -r -d '' d; do
        if [[ -f "${d}/metadata.yaml" ]]; then
            BAG_PATH="$d"
            break
        fi
    done < <(find "$INPUT_DIR" -maxdepth 1 -type d -name "$PATTERN" -print0 | sort -z)
    if [[ -z "$BAG_PATH" ]]; then
        echo "ERROR: No rosbag matching '$PATTERN' found in $INPUT_DIR"
        echo "Usage: $0 [/path/to/rosbag_directory]"
        exit 1
    fi
fi

echo "=============================================="
echo "Plot arm joints (action vs state)"
echo "=============================================="
echo "  Bag:    $BAG_PATH"
echo "  Output: $OUTPUT_DIR"
[[ -n "$MAX_DURATION_SEC" ]] && echo "  Crop:   first ${MAX_DURATION_SEC}s"
echo ""

mkdir -p "$OUTPUT_DIR"

PLOT_ARGS=(
    --input "$BAG_PATH"
    --output-dir "$OUTPUT_DIR"
    --action-topic "$ACTION_TOPIC"
    --state-topic "$STATE_TOPIC"
    --arm-joint-names "joint1,joint2,joint3,joint4,joint5,joint6"
    --arm-joint-dim 6
)
[[ -n "$MAX_DURATION_SEC" ]] && PLOT_ARGS+=(--max-duration "$MAX_DURATION_SEC")

python3 "$PLOT_SCRIPT" "${PLOT_ARGS[@]}"

echo ""
echo "Plots saved to: $OUTPUT_DIR"
