#!/bin/bash
#
# Test run: plot arm ACTION (gello desired) vs STATE (actual) for one rosbag sequence.
# Saves PNGs to disk for visual verification of parsing.
#
# Plot arm ACTION vs STATE for one rosbag (visualize one example episode).
# yam_gello_controller subscribes to /joint_states; defaults use that topic for both.
#
# If you get "No messages on action/state topic", run:
#   ./01_inspect_rosbag.sh /path/to/your/bag
# then set ACTION_TOPIC and STATE_TOPIC (or env) to match.
#
# Usage:
#   cd ~/tetheria/tidyverse-hand/tools/commands
#   ./06_plot_arm_action_state_test.sh                          # first bag, full length
#   ./06_plot_arm_action_state_test.sh /path/to/rosbag2_xxx     # this bag
#   MAX_DURATION_SEC=60 ./06_plot_arm_action_state_test.sh      # first 60s only (one episode)
#

set -e

INPUT_DIR="${INPUT_DIR:-/mnt/sandisk/data/tidyverse-hand-2026-01-19}"
PATTERN="${PATTERN:-rosbag2_2026_01_18*}"
OUTPUT_DIR="${OUTPUT_DIR:-$(dirname "$0")/../plots_arm_action_state}"
# Topic names (match 02_batch_convert TOPIC_JOINT_STATES / ARM_JOINT_STATES_TOPIC)
ACTION_TOPIC="${ACTION_TOPIC:-/joint_states}"
STATE_TOPIC="${STATE_TOPIC:-/joint_states}"
# Optional: plot only first N seconds (e.g. 60 for one episode)
MAX_DURATION_SEC="${MAX_DURATION_SEC:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLOT_SCRIPT="${SCRIPT_DIR}/plot_arm_action_state.py"

if [[ ! -f "$PLOT_SCRIPT" ]]; then
    echo "ERROR: Plot script not found: $PLOT_SCRIPT"
    exit 1
fi

# Resolve one bag
if [[ -n "$1" ]]; then
    BAG_PATH="$1"
    if [[ ! -d "$BAG_PATH" ]]; then
        echo "ERROR: Bag path is not a directory: $BAG_PATH"
        exit 1
    fi
    if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
        echo "ERROR: Not a rosbag directory (no metadata.yaml): $BAG_PATH"
        exit 1
    fi
else
    # First matching bag under INPUT_DIR
    BAG_PATH=""
    while IFS= read -r -d '' d; do
        if [[ -f "${d}/metadata.yaml" ]]; then
            BAG_PATH="$d"
            break
        fi
    done < <(find "$INPUT_DIR" -maxdepth 1 -type d -name "$PATTERN" -print0 | sort -z)
    if [[ -z "$BAG_PATH" ]]; then
        echo "ERROR: No rosbag found matching $PATTERN in $INPUT_DIR"
        echo "Usage: $0 [path/to/rosbag_directory]"
        exit 1
    fi
fi

echo "=============================================="
echo "Plot arm ACTION vs STATE (test run)"
echo "=============================================="
echo "Bag:    $BAG_PATH"
echo "Output: $OUTPUT_DIR"
echo ""

mkdir -p "$OUTPUT_DIR"

# System matplotlib is built for NumPy 1.x; default python3 may have NumPy 2.x.
# Ensure numpy<2 so matplotlib works with default env python3.
if python3 -c "import numpy; exit(0 if numpy.__version__.startswith('1.') else 1)" 2>/dev/null; then
    :
else
    echo "Installing numpy<2 for compatibility with system matplotlib..."
    python3 -m pip install --user 'numpy<2' -q 2>/dev/null || true
fi

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
echo "Plots saved under: $OUTPUT_DIR"
ls -la "$OUTPUT_DIR"
