#!/bin/bash
#
# Batch convert rosbag2_2026_01_18* data to GR00T LeRobot v2 format
#
# Data source: /mnt/sandisk/data/tidyverse-hand-2026-01-19/
# Output: /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18/
#
# Camera configuration (verified via ros2 bag info):
#   - Head camera:  /camera_0/color
#   - Wrist camera: /camera_4/color
#   - Base camera:  /logitech_base/color
#
# Prerequisites:
#   - ROS2 Humble
#   - ffmpeg (for video encoding)
#   - Python packages: rosbags, pyarrow, pillow, numpy
#
# Usage:
#   cd ~/tetheria/tidyverse-hand/tools/commands
#   ./02_batch_convert_2026_01_18.sh
#
# Or with options:
#   ./02_batch_convert_2026_01_18.sh --no-video  # Skip video encoding (faster)
#   ./02_batch_convert_2026_01_18.sh --dry-run   # Just show what would be processed
#

set -e

# ============================================================================
# Configuration
# ============================================================================

INPUT_DIR="/mnt/sandisk/data/tidyverse-hand-2026-01-19"
OUTPUT_ROOT="/mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18"
PATTERN="rosbag2_2026_01_18*"
REPO_ID="tidyverse_hand_2026_01_18"
FPS=30
TASK="hand teleop demonstration"

# Parse arguments
USE_VIDEO=true
DRY_RUN=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-video)
            USE_VIDEO=false
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--no-video] [--dry-run] [--help]"
            echo ""
            echo "Options:"
            echo "  --no-video  Skip video encoding (faster, data only)"
            echo "  --dry-run   Show what would be processed without running"
            echo "  --help      Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# ============================================================================
# Prerequisites Check
# ============================================================================

echo "=============================================="
echo "Batch ROS2 Rosbag to GR00T LeRobot Converter"
echo "=============================================="
echo ""
echo "Checking prerequisites..."

# Check ffmpeg if video encoding is enabled (skip for dry-run)
if $USE_VIDEO && ! $DRY_RUN; then
    if ! command -v ffmpeg &> /dev/null; then
        echo ""
        echo "WARNING: ffmpeg not found!"
        echo "Video encoding requires ffmpeg. Install with:"
        echo "  sudo apt update && sudo apt install -y ffmpeg"
        echo ""
        echo "Options:"
        echo "  1. Install ffmpeg and run again"
        echo "  2. Run with --no-video flag to skip video encoding"
        echo ""
        read -p "Continue without video encoding? [y/N] " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            USE_VIDEO=false
            echo "Proceeding without video encoding..."
        else
            exit 1
        fi
    else
        echo "  ffmpeg: OK"
    fi
elif $USE_VIDEO && $DRY_RUN; then
    if command -v ffmpeg &> /dev/null; then
        echo "  ffmpeg: OK"
    else
        echo "  ffmpeg: NOT FOUND (will need to install for video encoding)"
    fi
fi

# Check input directory
if [[ ! -d "$INPUT_DIR" ]]; then
    echo "ERROR: Input directory not found: $INPUT_DIR"
    exit 1
fi
echo "  Input directory: OK"

# ============================================================================
# Find Matching Rosbags
# ============================================================================

echo ""
echo "Scanning for rosbag folders matching: $PATTERN"

MATCHING_BAGS=()
while IFS= read -r -d '' bag; do
    if [[ -f "${bag}/metadata.yaml" ]]; then
        MATCHING_BAGS+=("$bag")
    fi
done < <(find "$INPUT_DIR" -maxdepth 1 -type d -name "$PATTERN" -print0 | sort -z)

NUM_BAGS=${#MATCHING_BAGS[@]}

if [[ $NUM_BAGS -eq 0 ]]; then
    echo "ERROR: No rosbag folders matching '$PATTERN' found"
    exit 1
fi

echo "Found $NUM_BAGS rosbag folders:"
echo ""
for i in "${!MATCHING_BAGS[@]}"; do
    bag_name=$(basename "${MATCHING_BAGS[$i]}")
    if [[ $i -lt 5 ]]; then
        echo "  [$((i+1))] $bag_name"
    elif [[ $i -eq 5 ]]; then
        echo "  ..."
    elif [[ $i -ge $((NUM_BAGS - 2)) ]]; then
        echo "  [$((i+1))] $bag_name"
    fi
done
echo ""

if $DRY_RUN; then
    echo "DRY RUN: Would process $NUM_BAGS rosbags"
    echo "Output would be saved to: $OUTPUT_ROOT"
    exit 0
fi

# ============================================================================
# Create Temporary Directory with Symlinks
# ============================================================================

TEMP_DIR=$(mktemp -d -t rosbag_batch_XXXXXX)
echo "Creating temporary directory: $TEMP_DIR"

cleanup() {
    echo ""
    echo "Cleaning up temporary directory..."
    rm -rf "$TEMP_DIR"
}
trap cleanup EXIT

for bag in "${MATCHING_BAGS[@]}"; do
    bag_name=$(basename "$bag")
    ln -s "$bag" "${TEMP_DIR}/${bag_name}"
done
echo "Created $NUM_BAGS symlinks"

# ============================================================================
# Source ROS2 Environment
# ============================================================================

echo ""
echo "Sourcing ROS2 environment..."

if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
    echo "  Sourced: /opt/ros/humble/setup.bash"
else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble"
    exit 1
fi

ROS2_WS="$HOME/tetheria/aero-open-ros2"
if [[ -f "${ROS2_WS}/install/setup.bash" ]]; then
    source "${ROS2_WS}/install/setup.bash"
    echo "  Sourced: ${ROS2_WS}/install/setup.bash"
fi

# ============================================================================
# Run Conversion
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONVERT_SCRIPT="${SCRIPT_DIR}/convert_rosbag_to_groot_lerobot.py"

if [[ ! -f "$CONVERT_SCRIPT" ]]; then
    echo "ERROR: Conversion script not found: $CONVERT_SCRIPT"
    exit 1
fi

echo ""
echo "=============================================="
echo "Starting conversion..."
echo "=============================================="
echo ""
echo "Configuration:"
echo "  Input: $TEMP_DIR (symlinks to $INPUT_DIR)"
echo "  Output: $OUTPUT_ROOT"
echo "  FPS: $FPS"
echo "  Video: $USE_VIDEO"
echo "  Episodes: $NUM_BAGS"
echo ""

CMD=(
    python3 "$CONVERT_SCRIPT"
    --input-dir "$TEMP_DIR"
    --output-root "$OUTPUT_ROOT"
    --repo-id "$REPO_ID"
    --robot-type "aero_hand"
    --fps "$FPS"
    --task "$TASK"
    --image-h 480
    --image-w 640
    --topic-cmd-vel "/spacemouse/cmd_vel"
    --topic-joint-states "/joint_states"
    --topic-hand-control "/right/joint_control"
    --topic-actuator-states "/right/actuator_states"
    --topic-manus "/manus_glove_0"
    --arm-joint-dim 7
    --hand-joint-dim 16
    --num-actuators 7
    --manus-dim 20
    --overwrite
)

if $USE_VIDEO; then
    CMD+=(
        --use-video
        --camera-head "/camera_0/color"
        --camera-wrist "/camera_4/color"
        --camera-base "/logitech_base/color"
    )
fi

echo "Running: ${CMD[*]}"
echo ""

"${CMD[@]}"

echo ""
echo "=============================================="
echo "Conversion complete!"
echo "=============================================="
echo ""
echo "Output saved to: $OUTPUT_ROOT"
echo ""
echo "Directory structure:"
ls -la "$OUTPUT_ROOT" 2>/dev/null || echo "  (output directory not accessible)"
echo ""
echo "Meta files:"
ls -la "$OUTPUT_ROOT/meta/" 2>/dev/null || echo "  (meta directory not accessible)"
