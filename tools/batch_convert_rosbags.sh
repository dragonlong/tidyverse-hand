#!/bin/bash
#
# Batch convert multiple ROS2 rosbag folders to GR00T LeRobot v2 format.
#
# This script:
#   1. Filters rosbag folders by pattern (default: rosbag2_2026_01_18*)
#   2. Sorts them by timestamp (folder name ordering)
#   3. Creates symlinks in a temp directory
#   4. Runs the conversion script to process all as sequential episodes
#   5. Cleans up
#
# Usage:
#   ./batch_convert_rosbags.sh [OPTIONS]
#
# Example:
#   ./batch_convert_rosbags.sh \
#       --input-dir /mnt/sandisk/data/tidyverse-hand-2026-01-19 \
#       --output-root /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18 \
#       --pattern "rosbag2_2026_01_18*" \
#       --fps 30 \
#       --use-video
#

set -e  # Exit on error

# ============================================================================
# Default Configuration
# ============================================================================

INPUT_DIR="/mnt/sandisk/data/tidyverse-hand-2026-01-19"
OUTPUT_ROOT="/mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18"
PATTERN="rosbag2_2026_01_18*"
REPO_ID="tidyverse_hand_2026_01_18"
ROBOT_TYPE="aero_hand"
FPS=30
TASK="hand teleop demonstration"
USE_VIDEO=""
OVERWRITE=""
DEBUG_TOPICS=""

# Camera topics (verified from rosbag inspection)
CAMERA_HEAD="/camera_0/color"
CAMERA_WRIST="/camera_4/color"
CAMERA_BASE="/logitech_base/color"

# Image dimensions
IMAGE_H=480
IMAGE_W=640

# Topic configuration
TOPIC_CMD_VEL="/spacemouse/cmd_vel"
TOPIC_JOINT_STATES="/joint_states"
TOPIC_HAND_CONTROL="/right/joint_control"
TOPIC_ACTUATOR_STATES="/right/actuator_states"
TOPIC_MANUS="/manus_glove_0"

# Dimension configuration
ARM_JOINT_DIM=7
HAND_JOINT_DIM=16
NUM_ACTUATORS=7
MANUS_DIM=20

# Script directory (for finding convert_rosbag_to_groot_lerobot.py)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERT_SCRIPT="${SCRIPT_DIR}/convert_rosbag_to_groot_lerobot.py"

# ============================================================================
# Parse Arguments
# ============================================================================

while [[ $# -gt 0 ]]; do
    case $1 in
        --input-dir)
            INPUT_DIR="$2"
            shift 2
            ;;
        --output-root)
            OUTPUT_ROOT="$2"
            shift 2
            ;;
        --pattern)
            PATTERN="$2"
            shift 2
            ;;
        --repo-id)
            REPO_ID="$2"
            shift 2
            ;;
        --robot-type)
            ROBOT_TYPE="$2"
            shift 2
            ;;
        --fps)
            FPS="$2"
            shift 2
            ;;
        --task)
            TASK="$2"
            shift 2
            ;;
        --use-video)
            USE_VIDEO="--use-video"
            shift
            ;;
        --overwrite)
            OVERWRITE="--overwrite"
            shift
            ;;
        --debug-topics)
            DEBUG_TOPICS="--debug-topics"
            shift
            ;;
        --camera-head)
            CAMERA_HEAD="$2"
            shift 2
            ;;
        --camera-wrist)
            CAMERA_WRIST="$2"
            shift 2
            ;;
        --camera-base)
            CAMERA_BASE="$2"
            shift 2
            ;;
        --image-h)
            IMAGE_H="$2"
            shift 2
            ;;
        --image-w)
            IMAGE_W="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --input-dir DIR       Source directory containing rosbag folders"
            echo "  --output-root DIR     Output directory for LeRobot v2 dataset"
            echo "  --pattern PATTERN     Glob pattern to filter rosbag folders (default: rosbag2_2026_01_18*)"
            echo "  --repo-id ID          Repository ID for the dataset"
            echo "  --robot-type TYPE     Robot type (default: aero_hand)"
            echo "  --fps FPS             Frames per second (default: 30)"
            echo "  --task TASK           Task description"
            echo "  --use-video           Enable video encoding"
            echo "  --overwrite           Overwrite existing output"
            echo "  --debug-topics        Print available topics"
            echo "  --camera-head TOPIC   Camera head topic"
            echo "  --camera-wrist TOPIC  Camera wrist topic"
            echo "  --camera-base TOPIC   Camera base topic"
            echo "  --image-h HEIGHT      Image height (default: 480)"
            echo "  --image-w WIDTH       Image width (default: 640)"
            echo "  --help, -h            Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# ============================================================================
# Validate Inputs
# ============================================================================

if [[ ! -d "$INPUT_DIR" ]]; then
    echo "Error: Input directory does not exist: $INPUT_DIR"
    exit 1
fi

if [[ ! -f "$CONVERT_SCRIPT" ]]; then
    echo "Error: Conversion script not found: $CONVERT_SCRIPT"
    exit 1
fi

# ============================================================================
# Find and Filter Rosbag Folders
# ============================================================================

echo "=============================================="
echo "Batch ROS2 Rosbag to GR00T LeRobot Converter"
echo "=============================================="
echo ""
echo "Input directory: $INPUT_DIR"
echo "Output directory: $OUTPUT_ROOT"
echo "Pattern filter: $PATTERN"
echo "FPS: $FPS"
echo ""

# Find matching rosbag folders (sorted by name = timestamp order)
MATCHING_BAGS=()
while IFS= read -r -d '' bag; do
    # Check if it's a valid rosbag (has metadata.yaml)
    if [[ -f "${bag}/metadata.yaml" ]]; then
        MATCHING_BAGS+=("$bag")
    fi
done < <(find "$INPUT_DIR" -maxdepth 1 -type d -name "$PATTERN" -print0 | sort -z)

NUM_BAGS=${#MATCHING_BAGS[@]}

if [[ $NUM_BAGS -eq 0 ]]; then
    echo "Error: No rosbag folders matching pattern '$PATTERN' found in $INPUT_DIR"
    exit 1
fi

echo "Found $NUM_BAGS rosbag folders matching pattern:"
for i in "${!MATCHING_BAGS[@]}"; do
    bag_name=$(basename "${MATCHING_BAGS[$i]}")
    if [[ $i -lt 3 ]] || [[ $i -ge $((NUM_BAGS - 2)) ]]; then
        echo "  [$((i+1))/$NUM_BAGS] $bag_name"
    elif [[ $i -eq 3 ]]; then
        echo "  ..."
    fi
done
echo ""

# ============================================================================
# Create Temporary Directory with Symlinks
# ============================================================================

TEMP_DIR=$(mktemp -d -t rosbag_batch_XXXXXX)
echo "Creating temporary directory with symlinks: $TEMP_DIR"

# Create symlinks in sorted order
for bag in "${MATCHING_BAGS[@]}"; do
    bag_name=$(basename "$bag")
    ln -s "$bag" "${TEMP_DIR}/${bag_name}"
done

echo "Created $NUM_BAGS symlinks"
echo ""

# ============================================================================
# Cleanup Function
# ============================================================================

cleanup() {
    echo ""
    echo "Cleaning up temporary directory..."
    rm -rf "$TEMP_DIR"
    echo "Done."
}

# Set trap to cleanup on exit (normal or error)
trap cleanup EXIT

# ============================================================================
# Source ROS2 Environment
# ============================================================================

echo "Sourcing ROS2 environment..."

# Source ROS2 Humble
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    echo "Warning: /opt/ros/humble/setup.bash not found"
fi

# Source workspace setup if available
ROS2_WS="$HOME/tetheria/aero-open-ros2"
if [[ -f "${ROS2_WS}/install/setup.bash" ]]; then
    source "${ROS2_WS}/install/setup.bash"
    echo "Sourced workspace: ${ROS2_WS}"
fi

echo ""

# ============================================================================
# Build Command
# ============================================================================

CMD=(
    python3 "$CONVERT_SCRIPT"
    --input-dir "$TEMP_DIR"
    --output-root "$OUTPUT_ROOT"
    --repo-id "$REPO_ID"
    --robot-type "$ROBOT_TYPE"
    --fps "$FPS"
    --task "$TASK"
    --image-h "$IMAGE_H"
    --image-w "$IMAGE_W"
    --topic-cmd-vel "$TOPIC_CMD_VEL"
    --topic-joint-states "$TOPIC_JOINT_STATES"
    --topic-hand-control "$TOPIC_HAND_CONTROL"
    --topic-actuator-states "$TOPIC_ACTUATOR_STATES"
    --topic-manus "$TOPIC_MANUS"
    --arm-joint-dim "$ARM_JOINT_DIM"
    --hand-joint-dim "$HAND_JOINT_DIM"
    --num-actuators "$NUM_ACTUATORS"
    --manus-dim "$MANUS_DIM"
)

# Add optional flags
if [[ -n "$USE_VIDEO" ]]; then
    CMD+=("$USE_VIDEO")
    if [[ -n "$CAMERA_HEAD" ]]; then
        CMD+=(--camera-head "$CAMERA_HEAD")
    fi
    if [[ -n "$CAMERA_WRIST" ]]; then
        CMD+=(--camera-wrist "$CAMERA_WRIST")
    fi
    if [[ -n "$CAMERA_BASE" ]]; then
        CMD+=(--camera-base "$CAMERA_BASE")
    fi
fi

if [[ -n "$OVERWRITE" ]]; then
    CMD+=("$OVERWRITE")
fi

if [[ -n "$DEBUG_TOPICS" ]]; then
    CMD+=("$DEBUG_TOPICS")
fi

# ============================================================================
# Run Conversion
# ============================================================================

echo "=============================================="
echo "Running conversion..."
echo "=============================================="
echo ""
echo "Command:"
echo "  ${CMD[*]}"
echo ""

"${CMD[@]}"

echo ""
echo "=============================================="
echo "Batch conversion complete!"
echo "=============================================="
echo ""
echo "Output saved to: $OUTPUT_ROOT"
echo "Total rosbags processed: $NUM_BAGS"
