#!/bin/bash
#
# Batch convert rosbag2_2026_01_18* data to GR00T LeRobot v2 format
#
# Data source: /mnt/sandisk/data/tidyverse-hand-2026-01-19/
# Output: /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18/
#
# Before first run: inspect one bag to confirm topic names:
#   ./01_inspect_rosbag.sh /mnt/sandisk/data/tidyverse-hand-2026-01-19/rosbag2_2026_01_18-12_07_40
# Then set TOPIC_* and ARM_JOINT_STATES_TOPIC in the Configuration section
# if your bags differ.
#
# ============================================================================
# Channel Layout (observation.state = 34-dim, action = 26-dim)
# ============================================================================
#
# observation.state [34]
# -----------------------------------------------------------------------
#  Index  | Dim | Name                       | Source Topic / Msg Type
# --------|-----|----------------------------|----------------------------
#   0– 6  |  7  | arm_joint_positions        | /joint_states
#         |     |   (joint1 .. joint7)       |   sensor_msgs/JointState
#   7–13  |  7  | hand_actuator_positions    | /right/actuator_states
#         |     |                            |   aero_hand_open_msgs/ActuatorStates
#  14–33  | 20  | manus_glove_ergonomics     | /manus_glove_0
#         |     |                            |   manus_ros2_msgs/ManusGlove
#
# action [26]
# -----------------------------------------------------------------------
#  Index  | Dim | Name                       | Source Topic / Msg Type
# --------|-----|----------------------------|----------------------------
#   0– 2  |  3  | base_velocity (vx,vy,wz)   | /spacemouse/cmd_vel
#         |     |                            |   geometry_msgs/Twist
#   3– 9  |  7  | arm_joint_targets          | /joint_states
#         |     |   (joint1 .. joint7)       |   sensor_msgs/JointState
#  10–25  | 16  | hand_joint_targets         | /right/joint_control
#         |     |   (16 finger DOFs)         |   aero_hand_open_msgs/JointControl
#
# observation.images (3 cameras, 480×640×3 RGB)
# -----------------------------------------------------------------------
#  Key                          | Source Topic
# ------------------------------|-------------------------------------------
#  observation.images.head      | /camera_0/color
#  observation.images.wrist     | /camera_4/color
#  observation.images.base      | /logitech_base/color
#
# ============================================================================
#
# Arm joint states: by default uses /joint_states with name-based extraction
# (joint1..joint7). If your recording has arm on a separate topic (e.g. gello
# real mode uses /right/gello_js), set ARM_JOINT_STATES_TOPIC below.
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
#   ./02_batch_convert_2026_01_18.sh --max-size 5   # Skip bags larger than 5GB
#   ./02_batch_convert_2026_01_18.sh --upload-hf    # Upload to HuggingFace after conversion
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

# -----------------------------------------------------------------------------
# Topic names (must match what was recorded)
# Run: ./01_inspect_rosbag.sh /path/to/one/rosbag  to list topics in your bags.
# With yam_gello_controller, arm commands are on /joint_states (same as below).
# Expected:
#   - Arm:   JointState on /joint_states (or /right/gello_js in real gello mode)
#   - Hand:  JointControl /right/joint_control, ActuatorStates /right/actuator_states
#   - Base:  Twist /spacemouse/cmd_vel   Glove: ManusGlove /manus_glove_0
# -----------------------------------------------------------------------------
TOPIC_CMD_VEL="/spacemouse/cmd_vel"
TOPIC_JOINT_STATES="/joint_states"
TOPIC_HAND_CONTROL="/right/joint_control"
TOPIC_ACTUATOR_STATES="/right/actuator_states"
TOPIC_MANUS="/manus_glove_0"
# Arm joint states: use this if arm is on a different topic (e.g. /right/gello_js). Else leave empty.
ARM_JOINT_STATES_TOPIC=""
# Arm joint names (gello: joint1..joint6; we use 7 dims, 7th can be 0). Leave empty for default.
ARM_JOINT_NAMES="joint1,joint2,joint3,joint4,joint5,joint6,joint7"

# HuggingFace configuration
HF_REPO_ID="dragonx/tidyverse_hand_2026_01_18"  # Change to your HF username/repo

# Size threshold in GB (bags larger than this will be skipped)
# Set to 0 to disable size checking
# Based on folder scan (2026-01-19):
#   - Most bags: 184M - 2.5G (safe)
#   - Larger bags: 3-5G (9 bags)
#   - Outliers: 5.7G, 6.4G, 14G (skip these)
MAX_SIZE_GB=5

# Log file for skipped bags
SKIPPED_LOG="${OUTPUT_ROOT}/skipped_bags.txt"

# Parse arguments
USE_VIDEO=true
DRY_RUN=false
UPLOAD_HF=false

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
        --max-size)
            MAX_SIZE_GB="$2"
            shift 2
            ;;
        --no-size-limit)
            MAX_SIZE_GB=0
            shift
            ;;
        --upload-hf)
            UPLOAD_HF=true
            shift
            ;;
        --hf-repo)
            HF_REPO_ID="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [--no-video] [--dry-run] [--max-size GB] [--no-size-limit] [--upload-hf] [--hf-repo REPO] [--help]"
            echo ""
            echo "Options:"
            echo "  --no-video      Skip video encoding (faster, data only)"
            echo "  --dry-run       Show what would be processed without running"
            echo "  --max-size GB   Skip bags larger than GB gigabytes (default: $MAX_SIZE_GB)"
            echo "  --no-size-limit Disable size checking (process all bags)"
            echo "  --upload-hf     Upload dataset to HuggingFace after conversion"
            echo "  --hf-repo REPO  HuggingFace repo ID (default: $HF_REPO_ID)"
            echo "  --help          Show this help message"
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

# Function to get folder size in GB
get_folder_size_gb() {
    local folder="$1"
    local size_bytes
    size_bytes=$(du -sb "$folder" 2>/dev/null | cut -f1)
    if [[ -z "$size_bytes" ]]; then
        echo "0"
    else
        # Convert to GB with 2 decimal places
        echo "scale=2; $size_bytes / 1073741824" | bc
    fi
}

# Find all matching bags first
ALL_BAGS=()
while IFS= read -r -d '' bag; do
    if [[ -f "${bag}/metadata.yaml" ]]; then
        ALL_BAGS+=("$bag")
    fi
done < <(find "$INPUT_DIR" -maxdepth 1 -type d -name "$PATTERN" -print0 | sort -z)

NUM_ALL_BAGS=${#ALL_BAGS[@]}

if [[ $NUM_ALL_BAGS -eq 0 ]]; then
    echo "ERROR: No rosbag folders matching '$PATTERN' found"
    exit 1
fi

echo "Found $NUM_ALL_BAGS rosbag folders total"

# Filter bags by size if size limit is enabled
MATCHING_BAGS=()
SKIPPED_BAGS=()

if [[ $MAX_SIZE_GB -gt 0 ]]; then
    echo ""
    echo "Checking folder sizes (max: ${MAX_SIZE_GB}GB)..."
    MAX_SIZE_BYTES=$(echo "$MAX_SIZE_GB * 1073741824" | bc | cut -d. -f1)
    
    for bag in "${ALL_BAGS[@]}"; do
        bag_name=$(basename "$bag")
        size_bytes=$(du -sb "$bag" 2>/dev/null | cut -f1)
        size_gb=$(echo "scale=2; $size_bytes / 1073741824" | bc)
        
        if [[ $size_bytes -gt $MAX_SIZE_BYTES ]]; then
            echo "  SKIP: $bag_name (${size_gb}GB > ${MAX_SIZE_GB}GB)"
            SKIPPED_BAGS+=("$bag|$size_gb")
        else
            echo "  OK:   $bag_name (${size_gb}GB)"
            MATCHING_BAGS+=("$bag")
        fi
    done
else
    echo "Size checking disabled (--no-size-limit or MAX_SIZE_GB=0)"
    MATCHING_BAGS=("${ALL_BAGS[@]}")
fi

NUM_BAGS=${#MATCHING_BAGS[@]}
NUM_SKIPPED=${#SKIPPED_BAGS[@]}

echo ""
echo "Summary:"
echo "  Total found:    $NUM_ALL_BAGS"
echo "  To process:     $NUM_BAGS"
echo "  Skipped (size): $NUM_SKIPPED"

if [[ $NUM_BAGS -eq 0 ]]; then
    echo ""
    echo "ERROR: No rosbag folders to process after size filtering"
    echo "Consider increasing --max-size threshold or using --no-size-limit"
    exit 1
fi

echo ""
echo "Bags to process:"
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

# Log skipped bags
if [[ $NUM_SKIPPED -gt 0 ]]; then
    echo ""
    echo "Skipped bags (too large):"
    for entry in "${SKIPPED_BAGS[@]}"; do
        bag_path="${entry%|*}"
        size_gb="${entry#*|}"
        bag_name=$(basename "$bag_path")
        echo "  - $bag_name (${size_gb}GB)"
    done
fi
echo ""

if $DRY_RUN; then
    echo "DRY RUN: Would process $NUM_BAGS rosbags (skipped $NUM_SKIPPED)"
    echo "Output would be saved to: $OUTPUT_ROOT"
    
    if [[ $NUM_SKIPPED -gt 0 ]]; then
        echo ""
        echo "Skipped bags log would be saved to: $SKIPPED_LOG"
    fi
    exit 0
fi

# Write skipped bags to log file
if [[ $NUM_SKIPPED -gt 0 ]]; then
    mkdir -p "$(dirname "$SKIPPED_LOG")"
    {
        echo "# Skipped rosbags - exceeded size threshold (${MAX_SIZE_GB}GB)"
        echo "# Generated: $(date -Iseconds)"
        echo "# Pattern: $PATTERN"
        echo "# Input directory: $INPUT_DIR"
        echo "#"
        echo "# Format: folder_name | size_gb | full_path"
        echo ""
        for entry in "${SKIPPED_BAGS[@]}"; do
            bag_path="${entry%|*}"
            size_gb="${entry#*|}"
            bag_name=$(basename "$bag_path")
            echo "$bag_name | ${size_gb}GB | $bag_path"
        done
    } > "$SKIPPED_LOG"
    echo "Skipped bags logged to: $SKIPPED_LOG"
fi

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
# Run Conversion (One Bag at a Time for Memory Management)
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONVERT_SCRIPT="${SCRIPT_DIR}/convert_rosbag_to_groot_lerobot.py"

if [[ ! -f "$CONVERT_SCRIPT" ]]; then
    echo "ERROR: Conversion script not found: $CONVERT_SCRIPT"
    exit 1
fi

echo ""
echo "=============================================="
echo "Starting conversion (one bag at a time)..."
echo "=============================================="
echo ""
echo "Configuration:"
echo "  Input: $INPUT_DIR"
echo "  Output: $OUTPUT_ROOT"
echo "  FPS: $FPS"
echo "  Video: $USE_VIDEO"
echo "  Episodes: $NUM_BAGS"
if [[ $MAX_SIZE_GB -gt 0 ]]; then
    echo "  Max size: ${MAX_SIZE_GB}GB (skipped $NUM_SKIPPED bags)"
else
    echo "  Max size: unlimited"
fi
echo ""
echo "Processing bags individually to free memory after each..."
echo ""

# Track progress
PROCESSED=0
FAILED=0
FAILED_BAGS=()

for i in "${!MATCHING_BAGS[@]}"; do
    bag="${MATCHING_BAGS[$i]}"
    bag_name=$(basename "$bag")
    episode_num=$((i + 1))
    
    echo "=============================================="
    echo "[$episode_num/$NUM_BAGS] Processing: $bag_name"
    echo "=============================================="
    
    # Create a temporary directory for this single bag
    TEMP_DIR=$(mktemp -d -t rosbag_single_XXXXXX)
    ln -s "$bag" "${TEMP_DIR}/${bag_name}"
    
    # Build the command
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
        --topic-cmd-vel "$TOPIC_CMD_VEL"
        --topic-joint-states "$TOPIC_JOINT_STATES"
        --topic-hand-control "$TOPIC_HAND_CONTROL"
        --topic-actuator-states "$TOPIC_ACTUATOR_STATES"
        --topic-manus "$TOPIC_MANUS"
        --arm-joint-dim 7       # state[0:7] & action[3:10]  joint1..joint7
        --hand-joint-dim 16    # action[10:26]  finger DOFs from JointControl
        --num-actuators 7      # state[7:14]   actuator feedback
        --manus-dim 20         # state[14:34]  manus glove ergonomics
    )
    
    if [[ -n "$ARM_JOINT_NAMES" ]]; then
        CMD+=(--arm-joint-names "$ARM_JOINT_NAMES")
    fi
    if [[ -n "$ARM_JOINT_STATES_TOPIC" ]]; then
        CMD+=(--topic-arm-joint-states "$ARM_JOINT_STATES_TOPIC")
    fi
    
    # First bag: overwrite; subsequent bags: append (auto-detects indices)
    if [[ $i -eq 0 ]]; then
        CMD+=(--overwrite)
    else
        CMD+=(--append)
    fi
    
    if $USE_VIDEO; then
        CMD+=(
            --use-video
            --camera-head "/camera_0/color"
            --camera-wrist "/camera_4/color"
            --camera-base "/logitech_base/color"
        )
    fi
    
    # Run conversion for this single bag
    echo "Running: ${CMD[*]}"
    echo ""
    
    if "${CMD[@]}"; then
        echo ""
        echo "SUCCESS: $bag_name"
        PROCESSED=$((PROCESSED + 1))
    else
        echo ""
        echo "FAILED: $bag_name"
        FAILED=$((FAILED + 1))
        FAILED_BAGS+=("$bag_name")
    fi
    
    # Clean up temp directory immediately to help free memory
    rm -rf "$TEMP_DIR"
    
    # Force garbage collection hint (Python process already exited, memory freed)
    echo "Memory freed (process exited)"
    echo ""
    
    # Show progress
    echo "Progress: $PROCESSED processed, $FAILED failed, $((NUM_BAGS - episode_num)) remaining"
    echo ""
done

echo ""
echo "=============================================="
echo "Conversion complete!"
echo "=============================================="
echo ""
echo "Summary:"
echo "  Total bags:   $NUM_BAGS"
echo "  Processed:    $PROCESSED"
echo "  Failed:       $FAILED"
echo "  Skipped:      $NUM_SKIPPED"

if [[ $FAILED -gt 0 ]]; then
    echo ""
    echo "Failed bags:"
    for bag in "${FAILED_BAGS[@]}"; do
        echo "  - $bag"
    done
    
    # Log failed bags
    FAILED_LOG="${OUTPUT_ROOT}/failed_bags.txt"
    {
        echo "# Failed rosbags during conversion"
        echo "# Generated: $(date -Iseconds)"
        echo ""
        for bag in "${FAILED_BAGS[@]}"; do
            echo "$bag"
        done
    } > "$FAILED_LOG"
    echo ""
    echo "Failed bags logged to: $FAILED_LOG"
fi

echo ""
echo "Output saved to: $OUTPUT_ROOT"
echo ""
echo "Directory structure:"
ls -la "$OUTPUT_ROOT" 2>/dev/null || echo "  (output directory not accessible)"
echo ""
echo "Meta files:"
ls -la "$OUTPUT_ROOT/meta/" 2>/dev/null || echo "  (meta directory not accessible)"

# ============================================================================
# Upload to HuggingFace (optional)
# ============================================================================

if $UPLOAD_HF; then
    echo ""
    echo "=============================================="
    echo "Uploading to HuggingFace..."
    echo "=============================================="
    echo ""
    echo "Repository: $HF_REPO_ID"
    echo ""
    
    # Check if huggingface_hub is installed
    if ! python3 -c "import huggingface_hub" 2>/dev/null; then
        echo "ERROR: huggingface_hub not installed. Install with:"
        echo "  pip install huggingface_hub"
        exit 1
    fi
    
    # Check if logged in
    if ! python3 -c "from huggingface_hub import HfApi; HfApi().whoami()" 2>/dev/null; then
        echo "ERROR: Not logged in to HuggingFace. Login with:"
        echo "  huggingface-cli login"
        exit 1
    fi
    
    # Upload using Python script
    python3 << EOF
import os
from pathlib import Path
from huggingface_hub import HfApi, create_repo

repo_id = "$HF_REPO_ID"
local_dir = Path("$OUTPUT_ROOT")

print(f"Creating/updating repo: {repo_id}")

# Create repo if it doesn't exist (dataset type)
try:
    create_repo(repo_id, repo_type="dataset", exist_ok=True)
    print(f"  Repo ready: https://huggingface.co/datasets/{repo_id}")
except Exception as e:
    print(f"  Note: {e}")

# Upload the entire dataset directory
api = HfApi()
print(f"\\nUploading dataset from: {local_dir}")
print("This may take a while for large datasets...")

api.upload_folder(
    folder_path=str(local_dir),
    repo_id=repo_id,
    repo_type="dataset",
    commit_message="Upload LeRobot v2 dataset from rosbag conversion",
)

print(f"\\nUpload complete!")
print(f"Dataset URL: https://huggingface.co/datasets/{repo_id}")
EOF

    if [[ $? -eq 0 ]]; then
        echo ""
        echo "=============================================="
        echo "HuggingFace upload complete!"
        echo "=============================================="
        echo ""
        echo "Dataset URL: https://huggingface.co/datasets/$HF_REPO_ID"
    else
        echo ""
        echo "ERROR: HuggingFace upload failed"
    fi
fi
