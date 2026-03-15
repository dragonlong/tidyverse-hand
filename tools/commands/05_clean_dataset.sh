#!/bin/bash
#
# Step 5: Drop incomplete episodes from a converted GR00T LeRobot v2 dataset.
#
# Removes episodes where arm joint states/actions or manus glove are all-zero
# (i.e., those topics were not recorded in the source rosbag). Re-numbers the
# remaining episodes consecutively and rewrites meta/info.json and
# meta/episodes.jsonl. Videos for dropped episodes are deleted.
#
# Spacemouse base velocity (action[0:3]) is NOT required — it is legitimately
# zero when the base was stationary during a demonstration.
#
# Usage:
#   ./05_clean_dataset.sh [source_dataset] [output_dataset]
#
# Examples:
#   # Write cleaned copy alongside source (safe default):
#   ./05_clean_dataset.sh \
#     /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18 \
#     /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18_clean
#
#   # In-place (overwrites source — always validate first):
#   ./05_clean_dataset.sh /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18 --in-place
#
#   # Dry-run only (no writes):
#   ./05_clean_dataset.sh /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18 --dry-run

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLEAN_SCRIPT="${SCRIPT_DIR}/clean_lerobot_dataset.py"

if [[ ! -f "$CLEAN_SCRIPT" ]]; then
    echo "ERROR: Clean script not found: $CLEAN_SCRIPT"
    exit 1
fi

DATASET="${1:-/mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18}"

echo "=============================================="
echo "Dataset cleaning (drop incomplete episodes)"
echo "=============================================="
echo "  Source: $DATASET"
echo ""

# Pass remaining args directly to Python (--output, --in-place, --dry-run, --yes, etc.)
if [[ -n "$2" ]] && [[ "$2" != --* ]]; then
    # Second positional arg is output directory
    OUTPUT_DIR="$2"
    shift 2
    python3 "$CLEAN_SCRIPT" --dataset "$DATASET" --output "$OUTPUT_DIR" "$@"
else
    shift 1 2>/dev/null || true
    python3 "$CLEAN_SCRIPT" --dataset "$DATASET" "$@"
fi

echo ""
echo "Next: run ./04_validate.sh on the cleaned dataset to confirm."
