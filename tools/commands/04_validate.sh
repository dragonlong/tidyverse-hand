#!/bin/bash
#
# Step 4: Validate a converted GR00T LeRobot v2 dataset.
#
# Checks every episode's parquet for zero-valued channels and reports which
# episodes have complete data (arm + hand + manus all non-zero).
#
# Usage:
#   ./04_validate.sh [dataset_path]
#
# Examples:
#   ./04_validate.sh
#   ./04_validate.sh /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18
#   ./04_validate.sh /path/to/dataset --verbose

set -e

DATASET="${1:-/mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18}"
SHIFT_DONE=false
if [[ -n "$1" ]] && [[ "$1" != --* ]]; then
    shift
    SHIFT_DONE=true
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VALIDATE_SCRIPT="${SCRIPT_DIR}/validate_lerobot_dataset.py"

if [[ ! -f "$VALIDATE_SCRIPT" ]]; then
    echo "ERROR: Validator not found: $VALIDATE_SCRIPT"
    exit 1
fi

echo "=============================================="
echo "Dataset validation"
echo "=============================================="
echo "  Dataset: $DATASET"
echo ""

python3 "$VALIDATE_SCRIPT" --dataset "$DATASET" "$@"

echo ""
echo "If episodes are missing arm/manus data, run ./05_clean_dataset.sh to drop them."
