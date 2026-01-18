#!/bin/bash
# Upload rosbag folders to remote machine via rsync
# Shows progress and transfer speed

set -e

# Configuration
REMOTE_USER="dragonx"
REMOTE_HOST="dragonx-tp.local"
REMOTE_DEST="~/tetheria/tidyverse-hand/data"

# Source folders to upload
SOURCE_FOLDERS=(
    "/home/dragonx/tetheria/aero-open-ros2/rosbag2_2026_01_16-17_09_38"
    "/home/dragonx/tetheria/aero-open-ros2/rosbag2_2026_01_16-17_16_47"
)

echo "=== Uploading rosbag folders to ${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DEST} ==="
echo ""

for folder in "${SOURCE_FOLDERS[@]}"; do
    if [ -d "$folder" ]; then
        echo "Uploading: $folder"
        echo "----------------------------------------"
        rsync -avz --progress --info=progress2 \
            "$folder" \
            "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DEST}/"
        echo ""
        echo "Done: $(basename $folder)"
        echo ""
    else
        echo "WARNING: Folder not found: $folder"
    fi
done

echo "=== All uploads complete ==="
