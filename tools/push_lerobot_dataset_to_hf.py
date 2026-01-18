#!/usr/bin/env python3
"""
Create a Hugging Face *dataset* repo (if needed) and upload a local LeRobot dataset folder.

This is a tiny wrapper around `lerobot.datasets.lerobot_dataset.LeRobotDataset.push_to_hub()`.

Usage example:
  python tidyverse/tools/push_lerobot_dataset_to_hf.py \
    --dataset-root /Users/xiaoli/projects/code/tidyverse/data/evan_house_split_1g_lerobot_v3 \
    --repo-id xiaoli/evan_house_split_1g_lerobot_v3 \
    --private 1 \
    --upload-large-folder 1
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--dataset-root",
        type=Path,
        required=True,
        help="Local LeRobot dataset folder containing meta/, data/, videos/.",
    )
    p.add_argument(
        "--repo-id",
        type=str,
        required=True,
        help="Destination HF dataset repo id like 'username/dataset_name'.",
    )
    p.add_argument("--branch", type=str, default=None, help="Optional branch name to push to.")
    p.add_argument(
        "--private",
        type=int,
        default=0,
        help="1 to create/push a private dataset repo; 0 for public.",
    )
    p.add_argument(
        "--push-videos",
        type=int,
        default=1,
        help="1 to upload videos/; 0 to skip videos/ (metadata+parquet only).",
    )
    p.add_argument(
        "--upload-large-folder",
        type=int,
        default=0,
        help="1 to use HF's large folder uploader (recommended for big datasets).",
    )
    args = p.parse_args()

    dataset_root = args.dataset_root.expanduser().resolve()

    # Import lazily so running `--help` doesn't require lerobot deps.
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(
        repo_id=args.repo_id,
        root=dataset_root,
        episodes=None,
        download_videos=False,
    )

    ds.push_to_hub(
        branch=args.branch,
        private=bool(args.private),
        push_videos=bool(args.push_videos),
        upload_large_folder=bool(args.upload_large_folder),
    )

    print(f"Uploaded dataset to: https://huggingface.co/datasets/{args.repo_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


