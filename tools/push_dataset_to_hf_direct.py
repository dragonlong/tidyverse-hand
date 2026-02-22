#!/usr/bin/env python3
"""
Upload a local dataset folder to HuggingFace directly using huggingface_hub.

This bypasses LeRobot's version validation, useful for GR00T v2.0 format datasets.

Usage:
  python push_dataset_to_hf_direct.py \
    --dataset-root /path/to/dataset \
    --repo-id username/dataset_name \
    --private 0
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(
        description="Upload dataset folder to HuggingFace (bypasses LeRobot validation)"
    )
    p.add_argument(
        "--dataset-root",
        type=Path,
        required=True,
        help="Local dataset folder containing meta/, data/, videos/.",
    )
    p.add_argument(
        "--repo-id",
        type=str,
        required=True,
        help="Destination HF dataset repo id like 'username/dataset_name'.",
    )
    p.add_argument(
        "--private",
        type=int,
        default=0,
        help="1 to create a private repo; 0 for public (default).",
    )
    p.add_argument(
        "--commit-message",
        type=str,
        default="Upload GR00T LeRobot v2.0 dataset",
        help="Commit message for the upload.",
    )
    args = p.parse_args()

    dataset_root = args.dataset_root.expanduser().resolve()

    if not dataset_root.exists():
        print(f"ERROR: Dataset root does not exist: {dataset_root}")
        return 1

    # Check for expected structure
    expected_dirs = ["meta", "data"]
    for d in expected_dirs:
        if not (dataset_root / d).exists():
            print(f"WARNING: Expected directory not found: {d}/")

    from huggingface_hub import HfApi, create_repo

    api = HfApi()
    
    # Check logged in user
    try:
        user_info = api.whoami()
        username = user_info["name"]
        print(f"Logged in as: {username}")
    except Exception as e:
        print(f"ERROR: Not logged in to HuggingFace: {e}")
        print("Please run: huggingface-cli login")
        return 1
    
    # Validate repo_id matches logged in user
    repo_owner = args.repo_id.split("/")[0]
    if repo_owner != username:
        print(f"WARNING: Repo owner '{repo_owner}' doesn't match logged in user '{username}'")
        print(f"         Consider using: {username}/{args.repo_id.split('/')[-1]}")
    
    print(f"Dataset root: {dataset_root}")
    print(f"Repo ID: {args.repo_id}")
    print(f"Private: {bool(args.private)}")
    print()

    # Create repo if needed
    print("Creating repository...")
    try:
        create_repo(
            args.repo_id,
            repo_type="dataset",
            private=bool(args.private),
            exist_ok=True,
        )
        print(f"  Repository ready: https://huggingface.co/datasets/{args.repo_id}")
    except Exception as e:
        print(f"ERROR: Failed to create repository: {e}")
        return 1

    # Upload folder using upload_large_folder for large datasets
    print()
    print("Uploading dataset folder (using large folder uploader)...")
    print("This may take a while for large datasets with videos...")
    print()

    try:
        api.upload_large_folder(
            folder_path=str(dataset_root),
            repo_id=args.repo_id,
            repo_type="dataset",
        )
    except Exception as e:
        print(f"Large folder upload failed, trying regular upload: {e}")
        api.upload_folder(
            folder_path=str(dataset_root),
            repo_id=args.repo_id,
            repo_type="dataset",
            commit_message=args.commit_message,
        )

    print()
    print("=" * 50)
    print("Upload complete!")
    print("=" * 50)
    print(f"Dataset URL: https://huggingface.co/datasets/{args.repo_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
