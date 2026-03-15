#!/usr/bin/env python3
"""
Clean a GR00T LeRobot v2 dataset by dropping incomplete episodes.

Drops any episode where required channels are all-zero (source rosbag lacked
those topics).  By default requires arm + hand + manus to all be non-zero:

  observation.state[0:7]  — arm joint positions   (/joint_states)
  observation.state[7:14] — hand actuator state    (/right/actuator_states)
  observation.state[14:34]— manus glove            (/manus_glove_0)
  action[3:10]            — arm joint targets      (/joint_states)
  action[10:26]           — hand joint targets     (/right/joint_control)

Spacemouse base velocity (action[0:3]) is NOT required — it is legitimately
zero when the base was stationary during a demonstration.

After dropping, episodes are re-numbered 0, 1, 2, ... and meta files are
rewritten.  Videos for each camera are renamed to match.

Usage:
    # Write cleaned copy to a new directory (safe default):
    python3 clean_lerobot_dataset.py \\
        --dataset /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18 \\
        --output  /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18_clean

    # In-place (overwrites source — make a backup first):
    python3 clean_lerobot_dataset.py \\
        --dataset /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18 \\
        --in-place

    # Dry-run (show what would be dropped, write nothing):
    python3 clean_lerobot_dataset.py \\
        --dataset /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18 \\
        --dry-run

    # Relax requirements (drop arm check, keep hand-only episodes):
    python3 clean_lerobot_dataset.py \\
        --dataset ... --no-require-arm --no-require-manus
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

# ── channel slices (must match convert_rosbag_to_groot_lerobot.py) ────────────

STATE_ARM   = slice(0, 7)
STATE_HAND  = slice(7, 14)
STATE_MANUS = slice(14, 34)
ACTION_ARM  = slice(3, 10)
ACTION_HAND = slice(10, 26)

EPISODES_PER_CHUNK = 1000  # standard LeRobot v2 chunk size


# ── helpers ───────────────────────────────────────────────────────────────────

def _chunk(ep_idx: int) -> int:
    return ep_idx // EPISODES_PER_CHUNK


def _parquet_path(root: Path, ep_idx: int) -> Path:
    return root / f"data/chunk-{_chunk(ep_idx):03d}/episode_{ep_idx:06d}.parquet"


def _video_path(root: Path, ep_idx: int, video_key: str) -> Path:
    return root / f"videos/chunk-{_chunk(ep_idx):03d}/{video_key}/episode_{ep_idx:06d}.mp4"


def _discover_video_keys(root: Path, ep_idx: int) -> list[str]:
    """Return all video key names that have a file for this episode."""
    chunk_dir = root / f"videos/chunk-{_chunk(ep_idx):03d}"
    if not chunk_dir.exists():
        return []
    keys = []
    for key_dir in sorted(chunk_dir.iterdir()):
        if key_dir.is_dir() and _video_path(root, ep_idx, key_dir.name).exists():
            keys.append(key_dir.name)
    return keys


def _is_nonzero(arr: np.ndarray) -> bool:
    return not np.all(arr == 0)


def _validate_episode(parquet: Path, require_arm: bool, require_manus: bool) -> tuple[bool, str]:
    """Return (is_valid, reason_if_invalid)."""
    tbl = pq.read_table(parquet, columns=["observation.state", "action"])
    state  = np.array(tbl["observation.state"].to_pylist(), dtype=np.float32)
    action = np.array(tbl["action"].to_pylist(), dtype=np.float32)

    missing = []
    if require_arm:
        if not _is_nonzero(state[:, STATE_ARM]):
            missing.append("arm_state")
        if not _is_nonzero(action[:, ACTION_ARM]):
            missing.append("arm_action")
    if not _is_nonzero(state[:, STATE_HAND]):
        missing.append("hand_state")
    if not _is_nonzero(action[:, ACTION_HAND]):
        missing.append("hand_action")
    if require_manus and not _is_nonzero(state[:, STATE_MANUS]):
        missing.append("manus_state")

    if missing:
        return False, "missing: " + ", ".join(missing)
    return True, ""


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(description="Drop incomplete episodes from a GR00T LeRobot v2 dataset")
    ap.add_argument("--dataset", required=True, help="Source dataset root")
    grp = ap.add_mutually_exclusive_group()
    grp.add_argument("--output", help="Output directory (default: <dataset>_clean)")
    grp.add_argument("--in-place", action="store_true", help="Rewrite source dataset in-place")
    ap.add_argument("--dry-run", action="store_true", help="Show what would be dropped, write nothing")
    ap.add_argument("--yes", "-y", action="store_true", help="Skip confirmation prompt")
    ap.add_argument("--no-require-arm",   dest="require_arm",   action="store_false", default=True)
    ap.add_argument("--no-require-manus", dest="require_manus", action="store_false", default=True)
    args = ap.parse_args()

    src = Path(args.dataset)
    if not src.exists():
        print(f"ERROR: dataset not found: {src}", file=sys.stderr)
        sys.exit(1)

    # Determine output root
    if args.dry_run:
        dst = None
    elif args.in_place:
        dst = src
    elif args.output:
        dst = Path(args.output)
    else:
        dst = src.parent / (src.name + "_clean")

    # Load info.json
    info_file = src / "meta" / "info.json"
    info = json.loads(info_file.read_text())
    total_episodes = info["total_episodes"]

    # Validate each episode
    print(f"Source:  {src}")
    print(f"Output:  {'(dry-run)' if args.dry_run else ('(in-place)' if args.in_place else dst)}")
    print(f"Require: arm={'yes' if args.require_arm else 'no'}  manus={'yes' if args.require_manus else 'no'}")
    print()
    print(f"Scanning {total_episodes} episodes ...")
    print()

    keep: list[int] = []
    drop: list[tuple[int, str]] = []

    for ep_idx in range(total_episodes):
        parquet = _parquet_path(src, ep_idx)
        if not parquet.exists():
            drop.append((ep_idx, "parquet file missing"))
            continue
        valid, reason = _validate_episode(parquet, args.require_arm, args.require_manus)
        if valid:
            keep.append(ep_idx)
        else:
            drop.append((ep_idx, reason))

    # Report
    print(f"  KEEP: {len(keep)} episodes  →  indices {keep}")
    if drop:
        print(f"  DROP: {len(drop)} episodes:")
        for ep_idx, reason in drop:
            print(f"    ep {ep_idx:03d}  ({reason})")
    print()

    if not keep:
        print("ERROR: no episodes would remain after cleaning. Aborting.", file=sys.stderr)
        sys.exit(1)

    if args.dry_run:
        total_keep = sum(
            pq.read_metadata(_parquet_path(src, i)).num_rows for i in keep
        )
        print(f"Dry-run: {len(keep)} episodes / {total_keep:,} frames would be kept")
        return

    # Confirm
    if not args.yes:
        ans = input(f"Drop {len(drop)} episodes and keep {len(keep)}? [y/N] ").strip().lower()
        if ans != "y":
            print("Aborted.")
            sys.exit(0)

    # Discover video keys from first kept episode
    video_keys = _discover_video_keys(src, keep[0])
    if not video_keys:
        print("  Note: no video files found (data-only dataset)")

    # ── write to staging directory ────────────────────────────────────────────
    # Always stage to a temp dir first, then atomically replace the destination.
    use_temp = args.in_place or (dst == src)

    if use_temp:
        staging = Path(tempfile.mkdtemp(prefix="lerobot_clean_", dir=src.parent))
    else:
        staging = dst
        staging.mkdir(parents=True, exist_ok=True)

    print(f"Writing {len(keep)} episodes to staging dir ...")

    new_episodes_meta: list[dict] = []
    total_frames = 0

    for new_idx, old_idx in enumerate(keep):
        # ── parquet ───────────────────────────────────────────────────────────
        old_parquet = _parquet_path(src, old_idx)
        new_parquet = _parquet_path(staging, new_idx)
        new_parquet.parent.mkdir(parents=True, exist_ok=True)

        # Read and update episode_index column if present
        tbl = pq.read_table(old_parquet)
        col_names = tbl.schema.names
        if "episode_index" in col_names:
            import pyarrow as pa
            new_col = pa.array([new_idx] * len(tbl), type=pa.int64())
            tbl = tbl.set_column(col_names.index("episode_index"), "episode_index", new_col)
        pq.write_table(tbl, new_parquet, compression="snappy")

        n_frames = len(tbl)
        total_frames += n_frames

        # Episode metadata (length and task)
        old_ep_meta = None
        ep_jsonl = src / "meta" / "episodes.jsonl"
        if ep_jsonl.exists():
            for line in ep_jsonl.read_text().splitlines():
                m = json.loads(line)
                if m["episode_index"] == old_idx:
                    old_ep_meta = m
                    break
        new_episodes_meta.append({
            "episode_index": new_idx,
            "tasks": old_ep_meta["tasks"] if old_ep_meta else ["hand teleop demonstration"],
            "length": n_frames,
        })

        # ── videos ────────────────────────────────────────────────────────────
        for vk in video_keys:
            old_vid = _video_path(src, old_idx, vk)
            new_vid = _video_path(staging, new_idx, vk)
            new_vid.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(old_vid, new_vid)

        print(f"  ep {old_idx:03d} → {new_idx:03d}  ({n_frames} frames)")

    # ── meta ─────────────────────────────────────────────────────────────────
    meta_dst = staging / "meta"
    meta_dst.mkdir(parents=True, exist_ok=True)

    # Copy modality.json and tasks.jsonl unchanged
    for fname in ("modality.json", "tasks.jsonl"):
        src_file = src / "meta" / fname
        if src_file.exists():
            shutil.copy2(src_file, meta_dst / fname)

    # Rewrite episodes.jsonl
    (meta_dst / "episodes.jsonl").write_text(
        "\n".join(json.dumps(e) for e in new_episodes_meta) + "\n"
    )

    # Rewrite info.json
    new_info = dict(info)
    new_info["total_episodes"] = len(keep)
    new_info["total_frames"]   = total_frames
    new_info["total_videos"]   = len(keep) * len(video_keys)
    chunks_size = new_info.get("chunks_size", EPISODES_PER_CHUNK)
    new_info["chunks_size"]  = chunks_size
    new_info["total_chunks"] = max(1, -(-len(keep) // chunks_size))  # ceiling division
    (meta_dst / "info.json").write_text(json.dumps(new_info, indent=2))

    # ── atomically replace destination ───────────────────────────────────────
    if use_temp:
        # Delete source content and replace with staging
        print()
        print(f"Replacing {src} with cleaned dataset ...")
        # Remove old data/videos/meta from src
        for sub in ("data", "videos", "meta"):
            old = src / sub
            if old.exists():
                shutil.rmtree(old)
        # Move staging subdirs into src
        for sub in staging.iterdir():
            shutil.move(str(sub), src / sub.name)
        staging.rmdir()
        final = src
    else:
        final = dst

    print()
    print(f"Done!  {len(keep)} episodes / {total_frames:,} frames → {final}")
    print()
    print("Run validate to confirm:")
    print(f"  python3 validate_lerobot_dataset.py --dataset {final}")


if __name__ == "__main__":
    main()
