#!/usr/bin/env python3
"""
Validate a converted GR00T LeRobot v2 dataset.

Checks each episode's parquet file for:
  - Arm joint positions (observation.state[0:7])   from /joint_states
  - Arm joint targets  (action[3:10])               from /joint_states
  - Hand actuator pos  (observation.state[7:14])    from /right/actuator_states
  - Manus glove        (observation.state[14:34])   from /manus_glove_0
  - Hand joint targets (action[10:26])              from /right/joint_control
  - Spacemouse cmd_vel (action[0:3])                from /spacemouse/cmd_vel

Reports per-episode validity and a summary with valid episode indices.

Usage:
    python3 validate_lerobot_dataset.py \\
        --dataset /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18

    python3 validate_lerobot_dataset.py \\
        --dataset /mnt/sandisk/data/lerobot_v2/tidyverse_hand_2026_01_18 \\
        --verbose
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq


# ── channel slices (must match convert_rosbag_to_groot_lerobot.py layout) ────

STATE_ARM    = slice(0, 7)      # arm joint positions
STATE_HAND   = slice(7, 14)     # hand actuator positions
STATE_MANUS  = slice(14, 34)    # manus glove ergonomics

ACTION_BASE  = slice(0, 3)      # spacemouse cmd_vel
ACTION_ARM   = slice(3, 10)     # arm joint targets
ACTION_HAND  = slice(10, 26)    # hand joint targets (finger DOFs)


def _nonzero(arr: np.ndarray) -> bool:
    return not np.all(arr == 0)


def _std(arr: np.ndarray) -> float:
    return float(arr.std(axis=0).mean())


def validate_episode(parquet_path: Path) -> dict:
    tbl = pq.read_table(parquet_path, columns=["observation.state", "action"])
    state  = np.array(tbl["observation.state"].to_pylist(), dtype=np.float32)
    action = np.array(tbl["action"].to_pylist(), dtype=np.float32)

    s_arm   = state[:, STATE_ARM]
    s_hand  = state[:, STATE_HAND]
    s_manus = state[:, STATE_MANUS]
    a_base  = action[:, ACTION_BASE]
    a_arm   = action[:, ACTION_ARM]
    a_hand  = action[:, ACTION_HAND]

    channels = {
        "arm_state":    (_nonzero(s_arm),   _std(s_arm)),
        "hand_state":   (_nonzero(s_hand),  _std(s_hand)),
        "manus_state":  (_nonzero(s_manus), _std(s_manus)),
        "base_action":  (_nonzero(a_base),  _std(a_base)),
        "arm_action":   (_nonzero(a_arm),   _std(a_arm)),
        "hand_action":  (_nonzero(a_hand),  _std(a_hand)),
    }

    # "complete" = arm + hand + manus all present (base_action/spacemouse may be
    # legitimately zero when the base was not driven during the demonstration)
    has_arm   = channels["arm_state"][0] and channels["arm_action"][0]
    has_hand  = channels["hand_state"][0] and channels["hand_action"][0]
    has_manus = channels["manus_state"][0]
    complete  = has_arm and has_hand and has_manus

    return {
        "length": len(state),
        "channels": channels,
        "complete": complete,
        "has_arm": has_arm,
        "has_hand": has_hand,
        "has_manus": has_manus,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Validate GR00T LeRobot v2 dataset")
    ap.add_argument("--dataset", required=True, help="Path to dataset root directory")
    ap.add_argument("--verbose", "-v", action="store_true", help="Show per-channel std values")
    args = ap.parse_args()

    root = Path(args.dataset)
    if not root.exists():
        print(f"ERROR: dataset directory not found: {root}", file=sys.stderr)
        sys.exit(1)

    # Load episodes metadata
    episodes_file = root / "meta" / "episodes.jsonl"
    episodes_meta = {}
    if episodes_file.exists():
        for line in episodes_file.read_text().splitlines():
            e = json.loads(line)
            episodes_meta[e["episode_index"]] = e

    # Find parquet files across all chunks
    data_dir = root / "data"
    parquet_files = sorted(data_dir.rglob("episode_*.parquet"))

    if not parquet_files:
        print(f"ERROR: No parquet files found under {data_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Dataset: {root}")
    print(f"Episodes found: {len(parquet_files)}")
    print()

    # Header
    hdr = f"{'Ep':>4}  {'Len':>5}  {'Arm':>4}  {'Hand':>5}  {'Manus':>6}  {'Base':>5}  {'ArmAct':>7}  {'HndAct':>7}  Status"
    print(hdr)
    print("-" * len(hdr))

    results = {}
    for pf in parquet_files:
        ep_idx = int(pf.stem.split("_")[-1])
        r = validate_episode(pf)
        results[ep_idx] = r

        def flag(ch: str) -> str:
            ok, std = r["channels"][ch]
            if args.verbose:
                return f"{std:6.3f}" if ok else "  ZERO"
            return "  OK " if ok else "ZERO"

        status = "COMPLETE" if r["complete"] else ("ARM_MISS" if not r["has_arm"] else ("HAND_MISS" if not r["has_hand"] else ("NO_MANUS" if not r["has_manus"] else "PARTIAL")))
        width = 6 if args.verbose else 4
        print(
            f"{ep_idx:>4}  {r['length']:>5}  "
            f"{flag('arm_state'):>{width}}  {flag('hand_state'):>{width+1}}  "
            f"{flag('manus_state'):>{width+2}}  {flag('base_action'):>{width+1}}  "
            f"{flag('arm_action'):>{width+3}}  {flag('hand_action'):>{width+3}}  "
            f"{status}"
        )

    print()

    # Summary
    complete_eps  = [i for i, r in results.items() if r["complete"]]
    arm_miss_eps  = [i for i, r in results.items() if not r["has_arm"]]
    hand_miss_eps = [i for i, r in results.items() if not r["has_hand"]]
    no_manus_eps  = [i for i, r in results.items() if r["has_arm"] and r["has_hand"] and not r["has_manus"]]
    base_zero_eps = [i for i, r in results.items() if r["complete"] and not r["channels"]["base_action"][0]]

    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"  Total episodes:                    {len(results)}")
    print(f"  COMPLETE (arm+hand+manus):         {len(complete_eps)}")
    if base_zero_eps:
        print(f"    of which base (spacemouse) zero: {len(base_zero_eps)}  (base stationary, OK)")
    if arm_miss_eps:
        print(f"  Missing arm (/joint_states):       {len(arm_miss_eps)}")
    if hand_miss_eps:
        print(f"  Missing hand data:                 {len(hand_miss_eps)}")
    if no_manus_eps:
        print(f"  Has arm+hand but no manus:         {len(no_manus_eps)}")
    print()

    if complete_eps:
        print(f"  Usable episode indices: {complete_eps}")
    if arm_miss_eps:
        print(f"  Missing arm:            {arm_miss_eps}")

    total_frames_complete = sum(results[i]["length"] for i in complete_eps)
    total_frames_all      = sum(r["length"] for r in results.values())
    avg_len = total_frames_complete / len(complete_eps) if complete_eps else 0
    print()
    print(f"  Frames in complete episodes: {total_frames_complete:,} / {total_frames_all:,} total")
    print(f"  Avg episode length (complete): {avg_len:.0f} frames  ({avg_len/30:.1f}s @ 30fps)")


if __name__ == "__main__":
    main()
