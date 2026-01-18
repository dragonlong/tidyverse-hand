#!/usr/bin/env python3
"""
Flip (or rotate) all videos in a LeRobot-format dataset folder.

This is useful when you converted data into LeRobot format and later realize the camera
orientation should be corrected.

Expected dataset structure (subset):
  <dataset_root>/
    videos/
      <camera_key>/
        chunk-000/
          file-000.mp4
          ...

By default this script rotates videos 180 degrees (what most people mean by "upside down").
It does *not* modify any metadata files, because paths stay the same.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path


def _ffmpeg_exists() -> bool:
    return shutil.which("ffmpeg") is not None


def _build_filter(mode: str) -> str:
    # "upside down" is typically a 180° rotation, not a mirror flip.
    if mode == "rotate180":
        return "hflip,vflip"
    if mode == "vflip":
        return "vflip"
    if mode == "hflip":
        return "hflip"
    raise ValueError(f"Unsupported mode: {mode}")


def _process_one(
    mp4_path_str: str,
    mode: str,
    overwrite: bool,
    backup_ext: str | None,
    crf: int,
    preset: str,
) -> tuple[str, str]:
    mp4_path = Path(mp4_path_str)
    if not mp4_path.exists():
        return (mp4_path_str, "missing")

    tmp_out = mp4_path.with_suffix(".tmp.mp4")
    bak = mp4_path.with_suffix(mp4_path.suffix + backup_ext) if backup_ext else None

    if tmp_out.exists():
        tmp_out.unlink()

    # If not overwriting and backup already exists, assume we already processed this file.
    if not overwrite and bak is not None and bak.exists():
        return (mp4_path_str, "skipped_existing_backup")

    vf = _build_filter(mode)

    # Re-encode video; keep audio stream if present.
    # - Map all streams, but only re-encode video.
    # - "faststart" helps for playback.
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",  # we control overwrite at the file-move level
        "-i",
        str(mp4_path),
        "-map",
        "0",
        "-vf",
        vf,
        "-c:v",
        "libx264",
        "-crf",
        str(crf),
        "-preset",
        preset,
        "-movflags",
        "+faststart",
        "-c:a",
        "copy",
        str(tmp_out),
    ]

    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    except subprocess.CalledProcessError as e:
        # Best effort cleanup
        if tmp_out.exists():
            tmp_out.unlink()
        return (mp4_path_str, f"ffmpeg_failed: {e.stderr.strip()}")

    # Replace original atomically-ish: move original to backup (optional), then move tmp into place.
    try:
        if bak is not None:
            if bak.exists():
                if overwrite:
                    bak.unlink()
                else:
                    # don't clobber backup; caller asked not to overwrite
                    tmp_out.unlink()
                    return (mp4_path_str, "skipped_backup_exists")
            mp4_path.replace(bak)
        else:
            # No backup requested: remove original before replacing
            mp4_path.unlink()

        tmp_out.replace(mp4_path)
    except Exception as e:
        # Try to roll back
        if mp4_path.exists() and bak is not None and not bak.exists():
            pass
        if tmp_out.exists():
            tmp_out.unlink()
        return (mp4_path_str, f"replace_failed: {e}")

    return (mp4_path_str, "ok")


def _iter_mp4s(dataset_root: Path) -> list[Path]:
    videos_root = dataset_root / "videos"
    if not videos_root.exists():
        raise FileNotFoundError(f"No 'videos/' folder found under: {dataset_root}")
    return sorted(videos_root.rglob("*.mp4"))


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--dataset-root",
        type=Path,
        required=True,
        help="Path to the LeRobot dataset folder (the one containing meta/, data/, videos/).",
    )
    p.add_argument(
        "--mode",
        choices=["rotate180", "vflip", "hflip"],
        default="rotate180",
        help="How to transform frames. 'rotate180' is the usual fix for upside-down cameras.",
    )
    p.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 4) // 2))
    p.add_argument("--crf", type=int, default=18, help="x264 CRF (lower is higher quality).")
    p.add_argument("--preset", type=str, default="fast", help="x264 preset (e.g. ultrafast..veryslow).")
    p.add_argument(
        "--no-backup",
        action="store_true",
        help="Do not keep backups. By default we keep '<file>.mp4.orig'.",
    )
    p.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing backups and re-process even if a backup exists.",
    )
    p.add_argument("--dry-run", action="store_true", help="Print what would be changed.")
    args = p.parse_args()

    if not _ffmpeg_exists():
        raise SystemExit("ffmpeg not found on PATH. Install it (e.g. `conda install -c conda-forge ffmpeg`).")

    dataset_root = args.dataset_root.expanduser().resolve()
    mp4s = _iter_mp4s(dataset_root)
    if not mp4s:
        print(f"No .mp4 files found under {dataset_root/'videos'}")
        return 0

    backup_ext = None if args.no_backup else ".orig"

    if args.dry_run:
        print(f"[dry-run] Would process {len(mp4s)} files under: {dataset_root/'videos'}")
        print(f"[dry-run] mode={args.mode} crf={args.crf} preset={args.preset} backup_ext={backup_ext}")
        for pth in mp4s[:10]:
            print(f"[dry-run] {pth}")
        if len(mp4s) > 10:
            print(f"[dry-run] ... +{len(mp4s) - 10} more")
        return 0

    workers = max(1, int(args.workers))
    print(f"Processing {len(mp4s)} videos with {workers} worker(s). mode={args.mode} backup_ext={backup_ext}")

    ok = 0
    skipped = 0
    failed = 0

    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = [
            ex.submit(
                _process_one,
                str(pth),
                args.mode,
                args.overwrite,
                backup_ext,
                args.crf,
                args.preset,
            )
            for pth in mp4s
        ]
        for fut in as_completed(futs):
            path, status = fut.result()
            if status == "ok":
                ok += 1
            elif status.startswith("skipped"):
                skipped += 1
            else:
                failed += 1
                print(f"[ERROR] {path}: {status}", file=sys.stderr)

    print(f"Done. ok={ok} skipped={skipped} failed={failed}")
    if failed:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


