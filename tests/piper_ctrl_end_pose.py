#!/usr/bin/env python3
# -*-coding:utf8-*-

"""
Record and replay Piper end-effector poses using the SDK demo interface.

This script mirrors the vendor example but adds the ability to:
    * record live end-poses (converted to cm / degrees) into a txt file
    * replay a saved sequence for an arbitrary number of steps

Units:
    - Position feedback from the SDK is in 0.001 mm. We convert to cm by dividing by 10_000.
    - Orientation feedback is in 0.001 degrees. We convert to degrees by dividing by 1_000.
    - When replaying, we convert back to SDK units before calling EndPoseCtrl.
python tidybot2/tests/piper_ctrl_end_pose.py --play 10 --sequence-file tidybot2/tests/end_pose_sequence.txt
"""

import argparse
import time
from pathlib import Path
from typing import List, Sequence, Tuple

from piper_sdk import C_PiperInterface_V2, LogLevel

DEFAULT_SEQUENCE_FILE = Path(__file__).with_name("end_pose_sequence.txt")
DEFAULT_PERIOD_SEC = 1.0
CM_TO_PICO_MM = 10_000.0  # 1 cm = 10,000 units of 0.001 mm


def connect_piper() -> C_PiperInterface_V2:
    """Connect to the Piper arm and enable CAN control."""
    piper = C_PiperInterface_V2("can_piper", logger_level=LogLevel.INFO)
    piper.ConnectPort()
    while not piper.EnablePiper():
        print("...Waiting for Piper arm to enable...")
        time.sleep(0.05)
    piper.GripperCtrl(0, 1000, 0x01, 0x00)
    return piper


def pose_to_cm_deg(pose: C_PiperInterface_V2.ArmEndPose) -> Tuple[float, float, float, float, float, float]:
    end_pose = pose.end_pose
    x_cm = end_pose.X_axis / CM_TO_PICO_MM
    y_cm = end_pose.Y_axis / CM_TO_PICO_MM
    z_cm = end_pose.Z_axis / CM_TO_PICO_MM
    rx_deg = end_pose.RX_axis / 1000.0
    ry_deg = end_pose.RY_axis / 1000.0
    rz_deg = end_pose.RZ_axis / 1000.0
    return x_cm, y_cm, z_cm, rx_deg, ry_deg, rz_deg


def cm_deg_to_raw(
    cm_vals: Sequence[float], deg_vals: Sequence[float]
) -> Tuple[int, int, int, int, int, int]:
    X = int(round(cm_vals[0] * CM_TO_PICO_MM))
    Y = int(round(cm_vals[1] * CM_TO_PICO_MM))
    Z = int(round(cm_vals[2] * CM_TO_PICO_MM))
    RX = int(round(deg_vals[0] * 1000.0))
    RY = int(round(deg_vals[1] * 1000.0))
    RZ = int(round(deg_vals[2] * 1000.0))
    return X, Y, Z, RX, RY, RZ


def record_sequence(piper: C_PiperInterface_V2, steps: int, outfile: Path, period: float) -> None:
    print(f"Recording {steps} samples to {outfile} (period={period:.3f}s)")
    with outfile.open("w", encoding="utf-8") as f:
        for idx in range(steps):
            pose = piper.GetArmEndPoseMsgs()
            sample = pose_to_cm_deg(pose)
            f.write(",".join(f"{val:.6f}" for val in sample) + "\n")
            print(f"[{idx + 1}/{steps}] pose_cm_deg={sample}")
            time.sleep(period)
    print(f"Saved sequence to {outfile}")


def load_sequence(path: Path) -> List[Tuple[float, float, float, float, float, float]]:
    if not path.exists():
        raise FileNotFoundError(f"Sequence file not found: {path}")
    sequence: List[Tuple[float, float, float, float, float, float]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [float(val) for val in line.split(",")]
            if len(parts) != 6:
                raise ValueError(f"Each line must contain 6 comma-separated values, got: {line}")
            sequence.append(tuple(parts))  # type: ignore[arg-type]
    if not sequence:
        raise ValueError(f"Sequence file is empty: {path}")
    return sequence


def replay_sequence(
    piper: C_PiperInterface_V2,
    sequence: Sequence[Tuple[float, float, float, float, float, float]],
    steps: int,
    period: float,
) -> None:
    total_steps = min(steps, len(sequence))
    print(f"Replaying {total_steps}/{len(sequence)} steps (period={period:.3f}s)")
    piper.MotionCtrl_2(0x01, 0x00, 100, 0x00)
    for idx in range(total_steps):
        sample = sequence[idx]
        cm_vals = sample[:3]
        deg_vals = sample[3:]
        X, Y, Z, RX, RY, RZ = cm_deg_to_raw(cm_vals, deg_vals)
        print(f"[{idx + 1}/{total_steps}] EndPoseCtrl({X}, {Y}, {Z}, {RX}, {RY}, {RZ})")
        piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
        time.sleep(period)
    print("Replay complete.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record or replay Piper end-effector pose sequences.")
    parser.add_argument("--sequence-file", type=Path, default=DEFAULT_SEQUENCE_FILE, help="Path to sequence txt file")
    parser.add_argument("--period", type=float, default=DEFAULT_PERIOD_SEC, help="Sample/replay period in seconds")
    parser.add_argument("--record", type=int, help="Record N samples from live feedback")
    parser.add_argument("--play", type=int, default=1, help="Replay N samples from the sequence (default: entire file)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if (args.record is None) == (args.play is None):
        raise SystemExit("Specify exactly one of --record or --play")

    piper = connect_piper()
    try:
        if args.record is not None:
            record_sequence(piper, args.record, args.sequence_file, args.period)
        else:
            sequence = load_sequence(args.sequence_file)
            steps = args.play if args.play and args.play > 0 else len(sequence)
            replay_sequence(piper, sequence, steps, args.period)
    finally:
        try:
            piper.DisconnectPort()
        except Exception:
            pass


if __name__ == "__main__":
    main()