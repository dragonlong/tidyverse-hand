#!/usr/bin/env python3
"""
Plot arm joint ACTION (desired from gello_publisher) vs STATE (actual arm angles).

- ACTION = JointState from the topic gello_publisher publishes to (desired/target
  from human teleop). In gello_publisher: sim_mode -> /joint_states,
  real_mode -> /right/gello_js (see aero-open-ros2/src/gello_controller/.../gello_publisher.py).
- STATE  = JointState from the topic that carries actual robot arm joint angles
  (e.g. robot feedback on /joint_states or a dedicated topic).

Parsing:
- Both are sensor_msgs/msg/JointState (name + position arrays).
- Arm joints are selected by name (e.g. joint1..joint6 for 6-DOF gello).

Usage:
  # Source ROS2 workspace if needed for custom msgs (optional for JointState-only bags):
  # source /opt/ros/humble/setup.bash
  # source ~/tetheria/aero-open-ros2/install/setup.bash

  cd ~/tetheria/tidyverse-hand/tools
  python3 plot_arm_action_state.py --input /path/to/rosbag2_2026_01_18_xxx --output-dir ./plots

  # Default: /joint_states for both (yam_gello_controller setup)
  python3 plot_arm_action_state.py --input /path/to/bag --output-dir ./plots

  # First 60s only (one episode): --max-duration 60
  python3 plot_arm_action_state.py --input /path/to/bag --output-dir ./plots --max-duration 60

  # Override topics if your bag uses different names:
  python3 plot_arm_action_state.py --input /path/to/bag --action-topic /joint_states --state-topic /joint_states --output-dir ./plots
"""

from __future__ import annotations

# Use default env python3 + system matplotlib: load matplotlib first so numpy
# is loaded in an order compatible with matplotlib (numpy<2 if using system matplotlib).
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: F401

import argparse
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

# Reuse rosbag reading and JointState parsing from the converter
from convert_rosbag_to_groot_lerobot import (
    read_rosbag_topics,
    extract_joint_state_by_names,
)


# Default arm joint names (gello_publisher uses joint1..joint6)
DEFAULT_ARM_JOINT_NAMES = [f"joint{i+1}" for i in range(6)]
# yam_gello_controller subscribes to /joint_states; gello_publisher publishes there (sim) or /right/gello_js (real)
DEFAULT_ACTION_TOPIC = "/joint_states"
DEFAULT_STATE_TOPIC = "/joint_states"


def extract_timeseries(
    collected: Dict[str, List[Any]],
    topic: str,
    joint_names: Sequence[str],
    num_joints: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract (times_sec, positions_rad) from a JointState topic.
    positions_rad shape: (n_msgs, num_joints).
    """
    stream = collected.get(topic, [])
    if not stream:
        return np.array([]), np.zeros((0, num_joints), dtype=np.float32)
    times = np.array([m.t for m in stream], dtype=np.float64)
    positions = np.stack(
        [
            extract_joint_state_by_names(m.msg, joint_names, num_joints)
            for m in stream
        ],
        axis=0,
    )
    return times, positions


def plot_action_state(
    bag_path: Path,
    output_dir: Path,
    action_topic: str = DEFAULT_ACTION_TOPIC,
    state_topic: str = DEFAULT_STATE_TOPIC,
    arm_joint_names: Optional[Sequence[str]] = None,
    arm_joint_dim: int = 7,
    max_duration_sec: Optional[float] = None,
) -> List[Path]:
    """
    Read one rosbag, parse action (gello output) and state (actual arm), plot per-joint
    time series and overlay, save figures to output_dir.
    Returns list of saved file paths.
    """
    # plt already imported at top (with Agg backend for default env)

    if arm_joint_names is None:
        arm_joint_names = DEFAULT_ARM_JOINT_NAMES
    num_joints = min(arm_joint_dim, len(arm_joint_names), 7)
    joint_labels = list(arm_joint_names)[:num_joints]

    # Collect both topics (keys in collected are exactly the strings we pass)
    topics = list({action_topic, state_topic})
    collected = read_rosbag_topics(bag_path, topics, debug_topics=False)

    action_times, action_pos = extract_timeseries(
        collected, action_topic, arm_joint_names, num_joints
    )
    state_times, state_pos = extract_timeseries(
        collected, state_topic, arm_joint_names, num_joints
    )

    has_action = len(action_times) > 0
    has_state = len(state_times) > 0
    if not has_action and not has_state:
        counts = {k: len(v) for k, v in collected.items()}
        raise RuntimeError(
            f"No messages on action topic '{action_topic}' or state topic '{state_topic}'.\n"
            f"Collected topic counts: {counts}\n"
            f"Run: ./01_inspect_rosbag.sh <bag_path>  to list topics in your bag, then set --action-topic / --state-topic to match."
        )

    # Optionally trim to first N seconds (e.g. one episode)
    if max_duration_sec is not None and max_duration_sec > 0:
        t0_trim = float("inf")
        if has_action:
            t0_trim = min(t0_trim, float(action_times[0]))
        if has_state:
            t0_trim = min(t0_trim, float(state_times[0]))
        if t0_trim != float("inf"):
            t1_trim = t0_trim + max_duration_sec
            if has_action:
                mask = (action_times >= t0_trim) & (action_times <= t1_trim)
                action_times = action_times[mask]
                action_pos = action_pos[mask]
            if has_state:
                mask = (state_times >= t0_trim) & (state_times <= t1_trim)
                state_times = state_times[mask]
                state_pos = state_pos[mask]
            has_action = has_action and len(action_times) > 0
            has_state = has_state and len(state_times) > 0

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    bag_name = bag_path.name if bag_path.is_dir() else bag_path.stem
    saved: List[Path] = []

    # Relative time for plotting
    t0 = float("inf")
    if has_action:
        t0 = min(t0, float(action_times[0]))
    if has_state:
        t0 = min(t0, float(state_times[0]))
    if t0 == float("inf"):
        t0 = 0.0

    # ---- Figure 1: Action (desired) channels ----
    if has_action:
        fig1, axes1 = plt.subplots(num_joints, 1, figsize=(10, 2 * num_joints), sharex=True)
        if num_joints == 1:
            axes1 = [axes1]
        t_rel = action_times - t0
        for i, ax in enumerate(axes1):
            ax.plot(t_rel, action_pos[:, i], color="C0", alpha=0.8, label="action (desired)")
            ax.set_ylabel(f"{joint_labels[i]}\n(rad)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.3)
        axes1[-1].set_xlabel("Time (s)")
        fig1.suptitle(f"Arm ACTION (desired from gello) — {bag_name}\nTopic: {action_topic}", fontsize=10)
        fig1.tight_layout()
        out1 = output_dir / f"arm_action_{bag_name}.png"
        fig1.savefig(out1, dpi=150)
        plt.close(fig1)
        saved.append(out1)
        print(f"Saved: {out1}")

    # ---- Figure 2: State (actual) channels ----
    if has_state:
        fig2, axes2 = plt.subplots(num_joints, 1, figsize=(10, 2 * num_joints), sharex=True)
        if num_joints == 1:
            axes2 = [axes2]
        t_rel = state_times - t0
        for i, ax in enumerate(axes2):
            ax.plot(t_rel, state_pos[:, i], color="C1", alpha=0.8, label="state (actual)")
            ax.set_ylabel(f"{joint_labels[i]}\n(rad)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.3)
        axes2[-1].set_xlabel("Time (s)")
        fig2.suptitle(f"Arm STATE (actual joint angles) — {bag_name}\nTopic: {state_topic}", fontsize=10)
        fig2.tight_layout()
        out2 = output_dir / f"arm_state_{bag_name}.png"
        fig2.savefig(out2, dpi=150)
        plt.close(fig2)
        saved.append(out2)
        print(f"Saved: {out2}")

    # ---- Figure 3: Overlay action vs state per joint (same time base via nearest-neighbor) ----
    if has_action and has_state:
        # Resample to a common time grid (use action time as reference; state may be different rate)
        t_min = max(action_times[0], state_times[0])
        t_max = min(action_times[-1], state_times[-1])
        if t_max > t_min:
            # Common grid at ~30 Hz over [t_min, t_max]
            n_pts = max(100, int((t_max - t_min) * 30))
            t_common = np.linspace(t_min, t_max, n_pts)
            action_interp = np.zeros((n_pts, num_joints), dtype=np.float32)
            state_interp = np.zeros((n_pts, num_joints), dtype=np.float32)
            for j in range(num_joints):
                action_interp[:, j] = np.interp(t_common, action_times, action_pos[:, j])
                state_interp[:, j] = np.interp(t_common, state_times, state_pos[:, j])
            t_rel_common = t_common - t0

            fig3, axes3 = plt.subplots(num_joints, 1, figsize=(10, 2 * num_joints), sharex=True)
            if num_joints == 1:
                axes3 = [axes3]
            for i, ax in enumerate(axes3):
                ax.plot(t_rel_common, action_interp[:, i], color="C0", alpha=0.8, label="action (desired)")
                ax.plot(t_rel_common, state_interp[:, i], color="C1", alpha=0.8, label="state (actual)")
                ax.set_ylabel(f"{joint_labels[i]}\n(rad)")
                ax.legend(loc="upper right", fontsize=8)
                ax.grid(True, alpha=0.3)
            axes3[-1].set_xlabel("Time (s)")
            fig3.suptitle(f"Arm ACTION vs STATE — {bag_name}", fontsize=10)
            fig3.tight_layout()
            out3 = output_dir / f"arm_action_vs_state_{bag_name}.png"
            fig3.savefig(out3, dpi=150)
            plt.close(fig3)
            saved.append(out3)
            print(f"Saved: {out3}")
        else:
            print("Skipping action vs state overlay (no overlapping time range).")

    return saved


def main():
    parser = argparse.ArgumentParser(
        description="Plot arm joint ACTION (gello desired) vs STATE (actual) from one rosbag.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--input", type=Path, required=True, help="Path to one rosbag directory (e.g. rosbag2_2026_01_18_...)")
    parser.add_argument("--output-dir", type=Path, default=Path("./plots"), help="Directory to save PNGs (default: ./plots)")
    parser.add_argument(
        "--action-topic",
        type=str,
        default=DEFAULT_ACTION_TOPIC,
        help="Topic for desired arm joints (gello_publisher output). Sim: /joint_states, real: /right/gello_js",
    )
    parser.add_argument(
        "--state-topic",
        type=str,
        default=DEFAULT_STATE_TOPIC,
        help="Topic for actual arm joint angles (robot feedback)",
    )
    parser.add_argument(
        "--arm-joint-names",
        type=str,
        default=None,
        help="Comma-separated arm joint names (default: joint1,joint2,...,joint6)",
    )
    parser.add_argument("--arm-joint-dim", type=int, default=6, help="Number of arm joints (default: 6)")
    parser.add_argument("--max-duration", type=float, default=None, metavar="SEC",
                        help="Plot only first SEC seconds (e.g. one episode)")
    args = parser.parse_args()

    arm_joint_names = None
    if args.arm_joint_names:
        arm_joint_names = [s.strip() for s in args.arm_joint_names.split(",") if s.strip()]

    print(f"Action topic (desired): {args.action_topic}")
    print(f"State topic (actual):  {args.state_topic}")
    print(f"Arm joint names: {arm_joint_names or DEFAULT_ARM_JOINT_NAMES}")
    print(f"Output dir: {args.output_dir}")

    saved = plot_action_state(
        bag_path=args.input,
        output_dir=args.output_dir,
        action_topic=args.action_topic,
        state_topic=args.state_topic,
        arm_joint_names=arm_joint_names,
        arm_joint_dim=args.arm_joint_dim,
        max_duration_sec=args.max_duration,
    )
    print(f"Done. Saved {len(saved)} plot(s).")


if __name__ == "__main__":
    main()
