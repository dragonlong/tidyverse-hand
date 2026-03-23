#!/usr/bin/env python3
# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Full Dragonbot MuJoCo visualiser — mobile base + YAM arm + Aero Hand.

Subscribes to:
  /spacemouse/cmd_vel   (geometry_msgs/Twist)        — 3-DOF base velocity
  /joint_states         (sensor_msgs/JointState)      — 6 YAM arm joints
  /right/joint_control  (aero_hand_open_msgs/JointControl) — 16 hand joints

Applies all three via FK (mj_forward only, no physics step) to the Dragonbot
combined MuJoCo scene and renders it in a passive viewer window.

Base velocity is integrated in real-time into (joint_x, joint_y, joint_th)
using holonomic kinematics:

    x   += (vx·cos θ − vy·sin θ) · dt
    y   += (vx·sin θ + vy·cos θ) · dt
    θ   += ω · dt

Usage::

    ros2 run aero_hand_open_teleop mujoco_dragonbot_viewer

    # Custom scene:
    ros2 run aero_hand_open_teleop mujoco_dragonbot_viewer \\
        --ros-args -p scene_xml:=/path/to/scene.xml

Or via the helper script::

    bash ~/tetheria/tidyverse-hand/scripts/utils/sim_dragonbot.sh
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from aero_hand_open_msgs.msg import JointControl

# ---------------------------------------------------------------------------
# Joint name tables
# ---------------------------------------------------------------------------

_BASE_JOINT_NAMES: list[str] = ["joint_x", "joint_y", "joint_th"]

_ARM_JOINT_NAMES: list[str] = [
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
]

_HAND_JOINT_BASE_NAMES: list[str] = [
    "right_thumb_cmc_abd",    # 0
    "right_thumb_cmc_flex",   # 1
    "right_thumb_mcp",        # 2
    "right_thumb_ip",         # 3
    "right_index_mcp_flex",   # 4
    "right_index_pip",        # 5
    "right_index_dip",        # 6
    "right_middle_mcp_flex",  # 7
    "right_middle_pip",       # 8
    "right_middle_dip",       # 9
    "right_ring_mcp_flex",    # 10
    "right_ring_pip",         # 11
    "right_ring_dip",         # 12
    "right_pinky_mcp_flex",   # 13
    "right_pinky_pip",        # 14
    "right_pinky_dip",        # 15
]

_HAND_JOINT_NAMES: list[str] = [f"hand_{n}" for n in _HAND_JOINT_BASE_NAMES]

_DEFAULT_SCENE = (
    "/home/dragonx/tetheria/tidyverse-hand/models/dragonbot/scene.xml"
)


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class DragonbotMujocoViewer(Node):
    """Bridges all three Dragonbot input streams into a MuJoCo FK display."""

    def __init__(self) -> None:
        super().__init__("dragonbot_mujoco_viewer")

        self.declare_parameter("scene_xml", _DEFAULT_SCENE)
        scene_path = self.get_parameter("scene_xml").value

        # ── Load model ───────────────────────────────────────────────────────
        self._model = mujoco.MjModel.from_xml_path(scene_path)
        self._data = mujoco.MjData(self._model)

        # ── Resolve qpos addresses ───────────────────────────────────────────
        self._base_qpos_addrs = self._resolve_joints(_BASE_JOINT_NAMES, "BASE")
        self._arm_qpos_addrs = self._resolve_joints(_ARM_JOINT_NAMES, "ARM")
        self._hand_qpos_addrs = self._resolve_joints(_HAND_JOINT_NAMES, "HAND")

        mujoco.mj_forward(self._model, self._data)

        # ── State buffers (written by ROS thread, read by main thread) ────────
        self._lock = threading.Lock()

        # Base: integrated world pose
        self._base_x: float = 0.0
        self._base_y: float = 0.0
        self._base_th: float = 0.0
        self._last_twist_time: float | None = None

        # Latest velocity command
        self._twist_vx: float = 0.0
        self._twist_vy: float = 0.0
        self._twist_wz: float = 0.0
        self._twist_stamp: float = 0.0  # monotonic seconds

        # Arm + hand
        self._arm_joints: list[float] | None = None
        self._hand_joints: list[float] | None = None

        # Hz counters
        self._base_frames = 0
        self._arm_frames = 0
        self._hand_frames = 0

        # ── QoS ──────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Twist, "spacemouse/cmd_vel",
                                 self._base_cb, sensor_qos)
        self.create_subscription(JointState, "joint_states",
                                 self._arm_cb, sensor_qos)
        self.create_subscription(JointControl, "right/joint_control",
                                 self._hand_cb, sensor_qos)

        n_base = sum(a >= 0 for a in self._base_qpos_addrs)
        n_arm  = sum(a >= 0 for a in self._arm_qpos_addrs)
        n_hand = sum(a >= 0 for a in self._hand_qpos_addrs)
        self.get_logger().info(
            f"DragonbotMujocoViewer ready.\n"
            f"  Scene : {scene_path}\n"
            f"  BASE  : {n_base}/3  mapped (/spacemouse/cmd_vel)\n"
            f"  ARM   : {n_arm}/6  mapped (/joint_states)\n"
            f"  HAND  : {n_hand}/16 mapped (/right/joint_control)\n"
            f"  Waiting for messages..."
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _resolve_joints(
        self, names: list[str], label: str
    ) -> list[int]:
        addrs: list[int] = []
        missing: list[str] = []
        for jname in names:
            jid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, jname
            )
            if jid == -1:
                missing.append(jname)
                addrs.append(-1)
            else:
                addrs.append(int(self._model.jnt_qposadr[jid]))
        if missing:
            self.get_logger().warn(
                f"{label} joints not found in model: {missing}"
            )
        return addrs

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _base_cb(self, msg: Twist) -> None:
        with self._lock:
            self._twist_vx = msg.linear.x
            self._twist_vy = msg.linear.y
            self._twist_wz = msg.angular.z
            self._twist_stamp = time.monotonic()
            self._base_frames += 1

    def _arm_cb(self, msg: JointState) -> None:
        positions = list(msg.position)
        if len(positions) < 6:
            self.get_logger().warn_once(
                f"Expected ≥6 arm positions, got {len(positions)}"
            )
            return
        with self._lock:
            self._arm_joints = positions[:6]
            self._arm_frames += 1

    def _hand_cb(self, msg: JointControl) -> None:
        if len(msg.target_positions) != 16:
            self.get_logger().warn_once(
                f"Expected 16 hand positions, got {len(msg.target_positions)}"
            )
            return
        with self._lock:
            self._hand_joints = list(msg.target_positions)
            self._hand_frames += 1

    # ── Apply state to sim (called from main thread) ──────────────────────────

    def apply_to_sim(self, dt: float) -> None:
        """Integrate base velocity and write all joint state into MuJoCo qpos."""
        with self._lock:
            vx = self._twist_vx
            vy = self._twist_vy
            wz = self._twist_wz
            twist_stamp = self._twist_stamp
            arm = self._arm_joints
            hand = self._hand_joints

        now = time.monotonic()

        # Stop integrating base if no Twist received for >0.5 s (deadman)
        if twist_stamp is not None and (now - twist_stamp) < 0.5:
            th = self._base_th
            self._base_x  += (vx * math.cos(th) - vy * math.sin(th)) * dt
            self._base_y  += (vx * math.sin(th) + vy * math.cos(th)) * dt
            self._base_th += wz * dt

        # Write base qpos
        if self._base_qpos_addrs[0] >= 0:
            self._data.qpos[self._base_qpos_addrs[0]] = self._base_x
        if self._base_qpos_addrs[1] >= 0:
            self._data.qpos[self._base_qpos_addrs[1]] = self._base_y
        if self._base_qpos_addrs[2] >= 0:
            self._data.qpos[self._base_qpos_addrs[2]] = self._base_th

        # Write arm qpos
        if arm is not None:
            for i, addr in enumerate(self._arm_qpos_addrs):
                if addr >= 0 and i < len(arm):
                    self._data.qpos[addr] = arm[i]

        # Write hand qpos
        if hand is not None:
            for i, addr in enumerate(self._hand_qpos_addrs):
                if addr >= 0:
                    self._data.qpos[addr] = hand[i]

        mujoco.mj_forward(self._model, self._data)

    def log_hz(self, elapsed: float) -> None:
        with self._lock:
            bn, self._base_frames = self._base_frames, 0
            an, self._arm_frames  = self._arm_frames,  0
            hn, self._hand_frames = self._hand_frames,  0
        print(
            f"[dragonbot_viewer]  base {bn/elapsed:.1f} Hz  "
            f"arm {an/elapsed:.1f} Hz  hand {hn/elapsed:.1f} Hz"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DragonbotMujocoViewer()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True, name="ros_spin"
    )
    ros_thread.start()

    last_log = time.monotonic()
    last_tick = time.monotonic()

    with mujoco.viewer.launch_passive(node._model, node._data) as vis:
        while vis.is_running():
            now = time.monotonic()
            dt = now - last_tick
            last_tick = now

            node.apply_to_sim(dt)
            vis.sync()
            time.sleep(0.01)  # ~100 Hz render cap

            if now - last_log >= 5.0:
                node.log_hz(now - last_log)
                last_log = now

    rclpy.shutdown()


if __name__ == "__main__":
    main()
