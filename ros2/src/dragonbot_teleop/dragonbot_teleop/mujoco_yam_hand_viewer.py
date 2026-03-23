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

"""Combined YAM arm + Aero Hand MuJoCo visualiser.

Subscribes to:
  /joint_states       (sensor_msgs/JointState)   — 6 YAM arm joints from Gello
  /right/joint_control (aero_hand_open_msgs/JointControl) — 16 hand joints

Applies both via FK (mj_forward only, no physics step) to the combined
yam_with_hand MuJoCo scene and renders it in a passive viewer window.

Usage::

    ros2 run aero_hand_open_teleop mujoco_yam_hand_viewer

    # With a custom scene:
    ros2 run aero_hand_open_teleop mujoco_yam_hand_viewer \\
        --ros-args -p scene_xml:=/path/to/scene_yam_with_hand.xml

Or via the helper script::

    bash ~/tetheria/tidyverse-hand/scripts/utils/sim_yam_with_hand.sh
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from aero_hand_open_msgs.msg import JointControl

# ---------------------------------------------------------------------------
# Joint name tables
# ---------------------------------------------------------------------------

# YAM arm joints — positional order matches Gello publisher output
_ARM_JOINT_NAMES: list[str] = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]

# Hand joints (prefixed with "hand_" in the combined scene due to <attach prefix>)
# Index order matches JointControl.target_positions[0..15]
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
    "/home/dragonx/tetheria/tidyverse-hand"
    "/models/yam_with_hand/scene_yam_with_hand.xml"
)


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class YamHandMujocoViewer(Node):
    """Bridges /joint_states and /right/joint_control into MuJoCo FK display."""

    def __init__(self) -> None:
        super().__init__("yam_hand_mujoco_viewer")

        self.declare_parameter("scene_xml", _DEFAULT_SCENE)
        scene_path = self.get_parameter("scene_xml").value

        # ── Load MuJoCo model ────────────────────────────────────────────────
        self._model = mujoco.MjModel.from_xml_path(scene_path)
        self._data = mujoco.MjData(self._model)

        # ── Resolve YAM arm joint qpos addresses ─────────────────────────────
        self._arm_qpos_addrs: list[int] = []
        arm_missing: list[str] = []
        for jname in _ARM_JOINT_NAMES:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid == -1:
                arm_missing.append(jname)
                self._arm_qpos_addrs.append(-1)
            else:
                self._arm_qpos_addrs.append(int(self._model.jnt_qposadr[jid]))
        if arm_missing:
            self.get_logger().warn(f"ARM joints not found in model: {arm_missing}")

        # ── Resolve hand joint qpos addresses ────────────────────────────────
        self._hand_qpos_addrs: list[int] = []
        hand_missing: list[str] = []
        for jname in _HAND_JOINT_NAMES:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid == -1:
                hand_missing.append(jname)
                self._hand_qpos_addrs.append(-1)
            else:
                self._hand_qpos_addrs.append(int(self._model.jnt_qposadr[jid]))
        if hand_missing:
            self.get_logger().warn(f"HAND joints not found in model: {hand_missing}")

        # Initial FK pass so the viewer opens with the default pose.
        mujoco.mj_forward(self._model, self._data)

        # ── Thread-safe state buffers ─────────────────────────────────────────
        self._lock = threading.Lock()
        self._arm_joints: list[float] | None = None
        self._hand_joints: list[float] | None = None
        self._arm_frame_count = 0
        self._hand_frame_count = 0

        # ── QoS ──────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(
            JointState,
            "joint_states",
            self._arm_cb,
            sensor_qos,
        )
        self.create_subscription(
            JointControl,
            "right/joint_control",
            self._hand_cb,
            sensor_qos,
        )

        arm_mapped = sum(a >= 0 for a in self._arm_qpos_addrs)
        hand_mapped = sum(a >= 0 for a in self._hand_qpos_addrs)
        self.get_logger().info(
            f"YamHandMujocoViewer ready.\n"
            f"  Scene : {scene_path}\n"
            f"  Joints: {self._model.njnt} total in model\n"
            f"  ARM   : {arm_mapped}/6 mapped  (/joint_states)\n"
            f"  HAND  : {hand_mapped}/16 mapped (/right/joint_control)\n"
            f"  Waiting for messages..."
        )

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _arm_cb(self, msg: JointState) -> None:
        positions = list(msg.position)
        if len(positions) < 6:
            self.get_logger().warn_once(
                f"Expected ≥6 arm positions, got {len(positions)}"
            )
            return
        with self._lock:
            self._arm_joints = positions[:6]
            self._arm_frame_count += 1

    def _hand_cb(self, msg: JointControl) -> None:
        if len(msg.target_positions) != 16:
            self.get_logger().warn_once(
                f"Expected 16 hand positions, got {len(msg.target_positions)}"
            )
            return
        with self._lock:
            self._hand_joints = list(msg.target_positions)
            self._hand_frame_count += 1

    # ── Called from main thread each viewer tick ──────────────────────────────

    def apply_to_sim(self) -> None:
        """Write latest joint commands into MuJoCo qpos and run FK."""
        with self._lock:
            arm = self._arm_joints
            hand = self._hand_joints

        updated = False

        if arm is not None:
            for i, addr in enumerate(self._arm_qpos_addrs):
                if addr >= 0 and i < len(arm):
                    self._data.qpos[addr] = arm[i]
            updated = True

        if hand is not None:
            for i, addr in enumerate(self._hand_qpos_addrs):
                if addr >= 0:
                    self._data.qpos[addr] = hand[i]
            updated = True

        if updated:
            mujoco.mj_forward(self._model, self._data)

    def log_hz(self, elapsed: float) -> None:
        with self._lock:
            arm_n = self._arm_frame_count
            hand_n = self._hand_frame_count
            self._arm_frame_count = 0
            self._hand_frame_count = 0
        arm_hz = arm_n / elapsed
        hand_hz = hand_n / elapsed
        print(
            f"[yam_hand_viewer]  arm {arm_hz:.1f} Hz  hand {hand_hz:.1f} Hz"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YamHandMujocoViewer()

    # ROS callbacks run in a background daemon thread; MuJoCo viewer owns main.
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True, name="ros_spin"
    )
    ros_thread.start()

    last_log = time.monotonic()

    with mujoco.viewer.launch_passive(node._model, node._data) as vis:
        while vis.is_running():
            node.apply_to_sim()
            vis.sync()
            time.sleep(0.01)  # ~100 Hz render cap

            now = time.monotonic()
            if now - last_log >= 5.0:
                node.log_hz(now - last_log)
                last_log = now

    rclpy.shutdown()


if __name__ == "__main__":
    main()
