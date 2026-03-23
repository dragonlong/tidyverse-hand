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

"""Real-time Aero Hand visualization in MuJoCo.

Subscribes to /right/joint_control (JointControl, 16 DOF) and drives the
MuJoCo hand model by setting joint positions kinematically (mj_forward only —
no physics step, no tendon solver).  No hardware required.

Run alongside the full Quest 3 retargeting pipeline to verify the 16-joint
mapping before connecting the robot:

    # Terminal 1 (local PC)
    ros2 launch aero_hand_open_teleop quest3_teleop.launch.py

    # Terminal 2
    ros2 run aero_hand_open_teleop mujoco_hand_viewer

Or via the helper script:
    bash ~/tetheria/commands/09_mujoco_hand_viewer.sh
"""

import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from aero_hand_open_msgs.msg import JointControl


# ---------------------------------------------------------------------------
# JointControl index → MuJoCo joint name
# Matches JOINT_NAME_MAP in normalize.py, prefixed with "right_".
# ---------------------------------------------------------------------------

_JOINT_NAMES = [
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

_DEFAULT_SCENE = (
    "/home/dragonx/tetheria/mujoco_menagerie"
    "/tetheria_aero_hand_open/scene_right.xml"
)


# ---------------------------------------------------------------------------
# ROS node — subscribes and caches latest joint command
# ---------------------------------------------------------------------------

class AeroHandMujocoViewer(Node):
    """Bridges /right/joint_control into a MuJoCo kinematic display."""

    def __init__(self) -> None:
        super().__init__("aero_hand_mujoco_viewer")

        self.declare_parameter("scene_xml", _DEFAULT_SCENE)
        scene_path = self.get_parameter("scene_xml").value

        # ── Load MuJoCo model ────────────────────────────────────────────────
        self._model = mujoco.MjModel.from_xml_path(scene_path)
        self._data = mujoco.MjData(self._model)

        # Resolve qpos addresses for all 16 joints by name.
        # -1 means the joint was not found (model mismatch — warn at startup).
        self._qpos_addrs: list[int] = []
        missing = []
        for jname in _JOINT_NAMES:
            jid = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_JOINT, jname
            )
            if jid == -1:
                missing.append(jname)
                self._qpos_addrs.append(-1)
            else:
                self._qpos_addrs.append(int(self._model.jnt_qposadr[jid]))

        if missing:
            self.get_logger().warn(
                f"Joints not found in model (will be skipped): {missing}"
            )

        # Propagate FK once so the viewer opens in the model's default pose.
        mujoco.mj_forward(self._model, self._data)

        # ── Thread-safe joint buffer ──────────────────────────────────────────
        self._lock = threading.Lock()
        self._latest: list[float] | None = None
        self._frame_count = 0

        # ── ROS subscription ─────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            JointControl,
            "right/joint_control",
            self._joint_control_cb,
            sensor_qos,
        )

        self.get_logger().info(
            f"AeroHandMujocoViewer ready.\n"
            f"  Scene : {scene_path}\n"
            f"  Joints: {self._model.njnt} total, "
            f"{sum(a >= 0 for a in self._qpos_addrs)}/16 mapped\n"
            f"  Waiting for /right/joint_control messages..."
        )

    # ── ROS callback ─────────────────────────────────────────────────────────

    def _joint_control_cb(self, msg: JointControl) -> None:
        if len(msg.target_positions) != 16:
            self.get_logger().warn_once(
                f"Expected 16 positions, got {len(msg.target_positions)} — skipping."
            )
            return
        with self._lock:
            self._latest = list(msg.target_positions)
            self._frame_count += 1

    # ── Called from main thread by viewer loop ────────────────────────────────

    def apply_to_sim(self) -> bool:
        """Push latest joint command into MuJoCo qpos and run FK.

        Returns True if a new frame was applied.
        """
        with self._lock:
            joints = self._latest

        if joints is None:
            return False

        for i, addr in enumerate(self._qpos_addrs):
            if addr >= 0:
                self._data.qpos[addr] = joints[i]

        # FK only — no physics step, no constraint enforcement.
        mujoco.mj_forward(self._model, self._data)
        return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = AeroHandMujocoViewer()

    # Spin ROS callbacks in a background daemon thread so the MuJoCo viewer
    # can own the main thread (required by GLFW/OpenGL on most platforms).
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

            # Print Hz to terminal once per second
            now = time.monotonic()
            if now - last_log >= 5.0:
                with node._lock:
                    n = node._frame_count
                    node._frame_count = 0
                hz = n / (now - last_log)
                print(f"[mujoco_hand_viewer] receiving {hz:.1f} Hz joint commands")
                last_log = now

    rclpy.shutdown()


if __name__ == "__main__":
    main()
