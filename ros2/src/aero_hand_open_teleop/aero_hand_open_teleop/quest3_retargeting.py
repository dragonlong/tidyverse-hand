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

"""Retarget Quest 3 hand-tracking landmarks to Aero Hand joint commands.

Subscribes to the 21-landmark PoseArray topics published by
``hand_tracking_sdk_ros2`` and outputs 16-DOF ``JointControl`` messages
compatible with the Aero Hand node.

Landmark-to-angle pipeline:
1. Extract 21 3-D positions from ``PoseArray``.
2. Build a hand-local coordinate frame (wrist-centred, palm-aligned).
3. Expand to the 25-keypoint layout used by the existing mediapipe
   retargeting (wrist prepended as CMC surrogate for each finger).
4. Compute joint angles via three-point geometry.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np

from geometry_msgs.msg import PoseArray
from aero_hand_open_msgs.msg import JointControl
from aero_open_sdk.aero_hand_constants import AeroHandConstants
from aero_hand_open_teleop.utils.normalize import normalize_joint_state
from aero_hand_open_teleop.utils.load_normalize_config import load_normalize_config

# Quest 3 / MediaPipe-style 21-landmark indices
_WRIST = 0
_THUMB_CMC = 1
_THUMB_MCP = 2
_THUMB_IP = 3
_THUMB_TIP = 4
_INDEX_MCP = 5
_INDEX_PIP = 6
_INDEX_DIP = 7
_INDEX_TIP = 8
_MIDDLE_MCP = 9
_MIDDLE_PIP = 10
_MIDDLE_DIP = 11
_MIDDLE_TIP = 12
_RING_MCP = 13
_RING_PIP = 14
_RING_DIP = 15
_RING_TIP = 16
_PINKY_MCP = 17
_PINKY_PIP = 18
_PINKY_DIP = 19
_PINKY_TIP = 20

_EXPECTED_LANDMARKS = 21


class Quest3Retargeting(Node):
    def __init__(self):
        super().__init__("quest3_retargeting")

        self.declare_parameter("ema_alpha", 0.7)
        self.ema_alpha = self.get_parameter("ema_alpha").value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.left_sub = self.create_subscription(
            PoseArray, "hands/left/landmarks", self._left_callback, sensor_qos
        )
        self.right_sub = self.create_subscription(
            PoseArray, "hands/right/landmarks", self._right_callback, sensor_qos
        )

        self.joint_states_right_pub = self.create_publisher(
            JointControl, "right/joint_control", 10
        )
        self.joint_states_left_pub = self.create_publisher(
            JointControl, "left/joint_control", 10
        )

        self.joint_ll_rad = np.deg2rad(AeroHandConstants.joint_lower_limits)
        self.joint_ul_rad = np.deg2rad(AeroHandConstants.joint_upper_limits)

        # Use the mediapipe config: Quest 3 uses the same 3-point geometry as
        # MediaPipe, so this calibration matches the raw angle ranges produced
        # here.  default_user was calibrated for Manus atan2-from-pose-data
        # and gives wrong valley/peak values for this geometric computation.
        self.normalize_config = load_normalize_config("default_mediapipe")

        # EMA cache operates on the 16 output joint angles (after angle
        # extraction), not on 3D landmark positions.  Smoothing positions in
        # the hand-local frame causes subtle blending artefacts when the wrist
        # rotates quickly because the frame origin/orientation changes each
        # frame.
        self._cache_left = np.zeros(16)
        self._cache_right = np.zeros(16)

        self.get_logger().info("Quest 3 Retargeting Node started.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _left_callback(self, msg: PoseArray):
        self._process(msg, "left")

    def _right_callback(self, msg: PoseArray):
        self._process(msg, "right")

    def _process(self, msg: PoseArray, side: str):
        if len(msg.poses) != _EXPECTED_LANDMARKS:
            self.get_logger().warn(
                f"Expected {_EXPECTED_LANDMARKS} landmarks, got {len(msg.poses)}"
            )
            return

        raw = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in msg.poses]
        )

        local = self._to_hand_local(raw, side)
        expanded = self._expand_to_25(local)
        raw_angles = np.array(self._retarget(expanded))

        # Apply EMA to joint angles (not 3D positions) so that wrist
        # translation/rotation does not mix frames across the smoothing window.
        if side == "right":
            self._cache_right = (
                self.ema_alpha * raw_angles
                + (1 - self.ema_alpha) * self._cache_right
            )
            joint_values = self._cache_right.copy()
        else:
            self._cache_left = (
                self.ema_alpha * raw_angles
                + (1 - self.ema_alpha) * self._cache_left
            )
            joint_values = self._cache_left.copy()

        joint_values = np.clip(
            joint_values, self.joint_ll_rad, self.joint_ul_rad
        ).tolist()

        for i in range(4):
            joint_values[i] = normalize_joint_state(
                joint_values[i], i, self.normalize_config
            )

        self._publish(side, joint_values)

    # ------------------------------------------------------------------
    # Coordinate transform
    # ------------------------------------------------------------------

    @staticmethod
    def _to_hand_local(landmarks: np.ndarray, side: str) -> np.ndarray:
        """Transform world-frame landmarks into a wrist-centred hand frame.

        Hand-local axes (same convention as mediapipe_mocap):
          X  = lateral (index → ring direction, flipped for left hand)
          Z  = along the hand (wrist → middle MCP)
          Y  = palm normal (cross of Z and X)
        """
        x_axis = landmarks[_INDEX_MCP] - landmarks[_RING_MCP]
        norm = np.linalg.norm(x_axis)
        if norm < 1e-8:
            return landmarks
        x_axis /= norm
        if side == "left":
            x_axis = -x_axis

        z_axis = landmarks[_MIDDLE_MCP] - landmarks[_WRIST]
        norm = np.linalg.norm(z_axis)
        if norm < 1e-8:
            return landmarks
        z_axis /= norm

        y_axis = np.cross(z_axis, x_axis)
        norm = np.linalg.norm(y_axis)
        if norm < 1e-8:
            return landmarks
        y_axis /= norm

        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)

        base = landmarks[_WRIST].copy()
        rot = np.array([x_axis, y_axis, z_axis]).T
        return (landmarks - base) @ rot

    @staticmethod
    def _expand_to_25(lm: np.ndarray) -> np.ndarray:
        """Expand 21 landmarks to the 25-keypoint layout.

        Each finger group gets the wrist prepended as its CMC surrogate,
        matching the convention used by ``mediapipe_mocap`` and consumed
        by ``mediapipe_retargeting``.
        """
        return np.array([
            lm[_WRIST], lm[_THUMB_CMC], lm[_THUMB_MCP], lm[_THUMB_IP], lm[_THUMB_TIP],
            lm[_WRIST], lm[_INDEX_MCP], lm[_INDEX_PIP], lm[_INDEX_DIP], lm[_INDEX_TIP],
            lm[_WRIST], lm[_MIDDLE_MCP], lm[_MIDDLE_PIP], lm[_MIDDLE_DIP], lm[_MIDDLE_TIP],
            lm[_WRIST], lm[_RING_MCP], lm[_RING_PIP], lm[_RING_DIP], lm[_RING_TIP],
            lm[_WRIST], lm[_PINKY_MCP], lm[_PINKY_PIP], lm[_PINKY_DIP], lm[_PINKY_TIP],
        ])

    # ------------------------------------------------------------------
    # Joint angle extraction
    # ------------------------------------------------------------------

    @staticmethod
    def _angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
        """Angle at vertex *b* formed by points *a-b-c*."""
        ba = a - b
        bc = c - b
        cos = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-8)
        return float(np.arccos(np.clip(cos, -1.0, 1.0)))

    def _get_thumb_joints(self, lm: np.ndarray) -> list:
        """Return [cmc_abd, cmc_flex, mcp, ip] from 5 thumb landmarks."""
        a = np.array([lm[2][0], lm[1][1], 0.0])
        b = np.array([lm[1][0], lm[1][1], 0.0])
        c = np.array([lm[2][0], lm[2][1], 0.0])
        cmc_abd = self._angle(a, b, c)

        cmc_flex = np.pi - self._angle(lm[0], lm[1], lm[2])
        mcp = np.pi - self._angle(lm[1], lm[2], lm[3])
        ip = np.pi - self._angle(lm[2], lm[3], lm[4])
        return [cmc_abd, cmc_flex, mcp, ip]

    def _get_finger_joints(self, lm: np.ndarray) -> list:
        """Return [mcp_flex, pip, dip] from 5 finger landmarks."""
        mcp = np.pi - self._angle(lm[0], lm[1], lm[2])
        pip = np.pi - self._angle(lm[1], lm[2], lm[3])
        dip = np.pi - self._angle(lm[2], lm[3], lm[4])
        return [mcp, pip, dip]

    def _retarget(self, lm25: np.ndarray) -> list:
        """Convert 25-keypoint array to 16 Aero Hand joint angles."""
        thumb = self._get_thumb_joints(lm25[0:5])
        index = self._get_finger_joints(lm25[5:10])
        middle = self._get_finger_joints(lm25[10:15])
        ring = self._get_finger_joints(lm25[15:20])
        pinky = self._get_finger_joints(lm25[20:25])
        return thumb + index + middle + ring + pinky

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _publish(self, side: str, joint_values: list):
        msg = JointControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.target_positions = joint_values
        if side == "right":
            self.joint_states_right_pub.publish(msg)
        else:
            self.joint_states_left_pub.publish(msg)


def main():
    rclpy.init()
    node = Quest3Retargeting()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
