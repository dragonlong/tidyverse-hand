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

"""No-hardware tests for the Quest3 YAM arm IK teleoperation node.

Covers:
  - _se3_compose: pure geometry invariants
  - _solve_ik: shape, finiteness, joint-limit compliance, determinism
  - Quest3YamTeleop node in sim_mode: reference capture, zero-delta IK,
    out-of-workspace clamping
"""

import os
import sys

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation

# Make the package importable without a colcon install.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from dragonbot_teleop.quest3_yam_teleop import Quest3YamTeleop, _se3_compose, _solve_ik

URDF_PATH = "/home/dragonx/tetheria/i2rt/i2rt/robot_models/yam/yam.urdf"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def yam_robot():
    """Load YAM URDF and build pyroki Robot once for all IK tests."""
    import pyroki as pk
    import yourdfpy

    mesh_dir = os.path.join(os.path.dirname(URDF_PATH), "assets")
    urdf = yourdfpy.URDF.load(URDF_PATH, mesh_dir=mesh_dir)
    return pk.Robot.from_urdf(urdf)


@pytest.fixture(scope="module")
def sim_node():
    """Spin up Quest3YamTeleop in sim_mode (no CAN / serial hardware)."""
    import rclpy

    rclpy.init()
    node = Quest3YamTeleop(
        parameter_overrides=[Parameter("sim_mode", value=True)]
    )
    yield node
    node.destroy_node()
    rclpy.shutdown()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _pose_stamped(node, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)
    return msg


# ---------------------------------------------------------------------------
# _se3_compose
# ---------------------------------------------------------------------------


def test_se3_compose_identity():
    """Identity ⊕ identity = identity."""
    pos_out, rot_out = _se3_compose(
        np.zeros(3), Rotation.identity(), np.zeros(3), Rotation.identity()
    )
    np.testing.assert_allclose(pos_out, np.zeros(3), atol=1e-9)
    np.testing.assert_allclose(rot_out.as_matrix(), np.eye(3), atol=1e-9)


def test_se3_compose_translation_in_rotated_frame():
    """A rotated 90° around Z, then translated +1 along A's X → world Y."""
    pos_out, _ = _se3_compose(
        np.zeros(3),
        Rotation.from_euler("z", np.pi / 2),
        np.array([1.0, 0.0, 0.0]),
        Rotation.identity(),
    )
    np.testing.assert_allclose(pos_out, np.array([0.0, 1.0, 0.0]), atol=1e-9)


def test_se3_compose_rotation_composes():
    """Two 90° Z-rotations compose to 180°."""
    r90 = Rotation.from_euler("z", np.pi / 2)
    _, rot_out = _se3_compose(np.zeros(3), r90, np.zeros(3), r90)
    expected = Rotation.from_euler("z", np.pi).as_matrix()
    np.testing.assert_allclose(rot_out.as_matrix(), expected, atol=1e-9)


# ---------------------------------------------------------------------------
# _solve_ik
# ---------------------------------------------------------------------------


def test_solve_ik_returns_6_joints(yam_robot):
    joints = _solve_ik(
        yam_robot, "link_6", np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.25, 0.0, 0.26])
    )
    assert joints.shape == (6,)


def test_solve_ik_joints_finite(yam_robot):
    joints = _solve_ik(
        yam_robot, "link_6", np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.25, 0.0, 0.26])
    )
    assert np.all(np.isfinite(joints)), f"IK returned non-finite joints: {joints}"


def test_solve_ik_joints_within_limits(yam_robot):
    """Solved joints must not violate URDF joint limits (with small tolerance)."""
    # Limits from i2rt/get_robot.py plus 0.15 buffer:
    limits_lower = np.array([-2.767, -0.15, -0.15, -1.72, -1.72, -2.24])
    limits_upper = np.array([3.28, 3.80, 3.28, 1.72, 1.72, 2.24])
    joints = _solve_ik(
        yam_robot, "link_6", np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.25, 0.0, 0.26])
    )
    assert np.all(joints >= limits_lower - 0.01), f"below lower limit: {joints}"
    assert np.all(joints <= limits_upper + 0.01), f"above upper limit: {joints}"


def test_solve_ik_consistent_on_same_target(yam_robot):
    """Identical IK queries must return identical results (determinism)."""
    wxyz = np.array([1.0, 0.0, 0.0, 0.0])
    pos = np.array([0.25, 0.0, 0.26])
    j1 = _solve_ik(yam_robot, "link_6", wxyz, pos)
    j2 = _solve_ik(yam_robot, "link_6", wxyz, pos)
    np.testing.assert_allclose(j1, j2, atol=1e-5)


# ---------------------------------------------------------------------------
# Quest3YamTeleop node (sim_mode)
# ---------------------------------------------------------------------------


def test_first_message_captures_reference(sim_node):
    """First wrist message should set the reference pose and not trigger IK."""
    # Reset state so this test is order-independent.
    sim_node._wrist_ref_pos = None
    sim_node._wrist_ref_rot = None

    msg = _pose_stamped(sim_node, 0.1, 0.2, 0.3)
    sim_node._wrist_callback(msg)

    np.testing.assert_allclose(sim_node._wrist_ref_pos, [0.1, 0.2, 0.3])


def test_zero_delta_does_not_crash(sim_node):
    """Sending the same pose as reference (delta=0) must complete without error."""
    # Ensure reference is set.
    if sim_node._wrist_ref_pos is None:
        sim_node._wrist_callback(_pose_stamped(sim_node, 0.1, 0.2, 0.3))

    ref = sim_node._wrist_ref_pos.copy()
    msg = _pose_stamped(sim_node, *ref)
    sim_node._wrist_callback(msg)  # must not raise


def test_out_of_workspace_does_not_crash(sim_node):
    """An extreme wrist displacement must be clamped and IK must still succeed."""
    # Ensure reference is set.
    if sim_node._wrist_ref_pos is None:
        sim_node._wrist_callback(_pose_stamped(sim_node, 0.1, 0.2, 0.3))

    home_snapshot = sim_node._home_ee_pos.copy()
    msg = _pose_stamped(sim_node, 999.0, 999.0, 999.0)
    sim_node._wrist_callback(msg)  # must not raise

    # Node home target must be unchanged (clamping only affects local ee_target).
    np.testing.assert_allclose(sim_node._home_ee_pos, home_snapshot)
