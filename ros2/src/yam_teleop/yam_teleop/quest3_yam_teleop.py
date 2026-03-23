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

"""Quest 3 right wrist pose → YAM arm IK delta teleoperation.

Subscribes to ``hands/right/wrist_pose`` (geometry_msgs/PoseStamped, FLU world
frame) and maps wrist motion relative to a captured reference pose into YAM arm
end-effector targets, solved via pyroki IK and commanded to hardware.

Control scheme (delta / relative):
  1. First valid message after the hand is stable captures the reference wrist
     pose.  The arm is assumed to be at its resting zero-joint home at this
     moment, so hand position == arm home == zero delta.
  2. Each subsequent message computes:
       delta_pos = (wrist_pos - wrist_ref_pos) * position_scale
       delta_rot = wrist_ref_rot.inv() * wrist_rot
  3. EE target = home_ee_pose ⊕ (delta_pos, delta_rot)
  4. Workspace-clamped EE target → pyroki IK → 6 joint angles → /joint_states

FLU world frame: X=Forward, Y=Left, Z=Up (Quest Unity-left → FLU conversion
is done upstream by hand_tracking_sdk_ros2).  The YAM arm base frame is assumed
co-oriented with the world frame for this delta scheme (no explicit rotation
transform needed; add ``arm_rotation_rpy`` parameter if misaligned).

Viser web viewer is available at http://localhost:<viser_port> (default 8080)
and shows the URDF arm pose updated in real-time from IK solutions.
"""

import os
import time
import threading

import jax
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import yourdfpy
import viser
import viser.extras
import pyroki as pk
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls

from robot_arms.yam_arm import YamArm


# ---------------------------------------------------------------------------
# IK solver — replicated from robots_realtime pyroki_snippets/_solve_ik.py
# ---------------------------------------------------------------------------

@jdc.jit
def _solve_ik_jax(
    robot: pk.Robot,
    target_link_index: jax.Array,
    target_wxyz: jax.Array,
    target_position: jax.Array,
) -> jax.Array:
    joint_var = robot.joint_var_cls(0)
    variables = [joint_var]
    costs = [
        pk.costs.pose_cost_analytic_jac(
            robot,
            joint_var,
            jaxlie.SE3.from_rotation_and_translation(
                jaxlie.SO3(target_wxyz), target_position
            ),
            target_link_index,
            pos_weight=50.0,
            ori_weight=10.0,
        ),
        pk.costs.limit_constraint(robot, joint_var),
    ]
    sol = (
        jaxls.LeastSquaresProblem(costs=costs, variables=variables)
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
        )
    )
    return sol[joint_var]


def _solve_ik(
    robot: pk.Robot,
    target_link_name: str,
    target_wxyz_w_first: np.ndarray,
    target_position: np.ndarray,
) -> np.ndarray:
    """Solve IK for ``target_link_name``.

    Args:
        target_wxyz_w_first: quaternion [w, x, y, z].
        target_position: [x, y, z] in robot base frame.

    Returns:
        Joint angles shape (6,).
    """
    assert target_wxyz_w_first.shape == (4,) and target_position.shape == (3,)
    idx = robot.links.names.index(target_link_name)
    cfg = _solve_ik_jax(
        robot,
        jnp.array(idx),
        jnp.array(target_wxyz_w_first),
        jnp.array(target_position),
    )
    assert cfg.shape == (robot.joints.num_actuated_joints,)
    return np.array(cfg)


# ---------------------------------------------------------------------------
# SE3 helper
# ---------------------------------------------------------------------------

def _se3_compose(
    pos_a: np.ndarray,
    rot_a: Rotation,
    pos_b_in_a: np.ndarray,
    rot_b: Rotation,
):
    """Compose two SE3 transforms: T_result = T_a @ T_b."""
    return pos_a + rot_a.apply(pos_b_in_a), rot_a * rot_b


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class Quest3YamTeleop(Node):
    """Maps Quest 3 right wrist pose to YAM arm via delta IK teleoperation."""

    # Fixed TCP offset from link_6 origin to gripper tip (from yam_pyroki.py).
    _TCP_OFFSET_POS = np.array([0.0, 0.04, -0.13])
    _TCP_OFFSET_ROT = Rotation.identity()

    def __init__(self, **kwargs) -> None:
        super().__init__("quest3_yam_teleop", **kwargs)

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("sim_mode", False)
        self.declare_parameter("can_num", 0)
        self.declare_parameter(
            "urdf_path",
            "/home/dragonx/tetheria/i2rt/i2rt/robot_models/yam/yam.urdf",
        )
        self.declare_parameter("position_scale", 1.0)
        self.declare_parameter("home_ee_xyz", [0.25, 0.0, 0.26])
        self.declare_parameter("home_ee_rpy", [1.5708, 0.0, 1.5708])
        self.declare_parameter("workspace_min", [-0.1, -0.5, 0.0])
        self.declare_parameter("workspace_max", [0.6, 0.5, 0.7])
        self.declare_parameter("viser_port", 8080)

        sim_mode: bool = self.get_parameter("sim_mode").value
        can_num: int = self.get_parameter("can_num").value
        urdf_path: str = self.get_parameter("urdf_path").value
        self._position_scale: float = self.get_parameter("position_scale").value
        home_ee_xyz = np.array(self.get_parameter("home_ee_xyz").value)
        home_ee_rpy = list(self.get_parameter("home_ee_rpy").value)
        self._workspace_min = np.array(self.get_parameter("workspace_min").value)
        self._workspace_max = np.array(self.get_parameter("workspace_max").value)
        viser_port: int = self.get_parameter("viser_port").value

        # ── IK robot model ──────────────────────────────────────────────────
        mesh_dir = os.path.join(os.path.dirname(urdf_path), "assets")
        self._urdf = yourdfpy.URDF.load(urdf_path, mesh_dir=mesh_dir)
        self._robot = pk.Robot.from_urdf(self._urdf)
        self.get_logger().info(
            f"Loaded YAM URDF: {urdf_path} "
            f"({self._robot.joints.num_actuated_joints} actuated joints)"
        )

        # ── Home EE target (handle + TCP offset composed) ───────────────────
        home_handle_rot = Rotation.from_euler("xyz", home_ee_rpy)
        self._home_ee_pos, self._home_ee_rot = _se3_compose(
            home_ee_xyz, home_handle_rot,
            self._TCP_OFFSET_POS, self._TCP_OFFSET_ROT,
        )

        # ── Teleop state (lock guards ref reset from viser thread) ───────────
        self._ref_lock = threading.Lock()
        self._wrist_ref_pos: np.ndarray | None = None
        self._wrist_ref_rot: Rotation | None = None

        # ── Latest IK result for visualizer (written by ROS cb, read by vis) ─
        self._vis_lock = threading.Lock()
        self._vis_joints: np.ndarray | None = None
        self._vis_ee_pos: np.ndarray | None = None
        self._vis_ee_wxyz: np.ndarray | None = None  # [w, x, y, z]

        # ── Hardware ─────────────────────────────────────────────────────────
        self._arm: YamArm | None = None
        if not sim_mode:
            self._arm = YamArm(can_num=str(can_num))
            self._arm.connect()
            self._arm.move_to_home_position()
            self.get_logger().info("YAM arm connected and homed.")
        else:
            self.get_logger().info("SIM MODE: hardware commands will be logged only.")

        # ── Viser web viewer ─────────────────────────────────────────────────
        self._setup_viser(viser_port)

        # ── ROS interfaces ───────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseStamped,
            "hands/right/wrist_pose",
            self._wrist_callback,
            sensor_qos,
        )
        self._joint_pub = self.create_publisher(JointState, "joint_states", 10)

        # ── JIT warmup ───────────────────────────────────────────────────────
        threading.Thread(target=self._warmup_ik, daemon=True).start()

        self.get_logger().info(
            f"Quest3YamTeleop started (sim_mode={sim_mode}, can_num={can_num}). "
            "Waiting for first wrist pose to capture reference..."
        )

    # ── Viser setup ──────────────────────────────────────────────────────────

    def _setup_viser(self, port: int) -> None:
        self._viser = viser.ViserServer(port=port)

        # Scene: base frame + URDF robot + ground grid
        self._viser.scene.add_frame("/base", show_axes=False)
        self._urdf_vis = viser.extras.ViserUrdf(
            self._viser, self._urdf, root_node_name="/base"
        )
        self._viser.scene.add_grid("ground", width=2, height=2, cell_size=0.1)

        # EE target frame (axes show where IK is targeting)
        self._ee_frame = self._viser.scene.add_frame(
            "/ee_target",
            axes_length=0.08,
            axes_radius=0.005,
            origin_radius=0.01,
        )
        # Initialise at home EE position
        home_qxyzw = self._home_ee_rot.as_quat()
        self._ee_frame.position = tuple(self._home_ee_pos)
        self._ee_frame.wxyz = (home_qxyzw[3], home_qxyzw[0], home_qxyzw[1], home_qxyzw[2])

        # GUI
        self._gui_status = self._viser.gui.add_markdown(
            "**Status:** Waiting for Quest 3 stream…"
        )
        self._gui_ee_x = self._viser.gui.add_number("EE X (m)", initial_value=self._home_ee_pos[0], disabled=True)
        self._gui_ee_y = self._viser.gui.add_number("EE Y (m)", initial_value=self._home_ee_pos[1], disabled=True)
        self._gui_ee_z = self._viser.gui.add_number("EE Z (m)", initial_value=self._home_ee_pos[2], disabled=True)
        self._gui_timing = self._viser.gui.add_number("IK time (ms)", initial_value=0.0, disabled=True)
        self._reset_btn = self._viser.gui.add_button("Reset Reference Pose")

        @self._reset_btn.on_click
        def _(_) -> None:
            with self._ref_lock:
                self._wrist_ref_pos = None
                self._wrist_ref_rot = None
            self._gui_status.content = (
                "**Status:** Reference cleared — hold hand still to re-capture."
            )
            self.get_logger().info("Reference pose reset via Viser button.")

        # Start visualizer update loop in daemon thread
        threading.Thread(target=self._vis_loop, daemon=True).start()
        self.get_logger().info(f"Viser web viewer: http://localhost:{port}")

    def _vis_loop(self) -> None:
        """30 Hz loop: push latest IK result into the Viser scene."""
        while True:
            with self._vis_lock:
                joints = self._vis_joints
                ee_pos = self._vis_ee_pos
                ee_wxyz = self._vis_ee_wxyz

            if joints is not None:
                self._urdf_vis.update_cfg(joints)

            if ee_pos is not None and ee_wxyz is not None:
                self._ee_frame.position = tuple(ee_pos)
                self._ee_frame.wxyz = tuple(ee_wxyz)
                self._gui_ee_x.value = float(ee_pos[0])
                self._gui_ee_y.value = float(ee_pos[1])
                self._gui_ee_z.value = float(ee_pos[2])

            time.sleep(1.0 / 30.0)

    # ── IK warmup ────────────────────────────────────────────────────────────

    def _warmup_ik(self) -> None:
        dummy_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
        _solve_ik(self._robot, "link_6", dummy_wxyz, self._home_ee_pos)
        self.get_logger().info("IK JIT warmup complete.")
        self._gui_status.content = (
            "**Status:** IK ready. Hold right hand in neutral position, "
            "then stay still — first pose will be captured as reference (= arm home)."
        )

    # ── Wrist callback ────────────────────────────────────────────────────────

    def _wrist_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        wrist_pos = np.array([p.x, p.y, p.z])
        wrist_rot = Rotation.from_quat([q.x, q.y, q.z, q.w])  # scipy: [x,y,z,w]

        # Capture reference on first message (arm is at home / zero joints).
        with self._ref_lock:
            ref_pos = self._wrist_ref_pos
            ref_rot = self._wrist_ref_rot

        if ref_pos is None:
            with self._ref_lock:
                self._wrist_ref_pos = wrist_pos.copy()
                self._wrist_ref_rot = wrist_rot
            self._gui_status.content = (
                "**Status:** ✓ Reference captured — teleop active. "
                "Move hand to drive arm."
            )
            self.get_logger().info("Reference wrist pose captured. Teleop active.")
            return

        # Delta in Quest world (FLU) frame.
        delta_pos = (wrist_pos - ref_pos) * self._position_scale
        delta_rot = ref_rot.inv() * wrist_rot

        # Apply delta to home EE target.
        ee_target_pos = self._home_ee_pos + delta_pos
        ee_target_rot = self._home_ee_rot * delta_rot

        # Workspace clamp.
        ee_target_pos = np.clip(ee_target_pos, self._workspace_min, self._workspace_max)

        # Convert to [w,x,y,z] for pyroki.
        qxyzw = ee_target_rot.as_quat()
        target_wxyz = np.array([qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]])

        # IK solve (timed for GUI display).
        t0 = time.monotonic()
        joint_angles = _solve_ik(self._robot, "link_6", target_wxyz, ee_target_pos)
        ik_ms = (time.monotonic() - t0) * 1000.0
        self._gui_timing.value = round(0.95 * self._gui_timing.value + 0.05 * ik_ms, 2)

        # Cache for visualizer thread.
        with self._vis_lock:
            self._vis_joints = joint_angles
            self._vis_ee_pos = ee_target_pos
            self._vis_ee_wxyz = target_wxyz

        # Publish /joint_states (consumed by yam_gello_controller on mobile PC).
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint_{i + 1}" for i in range(len(joint_angles))]
        js.position = joint_angles.tolist()
        self._joint_pub.publish(js)

        # Command hardware or log.
        if self._arm is not None:
            self._arm.move_j(joint_angles.tolist())
        else:
            self.get_logger().debug(
                f"[SIM] joint_angles={np.round(joint_angles, 3).tolist()}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Quest3YamTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
