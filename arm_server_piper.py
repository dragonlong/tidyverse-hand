import argparse
import signal
import threading
import time
from multiprocessing.managers import BaseManager as MPBaseManager
from typing import Any, Dict, Optional, Sequence

import numpy as np
from piper_sdk import euler_convert_quat

from constants import ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
from piper import DEFAULT_MOVE_MODE, DEFAULT_SPEED_PERCENT, PiperArm

RESET_SETTLE_SEC = 0.5
DEMO_SWITCH_PERIOD = 200  # Loop iterations before toggling pose
DEMO_SLEEP_SEC = 0.01


class Arm:
    """High-level RPC controller for the Piper arm (drop-in replacement for Kinova Arm)."""

    def __init__(
        self,
        *,
        speed_percent: int = DEFAULT_SPEED_PERCENT,
        move_mode: int = DEFAULT_MOVE_MODE,
        verbose: bool = True,
    ) -> None:
        self._default_speed_percent = speed_percent
        self._default_move_mode = move_mode
        self.arm = PiperArm(
            speed_percent=speed_percent,
            move_mode=move_mode,
            verbose=verbose,
        )
        # Attributes kept for compatibility with the Kinova controller interface.
        self.controller = None
        self.command_queue = None
        self.ik_solver = None

    def reset(self) -> None:
        """Bring the Piper arm into a known state and ensure motion profile defaults."""
        self.arm.reset()
        self.arm.set_motion_profile(
            speed_percent=self._default_speed_percent,
            move_mode=self._default_move_mode,
        )
        time.sleep(RESET_SETTLE_SEC)

    def execute_action(self, action: Dict[str, Any]) -> None:
        """Execute a high-level action dictionary (same schema as Kinova Arm)."""
        # Note: The underlying Piper control units are 0.001mm (position) and 0.001deg (orientation).
        # However, this high-level API expects standard units which are automatically converted:
        # - arm_pos: [x, y, z] in meters
        # - arm_quat: [x, y, z, w] quaternion (normalized)
        # - gripper_pos: [0.0 (closed) ... 1.0 (open)] normalized
        if action is None:
            raise ValueError("Action dictionary cannot be None")

        arm_pos = self._coerce_vector(action.get("arm_pos"), 3, "arm_pos")
        arm_quat = self._coerce_vector(action.get("arm_quat"), 4, "arm_quat")
        if arm_pos is None or arm_quat is None:
            raise NotImplementedError("PiperArm requires absolute arm_pos and arm_quat commands")

        gripper_pos = self._coerce_gripper_value(action.get("gripper_pos"))
        self.arm.command_cartesian(
            arm_pos,
            arm_quat,
            gripper_pos,
            speed_percent=action.get("piper_speed_percent"),
            move_mode=action.get("piper_move_mode"),
            gripper_effort=action.get("piper_gripper_effort"),
            verbose_override=action.get("piper_verbose"),
        )

    def get_state(self) -> Dict[str, Any]:
        """Return the Piper arm observation dictionary."""
        return self.arm.get_state()

    def close(self) -> None:
        self.arm.close()

    @staticmethod
    def _coerce_vector(raw_value: Optional[Sequence[float]], expected_size: int, name: str) -> Optional[np.ndarray]:
        if raw_value is None:
            return None
        vec = np.asarray(raw_value, dtype=np.float64)
        if vec.shape != (expected_size,):
            raise ValueError(f"{name} must have shape ({expected_size},), got {vec.shape}")
        return vec

    @staticmethod
    def _coerce_gripper_value(raw_value: Any) -> Optional[float]:
        if raw_value is None:
            return None
        if isinstance(raw_value, np.ndarray):
            if raw_value.size == 0:
                return None
            return float(raw_value.reshape(-1)[0])
        if isinstance(raw_value, (list, tuple)):
            if len(raw_value) == 0:
                return None
            return float(raw_value[0])
        return float(raw_value)


class ArmManager(MPBaseManager):
    pass


ArmManager.register("Arm", Arm)


class _GracefulShutdown:
    """Signal-aware helper to ensure we leave the robot in a known state."""

    def __init__(self, *signals_to_handle: int) -> None:
        self._stop_event = threading.Event()
        self._signals = signals_to_handle or (signal.SIGINT, signal.SIGTERM)
        for sig in self._signals:
            signal.signal(sig, self._handle_signal)

    def _handle_signal(self, signum, _frame) -> None:
        print(f"\nReceived signal {signum}, requesting shutdown...")
        self._stop_event.set()

    def should_stop(self) -> bool:
        return self._stop_event.is_set()

    def stop(self) -> None:
        self._stop_event.set()


def _quat_from_degrees(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    return np.asarray(euler_convert_quat(roll, pitch, yaw), dtype=np.float64)


def _vector3_from_arg(text: str, name: str) -> np.ndarray:
    try:
        parts = [float(x) for x in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must be comma-separated floats") from exc
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"{name} must contain exactly 3 values")
    return np.array(parts, dtype=np.float64)


def _vector2_from_arg(text: str, name: str) -> np.ndarray:
    try:
        parts = [float(x) for x in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must be comma-separated floats") from exc
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"{name} must contain exactly 2 values")
    return np.array(parts, dtype=np.float64)


def run_demo_end_pose(
    *,
    verbose: bool = True,
    speed_percent: int = DEFAULT_SPEED_PERCENT,
    switch_period: int = DEMO_SWITCH_PERIOD,
    sleep_sec: float = DEMO_SLEEP_SEC,
    pose_a_mm: Sequence[float] = (57.0, 0.0, 215.0),
    pose_b_mm: Sequence[float] = (87.0, 0.0, 260.0),
    orientation_deg: Sequence[float] = (0.0, 85.0, 0.0),
    gripper_range: Sequence[float] = (0.0, 1.0),
    gripper_switch_period: Optional[int] = None,
) -> None:
    """Replicate the vendor end-pose demo using the PiperArm helper."""
    speed_percent = int(np.clip(speed_percent, 0, 100))
    pose_a_m = np.asarray(pose_a_mm, dtype=np.float64) / 1000.0
    pose_b_m = np.asarray(pose_b_mm, dtype=np.float64) / 1000.0
    orientation_quat = _quat_from_degrees(*orientation_deg)
    gripper_min = float(np.clip(gripper_range[0], 0.0, 1.0))
    gripper_max = float(np.clip(gripper_range[1], 0.0, 1.0))
    if gripper_min > gripper_max:
        gripper_min, gripper_max = gripper_max, gripper_min
    gripper_period = max(1, gripper_switch_period or switch_period)

    arm = PiperArm(verbose=verbose)
    killer = _GracefulShutdown()
    starting_speed = arm.speed_percent
    starting_move_mode = arm.move_mode
    poses_m = [pose_a_m, pose_b_m]
    index = 0
    step = 0
    print(
        "Starting Piper end-pose demo. Press Ctrl+C to stop.\n"
        f"Pose #1 (mm): {pose_a_mm}\nPose #2 (mm): {pose_b_mm}\n"
        f"Speed percent: {speed_percent}%  |  Sleep: {sleep_sec:.3f}s  |  Switch period: {switch_period} | Gripper period: {gripper_period}"
    )
    # breakpoint()
    try:
        gripper_open = True
        arm.reset()
        arm.set_motion_profile(speed_percent=speed_percent, move_mode=DEFAULT_MOVE_MODE)
        while not killer.should_stop():
            position = poses_m[index]
            arm.command_cartesian(
                position,
                orientation_quat,
                gripper_normalized=gripper_max if gripper_open else gripper_min,
                speed_percent=speed_percent,
                move_mode=DEFAULT_MOVE_MODE,
            )
            time.sleep(sleep_sec)
            step += 1
            if step % max(1, switch_period) == 0:
                index = (index + 1) % len(poses_m)
                pose_mm = poses_m[index] * 1000.0
                print(f"Switched to pose #{index + 1}: {pose_mm.tolist()} mm")
            if step % gripper_period == 0:
                gripper_open = not gripper_open
                print(
                    f"Gripper {'open' if gripper_open else 'close'} -> "
                    f"{(gripper_max if gripper_open else gripper_min):.3f} (norm)"
                )
    except KeyboardInterrupt:
        print("Demo interrupted by user.")
    finally:
        # Restore the motion profile so the arm state is consistent regardless of exit path.
        try:
            arm.set_motion_profile(speed_percent=starting_speed, move_mode=starting_move_mode)
        except Exception:
            pass
        arm.close()


def run_arm_server() -> None:
    manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    server = manager.get_server()
    print(f"Piper arm manager server started at {ARM_RPC_HOST}:{ARM_RPC_PORT}")
    server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description="Piper arm RPC server and demos")
    parser.add_argument(
        "--mode",
        choices=("server", "demo"),
        default="server",
        help="Run the RPC server (default) or the built-in end-pose demo",
    )
    parser.add_argument(
        "--demo-verbose",   
        action="store_true",
        help="Enable verbose logging for the Piper demo",
    )
    parser.add_argument(
        "--demo-speed-percent",
        type=int,
        default=DEFAULT_SPEED_PERCENT,
        help="Speed percentage for demo EndPoseCtrl commands (0-100)",
    )   
    parser.add_argument(
        "--demo-switch-period",
        type=int,
        default=DEMO_SWITCH_PERIOD,
        help="Number of iterations before toggling demo pose",
    )
    parser.add_argument(
        "--demo-sleep-sec",
        type=float,
        default=DEMO_SLEEP_SEC,
        help="Sleep duration between demo commands (seconds)",
    )
    parser.add_argument(
        "--demo-pose-a-mm",
        default="57.0,0.0,215.0",
        help="Pose #1 in millimetres: X,Y,Z (default: 57,0,215)",
    )
    parser.add_argument(
        "--demo-pose-b-mm",
        default="87.0,0.0,260.0",
        help="Pose #2 in millimetres: X,Y,Z (default: 87,0,260)",
    )
    parser.add_argument(
        "--demo-orientation-deg",
        default="0.0,85.0,0.0",
        help="Fixed orientation in degrees: roll,pitch,yaw (default: 0,85,0)",
    )
    parser.add_argument(
        "--demo-gripper-range",
        default="0.0,1.0",
        help="Gripper range normalized: min,max (default: 0.0,1.0)",
    )
    parser.add_argument(
        "--demo-gripper-switch-period",
        type=int,
        default=None,
        help="Iterations before toggling gripper; defaults to switch period",
    )
    args = parser.parse_args()

    if args.mode == "demo":
        pose_a = _vector3_from_arg(args.demo_pose_a_mm, "demo-pose-a-mm")
        pose_b = _vector3_from_arg(args.demo_pose_b_mm, "demo-pose-b-mm")
        orientation = _vector3_from_arg(args.demo_orientation_deg, "demo-orientation-deg")
        gripper_range = _vector2_from_arg(args.demo_gripper_range, "demo-gripper-range")
        run_demo_end_pose(
            verbose=args.demo_verbose,
            speed_percent=args.demo_speed_percent,
            switch_period=args.demo_switch_period,
            sleep_sec=args.demo_sleep_sec,
            pose_a_mm=pose_a,
            pose_b_mm=pose_b,
            orientation_deg=orientation,
            gripper_range=gripper_range,
            gripper_switch_period=args.demo_gripper_switch_period,
        )
    else:
        run_arm_server()


if __name__ == "__main__":
    main()
