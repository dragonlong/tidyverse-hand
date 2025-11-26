import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from piper_sdk import (
    C_PiperInterface_V2,
    LogLevel,
    quat_convert_euler,
    euler_convert_quat,
)


MICRO_MM_PER_M = 1_000_000  # meters -> 0.001 mm integer units
MILLI_DEG_PER_RAD = 180_000 / np.pi  # radians -> 0.001 degrees
RAD_PER_MILLI_DEG = np.pi / 180_000
DEFAULT_SPEED_PERCENT = 100  # Matches vendor EndPose demo MotionCtrl_2 setting
DEFAULT_MOVE_MODE = 0x00  # Align with MotionCtrl_2 move_mode used in vendor demo
DEFAULT_GRIPPER_RANGE_M = (0.0, 0.06)
DEFAULT_GRIPPER_EFFORT = 1000 # x 0.001 Nm
ENABLE_TIMEOUT_SEC = 8.0
ENABLE_RETRY_SEC = 0.05


@dataclass
class PiperArmConfig:
    can_name: str = "can_piper"
    judge_flag: bool = False
    can_auto_init: bool = True
    dh_is_offset: int = 1
    start_sdk_joint_limit: bool = True
    start_sdk_gripper_limit: bool = True
    logger_level: LogLevel = LogLevel.INFO
    speed_percent: int = DEFAULT_SPEED_PERCENT
    move_mode: int = DEFAULT_MOVE_MODE
    gripper_range_m: Tuple[float, float] = DEFAULT_GRIPPER_RANGE_M
    gripper_effort: int = DEFAULT_GRIPPER_EFFORT
    verbose: bool = False


@dataclass
class PiperCommand:
    """Container capturing the exact low-level command sent to the Piper SDK."""

    position_units: Tuple[int, int, int]
    orientation_units: Tuple[int, int, int]
    gripper_units: Optional[int]
    speed_percent: int
    move_mode: int
    gripper_effort: int
    issued_at: float


class PiperArm:
    """Thin wrapper around `C_PiperInterface_V2` with metric-friendly helpers."""

    def __init__(
        self,
        *,
        can_name: str = "can_piper",
        judge_flag: bool = False,
        can_auto_init: bool = True,
        dh_is_offset: int = 1,
        start_sdk_joint_limit: bool = True,
        start_sdk_gripper_limit: bool = True,
        logger_level: LogLevel = LogLevel.INFO,
        speed_percent: int = DEFAULT_SPEED_PERCENT,
        move_mode: int = DEFAULT_MOVE_MODE,
        gripper_range_m: Tuple[float, float] = DEFAULT_GRIPPER_RANGE_M,
        gripper_effort: int = DEFAULT_GRIPPER_EFFORT,
        verbose: bool = False,
    ) -> None:
        self.config = PiperArmConfig(
            can_name=can_name,
            judge_flag=judge_flag,
            can_auto_init=can_auto_init,
            dh_is_offset=dh_is_offset,
            start_sdk_joint_limit=start_sdk_joint_limit,
            start_sdk_gripper_limit=start_sdk_gripper_limit,
            logger_level=logger_level,
            speed_percent=speed_percent,
            move_mode=move_mode,
            gripper_range_m=gripper_range_m,
            gripper_effort=gripper_effort,
            verbose=verbose,
        )

        self.can_name = self.config.can_name
        self.judge_flag = self.config.judge_flag
        self.can_auto_init = self.config.can_auto_init
        self.dh_is_offset = self.config.dh_is_offset
        self.start_sdk_joint_limit = self.config.start_sdk_joint_limit
        self.start_sdk_gripper_limit = self.config.start_sdk_gripper_limit
        self.logger_level = self.config.logger_level
        self.speed_percent = int(np.clip(self.config.speed_percent, 0, 100))
        self.move_mode = int(self.config.move_mode)
        self.gripper_range_m = self.config.gripper_range_m
        self.gripper_effort = DEFAULT_GRIPPER_EFFORT
        self._set_gripper_effort(self.config.gripper_effort)
        self.verbose = bool(self.config.verbose)

        self._client: Optional[C_PiperInterface_V2] = None
        self._connected = False
        self._lock = threading.RLock()
        self._last_command: Optional[PiperCommand] = None
        self.is_mit_mode = 0x00  # Piper firmware default

    # ----------------------------------------------------------------------------------
    # Lifecycle helpers
    # ----------------------------------------------------------------------------------
    def set_motion_profile(self, *, speed_percent: Optional[int] = None, move_mode: Optional[int] = None) -> None:
        """Update the cached motion profile parameters and push them to the arm if connected."""
        with self._lock:
            self._set_motion_profile(speed_percent=speed_percent, move_mode=move_mode, apply_now=True)

    def set_speed_percent(self, speed_percent: int) -> None:
        self.set_motion_profile(speed_percent=speed_percent)

    def set_move_mode(self, move_mode: int) -> None:
        self.set_motion_profile(move_mode=move_mode)

    def set_gripper_effort(self, gripper_effort: int) -> None:
        with self._lock:
            self._set_gripper_effort(gripper_effort)

    def set_verbose(self, verbose: bool) -> None:
        with self._lock:
            self.verbose = bool(verbose)

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._client = C_PiperInterface_V2(
                can_name=self.can_name,
                judge_flag=self.judge_flag,
                can_auto_init=self.can_auto_init,
                dh_is_offset=self.dh_is_offset,
                start_sdk_joint_limit=self.start_sdk_joint_limit,
                start_sdk_gripper_limit=self.start_sdk_gripper_limit,
                logger_level=self.logger_level,
                log_to_file=False,
                log_file_path=None,
            )
            self._client.ConnectPort()
            self._wait_for_enable()
            self._client.EnableArm()
            # Enable and zero the gripper controller before issuing motion commands
            self._client.GripperCtrl(0, self.gripper_effort, 0x02, 0x00)
            self._client.GripperCtrl(0, self.gripper_effort, 0x01, 0x00)
            self._apply_motion_mode()
            self._connected = True

    def _wait_for_enable(self) -> None:
        assert self._client is not None
        deadline = time.time() + ENABLE_TIMEOUT_SEC
        while time.time() < deadline:
            if self._client.EnablePiper():
                return
            time.sleep(ENABLE_RETRY_SEC)
        raise TimeoutError("Timed out waiting for Piper arm to enable")

    def _apply_motion_mode(self) -> None:
        assert self._client is not None
        self._client.ModeCtrl(
            ctrl_mode=0x01,
            move_mode=self.move_mode,
            move_spd_rate_ctrl=self.speed_percent,
            is_mit_mode=self.is_mit_mode,
        )

    def ensure_ready(self) -> None:
        if not self._connected:
            self.connect()

    def reset(self) -> None:
        """Prepare the arm for teleoperation. Homing is not implemented yet."""
        self.ensure_ready()
        # NOTE: Piper firmware-specific homing / retract sequences are not implemented.
        # Users should call vendor-provided commands externally if required.
        self.open_gripper()

    def close(self, *, disable_arm: bool = False) -> None:
        with self._lock:
            if not self._client:
                return
            if disable_arm:
                try:
                    self._client.DisableArm()
                except Exception:
                    pass
            try:
                self._client.DisconnectPort()
            except Exception:
                pass
            self._client = None
            self._connected = False

    # ----------------------------------------------------------------------------------
    # Command helpers
    # ----------------------------------------------------------------------------------
    def command_cartesian(
        self,
        arm_pos_m: Sequence[float],
        arm_quat_xyzw: Sequence[float],
        gripper_normalized: Optional[float],
        *,
        speed_percent: Optional[int] = None,
        move_mode: Optional[int] = None,
        gripper_effort: Optional[int] = None,
        verbose_override: Optional[bool] = None,
    ) -> PiperCommand:
        """Send a cartesian pose command (meters + quaternion) to the Piper arm."""
        if arm_pos_m is None:
            raise NotImplementedError("Cartesian position commands are required for PiperArm")
        if arm_quat_xyzw is None:
            raise NotImplementedError("Orientation control is not implemented without quaternions")

        self.ensure_ready()
        with self._lock:
            if speed_percent is not None or move_mode is not None:
                self._set_motion_profile(speed_percent=speed_percent, move_mode=move_mode, apply_now=True)
            if gripper_effort is not None:
                self._set_gripper_effort(gripper_effort)

            position_units, orientation_units = self._format_cartesian_command(arm_pos_m, arm_quat_xyzw)
            gripper_units = self._to_gripper_units(gripper_normalized)

            assert self._client is not None
            self._client.EndPoseCtrl(*position_units, *orientation_units)
            if gripper_units is not None:
                self._client.GripperCtrl(abs(gripper_units), self.gripper_effort, 0x01, 0x00)

            command = PiperCommand(
                position_units=position_units,
                orientation_units=orientation_units,
                gripper_units=gripper_units,
                speed_percent=self.speed_percent,
                move_mode=self.move_mode,
                gripper_effort=self.gripper_effort,
                issued_at=time.time(),
            )
            self._last_command = command

            if self._should_log(verbose_override):
                print(
                    "[PiperArm] Mode=0x%02X Speed=%d%% pos=%s rot=%s grip=%s effort=%d"
                    % (
                        self.move_mode,
                        self.speed_percent,
                        position_units,
                        orientation_units,
                        gripper_units,
                        self.gripper_effort,
                    )
                )

            return command

    def open_gripper(self) -> None:
        self.command_cartesian_with_gripper_only(1.0)

    def close_gripper(self) -> None:
        self.command_cartesian_with_gripper_only(0.0)

    def command_cartesian_with_gripper_only(
        self,
        gripper_normalized: float,
        *,
        gripper_effort: Optional[int] = None,
        verbose_override: Optional[bool] = None,
    ) -> None:
        """Only send a gripper command (meters normalized between [0, 1])."""
        self.ensure_ready()
        with self._lock:
            if gripper_effort is not None:
                self._set_gripper_effort(gripper_effort)
            units = self._to_gripper_units(gripper_normalized)
            if units is None:
                return
            assert self._client is not None
            self._client.GripperCtrl(abs(units), self.gripper_effort, 0x01, 0x00)
            if self._should_log(verbose_override):
                print(f"[PiperArm] GripperCtrl units={units} effort={self.gripper_effort}")

    # ----------------------------------------------------------------------------------
    # Feedback helpers
    # ----------------------------------------------------------------------------------
    def get_state(self) -> dict:
        self.ensure_ready()
        assert self._client is not None
        pose_msg = self._client.GetArmEndPoseMsgs()
        grip_msg = self._client.GetArmGripperMsgs()
        if pose_msg is None:
            raise RuntimeError("Piper SDK returned no end-effector pose data")

        pose_axes = getattr(pose_msg, "end_pose", pose_msg)
        pose_units = (
            int(pose_axes.X_axis),
            int(pose_axes.Y_axis),
            int(pose_axes.Z_axis),
            int(pose_axes.RX_axis),
            int(pose_axes.RY_axis),
            int(pose_axes.RZ_axis),
        )

        arm_pos = np.array(pose_units[:3], dtype=np.float64) / MICRO_MM_PER_M
        euler_mdeg = np.array(pose_units[3:], dtype=np.float64)
        euler_rad = euler_mdeg * RAD_PER_MILLI_DEG
        arm_quat = np.array(euler_convert_quat(*euler_rad), dtype=np.float64)

        gripper_units = None
        gripper_effort_feedback = None
        gripper_status_code = None
        if grip_msg:
            grip_state = getattr(grip_msg, "gripper_state", grip_msg)
            gripper_units = int(grip_state.grippers_angle)
            gripper_effort_feedback = int(grip_state.grippers_effort)
            gripper_status_code = int(grip_state.status_code)
        gripper_norm = self._from_gripper_units(gripper_units)

        state = {
            "arm_pos": arm_pos,
            "arm_quat": arm_quat,
            "gripper_pos": np.array([gripper_norm], dtype=np.float64),
            "arm_pose_units": np.array(pose_units, dtype=np.int64),
            "arm_euler_mdeg": euler_mdeg,
            "piper_speed_percent": float(self.speed_percent),
            "piper_move_mode": int(self.move_mode),
            "piper_gripper_effort": int(self.gripper_effort),
            "gripper_angle_units": gripper_units,
            "gripper_effort_units": gripper_effort_feedback,
            "gripper_status_code": gripper_status_code,
        }
        if self._last_command is not None:
            state["piper_last_command"] = self._last_command
        return state

    # ----------------------------------------------------------------------------------
    # Unit conversions
    # ----------------------------------------------------------------------------------
    def _to_position_units(self, arm_pos_m: Sequence[float]) -> Tuple[int, int, int]:
        vec = np.asarray(arm_pos_m, dtype=np.float64)
        if vec.shape != (3,):
            raise ValueError(f"arm_pos must be shape (3,), got {vec.shape}")
        return tuple(int(np.round(val)) for val in vec * MICRO_MM_PER_M)

    def _format_cartesian_command(
        self, arm_pos_m: Sequence[float], arm_quat_xyzw: Sequence[float]
    ) -> Tuple[Tuple[int, int, int], Tuple[int, int, int]]:
        return self._to_position_units(arm_pos_m), self._to_orientation_units(arm_quat_xyzw)

    def _to_orientation_units(self, arm_quat_xyzw: Sequence[float]) -> Tuple[int, int, int]:
        quat = np.asarray(arm_quat_xyzw, dtype=np.float64)
        if quat.shape != (4,):
            raise ValueError(f"arm_quat must be shape (4,), got {quat.shape}")
        quat = self._normalize_quaternion(quat)
        roll, pitch, yaw = quat_convert_euler(quat[0], quat[1], quat[2], quat[3])
        milli_deg = np.degrees([roll, pitch, yaw]) * 1000.0
        return tuple(int(np.round(val)) for val in milli_deg)

    def _to_gripper_units(self, gripper_normalized: Optional[float]) -> Optional[int]:
        if gripper_normalized is None:
            return None
        value = float(gripper_normalized)
        if np.isnan(value):
            return None
        value = float(np.clip(value, 0.0, 1.0))
        g_min, g_max = self.gripper_range_m
        gripper_m = g_min + value * (g_max - g_min)
        return int(np.round(gripper_m * MICRO_MM_PER_M))

    def _from_gripper_units(self, units: Optional[int]) -> float:
        if units is None:
            return np.nan
        g_min, g_max = self.gripper_range_m
        span = max(g_max - g_min, 1e-6)
        gripper_m = units / MICRO_MM_PER_M
        return float(np.clip((gripper_m - g_min) / span, 0.0, 1.0))

    def _should_log(self, override: Optional[bool]) -> bool:
        return self.verbose if override is None else bool(override)

    def _normalize_quaternion(self, quat: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(quat)
        if not np.isfinite(norm) or norm < 1e-9:
            raise ValueError(f"arm_quat must have non-zero norm, got {norm}")
        return quat / norm

    def _set_motion_profile(
        self,
        *,
        speed_percent: Optional[int] = None,
        move_mode: Optional[int] = None,
        apply_now: bool = False,
    ) -> bool:
        changed = False
        if speed_percent is not None:
            clamped = int(np.clip(speed_percent, 0, 100))
            if clamped != self.speed_percent:
                self.speed_percent = clamped
                changed = True
        if move_mode is not None:
            move_mode_int = int(move_mode)
            if move_mode_int != self.move_mode:
                self.move_mode = move_mode_int
                changed = True
        if (
            changed
            and apply_now
            and self._connected
            and self._client is not None
        ):
            self._apply_motion_mode()
        return changed

    def _set_gripper_effort(self, gripper_effort: Optional[int]) -> None:
        if gripper_effort is None:
            return
        value = int(np.clip(gripper_effort, 0, 5000))
        self.gripper_effort = value

    @property
    def last_command(self) -> Optional[PiperCommand]:
        with self._lock:
            return self._last_command

