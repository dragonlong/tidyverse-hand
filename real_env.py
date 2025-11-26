# Author: Jimmy Wu
# Date: October 2024

from cameras import LogitechCamera
from constants import BASE_RPC_HOST, BASE_RPC_PORT, ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
from constants import BASE_CAMERA_SERIAL, WRIST_CAMERA_SERIAL
from arm_server_piper import ArmManager
from base_server import BaseManager

class RealEnv:
    def __init__(self):
        # RPC server connection for base
        base_manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
        try:
            base_manager.connect()
        except ConnectionRefusedError as e:
            raise Exception('Could not connect to base RPC server, is base_server.py running?') from e

        # RPC server connection for arm
        arm_manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        try:
            arm_manager.connect()
        except ConnectionRefusedError as e:
            raise Exception('Could not connect to arm RPC server, is arm_server.py running?') from e

        # RPC proxy objects
        self.base = base_manager.Base(max_vel=(0.5, 0.5, 1.57), max_accel=(0.5, 0.5, 1.57))
        self.arm = arm_manager.Arm()

        # Cameras
        self.base_camera = None
        try:
            self.base_camera = LogitechCamera(BASE_CAMERA_SERIAL)
        except Exception as e:
            print(f"Warning: Could not initialize base camera: {e}")

        self.wrist_camera = None
        try:
            self.wrist_camera = LogitechCamera(WRIST_CAMERA_SERIAL)
        except Exception as e:
            print(f"Warning: Could not initialize wrist camera: {e}")

    def get_obs(self):
        obs = {}
        obs.update(self.base.get_state())
        obs.update(self.arm.get_state())
        obs['base_image'] = self.base_camera.get_image() if self.base_camera else None
        obs['wrist_image'] = self.wrist_camera.get_image() if self.wrist_camera else None
        return obs

    def reset(self):
        print('Resetting base...')
        self.base.reset()

        print('Resetting arm...')
        self.arm.reset()

        print('Robot has been reset')

    def step(self, action):
        # Note: We intentionally do not return obs here to prevent the policy from using outdated data
        self.base.execute_action(action)  # Non-blocking
        self.arm.execute_action(action)   # Non-blocking

    def close(self):
        self.base.close()
        self.arm.close()
        if self.base_camera:
            self.base_camera.close()
        if self.wrist_camera:
            self.wrist_camera.close()

if __name__ == '__main__':
    import time
    import numpy as np
    from constants import POLICY_CONTROL_PERIOD
    env = RealEnv()
    try:
        while True:
            env.reset()
            for _ in range(100):
                pose_candidates = [
                    np.array([0.057, 0.0, 0.200]),
                    np.array([0.087, 0.0, 0.255]),
                ]
                demo_arm_pos = pose_candidates[_ % len(pose_candidates)] + 0.005 * (np.random.rand(3) - 0.5)
                quat_candidates = [
                    np.array([0.0, np.sqrt(0.5), 0.0, np.sqrt(0.5)]),  # 0°, 90°, 0°
                    np.array([0.0, np.sin(np.deg2rad(60.0) / 2), 0.0, np.cos(np.deg2rad(60.0) / 2)]),  # 0°, 120°, 0°
                ]
                demo_arm_quat = quat_candidates[_ % len(quat_candidates)]
                action = {
                    'base_pose': 0.1 * np.random.rand(3) - np.array([0.05, 0.05, 0.05]),
                    'arm_pos': demo_arm_pos,
                    'arm_quat': demo_arm_quat,
                    'gripper_pos': np.random.rand(1),
                }
                env.step(action)
                obs = env.get_obs()
                def _summarize_value(value):
                    if hasattr(value, "ndim") and hasattr(value, "shape"):
                        if value.ndim == 3:
                            return value.shape
                        return value
                    return value
                print([(k, _summarize_value(v)) for (k, v) in obs.items()])
                time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
    finally:
        env.close()
