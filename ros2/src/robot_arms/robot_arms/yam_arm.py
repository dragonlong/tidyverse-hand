#!/usr/bin/env python3
from typing import List, Optional
from robot_arms.robot_arm import RobotArm

import pyudev, subprocess
from i2rt.robots.utils import GripperType
from i2rt.robots.get_robot import get_yam_robot

can_num_to_serial = {
    0: "004E003C594E501820313332",
}

class YamArm(RobotArm):
    def __init__(self, can_num: str):
        self.can_port = self.setup_can(int(can_num))
        self.arm = None

    def connect(self) -> None:
        try:
            self.arm = get_yam_robot(
                channel=self.can_port,
                gripper_type=GripperType.NO_GRIPPER, ## Gripper: NO_GRIPPER, LINEAR_4310
                zero_gravity_mode=False,
            )
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Yam Arm on {self.can_port}: {e}")
    
    def disconnect(self) -> None:
        pass

    def move_j(
        self,
        joint_positions: List[float],
        velocity: Optional[float] = None,
        blocking: bool = False
    ) -> None:
        if self.arm is None:
            raise RuntimeError("Yam Arm is not connected. Call connect() first.")
        
        self.arm.command_joint_pos(joint_positions)

    def move_to_home_position(
        self,
        velocity: Optional[float] = None,
        blocking: bool = True
    ) -> None:
        if self.arm is None:
            raise RuntimeError("Yam Arm is not connected. Call connect() first.")
        return self.arm.command_joint_pos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    def move_l(self, pose: List[float], velocity: float | None = None, blocking: bool = False) -> None:
        raise NotImplementedError("Not implemented for Yam Arm")

    def get_joint_positions(self) -> List[float]:
        return self.arm.get_joint_pos()

    def get_end_effector_pose(self) -> List[float]:
        raise NotImplementedError("Not implemented for Yam Arm")

    @staticmethod
    def bring_up_can_interface(iface: str, bitrate: int = 1_000_000):
        """Bring up the CAN interface with the specified bitrate."""
        try:
            subprocess.run(["sudo", "ip", "link", "set", iface, "down"], check=True)
            subprocess.run(
                [
                    "sudo",
                    "ip",
                    "link",
                    "set",
                    iface,
                    "type",
                    "can",
                    "bitrate",
                    str(bitrate),
                ],
                check=True,
            )
            subprocess.run(["sudo", "ip", "link", "set", iface, "up"], check=True)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to bring up CAN interface {iface}: {e}")

    def setup_can(self, can_num: int):
        can_adapter_serial = can_num_to_serial[can_num]
        if not can_adapter_serial:
            raise ValueError(
                f"Invalid CAN number: {can_num}. No serial defined. Available can serials: {list(can_num_to_serial.keys())}"
            )

        ## Get the CAN interface name based on the serial number
        ctx = pyudev.Context()
        for dev in ctx.list_devices(subsystem="net"):
            if dev.sys_name.startswith("can"):
                usb = dev.find_parent("usb", "usb_device")
                print(
                    f"CAN interface found with ID: {usb.properties.get('ID_SERIAL_SHORT')}"
                )
                if (
                    usb is not None
                    and usb.properties.get("ID_SERIAL_SHORT") == can_adapter_serial
                ):
                    print(
                        f"Found CAN interface: {dev.sys_name} with serial {can_adapter_serial}"
                    )
                    can_name = dev.sys_name
                    self.bring_up_can_interface(can_name)
                    return can_name
        raise ValueError(f"CAN interface with serial {can_adapter_serial} not found.")


if __name__ == "__main__":
    """Example usage of YamArm"""
    yam_arm = YamArm(can_num='0')
    yam_arm.connect()
    yam_arm.move_to_home_position()

    ## Test move_j
    yam_arm.move_j([0.2, 0.2, 0.2, 0.0, 0.0, 0.0])

    # side = +1
    # j = 0.0
    # import time
    # try:
    #     while True: 
    #         j += 0.1 * side
    #         if j > 0.5 or j < -0.5:
    #             side *= -1
    #         yam_arm.move_j([0.0, 0.2, 0.2, j, 0.0, 0.0])
    #         time.sleep(0.01)
    # except KeyboardInterrupt:
    #     pass