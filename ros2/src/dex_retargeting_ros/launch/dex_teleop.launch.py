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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ## Set the ports for the right and left hands here.
    ## If you want to use only one hand, set the other port to ''
    ## To find the port for your hand on Linux, you can use the command:
    ##   ls /dev/serial/by-id/
    ## Example port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A9J6J4J3-if00-port0
    right_hand_port = ""
    left_hand_port = ""
    baudrate = 921600
    feedback_frequency = 100.0  # Hz

    ## Nodes
    mediapipe_mocap = Node(
        package="aero_hand_open_teleop",
        executable="mediapipe_mocap",
        name="mediapipe_mocap",
        output="screen",
        emulate_tty=True,
    )

    dex_retargeting_node = Node(
        package="dex_retargeting_ros",
        executable="dex_retargeting_node",
        name="dex_retargeting_node",
        output="screen",
        emulate_tty=True,
    )

    aero_hand_node = Node(
        package="aero_hand_open",
        executable="aero_hand_node",
        name="aero_hand_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "right_port": right_hand_port,
                "left_port": left_hand_port,
                "baudrate": baudrate,
                "feedback_frequency": feedback_frequency,
            }
        ],
    )

    return LaunchDescription(
        [
            mediapipe_mocap,
            dex_retargeting_node,
            aero_hand_node,
        ]
    )
