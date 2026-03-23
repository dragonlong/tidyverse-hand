# Copyright 2025 TetherIA, Inc.
# Licensed under the Apache License, Version 2.0

"""Launch Quest 3 full teleop (local PC): bridge + finger retargeting only.

Arm control is handled by the Gello controller (yam_gello_controller.py) on
the mobile PC, which subscribes to /joint_states published by gello_publisher.py
on the local PC.  This launch file covers only the hand control side.

aero_hand_node lives on the mobile PC and subscribes to /right/joint_control
over the shared ROS_DOMAIN_ID.

Launch arguments:
  params_file  string  Bridge YAML params path (defaults to built-in).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = Path(get_package_share_directory("hand_tracking_sdk_ros2"))
    default_params = str(bridge_share / "config" / "bridge.params.yaml")

    params_file = LaunchConfiguration("params_file")

    hand_tracking_bridge = Node(
        package="hand_tracking_sdk_ros2",
        executable="hand_tracking_bridge",
        name="hand_tracking_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
    )

    quest3_retargeting = Node(
        package="dragonbot_teleop",
        executable="quest3_retargeting",
        name="quest3_retargeting",
        output="screen",
        emulate_tty=True,
        parameters=[{"ema_alpha": 0.7}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to bridge YAML parameters file.",
        ),
        hand_tracking_bridge,
        quest3_retargeting,
    ])
