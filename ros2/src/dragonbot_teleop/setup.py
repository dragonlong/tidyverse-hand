# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "dragonbot_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@tetheria.ai",
    description="Dragonbot teleoperation: base (SpaceMouse), arm (Quest 3 wrist IK), and hand (Quest 3 retargeting)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "quest3_yam_teleop = dragonbot_teleop.quest3_yam_teleop:main",
            "quest3_retargeting = dragonbot_teleop.quest3_retargeting:main",
            "spacemouse = dragonbot_teleop.spacemouse:main",
            "mujoco_dragonbot_viewer = dragonbot_teleop.mujoco_dragonbot_viewer:main",
            "mujoco_hand_viewer = dragonbot_teleop.mujoco_hand_viewer:main",
            "mujoco_yam_hand_viewer = dragonbot_teleop.mujoco_yam_hand_viewer:main",
        ],
    },
)
