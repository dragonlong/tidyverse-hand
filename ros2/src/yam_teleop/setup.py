# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "yam_teleop"

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
    description="Quest 3 wrist pose to YAM arm IK teleoperation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "quest3_yam_teleop = yam_teleop.quest3_yam_teleop:main",
        ],
    },
)
