import os
from glob import glob
from setuptools import find_packages, setup

package_name = "robot_state_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Surya",
    maintainer_email="karthikeya7890@gmail.com",
    description=(
        "Custom ROS2 node: 10Hz xyz telemetry publisher, Nav2 waypoint "
        "goal receiver, and cmd_vel threshold monitor."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_state_manager_node = robot_state_manager.robot_state_manager_node:main",
        ],
    },
)
