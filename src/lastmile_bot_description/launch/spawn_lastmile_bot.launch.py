#!/usr/bin/env python3
"""
Spawn lastmile_bot (the Minecraft-styled delivery-bot character) into an
already-running lastmile_world (start Gazebo separately via
`ros2 launch lastmile_description spawn_world.launch.py gui:=true` first,
or use this launch file's `start_world` arg to bring the world up too).

This is the fun/branding alternative to spawn_turtlebot3 in
lastmile_description/launch/spawn_world.launch.py - TurtleBot3 remains the
default, validated path; use this when you specifically want lastmile_bot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# Same spawn point as TurtleBot3: the assignment's origin (0, 0), dropped from
# 10cm so the wheels settle onto the reconstructed floor (~2cm above z=0 here).
SPAWN_X = "0.0"
SPAWN_Y = "0.0"
SPAWN_Z = "0.10"


def generate_launch_description():
    bot_pkg_share = get_package_share_directory("lastmile_bot_description")
    xacro_path = os.path.join(bot_pkg_share, "urdf", "lastmile_bot.urdf.xacro")
    robot_description = Command(["xacro ", xacro_path])

    start_world_arg = DeclareLaunchArgument("start_world", default_value="false")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lastmile_description"),
                "launch", "spawn_world.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_world")),
        # don't also spawn TurtleBot3 when this launch file is bringing the world up -
        # both robots' diff-drive plugins would otherwise fight over /cmd_vel
        launch_arguments={"gui": "true", "spawn_turtlebot3": "false"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_bot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_lastmile_bot",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "lastmile_bot",
            "-x", SPAWN_X, "-y", SPAWN_Y, "-z", SPAWN_Z,
        ],
    )

    return LaunchDescription([
        start_world_arg,
        world_launch,
        robot_state_publisher,
        spawn_bot,
    ])
