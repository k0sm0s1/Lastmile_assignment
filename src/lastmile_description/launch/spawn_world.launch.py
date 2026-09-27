#!/usr/bin/env python3
"""
Task 1: bring up Gazebo Classic with the scanned environment loaded as a
static collidable model, and spawn a TurtleBot3 (Waffle) into it at the
assignment's designated origin (0, 0, 0).

(0, 0) is the start of the SLAM trajectory that produced the scan and sits
in the middle of the corridor (~0.86 m from the nearest wall - checked
against both the occupancy grid and the Gazebo mesh). The robot is dropped
from z = 0.10 m so its wheels settle onto the reconstructed floor surface
(which is ~2 cm above z = 0 at the origin) instead of starting interpenetrated.

This launch file also starts robot_state_publisher (with sim time). Without it
there is no base_footprint -> base_link -> base_scan transform, AMCL cannot
place a single laser scan, and it never publishes map -> odom.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SPAWN_X = "0.0"
SPAWN_Y = "0.0"
SPAWN_Z = "0.10"
SPAWN_YAW = "0.0"

# Obstacle mode (obstacles:=true): three unmapped boxes in a slalom along the
# middle of the east-west corridor (free width there is about y -0.85 .. 0.8).
# Each leaves a ~1 m gap for the 0.44 m robot and stays clear of the north-arm
# junction (x 7.5-10). They are NOT in lastmile_map.pgm.
OBSTACLES = [("obstacle_1", 4.5, 0.35), ("obstacle_2", 6.5, -0.35), ("obstacle_3", 10.5, 0.35)]


def generate_launch_description():
    pkg_share = get_package_share_directory("lastmile_description")
    world_path = os.path.join(pkg_share, "worlds", "lastmile_world.world")
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    tb3_model_sdf = os.path.join(tb3_gazebo_share, "models", "turtlebot3_waffle", "model.sdf")

    tb3_model_env = SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle")
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.join(pkg_share, "models") + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""),
    )

    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")),
        launch_arguments={"world": world_path, "verbose": "true"}.items(),
    )
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    spawn_tb3 = Node(
        package="gazebo_ros", executable="spawn_entity.py", name="spawn_turtlebot3",
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_turtlebot3")),
        arguments=["-entity", "turtlebot3_waffle", "-file", tb3_model_sdf,
                   "-x", SPAWN_X, "-y", SPAWN_Y, "-z", SPAWN_Z, "-Y", SPAWN_YAW],
    )
    tb3_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gazebo_share, "launch", "robot_state_publisher.launch.py")),
        condition=IfCondition(LaunchConfiguration("spawn_turtlebot3")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    obstacle_sdf = os.path.join(pkg_share, "models", "obstacle_box", "model.sdf")
    spawn_obstacles = [
        Node(package="gazebo_ros", executable="spawn_entity.py", name=f"spawn_{name}", output="screen",
             condition=IfCondition(LaunchConfiguration("obstacles")),
             arguments=["-entity", name, "-file", obstacle_sdf, "-x", str(x), "-y", str(y), "-z", "0.0"])
        for name, x, y in OBSTACLES
    ]

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("obstacles", default_value="false",
                              description="spawn the three unmapped slalom boxes"),
        # Set false when spawning a different robot into this world instead
        # (e.g. lastmile_bot_description's spawn_lastmile_bot.launch.py) -
        # both robots' diff-drive plugins listen on /cmd_vel.
        DeclareLaunchArgument("spawn_turtlebot3", default_value="true"),
        tb3_model_env,
        gazebo_model_path,
        gzserver_launch,
        gzclient_launch,
        tb3_state_publisher,
        spawn_tb3,
        *spawn_obstacles,
    ])
