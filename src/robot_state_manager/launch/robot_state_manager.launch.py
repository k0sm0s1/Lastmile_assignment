#!/usr/bin/env python3
"""Task 3 launch: start the robot_state_manager node with tunable thresholds."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("linear_velocity_threshold", default_value="0.20"),
        DeclareLaunchArgument("angular_velocity_threshold", default_value="0.80"),
        DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="robot_state_manager",
            executable="robot_state_manager_node",
            name="robot_state_manager",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "linear_velocity_threshold": LaunchConfiguration("linear_velocity_threshold"),
                "angular_velocity_threshold": LaunchConfiguration("angular_velocity_threshold"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            }],
        ),
    ])
