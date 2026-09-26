#!/usr/bin/env python3
"""Start the web dashboard (open http://localhost:8080 in a Windows/Linux browser)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        Node(package="lastmile_dashboard", executable="dashboard_node", name="lastmile_dashboard",
             output="screen",
             parameters=[{"use_sim_time": True, "port": LaunchConfiguration("port")}]),
    ])
