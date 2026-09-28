#!/usr/bin/env python3
"""Delivery mode: two-stop mission page at http://localhost:8081 (needs the Task 3 node running)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8081"),
        Node(package="lastmile_dashboard", executable="delivery_node", name="delivery_node", output="screen",
             parameters=[{"use_sim_time": True, "port": LaunchConfiguration("port")}]),
    ])
