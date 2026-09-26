#!/usr/bin/env python3
"""Launch lastmile_bot's cosmetic idle animation node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lastmile_bot_animator",
            executable="idle_animator_node",
            name="lastmile_bot_idle_animator",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
