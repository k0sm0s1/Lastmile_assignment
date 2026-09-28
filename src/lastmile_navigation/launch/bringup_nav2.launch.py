#!/usr/bin/env python3
"""
Task 2: bring up map_server + AMCL + the rest of the Nav2 stack against the
pre-baked static map (lastmile_map.yaml), giving a stable map->odom transform
that combines with the TurtleBot3 Gazebo plugin's own odom->base_link to
complete the frame chain the assignment asks for.

Run this after spawn_world.launch.py (Task 1) is already up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_pkg_share = get_package_share_directory("lastmile_navigation")
    default_map = os.path.join(nav_pkg_share, "maps", "lastmile_map.yaml")
    # bounded planning map (outside the building = occupied), costmaps only
    nav_map = os.path.join(nav_pkg_share, "maps", "lastmile_nav_map.yaml")
    default_params = os.path.join(nav_pkg_share, "params", "nav2_params.yaml")
    obstacle_params = os.path.join(nav_pkg_share, "params", "nav2_params_obstacles.yaml")
    obstacles = LaunchConfiguration("obstacles")
    # obstacles:=true switches the default params to the obstacle-mode file
    params_default = PythonExpression(["'", obstacle_params, "' if '", obstacles, "' == 'true' else '", default_params, "'"])

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={},
        convert_types=True,
    )

    lifecycle_nodes = [
        "map_server", "nav_map_server", "amcl", "controller_server", "planner_server",
        "behavior_server", "bt_navigator",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("obstacles", default_value="false",
                              description="live lidar obstacle avoidance (obstacle_mapper + extra costmap layer)"),
        DeclareLaunchArgument("params_file", default_value=params_default),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
        ),
        # /nav_map: same scan, but everything outside the building is a wall,
        # so the planner can't route through scan gaps into unscanned space
        Node(
            package="nav2_map_server", executable="map_server", name="nav_map_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time, "yaml_filename": nav_map,
                                            "topic_name": "nav_map"}],
        ),
        Node(
            package="nav2_amcl", executable="amcl", name="amcl",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_controller", executable="controller_server", name="controller_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_planner", executable="planner_server", name="planner_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server", name="behavior_server",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
            output="screen",
            parameters=[configured_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": lifecycle_nodes}],
        ),
        # obstacle mode: live lidar obstacles -> /obstacle_map
        Node(
            package="lastmile_obstacles", executable="obstacle_mapper", name="obstacle_mapper",
            output="screen", condition=IfCondition(obstacles),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
