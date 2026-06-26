import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def build_nodes(context):
    world_file = LaunchConfiguration("world_file").perform(context).strip()
    world_name = LaunchConfiguration("world_name").perform(context).strip()
    if not world_file and world_name:
        world_file = os.path.join(
            get_package_share_directory("jie_octomap"), "worlds", world_name
        )

    world_to_octomap_node = Node(
        package="jie_octomap",
        executable="world_to_octomap_node",
        name="world_to_octomap",
        output="screen",
        parameters=[
            {
                "world_file": world_file,
                "resolution": 0.2,
                "frame_id": "map",
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
            }
        ],
    )

    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": "map",
                "map_id": "world_jie_path_map",
                "source_world_file": world_file,
                "robot_radius": 0.12,
                "max_iterations": 500000,
                "snap_search_radius_cells": 12,
                "require_ground_support": True,
                "ground_support_xy_radius_cells": 1,
                "ground_support_depth_cells": 2,
            }
        ],
    )

    world_selector_gui_node = Node(
        package="jie_octomap",
        executable="world_selector_gui.py",
        name="world_selector_gui",
        output="screen",
        parameters=[
            {
                "initial_world_file": world_file,
                "world_file_cmd_topic": "/world_file_cmd",
                "occupied_marker_topic": "/octomap_occupied_markers",
                "preblocked_topic": "/preblocked_cells_markers",
                "traversable_topic": "/traversable_cells_markers",
            }
        ],
    )

    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
    )

    return [
        world_selector_gui_node,
        world_to_octomap_node,
        planner_node,
        map_package_manager_node,
    ]


def generate_launch_description():
    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value="",
        description="Absolute path to Gazebo .world/.sdf file. Overrides world_name.",
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="",
        description="World filename under the jie_octomap package worlds directory.",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value="true",
        description="Launch map package manager and PyQt save/load window",
    )

    return LaunchDescription(
        [
            world_file_arg,
            world_name_arg,
            launch_map_gui_arg,
            OpaqueFunction(function=build_nodes),
        ]
    )
