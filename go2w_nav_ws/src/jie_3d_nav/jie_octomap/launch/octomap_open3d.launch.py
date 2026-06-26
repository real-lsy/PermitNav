from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value="true",
        description="Launch map manager, save/load window, and map viewer window",
    )

    world_to_octomap_node = Node(
        package="jie_octomap",
        executable="world_to_octomap_node",
        name="world_to_octomap",
        output="screen",
        parameters=[
            {
                "frame_id": "map",
                "world_file": "/home/robot/lv2.world",
                "ground_surface_max_thickness_m": 0.6,
                "enable_stair_step_surface_mode": True,
                "stair_step_max_height_m": 0.5,
                "stair_step_max_depth_m": 0.8,
                "stair_step_min_width_m": 1.0,
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "world_file_cmd_topic": "/world_file_cmd",
            }
        ],
    )

    world_picker_gui_node = Node(
        package="jie_octomap",
        executable="world_file_picker_gui.py",
        name="world_file_picker_gui",
        output="screen",
        parameters=[
            {
                "world_file_cmd_topic": "/world_file_cmd",
            }
        ],
    )

    open3d_viewer_node = Node(
        package="jie_octomap",
        executable="octomap_open3d_viewer_node",
        name="open3d_octomap_viewer",
        output="screen",
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "__GLX_VENDOR_LIBRARY_NAME": "mesa",
            "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe",
        },
        parameters=[
            {
                "octomap_topic": "/octomap",
                "marker_topic": "/selection_markers",
                "path_topic": "/planned_path",
                "preblocked_marker_topic": "/preblocked_cells_markers",
            }
        ],
    )

    selector_node = Node(
        package="jie_octomap",
        executable="rviz_click_selector_node",
        name="rviz_click_selector",
        output="screen",
        parameters=[
            {
                "clicked_topic": "/clicked_point",
                "marker_topic": "/selection_markers",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
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
                "robot_radius": 0.25,
                "max_iterations": 500000,
                "snap_search_radius_cells": 12,
                "require_ground_support": True,
                "strict_direct_ground_support": False,
                "ground_support_xy_radius_cells": 1,
                "ground_support_depth_cells": 1,
                "enable_preblocked_costmap": True,
                "preblocked_costmap_radius_cells": 3,
                "preblocked_costmap_weight": 2.5,
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

    map_save_gui_node = Node(
        package="jie_octomap",
        executable="map_save_gui",
        name="map_save_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
    )

    map_viewer_gui_node = Node(
        package="jie_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
    )

    return LaunchDescription(
        [launch_map_gui_arg,
        world_to_octomap_node, 
        # world_picker_gui_node, 
        selector_node, 
        planner_node, 
        map_package_manager_node,
        map_save_gui_node,
        map_viewer_gui_node,
        open3d_viewer_node]
    )
