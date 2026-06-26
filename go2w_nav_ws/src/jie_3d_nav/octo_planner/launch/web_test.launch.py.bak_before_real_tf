import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_nav_file_config(config_path: str, validate_pcd_file: bool) -> tuple[str, str, str]:
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}

    relocalization_bin_file = os.path.abspath(
        os.path.expanduser(str(config.get("relocalization_bin_file", "")).strip())
    )
    relocalization_pcd_file = os.path.abspath(
        os.path.expanduser(str(config.get("relocalization_pcd_file", "")).strip())
    )
    map_package_dir = os.path.abspath(
        os.path.expanduser(str(config.get("map_package_dir", "")).strip())
    )

    if not relocalization_bin_file:
        raise RuntimeError(f"{config_path} 中未配置 relocalization_bin_file")
    if validate_pcd_file and not relocalization_pcd_file:
        raise RuntimeError(f"{config_path} 中未配置 relocalization_pcd_file")
    if not map_package_dir:
        raise RuntimeError(f"{config_path} 中未配置 map_package_dir")
    if not os.path.isfile(relocalization_bin_file):
        raise RuntimeError(f"重定位 .bin 文件不存在: {relocalization_bin_file}")
    if validate_pcd_file and not os.path.isfile(relocalization_pcd_file):
        raise RuntimeError(f"重定位 .pcd 文件不存在: {relocalization_pcd_file}")
    if not os.path.isdir(map_package_dir):
        raise RuntimeError(f"地图目录不存在: {map_package_dir}")

    return relocalization_bin_file, relocalization_pcd_file, map_package_dir


def load_d1_controller_params(config_path: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    return dict(config.get("d1_controller", {}).get("ros__parameters", {}))


def load_bool_config(config_path: str, key: str, default: bool) -> str:
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    return "true" if bool(config.get(key, default)) else "false"


def generate_launch_description():
    octo_planner_share = get_package_share_directory("octo_planner")
    nav_params_config = os.path.join(octo_planner_share, "config", "nav_params.yaml")
    show_rviz_default = load_bool_config(nav_params_config, "show_rviz", False)
    show_map_gui_default = load_bool_config(nav_params_config, "show_map_gui", False)

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value=show_rviz_default,
        description="Launch RViz and RViz-only point cloud publishers",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value=show_map_gui_default,
        description="Launch Qt map viewer and save/load windows",
    )
    launch_planner_arg = DeclareLaunchArgument(
        "launch_planner",
        default_value="true",
        description="Launch jie_path_node for interactive path planning",
    )
    launch_controller_arg = DeclareLaunchArgument(
        "launch_controller",
        default_value="false",
        description="Launch d1_controller for path execution",
    )
    launch_web_arg = DeclareLaunchArgument(
        "launch_web",
        default_value="true",
        description="Launch the web-based OctoMap viewer",
    )
    launch_rosbridge_arg = DeclareLaunchArgument(
        "launch_rosbridge",
        default_value="true",
        description="Launch rosbridge_websocket for the web client",
    )
    web_http_port_arg = DeclareLaunchArgument(
        "web_http_port",
        default_value="8080",
        description="HTTP port for the web viewer",
    )

    jie_octomap_share = get_package_share_directory("jie_octomap")

    odin1_loc_rviz_config_file = os.path.join(
        jie_octomap_share, "rviz", "odin1_loc.rviz"
    )
    web_root = os.path.join(jie_octomap_share, "web")
    web_server_script = os.path.abspath(
        os.path.join(
            jie_octomap_share,
            "..",
            "..",
            "lib",
            "jie_octomap",
            "no_cache_http_server.py",
        )
    )
    relocalization_bin_file, relocalization_pcd_file, map_package_dir = load_nav_file_config(
        nav_params_config,
        validate_pcd_file=(show_rviz_default == "true"),
    )
    d1_controller_params = load_d1_controller_params(nav_params_config)

    robot_desc = """<?xml version="1.0"?>
<robot name="web_test_robot">
  <link name="base_link"/>
</robot>
"""

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    static_odom_to_base_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    test_map_to_odom_tf_node = Node(
        package="octo_planner",
        executable="test_map_to_odom_tf_node",
        name="test_map_to_odom_tf_node",
        output="screen",
        parameters=[
            {
                "parent_frame": "map",
                "child_frame": "odom",
                "radius": 2.0,
                "orbit_period": 20.0,
                "spin_rate": 0.8,
            }
        ],
    )

    pcd_publisher_node = Node(
        package="jie_octomap",
        executable="pcd_file_publisher.py",
        name="pcd_file_publisher",
        output="screen",
        parameters=[
            {
                "pcd_path": relocalization_pcd_file,
                "topic": "/pcd_points",
                "frame_id": "map",
                "publish_hz": 1.0,
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", odin1_loc_rviz_config_file],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_planner")),
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "edited_occupied_marker_topic": "/edited_occupied_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": "map",
                "map_id": "loaded_map",
                "source_world_file": "",
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

    controller_node = Node(
        package="octo_planner",
        executable="d1_controller",
        name="d1_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_controller")),
        parameters=[
            d1_controller_params,
            {
                "path_topic": "/planned_path",
                "start_navigation_topic": "/start_navigation",
                "stop_navigation_topic": "/stop_navigation",
                "require_start_command": True,
                "cmd_vel_topic": "/cmd_vel",
                "manual_cmd_vel_topic": "/web_cmd_vel",
                "tracking_point_marker_topic": "/tracking_point_marker",
                "map_frame": "map",
                "base_frame": "base_link",
            }
        ],
    )

    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        parameters=[
            {
                "autoload_package_path": map_package_dir,
            }
        ],
    )

    occupied_marker_node = Node(
        package="jie_octomap",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": "map",
            }
        ],
    )

    map_viewer_gui_node = Node(
        package="jie_octomap",
        executable="map_viewer_gui",
        name="map_viewer_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
        additional_env={"MAP_VIEWER_DEFAULT_PACKAGE": map_package_dir},
        parameters=[
            {
                "tf_parent_frame": "map",
                "tf_child_frame": "base_link",
            }
        ],
    )

    map_save_gui_node = Node(
        package="jie_octomap",
        executable="map_save_gui",
        name="map_save_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
    )

    web_http_server = ExecuteProcess(
        cmd=[
            web_server_script,
            "--port",
            LaunchConfiguration("web_http_port"),
            "--directory",
            web_root,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_web")),
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_rosbridge")),
    )

    return LaunchDescription(
        [
            launch_rviz_arg,
            launch_map_gui_arg,
            launch_planner_arg,
            launch_controller_arg,
            launch_web_arg,
            launch_rosbridge_arg,
            web_http_port_arg,
            robot_state_publisher_node,
            static_odom_to_base_node,
            test_map_to_odom_tf_node,
            pcd_publisher_node,
            rviz_node,
            planner_node,
            controller_node,
            map_package_manager_node,
            occupied_marker_node,
            map_viewer_gui_node,
            map_save_gui_node,
            web_http_server,
            rosbridge_node,
        ]
    )
