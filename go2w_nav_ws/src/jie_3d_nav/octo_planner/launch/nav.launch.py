import os
import shutil
import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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


def update_relocalization_map_path(config_path: str, bin_file_path: str) -> None:
    with open(config_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    target_prefix = "  relocalization_map_abs_path:"
    updated = False
    for idx, line in enumerate(lines):
        stripped = line.lstrip()
        if stripped.startswith("relocalization_map_abs_path:") or line.startswith(target_prefix):
            comment = ""
            if "#" in line:
                comment = "  #" + line.split("#", 1)[1].rstrip("\n")
            lines[idx] = f'  relocalization_map_abs_path: "{bin_file_path}"{comment}\n'
            updated = True
            break

    if not updated:
        raise RuntimeError(
            f"配置文件中未找到 relocalization_map_abs_path 项: {config_path}"
        )

    with open(config_path, "w", encoding="utf-8") as f:
        f.writelines(lines)


def sync_config_to_driver(src_config_path: str, driver_package_dir: str) -> str:
    driver_config_candidates = []

    colcon_prefix_path = os.environ.get("COLCON_PREFIX_PATH", "")
    if colcon_prefix_path:
        first_prefix = colcon_prefix_path.split(":")[0]
        install_pos = first_prefix.find("/install")
        if install_pos != -1:
            workspace_root = first_prefix[:install_pos]
            driver_config_candidates.append(
                os.path.join(
                    workspace_root,
                    "src",
                    "odin_ros_driver",
                    "config",
                    "control_command.yaml",
                )
            )

    driver_config_candidates.append(
        os.path.join(driver_package_dir, "config", "control_command.yaml")
    )

    for driver_config_path in driver_config_candidates:
        driver_config_dir = os.path.dirname(driver_config_path)
        if os.path.isdir(driver_config_dir):
            shutil.copyfile(src_config_path, driver_config_path)
            return driver_config_path

    raise RuntimeError("未找到 odin_ros_driver/config/control_command.yaml 可同步的目标路径")


def generate_launch_description():
    octo_planner_share = get_package_share_directory("octo_planner")
    nav_params_config = os.path.join(octo_planner_share, "config", "nav_params.yaml")
    show_rviz_default = load_bool_config(nav_params_config, "show_rviz", False)
    show_map_gui_default = load_bool_config(nav_params_config, "show_map_gui", False)
    publish_d1_odom_default = load_bool_config(nav_params_config, "publish_d1_odom", False)
    use_static_odom_to_base_default = load_bool_config(
        nav_params_config, "use_static_odom_to_base", True
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value=show_rviz_default,
        description="Launch RViz and RViz-only point cloud publishers",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value=show_map_gui_default,
        description="Launch map viewer and save/load GUI windows",
    )
    publish_d1_odom_arg = DeclareLaunchArgument(
        "publish_d1_odom",
        default_value=publish_d1_odom_default,
        description="Publish dynamic odom -> base_link from d1_core",
    )
    use_static_odom_to_base_arg = DeclareLaunchArgument(
        "use_static_odom_to_base",
        default_value=use_static_odom_to_base_default,
        description="Publish a fixed odom -> base_link fallback transform",
    )
    launch_planner_arg = DeclareLaunchArgument(
        "launch_planner",
        default_value="true",
        description="Launch jie_path_node for interactive path planning",
    )
    launch_controller_arg = DeclareLaunchArgument(
        "launch_controller",
        default_value="true",
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

    d1_bringup_share = get_package_share_directory("d1_bringup")
    d1_description_share = get_package_share_directory("d1_description")
    jie_octomap_share = get_package_share_directory("jie_octomap")
    odin_ros_driver_share = get_package_share_directory("odin_ros_driver")

    d1_params_file = os.path.join(d1_bringup_share, "config", "d1_params.yaml")
    map_file = os.path.join(d1_bringup_share, "maps", "map.yaml")
    rviz_config_file = os.path.join(d1_bringup_share, "rviz", "navi.rviz")
    odin1_loc_rviz_config_file = os.path.join(
        jie_octomap_share, "rviz", "odin1_loc.rviz"
    )
    urdf_file = os.path.join(d1_description_share, "urdf", "d1.urdf")
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
    try:
        odin_costmap_share = get_package_share_directory("odin_costmap")
        loc_control_config_path = os.path.join(
            odin_costmap_share, "config", "loc_control_command.yaml"
        )
    except PackageNotFoundError:
        loc_control_config_path = os.path.join(
            jie_octomap_share, "config", "loc_control_command.yaml"
        )

    relocalization_bin_file, relocalization_pcd_file, map_package_dir = load_nav_file_config(
        nav_params_config,
        validate_pcd_file=(show_rviz_default == "true"),
    )
    d1_controller_params = load_d1_controller_params(nav_params_config)
    update_relocalization_map_path(loc_control_config_path, relocalization_bin_file)
    driver_control_config_path = sync_config_to_driver(
        loc_control_config_path, odin_ros_driver_share
    )

    with open(urdf_file, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    d1_core_node = Node(
        package="d1_bringup",
        executable="d1_core",
        name="d1_core_node",
        output="screen",
        parameters=[
            d1_params_file,
            {
                "publish_odom": ParameterValue(
                    LaunchConfiguration("publish_d1_odom"), value_type=bool
                )
            },
        ],
    )

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
        condition=IfCondition(LaunchConfiguration("use_static_odom_to_base")),
    )

    host_sdk_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{"config_file": driver_control_config_path}],
    )

    with open(driver_control_config_path, "r", encoding="utf-8") as f:
        pcd2depth_params = yaml.safe_load(f)
    pcd2depth_params["calib_file_path"] = os.path.join(
        odin_ros_driver_share, "config", "calib.yaml"
    )
    pcd2depth_node = Node(
        package="odin_ros_driver",
        executable="pcd2depth_ros2_node",
        name="pcd2depth_ros2_node",
        output="screen",
        parameters=[pcd2depth_params],
    )

    with open(driver_control_config_path, "r", encoding="utf-8") as f:
        reprojection_params = yaml.safe_load(f)
    reprojection_params["calib_file_path"] = os.path.join(
        odin_ros_driver_share, "config", "calib.yaml"
    )
    cloud_reprojection_node = Node(
        package="odin_ros_driver",
        executable="cloud_reprojection_ros2_node",
        name="cloud_reprojection_ros2_node",
        output="screen",
        parameters=[reprojection_params],
    )

    with open(driver_control_config_path, "r", encoding="utf-8") as f:
        overlay_params = yaml.safe_load(f)
    image_overlay_node = Node(
        package="odin_ros_driver",
        executable="image_overlay_node",
        name="image_overlay_node",
        output="screen",
        parameters=[overlay_params],
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
                "enable_tracking_debug_view": ParameterValue(
                    LaunchConfiguration("launch_map_gui"), value_type=bool
                ),
                "map_frame": "map",
                "base_frame": "odin1_base_link",
                "base_frame_candidates": "odin1_base_link,base_link,base_footprint",
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
                "tf_child_frame": "odin1_base_link",
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
            publish_d1_odom_arg,
            use_static_odom_to_base_arg,
            launch_planner_arg,
            launch_controller_arg,
            launch_web_arg,
            launch_rosbridge_arg,
            web_http_port_arg,
            host_sdk_node,
            pcd2depth_node,
            cloud_reprojection_node,
            image_overlay_node,
            pcd_publisher_node,
            rviz_node,
            d1_core_node,
            robot_state_publisher_node,
            static_odom_to_base_node,
            planner_node,
            controller_node,
            map_package_manager_node,
            occupied_marker_node,
            map_viewer_gui_node,
            #map_save_gui_node,
            web_http_server,
            rosbridge_node,
        ]
    )
