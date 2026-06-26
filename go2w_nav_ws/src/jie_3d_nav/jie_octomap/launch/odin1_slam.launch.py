
# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os
import shutil
import yaml 
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def sync_config_to_driver(src_config_path: str, driver_package_dir: str) -> str:
    driver_config_candidates = []

    colcon_prefix_path = os.environ.get("COLCON_PREFIX_PATH", "")
    if colcon_prefix_path:
        first_prefix = colcon_prefix_path.split(":")[0]
        install_pos = first_prefix.find("/install")
        if install_pos != -1:
            workspace_root = first_prefix[:install_pos]
            driver_config_candidates.append(
                os.path.join(workspace_root, "src", "odin_ros_driver", "config", "control_command.yaml")
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
    # Get package directory
    package_dir = get_package_share_directory('odin_ros_driver')
    try:
        costmap_package_dir = get_package_share_directory('odin_costmap')
        slam_control_config_path = os.path.join(
            costmap_package_dir, 'config', 'slam_control_command.yaml'
        )
    except PackageNotFoundError:
        octomap_package_dir = get_package_share_directory('jie_octomap')
        slam_control_config_path = os.path.join(
            octomap_package_dir, 'config', 'slam_control_command.yaml'
        )
    driver_control_config_path = sync_config_to_driver(slam_control_config_path, package_dir)
    
    # Declare configuration parameter
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=driver_control_config_path,
        description='Path to the control config YAML file'
    )
    
    # Add RViz2 configuration file parameter
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'config', 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )
    
    # Create main node
    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
       # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    pcd2depth_config_path = driver_control_config_path
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f) 
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path 
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',  
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # Cloud reprojection node
    reprojection_config_path = driver_control_config_path
    with open(reprojection_config_path, 'r') as f:
        reprojection_params = yaml.safe_load(f) 
    reprojection_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path 
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',  
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    # Image overlay node - overlays reprojected points on camera image
    overlay_config_path = driver_control_config_path
    with open(overlay_config_path, 'r') as f:
        overlay_params = yaml.safe_load(f)
    image_overlay_node = Node(
        package='odin_ros_driver',
        executable='image_overlay_node',  
        name='image_overlay_node',
        output='screen',
        parameters=[overlay_params]
    )

    save_bin_map_gui_node = Node(
        package='jie_octomap',
        executable='save_bin_map_gui.py',
        name='save_bin_map_gui',
        output='screen',
    )

    # Create RViz2 node - loads specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    
    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(rviz_config_arg) 
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(cloud_reprojection_node)
    ld.add_action(image_overlay_node)
    ld.add_action(save_bin_map_gui_node)
    ld.add_action(rviz_node) 
    
    return ld
