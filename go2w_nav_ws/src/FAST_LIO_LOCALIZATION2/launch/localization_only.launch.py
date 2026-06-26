from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pcd_map_path = LaunchConfiguration("map")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")

    declare_map_path = DeclareLaunchArgument(
        "map",
        default_value="/home/luo/go2w_nav_ws/maps/localization/start_aligned_to_big.pcd",
        description="Path to PCD map file"
    )

    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic",
        default_value="/map",
        description="Topic to publish PCD map"
    )

    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization.py",
        name="global_localization",
        output="screen",
        parameters=[{
            "map_voxel_size": 0.4,
            "scan_voxel_size": 0.1,
            "freq_localization": 0.5,
            "freq_global_map": 0.25,
            "localization_threshold": 0.8,
            "fov": 6.28319,
            "fov_far": 300,
            "pcd_map_path": pcd_map_path,
            "pcd_map_topic": pcd_map_topic
        }],
    )

    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
    )

    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{
            "file_name": pcd_map_path,
            "tf_frame": "map",
            "cloud_topic": pcd_map_topic,
            "period_ms_": 500
        }],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )

    return LaunchDescription([
        declare_map_path,
        declare_pcd_map_topic,
        pcd_publisher_node,
        global_localization_node,
        transform_fusion_node,
    ])