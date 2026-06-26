import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# import launch

################### user configure parameters for ros2 start ###################
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

# package_path = get_package_share_directory("livox_ros_driver2"
package_path = "/home/wheelchair2/livox_ws/src/livox_ros_driver2"   # replace with your own path
cur_config_path = package_path + '/config'
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
################### user configure parameters for ros2 end #####################

def generate_launch_description():
    inverted = LaunchConfiguration("inverted")
    declare_inverted = DeclareLaunchArgument("inverted", default_value="true", description="Specify if the lidar is inverted or not")
    
    xfer_format = LaunchConfiguration("xfer_format")    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
    declare_xfer_format = DeclareLaunchArgument("xfer_format", default_value="1", description="Declare livox msg")
    
    rviz = LaunchConfiguration("rviz")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="false", description="Start rviz for displaying pointclouds")
    
    livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        condition = UnlessCondition(inverted),
        )
    
    livox_driver_remap = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        remappings=[
                ("/livox/lidar", "/livox/inverted_lidar"),
                ("/livox/imu", "/livox/inverted_imu")
            ],
        condition = IfCondition(inverted)
        )
    
    invert_lidar_node = Node(
        package='fast_lio_localization',
        executable='invert_livox_scan.py',
        name='invert_livox_scan',
        output='screen',
        parameters=[{"xfer_format": xfer_format}],
        condition=IfCondition(inverted),
        )
    
    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path],
            condition = IfCondition(rviz),
        )


    return LaunchDescription([
        declare_inverted,
        declare_xfer_format,
        declare_rviz,
        livox_driver,
        livox_driver_remap,
        invert_lidar_node,
        livox_rviz,
    ])