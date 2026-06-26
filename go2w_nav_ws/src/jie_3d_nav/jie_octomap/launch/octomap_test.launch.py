from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("jie_octomap")
    rviz_config = os.path.join(pkg_share, "rviz", "octomap_test.rviz")

    octomap_node = Node(
        package="jie_octomap",
        executable="octomap_test",
        name="octomap_test",
        output="screen",
        parameters=[
            {
                "frame_id": "map",
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([octomap_node, rviz_node])
