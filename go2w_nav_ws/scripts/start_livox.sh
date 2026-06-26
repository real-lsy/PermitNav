#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/go2w_nav_ws/install/setup.bash

export ROS_DOMAIN_ID=30

echo "Starting Livox MID360 driver..."
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

ros2 launch livox_ros_driver2 msg_MID360_launch.py
