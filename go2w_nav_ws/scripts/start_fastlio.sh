#!/bin/bash
#关闭pcd保存
# sed -i 's/pcd_save_en:[[:space:]]*true/pcd_save_en: false/g' \
# ~/go2w_nav_ws/src/FAST_LIO_ROS2/config/mid360_nuc_mapping.yaml

# sed -i 's/pcd_save_en:[[:space:]]*true/pcd_save_en: false/g' \
# ~/go2w_nav_ws/install/fast_lio/share/fast_lio/config/mid360_nuc_mapping.yaml
set -e

source /opt/ros/humble/setup.bash
source ~/go2w_nav_ws/install/setup.bash

export ROS_DOMAIN_ID=30

echo "Starting FAST-LIO with MID360 config..."
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "PCD save is disabled."

ros2 launch fast_lio mapping.launch.py config_file:=mid360_nuc_mapping.yaml rviz:=false
