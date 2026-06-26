#!/bin/bash
set -e

WS="${GO2W_NAV_WS:-$HOME/go2w_nav_ws}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

export ROS_DOMAIN_ID=30
export LD_LIBRARY_PATH="$WS/third_party/unitree_sdk2/install/lib:$LD_LIBRARY_PATH"

ros2 run go2w_cmd_bridge go2w_cmd_bridge \
  --ros-args \
  -p network_interface:=enp2s0 \
  -p cmd_vel_topic:=/cmd_vel \
  -p enable_topic:=/go2w_cmd_enable \
  -p max_vx:=0.7 \
  -p max_vy:=0.5 \
  -p max_vyaw:=1.0 \
  -p cmd_timeout:=0.5 \
  -p start_enabled:=true \
  -p send_stopmove_on_disable:=true \
  -p speed_level:=-1
