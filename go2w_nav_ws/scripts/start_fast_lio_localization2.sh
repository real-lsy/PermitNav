#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source "$HOME/go2w_nav_ws/install/setup.bash"
export ROS_DOMAIN_ID=30

MAP_PCD="$HOME/go2w_nav_ws/maps/localization/big_localization_raw.pcd"

if [ ! -f "$MAP_PCD" ]; then
  echo "[ERROR] PCD map not found:"
  echo "  $MAP_PCD"
  exit 1
fi

echo "[INFO] Using localization map:"
echo "  $MAP_PCD"

ros2 launch fast_lio_localization localization_only.launch.py \
  map:="$MAP_PCD" \
  pcd_map_topic:=/map
