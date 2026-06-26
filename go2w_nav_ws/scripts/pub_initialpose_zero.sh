#!/bin/bash
set -e

WS="${GO2W_NAV_WS:-$HOME/go2w_nav_ws}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=30

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    frame_id: 'map'
  },
  pose: {
    pose: {
      position: {
        x: 0.0,
        y: 0.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    },
    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.1
    ]
  }
}"
