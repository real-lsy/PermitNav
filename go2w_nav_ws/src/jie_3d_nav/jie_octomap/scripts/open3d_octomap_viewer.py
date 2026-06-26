#!/usr/bin/env python3

import time
from typing import Optional

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Open3DOctomapViewer(Node):
    def __init__(self) -> None:
        super().__init__("open3d_octomap_viewer")

        self.declare_parameter("cloud_topic", "/octomap_points")
        self.declare_parameter("point_size", 4.0)

        cloud_topic = self.get_parameter("cloud_topic").get_parameter_value().string_value

        self._latest_points: Optional[np.ndarray] = None
        self._dirty = False
        self._cloud_count = 0

        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, 10)
        self.get_logger().info(f"Open3D viewer started. cloud_topic={cloud_topic}")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self._cloud_count += 1

        points_iter = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        points_list = list(points_iter)
        if len(points_list) == 0:
            return

        self._latest_points = np.asarray(points_list, dtype=np.float64)
        self._dirty = True
        if self._cloud_count % 10 == 0:
            self.get_logger().info(
                f"Received cloud messages: {self._cloud_count}, points={self._latest_points.shape[0]}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Open3DOctomapViewer()

    vis = o3d.visualization.Visualizer()
    ok = vis.create_window(window_name="Open3D OctoMap Viewer", width=1280, height=800)
    if not ok:
        node.get_logger().error("Open3D window create failed. Check DISPLAY/GL driver.")
        node.destroy_node()
        rclpy.shutdown()
        return

    pcd = o3d.geometry.PointCloud()
    geometry_added = False

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if node._dirty and node._latest_points is not None:
                points = node._latest_points
                node._dirty = False
                pcd.points = o3d.utility.Vector3dVector(points)
                colors = np.zeros_like(points)
                colors[:, 0] = 1.0
                colors[:, 1] = 0.55
                colors[:, 2] = 0.2
                pcd.colors = o3d.utility.Vector3dVector(colors)
                if not geometry_added:
                    vis.add_geometry(pcd)
                    vis.get_view_control().set_zoom(0.35)
                    geometry_added = True
                else:
                    vis.update_geometry(pcd)

            keep_running = vis.poll_events()
            vis.update_renderer()
            if not keep_running:
                node.get_logger().info("Open3D window closed.")
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        vis.destroy_window()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
