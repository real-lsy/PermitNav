#!/usr/bin/env python3

import struct

import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def rgb_to_float(r: int, g: int, b: int) -> float:
    rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack("f", struct.pack("I", rgb_uint32))[0]


class PcdFilePublisher(Node):
    def __init__(self) -> None:
        super().__init__("pcd_file_publisher")
        self.declare_parameter("pcd_path", "")
        self.declare_parameter("topic", "/pcd_points")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_hz", 1.0)

        pcd_path = str(self.get_parameter("pcd_path").value)
        topic = str(self.get_parameter("topic").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        if not pcd_path:
            raise RuntimeError("pcd_path parameter is empty")

        self.pub = self.create_publisher(PointCloud2, topic, 10)
        self.cloud_msg = self.load_pcd(
            pcd_path=pcd_path,
            frame_id=str(self.get_parameter("frame_id").value),
        )
        self.timer = self.create_timer(max(1.0 / publish_hz, 1e-3), self.timer_cb)
        self.get_logger().info(f"Loaded PCD: {pcd_path}, publishing to {topic}")

    def load_pcd(self, pcd_path: str, frame_id: str) -> PointCloud2:
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points, dtype=np.float32)
        if points.size == 0:
            raise RuntimeError(f"PCD is empty: {pcd_path}")

        header = Header()
        header.frame_id = frame_id

        has_color = pcd.has_colors()
        if has_color:
            colors = np.asarray(pcd.colors, dtype=np.float32)
            cloud_data = []
            for idx in range(points.shape[0]):
                r = int(np.clip(colors[idx, 0] * 255.0, 0, 255))
                g = int(np.clip(colors[idx, 1] * 255.0, 0, 255))
                b = int(np.clip(colors[idx, 2] * 255.0, 0, 255))
                cloud_data.append(
                    (
                        float(points[idx, 0]),
                        float(points[idx, 1]),
                        float(points[idx, 2]),
                        rgb_to_float(r, g, b),
                    )
                )
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            return point_cloud2.create_cloud(header, fields, cloud_data)

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_data = points.astype(np.float32).tolist()
        return point_cloud2.create_cloud(header, fields, cloud_data)

    def timer_cb(self) -> None:
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud_msg)


def main() -> None:
    rclpy.init()
    node = PcdFilePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
