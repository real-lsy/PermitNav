#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Ensure reliable message delivery
    history=HistoryPolicy.KEEP_LAST,        # Keep the last few messages
    depth=10                                # Increase buffer size
)

class LivoxLaserToPointcloud(Node):
    LIVOX_DTYPE = np.dtype([
        ('x', 'f4'),         # offset 0
        ('y', 'f4'),         # offset 4
        ('z', 'f4'),         # offset 8
        ('intensity', 'f4'), # offset 12
        ('tag', 'u1'),       # offset 16
        ('line', 'u1'),      # offset 17
        ('timestamp', 'f8')  # offset 18
        ])
    def __init__(self):
        super().__init__("Invert_Livox_Scan")

        xfer_format = self.declare_parameter("xfer_format", 0).value

        if xfer_format == 0:
            self.pub_scan = self.create_publisher(PointCloud2, "/livox/lidar", qos_profile=qos_profile)
            self.sub_scan = self.create_subscription(PointCloud2, "/livox/inverted_lidar", self.pointcloud2_callback, qos_profile=qos_profile)

        elif xfer_format == 1:
            self.pub_scan = self.create_publisher(CustomMsg, "/livox/lidar", qos_profile=qos_profile)
            self.sub_scan = self.create_subscription(CustomMsg, "/livox/inverted_lidar", self.custom_msg_callback, qos_profile=qos_profile)

        else:
            self.get_logger().error(f"Method undefined for xfer_format = {xfer_format}")
            self.destroy_node()
            
            return

        self.pub_imu = self.create_publisher(Imu, "/livox/imu", qos_profile=qos_profile)
        self.sub_imu = self.create_subscription(Imu, "/livox/inverted_imu", self.imu_callback, qos_profile=qos_profile)

    def pointcloud2_callback(self, msg: PointCloud2):
        # 1. Map the buffer to our structure (Zero-copy view)
        # We use frombuffer to interpret the raw bytes using our dtype
        data = np.frombuffer(msg.data, dtype=self.LIVOX_DTYPE).copy()

        # 2. Modify spatial coordinates
        # These operations are vectorized and extremely fast
        data['y'] = -data['y']
        data['z'] = -data['z']

        # 3. Reconstruct message
        # We copy the original message to keep all metadata (header, fields, etc.)
        out_msg = msg 
        out_msg.data = data.tobytes()
        
        self.pub_scan.publish(out_msg)

    def custom_msg_callback(self, msg: CustomMsg):
        for p in msg.points:
            p.y = -p.y
            p.z = -p.z
            
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.timebase = int(str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec))
        
        self.pub_scan.publish(msg)

    def imu_callback(self, msg: Imu):
        msg.angular_velocity.y = -msg.angular_velocity.y
        msg.angular_velocity.z = -msg.angular_velocity.z
        # msg.linear_acceleration.z = -msg.linear_acceleration.z
        
        # msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_imu.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxLaserToPointcloud()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()