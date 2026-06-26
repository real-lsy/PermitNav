#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ImuGToMs2(Node):
    def __init__(self):
        super().__init__('imu_g_to_ms2')

        self.scale = 9.80665

        # 原始 Livox IMU 可能是 BEST_EFFORT，所以订阅端用 BEST_EFFORT
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # FAST-LIO 通常用默认 RELIABLE 订阅，所以发布端用 RELIABLE
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.sub = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            sub_qos
        )

        self.pub = self.create_publisher(
            Imu,
            '/livox/imu_ms2',
            pub_qos
        )

        self.get_logger().info(
            'IMU converter started: /livox/imu -> /livox/imu_ms2, g to m/s^2, pub QoS RELIABLE'
        )

    def imu_callback(self, msg: Imu):
        out = Imu()
        out.header = msg.header

        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance

        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = msg.linear_acceleration.x * self.scale
        out.linear_acceleration.y = msg.linear_acceleration.y * self.scale
        out.linear_acceleration.z = msg.linear_acceleration.z * self.scale
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuGToMs2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
