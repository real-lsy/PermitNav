#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class LocalizationToBaseLinkTF(Node):
    def __init__(self):
        super().__init__("localization_to_base_link_tf")
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            "/localization",
            self.cb,
            20
        )
        self.get_logger().info("Publishing direct TF: map -> base_link from /localization")

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = LocalizationToBaseLinkTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
