#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsRepublisher(Node):
    def __init__(self):
        super().__init__("gps_republisher_node")

        self.sub = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            10
        )

        self.pub = self.create_publisher(
            NavSatFix,
            "/gps/fix_cov",
            10
        )

    def gps_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.position_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 1.0
        ]

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()