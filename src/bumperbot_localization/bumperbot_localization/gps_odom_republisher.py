#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GpsOdomRepublisher(Node):
    def __init__(self):
        super().__init__("gps_odom_republisher_node")

        self.sub = self.create_subscription(
            Odometry,
            "/bumperbot_controller/odom",
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(
            Odometry,
            "/odometry/gps_sim",
            10
        )

    def odom_callback(self, msg):
        gps = Odometry()
        gps.header.stamp = msg.header.stamp
        gps.header.frame_id = "odom"
        gps.child_frame_id = "base_footprint"

        gps.pose.pose.position.x = msg.pose.pose.position.x + np.random.normal(0.0,0.1)
        gps.pose.pose.position.y = msg.pose.pose.position.y + np.random.normal(0.0,0.1)
        gps.pose.pose.position.z = 0.0

        gps.pose.pose.orientation.w = 1.0

        gps.pose.covariance = [
            0.005, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.005, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0
        ]

        self.pub.publish(gps)


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()