#!/usr/bin/env python3

import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__("complementary_filter_node")

        self.declare_parameter("kf1_topic", "/odometry/kf_gps_imu")
        self.declare_parameter("kf2_topic", "/odometry/kf_gps_odom")
        self.declare_parameter("output_topic", "/odometry/kf_complementary")
        self.declare_parameter("alpha1", 0.5)
        self.declare_parameter("alpha2", 0.5)
        self.declare_parameter("publish_rate", 20.0)

        self.kf1_topic = self.get_parameter("kf1_topic").value
        self.kf2_topic = self.get_parameter("kf2_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.alpha1 = float(self.get_parameter("alpha1").value)
        self.alpha2 = float(self.get_parameter("alpha2").value)
        self.normalize_alphas()

        publish_rate = float(self.get_parameter("publish_rate").value)

        self.latest_kf1 = None
        self.latest_kf2 = None

        self.kf1_sub = self.create_subscription(Odometry, self.kf1_topic, self.kf1_callback, 10)
        self.kf2_sub = self.create_subscription(Odometry, self.kf2_topic, self.kf2_callback, 10)
        self.pub = self.create_publisher(Odometry, self.output_topic, 10)

        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self.publish_fused_odom)

        self.get_logger().info(
            f"Complementary filter started with alpha1={self.alpha1:.3f}, alpha2={self.alpha2:.3f}"
        )

    def normalize_alphas(self):
        total = self.alpha1 + self.alpha2
        if total <= 0.0:
            self.get_logger().warn("alpha1 + alpha2 must be positive; falling back to 0.5/0.5")
            self.alpha1 = 0.5
            self.alpha2 = 0.5
            return

        self.alpha1 /= total
        self.alpha2 /= total

    def kf1_callback(self, msg):
        self.latest_kf1 = msg

    def kf2_callback(self, msg):
        self.latest_kf2 = msg

    def publish_fused_odom(self):
        if self.latest_kf1 is None or self.latest_kf2 is None:
            return

        fused = copy.deepcopy(self.latest_kf1)
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.header.frame_id = self.latest_kf1.header.frame_id or "odom"
        fused.child_frame_id = self.latest_kf1.child_frame_id or "base_footprint"

        kf1_position = self.latest_kf1.pose.pose.position
        kf2_position = self.latest_kf2.pose.pose.position

        fused.pose.pose.position.x = self.alpha1 * kf1_position.x + self.alpha2 * kf2_position.x
        fused.pose.pose.position.y = self.alpha1 * kf1_position.y + self.alpha2 * kf2_position.y
        fused.pose.pose.position.z = 0.0

        fused.pose.pose.orientation = self.latest_kf1.pose.pose.orientation
        fused.twist = self.latest_kf2.twist
        fused.pose.covariance = self.blend_covariances(
            self.latest_kf1.pose.covariance,
            self.latest_kf2.pose.covariance,
        )
        fused.twist.covariance = self.latest_kf2.twist.covariance

        self.pub.publish(fused)

    def blend_covariances(self, covariance1, covariance2):
        return [
            self.alpha1 * self.alpha1 * c1 + self.alpha2 * self.alpha2 * c2
            for c1, c2 in zip(covariance1, covariance2)
        ]


def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()