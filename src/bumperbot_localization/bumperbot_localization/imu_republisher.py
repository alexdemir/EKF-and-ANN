#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher_node')
        self.imu_pub = self.create_publisher(Imu, "imu_ekf", 10)
        self.imu_sub = self.create_subscription(Imu, "imu/out", self.imu_callback, 10)

    def imu_callback(self, imu: Imu):
        imu.header.stamp = self.get_clock().now().to_msg()

        # EKF'nin sensörlere güvenebilmesi için covariance ekliyoruz
        # Planar robot için yaw ve angular z daha kritik
        imu.orientation_covariance = [
            0.08, 0.0, 0.0,
            0.0,  0.08, 0.0,
            0.0,  0.0,  0.015
        ]

        imu.angular_velocity_covariance = [
            0.03, 0.0,  0.0,
            0.0,  0.03, 0.0,
            0.0,  0.0,  0.01
        ]

        imu.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()