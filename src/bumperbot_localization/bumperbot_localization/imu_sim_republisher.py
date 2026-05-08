#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class ImuSimRepublisher(Node):

    def __init__(self):
        super().__init__("imu_sim_republisher_node")

        # --- Noise parameters ---
        self.yaw_noise_std_ = 0.05
        self.yaw_rate_noise_std_ = 0.03

        # --- Slowly accumulating drift (bias) ---
        self.yaw_bias_ = 0.0

        self.prev_yaw_ = None
        self.prev_time_ = None

        self.imu_pub_ = self.create_publisher(Imu, "/imu_sim", 10)

        self.odom_sub_ = self.create_subscription(
            Odometry,
            "/bumperbot_controller/odom",
            self.odom_callback,
            10
        )

        self.get_logger().info("IMU sim republisher started. Publishing /imu_sim")

    def odom_callback(self, msg: Odometry):
        q_in = msg.pose.pose.orientation

        roll, pitch, yaw = euler_from_quaternion([
            q_in.x,
            q_in.y,
            q_in.z,
            q_in.w
        ])

        # --- Add slow drift (bias) ---
        self.yaw_bias_ += np.random.normal(0.0, 0.002)
        yaw += self.yaw_bias_

        # --- Add measurement noise ---
        noisy_yaw = yaw + np.random.normal(0.0, self.yaw_noise_std_)

        q_out = quaternion_from_euler(0.0, 0.0, noisy_yaw)

        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = "imu_link"

        imu.orientation.x = q_out[0]
        imu.orientation.y = q_out[1]
        imu.orientation.z = q_out[2]
        imu.orientation.w = q_out[3]

        # --- Time ---
        now_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # --- Yaw rate computation ---
        yaw_rate = 0.0
        if self.prev_yaw_ is not None and self.prev_time_ is not None:
            dt = now_time - self.prev_time_
            if dt > 0.0:
                dyaw = math.atan2(
                    math.sin(noisy_yaw - self.prev_yaw_),
                    math.cos(noisy_yaw - self.prev_yaw_)
                )
                yaw_rate = dyaw / dt

        self.prev_yaw_ = noisy_yaw
        self.prev_time_ = now_time

        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = yaw_rate + np.random.normal(0.0, self.yaw_rate_noise_std_)

        # --- Not used ---
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0

        # --- Covariances ---
        imu.orientation_covariance = [
            999.0, 0.0, 0.0,
            0.0, 999.0, 0.0,
            0.0, 0.0, 0.2
        ]

        imu.angular_velocity_covariance = [
            999.0, 0.0, 0.0,
            0.0, 999.0, 0.0,
            0.0, 0.0, 0.2
        ]

        imu.linear_acceleration_covariance = [
            999.0, 0.0, 0.0,
            0.0, 999.0, 0.0,
            0.0, 0.0, 999.0
        ]

        self.imu_pub_.publish(imu)


def main():
    rclpy.init()
    node = ImuSimRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()