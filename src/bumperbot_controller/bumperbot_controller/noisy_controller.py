#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class NoisyController(Node):

    def __init__(self):
        super().__init__("noisy_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius_:.6f}")
        self.get_logger().info(f"Using wheel separation: {self.wheel_separation_:.6f}")

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Encoder noise is kept small so the dominant error is yaw drift
        self.encoder_noise_std_ = 0.0005

        # Small distance error
        self.linear_bias_ = 1.01

        # Moderate heading/dead-reckoning error for repeatable ANN dropout tests.
        self.angular_bias_ = 0.90

        # Small random yaw drift added at every update
        self.yaw_random_walk_std_ = 0.0008

        self.joint_sub_ = self.create_subscription(
            JointState, "joint_states", self.jointCallback, 10
        )

        self.odom_pub_ = self.create_publisher(
            Odometry, "bumperbot_controller/odom_noisy", 10
        )

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        self.prev_time_ = self.get_clock().now()

    def jointCallback(self, msg: JointState):
        if len(msg.position) < 2:
            self.get_logger().warn("joint_states does not contain enough wheel positions.")
            return

        wheel_encoder_left = msg.position[0] + np.random.normal(0.0, self.encoder_noise_std_)
        wheel_encoder_right = msg.position[1] + np.random.normal(0.0, self.encoder_noise_std_)

        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_

        dt = Time.from_msg(msg.header.stamp) - self.prev_time_
        dt_sec = dt.nanoseconds / S_TO_NS

        if dt_sec <= 0.0:
            return

        self.left_wheel_prev_pos_ = wheel_encoder_left
        self.right_wheel_prev_pos_ = wheel_encoder_right
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / dt_sec
        fi_right = dp_right / dt_sec

        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2.0
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2.0
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_

        # Apply dead-reckoning errors
        linear *= self.linear_bias_
        angular *= self.angular_bias_
        d_s *= self.linear_bias_
        d_theta *= self.angular_bias_

        # Add yaw drift to make IMU correction visible
        d_theta += np.random.normal(0.0, self.yaw_random_walk_std_)

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0.0, 0.0, self.theta_)

        self.odom_msg_.header.stamp = msg.header.stamp

        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.linear.y = 0.0
        self.odom_msg_.twist.twist.linear.z = 0.0

        self.odom_msg_.twist.twist.angular.x = 0.0
        self.odom_msg_.twist.twist.angular.y = 0.0
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_msg_.pose.covariance = [
            2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 2.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.5
        ]

        self.odom_msg_.twist.covariance = [
            0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.5
        ]

        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.header.stamp = msg.header.stamp
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]

        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
