#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, TwistStamped

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


ANSI_RED = "\033[1;31m"
ANSI_GREEN = "\033[1;32m"
ANSI_RESET = "\033[0m"


class NoisyController(Node):

    def __init__(self):
        super().__init__("noisy_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        self.declare_parameter("slip_start_sec", -1.0)
        self.declare_parameter("slip_duration_sec", 0.0)
        self.declare_parameter("slip_linear_scale", 1.0)
        self.declare_parameter("slip_angular_scale", 1.0)
        self.declare_parameter("paper_slip_mode", False)
        self.declare_parameter("nominal_cmd_topic", "/bumperbot_controller/cmd_vel_nominal")
        self.declare_parameter("nominal_cmd_timeout_sec", 0.75)
        self.declare_parameter("encoder_noise_std", 0.0002)
        self.declare_parameter("linear_bias", 1.0)
        self.declare_parameter("angular_bias", 1.0)
        self.declare_parameter("yaw_random_walk_std", 0.0002)
        self.declare_parameter("odom_pose_position_variance", 0.25)
        self.declare_parameter("odom_pose_yaw_variance", 0.25)
        self.declare_parameter("odom_twist_linear_variance", 0.10)
        self.declare_parameter("odom_twist_angular_variance", 0.10)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.slip_start_sec_ = self.get_parameter("slip_start_sec").get_parameter_value().double_value
        self.slip_duration_sec_ = self.get_parameter("slip_duration_sec").get_parameter_value().double_value
        self.slip_linear_scale_ = self.get_parameter("slip_linear_scale").get_parameter_value().double_value
        self.slip_angular_scale_ = self.get_parameter("slip_angular_scale").get_parameter_value().double_value
        self.paper_slip_mode_ = self.get_parameter("paper_slip_mode").get_parameter_value().bool_value
        self.nominal_cmd_topic_ = str(self.get_parameter("nominal_cmd_topic").value)
        self.nominal_cmd_timeout_sec_ = (
            self.get_parameter("nominal_cmd_timeout_sec").get_parameter_value().double_value
        )
        self.encoder_noise_std_ = self.get_parameter("encoder_noise_std").get_parameter_value().double_value
        self.linear_bias_ = self.get_parameter("linear_bias").get_parameter_value().double_value
        self.angular_bias_ = self.get_parameter("angular_bias").get_parameter_value().double_value
        self.yaw_random_walk_std_ = self.get_parameter("yaw_random_walk_std").get_parameter_value().double_value
        self.odom_pose_position_variance_ = self.get_parameter("odom_pose_position_variance").get_parameter_value().double_value
        self.odom_pose_yaw_variance_ = self.get_parameter("odom_pose_yaw_variance").get_parameter_value().double_value
        self.odom_twist_linear_variance_ = self.get_parameter("odom_twist_linear_variance").get_parameter_value().double_value
        self.odom_twist_angular_variance_ = self.get_parameter("odom_twist_angular_variance").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius_:.6f}")
        self.get_logger().info(f"Using wheel separation: {self.wheel_separation_:.6f}")
        self.get_logger().info(
            "Odom baseline: encoder_noise=%.6f, linear_bias=%.3f, angular_bias=%.3f, yaw_rw=%.6f"
            % (
                self.encoder_noise_std_,
                self.linear_bias_,
                self.angular_bias_,
                self.yaw_random_walk_std_,
            )
        )
        self.get_logger().info(
            "Odom slip case: start=%.2fs, duration=%.2fs, linear_scale=%.3f, angular_scale=%.3f"
            % (
                self.slip_start_sec_,
                self.slip_duration_sec_,
                self.slip_linear_scale_,
                self.slip_angular_scale_,
            )
        )
        self.get_logger().info(
            "Paper slip mode: enabled=%s, nominal_cmd_topic=%s, cmd_timeout=%.2fs"
            % (
                str(self.paper_slip_mode_),
                self.nominal_cmd_topic_,
                self.nominal_cmd_timeout_sec_,
            )
        )

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.joint_sub_ = self.create_subscription(
            JointState, "joint_states", self.jointCallback, 10
        )
        self.nominal_cmd_sub_ = self.create_subscription(
            TwistStamped, self.nominal_cmd_topic_, self.nominal_cmd_callback, 10
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
        self.start_time_ = None
        self.slip_was_active_ = False
        self.latest_nominal_linear_ = 0.0
        self.latest_nominal_angular_ = 0.0
        self.latest_nominal_cmd_stamp_sec_ = None
        self.paper_slip_missing_cmd_warned_ = False

    def nominal_cmd_callback(self, msg: TwistStamped):
        self.latest_nominal_linear_ = float(msg.twist.linear.x)
        self.latest_nominal_angular_ = float(msg.twist.angular.z)
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self.latest_nominal_cmd_stamp_sec_ = stamp_sec

    def jointCallback(self, msg: JointState):
        if len(msg.position) < 2:
            self.get_logger().warn("joint_states does not contain enough wheel positions.")
            return

        wheel_encoder_left = msg.position[0] + np.random.normal(0.0, self.encoder_noise_std_)
        wheel_encoder_right = msg.position[1] + np.random.normal(0.0, self.encoder_noise_std_)

        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_

        msg_time = Time.from_msg(msg.header.stamp)
        if self.start_time_ is None:
            self.start_time_ = msg_time

        dt = msg_time - self.prev_time_
        dt_sec = dt.nanoseconds / S_TO_NS

        if dt_sec <= 0.0:
            return

        self.left_wheel_prev_pos_ = wheel_encoder_left
        self.right_wheel_prev_pos_ = wheel_encoder_right
        self.prev_time_ = msg_time

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

        if self.is_slip_active(msg_time):
            if self.paper_slip_mode_ and self.has_recent_nominal_cmd(msg_time):
                # Paper-style slip approximation:
                # the real robot body can be slowed by the trajectory node, while
                # noisy odometry still integrates the nominal wheel command as if
                # the wheels kept spinning against a low-traction patch/obstacle.
                linear = self.latest_nominal_linear_ * self.linear_bias_ * self.slip_linear_scale_
                angular = self.latest_nominal_angular_ * self.angular_bias_ * self.slip_angular_scale_
                d_s = linear * dt_sec
                d_theta = angular * dt_sec
                slip_label = "PAPER-STYLE WHEEL-SPIN SLIP"
            else:
                linear *= self.slip_linear_scale_
                angular *= self.slip_angular_scale_
                d_s *= self.slip_linear_scale_
                d_theta *= self.slip_angular_scale_
                slip_label = "SIMULATED ODOMETRY SLIP"
                if self.paper_slip_mode_ and not self.paper_slip_missing_cmd_warned_:
                    self.get_logger().warn(
                        f"{ANSI_RED}===== PAPER SLIP MODE HAS NO RECENT NOMINAL CMD; "
                        f"falling back to scaled joint odometry. ====={ANSI_RESET}"
                    )
                    self.paper_slip_missing_cmd_warned_ = True
            if not self.slip_was_active_:
                self.get_logger().warn(
                    (
                        f"{ANSI_RED}===== {slip_label} STARTED ===== "
                        "linear_scale=%.3f, angular_scale=%.3f ====="
                        f"{ANSI_RESET}"
                    )
                    % (self.slip_linear_scale_, self.slip_angular_scale_)
                )
            self.slip_was_active_ = True
        elif self.slip_was_active_:
            self.get_logger().warn(
                f"{ANSI_GREEN}===== SIMULATED ODOMETRY SLIP ENDED ====={ANSI_RESET}"
            )
            self.slip_was_active_ = False

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
            self.odom_pose_position_variance_, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.odom_pose_position_variance_, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.odom_pose_yaw_variance_
        ]

        self.odom_msg_.twist.covariance = [
            self.odom_twist_linear_variance_, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.odom_twist_angular_variance_
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

    def is_slip_active(self, msg_time: Time) -> bool:
        if self.slip_start_sec_ < 0.0 or self.slip_duration_sec_ <= 0.0:
            return False
        if self.start_time_ is None:
            return False

        elapsed_sec = (msg_time - self.start_time_).nanoseconds / S_TO_NS
        return (
            elapsed_sec >= self.slip_start_sec_
            and elapsed_sec < self.slip_start_sec_ + self.slip_duration_sec_
        )

    def has_recent_nominal_cmd(self, msg_time: Time) -> bool:
        if self.latest_nominal_cmd_stamp_sec_ is None:
            return False
        if self.nominal_cmd_timeout_sec_ <= 0.0:
            return True
        msg_sec = msg_time.nanoseconds * 1e-9
        return (msg_sec - self.latest_nominal_cmd_stamp_sec_) <= self.nominal_cmd_timeout_sec_


def main():
    rclpy.init()
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
