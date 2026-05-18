#!/usr/bin/env python3

import csv
import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class EkfOutputLogger(Node):
    def __init__(self):
        super().__init__("ekf_output_logger_node")

        self.declare_parameter("ekf_topic", "/odometry/kf_gps_odom")
        self.declare_parameter("pseudo_topic", "/odometry/gps_or_ann")
        self.declare_parameter("gps_topic", "/odometry/gps_sim")
        self.declare_parameter("odom_topic", "/bumperbot_controller/odom_noisy")
        self.declare_parameter("ann_topic", "/odometry/ann")
        self.declare_parameter("kf_imu_topic", "/odometry/kf_gps_imu")
        self.declare_parameter("complementary_topic", "/odometry/kf_complementary")
        self.declare_parameter("final_topic", "/odometry/fuzzy_localization")
        self.declare_parameter("log_path", "~/bumperbot_ws/src/ekf_output_log.csv")

        self.ekf_topic = self.get_parameter("ekf_topic").value
        self.pseudo_topic = self.get_parameter("pseudo_topic").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.ann_topic = self.get_parameter("ann_topic").value
        self.kf_imu_topic = self.get_parameter("kf_imu_topic").value
        self.complementary_topic = self.get_parameter("complementary_topic").value
        self.final_topic = self.get_parameter("final_topic").value
        self.log_path = os.path.expanduser(str(self.get_parameter("log_path").value))

        self.latest_pseudo = None
        self.latest_gps = None
        self.latest_odom = None
        self.latest_ann = None
        self.latest_kf_imu = None
        self.latest_complementary = None
        self.latest_final = None

        self.log_file = None
        self.log_writer = None

        self.create_subscription(Odometry, self.ekf_topic, self.ekf_callback, 20)
        self.create_subscription(Odometry, self.pseudo_topic, self.pseudo_callback, 20)
        self.create_subscription(Odometry, self.gps_topic, self.gps_callback, 20)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(Odometry, self.ann_topic, self.ann_callback, 20)
        self.create_subscription(Odometry, self.kf_imu_topic, self.kf_imu_callback, 20)
        self.create_subscription(Odometry, self.complementary_topic, self.complementary_callback, 20)
        self.create_subscription(Odometry, self.final_topic, self.final_callback, 20)

        self.open_log_writer()
        self.get_logger().info(f"EKF output logging enabled: {self.log_path}")

    def pseudo_callback(self, msg):
        self.latest_pseudo = msg

    def gps_callback(self, msg):
        self.latest_gps = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def ann_callback(self, msg):
        self.latest_ann = msg

    def kf_imu_callback(self, msg):
        self.latest_kf_imu = msg

    def complementary_callback(self, msg):
        self.latest_complementary = msg

    def final_callback(self, msg):
        self.latest_final = msg

    def ekf_callback(self, msg):
        if self.log_writer is None:
            return

        ekf_xy = self.xy_from_msg(msg)
        pseudo_xy = self.xy_from_msg(self.latest_pseudo)
        gps_xy = self.xy_from_msg(self.latest_gps)
        odom_xy = self.xy_from_msg(self.latest_odom)
        ann_xy = self.xy_from_msg(self.latest_ann)
        kf_imu_xy = self.xy_from_msg(self.latest_kf_imu)
        complementary_xy = self.xy_from_msg(self.latest_complementary)
        final_xy = self.xy_from_msg(self.latest_final)

        self.log_writer.writerow([
            f"{self.stamp_to_seconds(msg.header.stamp):.9f}",
            *self.format_xy(ekf_xy),
            *self.format_xy(pseudo_xy),
            *self.format_xy(gps_xy),
            *self.format_xy(odom_xy),
            *self.format_xy(ann_xy),
            *self.format_xy(kf_imu_xy),
            *self.format_xy(complementary_xy),
            *self.format_xy(final_xy),
            self.format_error(ekf_xy, gps_xy),
            self.format_error(pseudo_xy, gps_xy),
            self.format_error(odom_xy, gps_xy),
            self.format_error(ann_xy, gps_xy),
            self.format_error(kf_imu_xy, gps_xy),
            self.format_error(complementary_xy, gps_xy),
            self.format_error(final_xy, gps_xy),
        ])
        self.log_file.flush()

    def open_log_writer(self):
        log_dir = os.path.dirname(self.log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(self.log_path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "stamp",
            "ekf_x",
            "ekf_y",
            "pseudo_x",
            "pseudo_y",
            "gps_x",
            "gps_y",
            "odom_x",
            "odom_y",
            "ann_x",
            "ann_y",
            "kf_imu_x",
            "kf_imu_y",
            "complementary_x",
            "complementary_y",
            "final_x",
            "final_y",
            "ekf_error_m",
            "pseudo_error_m",
            "odom_error_m",
            "ann_error_m",
            "kf_imu_error_m",
            "complementary_error_m",
            "final_error_m",
        ])
        self.log_file.flush()

    def close_log_writer(self):
        if self.log_file is None:
            return
        self.log_file.flush()
        self.log_file.close()
        self.log_file = None
        self.log_writer = None

    @staticmethod
    def xy_from_msg(msg):
        if msg is None:
            return (math.nan, math.nan)
        return (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    @staticmethod
    def format_xy(xy):
        return [f"{xy[0]:.9f}", f"{xy[1]:.9f}"]

    @staticmethod
    def format_error(a, b):
        if not all(math.isfinite(value) for value in (*a, *b)):
            return "nan"
        return f"{math.hypot(a[0] - b[0], a[1] - b[1]):.9f}"

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = EkfOutputLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_log_writer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
