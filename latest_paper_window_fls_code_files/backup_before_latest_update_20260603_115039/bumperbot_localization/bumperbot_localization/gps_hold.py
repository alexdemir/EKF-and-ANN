#!/usr/bin/env python3

import copy
import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


ANSI_RED = "\033[1;31m"
ANSI_GREEN = "\033[1;32m"
ANSI_RESET = "\033[0m"


class GpsHold(Node):
    def __init__(self):
        super().__init__("gps_hold_node")

        dynamic_float_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("gps_topic", "/odometry/gps_sim")
        self.declare_parameter("output_topic", "/odometry/gps_hold")
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("gps_timeout", 0.5)
        self.declare_parameter(
            "force_gps_dropout_after_sec",
            -1.0,
            dynamic_float_descriptor,
        )
        self.declare_parameter(
            "force_gps_dropout_duration_sec",
            0.0,
            dynamic_float_descriptor,
        )
        self.declare_parameter("forced_dropout_windows", "")
        self.declare_parameter("hold_position_variance", 4.0)
        self.declare_parameter("publish_hold_once", True)
        self.declare_parameter("save_log", True)
        self.declare_parameter("log_path", "~/bumperbot_ws/src/gps_hold_log.csv")

        self.gps_topic = self.get_parameter("gps_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frequency = float(self.get_parameter("frequency").value)
        self.gps_timeout = float(self.get_parameter("gps_timeout").value)
        self.force_gps_dropout_after_sec = float(
            self.get_parameter("force_gps_dropout_after_sec").value
        )
        self.force_gps_dropout_duration_sec = float(
            self.get_parameter("force_gps_dropout_duration_sec").value
        )
        self.forced_dropout_windows = self.parse_forced_dropout_windows(
            str(self.get_parameter("forced_dropout_windows").value)
        )
        self.hold_position_variance = float(self.get_parameter("hold_position_variance").value)
        self.publish_hold_once = bool(self.get_parameter("publish_hold_once").value)
        self.save_log = bool(self.get_parameter("save_log").value)
        self.log_path = os.path.expanduser(str(self.get_parameter("log_path").value))

        self.latest_gps = None
        self.last_gps_stamp = None
        self.last_valid_gps = None
        self.start_time = None
        self.was_available = False
        self.had_gps_dropout = False
        self.hold_published = False

        self.log_file = None

        self.create_subscription(Odometry, self.gps_topic, self.gps_callback, 20)
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.frequency, 1.0), self.timer_callback)

        self.open_log()
        self.get_logger().info(
            "GPS hold node started: publishing real GPS until dropout, then last GPS "
            f"position, force_after={self.force_gps_dropout_after_sec:.3f}s, "
            f"forced_windows={self.forced_dropout_windows}"
        )

    def gps_callback(self, msg):
        self.latest_gps = msg
        self.last_gps_stamp = self.stamp_to_seconds(msg.header.stamp)
        if not self.is_forced_dropout(self.clock_seconds()):
            self.last_valid_gps = copy.deepcopy(msg)

    def timer_callback(self):
        now = self.clock_seconds()
        if self.start_time is None:
            self.start_time = now

        gps_available = self.is_gps_available(now)
        if gps_available and self.latest_gps is not None:
            if self.had_gps_dropout:
                self.get_logger().info(
                    f"{ANSI_GREEN}===== GPS RESTORED ===== GPS hold switched "
                    f"back to live GPS measurements for KF2. ====={ANSI_RESET}"
                )
                self.had_gps_dropout = False
            self.publisher.publish(copy.deepcopy(self.latest_gps))
            self.last_valid_gps = copy.deepcopy(self.latest_gps)
            self.was_available = True
            self.hold_published = False
            self.write_log("gps", self.latest_gps)
            return

        if self.was_available:
            self.get_logger().warn(
                f"{ANSI_RED}===== GPS DROPOUT ===== GPS hold is seeding KF2 "
                f"with last GPS position. ====={ANSI_RESET}"
            )
            self.was_available = False
            self.had_gps_dropout = True

        if self.last_valid_gps is None:
            return

        if self.publish_hold_once and self.hold_published:
            self.write_log("dropout_no_gps", self.last_valid_gps)
            return

        held = copy.deepcopy(self.last_valid_gps)
        held.header.stamp = self.get_clock().now().to_msg()
        held.pose.covariance[0] = self.hold_position_variance
        held.pose.covariance[7] = self.hold_position_variance
        self.publisher.publish(held)
        self.hold_published = True
        self.write_log("hold", held)

    def is_gps_available(self, now):
        if self.is_forced_dropout(now):
            return False
        if self.gps_timeout <= 0.0:
            return False
        if self.last_gps_stamp is None:
            return False
        return 0.0 <= now - self.last_gps_stamp <= self.gps_timeout

    def is_forced_dropout(self, now):
        if self.start_time is None:
            return False
        if self.forced_dropout_windows:
            elapsed = now - self.start_time
            for start, duration in self.forced_dropout_windows:
                if elapsed < start:
                    continue
                if duration <= 0.0 or elapsed <= start + duration:
                    return True
            return False
        if self.force_gps_dropout_after_sec < 0.0:
            return False
        elapsed = now - self.start_time
        if elapsed < self.force_gps_dropout_after_sec:
            return False
        if self.force_gps_dropout_duration_sec <= 0.0:
            return True
        return elapsed <= (
            self.force_gps_dropout_after_sec + self.force_gps_dropout_duration_sec
        )

    def parse_forced_dropout_windows(self, text):
        windows = []
        text = (text or "").strip()
        if not text:
            return windows
        for item in text.split(","):
            item = item.strip()
            if not item:
                continue
            try:
                if ":" in item:
                    start_text, duration_text = item.split(":", 1)
                else:
                    start_text, duration_text = item, "0.0"
                start = float(start_text)
                duration = float(duration_text)
            except ValueError:
                self.get_logger().warn(
                    f"Ignoring invalid forced dropout window '{item}'. "
                    "Use 'start:duration,start:duration'."
                )
                continue
            if start < 0.0:
                self.get_logger().warn(
                    f"Ignoring forced dropout window with negative start: '{item}'."
                )
                continue
            windows.append((start, duration))
        windows.sort(key=lambda window: window[0])
        return windows

    def open_log(self):
        if not self.save_log:
            return
        log_dir = os.path.dirname(self.log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(self.log_path, "w", newline="")
        self.log_file.write("stamp,source,forced_dropout,x,y\n")
        self.log_file.flush()

    def write_log(self, source, msg):
        if self.log_file is None:
            return
        self.log_file.write(
            f"{self.clock_seconds():.9f},{source},{self.is_forced_dropout(self.clock_seconds())},"
            f"{msg.pose.pose.position.x:.9f},{msg.pose.pose.position.y:.9f}\n"
        )
        self.log_file.flush()

    def close_log(self):
        if self.log_file is None:
            return
        self.log_file.flush()
        self.log_file.close()
        self.log_file = None

    def clock_seconds(self):
        return self.stamp_to_seconds(self.get_clock().now().to_msg())

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = GpsHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
