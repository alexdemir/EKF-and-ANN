#!/usr/bin/env python3

from __future__ import annotations

import os
import re
import shutil
import stat
import sys
from datetime import datetime
from pathlib import Path


ROUTE_PY = r'''#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ScriptedTrajectory(Node):
    """Waypoint based square route for repeatable ANN/FLS tests."""

    def __init__(self):
        super().__init__("scripted_trajectory_node")

        self.declare_parameter("cmd_topic", "/bumperbot_controller/cmd_vel")
        self.declare_parameter("odom_topic", "/bumperbot_controller/odom")
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("profile", "paper_fls_waypoint")
        self.declare_parameter("side_length", 3.0)
        self.declare_parameter("laps", 6)
        self.declare_parameter("max_linear_speed", 0.88)
        self.declare_parameter("max_angular_speed", 1.60)
        self.declare_parameter("heading_gain", 2.1)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("corner_slow_radius", 0.90)
        self.declare_parameter("minimum_turn_linear_scale", 0.24)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("stop_at_end", True)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.frequency = float(self.get_parameter("frequency").value)
        self.profile = str(self.get_parameter("profile").value)
        self.side_length = float(self.get_parameter("side_length").value)
        self.laps = int(self.get_parameter("laps").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.corner_slow_radius = float(self.get_parameter("corner_slow_radius").value)
        self.minimum_turn_linear_scale = float(
            self.get_parameter("minimum_turn_linear_scale").value
        )
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.stop_at_end = bool(self.get_parameter("stop_at_end").value)

        self.max_linear_speed *= self.speed_scale
        self.max_angular_speed *= max(0.2, self.speed_scale)

        self.publisher = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.timer = self.create_timer(1.0 / max(self.frequency, 1.0), self.timer_callback)

        self.latest_pose = None
        self.origin = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.finished = False
        self.start_time = self.clock_seconds()
        self.last_status_time = -999.0

        self.get_logger().info(
            "Scripted waypoint route ready: "
            f"profile={self.profile}, side_length={self.side_length:.2f} m, "
            f"laps={self.laps}, max_v={self.max_linear_speed:.2f} m/s, "
            f"max_w={self.max_angular_speed:.2f} rad/s, "
            f"goal_tolerance={self.goal_tolerance:.2f} m"
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = (x, y, yaw)
        if self.origin is None:
            self.origin = self.latest_pose
            self.build_square_waypoints()

    def build_square_waypoints(self):
        ox, oy, oyaw = self.origin
        local_points = []
        for _ in range(max(self.laps, 1)):
            local_points.extend(
                [
                    (self.side_length, 0.0),
                    (self.side_length, self.side_length),
                    (0.0, self.side_length),
                    (0.0, 0.0),
                ]
            )

        c = math.cos(oyaw)
        s = math.sin(oyaw)
        self.waypoints = [
            (ox + c * lx - s * ly, oy + s * lx + c * ly) for lx, ly in local_points
        ]
        self.current_waypoint_index = 0
        self.get_logger().warn(
            "\033[96m===== SCRIPTED WAYPOINT ROUTE STARTED ===== "
            f"{len(self.waypoints)} waypoints, initial yaw={oyaw:.2f} rad. =====\033[0m"
        )

    def timer_callback(self):
        now = self.clock_seconds()
        if self.latest_pose is None or self.origin is None:
            self.publish_cmd(0.0, 0.0)
            return

        if now - self.start_time < self.start_delay_sec:
            self.publish_cmd(0.0, 0.0)
            return

        if self.finished:
            if self.stop_at_end:
                self.publish_cmd(0.0, 0.0)
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.finished = True
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                "\033[92m===== SCRIPTED WAYPOINT ROUTE FINISHED =====\033[0m"
            )
            return

        x, y, yaw = self.latest_pose
        target_x, target_y = self.waypoints[self.current_waypoint_index]
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        if distance <= self.goal_tolerance:
            self.current_waypoint_index += 1
            self.get_logger().warn(
                "\033[94m===== WAYPOINT REACHED ===== "
                f"{self.current_waypoint_index}/{len(self.waypoints)}, "
                f"distance={distance:.2f} m. =====\033[0m"
            )
            return

        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_heading - yaw)
        angular = clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        heading_scale = 1.0 - min(abs(heading_error), math.pi) / math.pi
        heading_scale = max(self.minimum_turn_linear_scale, heading_scale)
        distance_scale = clamp(distance / max(self.corner_slow_radius, 0.1), 0.35, 1.0)
        linear = self.max_linear_speed * heading_scale * distance_scale

        self.publish_cmd(linear, angular)
        if now - self.last_status_time > 4.0:
            self.last_status_time = now
            self.get_logger().info(
                "Scripted waypoint tracking: "
                f"wp={self.current_waypoint_index + 1}/{len(self.waypoints)}, "
                f"dist={distance:.2f} m, heading_error={heading_error:.2f} rad, "
                f"cmd_v={linear:.2f}, cmd_w={angular:.2f}"
            )

    def publish_cmd(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.publisher.publish(msg)

    def clock_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
'''


def backup(path: Path, stamp: str) -> None:
    if path.exists():
        shutil.copy2(path, path.with_suffix(path.suffix + f".bak_{stamp}"))


def replace_once(text: str, old: str, new: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Could not find insertion point:\n{old[:160]}")
    return text.replace(old, new, 1)


def set_yaml_value(text: str, key: str, value: str) -> str:
    pattern = rf"^(\s*{re.escape(key)}:\s*).*$"
    new_text, count = re.subn(pattern, rf"\g<1>{value}", text, flags=re.MULTILINE)
    if count == 0:
        raise RuntimeError(f"Missing YAML key: {key}")
    return new_text


def set_py_default(text: str, key: str, value: str) -> str:
    if value.startswith('"'):
        replacement = rf'self.declare_parameter("{key}", {value})'
    else:
        replacement = rf'self.declare_parameter("{key}", {value})'
    pattern = rf'self\.declare_parameter\("{re.escape(key)}", [^)]+\)'
    new_text, count = re.subn(pattern, replacement, text, count=1)
    if count == 0:
        raise RuntimeError(f"Missing Python default: {key}")
    return new_text


def update_controller_launch(path: Path, stamp: str) -> None:
    backup(path, stamp)
    text = path.read_text(encoding="utf-8")
    text = text.replace('default_value="paper_fls"', 'default_value="paper_fls_waypoint"')

    if "scripted_side_length_arg" not in text:
        text = replace_once(
            text,
            '''    scripted_start_delay_sec_arg = DeclareLaunchArgument(
        "scripted_start_delay_sec",
        default_value="2.0",
    )
''',
            '''    scripted_start_delay_sec_arg = DeclareLaunchArgument(
        "scripted_start_delay_sec",
        default_value="2.0",
    )
    scripted_side_length_arg = DeclareLaunchArgument(
        "scripted_side_length",
        default_value="3.0",
        description="Side length in meters for the waypoint square route.",
    )
    scripted_laps_arg = DeclareLaunchArgument(
        "scripted_laps",
        default_value="6",
        description="Number of square laps for the waypoint route.",
    )
    scripted_max_linear_speed_arg = DeclareLaunchArgument(
        "scripted_max_linear_speed",
        default_value="0.88",
        description="Maximum scripted forward speed in m/s.",
    )
    scripted_max_angular_speed_arg = DeclareLaunchArgument(
        "scripted_max_angular_speed",
        default_value="1.60",
        description="Maximum scripted yaw rate in rad/s.",
    )
    scripted_goal_tolerance_arg = DeclareLaunchArgument(
        "scripted_goal_tolerance",
        default_value="0.35",
        description="Waypoint acceptance radius in meters.",
    )
''',
        )

    if "scripted_side_length = LaunchConfiguration" not in text:
        text = replace_once(
            text,
            '''    scripted_trajectory_profile = LaunchConfiguration("scripted_trajectory_profile")
    scripted_speed_scale = LaunchConfiguration("scripted_speed_scale")
    scripted_start_delay_sec = LaunchConfiguration("scripted_start_delay_sec")
''',
            '''    scripted_trajectory_profile = LaunchConfiguration("scripted_trajectory_profile")
    scripted_speed_scale = LaunchConfiguration("scripted_speed_scale")
    scripted_start_delay_sec = LaunchConfiguration("scripted_start_delay_sec")
    scripted_side_length = LaunchConfiguration("scripted_side_length")
    scripted_laps = LaunchConfiguration("scripted_laps")
    scripted_max_linear_speed = LaunchConfiguration("scripted_max_linear_speed")
    scripted_max_angular_speed = LaunchConfiguration("scripted_max_angular_speed")
    scripted_goal_tolerance = LaunchConfiguration("scripted_goal_tolerance")
''',
        )

    if '"side_length": scripted_side_length' not in text:
        text = replace_once(
            text,
            '''                "profile": scripted_trajectory_profile,
                "speed_scale": scripted_speed_scale,
                "start_delay_sec": scripted_start_delay_sec,
                "use_sim_time": use_sim_time,
''',
            '''                "profile": scripted_trajectory_profile,
                "speed_scale": scripted_speed_scale,
                "start_delay_sec": scripted_start_delay_sec,
                "side_length": scripted_side_length,
                "laps": scripted_laps,
                "max_linear_speed": scripted_max_linear_speed,
                "max_angular_speed": scripted_max_angular_speed,
                "goal_tolerance": scripted_goal_tolerance,
                "use_sim_time": use_sim_time,
''',
        )

    if "scripted_side_length_arg," not in text:
        text = replace_once(
            text,
            '''            scripted_trajectory_profile_arg,
            scripted_speed_scale_arg,
            scripted_start_delay_sec_arg,
''',
            '''            scripted_trajectory_profile_arg,
            scripted_speed_scale_arg,
            scripted_start_delay_sec_arg,
            scripted_side_length_arg,
            scripted_laps_arg,
            scripted_max_linear_speed_arg,
            scripted_max_angular_speed_arg,
            scripted_goal_tolerance_arg,
''',
        )
    path.write_text(text, encoding="utf-8")


def update_cmake(path: Path, stamp: str) -> None:
    backup(path, stamp)
    text = path.read_text(encoding="utf-8")
    if "${PROJECT_NAME}/scripted_trajectory.py" not in text:
        text = text.replace(
            "  ${PROJECT_NAME}/noisy_controller.py\n",
            "  ${PROJECT_NAME}/noisy_controller.py\n  ${PROJECT_NAME}/scripted_trajectory.py\n",
            1,
        )
    path.write_text(text, encoding="utf-8")


def main() -> int:
    target = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path.cwd()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if not (target / "bumperbot_controller").exists():
        raise SystemExit(
            "Run this from ~/bumperbot_ws/src or pass that path as the first argument."
        )

    route_path = target / "bumperbot_controller/bumperbot_controller/scripted_trajectory.py"
    backup(route_path, stamp)
    route_path.write_text(ROUTE_PY, encoding="utf-8")
    route_path.chmod(route_path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    update_cmake(target / "bumperbot_controller/CMakeLists.txt", stamp)
    update_controller_launch(target / "bumperbot_controller/launch/controller.launch.py", stamp)

    fls_yaml = target / "bumperbot_localization/config/fuzzy_localization.yaml"
    backup(fls_yaml, stamp)
    text = fls_yaml.read_text(encoding="utf-8")
    for key, value in {
        "small_error": "1.35",
        "large_error": "2.10",
        "ann_weight_zero": "0.0",
        "ann_weight_small": "0.25",
        "ann_weight_large": "1.0",
        "adaptive_error_enabled": "false",
        "velocity_error_source": "max",
        "use_normalized_velocity_error": "false",
        "alpha_rise_tau": "0.12",
        "alpha_fall_tau": "1.20",
        "disagreement_boost_enabled": "false",
    }.items():
        text = set_yaml_value(text, key, value)
    fls_yaml.write_text(text, encoding="utf-8")

    fls_py = target / "bumperbot_localization/bumperbot_localization/fuzzy_localization.py"
    backup(fls_py, stamp)
    text = fls_py.read_text(encoding="utf-8")
    for key, value in {
        "small_error": "1.35",
        "large_error": "2.10",
        "ann_weight_zero": "0.0",
        "ann_weight_small": "0.25",
        "ann_weight_large": "1.0",
        "adaptive_error_enabled": "False",
        "velocity_error_source": '"max"',
        "use_normalized_velocity_error": "False",
        "alpha_rise_tau": "0.12",
        "alpha_fall_tau": "1.20",
        "disagreement_boost_enabled": "False",
    }.items():
        text = set_py_default(text, key, value)
    fls_py.write_text(text, encoding="utf-8")

    print("Updated waypoint scripted route and FLS gate.")
    print(f"Target: {target}")
    print("Next:")
    print("  cd ~/bumperbot_ws")
    print("  colcon build --packages-select bumperbot_localization bumperbot_controller")
    print("  source install/setup.bash")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
