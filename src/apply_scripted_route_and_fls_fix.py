#!/usr/bin/env python3

from __future__ import annotations

import os
import re
import shutil
import stat
import sys
from datetime import datetime
from pathlib import Path


SCRIPTED_TRAJECTORY = r'''#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


ANSI_CYAN = "\033[1;36m"
ANSI_GREEN = "\033[1;32m"
ANSI_YELLOW = "\033[1;33m"
ANSI_RESET = "\033[0m"


class ScriptedTrajectory(Node):
    """Publish repeatable cmd_vel profiles for paper-style outage tests."""

    def __init__(self):
        super().__init__("scripted_trajectory_node")

        self.declare_parameter("cmd_topic", "bumperbot_controller/cmd_vel")
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("profile", "paper_fls")
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("stop_at_end", True)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.frequency = float(self.get_parameter("frequency").value)
        self.profile_name = str(self.get_parameter("profile").value)
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.stop_at_end = bool(self.get_parameter("stop_at_end").value)

        self.segments = self.make_profile(self.profile_name)
        self.total_duration = sum(segment["duration"] for segment in self.segments)
        self.start_time = None
        self.last_segment_index = None
        self.finished = False

        self.publisher = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.frequency, 1.0), self.on_timer)

        self.get_logger().info(
            f"{ANSI_CYAN}Scripted trajectory ready: profile={self.profile_name}, "
            f"duration={self.total_duration:.1f}s, cmd_topic={self.cmd_topic}. "
            "Recommended test windows: GPS outages 45:20,85:35,145:30; "
            f"slip 100:80.{ANSI_RESET}"
        )

    def make_profile(self, profile_name):
        if profile_name != "paper_fls":
            raise ValueError("Only profile='paper_fls' is currently supported.")

        # Time-based route designed to create three paper-style outage cases:
        # 45-65s: no-slip, mixed straight/turn trend.
        # 85-120s: starts no-slip, then enters slip at 100s.
        # 145-175s: slip with changing turn direction.
        return [
            self.segment(8.0, 0.00, 0.00, "settle"),
            self.segment(18.0, 0.35, 0.00, "training straight"),
            self.segment(14.0, 0.32, 0.45, "training left arc"),
            self.segment(16.0, 0.45, 0.00, "outage-1 straight no-slip"),
            self.segment(14.0, 0.35, -0.50, "outage-1 right turn no-slip"),
            self.segment(16.0, 0.50, 0.00, "gps update straight"),
            self.segment(15.0, 0.35, 0.55, "outage-2 no-slip left turn"),
            self.segment(24.0, 0.45, 0.00, "outage-2 slip straight"),
            self.segment(18.0, 0.35, -0.55, "slip right arc"),
            self.segment(20.0, 0.45, 0.35, "outage-3 slip left arc"),
            self.segment(20.0, 0.35, -0.55, "outage-3 slip counter turn"),
            self.segment(12.0, 0.30, 0.00, "cooldown straight"),
        ]

    def segment(self, duration, linear, angular, label):
        return {
            "duration": float(duration),
            "linear": float(linear),
            "angular": float(angular),
            "label": label,
        }

    def on_timer(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now

        elapsed = (now - self.start_time).nanoseconds * 1e-9
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = "base_footprint"

        if elapsed < self.start_delay_sec:
            self.publish_command(cmd, 0.0, 0.0)
            return

        route_time = elapsed - self.start_delay_sec
        segment_index, segment = self.current_segment(route_time)

        if segment is None:
            if self.stop_at_end:
                self.publish_command(cmd, 0.0, 0.0)
                if not self.finished:
                    self.get_logger().warn(
                        f"{ANSI_GREEN}===== SCRIPTED TRAJECTORY FINISHED ===== "
                        f"profile={self.profile_name}, elapsed={elapsed:.1f}s ====={ANSI_RESET}"
                    )
                self.finished = True
            return

        if segment_index != self.last_segment_index:
            self.get_logger().warn(
                f"{ANSI_YELLOW}===== SCRIPTED SEGMENT {segment_index + 1}/{len(self.segments)}: "
                f"{segment['label']} ===== t={route_time:.1f}s, "
                f"v={segment['linear']:.2f}, w={segment['angular']:.2f}{ANSI_RESET}"
            )
            self.last_segment_index = segment_index

        self.publish_command(
            cmd,
            segment["linear"] * self.speed_scale,
            segment["angular"] * self.speed_scale,
        )

    def current_segment(self, route_time):
        cursor = 0.0
        for index, segment in enumerate(self.segments):
            cursor_next = cursor + segment["duration"]
            if route_time < cursor_next:
                return index, segment
            cursor = cursor_next
        return None, None

    def publish_command(self, msg, linear, angular):
        if not math.isfinite(linear) or not math.isfinite(angular):
            linear = 0.0
            angular = 0.0
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ScriptedTrajectory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
'''


def backup(path: Path, stamp: str) -> None:
    if path.exists():
        shutil.copy2(path, path.with_suffix(path.suffix + f".bak_{stamp}"))


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def write(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8", newline="\n")


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f"Could not find insertion point for {label}")
    return text.replace(old, new, 1)


def set_yaml_value(text: str, key: str, value: str) -> str:
    pattern = rf"^(\s*{re.escape(key)}:\s*).*$"
    repl = rf"\g<1>{value}"
    new_text, count = re.subn(pattern, repl, text, count=1, flags=re.MULTILINE)
    if count == 0:
        raise RuntimeError(f"Missing YAML key: {key}")
    return new_text


def set_py_default(text: str, key: str, value: str) -> str:
    pattern = rf'(self\.declare_parameter\("{re.escape(key)}",\s*)[^)]+(\))'
    repl = rf"\g<1>{value}\g<2>"
    new_text, count = re.subn(pattern, repl, text, count=1)
    if count == 0:
        raise RuntimeError(f"Missing declare_parameter: {key}")
    return new_text


def main() -> None:
    root = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else Path.cwd()
    if not (root / "bumperbot_controller").is_dir():
        raise SystemExit(
            f"Target root does not look like bumperbot_ws/src: {root}\n"
            "Run from ~/bumperbot_ws/src or pass it as an argument."
        )

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    scripted_path = root / "bumperbot_controller/bumperbot_controller/scripted_trajectory.py"
    scripted_path.parent.mkdir(parents=True, exist_ok=True)
    backup(scripted_path, stamp)
    write(scripted_path, SCRIPTED_TRAJECTORY)
    scripted_path.chmod(scripted_path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    print(f"updated {scripted_path}")

    cmake_path = root / "bumperbot_controller/CMakeLists.txt"
    backup(cmake_path, stamp)
    cmake = read(cmake_path)
    if "scripted_trajectory.py" not in cmake:
        cmake = replace_once(
            cmake,
            "  ${PROJECT_NAME}/noisy_controller.py\n",
            "  ${PROJECT_NAME}/noisy_controller.py\n  ${PROJECT_NAME}/scripted_trajectory.py\n",
            "CMake scripted install",
        )
    write(cmake_path, cmake)
    print(f"updated {cmake_path}")

    launch_path = root / "bumperbot_controller/launch/controller.launch.py"
    backup(launch_path, stamp)
    launch = read(launch_path)
    launch = replace_once(
        launch,
        '''    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )
''',
        '''    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )
    run_scripted_trajectory_arg = DeclareLaunchArgument(
        "run_scripted_trajectory",
        default_value="false",
        description="Publish a repeatable paper-style cmd_vel route for ANN/FLS validation.",
    )
    scripted_trajectory_profile_arg = DeclareLaunchArgument(
        "scripted_trajectory_profile",
        default_value="paper_fls",
    )
    scripted_speed_scale_arg = DeclareLaunchArgument(
        "scripted_speed_scale",
        default_value="1.0",
    )
    scripted_start_delay_sec_arg = DeclareLaunchArgument(
        "scripted_start_delay_sec",
        default_value="2.0",
    )
''',
        "scripted launch arguments",
    )
    launch = replace_once(
        launch,
        '''    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
''',
        '''    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    run_scripted_trajectory = LaunchConfiguration("run_scripted_trajectory")
    scripted_trajectory_profile = LaunchConfiguration("scripted_trajectory_profile")
    scripted_speed_scale = LaunchConfiguration("scripted_speed_scale")
    scripted_start_delay_sec = LaunchConfiguration("scripted_start_delay_sec")
''',
        "scripted launch configurations",
    )
    launch = replace_once(
        launch,
        '''    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
''',
        '''    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    scripted_trajectory = Node(
        package="bumperbot_controller",
        executable="scripted_trajectory.py",
        name="scripted_trajectory_node",
        output="screen",
        parameters=[
            {
                "profile": scripted_trajectory_profile,
                "speed_scale": scripted_speed_scale,
                "start_delay_sec": scripted_start_delay_sec,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(run_scripted_trajectory),
    )

    return LaunchDescription(
''',
        "scripted node definition",
    )
    launch = replace_once(
        launch,
        '''            use_simple_controller_arg,
''',
        '''            use_simple_controller_arg,
            run_scripted_trajectory_arg,
            scripted_trajectory_profile_arg,
            scripted_speed_scale_arg,
            scripted_start_delay_sec_arg,
''',
        "scripted launch argument list",
    )
    launch = replace_once(
        launch,
        '''            noisy_controller_launch,
''',
        '''            noisy_controller_launch,
            scripted_trajectory,
''',
        "scripted launch node list",
    )
    write(launch_path, launch)
    print(f"updated {launch_path}")

    yaml_path = root / "bumperbot_localization/config/fuzzy_localization.yaml"
    backup(yaml_path, stamp)
    yaml = read(yaml_path)
    yaml_values = {
        "small_error": "0.35",
        "large_error": "0.50",
        "ann_weight_zero": "0.0",
        "ann_weight_small": "0.45",
        "ann_weight_large": "1.0",
        "adaptive_error_enabled": "false",
        "adaptive_deadband_ratio": "0.0",
        "velocity_error_source": "corrected",
        "use_normalized_velocity_error": "false",
        "raw_imu_velocity_decay_tau": "10.0",
        "alpha_rise_tau": "0.12",
        "alpha_fall_tau": "0.45",
        "disagreement_boost_enabled": "false",
    }
    for key, value in yaml_values.items():
        yaml = set_yaml_value(yaml, key, value)
    write(yaml_path, yaml)
    print(f"updated {yaml_path}")

    py_path = root / "bumperbot_localization/bumperbot_localization/fuzzy_localization.py"
    backup(py_path, stamp)
    py = read(py_path)
    py_defaults = {
        "small_error": "0.35",
        "large_error": "0.50",
        "ann_weight_zero": "0.0",
        "ann_weight_small": "0.45",
        "ann_weight_large": "1.0",
        "adaptive_error_enabled": "False",
        "adaptive_deadband_ratio": "0.0",
        "velocity_error_source": '"corrected"',
        "use_normalized_velocity_error": "False",
        "raw_imu_velocity_decay_tau": "10.0",
        "alpha_rise_tau": "0.12",
        "alpha_fall_tau": "0.45",
        "disagreement_boost_enabled": "False",
    }
    for key, value in py_defaults.items():
        py = set_py_default(py, key, value)
    write(py_path, py)
    print(f"updated {py_path}")

    print("\nDone. Rebuild with:")
    print("  cd ~/bumperbot_ws")
    print("  colcon build --packages-select bumperbot_localization bumperbot_controller")
    print("  source install/setup.bash")


if __name__ == "__main__":
    main()
