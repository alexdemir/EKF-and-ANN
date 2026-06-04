#!/usr/bin/env python3

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
