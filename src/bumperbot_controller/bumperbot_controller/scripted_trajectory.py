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
    """Waypoint based routes for repeatable ANN/FLS tests.

    The node can follow the original square route or a paper-style mismatch
    route. The mismatch route intentionally changes heading trend, speed trend,
    and turn shape so GPS outage segments are not identical to the training
    square. This matches the paper's idea that ANN-only can fail when the
    outage trajectory differs from the just-learned trend.
    """

    def __init__(self):
        super().__init__("scripted_trajectory_node")

        self.declare_parameter("cmd_topic", "/bumperbot_controller/cmd_vel")
        self.declare_parameter("nominal_cmd_topic", "/bumperbot_controller/cmd_vel_nominal")
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
        self.declare_parameter("physical_slip_start_sec", -1.0)
        self.declare_parameter("physical_slip_duration_sec", 0.0)
        self.declare_parameter("physical_slip_linear_scale", 1.0)
        self.declare_parameter("physical_slip_angular_scale", 1.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.nominal_cmd_topic = str(self.get_parameter("nominal_cmd_topic").value)
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
        self.physical_slip_start_sec = float(
            self.get_parameter("physical_slip_start_sec").value
        )
        self.physical_slip_duration_sec = float(
            self.get_parameter("physical_slip_duration_sec").value
        )
        self.physical_slip_linear_scale = float(
            self.get_parameter("physical_slip_linear_scale").value
        )
        self.physical_slip_angular_scale = float(
            self.get_parameter("physical_slip_angular_scale").value
        )

        self.max_linear_speed *= self.speed_scale
        self.max_angular_speed *= max(0.2, self.speed_scale)

        self.publisher = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.nominal_publisher = self.create_publisher(
            TwistStamped, self.nominal_cmd_topic, 10
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.timer = self.create_timer(1.0 / max(self.frequency, 1.0), self.timer_callback)

        self.latest_pose = None
        self.origin = None
        self.waypoints = []
        self.waypoint_speed_scales = []
        self.current_waypoint_index = 0
        self.finished = False
        self.start_time = None
        self.last_status_time = -999.0
        self.physical_slip_was_active = False

        self.get_logger().info(
            "Scripted waypoint route ready: "
            f"profile={self.profile}, side_length={self.side_length:.2f} m, "
            f"laps={self.laps}, max_v={self.max_linear_speed:.2f} m/s, "
            f"max_w={self.max_angular_speed:.2f} rad/s, "
            f"goal_tolerance={self.goal_tolerance:.2f} m, "
            f"nominal_cmd_topic={self.nominal_cmd_topic}, "
            f"physical_slip_start={self.physical_slip_start_sec:.2f}s, "
            f"physical_slip_duration={self.physical_slip_duration_sec:.2f}s"
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.latest_pose = (x, y, yaw)
        if self.origin is None:
            self.origin = self.latest_pose
            self.start_time = self.clock_seconds()
            self.build_waypoints()

    def build_waypoints(self):
        ox, oy, oyaw = self.origin
        local_points = self.local_route_points()

        c = math.cos(oyaw)
        s = math.sin(oyaw)
        self.waypoints = []
        self.waypoint_speed_scales = []
        for lx, ly, speed_scale in local_points:
            self.waypoints.append((ox + c * lx - s * ly, oy + s * lx + c * ly))
            self.waypoint_speed_scales.append(float(speed_scale))
        self.current_waypoint_index = 0
        self.get_logger().warn(
            "\033[96m===== SCRIPTED WAYPOINT ROUTE STARTED ===== "
            f"profile={self.profile}, {len(self.waypoints)} waypoints, "
            f"initial yaw={oyaw:.2f} rad. =====\033[0m"
        )

    def local_route_points(self):
        profile = self.profile.strip().lower()
        if profile in (
            "paper_mismatch",
            "paper_mismatch_waypoint",
            "paper_fls_mismatch",
            "mismatch",
        ):
            return self.paper_mismatch_route_points()
        return self.square_route_points()

    def square_route_points(self):
        local_points = []
        for _ in range(max(self.laps, 1)):
            local_points.extend(
                [
                    (self.side_length, 0.0, 1.00),
                    (self.side_length, self.side_length, 0.75),
                    (0.0, self.side_length, 1.00),
                    (0.0, 0.0, 0.75),
                ]
            )
        return local_points

    def paper_mismatch_route_points(self):
        side = self.side_length
        # The route starts from the same origin as the square training path but
        # then uses diagonal moves, S-like crossings, and alternating mirrored
        # laps. Speed scales deliberately change from segment to segment.
        base_pattern = [
            (1.00 * side, 0.00 * side, 1.00),
            (1.35 * side, 0.38 * side, 0.62),
            (0.92 * side, 1.05 * side, 0.95),
            (0.35 * side, 1.18 * side, 0.48),
            (-0.18 * side, 0.82 * side, 0.88),
            (-0.35 * side, 0.22 * side, 0.55),
            (0.00 * side, 0.00 * side, 1.05),
            (0.62 * side, -0.40 * side, 0.72),
            (1.25 * side, -0.02 * side, 1.08),
            (0.72 * side, 0.58 * side, 0.50),
            (0.12 * side, 0.32 * side, 0.92),
            (0.00 * side, 0.00 * side, 0.65),
        ]
        local_points = []
        for lap in range(max(self.laps, 1)):
            if lap % 2 == 0:
                local_points.extend(base_pattern)
            else:
                # Mirror every second lap so the same controller sees reverse
                # lateral trend and different turn signs.
                local_points.extend((x, -y, speed) for x, y, speed in base_pattern)
        return local_points

    def timer_callback(self):
        now = self.clock_seconds()
        if self.latest_pose is None or self.origin is None:
            self.publish_cmd(0.0, 0.0)
            return

        if self.start_time is None or now - self.start_time < self.start_delay_sec:
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
        waypoint_speed_scale = self.current_waypoint_speed_scale()
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
        linear = (
            self.max_linear_speed
            * waypoint_speed_scale
            * heading_scale
            * distance_scale
        )

        self.publish_cmd(linear, angular)
        if now - self.last_status_time > 4.0:
            self.last_status_time = now
            self.get_logger().info(
                "Scripted waypoint tracking: "
                f"wp={self.current_waypoint_index + 1}/{len(self.waypoints)}, "
                f"dist={distance:.2f} m, heading_error={heading_error:.2f} rad, "
                f"segment_speed_scale={waypoint_speed_scale:.2f}, "
                f"cmd_v={linear:.2f}, cmd_w={angular:.2f}"
            )

    def current_waypoint_speed_scale(self):
        if self.current_waypoint_index >= len(self.waypoint_speed_scales):
            return 1.0
        return max(0.15, float(self.waypoint_speed_scales[self.current_waypoint_index]))

    def publish_cmd(self, linear, angular):
        nominal_linear = float(linear)
        nominal_angular = float(angular)
        actual_linear = nominal_linear
        actual_angular = nominal_angular

        if self.is_physical_slip_active():
            # Paper-style obstacle slip: the commanded wheel motion remains
            # available as the nominal command, while the body command is
            # reduced or stopped to emulate the robot being blocked/slipping.
            actual_linear *= self.physical_slip_linear_scale
            actual_angular *= self.physical_slip_angular_scale
            if not self.physical_slip_was_active:
                self.get_logger().warn(
                    "\033[1;31m===== PHYSICAL BODY SLIP ACTIVE ===== "
                    f"actual_cmd_scale=({self.physical_slip_linear_scale:.2f}, "
                    f"{self.physical_slip_angular_scale:.2f}), nominal command remains unscaled. "
                    "=====\033[0m"
                )
            self.physical_slip_was_active = True
        elif self.physical_slip_was_active:
            self.get_logger().warn(
                "\033[1;32m===== PHYSICAL BODY SLIP ENDED =====\033[0m"
            )
            self.physical_slip_was_active = False

        nominal_msg = self.make_twist(nominal_linear, nominal_angular)
        actual_msg = self.make_twist(actual_linear, actual_angular)
        self.nominal_publisher.publish(nominal_msg)
        self.publisher.publish(actual_msg)

    def make_twist(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        return msg

    def is_physical_slip_active(self):
        if self.start_time is None:
            return False
        if self.physical_slip_start_sec < 0.0 or self.physical_slip_duration_sec <= 0.0:
            return False
        elapsed = self.clock_seconds() - self.start_time
        return (
            elapsed >= self.physical_slip_start_sec
            and elapsed < self.physical_slip_start_sec + self.physical_slip_duration_sec
        )

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
