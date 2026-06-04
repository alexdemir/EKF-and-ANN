#!/usr/bin/env python3

import csv
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.node import Node


class FuzzyLocalization(Node):
    def __init__(self):
        super().__init__("fuzzy_localization_node")

        self.declare_parameter("ann_topic", "/odometry/ann")
        self.declare_parameter("kf2_topic", "/odometry/kf_gps_odom")
        self.declare_parameter("odom_topic", "/bumperbot_controller/odom_noisy")
        self.declare_parameter("imu_topic", "/imu_sim")
        self.declare_parameter("output_topic", "/odometry/fuzzy_localization")
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("small_error", 0.12)
        self.declare_parameter("large_error", 0.38)
        self.declare_parameter("ann_weight_zero", 0.0)
        self.declare_parameter("ann_weight_small", 0.35)
        self.declare_parameter("ann_weight_large", 1.0)
        self.declare_parameter("adaptive_error_enabled", True)
        self.declare_parameter("adaptive_error_mode", "excess")
        self.declare_parameter("adaptive_baseline_tau", 5.0)
        self.declare_parameter("adaptive_min_baseline", 0.05)
        self.declare_parameter("adaptive_min_odom_speed", 0.10)
        self.declare_parameter("adaptive_freeze_after_sec", 24.0)
        self.declare_parameter("adaptive_deadband_ratio", 0.03)
        self.declare_parameter("velocity_error_source", "corrected")
        self.declare_parameter("use_normalized_velocity_error", False)
        self.declare_parameter("min_velocity_norm_for_error", 0.15)
        self.declare_parameter("max_integrated_imu_speed", 2.0)
        self.declare_parameter("raw_imu_velocity_decay_tau", 10.0)
        self.declare_parameter("imu_velocity_decay_tau", 12.0)
        self.declare_parameter("imu_velocity_odom_correction_tau", 4.0)
        self.declare_parameter("imu_velocity_correction_gate", 1.2)
        self.declare_parameter("alpha_rise_tau", 0.08)
        self.declare_parameter("alpha_fall_tau", 0.35)
        self.declare_parameter("disagreement_boost_enabled", False)
        self.declare_parameter("disagreement_small", 0.75)
        self.declare_parameter("disagreement_large", 1.05)
        self.declare_parameter("disagreement_min_velocity_error", 1.75)
        self.declare_parameter("output_variance", 0.25)
        self.declare_parameter("save_log", True)
        self.declare_parameter("log_path", "~/bumperbot_ws/src/fuzzy_localization_log.csv")

        self.ann_topic = self.get_parameter("ann_topic").value
        self.kf2_topic = self.get_parameter("kf2_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frequency = float(self.get_parameter("frequency").value)
        self.small_error = float(self.get_parameter("small_error").value)
        self.large_error = float(self.get_parameter("large_error").value)
        self.ann_weight_zero = float(self.get_parameter("ann_weight_zero").value)
        self.ann_weight_small = float(self.get_parameter("ann_weight_small").value)
        self.ann_weight_large = float(self.get_parameter("ann_weight_large").value)
        self.adaptive_error_enabled = bool(
            self.get_parameter("adaptive_error_enabled").value
        )
        self.adaptive_error_mode = str(
            self.get_parameter("adaptive_error_mode").value
        ).strip().lower()
        if self.adaptive_error_mode not in ("ratio", "excess"):
            self.get_logger().warn(
                "adaptive_error_mode must be ratio or excess; using ratio"
            )
            self.adaptive_error_mode = "ratio"
        self.adaptive_baseline_tau = float(
            self.get_parameter("adaptive_baseline_tau").value
        )
        self.adaptive_min_baseline = float(
            self.get_parameter("adaptive_min_baseline").value
        )
        self.adaptive_min_odom_speed = float(
            self.get_parameter("adaptive_min_odom_speed").value
        )
        self.adaptive_freeze_after_sec = float(
            self.get_parameter("adaptive_freeze_after_sec").value
        )
        self.adaptive_deadband_ratio = float(
            self.get_parameter("adaptive_deadband_ratio").value
        )
        self.velocity_error_source = str(
            self.get_parameter("velocity_error_source").value
        ).strip().lower()
        if self.velocity_error_source not in ("raw", "corrected", "max"):
            self.get_logger().warn(
                "velocity_error_source must be raw, corrected, or max; using raw"
            )
            self.velocity_error_source = "raw"
        self.use_normalized_velocity_error = bool(
            self.get_parameter("use_normalized_velocity_error").value
        )
        self.min_velocity_norm_for_error = float(
            self.get_parameter("min_velocity_norm_for_error").value
        )
        self.max_integrated_imu_speed = float(self.get_parameter("max_integrated_imu_speed").value)
        self.raw_imu_velocity_decay_tau = float(
            self.get_parameter("raw_imu_velocity_decay_tau").value
        )
        self.imu_velocity_decay_tau = float(self.get_parameter("imu_velocity_decay_tau").value)
        self.imu_velocity_odom_correction_tau = float(
            self.get_parameter("imu_velocity_odom_correction_tau").value
        )
        self.imu_velocity_correction_gate = float(
            self.get_parameter("imu_velocity_correction_gate").value
        )
        self.alpha_rise_tau = float(self.get_parameter("alpha_rise_tau").value)
        self.alpha_fall_tau = float(self.get_parameter("alpha_fall_tau").value)
        self.disagreement_boost_enabled = bool(
            self.get_parameter("disagreement_boost_enabled").value
        )
        self.disagreement_small = float(self.get_parameter("disagreement_small").value)
        self.disagreement_large = float(self.get_parameter("disagreement_large").value)
        self.disagreement_min_velocity_error = float(
            self.get_parameter("disagreement_min_velocity_error").value
        )
        self.output_variance = float(self.get_parameter("output_variance").value)
        self.save_log = bool(self.get_parameter("save_log").value)
        self.log_path = os.path.expanduser(str(self.get_parameter("log_path").value))

        self.latest_ann = None
        self.latest_kf2 = None
        self.latest_odom = None
        self.latest_imu = None
        self.last_imu_stamp = None
        self.last_timer_stamp = None
        self.imu_body_velocity = np.zeros(2)
        self.imu_body_velocity_raw = np.zeros(2)
        self.latest_odom_body_velocity = np.zeros(2)
        self.latest_imu_body_velocity = np.zeros(2)
        self.latest_imu_body_velocity_raw = np.zeros(2)
        self.alpha_ann_filtered = self.clamp01(self.ann_weight_zero)
        self.velocity_error_baseline = None
        self.baseline_initialized = False

        self.log_file = None
        self.log_writer = None

        self.create_subscription(Odometry, self.ann_topic, self.ann_callback, 20)
        self.create_subscription(Odometry, self.kf2_topic, self.kf2_callback, 20)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 50)
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.frequency, 1.0), self.timer_callback)

        self.open_log_writer()
        self.get_logger().info(
            "Fuzzy localization node started: final_output = fuzzy(ANN, KF2), "
            f"velocity_error_source={self.velocity_error_source}, "
            f"adaptive_error_enabled={self.adaptive_error_enabled}, "
            f"adaptive_error_mode={self.adaptive_error_mode}, "
            f"disagreement_boost_enabled={self.disagreement_boost_enabled}"
        )

    def ann_callback(self, msg):
        self.latest_ann = msg

    def kf2_callback(self, msg):
        self.latest_kf2 = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def imu_callback(self, msg):
        stamp = self.stamp_to_seconds(msg.header.stamp)
        if self.last_imu_stamp is not None:
            dt = stamp - self.last_imu_stamp
            if 0.0 < dt < 0.5:
                yaw_rate = msg.angular_velocity.z
                vx_raw, vy_raw = self.imu_body_velocity_raw
                vx, vy = self.imu_body_velocity

                # Body-frame planar INS integration:
                # a_body = dv_body/dt + omega x v_body, so dv_body/dt = a_body - omega x v_body.
                raw_dvx = msg.linear_acceleration.x + yaw_rate * vy_raw
                raw_dvy = msg.linear_acceleration.y - yaw_rate * vx_raw
                self.imu_body_velocity_raw += np.array([raw_dvx, raw_dvy]) * dt

                dvx = msg.linear_acceleration.x + yaw_rate * vy
                dvy = msg.linear_acceleration.y - yaw_rate * vx
                self.imu_body_velocity += np.array([dvx, dvy]) * dt

                if self.raw_imu_velocity_decay_tau > 0.0:
                    raw_decay = math.exp(-dt / self.raw_imu_velocity_decay_tau)
                    self.imu_body_velocity_raw *= raw_decay

                if self.imu_velocity_decay_tau > 0.0:
                    decay = math.exp(-dt / self.imu_velocity_decay_tau)
                    self.imu_body_velocity *= decay

                self.correct_imu_velocity_with_odom(dt)

                raw_speed = float(np.linalg.norm(self.imu_body_velocity_raw))
                if raw_speed > self.max_integrated_imu_speed:
                    self.imu_body_velocity_raw *= self.max_integrated_imu_speed / raw_speed

                speed = float(np.linalg.norm(self.imu_body_velocity))
                if speed > self.max_integrated_imu_speed:
                    self.imu_body_velocity *= self.max_integrated_imu_speed / speed
        self.last_imu_stamp = stamp
        self.latest_imu = msg

    def correct_imu_velocity_with_odom(self, dt):
        if self.latest_odom is None:
            return
        if self.imu_velocity_odom_correction_tau <= 0.0:
            return

        odom_body_velocity = np.array([
            self.latest_odom.twist.twist.linear.x,
            self.latest_odom.twist.twist.linear.y,
        ], dtype=float)
        disagreement = float(np.linalg.norm(self.imu_body_velocity - odom_body_velocity))
        gate = max(self.imu_velocity_correction_gate, 1e-6)

        # Correct accelerometer integration drift only while IMU and odometry still agree.
        # When the disagreement grows, the gate closes so wheel slip is not hidden.
        correction_gate = math.exp(-0.5 * (disagreement / gate) ** 2)
        correction_alpha = (1.0 - math.exp(-dt / self.imu_velocity_odom_correction_tau))
        correction_alpha *= correction_gate
        self.imu_body_velocity += correction_alpha * (
            odom_body_velocity - self.imu_body_velocity
        )

    def timer_callback(self):
        if self.latest_ann is None or self.latest_kf2 is None or self.latest_odom is None:
            return

        ann_xy = self.xy_from_msg(self.latest_ann)
        kf2_xy = self.xy_from_msg(self.latest_kf2)
        (
            velocity_error,
            raw_velocity_error,
            odom_body_velocity,
            imu_body_velocity_raw,
            imu_body_velocity,
        ) = self.velocity_disagreement()
        odom_speed = float(np.linalg.norm(odom_body_velocity))
        self.update_velocity_error_baseline(velocity_error, odom_speed)
        fuzzy_input_error = self.fuzzy_input_error(velocity_error)
        if self.in_adaptive_deadband(fuzzy_input_error):
            alpha_target = 0.0
        else:
            alpha_target = self.fuzzy_ann_weight(fuzzy_input_error)
        ann_kf2_disagreement = float(np.linalg.norm(ann_xy - kf2_xy))
        disagreement_boost = self.disagreement_boost(
            ann_kf2_disagreement,
            fuzzy_input_error,
        )
        alpha_target = max(alpha_target, disagreement_boost)
        alpha_ann = self.filtered_alpha(alpha_target)
        alpha_kf2 = 1.0 - alpha_ann
        output_xy = alpha_ann * ann_xy + alpha_kf2 * kf2_xy

        msg = self.make_output_msg(output_xy, self.latest_kf2)
        self.publisher.publish(msg)
        self.write_log_row(
            ann_xy,
            kf2_xy,
            output_xy,
            alpha_ann,
            alpha_kf2,
            alpha_target,
            disagreement_boost,
            ann_kf2_disagreement,
            fuzzy_input_error,
            self.current_error_baseline(),
            self.baseline_initialized,
            velocity_error,
            raw_velocity_error,
            odom_body_velocity,
            imu_body_velocity_raw,
            imu_body_velocity,
        )

    def update_velocity_error_baseline(self, velocity_error, odom_speed):
        if not self.adaptive_error_enabled:
            return
        if self.clock_seconds() >= self.adaptive_freeze_after_sec:
            return
        if odom_speed < self.adaptive_min_odom_speed:
            return
        if not math.isfinite(velocity_error):
            return

        floor = max(self.adaptive_min_baseline, 1e-6)
        sample = max(float(velocity_error), floor)
        if self.velocity_error_baseline is None:
            self.velocity_error_baseline = sample
            self.baseline_initialized = True
            return

        if self.adaptive_baseline_tau <= 0.0:
            self.velocity_error_baseline = sample
            return

        dt = 1.0 / max(self.frequency, 1.0)
        alpha = 1.0 - math.exp(-dt / self.adaptive_baseline_tau)
        self.velocity_error_baseline += alpha * (sample - self.velocity_error_baseline)
        self.velocity_error_baseline = max(self.velocity_error_baseline, floor)

    def current_error_baseline(self):
        if not self.adaptive_error_enabled:
            return 1.0
        if self.velocity_error_baseline is None:
            return max(self.adaptive_min_baseline, 1e-6)
        return max(self.velocity_error_baseline, self.adaptive_min_baseline, 1e-6)

    def fuzzy_input_error(self, velocity_error):
        if not self.adaptive_error_enabled:
            return velocity_error
        baseline = self.current_error_baseline()
        if self.adaptive_error_mode == "excess":
            return max(0.0, velocity_error - baseline)
        return velocity_error / baseline

    def in_adaptive_deadband(self, fuzzy_input_error):
        if not self.adaptive_error_enabled:
            return False
        return fuzzy_input_error < self.adaptive_deadband_ratio

    def velocity_disagreement(self):
        odom_vx = self.latest_odom.twist.twist.linear.x
        odom_vy = self.latest_odom.twist.twist.linear.y
        odom_body_velocity = np.array([odom_vx, odom_vy], dtype=float)
        imu_body_velocity_raw = np.array(self.imu_body_velocity_raw, dtype=float)
        imu_body_velocity = np.array(self.imu_body_velocity, dtype=float)
        raw_velocity_error = float(np.linalg.norm(imu_body_velocity_raw - odom_body_velocity))
        corrected_velocity_error = float(np.linalg.norm(imu_body_velocity - odom_body_velocity))
        if self.velocity_error_source == "corrected":
            selected_velocity_error = corrected_velocity_error
        elif self.velocity_error_source == "max":
            selected_velocity_error = max(raw_velocity_error, corrected_velocity_error)
        else:
            selected_velocity_error = raw_velocity_error

        odom_speed = float(np.linalg.norm(odom_body_velocity))
        if self.use_normalized_velocity_error:
            velocity_error = selected_velocity_error / max(
                odom_speed,
                self.min_velocity_norm_for_error,
                1e-6,
            )
        else:
            velocity_error = selected_velocity_error

        self.latest_odom_body_velocity = odom_body_velocity
        self.latest_imu_body_velocity = imu_body_velocity
        self.latest_imu_body_velocity_raw = imu_body_velocity_raw
        return (
            velocity_error,
            raw_velocity_error,
            odom_body_velocity,
            imu_body_velocity_raw,
            imu_body_velocity,
        )

    def filtered_alpha(self, alpha_target):
        alpha_target = self.clamp01(alpha_target)
        now = self.clock_seconds()
        if self.last_timer_stamp is None:
            self.last_timer_stamp = now
            self.alpha_ann_filtered = alpha_target
            return self.alpha_ann_filtered

        dt = max(0.0, min(now - self.last_timer_stamp, 1.0))
        self.last_timer_stamp = now
        if dt <= 0.0:
            return self.alpha_ann_filtered

        tau = self.alpha_rise_tau if alpha_target > self.alpha_ann_filtered else self.alpha_fall_tau
        if tau <= 0.0:
            self.alpha_ann_filtered = alpha_target
        else:
            blend = 1.0 - math.exp(-dt / tau)
            self.alpha_ann_filtered += blend * (alpha_target - self.alpha_ann_filtered)
        self.alpha_ann_filtered = self.clamp01(self.alpha_ann_filtered)
        return self.alpha_ann_filtered

    def disagreement_boost(self, ann_kf2_disagreement, velocity_error):
        if not self.disagreement_boost_enabled:
            return 0.0
        if velocity_error < self.disagreement_min_velocity_error:
            return 0.0

        small = max(self.disagreement_small, 1e-6)
        large = max(self.disagreement_large, small + 1e-6)
        return self.clamp01((ann_kf2_disagreement - small) / (large - small))

    def fuzzy_ann_weight(self, velocity_error):
        small = max(self.small_error, 1e-6)
        large = max(self.large_error, small + 1e-6)

        mu_zero = max(0.0, 1.0 - velocity_error / small)
        if velocity_error <= small:
            mu_small = velocity_error / small
        elif velocity_error < large:
            mu_small = (large - velocity_error) / (large - small)
        else:
            mu_small = 0.0
        mu_small = max(0.0, min(1.0, mu_small))
        mu_large = max(0.0, min(1.0, (velocity_error - small) / (large - small)))

        total = mu_zero + mu_small + mu_large
        if total <= 1e-9:
            return self.clamp01(self.ann_weight_small)

        return self.clamp01(
            (
                mu_zero * self.ann_weight_zero
                + mu_small * self.ann_weight_small
                + mu_large * self.ann_weight_large
            ) / total
        )

    def make_output_msg(self, xy, source):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = source.header.frame_id or "odom"
        msg.child_frame_id = source.child_frame_id or "base_footprint"
        msg.pose.pose.position.x = float(xy[0])
        msg.pose.pose.position.y = float(xy[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = source.pose.pose.orientation
        msg.pose.covariance = [
            self.output_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.output_variance, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0,
        ]
        return msg

    def open_log_writer(self):
        if not self.save_log:
            return
        log_dir = os.path.dirname(self.log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(self.log_path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "stamp",
            "ann_x",
            "ann_y",
            "kf2_x",
            "kf2_y",
            "output_x",
            "output_y",
            "alpha_ann",
            "alpha_kf2",
            "alpha_target",
            "disagreement_boost",
            "ann_kf2_disagreement",
            "fuzzy_input_error",
            "velocity_error_baseline",
            "baseline_initialized",
            "velocity_error",
            "velocity_error_raw",
            "odom_speed",
            "odom_vx",
            "odom_vy",
            "imu_raw_vx",
            "imu_raw_vy",
            "imu_vx",
            "imu_vy",
        ])
        self.log_file.flush()

    def write_log_row(
        self,
        ann_xy,
        kf2_xy,
        output_xy,
        alpha_ann,
        alpha_kf2,
        alpha_target,
        disagreement_boost,
        ann_kf2_disagreement,
        fuzzy_input_error,
        velocity_error_baseline,
        baseline_initialized,
        velocity_error,
        raw_velocity_error,
        odom_body_velocity,
        imu_body_velocity_raw,
        imu_body_velocity,
    ):
        if self.log_writer is None:
            return
        odom_speed = float(np.linalg.norm(odom_body_velocity))
        self.log_writer.writerow([
            f"{self.clock_seconds():.9f}",
            f"{ann_xy[0]:.9f}",
            f"{ann_xy[1]:.9f}",
            f"{kf2_xy[0]:.9f}",
            f"{kf2_xy[1]:.9f}",
            f"{output_xy[0]:.9f}",
            f"{output_xy[1]:.9f}",
            f"{alpha_ann:.9f}",
            f"{alpha_kf2:.9f}",
            f"{alpha_target:.9f}",
            f"{disagreement_boost:.9f}",
            f"{ann_kf2_disagreement:.9f}",
            f"{fuzzy_input_error:.9f}",
            f"{velocity_error_baseline:.9f}",
            str(bool(baseline_initialized)),
            f"{velocity_error:.9f}",
            f"{raw_velocity_error:.9f}",
            f"{odom_speed:.9f}",
            f"{odom_body_velocity[0]:.9f}",
            f"{odom_body_velocity[1]:.9f}",
            f"{imu_body_velocity_raw[0]:.9f}",
            f"{imu_body_velocity_raw[1]:.9f}",
            f"{imu_body_velocity[0]:.9f}",
            f"{imu_body_velocity[1]:.9f}",
        ])
        self.log_file.flush()

    def close_log_writer(self):
        if self.log_file is None:
            return
        self.log_file.flush()
        self.log_file.close()
        self.log_file = None
        self.log_writer = None

    def clock_seconds(self):
        return self.stamp_to_seconds(self.get_clock().now().to_msg())

    @staticmethod
    def xy_from_msg(msg):
        return np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ], dtype=float)

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def clamp01(value):
        return max(0.0, min(1.0, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = FuzzyLocalization()
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
