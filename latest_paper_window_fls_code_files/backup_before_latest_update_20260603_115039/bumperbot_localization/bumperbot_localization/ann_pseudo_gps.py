#!/usr/bin/env python3

import copy
import csv
import hashlib
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


ANSI_RED = "\033[1;31m"
ANSI_GREEN = "\033[1;32m"
ANSI_YELLOW = "\033[1;33m"
ANSI_RESET = "\033[0m"


class SavedMlp:
    def __init__(self):
        self.w1 = None
        self.b1 = None
        self.w2 = None
        self.b2 = None
        self.w3 = None
        self.b3 = None
        self.target_scale = 10.0
        self.target_mode = "residual"
        self.target_origin = None
        self.activation = "tanh"
        self.input_frame = "world"
        self.input_mean = None
        self.input_std = None
        self.input_normalization = "none"
        self.samples = 0

    @property
    def loaded(self):
        return self.w1 is not None

    def load(self, path):
        data = np.load(path, allow_pickle=False)
        self.w1 = data["w1"]
        self.b1 = data["b1"]
        self.w2 = data["w2"]
        self.b2 = data["b2"]
        self.w3 = data["w3"]
        self.b3 = data["b3"]
        self.target_scale = float(data["target_scale"][0])
        self.target_mode = str(data["target_mode"][0])
        if "activation" in data.files:
            self.activation = str(data["activation"][0])
        else:
            self.activation = "tanh"
        if "input_frame" in data.files:
            self.input_frame = str(data["input_frame"][0])
        else:
            self.input_frame = "world"
        if "input_mean" in data.files and "input_std" in data.files:
            self.input_mean = np.array(data["input_mean"], dtype=float)
            self.input_std = np.array(data["input_std"], dtype=float)
            self.input_std = np.where(np.abs(self.input_std) < 1e-6, 1.0, self.input_std)
            self.input_normalization = (
                str(data["input_normalization"][0])
                if "input_normalization" in data.files
                else "standard"
            )
        else:
            self.input_mean = None
            self.input_std = None
            self.input_normalization = "none"
        self.target_origin = None
        if "target_origin" in data.files:
            target_origin = np.array(data["target_origin"], dtype=float)
            if target_origin.shape == (2,) and np.all(np.isfinite(target_origin)):
                self.target_origin = target_origin
        self.samples = int(data["samples"][0])

    def predict(self, x):
        x = self.normalized_input(x)
        if self.activation == "logsig":
            h1 = self.logsig(self.w1 @ x + self.b1)
            h2 = self.logsig(self.w2 @ h1 + self.b2)
        else:
            h1 = np.tanh(self.w1 @ x + self.b1)
            h2 = np.tanh(self.w2 @ h1 + self.b2)
        return self.w3 @ h2 + self.b3

    @staticmethod
    def logsig(value):
        return 1.0 / (1.0 + np.exp(-np.clip(value, -60.0, 60.0)))

    def normalized_input(self, x):
        if self.input_mean is None or self.input_std is None:
            return x
        if self.input_mean.shape != x.shape or self.input_std.shape != x.shape:
            return x
        return (x - self.input_mean) / self.input_std


class AnnPseudoGps(Node):
    def __init__(self):
        super().__init__("ann_pseudo_gps_node")

        self.declare_parameter("odom_topic", "/bumperbot_controller/odom_noisy")
        self.declare_parameter("imu_topic", "/imu_sim")
        self.declare_parameter("gps_odom_topic", "/odometry/gps_sim")
        self.declare_parameter("output_topic", "/odometry/gps_or_ann")
        self.declare_parameter("ann_debug_topic", "/odometry/ann")
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("gps_timeout", 0.5)
        dynamic_float_descriptor = ParameterDescriptor(dynamic_typing=True)
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
        self.declare_parameter("model_path", "~/bumperbot_ws/src/ann_model.npz")
        self.declare_parameter("ann_position_variance", 0.25)
        self.declare_parameter("fallback_position_variance", 4.0)
        self.declare_parameter("min_model_samples", 125)
        self.declare_parameter("fuzzy_fusion_enabled", True)
        self.declare_parameter("fuzzy_small_error", 0.10)
        self.declare_parameter("fuzzy_large_error", 0.60)
        self.declare_parameter("fuzzy_ann_weight_zero", 0.35)
        self.declare_parameter("fuzzy_ann_weight_small", 0.55)
        self.declare_parameter("fuzzy_ann_weight_large", 0.85)
        self.declare_parameter("save_log", True)
        self.declare_parameter("log_path", "~/bumperbot_ws/src/ann_pseudo_gps_log.csv")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.gps_odom_topic = self.get_parameter("gps_odom_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.ann_debug_topic = self.get_parameter("ann_debug_topic").value
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
        self.model_path = os.path.expanduser(str(self.get_parameter("model_path").value))
        self.ann_position_variance = float(self.get_parameter("ann_position_variance").value)
        self.fallback_position_variance = float(self.get_parameter("fallback_position_variance").value)
        self.min_model_samples = int(self.get_parameter("min_model_samples").value)
        self.fuzzy_fusion_enabled = bool(self.get_parameter("fuzzy_fusion_enabled").value)
        self.fuzzy_small_error = float(self.get_parameter("fuzzy_small_error").value)
        self.fuzzy_large_error = float(self.get_parameter("fuzzy_large_error").value)
        self.fuzzy_ann_weight_zero = float(self.get_parameter("fuzzy_ann_weight_zero").value)
        self.fuzzy_ann_weight_small = float(self.get_parameter("fuzzy_ann_weight_small").value)
        self.fuzzy_ann_weight_large = float(self.get_parameter("fuzzy_ann_weight_large").value)
        self.save_log = bool(self.get_parameter("save_log").value)
        self.log_path = os.path.expanduser(str(self.get_parameter("log_path").value))
        self.log_run_id = f"{int(time.time())}_{os.getpid()}"

        self.model = SavedMlp()
        self.model_mtime = None
        self.model_hash = ""

        self.latest_odom = None
        self.latest_imu = None
        self.latest_gps = None
        self.last_gps_stamp = None
        self.start_time = None
        self.last_imu_stamp = None
        self.last_yaw = None
        self.initial_odom_position = None
        self.gps_origin_xy = None

        self.cumulative_accel = np.zeros(3)
        self.imu_velocity = np.zeros(3)
        self.cumulative_imu_speed = 0.0
        self.cumulative_yaw = 0.0
        self.cumulative_odom_position = np.zeros(2)
        self.was_gps_available = False
        self.had_gps_dropout = False
        self.log_file = None
        self.log_writer = None
        self.log_rows_written = 0

        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 50)
        self.gps_sub = self.create_subscription(Odometry, self.gps_odom_topic, self.gps_callback, 20)

        self.output_pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.ann_pub = self.create_publisher(Odometry, self.ann_debug_topic, 10)

        period = 1.0 / max(self.frequency, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)

        self.open_log_writer()
        self.try_load_model()
        self.get_logger().info(
            "ANN pseudo GPS node started: loading saved residual ANN model for GPS "
            f"dropout, gps_timeout={self.gps_timeout:.3f}s, "
            f"force_after={self.force_gps_dropout_after_sec:.3f}s, "
            f"force_duration={self.force_gps_dropout_duration_sec:.3f}s, "
            f"forced_windows={self.forced_dropout_windows}, "
            f"fuzzy_fusion={self.fuzzy_fusion_enabled}"
        )

    def imu_callback(self, msg):
        stamp = self.stamp_to_seconds(msg.header.stamp)
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

        if self.last_imu_stamp is not None:
            dt = stamp - self.last_imu_stamp
            if 0.0 < dt < 0.5:
                self.cumulative_accel += accel * dt
                self.imu_velocity += accel * dt
                self.cumulative_imu_speed += float(np.linalg.norm(self.imu_velocity[:2])) * dt

                _, _, yaw = self.imu_orientation_from_msg(msg)
                if self.last_yaw is not None:
                    dyaw = math.atan2(
                        math.sin(yaw - self.last_yaw),
                        math.cos(yaw - self.last_yaw),
                    )
                    self.cumulative_yaw += dyaw
                self.last_yaw = yaw

        if self.last_yaw is None:
            _, _, self.last_yaw = self.imu_orientation_from_msg(msg)

        self.last_imu_stamp = stamp
        self.latest_imu = msg

    def odom_callback(self, msg):
        odom_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ], dtype=float)
        if self.initial_odom_position is None:
            self.initial_odom_position = odom_position.copy()
        self.cumulative_odom_position = odom_position - self.initial_odom_position
        self.latest_odom = msg

    def gps_callback(self, msg):
        self.latest_gps = msg
        self.last_gps_stamp = self.stamp_to_seconds(msg.header.stamp)
        gps_xy = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ], dtype=float)
        if self.gps_origin_xy is None:
            self.gps_origin_xy = gps_xy.copy()

    def timer_callback(self):
        if self.latest_odom is None or self.latest_imu is None:
            return

        self.try_load_model()
        now = self.clock_seconds()
        if self.start_time is None:
            self.start_time = now
        gps_available = self.is_gps_available(now)

        if gps_available and self.latest_gps is not None:
            if self.had_gps_dropout:
                self.get_logger().info(
                    f"{ANSI_GREEN}===== GPS RESTORED ===== pseudo GPS switched "
                    f"back to real GPS input. ====={ANSI_RESET}"
                )
                self.had_gps_dropout = False
            self.output_pub.publish(copy.deepcopy(self.latest_gps))
            ann_xy = self.publish_ann_debug()
            gps_xy = np.array([
                self.latest_gps.pose.pose.position.x,
                self.latest_gps.pose.pose.position.y,
            ], dtype=float)
            self.write_log_row("gps", gps_available, ann_xy, gps_xy)
            self.was_gps_available = True
            return

        if self.was_gps_available:
            if self.is_forced_dropout(now):
                self.get_logger().warn(
                    f"{ANSI_RED}===== FORCED GPS DROPOUT ACTIVE ===== pseudo GPS "
                    f"switched from real GPS to ANN/FLS output. ====={ANSI_RESET}"
                )
            else:
                self.get_logger().warn(
                    f"{ANSI_YELLOW}===== GPS TIMEOUT ===== switching to saved ANN "
                    f"pseudo-sensor. ====={ANSI_RESET}"
                )
            self.was_gps_available = False
            self.had_gps_dropout = True

        if not self.model_ready():
            fallback = self.make_position_msg(
                self.latest_odom,
                self.latest_odom.pose.pose.position.x,
                self.latest_odom.pose.pose.position.y,
                self.fallback_position_variance,
            )
            self.output_pub.publish(fallback)
            fallback_xy = np.array([
                self.latest_odom.pose.pose.position.x,
                self.latest_odom.pose.pose.position.y,
            ], dtype=float)
            self.write_log_row("odom_fallback", gps_available, None, fallback_xy)
            return

        ann_xy = self.predict_ann_position()
        output_xy, alpha_ann, alpha_odom, velocity_error = self.apply_fuzzy_fusion(ann_xy)
        ann_msg = self.make_position_msg(
            self.latest_odom,
            ann_xy[0],
            ann_xy[1],
            self.ann_position_variance,
        )
        output_msg = self.make_position_msg(
            self.latest_odom,
            output_xy[0],
            output_xy[1],
            self.ann_position_variance,
        )
        self.ann_pub.publish(ann_msg)
        self.output_pub.publish(output_msg)
        source = "fuzzy_ann_odom" if self.fuzzy_fusion_enabled else "ann"
        self.write_log_row(
            source,
            gps_available,
            ann_xy,
            output_xy,
            alpha_ann,
            alpha_odom,
            velocity_error,
        )

    def publish_ann_debug(self):
        if not self.model_ready():
            return None
        ann_xy = self.predict_ann_position()
        ann_msg = self.make_position_msg(
            self.latest_odom,
            ann_xy[0],
            ann_xy[1],
            self.ann_position_variance,
        )
        self.ann_pub.publish(ann_msg)
        return ann_xy

    def try_load_model(self):
        if not os.path.exists(self.model_path):
            return
        model_mtime = os.path.getmtime(self.model_path)
        if self.model_mtime == model_mtime:
            return

        try:
            self.model.load(self.model_path)
            self.model_mtime = model_mtime
            self.model_hash = self.file_sha256(self.model_path)
            self.get_logger().info(
                f"Loaded ANN model: {self.model_path}, samples={self.model.samples}, "
                f"target_mode={self.model.target_mode}, "
                f"activation={self.model.activation}, "
                f"input_frame={self.model.input_frame}, "
                f"input_normalization={self.model.input_normalization}, "
                f"target_origin_available={self.model.target_origin is not None}, "
                f"sha256={self.model_hash[:12]}"
            )
        except (OSError, KeyError, ValueError) as exc:
            self.get_logger().error(f"Could not load ANN model: {exc}")

    def open_log_writer(self):
        if not self.save_log:
            return

        try:
            log_dir = os.path.dirname(self.log_path)
            if log_dir:
                os.makedirs(log_dir, exist_ok=True)
            if os.path.exists(self.log_path) and os.path.getsize(self.log_path) > 0:
                root, ext = os.path.splitext(self.log_path)
                backup_path = f"{root}.previous_{self.log_run_id}{ext}"
                os.replace(self.log_path, backup_path)
                self.get_logger().warn(
                    f"{ANSI_YELLOW}Existing ANN pseudo GPS log was not overwritten. "
                    f"Moved previous log to: {backup_path}{ANSI_RESET}"
                )
            self.log_file = open(self.log_path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                "stamp",
                "log_run_id",
                "source",
                "gps_available",
                "forced_dropout",
                "model_loaded",
                "model_ready",
                "model_samples",
                "model_sha256",
                "model_target_mode",
                "model_input_normalization",
                "ann_x",
                "ann_y",
                "output_x",
                "output_y",
                "odom_x",
                "odom_y",
                "gps_x",
                "gps_y",
                "origin_source",
                "origin_x",
                "origin_y",
                "fuzzy_alpha_ann",
                "fuzzy_alpha_odom",
                "fuzzy_velocity_error",
                *[f"input_{index}" for index in range(15)],
            ])
            self.log_file.flush()
            self.get_logger().info(
                f"ANN pseudo GPS logging enabled: {self.log_path}, "
                f"run_id={self.log_run_id}"
            )
        except OSError as exc:
            self.log_file = None
            self.log_writer = None
            self.save_log = False
            self.get_logger().error(f"Could not open ANN pseudo GPS log file: {exc}")

    def write_log_row(
        self,
        source,
        gps_available,
        ann_xy,
        output_xy,
        fuzzy_alpha_ann=math.nan,
        fuzzy_alpha_odom=math.nan,
        fuzzy_velocity_error=math.nan,
    ):
        if self.log_writer is None or self.latest_odom is None:
            return

        odom_xy = np.array([
            self.latest_odom.pose.pose.position.x,
            self.latest_odom.pose.pose.position.y,
        ], dtype=float)
        gps_xy = np.array([math.nan, math.nan], dtype=float)
        if self.latest_gps is not None:
            gps_xy = np.array([
                self.latest_gps.pose.pose.position.x,
                self.latest_gps.pose.pose.position.y,
            ], dtype=float)
        if ann_xy is None:
            ann_xy = np.array([math.nan, math.nan], dtype=float)
        origin_source, origin_xy = self.prediction_origin()
        input_vector = self.build_input_vector()

        self.log_writer.writerow([
            f"{self.clock_seconds():.9f}",
            self.log_run_id,
            source,
            bool(gps_available),
            self.is_forced_dropout(self.clock_seconds()),
            self.model.loaded,
            self.model_ready(),
            self.model.samples,
            self.model_hash,
            self.model.target_mode,
            self.model.input_normalization,
            f"{ann_xy[0]:.9f}",
            f"{ann_xy[1]:.9f}",
            f"{output_xy[0]:.9f}",
            f"{output_xy[1]:.9f}",
            f"{odom_xy[0]:.9f}",
            f"{odom_xy[1]:.9f}",
            f"{gps_xy[0]:.9f}",
            f"{gps_xy[1]:.9f}",
            origin_source,
            f"{origin_xy[0]:.9f}",
            f"{origin_xy[1]:.9f}",
            f"{fuzzy_alpha_ann:.9f}",
            f"{fuzzy_alpha_odom:.9f}",
            f"{fuzzy_velocity_error:.9f}",
            *[f"{value:.9f}" for value in input_vector],
        ])
        self.log_rows_written += 1
        self.log_file.flush()

    def close_log_writer(self):
        if self.log_file is None:
            return
        self.log_file.flush()
        self.log_file.close()
        self.get_logger().info(
            f"ANN pseudo GPS log closed: rows={self.log_rows_written}, "
            f"path={self.log_path}, run_id={self.log_run_id}"
        )
        self.log_file = None
        self.log_writer = None

    def model_ready(self):
        return self.model.loaded and self.model.samples >= self.min_model_samples

    def predict_ann_position(self):
        x = self.build_input_vector()
        normalized_output = self.model.predict(x)
        output_xy = normalized_output * max(self.model.target_scale, 1e-6)
        if self.model.target_mode == "residual":
            predicted_relative_xy = self.cumulative_odom_position + output_xy
        else:
            predicted_relative_xy = output_xy

        _, origin_xy = self.prediction_origin()
        return origin_xy + predicted_relative_xy

    def apply_fuzzy_fusion(self, ann_xy):
        velocity_error = self.velocity_disagreement()
        if not self.fuzzy_fusion_enabled:
            return ann_xy, 1.0, 0.0, velocity_error

        alpha_ann = self.fuzzy_ann_weight(velocity_error)
        alpha_odom = 1.0 - alpha_ann
        odom_xy = np.array([
            self.latest_odom.pose.pose.position.x,
            self.latest_odom.pose.pose.position.y,
        ], dtype=float)
        fused_xy = alpha_ann * ann_xy + alpha_odom * odom_xy
        return fused_xy, alpha_ann, alpha_odom, velocity_error

    def fuzzy_ann_weight(self, velocity_error):
        small = max(self.fuzzy_small_error, 1e-6)
        large = max(self.fuzzy_large_error, small + 1e-6)

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
            return self.clamp01(self.fuzzy_ann_weight_small)

        alpha_ann = (
            mu_zero * self.fuzzy_ann_weight_zero
            + mu_small * self.fuzzy_ann_weight_small
            + mu_large * self.fuzzy_ann_weight_large
        ) / total
        return self.clamp01(alpha_ann)

    def velocity_disagreement(self):
        if self.latest_odom is None:
            return math.nan

        odom_vx = self.latest_odom.twist.twist.linear.x
        odom_vy = self.latest_odom.twist.twist.linear.y
        odom_speed = math.hypot(odom_vx, odom_vy)
        imu_speed = float(np.linalg.norm(self.imu_velocity[:2]))
        return abs(imu_speed - odom_speed)

    @staticmethod
    def clamp01(value):
        return max(0.0, min(1.0, float(value)))

    def prediction_origin(self):
        if self.gps_origin_xy is not None:
            return "first_gps", self.gps_origin_xy
        if self.model.target_origin is not None:
            return "model_target_origin", self.model.target_origin
        if self.initial_odom_position is not None:
            return "first_odom", self.initial_odom_position
        return "zero", np.zeros(2)

    def build_input_vector(self):
        imu = self.latest_imu
        odom = self.latest_odom
        _, _, yaw = self.imu_orientation()

        accel = np.array([
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ])
        accel_mag = float(np.linalg.norm(accel[:2]))
        cumulative_accel_mag = float(np.linalg.norm(self.cumulative_accel[:2]))
        imu_speed_mag = float(np.linalg.norm(self.imu_velocity[:2]))

        vx_body = odom.twist.twist.linear.x
        vy_body = odom.twist.twist.linear.y
        odom_vx = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
        odom_vy = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)
        odom_integral_x = self.cumulative_odom_position[0]
        odom_integral_y = self.cumulative_odom_position[1]

        raw = np.array([
            accel[0],
            accel[1],
            accel_mag,
            self.cumulative_accel[0],
            self.cumulative_accel[1],
            cumulative_accel_mag,
            self.imu_velocity[0],
            self.imu_velocity[1],
            imu_speed_mag,
            yaw,
            self.cumulative_yaw,
            odom_vx,
            odom_integral_x,
            odom_vy,
            odom_integral_y,
        ], dtype=float)

        scales = np.array([
            10.0,
            10.0,
            10.0,
            100.0,
            100.0,
            100.0,
            50.0,
            50.0,
            50.0,
            math.pi,
            100.0,
            3.0,
            10.0,
            3.0,
            10.0,
        ], dtype=float)
        return np.clip(raw / scales, -1.0, 1.0)

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

    def make_position_msg(self, source, x, y, xy_variance):
        msg = copy.deepcopy(source)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = source.header.frame_id or "odom"
        msg.child_frame_id = source.child_frame_id or "base_footprint"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.covariance = [
            xy_variance, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, xy_variance, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0,
        ]
        return msg

    def imu_orientation(self):
        q = self.latest_imu.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])

    @staticmethod
    def imu_orientation_from_msg(msg):
        q = msg.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])

    def clock_seconds(self):
        return self.stamp_to_seconds(self.get_clock().now().to_msg())

    @staticmethod
    def file_sha256(path):
        digest = hashlib.sha256()
        with open(path, "rb") as file_obj:
            for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = AnnPseudoGps()
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
