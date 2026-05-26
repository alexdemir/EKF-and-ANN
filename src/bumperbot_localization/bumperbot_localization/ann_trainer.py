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
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


ANSI_RED = "\033[1;31m"
ANSI_YELLOW = "\033[1;33m"
ANSI_RESET = "\033[0m"


class OnlineMlp:
    def __init__(self, regularization, step_scale, max_step_norm, seed=11):
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0.0, 0.05, (10, 15))
        self.b1 = np.zeros(10)
        self.w2 = rng.normal(0.0, 0.05, (5, 10))
        self.b2 = np.zeros(5)
        self.w3 = rng.normal(0.0, 0.05, (2, 5))
        self.b3 = np.zeros(2)

        self.regularization = regularization
        self.step_scale = step_scale
        self.max_step_norm = max_step_norm
        self.samples = 0
        self.last_loss = 0.0

        param_count = self.pack().size
        self.adam_moment_1 = np.zeros(param_count)
        self.adam_moment_2 = np.zeros(param_count)
        self.adam_step = 0
        self.inverse_hessian = np.eye(param_count)
        self.pending_step = None
        self.pending_gradient_delta = None
        self.regularization_mask = self.pack_weight_mask()
        self.input_mean = None
        self.input_std = None
        self.input_normalization = "none"
        self.loaded_samples = 0
        self.target_scale = None
        self.target_mode = None

    @staticmethod
    def logsig(value):
        return 1.0 / (1.0 + np.exp(-np.clip(value, -60.0, 60.0)))

    def predict(self, x):
        x = self.normalized_input(x)
        h1 = self.logsig(self.w1 @ x + self.b1)
        h2 = self.logsig(self.w2 @ h1 + self.b2)
        return self.w3 @ h2 + self.b3

    def train_batch(self, inputs, targets):
        theta = self.pack()
        loss, gradient = self.loss_and_gradient(inputs, targets)

        if self.pending_step is not None and self.pending_gradient_delta is not None:
            self.update_inverse_hessian(
                self.pending_step,
                self.pending_gradient_delta,
            )
            self.pending_step = None
            self.pending_gradient_delta = None

        direction = -self.inverse_hessian @ gradient
        if float(direction @ gradient) >= 0.0 or not np.all(np.isfinite(direction)):
            self.inverse_hessian = np.eye(theta.size)
            direction = -gradient

        step = self.step_scale * direction
        step_norm = float(np.linalg.norm(step))
        if step_norm > self.max_step_norm:
            step *= self.max_step_norm / step_norm

        accepted_theta = theta + step
        accepted_loss = math.inf
        accepted_gradient = gradient
        slope = float(gradient @ step)
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
            candidate_theta = theta + scale * step
            self.unpack(candidate_theta)
            candidate_loss, candidate_gradient = self.loss_and_gradient(inputs, targets)
            if candidate_loss <= loss + 1e-4 * scale * slope or candidate_loss < accepted_loss:
                accepted_theta = candidate_theta.copy()
                accepted_loss = candidate_loss
                accepted_gradient = candidate_gradient.copy()
            if candidate_loss <= loss + 1e-4 * scale * slope:
                break

        self.unpack(accepted_theta)
        self.pending_step = accepted_theta - theta
        self.pending_gradient_delta = accepted_gradient - gradient
        self.samples += len(inputs)
        self.last_loss = float(accepted_loss)
        return self.last_loss

    def update_inverse_hessian(self, step, gradient_delta):
        curvature = float(gradient_delta @ step)
        if curvature <= 1e-10 or not np.isfinite(curvature):
            return

        hessian_gradient_delta = self.inverse_hessian @ gradient_delta
        gradient_hessian_gradient = float(gradient_delta @ hessian_gradient_delta)
        if gradient_hessian_gradient <= 1e-12 or not np.isfinite(gradient_hessian_gradient):
            return

        self.inverse_hessian += (
            np.outer(step, step) / curvature
            - np.outer(hessian_gradient_delta, hessian_gradient_delta) / gradient_hessian_gradient
        )

    def loss_and_gradient(self, inputs, targets):
        inputs = self.normalized_inputs(inputs)
        batch_size = len(inputs)
        d_w1 = np.zeros_like(self.w1)
        d_b1 = np.zeros_like(self.b1)
        d_w2 = np.zeros_like(self.w2)
        d_b2 = np.zeros_like(self.b2)
        d_w3 = np.zeros_like(self.w3)
        d_b3 = np.zeros_like(self.b3)
        loss = 0.0

        for x, target in zip(inputs, targets):
            z1 = self.w1 @ x + self.b1
            h1 = self.logsig(z1)
            z2 = self.w2 @ h1 + self.b2
            h2 = self.logsig(z2)
            prediction = self.w3 @ h2 + self.b3
            error = prediction - target
            loss += 0.5 * float(error @ error)

            d_out = error
            d_w3 += np.outer(d_out, h2)
            d_b3 += d_out

            d_h2 = self.w3.T @ d_out
            d_z2 = d_h2 * h2 * (1.0 - h2)
            d_w2 += np.outer(d_z2, h1)
            d_b2 += d_z2

            d_h1 = self.w2.T @ d_z2
            d_z1 = d_h1 * h1 * (1.0 - h1)
            d_w1 += np.outer(d_z1, x)
            d_b1 += d_z1

        inv_batch = 1.0 / max(batch_size, 1)
        d_w1 *= inv_batch
        d_b1 *= inv_batch
        d_w2 *= inv_batch
        d_b2 *= inv_batch
        d_w3 *= inv_batch
        d_b3 *= inv_batch
        loss *= inv_batch

        theta = self.pack()
        weighted_theta = self.regularization_mask * theta
        loss += 0.5 * self.regularization * float(theta @ weighted_theta)
        gradient = self.pack_gradients(d_w1, d_b1, d_w2, d_b2, d_w3, d_b3)
        gradient += self.regularization * weighted_theta
        return loss, gradient

    def load(self, path):
        data = np.load(path, allow_pickle=False)
        activation = str(data["activation"][0]) if "activation" in data.files else "tanh"
        if activation != "logsig":
            raise ValueError(
                f"Only logsig models can be fine-tuned online; got activation={activation}"
            )

        self.w1 = np.array(data["w1"], dtype=float)
        self.b1 = np.array(data["b1"], dtype=float)
        self.w2 = np.array(data["w2"], dtype=float)
        self.b2 = np.array(data["b2"], dtype=float)
        self.w3 = np.array(data["w3"], dtype=float)
        self.b3 = np.array(data["b3"], dtype=float)
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
        self.loaded_samples = int(data["samples"][0]) if "samples" in data.files else 0
        self.target_scale = float(data["target_scale"][0]) if "target_scale" in data.files else None
        self.target_mode = str(data["target_mode"][0]) if "target_mode" in data.files else None

        param_count = self.pack().size
        self.inverse_hessian = np.eye(param_count)
        self.pending_step = None
        self.pending_gradient_delta = None
        self.regularization_mask = self.pack_weight_mask()

    def normalized_input(self, x):
        x = np.array(x, dtype=float)
        if self.input_mean is None or self.input_std is None:
            return x
        if self.input_mean.shape != x.shape or self.input_std.shape != x.shape:
            return x
        return (x - self.input_mean) / self.input_std

    def normalized_inputs(self, inputs):
        inputs = np.array(inputs, dtype=float)
        if self.input_mean is None or self.input_std is None:
            return inputs
        if inputs.ndim != 2 or inputs.shape[1] != self.input_mean.shape[0]:
            return inputs
        return (inputs - self.input_mean) / self.input_std

    def pack(self):
        return np.concatenate([
            self.w1.ravel(),
            self.b1,
            self.w2.ravel(),
            self.b2,
            self.w3.ravel(),
            self.b3,
        ])

    def unpack(self, theta):
        offset = 0
        size = self.w1.size
        self.w1 = theta[offset:offset + size].reshape(self.w1.shape)
        offset += size

        size = self.b1.size
        self.b1 = theta[offset:offset + size]
        offset += size

        size = self.w2.size
        self.w2 = theta[offset:offset + size].reshape(self.w2.shape)
        offset += size

        size = self.b2.size
        self.b2 = theta[offset:offset + size]
        offset += size

        size = self.w3.size
        self.w3 = theta[offset:offset + size].reshape(self.w3.shape)
        offset += size

        size = self.b3.size
        self.b3 = theta[offset:offset + size]

    @staticmethod
    def pack_gradients(d_w1, d_b1, d_w2, d_b2, d_w3, d_b3):
        return np.concatenate([
            d_w1.ravel(),
            d_b1,
            d_w2.ravel(),
            d_b2,
            d_w3.ravel(),
            d_b3,
        ])

    def pack_weight_mask(self):
        return np.concatenate([
            np.ones(self.w1.size),
            np.zeros(self.b1.size),
            np.ones(self.w2.size),
            np.zeros(self.b2.size),
            np.ones(self.w3.size),
            np.zeros(self.b3.size),
        ])


class AnnTrainer(Node):
    def __init__(self):
        super().__init__("ann_trainer_node")

        self.declare_parameter("imu_topic", "/imu_sim")
        self.declare_parameter("odom_topic", "/bumperbot_controller/odom_noisy")
        self.declare_parameter("gps_topic", "/odometry/gps_sim")
        self.declare_parameter("target_topic", "/odometry/kf_complementary")
        self.declare_parameter("prediction_topic", "/odometry/ann_training_prediction")
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
        self.declare_parameter("sample_period", 1.0)
        self.declare_parameter("rmse_goal", 1.0e-4)
        self.declare_parameter("log_every_samples", 1)
        self.declare_parameter("target_scale", 50.0)
        self.declare_parameter("target_mode", "absolute")
        self.declare_parameter("recent_window_samples", 50)
        self.declare_parameter("training_window_samples", 250)
        self.declare_parameter("regularization", 0.01)
        self.declare_parameter("step_scale", 0.01)
        self.declare_parameter("max_step_norm", 0.05)
        self.declare_parameter("max_training_iterations_per_sample", 4)
        self.declare_parameter("full_training_every_samples", 25)
        self.declare_parameter("full_training_iterations", 150)
        self.declare_parameter("min_samples_for_convergence", 50)
        self.declare_parameter("batch_size", 32)
        self.declare_parameter("buffer_size", 3500)
        self.declare_parameter("save_dataset", True)
        self.declare_parameter("dataset_path", "~/bumperbot_ws/src/ann_training_dataset.csv")
        self.declare_parameter("model_path", "~/bumperbot_ws/src/ann_model.npz")
        self.declare_parameter("save_model_every_samples", 25)
        self.declare_parameter("load_existing_model", False)
        self.declare_parameter("initial_model_path", "")

        self.imu_topic = self.get_parameter("imu_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.prediction_topic = self.get_parameter("prediction_topic").value
        self.gps_timeout = float(self.get_parameter("gps_timeout").value)
        self.force_gps_dropout_after_sec = float(
            self.get_parameter("force_gps_dropout_after_sec").value
        )
        self.force_gps_dropout_duration_sec = float(
            self.get_parameter("force_gps_dropout_duration_sec").value
        )
        self.sample_period = float(self.get_parameter("sample_period").value)
        self.rmse_goal = float(self.get_parameter("rmse_goal").value)
        self.log_every_samples = int(self.get_parameter("log_every_samples").value)
        self.target_scale = float(self.get_parameter("target_scale").value)
        self.target_mode = str(self.get_parameter("target_mode").value).lower()
        if self.target_mode not in ("absolute", "residual"):
            self.get_logger().warning(
                f"Unknown target_mode={self.target_mode}; falling back to absolute"
            )
            self.target_mode = "absolute"
        self.recent_window_samples = int(self.get_parameter("recent_window_samples").value)
        self.training_window_samples = int(self.get_parameter("training_window_samples").value)
        self.max_training_iterations_per_sample = int(
            self.get_parameter("max_training_iterations_per_sample").value
        )
        self.full_training_every_samples = int(self.get_parameter("full_training_every_samples").value)
        self.full_training_iterations = int(self.get_parameter("full_training_iterations").value)
        self.min_samples_for_convergence = int(self.get_parameter("min_samples_for_convergence").value)
        self.batch_size = int(self.get_parameter("batch_size").value)
        self.buffer_size = int(self.get_parameter("buffer_size").value)
        self.save_dataset = bool(self.get_parameter("save_dataset").value)
        self.dataset_path = os.path.expanduser(str(self.get_parameter("dataset_path").value))
        self.model_path = os.path.expanduser(str(self.get_parameter("model_path").value))
        self.save_model_every_samples = int(self.get_parameter("save_model_every_samples").value)
        self.load_existing_model = bool(self.get_parameter("load_existing_model").value)
        initial_model_path = str(self.get_parameter("initial_model_path").value).strip()
        self.initial_model_path = os.path.expanduser(initial_model_path) if initial_model_path else self.model_path

        regularization = float(self.get_parameter("regularization").value)
        step_scale = float(self.get_parameter("step_scale").value)
        max_step_norm = float(self.get_parameter("max_step_norm").value)
        self.model = OnlineMlp(regularization, step_scale, max_step_norm)
        self.loaded_model_hash = ""
        if self.load_existing_model:
            self.load_initial_model()

        self.latest_imu = None
        self.latest_odom = None
        self.last_gps_receive_time = None
        self.start_time = None
        self.forced_dropout_announced = False
        self.gps_unavailable_announced = False
        self.last_imu_stamp = None
        self.last_odom_stamp = None
        self.last_yaw = None
        self.initial_odom_position = None

        self.cumulative_accel = np.zeros(3)
        self.imu_velocity = np.zeros(3)
        self.cumulative_imu_speed = 0.0
        self.cumulative_yaw = 0.0
        self.cumulative_odom_position = np.zeros(2)

        self.input_buffer = []
        self.target_buffer = []
        self.odom_position_buffer = []
        self.target_origin = None
        self.last_sample_stamp = None
        self.last_sample_wall_time = None
        self.last_sample_dt = 0.0
        self.last_sample_wall_dt = 0.0
        self.last_training_duration = 0.0
        self.collected_samples = 0

        self.current_rmse = float("inf")
        self.current_odom_rmse = float("inf")
        self.current_recent_rmse = float("inf")
        self.current_recent_odom_rmse = float("inf")
        self.current_latest_error = float("inf")
        self.current_target_norm = 0.0
        self.current_prediction_norm = 0.0
        self.current_target_clipped = False
        self.last_training_mode = "online"
        self.dataset_file = None
        self.dataset_writer = None
        self.last_saved_sample = 0

        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 50)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.gps_sub = self.create_subscription(Odometry, self.gps_topic, self.gps_callback, 20)
        self.target_sub = self.create_subscription(Odometry, self.target_topic, self.target_callback, 20)
        self.prediction_pub = self.create_publisher(Odometry, self.prediction_topic, 10)

        self.open_dataset_writer()

        self.get_logger().info(
            "ANN trainer started in paper-target mode: 15 IMU/odometry inputs, "
            f"2 KF-complementary outputs, target_mode={self.target_mode}, "
            f"sample_period={self.sample_period:.2f}s, rmse_goal={self.rmse_goal:.1e}, "
            f"load_existing_model={self.load_existing_model}, "
            f"force_after={self.force_gps_dropout_after_sec:.3f}s"
        )

    def load_initial_model(self):
        if not os.path.exists(self.initial_model_path):
            self.get_logger().warning(
                f"Requested load_existing_model but file does not exist: {self.initial_model_path}"
            )
            return
        try:
            self.model.load(self.initial_model_path)
            if self.model.target_scale is not None:
                self.target_scale = self.model.target_scale
            if self.model.target_mode in ("absolute", "residual"):
                self.target_mode = self.model.target_mode
            self.loaded_model_hash = self.file_sha256(self.initial_model_path)
            self.get_logger().info(
                "Loaded initial ANN model for online fine-tuning: "
                f"{self.initial_model_path}, samples={self.model.loaded_samples}, "
                f"target_mode={self.target_mode}, target_scale={self.target_scale:.3f}, "
                f"input_normalization={self.model.input_normalization}, "
                f"sha256={self.loaded_model_hash[:12]}"
            )
        except (OSError, KeyError, ValueError) as exc:
            self.get_logger().error(
                f"Could not load initial ANN model {self.initial_model_path}: {exc}"
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
        stamp = self.stamp_to_seconds(msg.header.stamp)
        odom_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ], dtype=float)
        if self.initial_odom_position is None:
            self.initial_odom_position = odom_position.copy()

        self.cumulative_odom_position = odom_position - self.initial_odom_position

        self.last_odom_stamp = stamp
        self.latest_odom = msg

    def gps_callback(self, msg):
        self.last_gps_receive_time = self.clock_seconds()

    def target_callback(self, msg):
        if self.latest_imu is None or self.latest_odom is None:
            return

        now = self.clock_seconds()
        if self.start_time is None:
            self.start_time = now
        target_time = self.stamp_to_seconds(msg.header.stamp)
        if not self.is_gps_available(now):
            self.announce_gps_unavailable(now)
            return
        self.gps_unavailable_announced = False

        target_xy = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ], dtype=float)
        if self.target_origin is None:
            self.target_origin = target_xy.copy()

        target_relative_xy = target_xy - self.target_origin
        odom_xy = np.array([
            self.latest_odom.pose.pose.position.x,
            self.latest_odom.pose.pose.position.y,
        ], dtype=float)
        if self.initial_odom_position is None:
            odom_relative_xy = np.zeros(2)
        else:
            odom_relative_xy = odom_xy - self.initial_odom_position

        training_target_xy = self.training_target_from_relative(
            target_relative_xy,
            odom_relative_xy,
        )
        normalized_training_target_xy = self.normalize_target(training_target_xy)
        self.current_target_norm = float(np.linalg.norm(target_relative_xy))
        self.current_target_clipped = bool(np.any(np.abs(training_target_xy) > self.target_scale))

        if not self.should_collect_sample(target_time):
            self.publish_prediction(msg, odom_relative_xy)
            return

        wall_time = time.monotonic()
        if self.last_sample_stamp is not None:
            self.last_sample_dt = target_time - self.last_sample_stamp
        if self.last_sample_wall_time is not None:
            self.last_sample_wall_dt = wall_time - self.last_sample_wall_time

        input_vector = self.build_input_vector()
        self.input_buffer.append(input_vector)
        self.target_buffer.append(normalized_training_target_xy)
        self.odom_position_buffer.append(odom_relative_xy)
        self.collected_samples += 1
        self.last_sample_stamp = target_time
        self.last_sample_wall_time = wall_time

        if len(self.input_buffer) > self.buffer_size:
            self.input_buffer.pop(0)
            self.target_buffer.pop(0)
            self.odom_position_buffer.pop(0)

        self.train_until_rmse_goal()
        if (
            self.save_model_every_samples > 0
            and self.collected_samples % self.save_model_every_samples == 0
        ):
            self.save_model()
        predicted_relative_xy = self.prediction_from_model_output(
            self.model.predict(input_vector),
            odom_relative_xy,
        )
        self.write_dataset_row(
            target_time,
            input_vector,
            target_relative_xy,
            normalized_training_target_xy,
            odom_relative_xy,
            predicted_relative_xy,
        )
        self.publish_prediction(msg, odom_relative_xy)

    def should_collect_sample(self, stamp):
        if self.last_sample_stamp is None:
            return True
        return (stamp - self.last_sample_stamp) >= self.sample_period

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
        vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
        vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)

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
            vx_world,
            self.cumulative_odom_position[0],
            vy_world,
            self.cumulative_odom_position[1],
        ], dtype=float)

        scales = np.array([
            10.0, 10.0, 10.0,
            100.0, 100.0, 100.0,
            50.0, 50.0, 50.0,
            math.pi, 100.0,
            3.0, 10.0,
            3.0, 10.0,
        ], dtype=float)
        return np.clip(raw / scales, -1.0, 1.0)

    def imu_orientation(self):
        q = self.latest_imu.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])

    @staticmethod
    def imu_orientation_from_msg(msg):
        q = msg.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])

    def current_yaw(self):
        if self.latest_imu is None:
            return 0.0
        _, _, yaw = self.imu_orientation()
        return yaw

    def publish_prediction(self, target_msg, odom_relative_xy):
        predicted_relative_xy = self.prediction_from_model_output(
            self.model.predict(self.build_input_vector()),
            odom_relative_xy,
        )
        predicted_xy = predicted_relative_xy
        if self.target_origin is not None:
            predicted_xy = self.target_origin + predicted_relative_xy

        msg = copy.deepcopy(target_msg)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = target_msg.header.frame_id or "odom"
        msg.child_frame_id = target_msg.child_frame_id or "base_footprint"
        msg.pose.pose.position.x = float(predicted_xy[0])
        msg.pose.pose.position.y = float(predicted_xy[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 999.0,
        ]
        self.prediction_pub.publish(msg)

    def train_until_rmse_goal(self):
        training_start = time.monotonic()
        self.current_rmse = self.compute_rmse()
        self.current_odom_rmse = self.compute_odom_rmse()
        self.current_recent_rmse = self.compute_rmse(self.recent_window_samples)
        self.current_recent_odom_rmse = self.compute_odom_rmse(self.recent_window_samples)
        self.current_latest_error = self.compute_latest_error()
        self.current_prediction_norm = self.compute_latest_prediction_norm()

        if (
            self.collected_samples >= self.min_samples_for_convergence
            and self.current_rmse <= self.rmse_goal
        ):
            self.last_training_duration = time.monotonic() - training_start
            self.log_training_state("converged")
            return

        training_iterations = max(self.max_training_iterations_per_sample, 0)
        use_full_window = False
        self.last_training_mode = "online"
        if (
            self.full_training_every_samples > 0
            and self.collected_samples >= self.min_samples_for_convergence
            and self.collected_samples % self.full_training_every_samples == 0
        ):
            training_iterations = max(self.full_training_iterations, training_iterations)
            use_full_window = True
            self.last_training_mode = "full"

        if training_iterations <= 0:
            self.last_training_mode = "collect_only"
            self.last_training_duration = time.monotonic() - training_start
            self.log_training_state("active")
            return

        for _ in range(training_iterations):
            inputs, targets = self.training_batch(use_full_window)
            self.model.train_batch(inputs, targets)

        self.current_rmse = self.compute_rmse()
        self.current_odom_rmse = self.compute_odom_rmse()
        self.current_recent_rmse = self.compute_rmse(self.recent_window_samples)
        self.current_recent_odom_rmse = self.compute_odom_rmse(self.recent_window_samples)
        self.current_latest_error = self.compute_latest_error()
        self.current_prediction_norm = self.compute_latest_prediction_norm()
        self.last_training_duration = time.monotonic() - training_start
        self.log_training_state("active")

    def training_batch(self, use_full_window=False):
        sample_count = len(self.input_buffer)
        window_count = min(max(self.training_window_samples, 1), sample_count)
        start_index = sample_count - window_count
        batch_count = window_count if use_full_window else min(self.batch_size, window_count)

        if batch_count == window_count:
            indices = range(start_index, sample_count)
        else:
            indices = np.linspace(start_index, sample_count - 1, batch_count, dtype=int)

        inputs = np.array([self.input_buffer[i] for i in indices])
        targets = np.array([self.target_buffer[i] for i in indices])
        return inputs, targets

    def compute_rmse(self, window_size=None):
        if not self.input_buffer:
            return float("inf")

        input_buffer = self.input_buffer
        target_buffer = self.target_buffer
        odom_position_buffer = self.odom_position_buffer
        if window_size is not None and window_size > 0:
            input_buffer = input_buffer[-window_size:]
            target_buffer = target_buffer[-window_size:]
            odom_position_buffer = odom_position_buffer[-window_size:]

        squared_error_sum = 0.0
        for x, normalized_training_target_xy, odom_relative_xy in zip(
            input_buffer,
            target_buffer,
            odom_position_buffer,
        ):
            predicted_relative_xy = self.prediction_from_model_output(
                self.model.predict(x),
                odom_relative_xy,
            )
            target_relative_xy = self.relative_target_from_training_target(
                normalized_training_target_xy,
                odom_relative_xy,
            )
            error = predicted_relative_xy - target_relative_xy
            squared_error_sum += float(error @ error)
        return math.sqrt(squared_error_sum / max(len(input_buffer), 1))

    def compute_odom_rmse(self, window_size=None):
        if not self.target_buffer:
            return float("inf")

        target_buffer = self.target_buffer
        odom_position_buffer = self.odom_position_buffer
        if window_size is not None and window_size > 0:
            target_buffer = target_buffer[-window_size:]
            odom_position_buffer = odom_position_buffer[-window_size:]

        squared_error_sum = 0.0
        for normalized_training_target_xy, odom_relative_xy in zip(target_buffer, odom_position_buffer):
            target_relative_xy = self.relative_target_from_training_target(
                normalized_training_target_xy,
                odom_relative_xy,
            )
            error = np.array(odom_relative_xy) - target_relative_xy
            squared_error_sum += float(error @ error)
        return math.sqrt(squared_error_sum / max(len(target_buffer), 1))

    def compute_latest_error(self):
        if not self.input_buffer:
            return float("inf")

        predicted_relative_xy = self.prediction_from_model_output(
            self.model.predict(self.input_buffer[-1]),
            self.odom_position_buffer[-1],
        )
        target_relative_xy = self.relative_target_from_training_target(
            self.target_buffer[-1],
            self.odom_position_buffer[-1],
        )
        error = predicted_relative_xy - target_relative_xy
        return float(np.linalg.norm(error))

    def compute_latest_prediction_norm(self):
        if not self.input_buffer:
            return 0.0

        predicted_relative_xy = self.prediction_from_model_output(
            self.model.predict(self.input_buffer[-1]),
            self.odom_position_buffer[-1],
        )
        return float(np.linalg.norm(predicted_relative_xy))

    def log_training_state(self, state):
        interval = max(self.log_every_samples, 1)
        if self.collected_samples == 1 or self.collected_samples % interval == 0:
            improvement = 0.0
            if math.isfinite(self.current_odom_rmse) and self.current_odom_rmse > 1e-9:
                improvement = 100.0 * (1.0 - self.current_rmse / self.current_odom_rmse)

            recent_improvement = 0.0
            if math.isfinite(self.current_recent_odom_rmse) and self.current_recent_odom_rmse > 1e-9:
                recent_improvement = 100.0 * (
                    1.0 - self.current_recent_rmse / self.current_recent_odom_rmse
                )

            self.get_logger().info(
                f"ANN dataset_samples={self.collected_samples}, "
                f"buffer={len(self.input_buffer)}, rmse={self.current_rmse:.6f}, "
                f"odom_rmse={self.current_odom_rmse:.6f}, "
                f"recent_rmse={self.current_recent_rmse:.6f}, "
                f"recent_odom_rmse={self.current_recent_odom_rmse:.6f}, "
                f"latest_error={self.current_latest_error:.6f}, "
                f"improvement={improvement:.1f}%, "
                f"recent_improvement={recent_improvement:.1f}%, "
                f"target_norm={self.current_target_norm:.3f}, "
                f"prediction_norm={self.current_prediction_norm:.3f}, "
                f"target_clipped={self.current_target_clipped}, "
                f"sample_dt={self.last_sample_dt:.2f}s, "
                f"wall_dt={self.last_sample_wall_dt:.2f}s, "
                f"train_time={self.last_training_duration:.3f}s, "
                f"train_mode={self.last_training_mode}, "
                f"state={state}"
            )

    def open_dataset_writer(self):
        if not self.save_dataset:
            return

        try:
            dataset_dir = os.path.dirname(self.dataset_path)
            if dataset_dir:
                os.makedirs(dataset_dir, exist_ok=True)
            self.dataset_file = open(self.dataset_path, "w", newline="")
            self.dataset_writer = csv.writer(self.dataset_file)
            header = [
                "stamp",
                "sample_index",
                *[f"input_{i}" for i in range(15)],
                "target_x_m",
                "target_y_m",
                "target_x_norm",
                "target_y_norm",
                "odom_x_m",
                "odom_y_m",
                "target_norm_m",
                "odom_error_m",
                "training_target_x_m",
                "training_target_y_m",
                "training_target_x_norm",
                "training_target_y_norm",
                "prediction_x_m",
                "prediction_y_m",
                "prediction_error_m",
                "target_mode",
            ]
            self.dataset_writer.writerow(header)
            self.dataset_file.flush()
            self.get_logger().info(f"ANN dataset logging enabled: {self.dataset_path}")
        except OSError as exc:
            self.dataset_file = None
            self.dataset_writer = None
            self.save_dataset = False
            self.get_logger().error(f"Could not open ANN dataset file: {exc}")

    def write_dataset_row(
        self,
        stamp,
        input_vector,
        target_relative_xy,
        normalized_training_target_xy,
        odom_relative_xy,
        predicted_relative_xy,
    ):
        if self.dataset_writer is None:
            return

        odom_error = np.array(odom_relative_xy) - np.array(target_relative_xy)
        training_target_xy = self.denormalize_target(normalized_training_target_xy)
        prediction_error = np.array(predicted_relative_xy) - np.array(target_relative_xy)
        row = [
            f"{stamp:.9f}",
            self.collected_samples,
            *[f"{value:.9f}" for value in input_vector],
            f"{target_relative_xy[0]:.9f}",
            f"{target_relative_xy[1]:.9f}",
            f"{self.normalize_target(target_relative_xy)[0]:.9f}",
            f"{self.normalize_target(target_relative_xy)[1]:.9f}",
            f"{odom_relative_xy[0]:.9f}",
            f"{odom_relative_xy[1]:.9f}",
            f"{np.linalg.norm(target_relative_xy):.9f}",
            f"{np.linalg.norm(odom_error):.9f}",
            f"{training_target_xy[0]:.9f}",
            f"{training_target_xy[1]:.9f}",
            f"{normalized_training_target_xy[0]:.9f}",
            f"{normalized_training_target_xy[1]:.9f}",
            f"{predicted_relative_xy[0]:.9f}",
            f"{predicted_relative_xy[1]:.9f}",
            f"{np.linalg.norm(prediction_error):.9f}",
            self.target_mode,
        ]
        self.dataset_writer.writerow(row)
        self.dataset_file.flush()

    def close_dataset_writer(self):
        if self.dataset_file is None:
            return
        self.dataset_file.flush()
        self.dataset_file.close()
        self.dataset_file = None
        self.dataset_writer = None

    def save_model(self):
        try:
            model_dir = os.path.dirname(self.model_path)
            if model_dir:
                os.makedirs(model_dir, exist_ok=True)
            np.savez(
                self.model_path,
                w1=self.model.w1,
                b1=self.model.b1,
                w2=self.model.w2,
                b2=self.model.b2,
                w3=self.model.w3,
                b3=self.model.b3,
                target_scale=np.array([self.target_scale], dtype=float),
                target_mode=np.array([self.target_mode]),
                target_origin=(
                    self.target_origin
                    if self.target_origin is not None
                    else np.array([math.nan, math.nan], dtype=float)
                ),
                samples=np.array([self.model.loaded_samples + self.collected_samples], dtype=int),
                fine_tune_samples=np.array([self.collected_samples], dtype=int),
                initial_model_samples=np.array([self.model.loaded_samples], dtype=int),
                activation=np.array(["logsig"]),
                input_frame=np.array(["world"]),
                input_mean=(
                    self.model.input_mean
                    if self.model.input_mean is not None
                    else np.array([], dtype=float)
                ),
                input_std=(
                    self.model.input_std
                    if self.model.input_std is not None
                    else np.array([], dtype=float)
                ),
                input_normalization=np.array([self.model.input_normalization]),
                initial_model_path=np.array([self.initial_model_path]),
                initial_model_sha256=np.array([self.loaded_model_hash]),
            )
            self.last_saved_sample = self.collected_samples
            self.get_logger().info(
                f"ANN model saved: {self.model_path}, samples={self.collected_samples}"
            )
        except OSError as exc:
            self.get_logger().error(f"Could not save ANN model: {exc}")

    def is_gps_available(self, stamp):
        if self.is_forced_dropout(stamp):
            return False
        if self.last_gps_receive_time is None:
            return False
        return 0.0 <= stamp - self.last_gps_receive_time <= self.gps_timeout

    def is_forced_dropout(self, now):
        if self.start_time is None:
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

    def announce_gps_unavailable(self, now):
        if self.is_forced_dropout(now):
            if not self.forced_dropout_announced:
                self.get_logger().warn(
                    f"{ANSI_RED}===== FORCED GPS DROPOUT ACTIVE ===== "
                    "ANN trainer stopped collecting "
                    "samples and stopped online fine-tuning. Pseudo/hold/FLS test phase "
                    f"is now active. ====={ANSI_RESET}"
                )
                self.forced_dropout_announced = True
            return

        if not self.gps_unavailable_announced:
            self.get_logger().warn(
                f"{ANSI_YELLOW}===== GPS UNAVAILABLE ===== ANN trainer sample "
                f"collection is paused. ====={ANSI_RESET}"
            )
            self.gps_unavailable_announced = True

    def training_target_from_relative(self, target_relative_xy, odom_relative_xy):
        if self.target_mode == "residual":
            return target_relative_xy - odom_relative_xy
        return target_relative_xy

    def relative_target_from_training_target(self, normalized_training_target_xy, odom_relative_xy):
        training_target_xy = self.denormalize_target(normalized_training_target_xy)
        if self.target_mode == "residual":
            return np.array(odom_relative_xy) + training_target_xy
        return training_target_xy

    def prediction_from_model_output(self, normalized_output_xy, odom_relative_xy):
        output_xy = self.denormalize_target(normalized_output_xy)
        if self.target_mode == "residual":
            return np.array(odom_relative_xy) + output_xy
        return output_xy

    def normalize_target(self, target):
        scale = max(self.target_scale, 1e-6)
        return np.clip(target / scale, -1.0, 1.0)

    def denormalize_target(self, target):
        return target * max(self.target_scale, 1e-6)

    @staticmethod
    def stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def clock_seconds(self):
        return self.stamp_to_seconds(self.get_clock().now().to_msg())

    @staticmethod
    def file_sha256(path):
        digest = hashlib.sha256()
        with open(path, "rb") as file_obj:
            for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()


def main(args=None):
    rclpy.init(args=args)
    node = AnnTrainer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.collected_samples != node.last_saved_sample:
            node.save_model()
        node.close_dataset_writer()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
