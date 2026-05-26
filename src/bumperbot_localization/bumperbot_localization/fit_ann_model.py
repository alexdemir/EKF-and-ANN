#!/usr/bin/env python3

import argparse
import csv
import hashlib
import math
import os

import numpy as np


def logsig(value):
    return 1.0 / (1.0 + np.exp(-np.clip(value, -60.0, 60.0)))


class BatchMlp:
    def __init__(self, regularization, seed):
        rng = np.random.default_rng(seed)
        self.w1 = rng.normal(0.0, 0.05, (10, 15))
        self.b1 = np.zeros(10)
        self.w2 = rng.normal(0.0, 0.05, (5, 10))
        self.b2 = np.zeros(5)
        self.w3 = rng.normal(0.0, 0.05, (2, 5))
        self.b3 = np.zeros(2)
        self.regularization = regularization
        self.regularization_mask = self.pack_weight_mask()

    def predict(self, inputs):
        h1 = logsig(inputs @ self.w1.T + self.b1)
        h2 = logsig(h1 @ self.w2.T + self.b2)
        return h2 @ self.w3.T + self.b3

    def loss_and_gradient(self, inputs, targets):
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
            h1 = logsig(z1)
            z2 = self.w2 @ h1 + self.b2
            h2 = logsig(z2)
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
        return float(loss), gradient

    def fit(self, inputs, targets, max_iterations, max_step_norm):
        theta = self.pack()
        inverse_hessian = np.eye(theta.size)
        loss, gradient = self.loss_and_gradient(inputs, targets)

        for iteration in range(max_iterations):
            if float(np.linalg.norm(gradient)) < 1e-8:
                break

            direction = -inverse_hessian @ gradient
            if float(direction @ gradient) >= 0.0 or not np.all(np.isfinite(direction)):
                inverse_hessian = np.eye(theta.size)
                direction = -gradient

            step = direction
            step_norm = float(np.linalg.norm(step))
            if step_norm > max_step_norm:
                step *= max_step_norm / step_norm

            accepted = False
            old_theta = theta.copy()
            old_gradient = gradient.copy()
            old_loss = loss
            slope = float(old_gradient @ step)

            for scale in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625):
                candidate_theta = old_theta + scale * step
                self.unpack(candidate_theta)
                candidate_loss, candidate_gradient = self.loss_and_gradient(inputs, targets)
                if candidate_loss <= old_loss + 1e-4 * scale * slope:
                    theta = candidate_theta
                    loss = candidate_loss
                    gradient = candidate_gradient
                    accepted = True
                    break

            if not accepted:
                candidate_theta = old_theta + 0.01 * step
                self.unpack(candidate_theta)
                candidate_loss, candidate_gradient = self.loss_and_gradient(inputs, targets)
                if candidate_loss < old_loss:
                    theta = candidate_theta
                    loss = candidate_loss
                    gradient = candidate_gradient
                else:
                    self.unpack(old_theta)
                    inverse_hessian = np.eye(theta.size)
                    loss, gradient = self.loss_and_gradient(inputs, targets)
                    continue

            step_delta = theta - old_theta
            gradient_delta = gradient - old_gradient
            curvature = float(gradient_delta @ step_delta)
            if curvature > 1e-10 and np.isfinite(curvature):
                hessian_gradient_delta = inverse_hessian @ gradient_delta
                gradient_hessian_gradient = float(gradient_delta @ hessian_gradient_delta)
                if gradient_hessian_gradient > 1e-12 and np.isfinite(gradient_hessian_gradient):
                    inverse_hessian += (
                        np.outer(step_delta, step_delta) / curvature
                        - np.outer(hessian_gradient_delta, hessian_gradient_delta)
                        / gradient_hessian_gradient
                    )

            if iteration % 50 == 0:
                print(f"iteration={iteration}, loss={loss:.9f}")

        self.unpack(theta)
        return loss

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


def load_dataset(path):
    with open(path, newline="") as dataset_file:
        rows = list(csv.DictReader(dataset_file))
    if not rows:
        raise RuntimeError(f"No rows found in dataset: {path}")

    inputs = np.array([
        [float(row[f"input_{index}"]) for index in range(15)]
        for row in rows
    ], dtype=float)
    recorded_targets_m = np.array([
        [float(row["training_target_x_m"]), float(row["training_target_y_m"])]
        for row in rows
    ], dtype=float)
    odom_m = np.array([
        [float(row["odom_x_m"]), float(row["odom_y_m"])]
        for row in rows
    ], dtype=float)
    kf_target_m = np.array([
        [float(row["target_x_m"]), float(row["target_y_m"])]
        for row in rows
    ], dtype=float)
    recorded_target_mode = rows[-1].get("target_mode", "absolute")
    return rows, inputs, recorded_targets_m, odom_m, kf_target_m, recorded_target_mode


def targets_for_mode(kf_target_m, odom_m, recorded_targets_m, recorded_target_mode, target_mode):
    if target_mode == "auto":
        target_mode = recorded_target_mode
        targets_m = recorded_targets_m
    elif target_mode == "residual":
        targets_m = kf_target_m - odom_m
    elif target_mode == "absolute":
        targets_m = kf_target_m
    else:
        raise ValueError(f"Unsupported target mode: {target_mode}")
    return targets_m, target_mode


def fit_input_normalizer(inputs, mode):
    if mode == "none":
        return np.zeros(inputs.shape[1], dtype=float), np.ones(inputs.shape[1], dtype=float)

    input_mean = np.mean(inputs, axis=0)
    input_std = np.std(inputs, axis=0)
    input_std = np.where(input_std < 1e-6, 1.0, input_std)
    return input_mean, input_std


def normalize_inputs(inputs, input_mean, input_std):
    return (inputs - input_mean) / input_std


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as file_obj:
        for chunk in iter(lambda: file_obj.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rmse(error):
    return math.sqrt(float(np.mean(np.sum(error * error, axis=1))))


def evaluate_model(model, inputs, targets_m, odom_m, kf_target_m, target_mode, target_scale):
    predictions_m = model.predict(inputs) * target_scale
    if target_mode == "residual":
        predicted_positions_m = odom_m + predictions_m
    else:
        predicted_positions_m = predictions_m

    odom_rmse = rmse(odom_m - kf_target_m)
    ann_rmse = rmse(predicted_positions_m - kf_target_m)
    improvement = (odom_rmse - ann_rmse) / max(odom_rmse, 1e-9) * 100.0
    target_fit_rmse = rmse(predictions_m - targets_m)
    return odom_rmse, ann_rmse, improvement, target_fit_rmse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="~/bumperbot_ws/src/ann_training_dataset.csv")
    parser.add_argument("--model", default="~/bumperbot_ws/src/ann_model.npz")
    parser.add_argument("--target-scale", type=float, default=10.0)
    parser.add_argument("--regularization", type=float, default=0.000001)
    parser.add_argument("--iterations", type=int, default=2000)
    parser.add_argument("--max-step-norm", type=float, default=1.0)
    parser.add_argument("--restarts", type=int, default=2)
    parser.add_argument(
        "--target-mode",
        choices=("auto", "absolute", "residual"),
        default="auto",
        help=(
            "Training target to fit. 'auto' uses training_target_* columns from the CSV; "
            "'absolute' fits target_x/y; 'residual' fits target_x/y - odom_x/y."
        ),
    )
    parser.add_argument(
        "--input-normalization",
        choices=("standard", "none"),
        default="standard",
        help="Normalize ANN inputs before fitting and save the transform in the model.",
    )
    parser.add_argument(
        "--validation-fraction",
        type=float,
        default=0.20,
        help="Fraction of samples to hold out for validation.",
    )
    parser.add_argument(
        "--validation-mode",
        choices=("shuffle", "tail"),
        default="tail",
        help=(
            "'shuffle' holds out samples randomly across the route; 'tail' uses the final "
            "fraction of the recorded route."
        ),
    )
    parser.add_argument(
        "--validation-seed",
        type=int,
        default=42,
        help="Random seed used when --validation-mode=shuffle.",
    )
    args = parser.parse_args()

    dataset_path = os.path.expanduser(args.dataset)
    model_path = os.path.expanduser(args.model)
    dataset_hash = file_sha256(dataset_path)
    rows, inputs, recorded_targets_m, odom_m, kf_target_m, recorded_target_mode = load_dataset(
        dataset_path,
    )
    targets_m, target_mode = targets_for_mode(
        kf_target_m,
        odom_m,
        recorded_targets_m,
        recorded_target_mode,
        args.target_mode,
    )
    targets = np.clip(targets_m / max(args.target_scale, 1e-6), -1.0, 1.0)
    input_mean, input_std = fit_input_normalizer(inputs, args.input_normalization)
    model_inputs = normalize_inputs(inputs, input_mean, input_std)

    validation_fraction = min(max(args.validation_fraction, 0.0), 0.5)
    validation_count = int(round(len(rows) * validation_fraction))
    if len(rows) - validation_count < 50:
        validation_count = 0

    if validation_count > 0 and args.validation_mode == "shuffle":
        split_rng = np.random.default_rng(args.validation_seed)
        shuffled_indices = split_rng.permutation(len(rows))
        validation_indices = np.sort(shuffled_indices[:validation_count])
        train_indices = np.sort(shuffled_indices[validation_count:])
    else:
        split_index = len(rows) - validation_count
        train_indices = np.arange(0, split_index)
        validation_indices = np.arange(split_index, len(rows))

    train_inputs = model_inputs[train_indices]
    train_targets = targets[train_indices]

    best_model = None
    best_score = math.inf
    for seed in range(args.restarts):
        model = BatchMlp(args.regularization, seed=11 + seed)
        loss = model.fit(train_inputs, train_targets, args.iterations, args.max_step_norm)

        if validation_count > 0:
            _, validation_ann_rmse, _, _ = evaluate_model(
                model,
                model_inputs[validation_indices],
                targets_m[validation_indices],
                odom_m[validation_indices],
                kf_target_m[validation_indices],
                target_mode,
                args.target_scale,
            )
            score = validation_ann_rmse
            print(
                f"restart={seed}, final_loss={loss:.9f}, "
                f"validation_ann_rmse_m={validation_ann_rmse:.6f}"
            )
        else:
            score = loss
            print(f"restart={seed}, final_loss={loss:.9f}")

        if score < best_score:
            best_score = score
            best_model = model

    print(f"samples={len(rows)}")
    print(f"dataset_sha256={dataset_hash}")
    print(f"training_samples={len(train_indices)}")
    print(f"validation_samples={validation_count}")
    print(f"validation_mode={args.validation_mode}")
    print(f"target_mode={target_mode}")

    train_odom_rmse, train_ann_rmse, train_improvement, train_fit_rmse = evaluate_model(
        best_model,
        model_inputs[train_indices],
        targets_m[train_indices],
        odom_m[train_indices],
        kf_target_m[train_indices],
        target_mode,
        args.target_scale,
    )
    print(f"train_odom_rmse_m={train_odom_rmse:.6f}")
    print(f"train_ann_rmse_m={train_ann_rmse:.6f}")
    print(f"train_improvement_pct={train_improvement:.1f}")
    print(f"train_target_fit_rmse_m={train_fit_rmse:.6f}")

    if validation_count > 0:
        val_odom_rmse, val_ann_rmse, val_improvement, val_fit_rmse = evaluate_model(
            best_model,
            model_inputs[validation_indices],
            targets_m[validation_indices],
            odom_m[validation_indices],
            kf_target_m[validation_indices],
            target_mode,
            args.target_scale,
        )
        print(f"validation_odom_rmse_m={val_odom_rmse:.6f}")
        print(f"validation_ann_rmse_m={val_ann_rmse:.6f}")
        print(f"validation_improvement_pct={val_improvement:.1f}")
        print(f"validation_target_fit_rmse_m={val_fit_rmse:.6f}")

    all_odom_rmse, all_ann_rmse, all_improvement, all_fit_rmse = evaluate_model(
        best_model,
        model_inputs,
        targets_m,
        odom_m,
        kf_target_m,
        target_mode,
        args.target_scale,
    )
    print(f"all_odom_rmse_m={all_odom_rmse:.6f}")
    print(f"all_ann_rmse_m={all_ann_rmse:.6f}")
    print(f"all_improvement_pct={all_improvement:.1f}")
    print(f"all_target_fit_rmse_m={all_fit_rmse:.6f}")

    model_dir = os.path.dirname(model_path)
    if model_dir:
        os.makedirs(model_dir, exist_ok=True)
    np.savez(
        model_path,
        w1=best_model.w1,
        b1=best_model.b1,
        w2=best_model.w2,
        b2=best_model.b2,
        w3=best_model.w3,
        b3=best_model.b3,
        target_scale=np.array([args.target_scale], dtype=float),
        target_mode=np.array([target_mode]),
        target_origin=np.array([math.nan, math.nan], dtype=float),
        samples=np.array([len(rows)], dtype=int),
        activation=np.array(["logsig"]),
        input_frame=np.array(["world"]),
        input_mean=input_mean,
        input_std=input_std,
        input_normalization=np.array([args.input_normalization]),
        validation_mode=np.array([args.validation_mode]),
        dataset_path=np.array([dataset_path]),
        dataset_sha256=np.array([dataset_hash]),
    )
    print(f"saved_model={model_path}")


if __name__ == "__main__":
    main()
