from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from sditt.models.wrcp_net_a1 import WRCPNetA1, _Adam
from sditt.models.network_b_features import DIRECT_FEATURE_NAMES, FEATURE_NAMES
from sditt.training_data.network_b import TARGET_NAMES, NetworkBDataset


MODEL_NAME = "WRCP-Net B"
MODEL_SCHEMA = "wrcp-net-b-force-power-mlp-v1"
DIRECT_MODEL_SCHEMA = "wrcp-net-b-direct-force-v2"
DIRECT_RESIDUAL_MODEL_SCHEMA = "wrcp-net-b-direct-force-residual-v3"
DIRECT_HERTZ_FIXED_MODEL_SCHEMA = "wrcp-net-b-direct-force-hertz-fixed-v3"
NORMAL_FORCE_MODES = {"absolute_power", "hertz_residual", "hertz_fixed"}


@dataclass(frozen=True)
class WRCPNetB:
    network: WRCPNetA1
    feature_mean: np.ndarray
    feature_scale: np.ndarray
    target_mean: np.ndarray
    target_scale: np.ndarray
    moment_scale_m: float
    friction_limit: float
    feature_clip: float
    feature_names: tuple[str, ...] = FEATURE_NAMES
    model_schema: str = MODEL_SCHEMA
    normal_force_mode: str = "absolute_power"
    ood_feature_scale: np.ndarray | None = None

    def predict(self, features: np.ndarray) -> np.ndarray:
        values = np.asarray(features, dtype=float)
        single = values.ndim == 1
        if single:
            values = values[np.newaxis, :]
        if values.ndim != 2 or values.shape[1] != len(self.feature_names):
            raise ValueError(f"network-B features must have {len(self.feature_names)} columns")
        if not np.isfinite(values).all():
            raise ValueError("network-B features contain NaN or Inf")
        normalized = np.clip(
            (values - self.feature_mean) / self.feature_scale,
            -self.feature_clip,
            self.feature_clip,
        ).astype(np.float32)
        transformed = np.asarray(self.network.forward(normalized), dtype=float)
        transformed = transformed * self.target_scale + self.target_mean
        result = _inverse_target_transform(
            transformed,
            values,
            moment_scale_m=self.moment_scale_m,
            friction_limit=self.friction_limit,
            normal_force_mode=self.normal_force_mode,
            feature_names=self.feature_names,
        )
        return result[0] if single else result

    def in_distribution(self, features: np.ndarray, *, threshold: float = 4.0) -> np.ndarray:
        values = np.asarray(features, dtype=float)
        if values.ndim == 1:
            values = values[np.newaxis, :]
        if threshold <= 0.0:
            raise ValueError("network-B distribution threshold must be positive")
        ood_scale = (
            self.feature_scale
            if self.ood_feature_scale is None
            else self.ood_feature_scale
        )
        standardized = np.abs((values - self.feature_mean) / ood_scale)
        return np.max(standardized, axis=1) <= float(threshold)


@dataclass(frozen=True)
class WRCPNetBTrainingResult:
    model: WRCPNetB
    model_path: Path
    metrics_path: Path
    best_epoch: int
    metrics: dict[str, object]


def train_wrcp_net_b(
    train: NetworkBDataset,
    validation: NetworkBDataset,
    test: NetworkBDataset | None = None,
    *,
    output_dir: str | Path,
    hidden_sizes: Sequence[int] = (64, 64),
    epochs: int = 800,
    batch_size: int = 256,
    learning_rate: float = 8.0e-4,
    weight_decay: float = 1.0e-5,
    huber_delta: float = 0.5,
    patience: int = 80,
    seed: int = 20260720,
    moment_scale_m: float = 0.46,
    friction_limit: float = 0.55,
    feature_clip: float = 6.0,
    feature_names: tuple[str, ...] | None = None,
    training_metadata: dict[str, object] | None = None,
    normal_force_mode: str = "absolute_power",
    ood_training_quantile: float = 0.9999,
    ood_calibration_threshold: float = 4.0,
    loss_weights: Sequence[float] | None = None,
) -> WRCPNetBTrainingResult:
    if len(train) == 0 or len(validation) == 0:
        raise ValueError("network-B train and validation datasets must be non-empty")
    if test is not None and len(test) == 0:
        raise ValueError("network-B test dataset must be non-empty when provided")
    _validate_seed_isolation(train, validation, test)
    if epochs <= 0 or batch_size <= 0 or patience <= 0:
        raise ValueError("epochs, batch_size, and patience must be positive")
    if normal_force_mode not in NORMAL_FORCE_MODES:
        raise ValueError(
            "network-B normal_force_mode must be 'absolute_power', "
            "'hertz_residual', or 'hertz_fixed'"
        )
    if not 0.0 < ood_training_quantile < 1.0:
        raise ValueError("network-B OOD training quantile must be between 0 and 1")
    if ood_calibration_threshold <= 0.0:
        raise ValueError("network-B OOD calibration threshold must be positive")
    target_loss_weights = np.ones(len(TARGET_NAMES), dtype=np.float32)
    if loss_weights is not None:
        target_loss_weights = np.asarray(tuple(loss_weights), dtype=np.float32)
        if target_loss_weights.shape != (len(TARGET_NAMES),):
            raise ValueError("network-B loss_weights must contain seven values")
        if np.any(target_loss_weights < 0.0) or not np.any(
            target_loss_weights > 0.0
        ):
            raise ValueError(
                "network-B loss_weights must be non-negative with a positive sum"
            )
    loss_weight_sum = float(np.sum(target_loss_weights))

    feature_names = train.feature_names if feature_names is None else feature_names
    if feature_names not in {FEATURE_NAMES, DIRECT_FEATURE_NAMES}:
        raise ValueError("unsupported network-B training feature schema")
    if train.features.shape[1] != len(feature_names):
        raise ValueError("network-B dataset width does not match selected feature schema")
    if (
        normal_force_mode in {"hertz_residual", "hertz_fixed"}
        and feature_names != DIRECT_FEATURE_NAMES
    ):
        raise ValueError("Hertz-based normal force currently requires Direct features")
    feature_mean = np.mean(train.features, axis=0)
    feature_scale = np.std(train.features, axis=0)
    feature_scale = np.where(feature_scale > 1.0e-12, feature_scale, 1.0)
    ood_quantile_distance = np.quantile(
        np.abs(train.features - feature_mean),
        float(ood_training_quantile),
        axis=0,
    )
    ood_feature_scale = np.maximum(
        feature_scale,
        ood_quantile_distance / float(ood_calibration_threshold),
    )
    ood_feature_scale = np.where(
        ood_feature_scale > 1.0e-12,
        ood_feature_scale,
        1.0,
    )
    x_train = _normalize_features(train.features, feature_mean, feature_scale, feature_clip)
    x_validation = _normalize_features(validation.features, feature_mean, feature_scale, feature_clip)
    transformed_train = _target_transform(
        train.targets,
        train.features,
        moment_scale_m=moment_scale_m,
        normal_force_mode=normal_force_mode,
        feature_names=feature_names,
    )
    transformed_validation = _target_transform(
        validation.targets,
        validation.features,
        moment_scale_m=moment_scale_m,
        normal_force_mode=normal_force_mode,
        feature_names=feature_names,
    )
    target_mean = np.mean(transformed_train, axis=0)
    target_scale = np.std(transformed_train, axis=0)
    target_scale = np.where(target_scale > 1.0e-12, target_scale, 1.0)
    y_train = ((transformed_train - target_mean) / target_scale).astype(np.float32)
    y_validation = ((transformed_validation - target_mean) / target_scale).astype(np.float32)

    network = WRCPNetA1(
        input_size=len(feature_names),
        hidden_sizes=hidden_sizes,
        output_size=len(TARGET_NAMES),
        seed=seed,
    )
    optimizer = _Adam(network, learning_rate=learning_rate)
    rng = np.random.default_rng(seed)
    best_loss = float("inf")
    best_epoch = 0
    best_parameters = network.copy_parameters()
    stale_epochs = 0
    for epoch in range(int(epochs)):
        permutation = rng.permutation(len(train))
        for start in range(0, len(train), int(batch_size)):
            indexes = permutation[start : start + int(batch_size)]
            output, cache = network.forward(x_train[indexes], return_cache=True)
            difference = np.asarray(output, dtype=np.float32) - y_train[indexes]
            absolute = np.abs(difference)
            gradient = np.where(
                absolute <= huber_delta,
                difference,
                float(huber_delta) * np.sign(difference),
            )
            gradient *= target_loss_weights[np.newaxis, :]
            gradient /= indexes.size * loss_weight_sum
            weight_gradients, bias_gradients = network.backward(
                cache,
                gradient,
                weight_decay=weight_decay,
            )
            optimizer.step(network, weight_gradients, bias_gradients)
        validation_difference = np.asarray(network.forward(x_validation), dtype=float) - y_validation
        validation_loss = _huber_loss(
            validation_difference,
            huber_delta,
            weights=target_loss_weights,
        )
        if validation_loss < best_loss - 1.0e-6:
            best_loss = validation_loss
            best_epoch = epoch
            best_parameters = network.copy_parameters()
            stale_epochs = 0
        else:
            stale_epochs += 1
        if stale_epochs >= patience:
            break
    network.restore_parameters(best_parameters)
    model = WRCPNetB(
        network=network,
        feature_mean=feature_mean,
        feature_scale=feature_scale,
        target_mean=target_mean,
        target_scale=target_scale,
        moment_scale_m=float(moment_scale_m),
        friction_limit=float(friction_limit),
        feature_clip=float(feature_clip),
        feature_names=feature_names,
        model_schema=(
            (
                DIRECT_RESIDUAL_MODEL_SCHEMA
                if normal_force_mode == "hertz_residual"
                else DIRECT_HERTZ_FIXED_MODEL_SCHEMA
            )
            if feature_names == DIRECT_FEATURE_NAMES
            and normal_force_mode in {"hertz_residual", "hertz_fixed"}
            else (
                DIRECT_MODEL_SCHEMA
                if feature_names == DIRECT_FEATURE_NAMES
                else MODEL_SCHEMA
            )
        ),
        normal_force_mode=normal_force_mode,
        ood_feature_scale=ood_feature_scale,
    )

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    model_path = output / "model.npz"
    save_wrcp_net_b(model_path, model)
    split = {
        "train_seeds": sorted(int(value) for value in np.unique(train.irregularity_seed)),
        "validation_seeds": sorted(
            int(value) for value in np.unique(validation.irregularity_seed)
        ),
        "train_rows": len(train),
        "validation_rows": len(validation),
    }
    if test is not None:
        split["test_seeds"] = sorted(
            int(value) for value in np.unique(test.irregularity_seed)
        )
        split["test_rows"] = len(test)
    metrics: dict[str, object] = {
        "model": MODEL_NAME,
        "schema": model.model_schema,
        "best_epoch": int(best_epoch),
        "best_validation_huber_loss": float(best_loss),
        "hidden_sizes": [int(value) for value in hidden_sizes],
        "training_configuration": {
            "epochs": int(epochs),
            "batch_size": int(batch_size),
            "learning_rate": float(learning_rate),
            "weight_decay": float(weight_decay),
            "huber_delta": float(huber_delta),
            "patience": int(patience),
            "seed": int(seed),
            "friction_limit": float(friction_limit),
            "feature_clip": float(feature_clip),
            "normal_force_mode": normal_force_mode,
            "ood_training_quantile": float(ood_training_quantile),
            "ood_calibration_threshold": float(ood_calibration_threshold),
            "loss_weights": [float(value) for value in target_loss_weights],
        },
        "split": split,
        "train": force_metrics(train.targets, model.predict(train.features)),
        "validation": force_metrics(validation.targets, model.predict(validation.features)),
        "model_sha256": _sha256_file(model_path),
        "ood": {
            "calibration_threshold": float(ood_calibration_threshold),
            "training_quantile": float(ood_training_quantile),
            "train_out_of_distribution_fraction": float(
                np.mean(
                    ~model.in_distribution(
                        train.features,
                        threshold=float(ood_calibration_threshold),
                    )
                )
            ),
            "validation_out_of_distribution_fraction": float(
                np.mean(
                    ~model.in_distribution(
                        validation.features,
                        threshold=float(ood_calibration_threshold),
                    )
                )
            ),
        },
    }
    if training_metadata is not None:
        metrics["training_metadata"] = training_metadata
    if test is not None:
        metrics["test"] = force_metrics(test.targets, model.predict(test.features))
    metrics_path = output / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return WRCPNetBTrainingResult(model, model_path, metrics_path, best_epoch, metrics)


def save_wrcp_net_b(path: str | Path, model: WRCPNetB) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    values: dict[str, np.ndarray] = {
        "schema": np.array(model.model_schema),
        "feature_names": np.asarray(model.feature_names),
        "target_names": np.asarray(TARGET_NAMES),
        "layer_sizes": np.asarray(model.network.layer_sizes, dtype=np.int64),
        "feature_mean": model.feature_mean,
        "feature_scale": model.feature_scale,
        "target_mean": model.target_mean,
        "target_scale": model.target_scale,
        "moment_scale_m": np.array(model.moment_scale_m),
        "friction_limit": np.array(model.friction_limit),
        "feature_clip": np.array(model.feature_clip),
        "normal_force_mode": np.array(model.normal_force_mode),
        "ood_feature_scale": (
            model.feature_scale
            if model.ood_feature_scale is None
            else model.ood_feature_scale
        ),
    }
    for index, (weight, bias) in enumerate(zip(model.network.weights, model.network.biases, strict=True)):
        values[f"weight_{index}"] = weight
        values[f"bias_{index}"] = bias
    np.savez_compressed(destination, **values)


def load_wrcp_net_b(path: str | Path) -> WRCPNetB:
    with np.load(path, allow_pickle=False) as payload:
        schema = str(payload["schema"].item())
        if schema not in {
            MODEL_SCHEMA,
            DIRECT_MODEL_SCHEMA,
            DIRECT_RESIDUAL_MODEL_SCHEMA,
            DIRECT_HERTZ_FIXED_MODEL_SCHEMA,
        }:
            raise ValueError("unsupported WRCP-Net B model schema")
        feature_names = tuple(str(value) for value in payload["feature_names"])
        expected_features = (
            DIRECT_FEATURE_NAMES
            if schema
            in {
                DIRECT_MODEL_SCHEMA,
                DIRECT_RESIDUAL_MODEL_SCHEMA,
                DIRECT_HERTZ_FIXED_MODEL_SCHEMA,
            }
            else FEATURE_NAMES
        )
        if feature_names != expected_features:
            raise ValueError("WRCP-Net B feature schema does not match runtime")
        if tuple(str(value) for value in payload["target_names"]) != TARGET_NAMES:
            raise ValueError("WRCP-Net B target schema does not match runtime")
        sizes = tuple(int(value) for value in payload["layer_sizes"])
        network = WRCPNetA1(input_size=sizes[0], hidden_sizes=sizes[1:-1], output_size=sizes[-1])
        for index in range(len(network.weights)):
            network.weights[index][...] = payload[f"weight_{index}"]
            network.biases[index][...] = payload[f"bias_{index}"]
        return WRCPNetB(
            network=network,
            feature_mean=np.asarray(payload["feature_mean"], dtype=float),
            feature_scale=np.asarray(payload["feature_scale"], dtype=float),
            target_mean=np.asarray(payload["target_mean"], dtype=float),
            target_scale=np.asarray(payload["target_scale"], dtype=float),
            moment_scale_m=float(payload["moment_scale_m"].item()),
            friction_limit=float(payload["friction_limit"].item()),
            feature_clip=float(payload["feature_clip"].item()),
            feature_names=feature_names,
            model_schema=schema,
            normal_force_mode=(
                str(payload["normal_force_mode"].item())
                if "normal_force_mode" in payload.files
                else (
                    "hertz_residual"
                    if schema == DIRECT_RESIDUAL_MODEL_SCHEMA
                    else (
                        "hertz_fixed"
                        if schema == DIRECT_HERTZ_FIXED_MODEL_SCHEMA
                        else "absolute_power"
                    )
                )
            ),
            ood_feature_scale=(
                np.asarray(payload["ood_feature_scale"], dtype=float)
                if "ood_feature_scale" in payload.files
                else np.asarray(payload["feature_scale"], dtype=float)
            ),
        )


def force_metrics(reference: np.ndarray, prediction: np.ndarray) -> dict[str, object]:
    truth = np.asarray(reference, dtype=float)
    predicted = np.asarray(prediction, dtype=float)
    if truth.shape != predicted.shape or truth.ndim != 2 or truth.shape[1] != len(TARGET_NAMES):
        raise ValueError("force metric arrays have incompatible shapes")
    error = predicted - truth
    span = np.ptp(truth, axis=0)
    rms_reference = np.sqrt(np.mean(np.square(truth), axis=0))
    denominator = np.where(span > 1.0e-9, span, np.maximum(rms_reference, 1.0))
    per_target = {
        name: {
            "mae": float(np.mean(np.abs(error[:, index]))),
            "rmse": float(np.sqrt(np.mean(np.square(error[:, index])))),
            "nrmse_percent": float(100.0 * np.sqrt(np.mean(np.square(error[:, index]))) / denominator[index]),
        }
        for index, name in enumerate(TARGET_NAMES)
    }
    combined_reference = np.column_stack((truth[:, 1], truth[:, 2], truth[:, 3] + truth[:, 0]))
    combined_prediction = np.column_stack((predicted[:, 1], predicted[:, 2], predicted[:, 3] + predicted[:, 0]))
    vector_error = combined_prediction - combined_reference
    vector_scale = max(float(np.ptp(np.linalg.norm(combined_reference, axis=1))), 1.0)
    vector_rmse = float(np.sqrt(np.mean(np.sum(np.square(vector_error), axis=1))))
    return {
        "per_target": per_target,
        "combined_force_vector_rmse_N": vector_rmse,
        "combined_force_vector_nrmse_percent": 100.0 * vector_rmse / vector_scale,
    }


def _normalize_features(values: np.ndarray, mean: np.ndarray, scale: np.ndarray, clip: float) -> np.ndarray:
    return np.clip((values - mean) / scale, -clip, clip).astype(np.float32)


def _target_transform(
    targets: np.ndarray,
    features: np.ndarray,
    *,
    moment_scale_m: float,
    normal_force_mode: str = "absolute_power",
    feature_names: tuple[str, ...] | None = None,
) -> np.ndarray:
    values = np.asarray(targets, dtype=float)
    normal = np.maximum(values[:, 0], 1.0e-3)
    divisor = np.maximum(normal, 1.0)
    transformed_normal = normal ** (2.0 / 3.0)
    if normal_force_mode == "hertz_residual":
        names = _feature_names_for_values(features, feature_names)
        hertz_normal = _hertz_normal_force(features, names)
        transformed_normal = (normal - hertz_normal) / np.maximum(hertz_normal, 1.0)
    elif normal_force_mode == "hertz_fixed":
        transformed_normal = np.zeros_like(normal)
    elif normal_force_mode != "absolute_power":
        raise ValueError(f"unsupported Network B normal-force mode: {normal_force_mode}")
    return np.column_stack(
        (
            transformed_normal,
            values[:, 1:4] / divisor[:, np.newaxis],
            values[:, 4:7] / (divisor * float(moment_scale_m))[:, np.newaxis],
        )
    )


def _inverse_target_transform(
    transformed: np.ndarray,
    features: np.ndarray,
    *,
    moment_scale_m: float,
    friction_limit: float,
    normal_force_mode: str = "absolute_power",
    feature_names: tuple[str, ...] | None = None,
) -> np.ndarray:
    values = np.asarray(transformed, dtype=float)
    if normal_force_mode == "absolute_power":
        normal = np.maximum(values[:, 0], 0.0) ** 1.5
    elif normal_force_mode == "hertz_residual":
        names = _feature_names_for_values(features, feature_names)
        hertz_normal = _hertz_normal_force(features, names)
        normal = np.maximum(hertz_normal * (1.0 + values[:, 0]), 0.0)
    elif normal_force_mode == "hertz_fixed":
        names = _feature_names_for_values(features, feature_names)
        normal = _hertz_normal_force(features, names)
    else:
        raise ValueError(f"unsupported Network B normal-force mode: {normal_force_mode}")
    force_ratio = values[:, 1:4].copy()
    ratio_norm = np.linalg.norm(force_ratio, axis=1)
    force_ratio *= np.minimum(1.0, float(friction_limit) / np.maximum(ratio_norm, 1.0e-12))[:, np.newaxis]
    moment_ratio = np.clip(values[:, 4:7], -1.0, 1.0)
    return np.column_stack(
        (
            normal,
            force_ratio * normal[:, np.newaxis],
            moment_ratio * (normal * float(moment_scale_m))[:, np.newaxis],
        )
    )


def _feature_names_for_values(
    features: np.ndarray,
    feature_names: tuple[str, ...] | None,
) -> tuple[str, ...]:
    if feature_names is not None:
        return feature_names
    width = np.asarray(features).shape[1]
    if width == len(DIRECT_FEATURE_NAMES):
        return DIRECT_FEATURE_NAMES
    if width == len(FEATURE_NAMES):
        return FEATURE_NAMES
    raise ValueError("cannot infer Network B feature schema")


def _hertz_normal_force(
    features: np.ndarray,
    feature_names: tuple[str, ...],
) -> np.ndarray:
    values = np.asarray(features, dtype=float)
    penetration_index = feature_names.index("corrected_normal_penetration_m")
    permeability_index = feature_names.index(
        "elastic_permeability_m_per_N_2over3"
    )
    penetration = np.maximum(values[:, penetration_index], 0.0)
    permeability = np.maximum(values[:, permeability_index], 1.0e-20)
    return (penetration / permeability) ** 1.5


def _huber_loss(
    difference: np.ndarray,
    delta: float,
    *,
    weights: np.ndarray | None = None,
) -> float:
    absolute = np.abs(difference)
    loss = np.where(
        absolute <= delta,
        0.5 * difference**2,
        delta * (absolute - 0.5 * delta),
    )
    if weights is None:
        return float(np.mean(loss))
    values = np.asarray(weights, dtype=float)
    if values.shape != (loss.shape[1],):
        raise ValueError("Huber loss weights do not match output width")
    return float(np.sum(loss * values[np.newaxis, :]) / (loss.shape[0] * np.sum(values)))


def _validate_seed_isolation(
    train: NetworkBDataset,
    validation: NetworkBDataset,
    test: NetworkBDataset | None = None,
) -> None:
    groups = [
        set(int(value) for value in np.unique(item.irregularity_seed))
        for item in (train, validation)
    ]
    if groups[0] & groups[1]:
        raise ValueError("network-B dataset splits must use disjoint irregularity seeds")
    if test is None:
        return
    test_group = set(int(value) for value in np.unique(test.irregularity_seed))
    if groups[0] & test_group or groups[1] & test_group:
        raise ValueError("network-B dataset splits must use disjoint irregularity seeds")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()
