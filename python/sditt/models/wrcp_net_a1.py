from __future__ import annotations

import argparse
import csv
import hashlib
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from sditt.training_data.network_a import (
    FEATURE_NAMES,
    LABEL_NAMES,
    MAX_PATCHES,
    NetworkADataset,
    load_network_a_dataset,
)


MODEL_NAME = "WRCP-Net A1"
MODEL_ID = "wrcp-net-a1"
CLASS_NAMES = ("no_contact", "single_patch", "double_patch")
OUTPUT_SIZE = len(CLASS_NAMES) + MAX_PATCHES * len(LABEL_NAMES)


@dataclass(frozen=True)
class WRCPNetA1Prediction:
    patch_count: np.ndarray
    patch_mask: np.ndarray
    labels: np.ndarray
    class_probability: np.ndarray


@dataclass(frozen=True)
class WRCPNetA1TrainingResult:
    output_dir: Path
    model_path: Path
    manifest_path: Path
    metrics_path: Path
    best_epoch: int
    metrics: dict[str, Any]


class WRCPNetA1:
    """Two-hidden-layer multi-task MLP for network-A contact geometry."""

    def __init__(
        self,
        *,
        input_size: int = len(FEATURE_NAMES),
        hidden_sizes: Sequence[int] = (64, 64),
        output_size: int = OUTPUT_SIZE,
        seed: int = 20260719,
    ) -> None:
        sizes = (int(input_size), *(int(value) for value in hidden_sizes), int(output_size))
        if any(value <= 0 for value in sizes):
            raise ValueError("all WRCP-Net layer sizes must be positive")
        self.layer_sizes = sizes
        rng = np.random.default_rng(seed)
        self.weights: list[np.ndarray] = []
        self.biases: list[np.ndarray] = []
        for fan_in, fan_out in zip(sizes[:-1], sizes[1:], strict=True):
            scale = np.sqrt(2.0 / fan_in) if fan_out != sizes[-1] else np.sqrt(1.0 / fan_in)
            self.weights.append(rng.normal(0.0, scale, size=(fan_in, fan_out)).astype(np.float32))
            self.biases.append(np.zeros((fan_out,), dtype=np.float32))

    def forward(
        self,
        features: np.ndarray,
        *,
        return_cache: bool = False,
    ) -> np.ndarray | tuple[np.ndarray, tuple[list[np.ndarray], list[np.ndarray]]]:
        value = np.asarray(features, dtype=np.float32)
        activations = [value]
        preactivations: list[np.ndarray] = []
        for index, (weight, bias) in enumerate(zip(self.weights, self.biases, strict=True)):
            linear = value @ weight + bias
            preactivations.append(linear)
            value = np.maximum(linear, 0.0) if index < len(self.weights) - 1 else linear
            activations.append(value)
        if return_cache:
            return value, (activations, preactivations)
        return value

    def backward(
        self,
        cache: tuple[list[np.ndarray], list[np.ndarray]],
        output_gradient: np.ndarray,
        *,
        weight_decay: float,
    ) -> tuple[list[np.ndarray], list[np.ndarray]]:
        activations, preactivations = cache
        gradient = np.asarray(output_gradient, dtype=np.float32)
        weight_gradients: list[np.ndarray] = [np.empty_like(value) for value in self.weights]
        bias_gradients: list[np.ndarray] = [np.empty_like(value) for value in self.biases]
        for index in range(len(self.weights) - 1, -1, -1):
            if index < len(self.weights) - 1:
                gradient = gradient * (preactivations[index] > 0.0)
            weight_gradients[index] = (
                activations[index].T @ gradient + float(weight_decay) * self.weights[index]
            ).astype(np.float32, copy=False)
            bias_gradients[index] = np.sum(gradient, axis=0).astype(np.float32, copy=False)
            if index:
                gradient = gradient @ self.weights[index].T
        return weight_gradients, bias_gradients

    def copy_parameters(self) -> tuple[list[np.ndarray], list[np.ndarray]]:
        return (
            [value.copy() for value in self.weights],
            [value.copy() for value in self.biases],
        )

    def restore_parameters(self, values: tuple[Sequence[np.ndarray], Sequence[np.ndarray]]) -> None:
        weights, biases = values
        if len(weights) != len(self.weights) or len(biases) != len(self.biases):
            raise ValueError("WRCP-Net parameter depth mismatch")
        for target, source in zip(self.weights, weights, strict=True):
            if target.shape != np.asarray(source).shape:
                raise ValueError("WRCP-Net weight shape mismatch")
            target[...] = source
        for target, source in zip(self.biases, biases, strict=True):
            if target.shape != np.asarray(source).shape:
                raise ValueError("WRCP-Net bias shape mismatch")
            target[...] = source


class _Adam:
    def __init__(
        self,
        model: WRCPNetA1,
        *,
        learning_rate: float,
        beta1: float = 0.9,
        beta2: float = 0.999,
        epsilon: float = 1.0e-8,
    ) -> None:
        self.learning_rate = float(learning_rate)
        self.beta1 = float(beta1)
        self.beta2 = float(beta2)
        self.epsilon = float(epsilon)
        self.step_index = 0
        parameters = [*model.weights, *model.biases]
        self.first = [np.zeros_like(value) for value in parameters]
        self.second = [np.zeros_like(value) for value in parameters]

    def step(
        self,
        model: WRCPNetA1,
        weight_gradients: Sequence[np.ndarray],
        bias_gradients: Sequence[np.ndarray],
    ) -> None:
        self.step_index += 1
        parameters = [*model.weights, *model.biases]
        gradients = [*weight_gradients, *bias_gradients]
        correction1 = 1.0 - self.beta1**self.step_index
        correction2 = 1.0 - self.beta2**self.step_index
        for index, (parameter, gradient) in enumerate(zip(parameters, gradients, strict=True)):
            self.first[index] *= self.beta1
            self.first[index] += (1.0 - self.beta1) * gradient
            self.second[index] *= self.beta2
            self.second[index] += (1.0 - self.beta2) * np.square(gradient)
            first_hat = self.first[index] / correction1
            second_hat = self.second[index] / correction2
            parameter -= self.learning_rate * first_hat / (np.sqrt(second_hat) + self.epsilon)


def _softmax(logits: np.ndarray) -> np.ndarray:
    shifted = logits - np.max(logits, axis=1, keepdims=True)
    exponential = np.exp(shifted)
    return exponential / np.sum(exponential, axis=1, keepdims=True)


def _loss_and_output_gradient(
    output: np.ndarray,
    *,
    patch_count: np.ndarray,
    patch_mask: np.ndarray,
    normalized_labels: np.ndarray,
    class_weights: np.ndarray,
    regression_weight: float,
    slot_weights: np.ndarray,
    huber_delta: float,
) -> tuple[float, float, float, np.ndarray]:
    batch_size = output.shape[0]
    logits = output[:, : len(CLASS_NAMES)]
    regression = output[:, len(CLASS_NAMES) :].reshape(batch_size, MAX_PATCHES, len(LABEL_NAMES))
    probability = _softmax(logits)
    classes = np.asarray(patch_count, dtype=int)
    sample_weight = np.asarray(class_weights, dtype=np.float32)[classes]
    class_denominator = max(float(np.sum(sample_weight)), 1.0)
    class_loss = -float(
        np.sum(sample_weight * np.log(np.maximum(probability[np.arange(batch_size), classes], 1.0e-12)))
        / class_denominator
    )
    class_gradient = probability
    class_gradient[np.arange(batch_size), classes] -= 1.0
    class_gradient *= (sample_weight / class_denominator)[:, None]

    difference = regression - normalized_labels
    absolute = np.abs(difference)
    element_loss = np.where(
        absolute <= huber_delta,
        0.5 * np.square(difference),
        huber_delta * (absolute - 0.5 * huber_delta),
    )
    element_gradient = np.where(
        absolute <= huber_delta,
        difference,
        huber_delta * np.sign(difference),
    )
    regression_mask = np.asarray(patch_mask, dtype=np.float32) * np.asarray(slot_weights, dtype=np.float32)[None, :]
    regression_denominator = max(float(np.sum(regression_mask)) * len(LABEL_NAMES), 1.0)
    regression_loss = float(np.sum(element_loss * regression_mask[:, :, None]) / regression_denominator)
    regression_gradient = (
        float(regression_weight)
        * element_gradient
        * regression_mask[:, :, None]
        / regression_denominator
    )

    gradient = np.zeros_like(output, dtype=np.float32)
    gradient[:, : len(CLASS_NAMES)] = class_gradient
    gradient[:, len(CLASS_NAMES) :] = regression_gradient.reshape(batch_size, -1)
    total_loss = class_loss + float(regression_weight) * regression_loss
    return total_loss, class_loss, regression_loss, gradient


def _normalized_arrays(
    dataset: NetworkADataset,
    *,
    feature_mean: np.ndarray,
    feature_std: np.ndarray,
    label_mean: np.ndarray,
    label_std: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    features = (np.asarray(dataset.features, dtype=np.float32) - feature_mean) / feature_std
    labels = (np.asarray(dataset.labels, dtype=np.float32) - label_mean[None, None, :]) / label_std[
        None, None, :
    ]
    return features.astype(np.float32, copy=False), labels.astype(np.float32, copy=False)


def _evaluate_loss(
    model: WRCPNetA1,
    features: np.ndarray,
    dataset: NetworkADataset,
    normalized_labels: np.ndarray,
    *,
    class_weights: np.ndarray,
    regression_weight: float,
    slot_weights: np.ndarray,
    huber_delta: float,
    batch_size: int,
) -> tuple[float, float, float]:
    totals = np.zeros((3,), dtype=float)
    observations = 0
    for start in range(0, len(dataset), batch_size):
        stop = min(start + batch_size, len(dataset))
        output = np.asarray(model.forward(features[start:stop]), dtype=np.float32)
        losses = _loss_and_output_gradient(
            output,
            patch_count=dataset.patch_count[start:stop],
            patch_mask=dataset.patch_mask[start:stop],
            normalized_labels=normalized_labels[start:stop],
            class_weights=class_weights,
            regression_weight=regression_weight,
            slot_weights=slot_weights,
            huber_delta=huber_delta,
        )[:3]
        count = stop - start
        totals += np.asarray(losses) * count
        observations += count
    return tuple((totals / max(observations, 1)).tolist())  # type: ignore[return-value]


def _prediction_from_output(
    output: np.ndarray,
    *,
    label_mean: np.ndarray,
    label_std: np.ndarray,
) -> WRCPNetA1Prediction:
    probability = _softmax(output[:, : len(CLASS_NAMES)])
    patch_count = np.argmax(probability, axis=1).astype(np.int8)
    patch_mask = np.arange(MAX_PATCHES)[None, :] < patch_count[:, None]
    normalized = output[:, len(CLASS_NAMES) :].reshape(-1, MAX_PATCHES, len(LABEL_NAMES))
    labels = normalized * label_std[None, None, :] + label_mean[None, None, :]
    labels = labels.astype(np.float32, copy=False)
    labels[~patch_mask] = 0.0
    return WRCPNetA1Prediction(
        patch_count=patch_count,
        patch_mask=patch_mask,
        labels=labels,
        class_probability=probability.astype(np.float32, copy=False),
    )


def _classification_metrics(expected: np.ndarray, predicted: np.ndarray) -> dict[str, Any]:
    expected_values = np.asarray(expected, dtype=int)
    predicted_values = np.asarray(predicted, dtype=int)
    matrix = np.zeros((len(CLASS_NAMES), len(CLASS_NAMES)), dtype=int)
    np.add.at(matrix, (expected_values, predicted_values), 1)
    per_class: dict[str, Any] = {}
    f1_values = []
    for index, name in enumerate(CLASS_NAMES):
        true_positive = int(matrix[index, index])
        false_positive = int(np.sum(matrix[:, index]) - true_positive)
        false_negative = int(np.sum(matrix[index, :]) - true_positive)
        precision = true_positive / max(true_positive + false_positive, 1)
        recall = true_positive / max(true_positive + false_negative, 1)
        f1 = 2.0 * precision * recall / max(precision + recall, 1.0e-30)
        f1_values.append(f1)
        per_class[name] = {
            "support": int(np.sum(matrix[index, :])),
            "precision": precision,
            "recall": recall,
            "f1": f1,
        }
    return {
        "accuracy": float(np.mean(expected_values == predicted_values)),
        "macro_f1": float(np.mean(f1_values)),
        "confusion_matrix_expected_rows_predicted_columns": matrix.tolist(),
        "per_class": per_class,
    }


def _regression_metrics(
    expected: NetworkADataset,
    predicted_labels: np.ndarray,
) -> dict[str, Any]:
    valid = expected.patch_mask
    difference = np.asarray(predicted_labels, dtype=float) - np.asarray(expected.labels, dtype=float)
    result: dict[str, Any] = {}
    for label_index, name in enumerate(LABEL_NAMES):
        values = difference[:, :, label_index][valid]
        result[name] = {
            "mae": float(np.mean(np.abs(values))) if values.size else None,
            "rmse": float(np.sqrt(np.mean(np.square(values)))) if values.size else None,
        }
    slot_result: dict[str, Any] = {}
    for slot in range(MAX_PATCHES):
        slot_valid = valid[:, slot]
        slot_difference = difference[:, slot, :][slot_valid]
        slot_result[str(slot + 1)] = {
            "patches": int(np.count_nonzero(slot_valid)),
            "normalized_label_rmse": None,
            "mean_absolute_label_error": (
                float(np.mean(np.abs(slot_difference))) if slot_difference.size else None
            ),
        }
    return {"by_label": result, "by_slot": slot_result}


def _evaluate_predictions(
    model: WRCPNetA1,
    dataset: NetworkADataset,
    *,
    feature_mean: np.ndarray,
    feature_std: np.ndarray,
    label_mean: np.ndarray,
    label_std: np.ndarray,
) -> tuple[dict[str, Any], WRCPNetA1Prediction]:
    features, _ = _normalized_arrays(
        dataset,
        feature_mean=feature_mean,
        feature_std=feature_std,
        label_mean=label_mean,
        label_std=label_std,
    )
    output = np.asarray(model.forward(features), dtype=np.float32)
    prediction = _prediction_from_output(output, label_mean=label_mean, label_std=label_std)
    raw_normalized_labels = output[:, len(CLASS_NAMES) :].reshape(
        -1, MAX_PATCHES, len(LABEL_NAMES)
    )
    raw_labels = (
        raw_normalized_labels * label_std[None, None, :] + label_mean[None, None, :]
    )
    metrics = {
        "classification": _classification_metrics(dataset.patch_count, prediction.patch_count),
        "regression_on_teacher_valid_slots": _regression_metrics(dataset, raw_labels),
        "end_to_end_regression_with_predicted_mask": _regression_metrics(dataset, prediction.labels),
    }
    return metrics, prediction


def _save_model(
    path: Path,
    model: WRCPNetA1,
    *,
    feature_mean: np.ndarray,
    feature_std: np.ndarray,
    label_mean: np.ndarray,
    label_std: np.ndarray,
) -> None:
    payload: dict[str, np.ndarray] = {
        "layer_sizes": np.asarray(model.layer_sizes, dtype=np.int64),
        "feature_mean": np.asarray(feature_mean, dtype=np.float32),
        "feature_std": np.asarray(feature_std, dtype=np.float32),
        "label_mean": np.asarray(label_mean, dtype=np.float32),
        "label_std": np.asarray(label_std, dtype=np.float32),
        "feature_names": np.asarray(FEATURE_NAMES),
        "label_names": np.asarray(LABEL_NAMES),
        "class_names": np.asarray(CLASS_NAMES),
    }
    for index, (weight, bias) in enumerate(zip(model.weights, model.biases, strict=True)):
        payload[f"weight_{index}"] = weight
        payload[f"bias_{index}"] = bias
    np.savez_compressed(path, **payload)


def load_wrcp_net_a1(path: str | Path) -> tuple[WRCPNetA1, dict[str, np.ndarray]]:
    with np.load(Path(path), allow_pickle=False) as archive:
        layer_sizes = tuple(int(value) for value in archive["layer_sizes"])
        model = WRCPNetA1(
            input_size=layer_sizes[0],
            hidden_sizes=layer_sizes[1:-1],
            output_size=layer_sizes[-1],
        )
        for index in range(len(model.weights)):
            model.weights[index][...] = archive[f"weight_{index}"]
            model.biases[index][...] = archive[f"bias_{index}"]
        normalization = {
            "feature_mean": archive["feature_mean"].copy(),
            "feature_std": archive["feature_std"].copy(),
            "label_mean": archive["label_mean"].copy(),
            "label_std": archive["label_std"].copy(),
        }
    return model, normalization


def predict_wrcp_net_a1(
    model: WRCPNetA1,
    features: np.ndarray,
    normalization: Mapping[str, np.ndarray],
) -> WRCPNetA1Prediction:
    values = np.asarray(features, dtype=np.float32)
    if values.ndim == 1:
        values = values[None, :]
    normalized = (values - normalization["feature_mean"]) / normalization["feature_std"]
    output = np.asarray(model.forward(normalized), dtype=np.float32)
    return _prediction_from_output(
        output,
        label_mean=normalization["label_mean"],
        label_std=normalization["label_std"],
    )


def train_wrcp_net_a1(
    *,
    dataset_dir: str | Path,
    output_dir: str | Path,
    epochs: int = 500,
    batch_size: int = 256,
    learning_rate: float = 5.0e-4,
    hidden_sizes: Sequence[int] = (64, 64),
    regression_weight: float = 2.0,
    second_slot_weight: float = 4.0,
    weight_decay: float = 1.0e-5,
    huber_delta: float = 1.0,
    patience: int = 60,
    seed: int = 20260719,
) -> WRCPNetA1TrainingResult:
    if epochs <= 0 or batch_size <= 0 or patience <= 0:
        raise ValueError("epochs, batch size and patience must be positive")
    started = time.perf_counter()
    dataset_root = Path(dataset_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    train = load_network_a_dataset(dataset_root, split="train", dtype=np.float32)
    validation = load_network_a_dataset(dataset_root, split="validation", dtype=np.float32)
    test = load_network_a_dataset(dataset_root, split="test", dtype=np.float32)
    normalization_payload = json.loads((dataset_root / "normalization.json").read_text(encoding="utf-8"))
    feature_mean = np.asarray(normalization_payload["feature_mean"], dtype=np.float32)
    feature_std = np.asarray(normalization_payload["feature_std"], dtype=np.float32)
    label_mean = np.asarray(normalization_payload["label_mean"], dtype=np.float32)
    label_std = np.asarray(normalization_payload["label_std"], dtype=np.float32)
    train_features, train_labels = _normalized_arrays(
        train,
        feature_mean=feature_mean,
        feature_std=feature_std,
        label_mean=label_mean,
        label_std=label_std,
    )
    validation_features, validation_labels = _normalized_arrays(
        validation,
        feature_mean=feature_mean,
        feature_std=feature_std,
        label_mean=label_mean,
        label_std=label_std,
    )

    class_counts = np.bincount(train.patch_count.astype(int), minlength=len(CLASS_NAMES)).astype(float)
    class_weights = np.sqrt(np.max(class_counts) / np.maximum(class_counts, 1.0)).astype(np.float32)
    class_weights /= np.mean(class_weights)
    slot_weights = np.asarray([1.0, float(second_slot_weight)], dtype=np.float32)
    model = WRCPNetA1(hidden_sizes=hidden_sizes, seed=seed)
    optimizer = _Adam(model, learning_rate=learning_rate)
    rng = np.random.default_rng(seed)
    history: list[dict[str, Any]] = []
    best_parameters = model.copy_parameters()
    best_validation = np.inf
    best_epoch = 0
    stale_epochs = 0

    for epoch in range(1, epochs + 1):
        indexes = rng.permutation(len(train))
        batch_totals = np.zeros((3,), dtype=float)
        observations = 0
        for start in range(0, len(indexes), batch_size):
            batch_indexes = indexes[start : start + batch_size]
            output, cache = model.forward(train_features[batch_indexes], return_cache=True)
            total_loss, class_loss, regression_loss, output_gradient = _loss_and_output_gradient(
                output,
                patch_count=train.patch_count[batch_indexes],
                patch_mask=train.patch_mask[batch_indexes],
                normalized_labels=train_labels[batch_indexes],
                class_weights=class_weights,
                regression_weight=regression_weight,
                slot_weights=slot_weights,
                huber_delta=huber_delta,
            )
            weight_gradients, bias_gradients = model.backward(
                cache,
                output_gradient,
                weight_decay=weight_decay,
            )
            optimizer.step(model, weight_gradients, bias_gradients)
            count = len(batch_indexes)
            batch_totals += np.asarray((total_loss, class_loss, regression_loss)) * count
            observations += count
        train_losses = batch_totals / max(observations, 1)
        validation_losses = _evaluate_loss(
            model,
            validation_features,
            validation,
            validation_labels,
            class_weights=class_weights,
            regression_weight=regression_weight,
            slot_weights=slot_weights,
            huber_delta=huber_delta,
            batch_size=batch_size,
        )
        history.append(
            {
                "epoch": epoch,
                "train_total": float(train_losses[0]),
                "train_classification": float(train_losses[1]),
                "train_regression": float(train_losses[2]),
                "validation_total": float(validation_losses[0]),
                "validation_classification": float(validation_losses[1]),
                "validation_regression": float(validation_losses[2]),
            }
        )
        if validation_losses[0] < best_validation - 1.0e-6:
            best_validation = validation_losses[0]
            best_parameters = model.copy_parameters()
            best_epoch = epoch
            stale_epochs = 0
        else:
            stale_epochs += 1
        if epoch == 1 or epoch % 10 == 0:
            print(
                f"epoch={epoch:03d} train={train_losses[0]:.6f} "
                f"validation={validation_losses[0]:.6f} best={best_validation:.6f}",
                flush=True,
            )
        if stale_epochs >= patience:
            print(f"early_stop epoch={epoch} best_epoch={best_epoch}", flush=True)
            break

    model.restore_parameters(best_parameters)
    validation_metrics, _ = _evaluate_predictions(
        model,
        validation,
        feature_mean=feature_mean,
        feature_std=feature_std,
        label_mean=label_mean,
        label_std=label_std,
    )
    test_metrics, test_prediction = _evaluate_predictions(
        model,
        test,
        feature_mean=feature_mean,
        feature_std=feature_std,
        label_mean=label_mean,
        label_std=label_std,
    )
    metrics = {
        "model_name": MODEL_NAME,
        "best_epoch": best_epoch,
        "best_validation_loss": float(best_validation),
        "validation": validation_metrics,
        "test": test_metrics,
    }

    model_path = destination / "model.npz"
    _save_model(
        model_path,
        model,
        feature_mean=feature_mean,
        feature_std=feature_std,
        label_mean=label_mean,
        label_std=label_std,
    )
    metrics_path = destination / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    with (destination / "history.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(history[0]))
        writer.writeheader()
        writer.writerows(history)
    np.savez_compressed(
        destination / "test_predictions.npz",
        expected_patch_count=test.patch_count,
        predicted_patch_count=test_prediction.patch_count,
        predicted_patch_mask=test_prediction.patch_mask,
        predicted_labels=test_prediction.labels,
        class_probability=test_prediction.class_probability,
        sample_id=test.metadata["sample_id"],
    )
    dataset_manifest_path = dataset_root / "manifest.json"
    manifest = {
        "model_name": MODEL_NAME,
        "model_id": MODEL_ID,
        "architecture": {
            "type": "multi_task_mlp",
            "layer_sizes": list(model.layer_sizes),
            "activation": "relu",
            "classification_head": list(CLASS_NAMES),
            "regression_shape": [MAX_PATCHES, len(LABEL_NAMES)],
        },
        "dataset": {
            "path": str(dataset_root.resolve()),
            "manifest_sha256": _sha256_file(dataset_manifest_path),
            "train_samples": len(train),
            "validation_samples": len(validation),
            "test_samples": len(test),
        },
        "normalization_source": "dataset train split",
        "hyperparameters": {
            "epochs_requested": int(epochs),
            "epochs_completed": len(history),
            "batch_size": int(batch_size),
            "learning_rate": float(learning_rate),
            "regression_weight": float(regression_weight),
            "second_slot_weight": float(second_slot_weight),
            "weight_decay": float(weight_decay),
            "huber_delta": float(huber_delta),
            "patience": int(patience),
            "seed": int(seed),
            "class_weights": class_weights.tolist(),
        },
        "best_epoch": best_epoch,
        "elapsed_seconds": time.perf_counter() - started,
        "model_sha256": _sha256_file(model_path),
        "source_sha256": _sha256_file(Path(__file__)),
        "metrics_file": metrics_path.name,
        "history_file": "history.csv",
        "test_predictions_file": "test_predictions.npz",
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_summary(destination / "summary.md", metrics, manifest)
    return WRCPNetA1TrainingResult(
        output_dir=destination,
        model_path=model_path,
        manifest_path=manifest_path,
        metrics_path=metrics_path,
        best_epoch=best_epoch,
        metrics=metrics,
    )


def _write_summary(path: Path, metrics: Mapping[str, Any], manifest: Mapping[str, Any]) -> None:
    classification = metrics["test"]["classification"]
    regression = metrics["test"]["regression_on_teacher_valid_slots"]["by_label"]
    lines = [
        f"# {MODEL_NAME} Training Summary",
        "",
        f"- best epoch: `{metrics['best_epoch']}`",
        f"- test contact-state accuracy: `{classification['accuracy']:.6f}`",
        f"- test macro F1: `{classification['macro_f1']:.6f}`",
        f"- test double-patch recall: `{classification['per_class']['double_patch']['recall']:.6f}`",
        f"- corrected rail-y MAE: `{regression['corrected_rail_y_m']['mae']:.9g} m`",
        f"- corrected vertical penetration MAE: `{regression['corrected_vertical_penetration_m']['mae']:.9g} m`",
        f"- corrected contact angle MAE: `{regression['corrected_contact_angle_rad']['mae']:.9g} rad`",
        f"- elapsed: `{manifest['elapsed_seconds']:.3f} s`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _sha256_file(path: Path, *, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_size):
            digest.update(chunk)
    return digest.hexdigest()


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=f"Train or inspect {MODEL_NAME}.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    train = subparsers.add_parser("train", help="Train the first network-A contact surrogate.")
    train.add_argument("--dataset", type=Path, default=Path("outputs/network_a_dataset_v1"))
    train.add_argument("--output", type=Path, default=Path("outputs/wrcp_net_a1"))
    train.add_argument("--epochs", type=int, default=500)
    train.add_argument("--batch-size", type=int, default=256)
    train.add_argument("--learning-rate", type=float, default=5.0e-4)
    train.add_argument("--hidden-sizes", type=int, nargs="+", default=(64, 64))
    train.add_argument("--regression-weight", type=float, default=2.0)
    train.add_argument("--second-slot-weight", type=float, default=4.0)
    train.add_argument("--weight-decay", type=float, default=1.0e-5)
    train.add_argument("--huber-delta", type=float, default=1.0)
    train.add_argument("--patience", type=int, default=60)
    train.add_argument("--seed", type=int, default=20260719)
    inspect = subparsers.add_parser("inspect", help="Print compact saved-model metrics.")
    inspect.add_argument("path", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argument_parser().parse_args(argv)
    if args.command == "train":
        result = train_wrcp_net_a1(
            dataset_dir=args.dataset,
            output_dir=args.output,
            epochs=args.epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            hidden_sizes=args.hidden_sizes,
            regression_weight=args.regression_weight,
            second_slot_weight=args.second_slot_weight,
            weight_decay=args.weight_decay,
            huber_delta=args.huber_delta,
            patience=args.patience,
            seed=args.seed,
        )
        print(f"model={MODEL_NAME}")
        print(f"best_epoch={result.best_epoch}")
        print(f"wrote {result.model_path}")
        print(f"wrote {result.metrics_path}")
        return 0
    metrics = json.loads((args.path / "metrics.json").read_text(encoding="utf-8"))
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
