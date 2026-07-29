from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment

from sditt.models.wrcp_net_a1_direct import (
    geometry_from_training_targets,
    geometry_to_history,
    load_wrcp_net_a1_direct,
)
from sditt.training_data.network_a_direct import load_direct_dataset


def validate_network_a1_direct(
    *,
    model_path: str | Path,
    dataset_path: str | Path,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    model = load_wrcp_net_a1_direct(model_path)
    dataset = load_direct_dataset(dataset_path)
    stage = np.asarray(
        dataset.metadata.get("stage", np.full((len(dataset),), "")),
        dtype=str,
    )
    evaluation_rows = stage == "Cal"
    strict_predictions, strict_failures = _closed_loop_predictions(model, dataset)

    evaluation_model = replace(
        model,
        topology_confidence_min=0.0,
        ood_threshold=1.0e12,
    )
    confusion = np.zeros((3, 3), dtype=int)
    errors: dict[str, list[float]] = {
        "patch_start_y_m": [],
        "patch_end_y_m": [],
        "corrected_center_y_m": [],
        "corrected_vertical_penetration_m": [],
        "peak_vertical_penetration_m": [],
        "corrected_contact_angle_rad": [],
        "peak_contact_angle_rad": [],
        "shape_moment_1": [],
        "shape_moment_1p5": [],
        "shape_moment_2": [],
    }
    truth_moments: list[np.ndarray] = []
    prediction_moments: list[np.ndarray] = []
    evaluation_predictions, evaluation_failures = _closed_loop_predictions(
        evaluation_model,
        dataset,
    )
    hard_decode_failures = int(sum(evaluation_failures.values()))
    for row in range(len(dataset)):
        if not evaluation_rows[row]:
            continue
        expected_count = int(dataset.patch_count[row])
        truth = geometry_from_training_targets(
            features=dataset.features[row],
            targets=dataset.targets[row],
            patch_count=expected_count,
        )
        prediction = evaluation_predictions[row]
        if prediction is None:
            continue
        predicted_count = len(prediction.patches)
        confusion[expected_count, predicted_count] += 1
        if predicted_count != expected_count or expected_count == 0:
            continue
        cost = np.abs(
            np.asarray([patch.corrected_rail_point[0] for patch in prediction.patches])[:, None]
            - np.asarray([patch.corrected_rail_point[0] for patch in truth.patches])[None, :]
        )
        predicted_rows, truth_columns = linear_sum_assignment(cost)
        for predicted_index, truth_index in zip(predicted_rows, truth_columns, strict=True):
            predicted_patch = prediction.patches[int(predicted_index)]
            truth_patch = truth.patches[int(truth_index)]
            values = {
                "patch_start_y_m": (predicted_patch.start_y, truth_patch.start_y),
                "patch_end_y_m": (predicted_patch.end_y, truth_patch.end_y),
                "corrected_center_y_m": (
                    predicted_patch.corrected_rail_point[0],
                    truth_patch.corrected_rail_point[0],
                ),
                "corrected_vertical_penetration_m": (
                    predicted_patch.corrected_vertical_penetration,
                    truth_patch.corrected_vertical_penetration,
                ),
                "peak_vertical_penetration_m": (
                    predicted_patch.peak_vertical_penetration,
                    truth_patch.peak_vertical_penetration,
                ),
                "corrected_contact_angle_rad": (
                    predicted_patch.contact_angle,
                    truth_patch.contact_angle,
                ),
                "peak_contact_angle_rad": (
                    predicted_patch.peak_contact_angle,
                    truth_patch.peak_contact_angle,
                ),
            }
            for name, (actual, expected) in values.items():
                errors[name].append(float(actual - expected))
            for index, name in enumerate(("shape_moment_1", "shape_moment_1p5", "shape_moment_2")):
                errors[name].append(
                    float(predicted_patch.shape_moments[index] - truth_patch.shape_moments[index])
                )
            prediction_moments.append(np.asarray(predicted_patch.shape_moments, dtype=float))
            truth_moments.append(np.asarray(truth_patch.shape_moments, dtype=float))

    support = np.sum(confusion, axis=1)
    recall = np.divide(np.diag(confusion), np.maximum(support, 1))
    precision = np.divide(np.diag(confusion), np.maximum(np.sum(confusion, axis=0), 1))
    f1 = np.divide(2.0 * precision * recall, np.maximum(precision + recall, np.finfo(float).eps))
    regression = {
        name: _error_metrics(np.asarray(values, dtype=float)) for name, values in errors.items()
    }
    moment_nrmse = _moment_nrmse(
        np.asarray(truth_moments, dtype=float),
        np.asarray(prediction_moments, dtype=float),
    )
    moment_relative_rmse = _moment_relative_rmse(
        np.asarray(truth_moments, dtype=float),
        np.asarray(prediction_moments, dtype=float),
    )
    metrics = {
        "model_path": str(Path(model_path).resolve()),
        "dataset_path": str(Path(dataset_path).resolve()),
        "dataset_samples": len(dataset),
        "evaluated_cal_samples": int(np.sum(evaluation_rows)),
        "evaluation_history": "closed-loop-autoregressive",
        "classification": {
            "accuracy": float(np.trace(confusion) / max(np.sum(confusion), 1)),
            "macro_f1": float(np.mean(f1)),
            "macro_f1_supported_classes": float(np.mean(f1[support > 0])),
            "class_support": support.tolist(),
            "per_class_recall": recall.tolist(),
            "confusion_matrix": confusion.tolist(),
        },
        "regression_conditional_on_correct_topology": regression,
        "shape_moment_nrmse_percent": moment_nrmse.tolist(),
        "shape_moment_relative_rmse_percent": moment_relative_rmse.tolist(),
        "strict_guard_failures": strict_failures,
        "hard_decode_failures_with_guards_disabled": int(hard_decode_failures),
        "history_continuity": _history_continuity_metrics(
            dataset,
            strict_predictions,
            evaluation_predictions,
        ),
    }
    metrics["acceptance"] = _acceptance(metrics)
    if output_path is not None:
        destination = Path(output_path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return metrics


def _closed_loop_predictions(
    model: Any,
    dataset: Any,
) -> tuple[list[Any | None], dict[str, int]]:
    predictions: list[Any | None] = [None] * len(dataset)
    predicted_history: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    failures: dict[str, int] = {}
    stage = np.asarray(
        dataset.metadata.get("stage", np.full((len(dataset),), "")),
        dtype=str,
    )
    for row, previous_value in enumerate(dataset.previous_row):
        if stage[row] != "Cal":
            # Preload remains traditional in deployment.  It supplies the
            # stored history anchor for Cal-1 but is never predicted by A1.
            continue
        previous = int(previous_value)
        use_model_history = (
            previous >= 0
            and stage[row] == "Cal"
            and stage[previous] == "Cal"
        )
        if use_model_history:
            if previous not in predicted_history:
                key = "closed-loop history unavailable after an upstream failure"
                failures[key] = failures.get(key, 0) + 1
                continue
            history, history_mask = predicted_history[previous]
        else:
            # Preload is never neural in deployment, and Cal-1 consumes the
            # final traditional Preload geometry stored in the dataset.
            history = dataset.history[row]
            history_mask = dataset.history_mask[row]
        try:
            result = model.predict(
                features=dataset.features[row],
                history=history,
                history_mask=history_mask,
            )
        except RuntimeError as exc:
            key = str(exc).split(":", 1)[0]
            failures[key] = failures.get(key, 0) + 1
            continue
        predictions[row] = result.geometry
        predicted_history[row] = geometry_to_history(result.geometry)
    return predictions, failures


def _history_continuity_metrics(
    dataset: Any,
    strict_predictions: list[Any | None],
    evaluation_predictions: list[Any | None],
) -> dict[str, Any]:
    stage = np.asarray(
        dataset.metadata.get("stage", np.full((len(dataset),), "")),
        dtype=str,
    )
    previous = np.asarray(dataset.previous_row, dtype=int)
    transition = np.zeros((len(dataset),), dtype=bool)
    valid_previous = previous >= 0
    transition[valid_previous] = (
        (stage[valid_previous] == "Cal")
        & (stage[previous[valid_previous]] == "Preload")
    )

    depth = np.zeros((len(dataset),), dtype=int)
    for row, previous_row in enumerate(previous):
        if (
            previous_row >= 0
            and stage[row] == "Cal"
            and stage[previous_row] == "Cal"
        ):
            depth[row] = depth[previous_row] + 1

    def topology_accuracy(indexes: np.ndarray) -> float:
        if not indexes.size:
            return float("nan")
        correct = []
        for row in indexes:
            geometry = evaluation_predictions[int(row)]
            correct.append(
                geometry is not None
                and len(geometry.patches) == int(dataset.patch_count[int(row)])
            )
        return float(np.mean(correct))

    transition_indexes = np.flatnonzero(transition)
    depth_buckets: dict[str, dict[str, float | int]] = {}
    for name, selected in (
        ("depth_0", (stage == "Cal") & (depth == 0)),
        ("depth_1_to_8", (stage == "Cal") & (depth >= 1) & (depth <= 8)),
        ("depth_9_plus", (stage == "Cal") & (depth >= 9)),
    ):
        indexes = np.flatnonzero(selected)
        depth_buckets[name] = {
            "samples": int(indexes.size),
            "unguarded_topology_accuracy": topology_accuracy(indexes),
        }
    return {
        "preload_to_cal_transition": {
            "samples": int(transition_indexes.size),
            "strict_guard_pass_rate": (
                float(np.mean([strict_predictions[int(row)] is not None for row in transition_indexes]))
                if transition_indexes.size
                else float("nan")
            ),
            "unguarded_topology_accuracy": topology_accuracy(transition_indexes),
        },
        "closed_loop_depth_buckets": depth_buckets,
    }


def _error_metrics(error: np.ndarray) -> dict[str, float | int]:
    if not error.size:
        return {"samples": 0, "mae": float("nan"), "p95_absolute_error": float("nan"), "rmse": float("nan")}
    return {
        "samples": int(error.size),
        "mae": float(np.mean(np.abs(error))),
        "p95_absolute_error": float(np.quantile(np.abs(error), 0.95)),
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
    }


def _moment_nrmse(truth: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    if truth.size == 0:
        return np.full((3,), np.nan)
    rmse = np.sqrt(np.mean(np.square(prediction - truth), axis=0))
    scale = np.maximum(np.ptp(truth, axis=0), np.finfo(float).eps)
    return 100.0 * rmse / scale


def _moment_relative_rmse(truth: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    """Normalize moment RMSE by its physical magnitude, not split-local range."""

    if truth.size == 0:
        return np.full((3,), np.nan)
    rmse = np.sqrt(np.mean(np.square(prediction - truth), axis=0))
    scale = np.maximum(np.mean(np.abs(truth), axis=0), np.finfo(float).eps)
    return 100.0 * rmse / scale


def _acceptance(metrics: dict[str, Any]) -> dict[str, bool | None]:
    classification = metrics["classification"]
    regression = metrics["regression_conditional_on_correct_topology"]
    shape_names = ("shape_moment_1", "shape_moment_1p5", "shape_moment_2")
    shape_mae = np.asarray([regression[name]["mae"] for name in shape_names])
    shape_p95 = np.asarray(
        [regression[name]["p95_absolute_error"] for name in shape_names]
    )
    support = np.asarray(classification["class_support"], dtype=int)
    return {
        "topology_accuracy_ge_0p99": classification["accuracy"] >= 0.99,
        "supported_class_macro_f1_ge_0p98": (
            classification["macro_f1_supported_classes"] >= 0.98
        ),
        "no_contact_recall_ge_0p98": (
            None if support[0] == 0 else classification["per_class_recall"][0] >= 0.98
        ),
        "double_patch_recall_ge_0p99": (
            None if support[2] == 0 else classification["per_class_recall"][2] >= 0.99
        ),
        "center_mae_le_0p10mm": regression["corrected_center_y_m"]["mae"] <= 1.0e-4,
        "center_p95_le_0p30mm": regression["corrected_center_y_m"]["p95_absolute_error"] <= 3.0e-4,
        "penetration_mae_le_2um": regression["corrected_vertical_penetration_m"]["mae"] <= 2.0e-6,
        "penetration_p95_le_8um": regression["corrected_vertical_penetration_m"]["p95_absolute_error"] <= 8.0e-6,
        "angle_mae_le_0p002rad": regression["corrected_contact_angle_rad"]["mae"] <= 0.002,
        "angle_p95_le_0p006rad": regression["corrected_contact_angle_rad"]["p95_absolute_error"] <= 0.006,
        "shape_moments_nrmse_le_2percent": bool(
            np.all(np.asarray(metrics["shape_moment_nrmse_percent"]) <= 2.0)
        ),
        "shape_moments_relative_rmse_le_0p2percent": bool(
            np.all(
                np.asarray(metrics["shape_moment_relative_rmse_percent"]) <= 0.2
            )
        ),
        "shape_moments_mae_le_0p0005": bool(np.all(shape_mae <= 5.0e-4)),
        "shape_moments_p95_le_0p001": bool(np.all(shape_p95 <= 1.0e-3)),
        "shape_moments_physical_gate_passed": bool(
            np.all(
                np.asarray(metrics["shape_moment_relative_rmse_percent"]) <= 0.2
            )
            and np.all(shape_mae <= 5.0e-4)
            and np.all(shape_p95 <= 1.0e-3)
        ),
        "hard_decode_failures_eq_0": metrics["hard_decode_failures_with_guards_disabled"] == 0,
    }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate WRCP-Net A1 Direct static gates.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    metrics = validate_network_a1_direct(
        model_path=args.model,
        dataset_path=args.dataset,
        output_path=args.output,
    )
    print(json.dumps(metrics["acceptance"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
