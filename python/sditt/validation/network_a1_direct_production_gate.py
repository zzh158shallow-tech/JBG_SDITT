from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from sditt.models.wrcp_net_a1_direct import FEATURE_NAMES, HISTORY_NAMES, load_wrcp_net_a1_direct
from sditt.training_data.network_a_direct import load_direct_dataset


REQUIRED_TRAINING_SEEDS = 33
REQUIRED_TRANSITION_STEPS = 128
SPLIT_DISTANCE_LIMITS = {
    "train": 3.5,
    "validation": 3.5,
    "test": 3.75,
}
REQUIRED_CAL_ROLLOUT_STEPS = 600


def evaluate_transition_gate(
    *,
    model_path: str | Path,
    dataset_dir: str | Path,
    output_path: str | Path,
    required_training_seeds: int = REQUIRED_TRAINING_SEEDS,
    required_transition_steps: int = REQUIRED_TRANSITION_STEPS,
) -> dict[str, Any]:
    """Evaluate strict Preload-to-Cal coverage without using evaluation data for calibration."""

    model = load_wrcp_net_a1_direct(model_path)
    root = Path(dataset_dir)
    split_reports: dict[str, Any] = {}
    gates: dict[str, bool] = {
        "runtime_ood_threshold_eq_4": bool(np.isclose(model.ood_threshold, 4.0)),
    }
    for split, limit in SPLIT_DISTANCE_LIMITS.items():
        dataset = load_direct_dataset(root / f"{split}.npz")
        stage = np.asarray(dataset.metadata["stage"], dtype=str)
        step = np.asarray(dataset.metadata["step_index"], dtype=int)
        seeds = np.asarray(dataset.metadata["irregularity_seed"], dtype=int)
        wheelset = np.asarray(dataset.metadata["wheelset"], dtype=str)
        side = np.asarray(dataset.metadata["side"], dtype=str)
        source = np.asarray(dataset.metadata["source"], dtype=str)
        accepted = np.asarray(dataset.metadata["accepted_step_label"], dtype=bool)
        unique_seeds = sorted(int(value) for value in np.unique(seeds))

        cal1 = np.flatnonzero((stage == "Cal") & (step == 1))
        transition = np.flatnonzero(
            (stage == "Cal")
            & (step >= 1)
            & (step <= required_transition_steps)
        )
        feature_distance, feature_field = _feature_distances(model, dataset.features[transition])
        history_distance, history_field = _history_distances(
            model,
            dataset.history[transition],
            dataset.history_mask[transition],
        )
        topology_correct, topology_confidence = _topology_results(model, dataset, transition)
        rows = []
        for offset, row in enumerate(transition):
            rows.append(
                {
                    "seed": int(seeds[row]),
                    "step": int(step[row]),
                    "wheelset": str(wheelset[row]),
                    "side": str(side[row]),
                    "feature_distance": float(feature_distance[offset]),
                    "feature_field": str(feature_field[offset]),
                    "history_distance": float(history_distance[offset]),
                    "history_field": str(history_field[offset]),
                    "topology_correct": bool(topology_correct[offset]),
                    "topology_confidence": float(topology_confidence[offset]),
                }
            )

        expected_cal1_rows = 8 * len(unique_seeds)
        expected_transition_rows = (
            8 * len(unique_seeds) * int(required_transition_steps)
        )
        transition_coverage = _transition_window_coverage(
            stage=stage,
            step=step,
            seeds=seeds,
            wheelset=wheelset,
            side=side,
            required_steps=required_transition_steps,
        )
        finite = bool(
            np.isfinite(dataset.features).all()
            and np.isfinite(dataset.history[dataset.history_mask]).all()
            and np.isfinite(dataset.targets[dataset.patch_mask]).all()
        )
        all_formal = bool(np.all(accepted & (source == "full_cal_accepted")))
        max_feature = float(np.max(feature_distance)) if transition.size else float("inf")
        max_history = float(np.max(history_distance)) if transition.size else float("inf")
        split_passed = bool(
            finite
            and all_formal
            and transition.size == expected_transition_rows
            and transition_coverage["passed"]
            and max_feature <= limit
            and max_history <= limit
            and bool(np.all(topology_correct))
        )
        split_reports[split] = {
            "samples": len(dataset),
            "seeds": unique_seeds,
            "distance_limit": limit,
            "cal1_rows": int(cal1.size),
            "expected_cal1_rows": int(expected_cal1_rows),
            "transition_rows": int(transition.size),
            "expected_transition_rows": int(expected_transition_rows),
            "max_feature_distance": max_feature,
            "max_history_distance": max_history,
            "minimum_topology_confidence": (
                float(np.min(topology_confidence)) if transition.size else 0.0
            ),
            "topology_correct_rows": int(np.sum(topology_correct)),
            "finite": finite,
            "formal_accepted_only": all_formal,
            "transition_window_coverage": transition_coverage,
            "passed": split_passed,
            "worst_rows": sorted(
                rows,
                key=lambda row: max(
                    float(row["feature_distance"]),
                    float(row["history_distance"]),
                ),
                reverse=True,
            )[:32],
        }
        gates[f"{split}_transition_passed"] = split_passed

    training_seed_count = len(split_reports["train"]["seeds"])
    gates["training_seed_count_ge_required"] = training_seed_count >= required_training_seeds
    report = {
        "schema": "network-a1-direct-production-transition-gate-v2",
        "model_path": str(Path(model_path).resolve()),
        "dataset_dir": str(root.resolve()),
        "runtime_ood_threshold": float(model.ood_threshold),
        "required_training_seeds": int(required_training_seeds),
        "required_transition_steps": int(required_transition_steps),
        "split_distance_limits": SPLIT_DISTANCE_LIMITS,
        "splits": split_reports,
        "gates": gates,
        "passed": bool(all(gates.values())),
        "calibration_boundary": (
            "all normalization and OOD envelopes come from the training split; "
            "validation and test splits are acceptance-only"
        ),
    }
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def evaluate_rollout_gate(
    *,
    progress_path: str | Path,
    output_path: str | Path,
    required_cal_steps: int = REQUIRED_CAL_ROLLOUT_STEPS,
) -> dict[str, Any]:
    """Require a continuous, finite, nonzero-force Cal rollout with no retries."""

    path = Path(progress_path)
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    cal = [row for row in rows if row.get("stage") == "Cal"]
    numeric_columns = (
        "time",
        "dt",
        "front_mileage",
        "iterations",
        "retry_count",
        "contact_force_norm",
        "total_force_norm",
        "max_patch_force_z",
    )
    patch_columns = sorted(
        name for name in (rows[0].keys() if rows else ()) if name.startswith("patch_")
    )
    finite = bool(
        cal
        and all(
            np.isfinite(float(row[name]))
            for row in cal
            for name in (*numeric_columns, *patch_columns)
        )
    )
    steps = np.asarray([int(row["step"]) for row in cal], dtype=int)
    mileage = np.asarray([float(row["front_mileage"]) for row in cal], dtype=float)
    retries = np.asarray([int(row["retry_count"]) for row in cal], dtype=int)
    patch_forces = np.asarray(
        [[float(row[name]) for name in patch_columns] for row in cal],
        dtype=float,
    ) if cal and patch_columns else np.empty((0, 0), dtype=float)
    contiguous = bool(
        steps.size >= required_cal_steps
        and np.array_equal(steps[:required_cal_steps], np.arange(1, required_cal_steps + 1))
    )
    increasing_mileage = bool(
        mileage.size >= required_cal_steps
        and np.all(np.diff(mileage[:required_cal_steps]) > 0.0)
    )
    no_retries = bool(retries.size >= required_cal_steps and np.all(retries[:required_cal_steps] == 0))
    all_patch_forces_positive = bool(
        patch_forces.shape[0] >= required_cal_steps
        and patch_forces.shape[1] == 8
        and np.all(patch_forces[:required_cal_steps] > 0.0)
    )
    gates = {
        "cal_steps_ge_required": len(cal) >= required_cal_steps,
        "cal_steps_contiguous_from_1": contiguous,
        "finite": finite,
        "front_mileage_strictly_increasing": increasing_mileage,
        "zero_retries": no_retries,
        "eight_patch_force_columns_positive": all_patch_forces_positive,
    }
    report = {
        "schema": "network-a1-direct-production-rollout-gate-v1",
        "progress_path": str(path.resolve()),
        "required_cal_steps": int(required_cal_steps),
        "observed_cal_steps": len(cal),
        "patch_force_columns": patch_columns,
        "gates": gates,
        "passed": bool(all(gates.values())),
    }
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def _runtime_scale(value: np.ndarray | None, size: int) -> np.ndarray:
    if value is None:
        return np.ones((size,), dtype=float)
    return np.asarray(value, dtype=float).reshape(size)


def _feature_distances(model: Any, features: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normalized = np.abs((features - model.feature_mean) / model.feature_scale)
    normalized /= _runtime_scale(model.feature_ood_scale, len(FEATURE_NAMES))
    indexes = np.argmax(normalized, axis=1)
    return normalized[np.arange(len(normalized)), indexes], np.asarray(FEATURE_NAMES)[indexes]


def _history_distances(
    model: Any,
    history: np.ndarray,
    history_mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    distances = np.zeros((len(history),), dtype=float)
    fields = np.full((len(history),), "none", dtype="<U64")
    scale = _runtime_scale(model.history_ood_scale, len(HISTORY_NAMES))
    for row in range(len(history)):
        active = history[row, history_mask[row]]
        if not active.size:
            continue
        normalized = np.abs((active - model.history_mean) / model.history_scale) / scale
        flat = int(np.argmax(normalized))
        _, field = np.unravel_index(flat, normalized.shape)
        distances[row] = float(normalized.reshape(-1)[flat])
        fields[row] = HISTORY_NAMES[field]
    return distances, fields


def _topology_results(
    model: Any,
    dataset: Any,
    indexes: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    correct = np.zeros((indexes.size,), dtype=bool)
    confidence = np.zeros((indexes.size,), dtype=float)
    for offset, row in enumerate(indexes):
        logits, _ = model.forward_raw(
            dataset.features[row],
            dataset.history[row],
            dataset.history_mask[row],
        )
        shifted = logits - np.max(logits)
        probability = np.exp(shifted) / np.sum(np.exp(shifted))
        predicted = int(np.argmax(probability))
        confidence[offset] = float(probability[predicted])
        try:
            runtime = model.predict(
                features=dataset.features[row],
                history=dataset.history[row],
                history_mask=dataset.history_mask[row],
            )
        except RuntimeError:
            continue
        # Evaluate the deployed topology after confidence, accepted-history
        # hysteresis and query/topology consistency arbitration.  Raw argmax
        # alone does not describe the branch used by the full-case runtime.
        correct[offset] = len(runtime.geometry.patches) == int(dataset.patch_count[row])
    return correct, confidence


def _transition_window_coverage(
    *,
    stage: np.ndarray,
    step: np.ndarray,
    seeds: np.ndarray,
    wheelset: np.ndarray,
    side: np.ndarray,
    required_steps: int,
) -> dict[str, Any]:
    missing: list[str] = []
    expected = set(range(1, required_steps + 1))
    for seed in np.unique(seeds):
        for wheel in ("FF", "FR", "RF", "RR"):
            for rail_side in ("L", "R"):
                selected = (
                    (stage == "Cal")
                    & (seeds == seed)
                    & (wheelset == wheel)
                    & (side == rail_side)
                    & (step <= required_steps)
                )
                actual = set(int(value) for value in step[selected])
                absent = sorted(expected - actual)
                if absent:
                    missing.append(f"{int(seed)}:{wheel}-{rail_side}:{absent[:5]}")
    return {
        "required_steps_per_lane": int(required_steps),
        "missing_lane_windows": missing,
        "passed": not missing,
    }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate strict Network A1 production gates.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    transition = subparsers.add_parser("transition")
    transition.add_argument("--model", type=Path, required=True)
    transition.add_argument("--dataset-dir", type=Path, required=True)
    transition.add_argument("--output", type=Path, required=True)
    transition.add_argument("--required-training-seeds", type=int, default=REQUIRED_TRAINING_SEEDS)
    transition.add_argument("--required-transition-steps", type=int, default=REQUIRED_TRANSITION_STEPS)
    rollout = subparsers.add_parser("rollout")
    rollout.add_argument("--progress", type=Path, required=True)
    rollout.add_argument("--output", type=Path, required=True)
    rollout.add_argument("--required-cal-steps", type=int, default=REQUIRED_CAL_ROLLOUT_STEPS)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.command == "transition":
        report = evaluate_transition_gate(
            model_path=args.model,
            dataset_dir=args.dataset_dir,
            output_path=args.output,
            required_training_seeds=args.required_training_seeds,
            required_transition_steps=args.required_transition_steps,
        )
    else:
        report = evaluate_rollout_gate(
            progress_path=args.progress,
            output_path=args.output,
            required_cal_steps=args.required_cal_steps,
        )
    print(json.dumps({"passed": report["passed"], "gates": report["gates"]}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
