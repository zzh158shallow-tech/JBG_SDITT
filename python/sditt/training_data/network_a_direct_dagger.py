from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from sditt.models.wrcp_net_a1_direct import (
    geometry_from_training_targets,
    geometry_to_history,
    load_wrcp_net_a1_direct,
)
from sditt.training_data.network_a_direct import (
    NetworkA1DirectDataset,
    load_direct_dataset,
    save_direct_dataset,
    validate_direct_dataset,
)


DAGGER_SCHEMA = "network-a-direct-dagger-dataset-v1"


def build_dagger_augmented_dataset(
    *,
    dataset_dir: str | Path,
    model_path: str | Path,
    output_dir: str | Path,
    low_confidence: float = 0.95,
    maximum_history_distance: float = 8.0,
    center_tolerance_m: float = 3.0e-4,
    penetration_tolerance_m: float = 8.0e-6,
    angle_tolerance_rad: float = 6.0e-3,
    shape_tolerance: float = 2.0e-2,
    round_index: int = 1,
) -> Path:
    """Aggregate teacher labels at histories induced by the deployed learner.

    Preload remains traditional.  Cal-1 is anchored to the final traditional
    Preload patch, while later Cal histories come from the learner until a
    strict guard would stop deployment.  The failing row is retained for
    teacher supervision, then the rollout is reset to the teacher so one bad
    branch cannot fill the dataset with unreachable downstream states.
    """

    if not 0.0 < low_confidence <= 1.0:
        raise ValueError("low_confidence must be in (0, 1]")
    if round_index <= 0:
        raise ValueError("round_index must be positive")
    if maximum_history_distance <= 0.0:
        raise ValueError("maximum_history_distance must be positive")

    source_dir = Path(dataset_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    train = load_direct_dataset(source_dir / "train.npz")
    validation = load_direct_dataset(source_dir / "validation.npz")
    test = load_direct_dataset(source_dir / "test.npz")
    model = load_wrcp_net_a1_direct(model_path)
    evaluation_model = replace(
        model,
        topology_confidence_min=0.0,
        ood_threshold=1.0e12,
    )

    stage = np.asarray(train.metadata.get("stage", np.full((len(train),), "")), dtype=str)
    source = np.asarray(train.metadata.get("source", np.full((len(train),), "")), dtype=str)
    learner_history: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    selected_rows: list[int] = []
    selected_history: list[np.ndarray] = []
    selected_history_mask: list[np.ndarray] = []
    records: list[dict[str, Any]] = []

    for row, previous_value in enumerate(train.previous_row):
        if stage[row] != "Cal" or source[row] == "sobol":
            continue
        previous = int(previous_value)
        if previous >= 0 and stage[previous] == "Cal" and previous in learner_history:
            history, history_mask = learner_history[previous]
        else:
            history = train.history[row].copy()
            history_mask = train.history_mask[row].copy()

        failure_message = ""
        prediction = None
        try:
            prediction = evaluation_model.predict(
                features=train.features[row],
                history=history,
                history_mask=history_mask,
            )
        except RuntimeError as exc:
            failure_message = str(exc)

        teacher_count = int(train.patch_count[row])
        teacher = geometry_from_training_targets(
            features=train.features[row],
            targets=train.targets[row],
            patch_count=teacher_count,
        )
        predicted_count = -1 if prediction is None else len(prediction.geometry.patches)
        confidence = (
            0.0
            if prediction is None
            else float(np.max(prediction.topology_probability))
        )
        feature_distance = (
            float("inf")
            if prediction is None
            else float(prediction.normalized_feature_distance)
        )
        history_distance = (
            float("inf")
            if prediction is None
            else float(prediction.normalized_history_distance)
        )
        reasons: list[str] = []
        if predicted_count != teacher_count:
            reasons.append("topology_mismatch")
        if confidence < low_confidence:
            reasons.append("low_confidence")
        if feature_distance > model.ood_threshold or history_distance > model.ood_threshold:
            reasons.append("ood_guard")
        if confidence < model.topology_confidence_min:
            reasons.append("confidence_guard")
        if prediction is None:
            reasons.append("hard_decode_failure")

        geometry_errors = _geometry_errors(prediction.geometry if prediction is not None else None, teacher)
        if geometry_errors["center_error_m"] > center_tolerance_m:
            reasons.append("center_error")
        if geometry_errors["penetration_error_m"] > penetration_tolerance_m:
            reasons.append("penetration_error")
        if geometry_errors["angle_error_rad"] > angle_tolerance_rad:
            reasons.append("angle_error")
        if geometry_errors["shape_error"] > shape_tolerance:
            reasons.append("shape_error")

        history_is_trainable = (
            np.isfinite(feature_distance)
            and np.isfinite(history_distance)
            and feature_distance <= maximum_history_distance
            and history_distance <= maximum_history_distance
        )
        if reasons and history_is_trainable:
            selected_rows.append(row)
            selected_history.append(np.asarray(history, dtype=float).copy())
            selected_history_mask.append(np.asarray(history_mask, dtype=bool).copy())
            records.append(
                {
                    "parent_row": int(row),
                    "sample_id": str(train.metadata["sample_id"][row]),
                    "group_id": str(train.metadata["group_id"][row]),
                    "irregularity_seed": int(train.metadata["irregularity_seed"][row]),
                    "wheelset": str(train.metadata["wheelset"][row]),
                    "side": str(train.metadata["side"][row]),
                    "step_index": int(train.metadata["step_index"][row]),
                    "teacher_patch_count": teacher_count,
                    "predicted_patch_count": predicted_count,
                    "confidence": confidence,
                    "feature_distance": feature_distance,
                    "history_distance": history_distance,
                    **geometry_errors,
                    "reasons": reasons,
                    "failure": failure_message,
                }
            )

        strict_guard_failure = (
            prediction is None
            or feature_distance > model.ood_threshold
            or history_distance > model.ood_threshold
            or confidence < model.topology_confidence_min
        )
        if strict_guard_failure:
            learner_history[row] = geometry_to_history(teacher)
        else:
            learner_history[row] = geometry_to_history(prediction.geometry)

    dagger = _build_dagger_rows(
        train,
        np.asarray(selected_rows, dtype=int),
        np.asarray(selected_history, dtype=float),
        np.asarray(selected_history_mask, dtype=bool),
        records,
        round_index=round_index,
    )
    augmented = _append_dagger_rows(train, dagger, records, round_index=round_index)
    validate_direct_dataset(augmented)
    _validate_group_isolation(augmented, validation, test)

    save_direct_dataset(destination / "train.npz", augmented)
    save_direct_dataset(destination / "validation.npz", validation)
    save_direct_dataset(destination / "test.npz", test)
    selection_path = destination / "dagger_selection.jsonl"
    selection_path.write_text(
        "".join(json.dumps(record, ensure_ascii=False) + "\n" for record in records),
        encoding="utf-8",
    )

    reason_counts: dict[str, int] = {}
    for record in records:
        for reason in record["reasons"]:
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
    selected_seeds = sorted(
        set(int(record["irregularity_seed"]) for record in records if record["irregularity_seed"] >= 0)
    )
    validation_groups = set(np.asarray(validation.metadata["group_id"], dtype=str))
    test_groups = set(np.asarray(test.metadata["group_id"], dtype=str))
    manifest = {
        "schema": DAGGER_SCHEMA,
        "round": int(round_index),
        "base_dataset_dir": str(source_dir.resolve()),
        "model_path": str(Path(model_path).resolve()),
        "model_sha256": _sha256(Path(model_path)),
        "policy": (
            "traditional Preload; teacher-anchored Cal-1; learner Cal rollout; "
            "select low-confidence/topology-mismatch/strict-guard rows; teacher reset after guard"
        ),
        "low_confidence_threshold": float(low_confidence),
        "maximum_history_distance": float(maximum_history_distance),
        "geometry_selection_tolerances": {
            "center_m": float(center_tolerance_m),
            "penetration_m": float(penetration_tolerance_m),
            "angle_rad": float(angle_tolerance_rad),
            "shape": float(shape_tolerance),
        },
        "base_train_samples": len(train),
        "dagger_samples": len(dagger),
        "augmented_train_samples": len(augmented),
        "reason_counts": reason_counts,
        "selected_irregularity_seeds": selected_seeds,
        "topology_counts": {
            str(value): int(np.sum(dagger.patch_count == value)) for value in range(3)
        },
        "split_isolation": {
            "train_validation_overlap": sorted(
                set(np.asarray(augmented.metadata["group_id"], dtype=str)) & validation_groups
            ),
            "train_test_overlap": sorted(
                set(np.asarray(augmented.metadata["group_id"], dtype=str)) & test_groups
            ),
        },
        "teacher_labels": (
            "copied from the already replay-verified traditional final-patch labels; "
            "only the history input is learner-induced"
        ),
        "files": {
            "train": "train.npz",
            "validation": "validation.npz",
            "test": "test.npz",
            "selection": selection_path.name,
        },
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def _geometry_errors(prediction: Any | None, teacher: Any) -> dict[str, float]:
    if prediction is None or len(prediction.patches) != len(teacher.patches):
        return {
            "center_error_m": float("inf"),
            "penetration_error_m": float("inf"),
            "angle_error_rad": float("inf"),
            "shape_error": float("inf"),
        }
    if not teacher.patches:
        return {
            "center_error_m": 0.0,
            "penetration_error_m": 0.0,
            "angle_error_rad": 0.0,
            "shape_error": 0.0,
        }
    predicted_patches = sorted(prediction.patches, key=lambda patch: patch.corrected_rail_point[0])
    teacher_patches = sorted(teacher.patches, key=lambda patch: patch.corrected_rail_point[0])
    return {
        "center_error_m": float(max(
            abs(actual.corrected_rail_point[0] - expected.corrected_rail_point[0])
            for actual, expected in zip(predicted_patches, teacher_patches, strict=True)
        )),
        "penetration_error_m": float(max(
            abs(actual.corrected_vertical_penetration - expected.corrected_vertical_penetration)
            for actual, expected in zip(predicted_patches, teacher_patches, strict=True)
        )),
        "angle_error_rad": float(max(
            abs(actual.contact_angle - expected.contact_angle)
            for actual, expected in zip(predicted_patches, teacher_patches, strict=True)
        )),
        "shape_error": float(max(
            np.max(np.abs(np.asarray(actual.shape_moments) - np.asarray(expected.shape_moments)))
            for actual, expected in zip(predicted_patches, teacher_patches, strict=True)
        )),
    }


def _build_dagger_rows(
    base: NetworkA1DirectDataset,
    rows: np.ndarray,
    history: np.ndarray,
    history_mask: np.ndarray,
    records: list[dict[str, Any]],
    *,
    round_index: int,
) -> NetworkA1DirectDataset:
    if not rows.size:
        raise RuntimeError("DAgger audit selected no rows")
    metadata = {key: np.asarray(values[rows]).copy() for key, values in base.metadata.items()}
    metadata["source"] = np.full((rows.size,), "dagger", dtype=str)
    metadata["sample_id"] = np.asarray(
        [f"dagger-r{round_index}:{base.metadata['sample_id'][row]}" for row in rows],
        dtype=str,
    )
    metadata["dagger_reason"] = np.asarray(
        ["+".join(record["reasons"]) for record in records],
        dtype=str,
    )
    metadata["dagger_confidence"] = np.asarray(
        [record["confidence"] for record in records],
        dtype=float,
    )
    metadata["dagger_parent_row"] = rows.astype(np.int64)
    metadata["accepted_step_label"] = np.zeros((rows.size,), dtype=bool)
    metadata["teacher_relabel"] = np.zeros((rows.size,), dtype=bool)
    metadata["topology_reference_available"] = np.ones((rows.size,), dtype=bool)
    metadata["topology_matches_accepted_trajectory"] = np.ones((rows.size,), dtype=bool)
    result = NetworkA1DirectDataset(
        features=base.features[rows].copy(),
        history=history,
        history_mask=history_mask,
        patch_count=base.patch_count[rows].copy(),
        patch_mask=base.patch_mask[rows].copy(),
        targets=base.targets[rows].copy(),
        previous_row=np.full((rows.size,), -1, dtype=np.int64),
        metadata=metadata,
    )
    validate_direct_dataset(result)
    return result


def _append_dagger_rows(
    base: NetworkA1DirectDataset,
    dagger: NetworkA1DirectDataset,
    records: list[dict[str, Any]],
    *,
    round_index: int,
) -> NetworkA1DirectDataset:
    metadata = {
        key: np.concatenate((np.asarray(values), np.asarray(dagger.metadata[key])))
        for key, values in base.metadata.items()
    }
    metadata["dagger_reason"] = np.concatenate(
        (
            np.full((len(base),), "", dtype=str),
            np.asarray(dagger.metadata["dagger_reason"], dtype=str),
        )
    )
    metadata["dagger_confidence"] = np.concatenate(
        (
            np.full((len(base),), np.nan, dtype=float),
            np.asarray(dagger.metadata["dagger_confidence"], dtype=float),
        )
    )
    metadata["dagger_parent_row"] = np.concatenate(
        (
            np.full((len(base),), -1, dtype=np.int64),
            np.asarray(dagger.metadata["dagger_parent_row"], dtype=np.int64),
        )
    )
    return NetworkA1DirectDataset(
        features=np.concatenate((base.features, dagger.features), axis=0),
        history=np.concatenate((base.history, dagger.history), axis=0),
        history_mask=np.concatenate((base.history_mask, dagger.history_mask), axis=0),
        patch_count=np.concatenate((base.patch_count, dagger.patch_count), axis=0),
        patch_mask=np.concatenate((base.patch_mask, dagger.patch_mask), axis=0),
        targets=np.concatenate((base.targets, dagger.targets), axis=0),
        previous_row=np.concatenate((base.previous_row, dagger.previous_row), axis=0),
        metadata=metadata,
    )


def _validate_group_isolation(
    train: NetworkA1DirectDataset,
    validation: NetworkA1DirectDataset,
    test: NetworkA1DirectDataset,
) -> None:
    groups = {
        "train": set(np.asarray(train.metadata["group_id"], dtype=str)),
        "validation": set(np.asarray(validation.metadata["group_id"], dtype=str)),
        "test": set(np.asarray(test.metadata["group_id"], dtype=str)),
    }
    for left, right in (("train", "validation"), ("train", "test"), ("validation", "test")):
        overlap = groups[left] & groups[right]
        if overlap:
            raise ValueError(f"DAgger split group leakage: {left}/{right} {sorted(overlap)[:3]}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect learner-history DAgger rows for WRCP-Net A1 Direct."
    )
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        default=Path("outputs/network_a_direct_set_v2_history_fixed"),
    )
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/network_a_direct_set_v3_dagger"),
    )
    parser.add_argument("--low-confidence", type=float, default=0.95)
    parser.add_argument("--maximum-history-distance", type=float, default=8.0)
    parser.add_argument("--center-tolerance-m", type=float, default=3.0e-4)
    parser.add_argument("--penetration-tolerance-m", type=float, default=8.0e-6)
    parser.add_argument("--angle-tolerance-rad", type=float, default=6.0e-3)
    parser.add_argument("--shape-tolerance", type=float, default=2.0e-2)
    parser.add_argument("--round", type=int, default=1)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    manifest = build_dagger_augmented_dataset(
        dataset_dir=args.dataset_dir,
        model_path=args.model,
        output_dir=args.output_dir,
        low_confidence=args.low_confidence,
        maximum_history_distance=args.maximum_history_distance,
        center_tolerance_m=args.center_tolerance_m,
        penetration_tolerance_m=args.penetration_tolerance_m,
        angle_tolerance_rad=args.angle_tolerance_rad,
        shape_tolerance=args.shape_tolerance,
        round_index=args.round,
    )
    print(f"wrote {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
