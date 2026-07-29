from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from sditt.models.wrcp_net_a1_direct import (
    ACCEPTED_INCREMENT_NAMES,
    HISTORY_NAMES,
    accepted_increment_state_from_history,
    accepted_increment_state_from_targets,
    load_wrcp_net_a1_direct,
    save_wrcp_net_a1_direct,
)
from sditt.training_data.network_a_direct import load_direct_dataset


DEFAULT_FLOORS = np.array(
    [2.0e-5, 1.0e-4, 1.0e-4, 2.0e-6, 2.0e-5, 2.0e-8, 3.0e-3, 3.0e-3, 3.0e-3],
    dtype=float,
)
DEFAULT_CEILINGS = np.array(
    [2.5e-4, 5.0e-4, 5.0e-4, 2.0e-5, 2.5e-4, 2.0e-7, 2.0e-2, 2.0e-2, 2.0e-2],
    dtype=float,
)
INDEPENDENT_HISTORY_NAMES = {
    "patch_start_y_m",
    "patch_end_y_m",
    "corrected_rail_y_m",
    "corrected_vertical_penetration_m",
    "peak_rail_y_m",
    "peak_vertical_penetration_m",
    "shape_moment_1",
    "shape_moment_1p5",
    "shape_moment_2",
}
DERIVED_HISTORY_NAMES = tuple(
    name for name in HISTORY_NAMES if name not in INDEPENDENT_HISTORY_NAMES
)


def calibrate_accepted_step_increment_limits(
    dataset_path: str | Path,
    *,
    source: str = "full_cal_accepted",
    quantile: float = 0.999,
    safety_factor: float = 8.0,
    floors: np.ndarray = DEFAULT_FLOORS,
    ceilings: np.ndarray = DEFAULT_CEILINGS,
) -> tuple[np.ndarray, int]:
    if not 0.0 < quantile <= 1.0:
        raise ValueError("increment quantile must be in (0, 1]")
    if safety_factor <= 0.0:
        raise ValueError("increment safety factor must be positive")
    dataset = load_direct_dataset(dataset_path)
    stage = np.asarray(dataset.metadata.get("stage", np.full((len(dataset),), "")), dtype=str)
    sources = np.asarray(dataset.metadata.get("source", np.full((len(dataset),), "")), dtype=str)
    increments: list[np.ndarray] = []
    for row in range(len(dataset)):
        if stage[row] != "Cal" or (source and sources[row] != source):
            continue
        count = int(dataset.patch_count[row])
        previous_indexes = np.flatnonzero(dataset.history_mask[row])
        if count <= 0 or previous_indexes.size != count:
            continue
        current = accepted_increment_state_from_targets(dataset.targets[row, :count])
        previous = accepted_increment_state_from_history(dataset.history[row, previous_indexes])
        if count == 2:
            direct = abs(current[0, 0] - previous[0, 0]) + abs(current[1, 0] - previous[1, 0])
            swapped = abs(current[0, 0] - previous[1, 0]) + abs(current[1, 0] - previous[0, 0])
            if swapped < direct:
                previous = previous[::-1]
        increments.extend(np.abs(current - previous))
    if not increments:
        raise ValueError(f"no matched accepted Cal increments found for source {source!r}")
    values = np.asarray(increments, dtype=float)
    lower = np.asarray(floors, dtype=float).reshape(len(ACCEPTED_INCREMENT_NAMES))
    upper = np.asarray(ceilings, dtype=float).reshape(len(ACCEPTED_INCREMENT_NAMES))
    if np.any(lower <= 0.0) or np.any(upper < lower):
        raise ValueError("increment floor/ceiling calibration is invalid")
    limits = np.clip(np.quantile(values, quantile, axis=0) * safety_factor, lower, upper)
    return limits, int(values.shape[0])


def write_stabilized_artifact(
    *,
    model_path: str | Path,
    dataset_path: str | Path,
    output_dir: str | Path,
    source: str = "full_cal_accepted",
    quantile: float = 0.999,
    safety_factor: float = 8.0,
    topology_hysteresis_min: float = 0.9,
    derived_rail_z_ood_multiplier: float = 1.0,
    derived_history_ood_multiplier: float = 1.0,
    shape_history_ood_multiplier: float = 1.0,
    limit_shape_increments: bool = True,
    recalibrate_ood_from_dataset: bool = False,
    shape_bounds_quantile: float = 0.001,
    shape_bounds_margin_fraction: float = 0.1,
    shape_moment_history_blend: float = 1.0,
) -> tuple[Path, Path]:
    if not 0.0 <= topology_hysteresis_min <= 1.0:
        raise ValueError("topology hysteresis minimum must be in [0, 1]")
    if derived_rail_z_ood_multiplier < 1.0:
        raise ValueError("derived rail-z OOD multiplier must be at least 1")
    if derived_history_ood_multiplier < 1.0:
        raise ValueError("derived-history OOD multiplier must be at least 1")
    if shape_history_ood_multiplier < 1.0:
        raise ValueError("shape-history OOD multiplier must be at least 1")
    if not 0.0 <= shape_bounds_quantile < 0.5:
        raise ValueError("shape-bounds quantile must be in [0, 0.5)")
    if shape_bounds_margin_fraction < 0.0:
        raise ValueError("shape-bounds margin fraction must be non-negative")
    if not 0.0 <= shape_moment_history_blend <= 1.0:
        raise ValueError("shape-moment history blend must be in [0, 1]")
    limits, matched_increments = calibrate_accepted_step_increment_limits(
        dataset_path,
        source=source,
        quantile=quantile,
        safety_factor=safety_factor,
    )
    if not limit_shape_increments:
        limits[-3:] = 1.0
    model = load_wrcp_net_a1_direct(model_path)
    if not model.has_fixed_profile_projection:
        raise ValueError("stabilized A1 Direct artifact requires fixed profile projection")
    calibration_dataset = load_direct_dataset(dataset_path)
    if recalibrate_ood_from_dataset:
        feature_standardized = (
            calibration_dataset.features - model.feature_mean
        ) / model.feature_scale
        active_history = calibration_dataset.history[
            calibration_dataset.history_mask
        ]
        history_standardized = (
            active_history - model.history_mean
        ) / model.history_scale
        feature_ood_scale = np.maximum(
            np.max(np.abs(feature_standardized), axis=0) / 3.0,
            1.0,
        )
        history_ood_scale = np.maximum(
            np.max(np.abs(history_standardized), axis=0) / 3.0,
            1.0,
        )
    else:
        feature_ood_scale = np.asarray(model.feature_ood_scale, dtype=float).copy()
        history_ood_scale = np.asarray(model.history_ood_scale, dtype=float).copy()
    physical_targets = np.asarray(calibration_dataset.targets, dtype=float)[
        calibration_dataset.patch_mask
    ]
    physical_angles = physical_targets[:, [7, 12]].reshape(-1)
    physical_moments = 1.0 / (1.0 + np.exp(-physical_targets[:, 13:16]))
    shape_lower_quantile = np.quantile(
        physical_moments,
        shape_bounds_quantile,
        axis=0,
    )
    shape_upper_quantile = np.quantile(
        physical_moments,
        1.0 - shape_bounds_quantile,
        axis=0,
    )
    shape_quantile_span = shape_upper_quantile - shape_lower_quantile
    shape_moment_min = np.clip(
        shape_lower_quantile - shape_bounds_margin_fraction * shape_quantile_span,
        1.0e-7,
        1.0 - 2.0e-7,
    )
    shape_moment_max = np.clip(
        shape_upper_quantile + shape_bounds_margin_fraction * shape_quantile_span,
        shape_moment_min + 1.0e-7,
        1.0 - 1.0e-7,
    )
    angle_min_rad = max(-1.45, float(np.quantile(physical_angles, 0.001) - 0.02))
    angle_max_rad = min(1.45, float(np.quantile(physical_angles, 0.999) + 0.02))
    for name in DERIVED_HISTORY_NAMES:
        history_ood_scale[HISTORY_NAMES.index(name)] *= derived_history_ood_multiplier
    for name in ("corrected_rail_z_m", "peak_rail_z_m"):
        history_ood_scale[HISTORY_NAMES.index(name)] *= derived_rail_z_ood_multiplier
    for name in ("shape_moment_1", "shape_moment_1p5", "shape_moment_2"):
        history_ood_scale[HISTORY_NAMES.index(name)] *= shape_history_ood_multiplier
    stabilized = replace(
        model,
        accepted_step_increment_limits=limits,
        topology_hysteresis_min=float(topology_hysteresis_min),
        feature_ood_scale=feature_ood_scale,
        history_ood_scale=history_ood_scale,
        angle_min_rad=angle_min_rad,
        angle_max_rad=angle_max_rad,
        shape_moment_min=shape_moment_min,
        shape_moment_max=shape_moment_max,
        shape_moment_history_blend=float(shape_moment_history_blend),
    )
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    output_model = destination / "model.npz"
    save_wrcp_net_a1_direct(output_model, stabilized)
    manifest = {
        "schema": "wrcp-net-a1-direct-accepted-step-stabilizer-v1",
        "base_model": str(Path(model_path).resolve()),
        "calibration_dataset": str(Path(dataset_path).resolve()),
        "source": source,
        "quantile": float(quantile),
        "safety_factor": float(safety_factor),
        "matched_patch_increments": matched_increments,
        "increment_names": list(ACCEPTED_INCREMENT_NAMES),
        "increment_limits": limits.tolist(),
        "increment_floors": DEFAULT_FLOORS.tolist(),
        "increment_ceilings": DEFAULT_CEILINGS.tolist(),
        "shape_increment_limits_enabled": bool(limit_shape_increments),
        "ood_recalibrated_from_dataset": bool(recalibrate_ood_from_dataset),
        "ood_training_support_target_distance": 3.0,
        "topology_hysteresis_min": float(topology_hysteresis_min),
        "physical_angle_min_rad": angle_min_rad,
        "physical_angle_max_rad": angle_max_rad,
        "physical_angle_calibration": "0.1/99.9 percentiles plus 0.02 rad margin",
        "shape_bounds_quantile": float(shape_bounds_quantile),
        "shape_bounds_margin_fraction": float(shape_bounds_margin_fraction),
        "shape_moment_lower_quantile": shape_lower_quantile.tolist(),
        "shape_moment_upper_quantile": shape_upper_quantile.tolist(),
        "shape_moment_min": shape_moment_min.tolist(),
        "shape_moment_max": shape_moment_max.tolist(),
        "shape_bounds_calibration": "training dataset only",
        "shape_moment_history_blend": float(shape_moment_history_blend),
        "derived_rail_z_ood_multiplier": float(derived_rail_z_ood_multiplier),
        "derived_rail_z_ood_names": ["corrected_rail_z_m", "peak_rail_z_m"],
        "derived_history_ood_multiplier": float(derived_history_ood_multiplier),
        "derived_history_ood_names": list(DERIVED_HISTORY_NAMES),
        "strict_independent_history_names": sorted(INDEPENDENT_HISTORY_NAMES),
        "shape_history_ood_multiplier": float(shape_history_ood_multiplier),
        "shape_history_ood_names": ["shape_moment_1", "shape_moment_1p5", "shape_moment_2"],
        "ood_threshold": float(stabilized.ood_threshold),
        "topology_confidence_min": float(stabilized.topology_confidence_min),
        "decision": (
            "global OOD threshold and independent-history envelopes are unchanged; configured "
            "multipliers apply only to deterministic derived history fields; increment limits "
            "apply only when accepted patch counts match; fixed profiles recompute dependent geometry"
        ),
    }
    manifest_path = destination / "stabilization_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return output_model, manifest_path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Add accepted-step stabilization to A1 Direct.")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("outputs/wrcp_net_a1_direct_full_cal_candidate/model.npz"),
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path("outputs/network_a_direct_set_v11_full_cal_seed20260716/train.npz"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/wrcp_net_a1_direct_full_cal_stabilized"),
    )
    parser.add_argument("--source", default="full_cal_accepted")
    parser.add_argument("--quantile", type=float, default=0.999)
    parser.add_argument("--safety-factor", type=float, default=8.0)
    parser.add_argument("--topology-hysteresis-min", type=float, default=0.9)
    parser.add_argument("--derived-rail-z-ood-multiplier", type=float, default=1.0)
    parser.add_argument("--derived-history-ood-multiplier", type=float, default=1.0)
    parser.add_argument("--shape-history-ood-multiplier", type=float, default=1.0)
    parser.add_argument(
        "--no-limit-shape-increments",
        action="store_true",
        help="retain physical moment bounds but do not clamp their accepted-step increments",
    )
    parser.add_argument(
        "--recalibrate-ood-from-dataset",
        action="store_true",
        help="recompute per-field envelopes from this training dataset at distance 3.0",
    )
    parser.add_argument("--shape-bounds-quantile", type=float, default=0.001)
    parser.add_argument("--shape-bounds-margin-fraction", type=float, default=0.1)
    parser.add_argument("--shape-moment-history-blend", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    model_path, manifest_path = write_stabilized_artifact(
        model_path=args.model,
        dataset_path=args.dataset,
        output_dir=args.output_dir,
        source=args.source,
        quantile=args.quantile,
        safety_factor=args.safety_factor,
        topology_hysteresis_min=args.topology_hysteresis_min,
        derived_rail_z_ood_multiplier=args.derived_rail_z_ood_multiplier,
        derived_history_ood_multiplier=args.derived_history_ood_multiplier,
        shape_history_ood_multiplier=args.shape_history_ood_multiplier,
        limit_shape_increments=not args.no_limit_shape_increments,
        recalibrate_ood_from_dataset=args.recalibrate_ood_from_dataset,
        shape_bounds_quantile=args.shape_bounds_quantile,
        shape_bounds_margin_fraction=args.shape_bounds_margin_fraction,
        shape_moment_history_blend=args.shape_moment_history_blend,
    )
    print(f"wrote {model_path}")
    print(f"wrote {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
