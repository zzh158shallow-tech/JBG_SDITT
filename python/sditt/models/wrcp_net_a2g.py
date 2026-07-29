from __future__ import annotations

import argparse
import csv
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from sditt.contact.geometry import (
    BoundaryExtrema,
    ContactPatch,
    MultiPointContactGeometry,
    WheelPose2D,
    extreme_boundary,
    prepare_contact_profile_geometry,
    quasi_elastic_correction,
)
from sditt.models.wrcp_net_a1 import (
    CLASS_NAMES,
    _Adam,
    _classification_metrics,
    _sha256_file,
    _softmax,
    WRCPNetA1,
)
from sditt.training_data.network_a import (
    FEATURE_NAMES,
    LABEL_NAMES,
    MAX_PATCHES,
    NetworkADataset,
    build_network_a_teacher_context,
    geometry_to_network_a_labels,
    load_network_a_dataset,
    teacher_geometry_from_features,
)


MODEL_NAME = "WRCP-Net A2G"
MODEL_ID = "wrcp-net-a2g"
GAP_GRID_SIZE = 257
GAP_CACHE_VERSION = "network-a-canonical-gap-field-v1"
RECONSTRUCTION_METHOD = "predicted_gap_topology_with_fixed_profile_consistency_refinement"


@dataclass(frozen=True)
class GapFieldCache:
    canonical_y_m: np.ndarray
    fields_by_split: dict[str, np.ndarray]


@dataclass(frozen=True)
class WRCPNetA2GPrediction:
    patch_count: np.ndarray
    class_probability: np.ndarray
    gap_field_m: np.ndarray


@dataclass(frozen=True)
class WRCPNetA2GTrainingResult:
    output_dir: Path
    model_path: Path
    metrics_path: Path
    best_epoch: int
    metrics: dict[str, Any]


class WRCPNetA2G(WRCPNetA1):
    """Geometry-aware MLP that predicts a canonical wheel-rail gap field."""

    def __init__(
        self,
        *,
        hidden_sizes: Sequence[int] = (128, 128),
        grid_size: int = GAP_GRID_SIZE,
        seed: int = 20260720,
    ) -> None:
        super().__init__(
            input_size=len(FEATURE_NAMES),
            hidden_sizes=hidden_sizes,
            output_size=len(CLASS_NAMES) + int(grid_size),
            seed=seed,
        )
        self.grid_size = int(grid_size)


def _canonical_grid(context: Any, *, grid_size: int) -> np.ndarray:
    ranges = []
    for side in ("L", "R"):
        rail_y = np.abs(np.asarray(context.track_profiles.profile[side], dtype=float)[:, 0])
        ranges.append((float(np.min(rail_y)), float(np.max(rail_y))))
    lower = max(value[0] for value in ranges) + 1.25e-4
    upper = min(value[1] for value in ranges) - 1.25e-4
    return np.linspace(lower, upper, int(grid_size), dtype=np.float64)


def _teacher_gap_on_grid(
    context: Any,
    features: np.ndarray,
    canonical_y_m: np.ndarray,
) -> np.ndarray:
    geometry = teacher_geometry_from_features(context, np.asarray(features, dtype=float))
    physical_y = np.abs(geometry.elastic_penetration[:, 0])
    order = np.argsort(physical_y)
    return np.interp(
        canonical_y_m,
        physical_y[order],
        geometry.elastic_penetration[order, 1],
    )


def build_gap_field_cache(
    *,
    dataset_dir: str | Path,
    output_dir: str | Path,
    repo_root: str | Path | None = None,
    grid_size: int = GAP_GRID_SIZE,
    force: bool = False,
) -> GapFieldCache:
    dataset_root = Path(dataset_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    manifest_path = destination / "manifest.json"
    split_paths = {name: destination / f"{name}.npz" for name in ("train", "validation", "test")}
    dataset_hash = _sha256_file(dataset_root / "manifest.json")
    if not force and manifest_path.exists() and all(path.exists() for path in split_paths.values()):
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if (
            manifest.get("cache_version") == GAP_CACHE_VERSION
            and manifest.get("dataset_manifest_sha256") == dataset_hash
            and int(manifest.get("grid_size", -1)) == int(grid_size)
        ):
            fields: dict[str, np.ndarray] = {}
            grid: np.ndarray | None = None
            for split, path in split_paths.items():
                with np.load(path, allow_pickle=False) as archive:
                    fields[split] = archive["gap_field_m"].copy()
                    if grid is None:
                        grid = archive["canonical_y_m"].copy()
            assert grid is not None
            return GapFieldCache(grid, fields)

    context = build_network_a_teacher_context(repo_root)
    grid = _canonical_grid(context, grid_size=grid_size)
    fields = {}
    for split, path in split_paths.items():
        dataset = load_network_a_dataset(dataset_root, split=split, dtype=np.float64)
        values = np.empty((len(dataset), grid.size), dtype=np.float32)
        for index, features in enumerate(dataset.features):
            values[index] = _teacher_gap_on_grid(context, features, grid)
        np.savez_compressed(
            path,
            canonical_y_m=grid,
            gap_field_m=values,
            sample_id=dataset.metadata["sample_id"],
        )
        fields[split] = values
        print(f"gap_cache split={split} samples={len(dataset)}", flush=True)
    manifest = {
        "cache_version": GAP_CACHE_VERSION,
        "dataset_manifest_sha256": dataset_hash,
        "grid_size": int(grid.size),
        "canonical_y_min_m": float(grid[0]),
        "canonical_y_max_m": float(grid[-1]),
        "orientation": "absolute rail lateral coordinate; left and right profiles mirror onto one ascending grid",
        "splits": {name: int(value.shape[0]) for name, value in fields.items()},
    }
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return GapFieldCache(grid, fields)


def _canonical_features(features: np.ndarray) -> np.ndarray:
    values = np.asarray(features, dtype=np.float32).copy()
    sign = np.where(values[:, 0] < 0.5, -1.0, 1.0).astype(np.float32)
    values[:, 1] *= sign
    values[:, 3] *= sign
    return values


def _field_loss_and_gradient(
    output: np.ndarray,
    *,
    patch_count: np.ndarray,
    target_normalized: np.ndarray,
    target_physical: np.ndarray,
    class_weights: np.ndarray,
    near_surface_scale_m: float,
    positive_weight: float,
    double_field_weight: float,
    field_weight: float,
    derivative_weight: float,
    huber_delta: float,
) -> tuple[float, float, float, float, np.ndarray]:
    batch_size = output.shape[0]
    logits = output[:, : len(CLASS_NAMES)]
    predicted = output[:, len(CLASS_NAMES) :]
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

    weights = 1.0 + 8.0 * np.exp(-np.abs(target_physical) / float(near_surface_scale_m))
    weights += float(positive_weight) * (target_physical > 0.0)
    weights *= np.where(classes[:, None] == 2, float(double_field_weight), 1.0)
    difference = predicted - target_normalized
    absolute = np.abs(difference)
    point_loss = np.where(
        absolute <= huber_delta,
        0.5 * np.square(difference),
        huber_delta * (absolute - 0.5 * huber_delta),
    )
    point_gradient = np.where(
        absolute <= huber_delta,
        difference,
        huber_delta * np.sign(difference),
    )
    point_denominator = max(float(np.sum(weights)), 1.0)
    field_loss = float(np.sum(weights * point_loss) / point_denominator)
    field_gradient = float(field_weight) * weights * point_gradient / point_denominator

    predicted_derivative = np.diff(predicted, axis=1)
    target_derivative = np.diff(target_normalized, axis=1)
    derivative_difference = predicted_derivative - target_derivative
    derivative_absolute = np.abs(derivative_difference)
    derivative_loss_values = np.where(
        derivative_absolute <= huber_delta,
        0.5 * np.square(derivative_difference),
        huber_delta * (derivative_absolute - 0.5 * huber_delta),
    )
    derivative_gradient_values = np.where(
        derivative_absolute <= huber_delta,
        derivative_difference,
        huber_delta * np.sign(derivative_difference),
    )
    derivative_loss = float(np.mean(derivative_loss_values))
    derivative_gradient_values *= float(derivative_weight) / max(derivative_difference.size, 1)
    field_gradient[:, :-1] -= derivative_gradient_values
    field_gradient[:, 1:] += derivative_gradient_values

    gradient = np.zeros_like(output, dtype=np.float32)
    gradient[:, : len(CLASS_NAMES)] = class_gradient
    gradient[:, len(CLASS_NAMES) :] = field_gradient
    total = class_loss + float(field_weight) * field_loss + float(derivative_weight) * derivative_loss
    return total, class_loss, field_loss, derivative_loss, gradient


def _normalized_gap_fields(
    train_field: np.ndarray,
    fields_by_split: Mapping[str, np.ndarray],
) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray]:
    mean = np.mean(train_field, axis=0).astype(np.float32)
    std = np.std(train_field, axis=0).astype(np.float32)
    std = np.maximum(std, 1.0e-6)
    normalized = {
        name: ((np.asarray(values, dtype=np.float32) - mean[None, :]) / std[None, :]).astype(
            np.float32, copy=False
        )
        for name, values in fields_by_split.items()
    }
    return normalized, mean, std


def _evaluate_loss(
    model: WRCPNetA2G,
    features: np.ndarray,
    dataset: NetworkADataset,
    target_normalized: np.ndarray,
    target_physical: np.ndarray,
    *,
    class_weights: np.ndarray,
    near_surface_scale_m: float,
    positive_weight: float,
    double_field_weight: float,
    field_weight: float,
    derivative_weight: float,
    huber_delta: float,
    batch_size: int,
) -> tuple[float, float, float, float]:
    total = np.zeros((4,), dtype=float)
    observations = 0
    for start in range(0, len(dataset), batch_size):
        stop = min(start + batch_size, len(dataset))
        output = np.asarray(model.forward(features[start:stop]), dtype=np.float32)
        losses = _field_loss_and_gradient(
            output,
            patch_count=dataset.patch_count[start:stop],
            target_normalized=target_normalized[start:stop],
            target_physical=target_physical[start:stop],
            class_weights=class_weights,
            near_surface_scale_m=near_surface_scale_m,
            positive_weight=positive_weight,
            double_field_weight=double_field_weight,
            field_weight=field_weight,
            derivative_weight=derivative_weight,
            huber_delta=huber_delta,
        )[:4]
        count = stop - start
        total += np.asarray(losses) * count
        observations += count
    return tuple((total / max(observations, 1)).tolist())  # type: ignore[return-value]


def _predict(
    model: WRCPNetA2G,
    features: np.ndarray,
    *,
    feature_mean: np.ndarray,
    feature_std: np.ndarray,
    gap_mean: np.ndarray,
    gap_std: np.ndarray,
) -> WRCPNetA2GPrediction:
    canonical = _canonical_features(np.asarray(features, dtype=np.float32))
    normalized = (canonical - feature_mean[None, :]) / feature_std[None, :]
    output = np.asarray(model.forward(normalized), dtype=np.float32)
    probability = _softmax(output[:, : len(CLASS_NAMES)])
    patch_count = np.argmax(probability, axis=1).astype(np.int8)
    gap = output[:, len(CLASS_NAMES) :] * gap_std[None, :] + gap_mean[None, :]
    return WRCPNetA2GPrediction(patch_count, probability.astype(np.float32), gap.astype(np.float32))


def _geometry_from_predicted_field(
    context: Any,
    features: np.ndarray,
    canonical_y_m: np.ndarray,
    gap_field_m: np.ndarray,
    desired_patch_count: int,
    *,
    profile_refinement: bool = True,
    candidate_profile_refinement: bool = False,
    anchor_patches: Sequence[ContactPatch] = (),
    candidate_margin_m: float = 2.0e-3,
) -> MultiPointContactGeometry:
    values = np.asarray(features, dtype=float)
    side = "L" if float(values[0]) < 0.5 else "R"
    wheel_profile = context.wheel_profiles.left if side == "L" else context.wheel_profiles.right
    angle_table = (
        context.wheel_profiles.contact_angle_left
        if side == "L"
        else context.wheel_profiles.contact_angle_right
    )
    structure = prepare_contact_profile_geometry(
        wheel_profile,
        angle_table,
        np.asarray(context.track_profiles.profile[side], dtype=float),
        pose=WheelPose2D(
            lateral=float(values[1]),
            vertical=float(values[2]),
            roll=float(values[3]),
            yaw=float(values[4]),
        ),
        min_overlap_margin=1.0e-4,
        dlb=context.dlb,
        prepared_rail=context.prepared_rail_interpolators[side],
    )
    native_y = structure.wheel_interp[:, 1]
    predicted = np.interp(np.abs(native_y), canonical_y_m, np.asarray(gap_field_m, dtype=float))
    if desired_patch_count <= 0:
        predicted = predicted - max(float(np.max(predicted)), 0.0) - 1.0e-9
    elastic = np.column_stack((native_y, predicted))
    boundaries = _positive_boundaries_fast(elastic)
    if desired_patch_count > 0 and boundaries.positive_extrema.shape[0] != desired_patch_count:
        # Match topology on the native profile grid. A threshold selected on
        # the compact learned grid can change component count after
        # interpolation, especially under irregularity-driven edge states.
        predicted_values = np.asarray(predicted, dtype=float)
        best_threshold = _topology_matching_threshold(
            predicted_values,
            desired_patch_count,
        )
        if best_threshold is not None:
            elastic = np.column_stack((native_y, predicted - best_threshold))
            boundaries = _positive_boundaries_fast(elastic)
    if (
        candidate_profile_refinement
        and boundaries.positive_extrema.shape[0] == int(desired_patch_count)
    ):
        candidate_geometry = _candidate_profile_refinement(
            structure=structure,
            features=values,
            angle_table=angle_table,
            network_boundaries=boundaries,
            anchor_patches=tuple(anchor_patches),
            candidate_margin_m=float(candidate_margin_m),
        )
        if candidate_geometry is not None:
            return candidate_geometry

    patches = quasi_elastic_correction(
        elastic,
        boundaries.positive_extrema,
        boundaries.starts,
        boundaries.ends,
        structure.wheel_interp,
        structure.rail_interp,
        structure.contact_angles,
        structure.wheel_profile_lateral,
        contact_angle_table=angle_table,
        yaw=float(features[4]),
        lateral=float(features[1]),
        roll=float(features[3]),
        assume_sorted=True,
    )

    # The learned field supplies the contact class/topology. Once that topology
    # agrees with the fixed profiles, reconstruct coordinates on the native
    # profile grid to remove compact-grid quantisation and field-shape error.
    # The profile stage is not allowed to override the network's predicted
    # patch count, so classification errors remain visible in end-to-end QA.
    if profile_refinement:
        profile_elastic = np.column_stack(
            (
                native_y,
                structure.wheel_interp[:, 2]
                - structure.rail_interp[:, 1]
                + float(values[2])
                + float(values[5]),
            )
        )
        profile_boundaries = extreme_boundary(profile_elastic, opt="max")
        profile_patches = quasi_elastic_correction(
            profile_elastic,
            profile_boundaries.positive_extrema,
            profile_boundaries.starts,
            profile_boundaries.ends,
            structure.wheel_interp,
            structure.rail_interp,
            structure.contact_angles,
            structure.wheel_profile_lateral,
            contact_angle_table=angle_table,
            yaw=float(features[4]),
            lateral=float(features[1]),
            roll=float(features[3]),
        )
        if len(profile_patches) == int(desired_patch_count):
            return MultiPointContactGeometry(
                has_contact=bool(profile_patches),
                elastic_penetration=profile_elastic,
                wheel_interp=structure.wheel_interp,
                rail_interp=structure.rail_interp,
                contact_angles=structure.contact_angles,
                wheel_profile_lateral=structure.wheel_profile_lateral,
                boundaries=profile_boundaries,
                patches=profile_patches,
            )
    if len(patches) > MAX_PATCHES:
        patches = tuple(
            sorted(patches, key=lambda patch: patch.peak_vertical_penetration, reverse=True)[:MAX_PATCHES]
        )
        patches = tuple(sorted(patches, key=lambda patch: patch.corrected_rail_point[0]))
    return MultiPointContactGeometry(
        has_contact=bool(patches),
        elastic_penetration=elastic,
        wheel_interp=structure.wheel_interp,
        rail_interp=structure.rail_interp,
        contact_angles=structure.contact_angles,
        wheel_profile_lateral=structure.wheel_profile_lateral,
        boundaries=boundaries,
        patches=patches,
    )


def _topology_matching_threshold(
    values: np.ndarray,
    desired_component_count: int,
) -> float | None:
    """Return the old closest-to-zero topology threshold without O(n^2) scans.

    The previous implementation tested every midpoint between unique field
    values and rebuilt a full Boolean vector for each threshold.  A descending
    one-dimensional rank/difference sweep computes the same component counts
    in O(n log n) time while retaining the original threshold ordering and tie
    behaviour.
    """

    field = np.asarray(values, dtype=float).reshape(-1)
    if field.size == 0:
        return None
    sorted_values, value_ranks = np.unique(field, return_inverse=True)
    midpoint_counts = _superlevel_midpoint_component_counts(value_ranks, sorted_values.size)
    thresholds = np.concatenate(
        (
            np.array([0.0]),
            sorted_values[:1] - 1.0e-12,
            0.5 * (sorted_values[:-1] + sorted_values[1:]),
            sorted_values[-1:] + 1.0e-12,
        )
    )
    counts = np.concatenate(
        (
            np.array([_positive_component_count(field > 0.0)], dtype=int),
            np.array(
                [_positive_component_count(field > float(thresholds[1]))],
                dtype=int,
            ),
            midpoint_counts,
            np.array(
                [_positive_component_count(field > float(thresholds[-1]))],
                dtype=int,
            ),
        )
    )
    matching = np.flatnonzero(counts == int(desired_component_count))
    if matching.size == 0:
        return None
    candidate_thresholds = thresholds[matching]
    return float(candidate_thresholds[int(np.argmin(np.abs(candidate_thresholds)))])


def _positive_boundaries_fast(elastic_penetration: np.ndarray) -> BoundaryExtrema:
    """Build only the positive components needed by Network A reconstruction."""

    data = np.asarray(elastic_penetration, dtype=float)
    if data.ndim != 2 or data.shape[1] != 2 or data.shape[0] == 0:
        empty = np.empty((0, 4), dtype=float)
        return BoundaryExtrema(empty, empty, empty, empty)
    positive = data[:, 1] >= 0.0
    starts = np.flatnonzero(positive & np.concatenate(([True], ~positive[:-1])))
    ends = np.flatnonzero(positive & np.concatenate((~positive[1:], [True])))
    count = min(starts.size, ends.size)
    starts = starts[:count]
    ends = ends[:count]
    peaks = np.asarray(
        [start + int(np.argmax(data[start : end + 1, 1])) for start, end in zip(starts, ends, strict=True)],
        dtype=int,
    )

    def rows(indexes: np.ndarray) -> np.ndarray:
        if indexes.size == 0:
            return np.empty((0, 4), dtype=float)
        return np.column_stack((indexes, data[indexes, 0], data[indexes, 1], np.zeros(indexes.size)))

    peak_rows = rows(peaks)
    return BoundaryExtrema(
        extrema=peak_rows.copy(),
        positive_extrema=peak_rows,
        starts=rows(starts),
        ends=rows(ends),
    )


def _superlevel_midpoint_component_counts(
    value_ranks: np.ndarray,
    unique_value_count: int,
) -> np.ndarray:
    """Count 1-D components above each ascending unique-value midpoint."""

    ranks = np.asarray(value_ranks, dtype=int).reshape(-1)
    count = max(int(unique_value_count) - 1, 0)
    result = np.zeros((count,), dtype=int)
    if count == 0 or ranks.size == 0:
        return result
    # For midpoint interval j, a rising edge contributes one component exactly
    # when rank(previous) <= j < rank(current).  Accumulating these intervals
    # with a difference array is equivalent to a union-find sweep but remains
    # inside vectorised NumPy operations.
    difference = np.zeros((int(unique_value_count),), dtype=int)
    first_rank = int(ranks[0])
    if first_rank > 0:
        difference[0] += 1
        difference[first_rank] -= 1
    rising = ranks[1:] > ranks[:-1]
    if np.any(rising):
        starts = ranks[:-1][rising]
        ends = ranks[1:][rising]
        difference += np.bincount(starts, minlength=int(unique_value_count))
        difference -= np.bincount(ends, minlength=int(unique_value_count))
    return np.cumsum(difference)[:count]


def _positive_component_count(positive: np.ndarray) -> int:
    mask = np.asarray(positive, dtype=bool).reshape(-1)
    if mask.size == 0:
        return 0
    return int(mask[0]) + int(np.count_nonzero(mask[1:] & ~mask[:-1]))


def _candidate_profile_refinement(
    *,
    structure: Any,
    features: np.ndarray,
    angle_table: np.ndarray,
    network_boundaries: BoundaryExtrema,
    anchor_patches: tuple[ContactPatch, ...],
    candidate_margin_m: float,
) -> MultiPointContactGeometry | None:
    """Refine only inside neural candidate regions on the fixed native profiles."""

    native_y = np.asarray(structure.wheel_interp[:, 1], dtype=float)
    if native_y.size < 3 or network_boundaries.positive_extrema.size == 0:
        return None
    physical = (
        np.asarray(structure.wheel_interp[:, 2], dtype=float)
        - np.asarray(structure.rail_interp[:, 1], dtype=float)
        + float(features[2])
        + float(features[5])
    )
    peak_rows: list[np.ndarray] = []
    start_rows: list[np.ndarray] = []
    end_rows: list[np.ndarray] = []
    used_peaks: set[int] = set()
    anchors = list(anchor_patches)
    for peak_row, start_row, end_row in zip(
        network_boundaries.positive_extrema,
        network_boundaries.starts,
        network_boundaries.ends,
        strict=True,
    ):
        start_index = int(np.clip(start_row[0], 0, native_y.size - 1))
        end_index = int(np.clip(end_row[0], 0, native_y.size - 1))
        candidate_y = float(peak_row[1])
        lo = min(float(native_y[start_index]), float(native_y[end_index])) - candidate_margin_m
        hi = max(float(native_y[start_index]), float(native_y[end_index])) + candidate_margin_m
        if anchors:
            nearest = min(
                anchors,
                key=lambda old: abs(float(old.corrected_rail_point[0]) - candidate_y),
            )
            anchor_y = float(nearest.corrected_rail_point[0])
            lo = min(lo, anchor_y - candidate_margin_m)
            hi = max(hi, anchor_y + candidate_margin_m)
        indexes = np.flatnonzero((native_y >= lo) & (native_y <= hi))
        if indexes.size == 0:
            return None
        ranked = indexes[np.argsort(physical[indexes])[::-1]]
        peak = next((int(index) for index in ranked if int(index) not in used_peaks), None)
        if peak is None or physical[peak] <= 0.0:
            return None
        used_peaks.add(peak)
        positive = physical > 0.0
        start = peak
        end = peak
        # The neural field selects the candidate peak. Expand only the connected
        # physical component that contains that peak so quasi-elastic centering
        # is not biased by an artificially truncated candidate window.
        while start > 0 and positive[start - 1]:
            start -= 1
        while end < native_y.size - 1 and positive[end + 1]:
            end += 1

        def row(index: int) -> np.ndarray:
            return np.array(
                [float(index), native_y[index], physical[index], 0.0],
                dtype=float,
            )

        peak_rows.append(row(peak))
        start_rows.append(row(start))
        end_rows.append(row(end))
    order = np.argsort([row[1] for row in peak_rows])
    peaks = np.stack([peak_rows[index] for index in order])
    starts = np.stack([start_rows[index] for index in order])
    ends = np.stack([end_rows[index] for index in order])
    physical_elastic = np.column_stack((native_y, physical))
    refined = quasi_elastic_correction(
        physical_elastic,
        peaks,
        starts,
        ends,
        structure.wheel_interp,
        structure.rail_interp,
        structure.contact_angles,
        structure.wheel_profile_lateral,
        contact_angle_table=angle_table,
        yaw=float(features[4]),
        lateral=float(features[1]),
        roll=float(features[3]),
        assume_sorted=True,
    )
    if len(refined) != network_boundaries.positive_extrema.shape[0]:
        return None
    boundaries = BoundaryExtrema(
        extrema=peaks.copy(),
        positive_extrema=peaks,
        starts=starts,
        ends=ends,
    )
    return MultiPointContactGeometry(
        has_contact=bool(refined),
        elastic_penetration=physical_elastic,
        wheel_interp=structure.wheel_interp,
        rail_interp=structure.rail_interp,
        contact_angles=structure.contact_angles,
        wheel_profile_lateral=structure.wheel_profile_lateral,
        boundaries=boundaries,
        patches=refined,
    )


def _geometry_metrics(
    context: Any,
    dataset: NetworkADataset,
    prediction: WRCPNetA2GPrediction,
    canonical_y_m: np.ndarray,
) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    predicted_count = prediction.patch_count.copy()
    predicted_labels = np.zeros_like(dataset.labels, dtype=np.float32)
    reconstructed_count = np.zeros_like(dataset.patch_count)
    matched_sample = np.zeros((len(dataset),), dtype=bool)
    absolute_errors: list[np.ndarray] = []
    contact_samples = dataset.patch_count > 0
    within_1mm = np.zeros((len(dataset),), dtype=bool)
    for index in range(len(dataset)):
        geometry = _geometry_from_predicted_field(
            context,
            dataset.features[index],
            canonical_y_m,
            prediction.gap_field_m[index],
            int(predicted_count[index]),
        )
        count, mask, labels = geometry_to_network_a_labels(geometry)
        reconstructed_count[index] = min(count, MAX_PATCHES)
        predicted_labels[index] = labels
        if count != int(dataset.patch_count[index]) or int(predicted_count[index]) != count:
            continue
        matched_sample[index] = True
        if count:
            difference = np.abs(labels[mask] - dataset.labels[index, mask])
            absolute_errors.append(difference)
            rail_y_index = LABEL_NAMES.index("corrected_rail_y_m")
            within_1mm[index] = bool(np.all(difference[:, rail_y_index] <= 1.0e-3))
    errors = np.concatenate(absolute_errors, axis=0) if absolute_errors else np.empty((0, len(LABEL_NAMES)))
    by_label = {
        name: {
            "mae": float(np.mean(errors[:, label_index])) if errors.size else None,
            "p95_absolute_error": float(np.quantile(errors[:, label_index], 0.95)) if errors.size else None,
            "max_absolute_error": float(np.max(errors[:, label_index])) if errors.size else None,
        }
        for label_index, name in enumerate(LABEL_NAMES)
    }
    metrics = {
        "classification": _classification_metrics(dataset.patch_count, predicted_count),
        "reconstructed_count_matches_teacher_rate": float(
            np.mean(reconstructed_count == dataset.patch_count)
        ),
        "end_to_end_count_matches_teacher_rate": float(np.mean(matched_sample)),
        "matched_contact_samples": int(np.count_nonzero(matched_sample & contact_samples)),
        "contact_samples": int(np.count_nonzero(contact_samples)),
        "contact_position_within_1mm_over_all_contact_samples": float(
            np.count_nonzero(within_1mm) / max(np.count_nonzero(contact_samples), 1)
        ),
        "conditional_on_correct_count": {"by_label": by_label},
    }
    return metrics, reconstructed_count, predicted_labels


def _save_model(
    path: Path,
    model: WRCPNetA2G,
    *,
    canonical_y_m: np.ndarray,
    feature_mean: np.ndarray,
    feature_std: np.ndarray,
    gap_mean: np.ndarray,
    gap_std: np.ndarray,
) -> None:
    payload: dict[str, np.ndarray] = {
        "layer_sizes": np.asarray(model.layer_sizes, dtype=np.int64),
        "canonical_y_m": np.asarray(canonical_y_m, dtype=np.float64),
        "feature_mean": np.asarray(feature_mean, dtype=np.float32),
        "feature_std": np.asarray(feature_std, dtype=np.float32),
        "gap_mean": np.asarray(gap_mean, dtype=np.float32),
        "gap_std": np.asarray(gap_std, dtype=np.float32),
        "feature_names": np.asarray(FEATURE_NAMES),
        "class_names": np.asarray(CLASS_NAMES),
    }
    for index, (weight, bias) in enumerate(zip(model.weights, model.biases, strict=True)):
        payload[f"weight_{index}"] = weight
        payload[f"bias_{index}"] = bias
    np.savez_compressed(path, **payload)


def load_wrcp_net_a2g(path: str | Path) -> tuple[WRCPNetA2G, dict[str, np.ndarray]]:
    with np.load(Path(path), allow_pickle=False) as archive:
        sizes = tuple(int(value) for value in archive["layer_sizes"])
        grid = archive["canonical_y_m"].copy()
        model = WRCPNetA2G(hidden_sizes=sizes[1:-1], grid_size=grid.size)
        for index in range(len(model.weights)):
            model.weights[index][...] = archive[f"weight_{index}"]
            model.biases[index][...] = archive[f"bias_{index}"]
        metadata = {
            "canonical_y_m": grid,
            "feature_mean": archive["feature_mean"].copy(),
            "feature_std": archive["feature_std"].copy(),
            "gap_mean": archive["gap_mean"].copy(),
            "gap_std": archive["gap_std"].copy(),
        }
    return model, metadata


def predict_wrcp_net_a2g(
    model: WRCPNetA2G,
    features: np.ndarray,
    metadata: Mapping[str, np.ndarray],
) -> WRCPNetA2GPrediction:
    values = np.asarray(features, dtype=np.float32)
    if values.ndim == 1:
        values = values[None, :]
    return _predict(
        model,
        values,
        feature_mean=metadata["feature_mean"],
        feature_std=metadata["feature_std"],
        gap_mean=metadata["gap_mean"],
        gap_std=metadata["gap_std"],
    )


def train_wrcp_net_a2g(
    *,
    dataset_dir: str | Path,
    cache_dir: str | Path,
    output_dir: str | Path,
    repo_root: str | Path | None = None,
    epochs: int = 500,
    batch_size: int = 256,
    learning_rate: float = 5.0e-4,
    hidden_sizes: Sequence[int] = (128, 128),
    field_weight: float = 2.0,
    derivative_weight: float = 0.2,
    positive_weight: float = 12.0,
    double_field_weight: float = 1.0,
    near_surface_scale_m: float = 5.0e-4,
    weight_decay: float = 1.0e-5,
    huber_delta: float = 1.0,
    patience: int = 60,
    seed: int = 20260720,
) -> WRCPNetA2GTrainingResult:
    started = time.perf_counter()
    dataset_root = Path(dataset_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    cache = build_gap_field_cache(
        dataset_dir=dataset_root,
        output_dir=cache_dir,
        repo_root=repo_root,
    )
    datasets = {
        name: load_network_a_dataset(dataset_root, split=name, dtype=np.float32)
        for name in ("train", "validation", "test")
    }
    canonical_features = {name: _canonical_features(value.features) for name, value in datasets.items()}
    feature_mean = np.mean(canonical_features["train"], axis=0).astype(np.float32)
    feature_std = np.maximum(np.std(canonical_features["train"], axis=0), 1.0e-6).astype(np.float32)
    normalized_features = {
        name: ((value - feature_mean[None, :]) / feature_std[None, :]).astype(np.float32)
        for name, value in canonical_features.items()
    }
    normalized_fields, gap_mean, gap_std = _normalized_gap_fields(
        cache.fields_by_split["train"], cache.fields_by_split
    )
    train = datasets["train"]
    validation = datasets["validation"]
    class_counts = np.bincount(train.patch_count.astype(int), minlength=len(CLASS_NAMES)).astype(float)
    class_weights = np.sqrt(np.max(class_counts) / np.maximum(class_counts, 1.0)).astype(np.float32)
    class_weights /= np.mean(class_weights)
    model = WRCPNetA2G(hidden_sizes=hidden_sizes, grid_size=cache.canonical_y_m.size, seed=seed)
    optimizer = _Adam(model, learning_rate=learning_rate)
    rng = np.random.default_rng(seed)
    best_parameters = model.copy_parameters()
    best_validation = np.inf
    best_epoch = 0
    stale = 0
    history: list[dict[str, Any]] = []
    for epoch in range(1, epochs + 1):
        indexes = rng.permutation(len(train))
        totals = np.zeros((4,), dtype=float)
        observations = 0
        for start in range(0, len(indexes), batch_size):
            batch_indexes = indexes[start : start + batch_size]
            output, forward_cache = model.forward(
                normalized_features["train"][batch_indexes], return_cache=True
            )
            losses = _field_loss_and_gradient(
                output,
                patch_count=train.patch_count[batch_indexes],
                target_normalized=normalized_fields["train"][batch_indexes],
                target_physical=cache.fields_by_split["train"][batch_indexes],
                class_weights=class_weights,
                near_surface_scale_m=near_surface_scale_m,
                positive_weight=positive_weight,
                double_field_weight=double_field_weight,
                field_weight=field_weight,
                derivative_weight=derivative_weight,
                huber_delta=huber_delta,
            )
            gradients = model.backward(forward_cache, losses[4], weight_decay=weight_decay)
            optimizer.step(model, *gradients)
            count = len(batch_indexes)
            totals += np.asarray(losses[:4]) * count
            observations += count
        train_losses = totals / max(observations, 1)
        validation_losses = _evaluate_loss(
            model,
            normalized_features["validation"],
            validation,
            normalized_fields["validation"],
            cache.fields_by_split["validation"],
            class_weights=class_weights,
            near_surface_scale_m=near_surface_scale_m,
            positive_weight=positive_weight,
            double_field_weight=double_field_weight,
            field_weight=field_weight,
            derivative_weight=derivative_weight,
            huber_delta=huber_delta,
            batch_size=batch_size,
        )
        history.append(
            {
                "epoch": epoch,
                "train_total": float(train_losses[0]),
                "train_classification": float(train_losses[1]),
                "train_field": float(train_losses[2]),
                "train_derivative": float(train_losses[3]),
                "validation_total": float(validation_losses[0]),
                "validation_classification": float(validation_losses[1]),
                "validation_field": float(validation_losses[2]),
                "validation_derivative": float(validation_losses[3]),
            }
        )
        if validation_losses[0] < best_validation - 1.0e-6:
            best_validation = validation_losses[0]
            best_parameters = model.copy_parameters()
            best_epoch = epoch
            stale = 0
        else:
            stale += 1
        if epoch == 1 or epoch % 10 == 0:
            print(
                f"epoch={epoch:03d} train={train_losses[0]:.6f} "
                f"validation={validation_losses[0]:.6f} best={best_validation:.6f}",
                flush=True,
            )
        if stale >= patience:
            print(f"early_stop epoch={epoch} best_epoch={best_epoch}", flush=True)
            break
    model.restore_parameters(best_parameters)

    context = build_network_a_teacher_context(repo_root)
    predictions = {
        name: _predict(
            model,
            datasets[name].features,
            feature_mean=feature_mean,
            feature_std=feature_std,
            gap_mean=gap_mean,
            gap_std=gap_std,
        )
        for name in ("validation", "test")
    }
    validation_metrics, _, _ = _geometry_metrics(
        context, validation, predictions["validation"], cache.canonical_y_m
    )
    test_metrics, reconstructed_count, reconstructed_labels = _geometry_metrics(
        context, datasets["test"], predictions["test"], cache.canonical_y_m
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
        canonical_y_m=cache.canonical_y_m,
        feature_mean=feature_mean,
        feature_std=feature_std,
        gap_mean=gap_mean,
        gap_std=gap_std,
    )
    metrics_path = destination / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    with (destination / "history.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(history[0]))
        writer.writeheader()
        writer.writerows(history)
    np.savez_compressed(
        destination / "test_predictions.npz",
        expected_patch_count=datasets["test"].patch_count,
        predicted_patch_count=predictions["test"].patch_count,
        reconstructed_patch_count=reconstructed_count,
        predicted_gap_field_m=predictions["test"].gap_field_m,
        reconstructed_labels=reconstructed_labels,
        class_probability=predictions["test"].class_probability,
        sample_id=datasets["test"].metadata["sample_id"],
    )
    manifest = {
        "model_name": MODEL_NAME,
        "model_id": MODEL_ID,
        "architecture": {
            "type": "canonical_gap_field_mlp_with_profile_geometry_reconstruction",
            "reconstruction_method": RECONSTRUCTION_METHOD,
            "layer_sizes": list(model.layer_sizes),
            "activation": "relu",
            "grid_size": int(cache.canonical_y_m.size),
        },
        "dataset_manifest_sha256": _sha256_file(dataset_root / "manifest.json"),
        "gap_cache_manifest_sha256": _sha256_file(Path(cache_dir) / "manifest.json"),
        "hyperparameters": {
            "epochs_requested": int(epochs),
            "epochs_completed": len(history),
            "batch_size": int(batch_size),
            "learning_rate": float(learning_rate),
            "field_weight": float(field_weight),
            "derivative_weight": float(derivative_weight),
            "positive_weight": float(positive_weight),
            "double_field_weight": float(double_field_weight),
            "near_surface_scale_m": float(near_surface_scale_m),
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
    (destination / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    _write_summary(destination / "summary.md", metrics, manifest)
    return WRCPNetA2GTrainingResult(destination, model_path, metrics_path, best_epoch, metrics)


def _write_summary(path: Path, metrics: Mapping[str, Any], manifest: Mapping[str, Any]) -> None:
    test = metrics["test"]
    classification = test["classification"]
    labels = test["conditional_on_correct_count"]["by_label"]
    rail_y = labels["corrected_rail_y_m"]
    penetration = labels["corrected_vertical_penetration_m"]
    angle = labels["corrected_contact_angle_rad"]
    lines = [
        f"# {MODEL_NAME} Training Summary",
        "",
        f"- best epoch: `{metrics['best_epoch']}`",
        f"- test contact-state accuracy: `{classification['accuracy']:.6f}`",
        f"- test double-patch recall: `{classification['per_class']['double_patch']['recall']:.6f}`",
        f"- end-to-end count match rate: `{test['end_to_end_count_matches_teacher_rate']:.6f}`",
        f"- contact position within 1 mm over all contact samples: `{test['contact_position_within_1mm_over_all_contact_samples']:.6f}`",
        f"- corrected rail-y conditional MAE: `{rail_y['mae']:.9g} m`",
        f"- corrected rail-y conditional P95: `{rail_y['p95_absolute_error']:.9g} m`",
        f"- corrected penetration conditional MAE: `{penetration['mae']:.9g} m`",
        f"- corrected angle conditional MAE: `{angle['mae']:.9g} rad`",
        f"- elapsed: `{manifest['elapsed_seconds']:.3f} s`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=f"Train or inspect {MODEL_NAME}.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    train = subparsers.add_parser("train")
    train.add_argument("--dataset", type=Path, default=Path("outputs/network_a_dataset_v1"))
    train.add_argument("--cache", type=Path, default=Path("outputs/network_a_gap_field_v1"))
    train.add_argument("--output", type=Path, default=Path("outputs/wrcp_net_a2g"))
    train.add_argument("--repo-root", type=Path, default=None)
    train.add_argument("--epochs", type=int, default=500)
    train.add_argument("--batch-size", type=int, default=256)
    train.add_argument("--learning-rate", type=float, default=5.0e-4)
    train.add_argument("--hidden-sizes", type=int, nargs="+", default=(128, 128))
    train.add_argument("--field-weight", type=float, default=2.0)
    train.add_argument("--derivative-weight", type=float, default=0.2)
    train.add_argument("--positive-weight", type=float, default=12.0)
    train.add_argument("--double-field-weight", type=float, default=1.0)
    train.add_argument("--near-surface-scale", type=float, default=5.0e-4)
    train.add_argument("--weight-decay", type=float, default=1.0e-5)
    train.add_argument("--huber-delta", type=float, default=1.0)
    train.add_argument("--patience", type=int, default=60)
    train.add_argument("--seed", type=int, default=20260720)
    inspect = subparsers.add_parser("inspect")
    inspect.add_argument("path", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argument_parser().parse_args(argv)
    if args.command == "train":
        result = train_wrcp_net_a2g(
            dataset_dir=args.dataset,
            cache_dir=args.cache,
            output_dir=args.output,
            repo_root=args.repo_root,
            epochs=args.epochs,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            hidden_sizes=args.hidden_sizes,
            field_weight=args.field_weight,
            derivative_weight=args.derivative_weight,
            positive_weight=args.positive_weight,
            double_field_weight=args.double_field_weight,
            near_surface_scale_m=args.near_surface_scale,
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
