from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping, Sequence

import numpy as np
from scipy.stats import qmc

from sditt.config import DEFAULT_OPERATING_CASE, ProjectPaths
from sditt.contact import MultiPointContactGeometry, WheelPose2D, multi_point_contact_geometry
from sditt.contact.full_case import DefaultTrackContactParameters
from sditt.profiles import (
    TrackProfileSet,
    WheelProfileSet,
    build_interval_basic_rail_profile_selector,
    build_track_profiles,
    build_wheel_profiles,
    offset_profile_to_track,
)
from sditt.simulation import (
    FullCaseAcceptedContactSnapshot,
    FullDefaultCaseSettings,
    run_default_full_case_driver,
)
from sditt.track import TrackIrregularitySettings
from sditt.vehicle import load_vehicle_parameters


SCHEMA_VERSION = "network-a-dataset-v1"
SOURCE_MODEL_ID = "python:sditt.contact.geometry.multi_point_contact_geometry:v1"
MAX_PATCHES = 2
BOUNDARY_PENETRATION_M = 2.0e-5
FLANGE_CONTACT_ANGLE_RAD = 0.20
MULTI_PATCH_MIN_SEPARATION_M = 1.0e-3
PARAMETER_CLASSES = ("normal", "flange_multi", "boundary", "no_contact")
FEATURE_NAMES = (
    "side_id",
    "delta_y_m",
    "delta_z_m",
    "roll_rad",
    "yaw_rad",
    "d0_m",
)
LABEL_NAMES = (
    "patch_start_y_m",
    "patch_end_y_m",
    "corrected_wheel_x_m",
    "corrected_wheel_y_m",
    "corrected_wheel_z_m",
    "corrected_rail_y_m",
    "corrected_rail_z_m",
    "wheel_profile_lateral_m",
    "corrected_vertical_penetration_m",
    "corrected_normal_penetration_m",
    "corrected_contact_angle_rad",
    "peak_wheel_x_m",
    "peak_wheel_y_m",
    "peak_wheel_z_m",
    "peak_rail_y_m",
    "peak_rail_z_m",
    "peak_vertical_penetration_m",
    "peak_normal_penetration_m",
    "peak_contact_angle_rad",
)
METADATA_FIELDS = (
    "sample_id",
    "source",
    "sample_class",
    "group_id",
    "front_mileage_m",
    "actual_mileage_m",
    "wheelset",
    "side",
    "stage",
    "step_index",
    "iteration",
    "time_s",
    "dt_s",
    "converged",
    "irregularity_model",
    "irregularity_seed",
)
_NUMERIC_METADATA_DTYPES: Mapping[str, Any] = {
    "front_mileage_m": float,
    "actual_mileage_m": float,
    "step_index": np.int64,
    "iteration": np.int64,
    "time_s": float,
    "dt_s": float,
    "converged": bool,
    "irregularity_seed": np.int64,
}
_DEFAULT_MIN_SPANS = np.array([0.020, 0.004, 0.010, 0.010, 0.002], dtype=float)

# Deterministic discovery anchors for the fixed LMA wheel and standard interval
# rail profiles.  Each row is side_id, delta_y, delta_z, roll, yaw and d0.  The
# anchors are only starting points: accepted samples are Sobol perturbations
# whose two-patch flange geometry is re-evaluated by the traditional teacher.
_FLANGE_MULTI_ANCHORS = np.array(
    [
        [0.0, -0.0146137826, -0.00600, -0.0117431760, -0.0079246464, 0.1671842170],
        [0.0, -0.0156663430, -0.00600, -0.0138700383, 0.0098594967, 0.1656192390],
        [0.0, -0.0178112500, -0.00790, -0.0181126640, -0.0034005982, 0.1644003700],
        [0.0, -0.0165327120, -0.00800, -0.0161204100, -0.0075219695, 0.1659671000],
        [0.0, -0.0163839250, -0.00785, -0.0145628810, 0.0096259688, 0.1669452100],
        [0.0, -0.0182427630, -0.00705, -0.0173346140, -0.0005077280, 0.1640947000],
        [1.0, 0.0146137826, -0.00600, 0.0117431760, 0.0079246464, 0.1671842170],
        [1.0, 0.0156663430, -0.00600, 0.0138700383, -0.0098594967, 0.1656192390],
        [1.0, 0.0178112500, -0.00790, 0.0181126640, 0.0034005982, 0.1644003700],
        [1.0, 0.0165327120, -0.00800, 0.0161204100, 0.0075219695, 0.1659671000],
        [1.0, 0.0163839250, -0.00785, 0.0145628810, -0.0096259688, 0.1669452100],
        [1.0, 0.0182427630, -0.00705, 0.0173346140, 0.0005077280, 0.1640947000],
    ],
    dtype=float,
)
_FLANGE_MULTI_PERTURBATION = np.array([0.0010, 0.0020, 0.0050, 0.0020], dtype=float)


@dataclass(frozen=True)
class NetworkADataset:
    """In-memory network-A samples using fixed arrays and columnar metadata."""

    features: np.ndarray
    patch_count: np.ndarray
    patch_mask: np.ndarray
    labels: np.ndarray
    metadata: dict[str, np.ndarray]

    def __len__(self) -> int:
        return int(self.features.shape[0])

    def subset(self, indexes: np.ndarray | Sequence[int]) -> "NetworkADataset":
        selection = np.asarray(indexes)
        return NetworkADataset(
            features=self.features[selection].copy(),
            patch_count=self.patch_count[selection].copy(),
            patch_mask=self.patch_mask[selection].copy(),
            labels=self.labels[selection].copy(),
            metadata={name: values[selection].copy() for name, values in self.metadata.items()},
        )

    @classmethod
    def concatenate(cls, datasets: Iterable["NetworkADataset"]) -> "NetworkADataset":
        items = [dataset for dataset in datasets if len(dataset)]
        if not items:
            return empty_network_a_dataset()
        return cls(
            features=np.concatenate([item.features for item in items], axis=0),
            patch_count=np.concatenate([item.patch_count for item in items], axis=0),
            patch_mask=np.concatenate([item.patch_mask for item in items], axis=0),
            labels=np.concatenate([item.labels for item in items], axis=0),
            metadata={
                name: np.concatenate([item.metadata[name] for item in items], axis=0)
                for name in METADATA_FIELDS
            },
        )


@dataclass(frozen=True)
class NetworkATeacherContext:
    paths: ProjectPaths
    wheel_profiles: WheelProfileSet
    track_profiles: TrackProfileSet
    dlb: float
    profile_hashes: dict[str, str]


@dataclass(frozen=True)
class NetworkAGenerationResult:
    output_dir: Path
    manifest_path: Path
    quality_report_path: Path
    dataset: NetworkADataset


class _SampleAccumulator:
    def __init__(self) -> None:
        self.features: list[np.ndarray] = []
        self.patch_count: list[int] = []
        self.patch_mask: list[np.ndarray] = []
        self.labels: list[np.ndarray] = []
        self.metadata: dict[str, list[Any]] = {name: [] for name in METADATA_FIELDS}
        self.rejected_overflow: list[dict[str, Any]] = []

    def append(
        self,
        features: np.ndarray,
        geometry: MultiPointContactGeometry,
        *,
        rail_shift_yz: Sequence[float] = (0.0, 0.0),
        metadata: Mapping[str, Any],
    ) -> bool:
        if len(geometry.patches) > MAX_PATCHES:
            self.rejected_overflow.append(
                {
                    "features": np.asarray(features, dtype=float).tolist(),
                    "patch_count": len(geometry.patches),
                    "group_id": str(metadata.get("group_id", "")),
                }
            )
            return False
        patch_count, patch_mask, labels = geometry_to_network_a_labels(
            geometry,
            rail_shift_yz=rail_shift_yz,
        )
        self.features.append(np.asarray(features, dtype=float).copy())
        self.patch_count.append(patch_count)
        self.patch_mask.append(patch_mask)
        self.labels.append(labels)
        complete = _complete_metadata(metadata, features=np.asarray(features, dtype=float), patch_count=patch_count)
        for name in METADATA_FIELDS:
            self.metadata[name].append(complete[name])
        return True

    def dataset(self) -> NetworkADataset:
        if not self.features:
            return empty_network_a_dataset()
        metadata = {
            name: np.asarray(values, dtype=_NUMERIC_METADATA_DTYPES.get(name, str))
            for name, values in self.metadata.items()
        }
        return NetworkADataset(
            features=np.stack(self.features).astype(float, copy=False),
            patch_count=np.asarray(self.patch_count, dtype=np.int8),
            patch_mask=np.stack(self.patch_mask).astype(bool, copy=False),
            labels=np.stack(self.labels).astype(float, copy=False),
            metadata=metadata,
        )


def empty_network_a_dataset() -> NetworkADataset:
    return NetworkADataset(
        features=np.zeros((0, len(FEATURE_NAMES)), dtype=float),
        patch_count=np.zeros((0,), dtype=np.int8),
        patch_mask=np.zeros((0, MAX_PATCHES), dtype=bool),
        labels=np.zeros((0, MAX_PATCHES, len(LABEL_NAMES)), dtype=float),
        metadata={
            name: np.asarray([], dtype=_NUMERIC_METADATA_DTYPES.get(name, str))
            for name in METADATA_FIELDS
        },
    )


def build_network_a_teacher_context(repo_root: str | Path | None = None) -> NetworkATeacherContext:
    paths = ProjectPaths.from_repo_root(repo_root)
    vehicle = load_vehicle_parameters(paths.default_vehicle_parameters, vlc=DEFAULT_OPERATING_CASE.vlc)
    wheel_profiles = build_wheel_profiles(
        paths.wheel_profile_dir,
        vehicle_type="CRH380A",
        drc=float(vehicle.values["Drc"]),
        dlb=float(vehicle.values["Dlb"]),
        r0=float(vehicle.values["R0"]),
    )
    selector = build_interval_basic_rail_profile_selector(repo_root=paths.root)
    selected = selector.select(0.0)
    track_parameters = DefaultTrackContactParameters()
    offsets = {}
    for rail, side in (("L1", "L"), ("R1", "R")):
        offsets[rail] = offset_profile_to_track(
            selected[rail].by_station["FF"],
            wheel_side=side,
            ori_prr=track_parameters.ori_prr,
            dis_rail_y=0.0,
            dis_rail_z=0.0,
            irregularity_y=0.0,
            irregularity_z=0.0,
            gauge=track_parameters.gauge,
            vertical_offset=track_parameters.vertical_offset,
        )
    track_profiles = build_track_profiles(
        offsets,
        left_dummy_rails=("L1",),
        right_dummy_rails=("R1",),
    )
    source_files = {
        "wheel_profile": paths.wheel_profile_dir / "LMA_UnitMM.txt",
        "rail_profile": paths.standard_basic_rail_profile,
        "vehicle_parameters": paths.default_vehicle_parameters,
    }
    return NetworkATeacherContext(
        paths=paths,
        wheel_profiles=wheel_profiles,
        track_profiles=track_profiles,
        dlb=float(vehicle.values["Dlb"]),
        profile_hashes={name: _sha256_file(path) for name, path in source_files.items()},
    )


def teacher_geometry_from_features(
    context: NetworkATeacherContext,
    features: np.ndarray,
) -> MultiPointContactGeometry:
    values = np.asarray(features, dtype=float)
    if values.shape != (len(FEATURE_NAMES),):
        raise ValueError(f"network-A feature vector must have shape {(len(FEATURE_NAMES),)}")
    side = "L" if int(round(values[0])) == 0 else "R"
    wheel_profile = context.wheel_profiles.left if side == "L" else context.wheel_profiles.right
    wheel_angles = (
        context.wheel_profiles.contact_angle_left
        if side == "L"
        else context.wheel_profiles.contact_angle_right
    )
    return multi_point_contact_geometry(
        wheel_profile,
        wheel_angles,
        np.asarray(context.track_profiles.profile[side], dtype=float),
        pose=WheelPose2D(
            lateral=float(values[1]),
            vertical=float(values[2]),
            roll=float(values[3]),
            yaw=float(values[4]),
        ),
        penetration_offset=float(values[5]),
        min_overlap_margin=1.0e-4,
        dlb=context.dlb,
    )


def geometry_to_network_a_labels(
    geometry: MultiPointContactGeometry,
    *,
    rail_shift_yz: Sequence[float] = (0.0, 0.0),
) -> tuple[int, np.ndarray, np.ndarray]:
    patch_count = len(geometry.patches)
    if patch_count > MAX_PATCHES:
        raise ValueError(f"network-A v1 supports at most {MAX_PATCHES} patches, got {patch_count}")
    shift_y, shift_z = (float(value) for value in rail_shift_yz)
    patch_mask = np.zeros((MAX_PATCHES,), dtype=bool)
    labels = np.zeros((MAX_PATCHES, len(LABEL_NAMES)), dtype=float)
    for index, patch in enumerate(geometry.patches):
        patch_mask[index] = True
        start_y = float(geometry.elastic_penetration[patch.start_index, 0]) - shift_y
        end_y = float(geometry.elastic_penetration[patch.end_index, 0]) - shift_y
        corrected_wheel = np.asarray(patch.corrected_wheel_point, dtype=float).copy()
        corrected_wheel[1] -= shift_y
        # TracePrinciple leaves wheel vertical translation out of the stored
        # traced wheel point; pose.vertical enters only the penetration curve.
        # Therefore only the lateral rail shift is removed from wheel points.
        corrected_rail = np.asarray(patch.corrected_rail_point, dtype=float).copy()
        corrected_rail[0] -= shift_y
        corrected_rail[1] -= shift_z
        peak_wheel = np.asarray(patch.peak_wheel_point, dtype=float).copy()
        peak_wheel[1] -= shift_y
        peak_rail = np.asarray(patch.peak_rail_point, dtype=float).copy()
        peak_rail[0] -= shift_y
        peak_rail[1] -= shift_z
        labels[index, :] = np.array(
            [
                start_y,
                end_y,
                *corrected_wheel,
                *corrected_rail,
                patch.wheel_profile_lateral,
                patch.corrected_vertical_penetration,
                patch.corrected_normal_penetration,
                patch.contact_angle,
                *peak_wheel,
                *peak_rail,
                patch.peak_vertical_penetration,
                patch.peak_normal_penetration,
                patch.peak_contact_angle,
            ],
            dtype=float,
        )
    return patch_count, patch_mask, labels


def _complete_metadata(
    metadata: Mapping[str, Any],
    *,
    features: np.ndarray,
    patch_count: int,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "sample_id": "",
        "source": "unknown",
        "sample_class": "no_contact" if patch_count == 0 else "normal",
        "group_id": "unknown",
        "front_mileage_m": np.nan,
        "actual_mileage_m": np.nan,
        "wheelset": "",
        "side": "L" if int(round(float(features[0]))) == 0 else "R",
        "stage": "",
        "step_index": -1,
        "iteration": -1,
        "time_s": np.nan,
        "dt_s": np.nan,
        "converged": True,
        "irregularity_model": "none",
        "irregularity_seed": -1,
    }
    result.update(metadata)
    if patch_count:
        # The caller can override this for generated boundary samples.
        result["sample_class"] = str(metadata.get("sample_class", "normal"))
    if not result["sample_id"]:
        digest = hashlib.sha256()
        digest.update(np.asarray(features, dtype=np.float64).tobytes())
        digest.update(str(result["source"]).encode())
        digest.update(str(result["group_id"]).encode())
        digest.update(str(result["step_index"]).encode())
        digest.update(str(result["wheelset"]).encode())
        digest.update(str(result["side"]).encode())
        result["sample_id"] = digest.hexdigest()[:24]
    return result


def _sample_class(geometry: MultiPointContactGeometry) -> str:
    if not geometry.patches:
        return "no_contact"
    maximum = max(patch.corrected_vertical_penetration for patch in geometry.patches)
    return "boundary" if maximum <= BOUNDARY_PENETRATION_M else "normal"


def _sha256_file(path: Path, *, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_size):
            digest.update(chunk)
    return digest.hexdigest()


class NetworkACoupledCollector:
    """Convert accepted full-case contact snapshots into compact side samples."""

    def __init__(
        self,
        *,
        group_id: str,
        source: str,
        irregularity_model: str,
        irregularity_seed: int,
    ) -> None:
        self.group_id = group_id
        self.source = source
        self.irregularity_model = irregularity_model
        self.irregularity_seed = irregularity_seed
        self.accumulator = _SampleAccumulator()

    def __call__(self, snapshot: FullCaseAcceptedContactSnapshot) -> None:
        contact = snapshot.wheel_rail_contact
        for wheelset, by_side in contact.geometry_by_wheelset_side.items():
            pose = snapshot.wheel_pose_by_wheelset[wheelset]
            actual_mileage = float(contact.con_ws[wheelset]["Mileage"])
            d0 = float(contact.d0_by_wheelset[wheelset])
            for side in ("L", "R"):
                geometry = by_side.get(side)
                if geometry is None:
                    continue
                rail_shift = np.asarray(
                    snapshot.effective_rail_displacement_by_wheelset_side[wheelset][side],
                    dtype=float,
                )
                features = np.array(
                    [
                        0.0 if side == "L" else 1.0,
                        pose.lateral - rail_shift[0],
                        pose.vertical - rail_shift[1],
                        pose.roll,
                        pose.yaw,
                        d0,
                    ],
                    dtype=float,
                )
                self.accumulator.append(
                    features,
                    geometry,
                    rail_shift_yz=rail_shift,
                    metadata={
                        "source": self.source,
                        "sample_class": _sample_class(geometry),
                        "group_id": self.group_id,
                        "front_mileage_m": snapshot.front_mileage,
                        "actual_mileage_m": actual_mileage,
                        "wheelset": wheelset,
                        "side": side,
                        "stage": snapshot.stage,
                        "step_index": snapshot.step_index,
                        "iteration": snapshot.iterations,
                        "time_s": snapshot.time,
                        "dt_s": snapshot.dt,
                        "converged": True,
                        "irregularity_model": self.irregularity_model,
                        "irregularity_seed": self.irregularity_seed,
                    },
                )


def collect_coupled_samples(
    count: int,
    *,
    repo_root: str | Path | None,
    group_id: str,
    irregularity_model: str,
    irregularity_seed: int,
    cut_freq: float | None = None,
    skip_samples: int = 0,
) -> tuple[NetworkADataset, list[dict[str, Any]]]:
    if count <= 0:
        return empty_network_a_dataset(), []
    collector = NetworkACoupledCollector(
        group_id=group_id,
        source="coupled_smooth" if irregularity_model == "none" else "coupled_irregular",
        irregularity_model=irregularity_model,
        irregularity_seed=irregularity_seed,
    )
    if skip_samples < 0:
        raise ValueError("skip_samples cannot be negative")
    samples_per_step = DEFAULT_OPERATING_CASE.n_wheels * 2
    # Both Preload and Cal use the same n_steps_per_stage setting.
    required_samples = count + int(skip_samples)
    steps_per_stage = max(1, int(np.ceil(required_samples / (2 * samples_per_step))))
    settings = FullDefaultCaseSettings(
        cut_freq=cut_freq,
        n_steps_per_stage=steps_per_stage,
        history_retention_steps=2,
        accepted_contact_callback=collector,
        track_irregularity=TrackIrregularitySettings(
            model=irregularity_model,
            seed=irregularity_seed,
        ),
    )
    run_default_full_case_driver(
        repo_root=repo_root,
        settings=settings,
        operating_case=DEFAULT_OPERATING_CASE,
    )
    dataset = collector.accumulator.dataset()
    if len(dataset) < required_samples:
        raise RuntimeError(
            f"coupled run produced {len(dataset)} valid samples, fewer than required {required_samples}"
        )
    return dataset.subset(np.arange(skip_samples, required_samples)), collector.accumulator.rejected_overflow


def derive_parameter_bounds(coupled_features: np.ndarray) -> np.ndarray:
    """Return deterministic low/high bounds for the five continuous features."""

    values = np.asarray(coupled_features, dtype=float)
    if values.ndim != 2 or values.shape[1] != len(FEATURE_NAMES):
        raise ValueError("coupled feature array has the wrong shape")
    if values.shape[0] == 0:
        centers = np.array([0.0, 0.0, 0.0, 0.0, 0.1705], dtype=float)
        spans = _DEFAULT_MIN_SPANS.copy()
        return np.column_stack((centers - spans / 2.0, centers + spans / 2.0))

    continuous = values[:, 1:]
    lower = np.quantile(continuous, 0.005, axis=0)
    upper = np.quantile(continuous, 0.995, axis=0)
    width = upper - lower
    lower -= 0.25 * width
    upper += 0.25 * width
    centers = 0.5 * (lower + upper)
    spans = np.maximum(upper - lower, _DEFAULT_MIN_SPANS)
    bounds = np.column_stack((centers - spans / 2.0, centers + spans / 2.0))
    return bounds


def generate_parameter_samples(
    quotas: Mapping[str, int],
    *,
    context: NetworkATeacherContext,
    bounds: np.ndarray,
    seed: int,
    group_start: int = 0,
    skip_accepted: Mapping[str, int] | None = None,
) -> tuple[NetworkADataset, list[dict[str, Any]]]:
    requested = {name: int(quotas.get(name, 0)) for name in PARAMETER_CLASSES}
    if any(value < 0 for value in requested.values()):
        raise ValueError("parameter quotas cannot be negative")
    limits = np.asarray(bounds, dtype=float)
    if limits.shape != (5, 2) or np.any(limits[:, 1] <= limits[:, 0]):
        raise ValueError("parameter bounds must have shape (5, 2) with increasing limits")

    accumulator = _SampleAccumulator()
    skipped_target = {
        name: int((skip_accepted or {}).get(name, 0)) for name in requested
    }
    ordinary_classes = ("normal", "boundary", "no_contact")
    engines = {
        name: qmc.Sobol(d=5, scramble=True, seed=seed + offset)
        for offset, name in enumerate(ordinary_classes)
    }
    accepted_by_class = {name: 0 for name in requested}
    seen_by_class = {name: 0 for name in requested}
    candidate_index = 0
    group_index = int(group_start)
    for target_class in ordinary_classes:
        target = requested[target_class]
        attempts = 0
        while accepted_by_class[target_class] < target:
            points = engines[target_class].random(256)
            for point in points:
                if accepted_by_class[target_class] >= target:
                    break
                attempts += 1
                candidate_index += 1
                side_id = float(candidate_index % 2)
                features = _parameter_feature_candidate(
                    point,
                    side_id=side_id,
                    target_class=target_class,
                    bounds=limits,
                    context=context,
                )
                if features is None:
                    continue
                geometry = teacher_geometry_from_features(context, features)
                actual_class = _sample_class(geometry)
                if actual_class != target_class:
                    continue
                if seen_by_class[target_class] < skipped_target[target_class]:
                    seen_by_class[target_class] += 1
                    continue
                seen_by_class[target_class] += 1
                if accepted_by_class[target_class] % 5 == 0:
                    group_index += 1
                accepted = accumulator.append(
                    features,
                    geometry,
                    metadata={
                        "source": "sobol",
                        "sample_class": target_class,
                        "group_id": f"sobol-{group_index:06d}",
                        "converged": True,
                        "irregularity_model": "none",
                        "irregularity_seed": -1,
                    },
                )
                if accepted:
                    accepted_by_class[target_class] += 1
            if attempts > max(20_000, target * 200):
                raise RuntimeError(
                    f"could not generate requested {target_class} quota: "
                    f"{accepted_by_class[target_class]}/{target} after {attempts} attempts"
                )

    flange_dataset, flange_rejected = _generate_flange_multi_samples(
        requested["flange_multi"],
        context=context,
        seed=seed + 100,
        skip_accepted=skipped_target["flange_multi"],
    )
    return (
        NetworkADataset.concatenate((accumulator.dataset(), flange_dataset)),
        accumulator.rejected_overflow + flange_rejected,
    )


def _generate_flange_multi_samples(
    count: int,
    *,
    context: NetworkATeacherContext,
    seed: int,
    skip_accepted: int = 0,
) -> tuple[NetworkADataset, list[dict[str, Any]]]:
    """Generate balanced left/right flange-related two-patch samples.

    Ordinary low-discrepancy sampling almost never enters these narrow regions.
    We therefore perturb deterministic profile-specific discovery anchors and
    solve the vertical offset from the teacher's penetration curve.  Every
    accepted sample is replayed through the full traditional geometry solver.
    """

    if count < 0 or skip_accepted < 0:
        raise ValueError("flange multi-contact counts cannot be negative")
    if count == 0:
        return empty_network_a_dataset(), []

    target_by_side = dict(zip((0, 1), _allocate_counts(count, 2), strict=True))
    skipped_counts = _allocate_counts(skip_accepted, 2) if skip_accepted else [0, 0]
    skipped_by_side = dict(zip((0, 1), skipped_counts, strict=True))
    accumulator = _SampleAccumulator()
    for side_id in (0, 1):
        anchors = _FLANGE_MULTI_ANCHORS[_FLANGE_MULTI_ANCHORS[:, 0] == float(side_id)]
        engine = qmc.Sobol(d=5, scramble=True, seed=seed + side_id)
        accepted = 0
        seen = 0
        attempts = 0
        while accepted < target_by_side[side_id]:
            for point in engine.random(256):
                if accepted >= target_by_side[side_id]:
                    break
                anchor = anchors[attempts % len(anchors)]
                attempts += 1
                features = _flange_multi_feature_candidate(
                    point,
                    anchor=anchor,
                    context=context,
                )
                if features is None:
                    continue
                geometry = teacher_geometry_from_features(context, features)
                if not _is_flange_multi_geometry(geometry):
                    continue
                if seen < skipped_by_side[side_id]:
                    seen += 1
                    continue
                sequence_index = skipped_by_side[side_id] + accepted
                accepted_sample = accumulator.append(
                    features,
                    geometry,
                    metadata={
                        "source": "sobol",
                        "sample_class": "flange_multi",
                        "group_id": (
                            f"flange-sobol-{'L' if side_id == 0 else 'R'}-"
                            f"{sequence_index // 5:06d}"
                        ),
                        "converged": True,
                        "irregularity_model": "none",
                        "irregularity_seed": -1,
                    },
                )
                if accepted_sample:
                    accepted += 1
            if attempts > max(50_000, (target_by_side[side_id] + skipped_by_side[side_id]) * 500):
                raise RuntimeError(
                    "could not generate requested flange multi-contact quota for "
                    f"side {side_id}: {accepted}/{target_by_side[side_id]} after {attempts} attempts"
                )
    return accumulator.dataset(), accumulator.rejected_overflow


def _flange_multi_feature_candidate(
    point: np.ndarray,
    *,
    anchor: np.ndarray,
    context: NetworkATeacherContext,
) -> np.ndarray | None:
    values = np.asarray(anchor, dtype=float).copy()
    perturbation = (np.asarray(point[:4], dtype=float) - 0.5) * 2.0 * _FLANGE_MULTI_PERTURBATION
    values[[1, 3, 4, 5]] += perturbation
    values[2] = 0.0
    zero_geometry = teacher_geometry_from_features(context, values)
    penetration = zero_geometry.elastic_penetration
    if penetration.shape[0] < 3:
        return None

    low_z = max(-0.012, float(anchor[2]) - 0.004)
    high_z = min(0.006, float(anchor[2]) + 0.004)
    valid_offsets: list[float] = []
    for delta_z in np.linspace(low_z, high_z, 161):
        positive = penetration[:, 1] + delta_z > 0.0
        starts = np.flatnonzero(positive & ~np.r_[False, positive[:-1]])
        ends = np.flatnonzero(positive & ~np.r_[positive[1:], False])
        if starts.size != 2:
            continue
        peaks = np.array(
            [start + int(np.argmax(penetration[start : end + 1, 1])) for start, end in zip(starts, ends)],
            dtype=int,
        )
        angles = np.abs(zero_geometry.contact_angles[peaks])
        separation = abs(float(penetration[peaks[1], 0] - penetration[peaks[0], 0]))
        if np.max(angles) >= FLANGE_CONTACT_ANGLE_RAD and separation >= MULTI_PATCH_MIN_SEPARATION_M:
            valid_offsets.append(float(delta_z))
    if not valid_offsets:
        return None
    selection = min(int(float(point[4]) * len(valid_offsets)), len(valid_offsets) - 1)
    values[2] = valid_offsets[selection]
    return values


def _is_flange_multi_geometry(geometry: MultiPointContactGeometry) -> bool:
    if len(geometry.patches) != 2:
        return False
    angles = np.abs([patch.contact_angle for patch in geometry.patches])
    rail_y = [float(patch.corrected_rail_point[0]) for patch in geometry.patches]
    return bool(
        np.max(angles) >= FLANGE_CONTACT_ANGLE_RAD
        and abs(rail_y[1] - rail_y[0]) >= MULTI_PATCH_MIN_SEPARATION_M
    )


def _parameter_feature_candidate(
    point: np.ndarray,
    *,
    side_id: float,
    target_class: str,
    bounds: np.ndarray,
    context: NetworkATeacherContext,
) -> np.ndarray | None:
    values = bounds[:, 0] + np.asarray(point, dtype=float) * (bounds[:, 1] - bounds[:, 0])
    delta_y, _delta_z, roll, yaw, d0 = values
    low_z, high_z = bounds[1]
    zero_features = np.array([side_id, delta_y, 0.0, roll, yaw, d0], dtype=float)
    zero_geometry = teacher_geometry_from_features(context, zero_features)
    if zero_geometry.elastic_penetration.size == 0:
        return None
    # delta_z enters the elastic-penetration curve as a uniform additive
    # offset, so the contact transition is available directly without an
    # expensive per-candidate bisection.
    threshold = -float(np.max(zero_geometry.elastic_penetration[:, 1]))
    if not (low_z < threshold < high_z):
        return None
    depth_fraction = float(point[1])
    if target_class == "boundary":
        depth = 1.0e-8 + depth_fraction * (0.90 * BOUNDARY_PENETRATION_M)
        delta_z = threshold + depth
    elif target_class == "normal":
        maximum_depth = high_z - threshold
        if maximum_depth <= 1.05 * BOUNDARY_PENETRATION_M:
            return None
        depth = BOUNDARY_PENETRATION_M * 1.10 + depth_fraction * (
            maximum_depth - BOUNDARY_PENETRATION_M * 1.10
        )
        delta_z = threshold + depth
    else:
        maximum_gap = threshold - low_z
        if maximum_gap <= 1.0e-8:
            return None
        delta_z = threshold - (1.0e-8 + depth_fraction * (maximum_gap - 1.0e-8))
    return np.array([side_id, delta_y, delta_z, roll, yaw, d0], dtype=float)


def assign_group_splits(
    dataset: NetworkADataset,
    *,
    train_count: int,
    validation_count: int,
    test_count: int,
) -> dict[str, np.ndarray]:
    targets = {"train": int(train_count), "validation": int(validation_count), "test": int(test_count)}
    if sum(targets.values()) != len(dataset):
        raise ValueError("split counts must add up to dataset size")
    groups = np.asarray(dataset.metadata["group_id"], dtype=str)
    sources = np.asarray(dataset.metadata["source"], dtype=str)
    classes = np.asarray(dataset.metadata["sample_class"], dtype=str)
    group_indexes = {group: np.flatnonzero(groups == group) for group in np.unique(groups)}
    remaining = targets.copy()
    assigned: dict[str, list[np.ndarray]] = {name: [] for name in targets}

    coupled_groups: dict[str, list[str]] = {"coupled_smooth": [], "coupled_irregular": []}
    sobol_groups: dict[str, list[str]] = {name: [] for name in PARAMETER_CLASSES}
    for group, indexes in group_indexes.items():
        source = str(sources[indexes[0]])
        sample_class = str(classes[indexes[0]])
        if source in coupled_groups:
            coupled_groups[source].append(group)
        elif source == "sobol":
            sobol_groups[sample_class].append(group)
        else:
            raise ValueError(f"unsupported split source {source!r}")

    split_order = tuple(targets)
    coupled_total = sum(
        int(group_indexes[group].size)
        for source_name in ("coupled_smooth", "coupled_irregular")
        for group in coupled_groups[source_name]
    )
    coupled_desired = {
        name: coupled_total * targets[name] / max(len(dataset), 1) for name in targets
    }
    coupled_assigned = {name: 0 for name in targets}
    for source_name in ("coupled_smooth", "coupled_irregular"):
        for group in sorted(coupled_groups[source_name]):
            indexes = group_indexes[group]
            candidates = [name for name in split_order if remaining[name] >= indexes.size]
            if not candidates:
                raise RuntimeError(f"coupled group {group} does not fit remaining split capacity")
            selected = max(
                candidates,
                key=lambda name: (
                    coupled_desired[name] - coupled_assigned[name],
                    remaining[name] / max(targets[name], 1),
                    -split_order.index(name),
                ),
            )
            assigned[selected].append(indexes)
            remaining[selected] -= int(indexes.size)
            coupled_assigned[selected] += int(indexes.size)

    # Allocate rare regimes proportionally before the ordinary-contact groups.
    # The latter are deliberately left until last so they can fill the exact
    # residual capacity without sacrificing validation/test rare-case coverage.
    for class_name in ("flange_multi", "boundary", "no_contact"):
        groups_for_class = sorted(sobol_groups[class_name])
        if class_name == "flange_multi":
            groups_for_class.sort(
                key=lambda group: (int(group.rsplit("-", 1)[1]), group)
            )
        group_targets = _proportional_counts(len(groups_for_class), targets)
        preferred_splits = [
            split_name
            for split_name in split_order
            for _ in range(group_targets[split_name])
        ]
        for group, preferred in zip(groups_for_class, preferred_splits, strict=True):
            indexes = group_indexes[group]
            candidates = [name for name in split_order if remaining[name] >= indexes.size]
            if not candidates:
                raise RuntimeError(f"Sobol group {group} does not fit remaining split capacity")
            selected = preferred if preferred in candidates else max(
                candidates,
                key=lambda name: (remaining[name], -split_order.index(name)),
            )
            assigned[selected].append(indexes)
            remaining[selected] -= int(indexes.size)

    for group in sorted(sobol_groups["normal"]):
        indexes = group_indexes[group]
        candidates = [name for name in split_order if remaining[name] >= indexes.size]
        if not candidates:
            raise RuntimeError(f"normal Sobol group {group} does not fit remaining split capacity")
        selected = max(candidates, key=lambda name: (remaining[name], -split_order.index(name)))
        assigned[selected].append(indexes)
        remaining[selected] -= int(indexes.size)

    if any(value != 0 for value in remaining.values()):
        raise RuntimeError(f"could not satisfy exact group split counts; remaining={remaining}")
    return {
        name: np.sort(np.concatenate(indexes) if indexes else np.zeros((0,), dtype=int))
        for name, indexes in assigned.items()
    }


def save_network_a_archive(path: str | Path, dataset: NetworkADataset) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, np.ndarray] = {
        "features": np.asarray(dataset.features, dtype=np.float64),
        "patch_count": np.asarray(dataset.patch_count, dtype=np.int8),
        "patch_mask": np.asarray(dataset.patch_mask, dtype=bool),
        "labels": np.asarray(dataset.labels, dtype=np.float64),
    }
    payload.update({f"meta__{name}": values for name, values in dataset.metadata.items()})
    np.savez_compressed(output, **payload)
    return output


def _load_network_a_archive(path: str | Path) -> NetworkADataset:
    with np.load(Path(path), allow_pickle=False) as archive:
        metadata = {name: archive[f"meta__{name}"].copy() for name in METADATA_FIELDS}
        return NetworkADataset(
            features=archive["features"].copy(),
            patch_count=archive["patch_count"].copy(),
            patch_mask=archive["patch_mask"].copy(),
            labels=archive["labels"].copy(),
            metadata=metadata,
        )


def load_network_a_dataset(
    path: str | Path,
    split: str = "train",
    dtype: str | np.dtype[Any] = np.float32,
) -> NetworkADataset:
    root = Path(path)
    if root.is_file():
        dataset = _load_network_a_archive(root)
    elif split == "all" and (root / "all_samples.npz").exists():
        dataset = _load_network_a_archive(root / "all_samples.npz")
    else:
        shard_paths = sorted((root / "shards").glob(f"{split}_*.npz"))
        if not shard_paths:
            raise FileNotFoundError(f"no network-A shards found for split {split!r} under {root}")
        dataset = NetworkADataset.concatenate(_load_network_a_archive(item) for item in shard_paths)
    target_dtype = np.dtype(dtype)
    return NetworkADataset(
        features=dataset.features.astype(target_dtype, copy=False),
        patch_count=dataset.patch_count,
        patch_mask=dataset.patch_mask,
        labels=dataset.labels.astype(target_dtype, copy=False),
        metadata=dataset.metadata,
    )


def iter_network_a_batches(
    dataset: NetworkADataset,
    *,
    batch_size: int,
    shuffle: bool = False,
    seed: int = 20260717,
) -> Iterator[NetworkADataset]:
    if batch_size <= 0:
        raise ValueError("batch_size must be positive")
    indexes = np.arange(len(dataset))
    if shuffle:
        np.random.default_rng(seed).shuffle(indexes)
    for start in range(0, len(indexes), batch_size):
        yield dataset.subset(indexes[start : start + batch_size])


def _write_split_shards(
    output_dir: Path,
    dataset: NetworkADataset,
    splits: Mapping[str, np.ndarray],
    *,
    shard_size: int = 5_000,
) -> list[str]:
    shard_dir = output_dir / "shards"
    shard_dir.mkdir(parents=True, exist_ok=True)
    for old_path in shard_dir.glob("*.npz"):
        old_path.unlink()
    written: list[str] = []
    for split, indexes in splits.items():
        for shard_index, start in enumerate(range(0, indexes.size, shard_size)):
            path = shard_dir / f"{split}_{shard_index:03d}.npz"
            save_network_a_archive(path, dataset.subset(indexes[start : start + shard_size]))
            written.append(str(path.relative_to(output_dir)))
    return written


def _normalization_payload(dataset: NetworkADataset, train_indexes: np.ndarray) -> dict[str, Any]:
    train = dataset.subset(train_indexes)
    feature_mean = np.mean(train.features, axis=0)
    feature_std = np.std(train.features, axis=0)
    feature_std = np.where(feature_std > 0.0, feature_std, 1.0)
    label_mean = np.zeros((len(LABEL_NAMES),), dtype=float)
    label_std = np.ones((len(LABEL_NAMES),), dtype=float)
    valid_labels = train.labels[train.patch_mask]
    if valid_labels.size:
        label_mean = np.mean(valid_labels, axis=0)
        computed = np.std(valid_labels, axis=0)
        label_std = np.where(computed > 0.0, computed, 1.0)
    return {
        "feature_names": list(FEATURE_NAMES),
        "feature_mean": feature_mean.tolist(),
        "feature_std": feature_std.tolist(),
        "label_names": list(LABEL_NAMES),
        "label_mean": label_mean.tolist(),
        "label_std": label_std.tolist(),
        "computed_from_split": "train",
    }


def generate_network_a_dataset(
    *,
    repo_root: str | Path | None = None,
    output_dir: str | Path | None = None,
    mode: str = "pilot",
    seed: int = 20260717,
    irregularity_seeds: Sequence[int] = (
        20260716,
        20260717,
        20260718,
        20260719,
        20260720,
        20260721,
        20260722,
        20260723,
    ),
    cut_freq: float | None = None,
    replay_samples: int = 500,
) -> NetworkAGenerationResult:
    if mode not in ("pilot", "production"):
        raise ValueError("mode must be 'pilot' or 'production'")
    minimum_seed_count = 4 if mode == "pilot" else 8
    if len(irregularity_seeds) < minimum_seed_count:
        raise ValueError(f"at least {minimum_seed_count} irregularity seeds are required for {mode} mode")
    context = build_network_a_teacher_context(repo_root)
    destination = (
        Path(output_dir)
        if output_dir is not None
        else context.paths.root / "python" / "outputs" / "network_a_dataset_v1"
    )
    destination.mkdir(parents=True, exist_ok=True)
    targets = _generation_targets(mode)

    existing = empty_network_a_dataset()
    existing_manifest: dict[str, Any] | None = None
    manifest_path = destination / "manifest.json"
    all_samples_path = destination / "all_samples.npz"
    if mode == "production" and manifest_path.exists() and all_samples_path.exists():
        existing_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if existing_manifest.get("schema_version") != SCHEMA_VERSION:
            raise RuntimeError("existing pilot dataset uses an incompatible schema")
        existing = _load_network_a_archive(all_samples_path)
        if len(existing) not in (targets["total"], _generation_targets("pilot")["total"]):
            raise RuntimeError(f"existing dataset has unexpected size {len(existing)}")
        manifest_targets = existing_manifest.get("targets", {})
        targets_match = all(manifest_targets.get(name) == value for name, value in targets.items())
        if len(existing) == targets["total"] and targets_match:
            return _finalize_dataset(
                context=context,
                dataset=existing,
                destination=destination,
                mode=mode,
                targets=targets,
                parameter_bounds=np.asarray(existing_manifest["parameter_bounds"], dtype=float),
                rejected_overflow=[],
                seed=seed,
                irregularity_seeds=irregularity_seeds,
                cut_freq=cut_freq,
                replay_samples=replay_samples,
            )
        if not targets_match:
            # Reuse the expensive accepted-step coupled states, but rebuild
            # parameter samples when the designed class composition changes.
            source = np.asarray(existing.metadata["source"], dtype=str)
            existing = existing.subset(np.flatnonzero(source != "sobol"))

    pieces: list[NetworkADataset] = [existing]
    rejected: list[dict[str, Any]] = []
    source_existing = np.asarray(existing.metadata["source"], dtype=str)
    group_existing = np.asarray(existing.metadata["group_id"], dtype=str)

    smooth_existing_count = int(np.count_nonzero(source_existing == "coupled_smooth"))
    smooth_missing = targets["coupled_smooth"] - smooth_existing_count
    if smooth_missing < 0:
        raise RuntimeError("existing pilot has too many smooth coupled samples")
    desired_smooth_groups = 1 if mode == "pilot" else 3
    current_smooth_groups = len(np.unique(group_existing[source_existing == "coupled_smooth"]))
    new_smooth_groups = max(0, desired_smooth_groups - current_smooth_groups)
    if smooth_missing and new_smooth_groups == 0:
        new_smooth_groups = 1
    for offset, count in enumerate(_allocate_counts(smooth_missing, new_smooth_groups)):
        group_id = f"smooth-{current_smooth_groups + offset:02d}"
        part, overflow = collect_coupled_samples(
            count,
            repo_root=context.paths.root,
            group_id=group_id,
            irregularity_model="none",
            irregularity_seed=-1,
            cut_freq=cut_freq,
        )
        pieces.append(part)
        rejected.extend(overflow)

    existing_now = NetworkADataset.concatenate(pieces)
    source_now = np.asarray(existing_now.metadata["source"], dtype=str)
    group_now = np.asarray(existing_now.metadata["group_id"], dtype=str)
    existing_irregular_count = int(np.count_nonzero(source_now == "coupled_irregular"))
    if mode == "pilot":
        selected_seeds = tuple(irregularity_seeds)[:4]
        seed_counts = _allocate_counts(targets["coupled_irregular"], len(selected_seeds))
    elif existing_irregular_count:
        # Preserve the four pilot realizations and add four new realizations;
        # rerunning a seed with a different stage length would repeat early
        # accepted states rather than form a true continuation.
        selected_seeds = tuple(irregularity_seeds)[:8]
        retained_counts = []
        for value in selected_seeds[:4]:
            retained_counts.append(
                int(np.count_nonzero(group_now == f"irregular-seed-{int(value)}"))
            )
        remaining_total = targets["coupled_irregular"] - sum(retained_counts)
        seed_counts = retained_counts + _allocate_counts(remaining_total, 4)
    else:
        selected_seeds = tuple(irregularity_seeds)[:8]
        seed_counts = _allocate_counts(targets["coupled_irregular"], len(selected_seeds))
    for irregularity_seed, target_seed_count in zip(selected_seeds, seed_counts, strict=True):
        group_id = f"irregular-seed-{int(irregularity_seed)}"
        existing_count = int(
            np.count_nonzero((source_now == "coupled_irregular") & (group_now == group_id))
        )
        missing = int(target_seed_count) - existing_count
        if missing < 0:
            raise RuntimeError(f"existing pilot has too many samples for {group_id}")
        if missing:
            part, overflow = collect_coupled_samples(
                missing,
                repo_root=context.paths.root,
                group_id=group_id,
                irregularity_model="china-ballastless",
                irregularity_seed=int(irregularity_seed),
                cut_freq=cut_freq,
            )
            pieces.append(part)
            rejected.extend(overflow)

    coupled_and_existing = NetworkADataset.concatenate(pieces)
    coupled_mask = np.asarray(coupled_and_existing.metadata["source"], dtype=str) != "sobol"
    coupled = coupled_and_existing.subset(np.flatnonzero(coupled_mask))
    if existing_manifest is not None and "parameter_bounds" in existing_manifest:
        parameter_bounds = np.asarray(existing_manifest["parameter_bounds"], dtype=float)
    else:
        parameter_bounds = derive_parameter_bounds(coupled.features)

    current = NetworkADataset.concatenate(pieces)
    source_current = np.asarray(current.metadata["source"], dtype=str)
    class_current = np.asarray(current.metadata["sample_class"], dtype=str)
    existing_parameter_counts = {
        name: int(np.count_nonzero((source_current == "sobol") & (class_current == name)))
        for name in PARAMETER_CLASSES
    }
    parameter_missing = {
        name: targets[f"parameter_{name}"] - existing_parameter_counts[name]
        for name in existing_parameter_counts
    }
    if any(value < 0 for value in parameter_missing.values()):
        raise RuntimeError("existing pilot has too many parameter samples")
    if any(parameter_missing.values()):
        group_start = _maximum_sobol_group(group_current=group_now)
        parameter_part, overflow = generate_parameter_samples(
            parameter_missing,
            context=context,
            bounds=parameter_bounds,
            seed=seed,
            group_start=group_start,
            skip_accepted=existing_parameter_counts,
        )
        pieces.append(parameter_part)
        rejected.extend(overflow)

    dataset = NetworkADataset.concatenate(pieces)
    if len(dataset) != targets["total"]:
        raise RuntimeError(f"generated {len(dataset)} samples, expected {targets['total']}")
    return _finalize_dataset(
        context=context,
        dataset=dataset,
        destination=destination,
        mode=mode,
        targets=targets,
        parameter_bounds=parameter_bounds,
        rejected_overflow=rejected,
        seed=seed,
        irregularity_seeds=selected_seeds,
        cut_freq=cut_freq,
        replay_samples=replay_samples,
    )


def _generation_targets(mode: str) -> dict[str, int]:
    if mode == "pilot":
        return {
            "total": 2_000,
            "coupled_smooth": 240,
            "coupled_irregular": 360,
            "parameter_normal": 760,
            "parameter_flange_multi": 80,
            "parameter_boundary": 350,
            "parameter_no_contact": 210,
            "train": 1_400,
            "validation": 300,
            "test": 300,
        }
    return {
        "total": 20_000,
        "coupled_smooth": 2_400,
        "coupled_irregular": 3_600,
        "parameter_normal": 7_600,
        "parameter_flange_multi": 800,
        "parameter_boundary": 3_500,
        "parameter_no_contact": 2_100,
        "train": 14_000,
        "validation": 3_000,
        "test": 3_000,
    }


def _allocate_counts(total: int, groups: int) -> list[int]:
    if total == 0:
        return []
    if groups <= 0:
        raise ValueError("positive group count is required for a nonzero quota")
    base, remainder = divmod(int(total), int(groups))
    return [base + (1 if index < remainder else 0) for index in range(groups)]


def _proportional_counts(total: int, targets: Mapping[str, int]) -> dict[str, int]:
    """Allocate an integer total using largest remainders and stable tie breaks."""

    denominator = sum(int(value) for value in targets.values())
    if denominator <= 0:
        raise ValueError("proportional allocation requires positive target weights")
    raw = {name: total * int(value) / denominator for name, value in targets.items()}
    counts = {name: int(np.floor(value)) for name, value in raw.items()}
    remainder = int(total) - sum(counts.values())
    order = tuple(targets)
    ranked = sorted(order, key=lambda name: (-(raw[name] - counts[name]), order.index(name)))
    for name in ranked[:remainder]:
        counts[name] += 1
    return counts


def _maximum_sobol_group(*, group_current: np.ndarray) -> int:
    maximum = 0
    for group in np.asarray(group_current, dtype=str):
        if group.startswith("sobol-"):
            try:
                maximum = max(maximum, int(group.split("-", 1)[1]))
            except ValueError:
                continue
    return maximum


def _finalize_dataset(
    *,
    context: NetworkATeacherContext,
    dataset: NetworkADataset,
    destination: Path,
    mode: str,
    targets: Mapping[str, int],
    parameter_bounds: np.ndarray,
    rejected_overflow: Sequence[Mapping[str, Any]],
    seed: int,
    irregularity_seeds: Sequence[int],
    cut_freq: float | None,
    replay_samples: int,
) -> NetworkAGenerationResult:
    splits = assign_group_splits(
        dataset,
        train_count=targets["train"],
        validation_count=targets["validation"],
        test_count=targets["test"],
    )
    quality = validate_network_a_dataset(
        context,
        dataset,
        splits=splits,
        targets=targets,
        rejected_overflow=rejected_overflow,
        replay_samples=replay_samples,
        seed=seed,
    )
    save_network_a_archive(destination / "all_samples.npz", dataset)
    shard_paths = _write_split_shards(destination, dataset, splits)
    np.savez_compressed(destination / "split_indices.npz", **splits)
    normalization = _normalization_payload(dataset, splits["train"])
    (destination / "normalization.json").write_text(
        json.dumps(normalization, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (destination / "rejected_overflow.json").write_text(
        json.dumps(list(rejected_overflow), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    _write_distribution_statistics(destination, dataset)
    quality_report_path = _write_quality_report(destination, quality)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "mode": mode,
        "source_model_id": SOURCE_MODEL_ID,
        "rail_layout": "interval",
        "dynamic_track_model": "turnout_ft_modal_surrogate",
        "dynamic_model_note": "The FT-Modal turnout dynamics affect sampled state distributions only.",
        "feature_names": list(FEATURE_NAMES),
        "label_names": list(LABEL_NAMES),
        "max_patches": MAX_PATCHES,
        "boundary_penetration_m": BOUNDARY_PENETRATION_M,
        "flange_contact_angle_rad": FLANGE_CONTACT_ANGLE_RAD,
        "multi_patch_min_separation_m": MULTI_PATCH_MIN_SEPARATION_M,
        "units": {"length": "m", "angle": "rad", "time": "s"},
        "sample_grain": "accepted_output_step_wheelset_side",
        "sample_count": len(dataset),
        "targets": dict(targets),
        "actual_counts": _dataset_counts(dataset),
        "parameter_bounds": np.asarray(parameter_bounds, dtype=float).tolist(),
        "parameter_bound_rule": "accepted coupled q0.005-q0.995 expanded by 25 percent with minimum spans",
        "flange_multi_sampling_rule": (
            "profile-specific Sobol perturbations around discovered flange-contact anchors; "
            "vertical offset solved from the teacher penetration curve and every sample teacher-verified"
        ),
        "sobol_seed": int(seed),
        "irregularity_seeds": [int(value) for value in irregularity_seeds],
        "cut_freq_hz": None if cut_freq is None else float(cut_freq),
        "profile_hashes_sha256": context.profile_hashes,
        "source_code_hashes_sha256": {
            "network_a_generator": _sha256_file(Path(__file__)),
            "contact_geometry": _sha256_file(context.paths.root / "python" / "sditt" / "contact" / "geometry.py"),
            "full_case_contact": _sha256_file(context.paths.root / "python" / "sditt" / "contact" / "full_case.py"),
        },
        "git_commit": _git_commit(context.paths.root),
        "git_worktree_dirty": _git_worktree_dirty(context.paths.root),
        "split_counts": {name: int(indexes.size) for name, indexes in splits.items()},
        "split_rule": "grouped by coupled trajectory or irregularity seed and Sobol batch",
        "normalization_source": "train",
        "shards": shard_paths,
        "quality": quality,
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return NetworkAGenerationResult(
        output_dir=destination,
        manifest_path=manifest_path,
        quality_report_path=quality_report_path,
        dataset=dataset,
    )


def _dataset_counts(dataset: NetworkADataset) -> dict[str, Any]:
    source = np.asarray(dataset.metadata["source"], dtype=str)
    sample_class = np.asarray(dataset.metadata["sample_class"], dtype=str)
    side = np.asarray(dataset.metadata["side"], dtype=str)
    irregularity_model = np.asarray(dataset.metadata["irregularity_model"], dtype=str)
    return {
        "by_source": {name: int(np.count_nonzero(source == name)) for name in np.unique(source)},
        "parameter_by_class": {
            name: int(np.count_nonzero((source == "sobol") & (sample_class == name)))
            for name in PARAMETER_CLASSES
        },
        "by_side": {name: int(np.count_nonzero(side == name)) for name in np.unique(side)},
        "by_irregularity_model": {
            name: int(np.count_nonzero(irregularity_model == name)) for name in np.unique(irregularity_model)
        },
        "by_patch_count": {
            str(value): int(np.count_nonzero(dataset.patch_count == value))
            for value in np.unique(dataset.patch_count)
        },
        "flange_multi_regimes": _flange_multi_regime_counts(dataset),
    }


def _flange_multi_regime_counts(dataset: NetworkADataset) -> dict[str, int]:
    sample_class = np.asarray(dataset.metadata["sample_class"], dtype=str)
    selected = np.flatnonzero(sample_class == "flange_multi")
    if selected.size == 0:
        return {"total": 0, "tread_flange": 0, "flange_flange": 0, "shallow_patch": 0}
    angles = np.abs(dataset.labels[selected, :, LABEL_NAMES.index("corrected_contact_angle_rad")])
    penetrations = dataset.labels[
        selected, :, LABEL_NAMES.index("corrected_vertical_penetration_m")
    ]
    flange = angles >= FLANGE_CONTACT_ANGLE_RAD
    return {
        "total": int(selected.size),
        "tread_flange": int(np.count_nonzero(np.any(flange, axis=1) & ~np.all(flange, axis=1))),
        "flange_flange": int(np.count_nonzero(np.all(flange, axis=1))),
        "shallow_patch": int(np.count_nonzero(np.min(penetrations, axis=1) <= BOUNDARY_PENETRATION_M)),
    }


def validate_network_a_dataset(
    context: NetworkATeacherContext,
    dataset: NetworkADataset,
    *,
    splits: Mapping[str, np.ndarray],
    targets: Mapping[str, int],
    rejected_overflow: Sequence[Mapping[str, Any]],
    replay_samples: int,
    seed: int,
) -> dict[str, Any]:
    errors: list[str] = []
    if dataset.features.shape != (len(dataset), len(FEATURE_NAMES)):
        errors.append("feature array shape mismatch")
    if dataset.labels.shape != (len(dataset), MAX_PATCHES, len(LABEL_NAMES)):
        errors.append("label array shape mismatch")
    if dataset.patch_mask.shape != (len(dataset), MAX_PATCHES):
        errors.append("patch-mask array shape mismatch")
    if not np.all(np.isfinite(dataset.features)):
        errors.append("features contain NaN or Inf")
    if np.any(dataset.patch_count != np.sum(dataset.patch_mask, axis=1)):
        errors.append("patch_count does not match patch_mask")
    valid_labels = dataset.labels[dataset.patch_mask]
    if valid_labels.size and not np.all(np.isfinite(valid_labels)):
        errors.append("valid labels contain NaN or Inf")
    if np.any(dataset.labels[~dataset.patch_mask] != 0.0):
        errors.append("inactive patch labels are not zero-filled")
    sample_ids = np.asarray(dataset.metadata["sample_id"], dtype=str)
    if np.unique(sample_ids).size != sample_ids.size:
        errors.append("sample IDs are not unique")

    split_groups = {
        name: set(np.asarray(dataset.metadata["group_id"], dtype=str)[indexes])
        for name, indexes in splits.items()
    }
    names = tuple(split_groups)
    for left_index, left in enumerate(names):
        for right in names[left_index + 1 :]:
            overlap = split_groups[left] & split_groups[right]
            if overlap:
                errors.append(f"group leakage between {left} and {right}: {sorted(overlap)[:3]}")

    counts = _dataset_counts(dataset)
    expected_source = {
        "coupled_smooth": int(targets["coupled_smooth"]),
        "coupled_irregular": int(targets["coupled_irregular"]),
        "sobol": int(
            sum(int(targets[f"parameter_{name}"]) for name in PARAMETER_CLASSES)
        ),
    }
    for name, expected in expected_source.items():
        actual = int(counts["by_source"].get(name, 0))
        if actual != expected:
            errors.append(f"source quota mismatch for {name}: {actual} != {expected}")
    for name in PARAMETER_CLASSES:
        actual = int(counts["parameter_by_class"].get(name, 0))
        expected = int(targets[f"parameter_{name}"])
        tolerance = max(1, int(np.ceil(expected * 0.02)))
        if abs(actual - expected) > tolerance:
            errors.append(f"parameter class quota mismatch for {name}: {actual} != {expected}")

    flange_indexes = np.flatnonzero(
        (np.asarray(dataset.metadata["source"], dtype=str) == "sobol")
        & (np.asarray(dataset.metadata["sample_class"], dtype=str) == "flange_multi")
    )
    for index in flange_indexes:
        geometry = teacher_geometry_from_features(context, dataset.features[index])
        if not _is_flange_multi_geometry(geometry):
            errors.append(f"invalid flange multi-contact sample at index {int(index)}")
            break

    source = np.asarray(dataset.metadata["source"], dtype=str)
    side = np.asarray(dataset.metadata["side"], dtype=str)
    irregularity = np.asarray(dataset.metadata["irregularity_model"], dtype=str)
    for split, indexes in splits.items():
        if set(side[indexes]) != {"L", "R"}:
            errors.append(f"split {split} does not cover both wheel sides")
        if "sobol" not in set(source[indexes]):
            errors.append(f"split {split} does not contain parameter samples")
        if "china-ballastless" not in set(irregularity[indexes]):
            errors.append(f"split {split} does not contain irregularity samples")
        split_classes = set(np.asarray(dataset.metadata["sample_class"], dtype=str)[indexes])
        if int(targets.get("parameter_flange_multi", 0)) and "flange_multi" not in split_classes:
            errors.append(f"split {split} does not contain flange multi-contact samples")
        split_flange_sides = set(
            side[indexes][
                np.asarray(dataset.metadata["sample_class"], dtype=str)[indexes] == "flange_multi"
            ]
        )
        if int(targets.get("parameter_flange_multi", 0)) >= 20 and split_flange_sides != {"L", "R"}:
            errors.append(f"split {split} does not cover both flange multi-contact wheel sides")

    overflow_rate = len(rejected_overflow) / max(len(dataset) + len(rejected_overflow), 1)
    if overflow_rate > 0.001:
        errors.append(f"more-than-two-patch overflow rate {overflow_rate:.6f} exceeds 0.001")

    replay_count = min(max(0, int(replay_samples)), len(dataset))
    replay_indexes = np.random.default_rng(seed).choice(len(dataset), size=replay_count, replace=False)
    max_abs_error = 0.0
    max_rel_error = 0.0
    replay_failures = 0
    for index in replay_indexes:
        geometry = teacher_geometry_from_features(context, dataset.features[index])
        patch_count, patch_mask, labels = geometry_to_network_a_labels(geometry)
        if patch_count != int(dataset.patch_count[index]) or not np.array_equal(
            patch_mask, dataset.patch_mask[index]
        ):
            replay_failures += 1
            continue
        valid = patch_mask
        if not np.any(valid):
            continue
        expected = dataset.labels[index, valid, :]
        actual = labels[valid, :]
        difference = np.abs(actual - expected)
        max_abs_error = max(max_abs_error, float(np.max(difference, initial=0.0)))
        relative = difference / np.maximum(np.abs(expected), 1.0e-30)
        max_rel_error = max(max_rel_error, float(np.max(relative, initial=0.0)))
        if not np.allclose(actual, expected, rtol=1.0e-10, atol=1.0e-12):
            replay_failures += 1
    if replay_failures:
        errors.append(f"teacher replay failed for {replay_failures}/{replay_count} samples")

    result = {
        "passed": not errors,
        "errors": errors,
        "replay_samples": replay_count,
        "replay_failures": replay_failures,
        "replay_max_abs_error": max_abs_error,
        "replay_max_rel_error": max_rel_error,
        "overflow_count": len(rejected_overflow),
        "overflow_rate": overflow_rate,
        "unique_sample_ids": int(np.unique(sample_ids).size),
        "valid_label_count": int(valid_labels.shape[0]),
        "flange_multi_regimes": counts["flange_multi_regimes"],
    }
    if errors:
        raise RuntimeError("network-A dataset validation failed: " + "; ".join(errors))
    return result


def _write_quality_report(output_dir: Path, quality: Mapping[str, Any]) -> Path:
    path = output_dir / "quality_report.md"
    lines = [
        "# Network A Dataset Quality Report",
        "",
        f"- passed: `{quality['passed']}`",
        f"- teacher replay samples: `{quality['replay_samples']}`",
        f"- teacher replay failures: `{quality['replay_failures']}`",
        f"- replay max absolute error: `{quality['replay_max_abs_error']:.17g}`",
        f"- replay max relative error: `{quality['replay_max_rel_error']:.17g}`",
        f"- more-than-two-patch overflow count: `{quality['overflow_count']}`",
        f"- overflow rate: `{quality['overflow_rate']:.6%}`",
        f"- unique sample IDs: `{quality['unique_sample_ids']}`",
        f"- valid patch labels: `{quality['valid_label_count']}`",
        f"- flange multi-contact samples: `{quality['flange_multi_regimes']['total']}`",
        f"- tread-flange samples: `{quality['flange_multi_regimes']['tread_flange']}`",
        f"- flange-flange samples: `{quality['flange_multi_regimes']['flange_flange']}`",
        f"- flange multi-contact samples with a shallow patch: `{quality['flange_multi_regimes']['shallow_patch']}`",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _write_distribution_statistics(output_dir: Path, dataset: NetworkADataset) -> None:
    valid_labels = dataset.labels[dataset.patch_mask]
    series: list[tuple[str, np.ndarray]] = [
        (name, dataset.features[:, index]) for index, name in enumerate(FEATURE_NAMES)
    ]
    if valid_labels.size:
        series.extend(
            [
                ("corrected_rail_y_m", valid_labels[:, LABEL_NAMES.index("corrected_rail_y_m")]),
                (
                    "corrected_vertical_penetration_m",
                    valid_labels[:, LABEL_NAMES.index("corrected_vertical_penetration_m")],
                ),
                (
                    "corrected_contact_angle_rad",
                    valid_labels[:, LABEL_NAMES.index("corrected_contact_angle_rad")],
                ),
            ]
        )
    series.append(("patch_count", dataset.patch_count.astype(float)))
    csv_lines = ["name,count,min,q01,q50,q99,max,mean,std"]
    for name, values in series:
        finite = np.asarray(values, dtype=float)
        finite = finite[np.isfinite(finite)]
        quantiles = np.quantile(finite, [0.01, 0.5, 0.99]) if finite.size else np.full(3, np.nan)
        csv_lines.append(
            ",".join(
                [
                    name,
                    str(finite.size),
                    f"{np.min(finite):.17g}" if finite.size else "nan",
                    f"{quantiles[0]:.17g}",
                    f"{quantiles[1]:.17g}",
                    f"{quantiles[2]:.17g}",
                    f"{np.max(finite):.17g}" if finite.size else "nan",
                    f"{np.mean(finite):.17g}" if finite.size else "nan",
                    f"{np.std(finite):.17g}" if finite.size else "nan",
                ]
            )
        )
    (output_dir / "distribution_statistics.csv").write_text("\n".join(csv_lines) + "\n", encoding="utf-8")
    _write_distribution_svg(output_dir / "distributions.svg", series)


def _write_distribution_svg(path: Path, series: Sequence[tuple[str, np.ndarray]]) -> None:
    columns = 2
    panel_width = 520
    panel_height = 220
    rows = int(np.ceil(len(series) / columns))
    width = columns * panel_width
    height = rows * panel_height
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#17202a}.title{font-size:15px;font-weight:600}.tick{font-size:10px;fill:#566573}</style>',
    ]
    for panel_index, (name, values) in enumerate(series):
        column = panel_index % columns
        row = panel_index // columns
        x0 = column * panel_width + 50
        y0 = row * panel_height + 35
        plot_width = panel_width - 80
        plot_height = panel_height - 75
        finite = np.asarray(values, dtype=float)
        finite = finite[np.isfinite(finite)]
        if finite.size:
            counts, edges = np.histogram(finite, bins=min(30, max(5, int(np.sqrt(finite.size)))))
            maximum = max(int(np.max(counts, initial=1)), 1)
            bar_width = plot_width / max(counts.size, 1)
            for bar_index, count in enumerate(counts):
                bar_height = plot_height * float(count) / maximum
                x = x0 + bar_index * bar_width
                y = y0 + plot_height - bar_height
                parts.append(
                    f'<rect x="{x:.2f}" y="{y:.2f}" width="{max(bar_width - 1.0, 0.5):.2f}" '
                    f'height="{bar_height:.2f}" fill="#2874a6" opacity="0.85"/>'
                )
            parts.append(f'<text class="tick" x="{x0}" y="{y0 + plot_height + 18}">{edges[0]:.3g}</text>')
            parts.append(
                f'<text class="tick" text-anchor="end" x="{x0 + plot_width}" y="{y0 + plot_height + 18}">{edges[-1]:.3g}</text>'
            )
        parts.append(f'<line x1="{x0}" y1="{y0 + plot_height}" x2="{x0 + plot_width}" y2="{y0 + plot_height}" stroke="#7f8c8d"/>')
        parts.append(f'<text class="title" x="{x0}" y="{y0 - 10}">{name}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def _git_commit(repo_root: Path) -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip() or None


def _git_worktree_dirty(repo_root: Path) -> bool | None:
    try:
        result = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return bool(result.stdout.strip())


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate or inspect the SDITT network-A dataset.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    generate = subparsers.add_parser("generate", help="Generate the pilot or production dataset.")
    generate.add_argument("--mode", choices=("pilot", "production"), default="pilot")
    generate.add_argument("--repo-root", type=Path, default=None)
    generate.add_argument("--output", type=Path, default=None)
    generate.add_argument("--seed", type=int, default=20260717)
    generate.add_argument("--cut-freq", type=float, default=None)
    generate.add_argument("--replay-samples", type=int, default=500)
    inspect = subparsers.add_parser("inspect", help="Print compact manifest and split information.")
    inspect.add_argument("path", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_argument_parser().parse_args(argv)
    if args.command == "generate":
        result = generate_network_a_dataset(
            repo_root=args.repo_root,
            output_dir=args.output,
            mode=args.mode,
            seed=args.seed,
            cut_freq=args.cut_freq,
            replay_samples=args.replay_samples,
        )
        print(f"wrote {result.manifest_path}")
        print(f"wrote {result.quality_report_path}")
        print(f"samples={len(result.dataset)}")
        return 0
    manifest = json.loads((args.path / "manifest.json").read_text(encoding="utf-8"))
    print(json.dumps({
        "schema_version": manifest["schema_version"],
        "sample_count": manifest["sample_count"],
        "actual_counts": manifest["actual_counts"],
        "split_counts": manifest["split_counts"],
        "quality": manifest["quality"],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
