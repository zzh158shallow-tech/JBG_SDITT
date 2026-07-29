from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from sditt.models.network_b_features import (
    DIRECT_FEATURE_NAMES,
    FEATURE_NAMES,
    build_network_b_features,
    direct_network_b_patch_shape_features,
    network_b_patch_shape_features,
)
from sditt.contact.geometry import DirectContactGeometry
from sditt.simulation import FullCaseAcceptedContactSnapshot


SCHEMA_VERSION = "network-b-dataset-v4"
DIRECT_SCHEMA_VERSION = "network-b-direct-dataset-v3"
LEGACY_SCHEMA_VERSIONS = {"network-b-dataset-v3"}
LEGACY_DIRECT_SCHEMA_VERSIONS = {"network-b-direct-dataset-v2"}
TARGET_NAMES = (
    "normal_force_N",
    "creep_force_track_x_N",
    "creep_force_track_y_N",
    "creep_force_track_z_N",
    "creep_moment_track_x_Nm",
    "creep_moment_track_y_Nm",
    "creep_moment_track_z_Nm",
)


@dataclass(frozen=True)
class NetworkBDataset:
    features: np.ndarray
    targets: np.ndarray
    irregularity_seed: np.ndarray
    front_mileage_m: np.ndarray
    actual_mileage_m: np.ndarray
    step_index: np.ndarray
    time_s: np.ndarray
    dt_s: np.ndarray
    iterations: np.ndarray
    retry_count: np.ndarray
    wheelset: np.ndarray
    side: np.ndarray
    patch_index: np.ndarray
    patch_count: np.ndarray
    accepted_step_label: np.ndarray
    teacher_match_distance_m: np.ndarray
    feature_names: tuple[str, ...] = FEATURE_NAMES

    def __len__(self) -> int:
        return int(self.features.shape[0])

    def subset(self, indexes: np.ndarray) -> "NetworkBDataset":
        selection = np.asarray(indexes)
        return NetworkBDataset(
            features=self.features[selection].copy(),
            targets=self.targets[selection].copy(),
            irregularity_seed=self.irregularity_seed[selection].copy(),
            front_mileage_m=self.front_mileage_m[selection].copy(),
            actual_mileage_m=self.actual_mileage_m[selection].copy(),
            step_index=self.step_index[selection].copy(),
            time_s=self.time_s[selection].copy(),
            dt_s=self.dt_s[selection].copy(),
            iterations=self.iterations[selection].copy(),
            retry_count=self.retry_count[selection].copy(),
            wheelset=self.wheelset[selection].copy(),
            side=self.side[selection].copy(),
            patch_index=self.patch_index[selection].copy(),
            patch_count=self.patch_count[selection].copy(),
            accepted_step_label=self.accepted_step_label[selection].copy(),
            teacher_match_distance_m=self.teacher_match_distance_m[selection].copy(),
            feature_names=self.feature_names,
        )

    @classmethod
    def concatenate(cls, datasets: list["NetworkBDataset"]) -> "NetworkBDataset":
        items = [dataset for dataset in datasets if len(dataset)]
        if not items:
            return empty_network_b_dataset()
        feature_names = items[0].feature_names
        if any(item.feature_names != feature_names for item in items[1:]):
            raise ValueError("cannot concatenate network-B datasets with different feature schemas")
        return cls(
            features=np.concatenate([item.features for item in items]),
            targets=np.concatenate([item.targets for item in items]),
            irregularity_seed=np.concatenate([item.irregularity_seed for item in items]),
            front_mileage_m=np.concatenate([item.front_mileage_m for item in items]),
            actual_mileage_m=np.concatenate([item.actual_mileage_m for item in items]),
            step_index=np.concatenate([item.step_index for item in items]),
            time_s=np.concatenate([item.time_s for item in items]),
            dt_s=np.concatenate([item.dt_s for item in items]),
            iterations=np.concatenate([item.iterations for item in items]),
            retry_count=np.concatenate([item.retry_count for item in items]),
            wheelset=np.concatenate([item.wheelset for item in items]),
            side=np.concatenate([item.side for item in items]),
            patch_index=np.concatenate([item.patch_index for item in items]),
            patch_count=np.concatenate([item.patch_count for item in items]),
            accepted_step_label=np.concatenate([item.accepted_step_label for item in items]),
            teacher_match_distance_m=np.concatenate(
                [item.teacher_match_distance_m for item in items]
            ),
            feature_names=feature_names,
        )


def empty_network_b_dataset(
    *,
    feature_names: tuple[str, ...] = FEATURE_NAMES,
) -> NetworkBDataset:
    return NetworkBDataset(
        features=np.zeros((0, len(feature_names)), dtype=np.float64),
        targets=np.zeros((0, len(TARGET_NAMES)), dtype=np.float64),
        irregularity_seed=np.zeros((0,), dtype=np.int64),
        front_mileage_m=np.zeros((0,), dtype=np.float64),
        actual_mileage_m=np.zeros((0,), dtype=np.float64),
        step_index=np.zeros((0,), dtype=np.int64),
        time_s=np.zeros((0,), dtype=np.float64),
        dt_s=np.zeros((0,), dtype=np.float64),
        iterations=np.zeros((0,), dtype=np.int64),
        retry_count=np.zeros((0,), dtype=np.int64),
        wheelset=np.zeros((0,), dtype="U2"),
        side=np.zeros((0,), dtype="U1"),
        patch_index=np.zeros((0,), dtype=np.int64),
        patch_count=np.zeros((0,), dtype=np.int64),
        accepted_step_label=np.zeros((0,), dtype=bool),
        teacher_match_distance_m=np.zeros((0,), dtype=np.float64),
        feature_names=feature_names,
    )


class NetworkBAcceptedStepCollector:
    """Collect traditional force labels from final accepted Cal steps only."""

    def __init__(
        self,
        *,
        irregularity_seed: int,
        require_shadow_teacher: bool = False,
        shard_dir: str | Path | None = None,
        shard_interval_m: float = 10.0,
        resume: bool = False,
    ) -> None:
        if shard_interval_m <= 0.0:
            raise ValueError("network-B shard interval must be positive")
        self.irregularity_seed = int(irregularity_seed)
        self.require_shadow_teacher = bool(require_shadow_teacher)
        self.shard_interval_m = float(shard_interval_m)
        self.shard_dir = None if shard_dir is None else Path(shard_dir)
        self._features: list[np.ndarray] = []
        self._targets: list[np.ndarray] = []
        self._front_mileage: list[np.ndarray] = []
        self._actual_mileage: list[np.ndarray] = []
        self._step_index: list[np.ndarray] = []
        self._time_s: list[np.ndarray] = []
        self._dt_s: list[np.ndarray] = []
        self._iterations: list[np.ndarray] = []
        self._retry_count: list[np.ndarray] = []
        self._wheelset: list[np.ndarray] = []
        self._side: list[np.ndarray] = []
        self._patch_index: list[np.ndarray] = []
        self._patch_count: list[np.ndarray] = []
        self._accepted_step_label: list[np.ndarray] = []
        self._teacher_match_distance: list[np.ndarray] = []
        self._feature_names: tuple[str, ...] | None = (
            DIRECT_FEATURE_NAMES if self.require_shadow_teacher else None
        )
        self._accepted_steps: dict[int, dict[str, float | int]] = {}
        self._topology_mismatch_sides = 0
        self._skipped_patch_rows = 0
        self._empty_contact_sides = 0
        self._last_mileage_bucket: int | None = None
        self._shard_paths: list[Path] = []
        self._next_shard_index = 0
        if self.shard_dir is not None:
            self._initialize_shards(resume=resume)

    def __call__(self, snapshot: FullCaseAcceptedContactSnapshot) -> None:
        if snapshot.stage != "Cal":
            return
        self._accepted_steps[int(snapshot.step_index)] = {
            "step_index": int(snapshot.step_index),
            "time_s": float(snapshot.time),
            "dt_s": float(snapshot.dt),
            "front_mileage_m": float(snapshot.front_mileage),
            "iterations": int(snapshot.iterations),
            "retry_count": int(snapshot.retry_count),
        }
        contact = snapshot.wheel_rail_contact
        for wheelset, pose in snapshot.wheel_pose_by_wheelset.items():
            values: dict[str, Any] = contact.con_ws[wheelset]
            actual_mileage = float(values["Mileage"])
            d0 = float(contact.d0_by_wheelset[wheelset])
            for side in ("L", "R"):
                normal = np.asarray(values["Normal_Force"][side], dtype=float)
                if normal.size == 0:
                    self._empty_contact_sides += 1
                    continue
                prhxf = np.asarray(values["Prhxf_T"][side], dtype=float)
                con_rel_vel = np.asarray(values["Con_RelVel"][side], dtype=float)
                geometry = contact.geometry_by_wheelset_side[wheelset][side]
                if geometry is None or len(geometry.patches) != normal.shape[0]:
                    raise ValueError("traditional geometry and force patch counts differ")
                direct = isinstance(geometry, DirectContactGeometry)
                feature_names = DIRECT_FEATURE_NAMES if direct else FEATURE_NAMES
                if self._feature_names is None:
                    self._feature_names = feature_names
                elif self._feature_names != feature_names:
                    raise ValueError("accepted-step collector cannot mix network-B feature schemas")
                if direct:
                    patch_bounds = np.asarray(
                        [
                            [patch.start_y, patch.end_y, patch.end_y - patch.start_y]
                            for patch in geometry.patches
                        ],
                        dtype=float,
                    )
                    patch_shape = direct_network_b_patch_shape_features(geometry)
                else:
                    patch_bounds = np.asarray(
                        [
                            [
                                geometry.elastic_penetration[patch.start_index, 0],
                                geometry.elastic_penetration[patch.end_index, 0],
                                geometry.elastic_penetration[patch.end_index, 0]
                                - geometry.elastic_penetration[patch.start_index, 0],
                            ]
                            for patch in geometry.patches
                        ],
                        dtype=float,
                    )
                    patch_shape = network_b_patch_shape_features(geometry)
                features = build_network_b_features(
                    side=side,
                    pose=pose,
                    d0=d0,
                    patch_ids=normal[:, 3],
                    con_wheel_2=np.asarray(values["Con_wheel_2_full"][side], dtype=float),
                    con_wheel_2_peak=np.asarray(values["Con_wheel_2_a"][side], dtype=float),
                    con_rail_1=np.asarray(values["Con_rail_1"][side], dtype=float),
                    con_rail_1_peak=np.asarray(values["Con_rail_1_a"][side], dtype=float),
                    con_rel_vel=con_rel_vel,
                    vsdc=np.asarray(values["Vsdc"][side], dtype=float),
                    vjsdc=np.asarray(values["Vjsdc"][side], dtype=float),
                    vgd=np.asarray(values["Vgd"][side], dtype=float),
                    patch_bounds=patch_bounds,
                    curvature_inputs=np.asarray(values["Network_B_Curvature_Inputs"][side], dtype=float),
                    patch_shape=patch_shape,
                    feature_names=feature_names,
                )
                if prhxf.shape != (normal.shape[0], 6):
                    raise ValueError("traditional Prhxf_T labels must have six columns")
                shadow_targets = np.asarray(
                    values.get("Network_B_Teacher_Targets", {}).get(side, []),
                    dtype=float,
                )
                teacher_topology_match = bool(
                    values.get("Network_B_Teacher_Topology_Match", {}).get(
                        side,
                        not self.require_shadow_teacher,
                    )
                )
                teacher_match_distance = np.asarray(
                    values.get("Network_B_Teacher_Match_Distance_m", {}).get(side, []),
                    dtype=float,
                ).reshape(-1)
                has_shadow_targets = shadow_targets.shape == (
                    normal.shape[0],
                    len(TARGET_NAMES),
                )
                if self.require_shadow_teacher and (
                    not teacher_topology_match or not has_shadow_targets
                ):
                    self._topology_mismatch_sides += 1
                    self._skipped_patch_rows += int(normal.shape[0])
                    continue
                targets = (
                    shadow_targets
                    if has_shadow_targets
                    else np.column_stack((normal[:, 0], prhxf))
                )
                if teacher_match_distance.shape != (normal.shape[0],):
                    if self.require_shadow_teacher:
                        raise ValueError(
                            "shadow-teacher match distance does not match the contact patch count"
                        )
                    teacher_match_distance = np.zeros((normal.shape[0],), dtype=float)
                if not np.isfinite(teacher_match_distance).all():
                    raise ValueError("network-B teacher match distance contains NaN or Inf")
                if not np.isfinite(targets).all() or np.any(targets[:, 0] < 0.0):
                    raise ValueError("traditional network-B targets are invalid")
                n = normal.shape[0]
                self._features.append(features)
                self._targets.append(targets)
                self._front_mileage.append(np.full((n,), snapshot.front_mileage, dtype=float))
                self._actual_mileage.append(np.full((n,), actual_mileage, dtype=float))
                self._step_index.append(np.full((n,), snapshot.step_index, dtype=np.int64))
                self._time_s.append(np.full((n,), snapshot.time, dtype=float))
                self._dt_s.append(np.full((n,), snapshot.dt, dtype=float))
                self._iterations.append(np.full((n,), snapshot.iterations, dtype=np.int64))
                self._retry_count.append(np.full((n,), snapshot.retry_count, dtype=np.int64))
                self._wheelset.append(np.full((n,), wheelset, dtype="U2"))
                self._side.append(np.full((n,), side, dtype="U1"))
                self._patch_index.append(np.arange(n, dtype=np.int64))
                self._patch_count.append(np.full((n,), n, dtype=np.int64))
                self._accepted_step_label.append(np.ones((n,), dtype=bool))
                self._teacher_match_distance.append(teacher_match_distance)
        self._flush_if_mileage_bucket_changed(snapshot.front_mileage)

    def dataset(self, *, include_shards: bool = True) -> NetworkBDataset:
        datasets: list[NetworkBDataset] = []
        if include_shards:
            datasets.extend(load_network_b_dataset(path) for path in self._shard_paths)
        buffered = self._buffered_dataset()
        if len(buffered):
            datasets.append(buffered)
        combined = NetworkBDataset.concatenate(datasets)
        return _deduplicate_network_b_dataset(combined)

    def finalize(self) -> NetworkBDataset:
        self.flush()
        return self.dataset()

    def flush(self) -> Path | None:
        dataset = self._buffered_dataset()
        path: Path | None = None
        if len(dataset):
            if self.shard_dir is None:
                return None
            path = self.shard_dir / f"shard_{self._next_shard_index:06d}.npz"
            temporary = path.with_suffix(".tmp.npz")
            save_network_b_dataset(temporary, dataset)
            temporary.replace(path)
            self._shard_paths.append(path)
            self._next_shard_index += 1
            self._clear_buffers()
        self._write_collection_state()
        return path

    def summary(self) -> dict[str, object]:
        dataset = self.dataset()
        accepted = [self._accepted_steps[key] for key in sorted(self._accepted_steps)]
        return {
            "seed": self.irregularity_seed,
            "rows": len(dataset),
            "accepted_steps": len(accepted),
            "first_step_index": None if not accepted else int(accepted[0]["step_index"]),
            "last_step_index": None if not accepted else int(accepted[-1]["step_index"]),
            "first_front_mileage_m": (
                None if not accepted else float(accepted[0]["front_mileage_m"])
            ),
            "last_front_mileage_m": (
                None if not accepted else float(accepted[-1]["front_mileage_m"])
            ),
            "retry_count": int(sum(int(item["retry_count"]) for item in accepted)),
            "topology_mismatch_sides": int(self._topology_mismatch_sides),
            "skipped_patch_rows": int(self._skipped_patch_rows),
            "empty_contact_sides": int(self._empty_contact_sides),
            "shards": [str(path) for path in self._shard_paths],
        }

    def _buffered_dataset(self) -> NetworkBDataset:
        if not self._features:
            return empty_network_b_dataset(feature_names=self._feature_names or FEATURE_NAMES)
        count = sum(values.shape[0] for values in self._features)
        return NetworkBDataset(
            features=np.concatenate(self._features),
            targets=np.concatenate(self._targets),
            irregularity_seed=np.full((count,), self.irregularity_seed, dtype=np.int64),
            front_mileage_m=np.concatenate(self._front_mileage),
            actual_mileage_m=np.concatenate(self._actual_mileage),
            step_index=np.concatenate(self._step_index),
            time_s=np.concatenate(self._time_s),
            dt_s=np.concatenate(self._dt_s),
            iterations=np.concatenate(self._iterations),
            retry_count=np.concatenate(self._retry_count),
            wheelset=np.concatenate(self._wheelset),
            side=np.concatenate(self._side),
            patch_index=np.concatenate(self._patch_index),
            patch_count=np.concatenate(self._patch_count),
            accepted_step_label=np.concatenate(self._accepted_step_label),
            teacher_match_distance_m=np.concatenate(self._teacher_match_distance),
            feature_names=self._feature_names or FEATURE_NAMES,
        )

    def _clear_buffers(self) -> None:
        for values in (
            self._features,
            self._targets,
            self._front_mileage,
            self._actual_mileage,
            self._step_index,
            self._time_s,
            self._dt_s,
            self._iterations,
            self._retry_count,
            self._wheelset,
            self._side,
            self._patch_index,
            self._patch_count,
            self._accepted_step_label,
            self._teacher_match_distance,
        ):
            values.clear()

    def _flush_if_mileage_bucket_changed(self, front_mileage: float) -> None:
        bucket = int(np.floor(float(front_mileage) / self.shard_interval_m))
        if self._last_mileage_bucket is None:
            self._last_mileage_bucket = bucket
            return
        if bucket != self._last_mileage_bucket:
            self._last_mileage_bucket = bucket
            self.flush()

    def _initialize_shards(self, *, resume: bool) -> None:
        assert self.shard_dir is not None
        self.shard_dir.mkdir(parents=True, exist_ok=True)
        existing = sorted(self.shard_dir.glob("shard_*.npz"))
        if existing and not resume:
            raise FileExistsError(
                f"network-B shard directory already contains data: {self.shard_dir}"
            )
        self._shard_paths = existing
        self._next_shard_index = len(existing)
        state_path = self.shard_dir / "collection_state.json"
        if resume and state_path.is_file():
            state = json.loads(state_path.read_text(encoding="utf-8"))
            if int(state.get("irregularity_seed", self.irregularity_seed)) != self.irregularity_seed:
                raise ValueError("network-B shard state uses a different irregularity seed")
            self._accepted_steps = {
                int(item["step_index"]): {
                    "step_index": int(item["step_index"]),
                    "time_s": float(item["time_s"]),
                    "dt_s": float(item["dt_s"]),
                    "front_mileage_m": float(item["front_mileage_m"]),
                    "iterations": int(item["iterations"]),
                    "retry_count": int(item["retry_count"]),
                }
                for item in state.get("accepted_steps", [])
            }
            feature_names = tuple(str(value) for value in state.get("feature_names", ()))
            self._feature_names = feature_names or None
            self._topology_mismatch_sides = int(state.get("topology_mismatch_sides", 0))
            self._skipped_patch_rows = int(state.get("skipped_patch_rows", 0))
            self._empty_contact_sides = int(state.get("empty_contact_sides", 0))
            last_bucket = state.get("last_mileage_bucket")
            self._last_mileage_bucket = None if last_bucket is None else int(last_bucket)
        elif resume and existing:
            dataset = NetworkBDataset.concatenate(
                [load_network_b_dataset(path) for path in existing]
            )
            self._feature_names = dataset.feature_names
            for step_index in np.unique(dataset.step_index):
                rows = np.flatnonzero(dataset.step_index == step_index)
                row = int(rows[-1])
                self._accepted_steps[int(step_index)] = {
                    "step_index": int(step_index),
                    "time_s": float(dataset.time_s[row]),
                    "dt_s": float(dataset.dt_s[row]),
                    "front_mileage_m": float(dataset.front_mileage_m[row]),
                    "iterations": int(dataset.iterations[row]),
                    "retry_count": int(dataset.retry_count[row]),
                }

    def _write_collection_state(self) -> None:
        if self.shard_dir is None:
            return
        state = {
            "schema": "network-b-accepted-collector-state-v1",
            "irregularity_seed": self.irregularity_seed,
            "require_shadow_teacher": self.require_shadow_teacher,
            "shard_interval_m": self.shard_interval_m,
            "feature_names": list(self._feature_names or ()),
            "accepted_steps": [
                self._accepted_steps[key] for key in sorted(self._accepted_steps)
            ],
            "topology_mismatch_sides": self._topology_mismatch_sides,
            "skipped_patch_rows": self._skipped_patch_rows,
            "empty_contact_sides": self._empty_contact_sides,
            "last_mileage_bucket": self._last_mileage_bucket,
            "shards": [path.name for path in self._shard_paths],
        }
        path = self.shard_dir / "collection_state.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(state, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)


def save_network_b_dataset(path: str | Path, dataset: NetworkBDataset) -> None:
    np.savez_compressed(
        path,
        schema_version=np.array(
            DIRECT_SCHEMA_VERSION if dataset.feature_names == DIRECT_FEATURE_NAMES else SCHEMA_VERSION
        ),
        feature_names=np.asarray(dataset.feature_names),
        target_names=np.asarray(TARGET_NAMES),
        features=dataset.features,
        targets=dataset.targets,
        irregularity_seed=dataset.irregularity_seed,
        front_mileage_m=dataset.front_mileage_m,
        actual_mileage_m=dataset.actual_mileage_m,
        step_index=dataset.step_index,
        time_s=dataset.time_s,
        dt_s=dataset.dt_s,
        iterations=dataset.iterations,
        retry_count=dataset.retry_count,
        wheelset=dataset.wheelset,
        side=dataset.side,
        patch_index=dataset.patch_index,
        patch_count=dataset.patch_count,
        accepted_step_label=dataset.accepted_step_label,
        teacher_match_distance_m=dataset.teacher_match_distance_m,
    )


def load_network_b_dataset(path: str | Path) -> NetworkBDataset:
    with np.load(path, allow_pickle=False) as payload:
        schema = str(payload["schema_version"].item())
        supported = {
            SCHEMA_VERSION,
            DIRECT_SCHEMA_VERSION,
            *LEGACY_SCHEMA_VERSIONS,
            *LEGACY_DIRECT_SCHEMA_VERSIONS,
        }
        if schema not in supported:
            raise ValueError("unsupported network-B dataset schema")
        feature_names = tuple(str(value) for value in payload["feature_names"])
        direct = schema == DIRECT_SCHEMA_VERSION or schema in LEGACY_DIRECT_SCHEMA_VERSIONS
        expected = DIRECT_FEATURE_NAMES if direct else FEATURE_NAMES
        if feature_names != expected:
            raise ValueError("network-B feature schema does not match runtime")
        if tuple(str(value) for value in payload["target_names"]) != TARGET_NAMES:
            raise ValueError("network-B target schema does not match runtime")
        count = int(payload["features"].shape[0])
        return NetworkBDataset(
            features=np.asarray(payload["features"], dtype=float),
            targets=np.asarray(payload["targets"], dtype=float),
            irregularity_seed=np.asarray(payload["irregularity_seed"], dtype=np.int64),
            front_mileage_m=np.asarray(payload["front_mileage_m"], dtype=float),
            actual_mileage_m=np.asarray(payload["actual_mileage_m"], dtype=float),
            step_index=np.asarray(payload["step_index"], dtype=np.int64),
            time_s=_payload_array(payload, "time_s", count, dtype=float, default=0.0),
            dt_s=_payload_array(payload, "dt_s", count, dtype=float, default=0.0),
            iterations=_payload_array(
                payload,
                "iterations",
                count,
                dtype=np.int64,
                default=0,
            ),
            retry_count=_payload_array(
                payload,
                "retry_count",
                count,
                dtype=np.int64,
                default=0,
            ),
            wheelset=np.asarray(payload["wheelset"]).astype("U2"),
            side=np.asarray(payload["side"]).astype("U1"),
            patch_index=_payload_array(
                payload,
                "patch_index",
                count,
                dtype=np.int64,
                default=0,
            ),
            patch_count=_payload_array(
                payload,
                "patch_count",
                count,
                dtype=np.int64,
                default=1,
            ),
            accepted_step_label=_payload_array(
                payload,
                "accepted_step_label",
                count,
                dtype=bool,
                default=True,
            ),
            teacher_match_distance_m=_payload_array(
                payload,
                "teacher_match_distance_m",
                count,
                dtype=float,
                default=0.0,
            ),
            feature_names=feature_names,
        )


def network_b_dataset_quality_report(dataset: NetworkBDataset) -> dict[str, object]:
    count = len(dataset)
    row_lengths = {
        name: int(np.asarray(getattr(dataset, name)).shape[0])
        for name in (
            "targets",
            "irregularity_seed",
            "front_mileage_m",
            "actual_mileage_m",
            "step_index",
            "time_s",
            "dt_s",
            "iterations",
            "retry_count",
            "wheelset",
            "side",
            "patch_index",
            "patch_count",
            "accepted_step_label",
            "teacher_match_distance_m",
        )
    }
    finite = bool(
        np.isfinite(dataset.features).all()
        and np.isfinite(dataset.targets).all()
        and np.isfinite(dataset.front_mileage_m).all()
        and np.isfinite(dataset.actual_mileage_m).all()
        and np.isfinite(dataset.time_s).all()
        and np.isfinite(dataset.dt_s).all()
        and np.isfinite(dataset.teacher_match_distance_m).all()
    )
    keys = _network_b_row_keys(dataset)
    unique_rows = int(np.unique(keys).size) if count else 0
    wheel_side = sorted(
        {
            f"{wheelset}-{side}"
            for wheelset, side in zip(dataset.wheelset, dataset.side, strict=True)
        }
    )
    monotonic = True
    for seed in np.unique(dataset.irregularity_seed):
        for wheelset in np.unique(dataset.wheelset[dataset.irregularity_seed == seed]):
            for side in ("L", "R"):
                indexes = np.flatnonzero(
                    (dataset.irregularity_seed == seed)
                    & (dataset.wheelset == wheelset)
                    & (dataset.side == side)
                )
                if indexes.size <= 1:
                    continue
                order = np.argsort(dataset.step_index[indexes], kind="stable")
                selected = indexes[order]
                if (
                    np.any(np.diff(dataset.step_index[selected]) < 0)
                    or np.any(np.diff(dataset.time_s[selected]) < -1.0e-12)
                    or np.any(np.diff(dataset.front_mileage_m[selected]) < -1.0e-9)
                    or np.any(np.diff(dataset.actual_mileage_m[selected]) < -1.0e-9)
                ):
                    monotonic = False
                    break
    issues: list[str] = []
    if dataset.features.shape != (count, len(dataset.feature_names)):
        issues.append("feature shape does not match feature schema")
    if dataset.targets.shape != (count, len(TARGET_NAMES)):
        issues.append("target shape does not match target schema")
    if any(length != count for length in row_lengths.values()):
        issues.append("metadata arrays have inconsistent row counts")
    if not finite:
        issues.append("dataset contains NaN or Inf")
    if unique_rows != count:
        issues.append("dataset contains duplicate patch keys")
    if np.any(dataset.targets[:, 0] < 0.0):
        issues.append("normal-force labels contain negative values")
    if np.any(dataset.patch_count < 1) or np.any(dataset.patch_count > 2):
        issues.append("patch_count is outside the supported range 1..2")
    if np.any(dataset.patch_index < 0) or np.any(dataset.patch_index >= dataset.patch_count):
        issues.append("patch_index is inconsistent with patch_count")
    if not np.all(dataset.accepted_step_label):
        issues.append("formal dataset contains non-accepted-step rows")
    if not monotonic:
        issues.append("time or mileage is not monotonic within a wheel/side trajectory")
    return {
        "valid": not issues,
        "issues": issues,
        "rows": count,
        "feature_count": int(dataset.features.shape[1]) if dataset.features.ndim == 2 else None,
        "target_count": int(dataset.targets.shape[1]) if dataset.targets.ndim == 2 else None,
        "feature_schema": list(dataset.feature_names),
        "seed_rows": {
            str(int(seed)): int(np.count_nonzero(dataset.irregularity_seed == seed))
            for seed in np.unique(dataset.irregularity_seed)
        },
        "wheel_side_coverage": wheel_side,
        "unique_patch_keys": unique_rows,
        "finite": finite,
        "monotonic": monotonic,
        "accepted_step_labels_only": bool(np.all(dataset.accepted_step_label)),
        "patch_count_distribution": {
            str(int(value)): int(np.count_nonzero(dataset.patch_count == value))
            for value in np.unique(dataset.patch_count)
        },
        "front_mileage_m": {
            "minimum": None if count == 0 else float(np.min(dataset.front_mileage_m)),
            "maximum": None if count == 0 else float(np.max(dataset.front_mileage_m)),
        },
        "teacher_match_distance_m": {
            "maximum": (
                None if count == 0 else float(np.max(dataset.teacher_match_distance_m))
            ),
            "p99": (
                None
                if count == 0
                else float(np.quantile(dataset.teacher_match_distance_m, 0.99))
            ),
        },
    }


def validate_network_b_dataset(
    dataset: NetworkBDataset,
    *,
    expected_seed: int | None = None,
    require_direct_schema: bool = False,
    require_all_wheel_sides: bool = False,
) -> dict[str, object]:
    report = network_b_dataset_quality_report(dataset)
    issues = list(report["issues"])
    seeds = set(int(value) for value in np.unique(dataset.irregularity_seed))
    if expected_seed is not None and seeds != {int(expected_seed)}:
        issues.append(
            f"dataset seeds {sorted(seeds)} do not match expected seed {int(expected_seed)}"
        )
    if require_direct_schema and dataset.feature_names != DIRECT_FEATURE_NAMES:
        issues.append("dataset does not use the Direct Network-B feature schema")
    expected_wheel_sides = {
        f"{wheelset}-{side}"
        for wheelset in ("FF", "FR", "RF", "RR")
        for side in ("L", "R")
    }
    coverage = set(str(value) for value in report["wheel_side_coverage"])
    if require_all_wheel_sides and coverage != expected_wheel_sides:
        issues.append(
            "wheel/side coverage is incomplete: "
            f"missing {sorted(expected_wheel_sides - coverage)}"
        )
    report["issues"] = issues
    report["valid"] = not issues
    if issues:
        raise ValueError("invalid Network-B dataset: " + "; ".join(issues))
    return report


def _payload_array(
    payload: Any,
    name: str,
    count: int,
    *,
    dtype: Any,
    default: float | int | bool,
) -> np.ndarray:
    if name in payload.files:
        values = np.asarray(payload[name], dtype=dtype)
        if values.shape != (count,):
            raise ValueError(f"network-B dataset field {name} has an invalid shape")
        return values
    return np.full((count,), default, dtype=dtype)


def _network_b_row_keys(dataset: NetworkBDataset) -> np.ndarray:
    return np.asarray(
        [
            (
                f"{int(seed)}:{int(step)}:{wheelset}:{side}:"
                f"{int(patch_index)}"
            )
            for seed, step, wheelset, side, patch_index in zip(
                dataset.irregularity_seed,
                dataset.step_index,
                dataset.wheelset,
                dataset.side,
                dataset.patch_index,
                strict=True,
            )
        ],
        dtype="U64",
    )


def _deduplicate_network_b_dataset(dataset: NetworkBDataset) -> NetworkBDataset:
    if len(dataset) <= 1:
        return dataset
    _, indexes = np.unique(_network_b_row_keys(dataset), return_index=True)
    if indexes.size == len(dataset):
        return dataset
    return dataset.subset(np.sort(indexes))
