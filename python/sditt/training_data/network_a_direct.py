from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from sditt.models.network_a1_direct_full_case import direct_geometry_from_traditional
from sditt.models.wrcp_net_a1_direct import (
    FEATURE_NAMES,
    HISTORY_NAMES,
    MAX_PATCHES,
    TARGET_NAMES,
    geometry_to_history,
)
from sditt.training_data.network_a import (
    METADATA_FIELDS,
    NetworkADataset,
    build_network_a_teacher_context,
    load_network_a_dataset,
    teacher_geometry_from_features,
)


SCHEMA_VERSION = "network-a-direct-set-dataset-v1"
SUPERVISION_METADATA_FIELDS = (
    "accepted_step_label",
    "teacher_relabel",
    "topology_reference_available",
    "topology_matches_accepted_trajectory",
)


def dagger_source_mask(source: np.ndarray) -> np.ndarray:
    values = np.char.lower(np.asarray(source, dtype=str))
    return np.char.find(values, "dagger") >= 0


@dataclass(frozen=True)
class NetworkA1DirectDataset:
    features: np.ndarray
    history: np.ndarray
    history_mask: np.ndarray
    patch_count: np.ndarray
    patch_mask: np.ndarray
    targets: np.ndarray
    previous_row: np.ndarray
    metadata: dict[str, np.ndarray]

    def __len__(self) -> int:
        return int(self.features.shape[0])

    def subset(self, indexes: np.ndarray) -> "NetworkA1DirectDataset":
        selection = np.asarray(indexes)
        row_map = {int(old): new for new, old in enumerate(selection)}
        previous_row = np.array(
            [row_map.get(int(self.previous_row[old]), -1) for old in selection],
            dtype=np.int64,
        )
        return NetworkA1DirectDataset(
            features=self.features[selection].copy(),
            history=self.history[selection].copy(),
            history_mask=self.history_mask[selection].copy(),
            patch_count=self.patch_count[selection].copy(),
            patch_mask=self.patch_mask[selection].copy(),
            targets=self.targets[selection].copy(),
            previous_row=previous_row,
            metadata={key: value[selection].copy() for key, value in self.metadata.items()},
        )


def ensure_supervision_metadata(dataset: NetworkA1DirectDataset) -> NetworkA1DirectDataset:
    """Add explicit supervision provenance to legacy and newly built datasets."""

    n = len(dataset)
    metadata = {key: np.asarray(value).copy() for key, value in dataset.metadata.items()}
    source = np.asarray(metadata.get("source", np.full((n,), "")), dtype=str)
    converged = np.asarray(metadata.get("converged", np.ones((n,), dtype=bool)), dtype=bool)
    dagger = dagger_source_mask(source)
    inferred_accepted = converged & (source != "sobol") & ~dagger
    defaults = {
        "accepted_step_label": inferred_accepted,
        "teacher_relabel": source == "dagger_runtime_iteration",
        "topology_reference_available": inferred_accepted,
        "topology_matches_accepted_trajectory": inferred_accepted,
    }
    changed = False
    for key, values in defaults.items():
        if key not in metadata:
            metadata[key] = np.asarray(values, dtype=bool)
            changed = True
        else:
            metadata[key] = np.asarray(metadata[key], dtype=bool)
    if not changed and all(dataset.metadata[key].dtype == bool for key in defaults):
        return dataset
    return NetworkA1DirectDataset(
        features=dataset.features,
        history=dataset.history,
        history_mask=dataset.history_mask,
        patch_count=dataset.patch_count,
        patch_mask=dataset.patch_mask,
        targets=dataset.targets,
        previous_row=dataset.previous_row,
        metadata=metadata,
    )
def build_direct_dataset(
    source: NetworkADataset,
    *,
    repo_root: str | Path | None = None,
    indexes: np.ndarray | None = None,
) -> NetworkA1DirectDataset:
    """Replay the traditional teacher and create final-patch direct labels."""

    selected = np.arange(len(source), dtype=int) if indexes is None else np.asarray(indexes, dtype=int)
    context = build_network_a_teacher_context(repo_root)
    previous_indexes = _previous_accepted_indexes(source)
    required = set(int(index) for index in selected)
    required.update(int(previous_indexes[index]) for index in selected if previous_indexes[index] >= 0)
    cache = {}
    for index in sorted(required):
        traditional = teacher_geometry_from_features(context, source.features[index])
        cache[index] = direct_geometry_from_traditional(traditional)

    n = selected.size
    history = np.zeros((n, MAX_PATCHES, len(HISTORY_NAMES)), dtype=float)
    history_mask = np.zeros((n, MAX_PATCHES), dtype=bool)
    targets = np.zeros((n, MAX_PATCHES, len(TARGET_NAMES)), dtype=float)
    patch_mask = np.zeros((n, MAX_PATCHES), dtype=bool)
    patch_count = np.zeros((n,), dtype=np.int8)
    direct_row_by_source = {int(source_index): row for row, source_index in enumerate(selected)}
    previous_row_values: list[int] = []
    for row, source_index in enumerate(selected):
        candidate = direct_row_by_source.get(int(previous_indexes[source_index]), -1)
        previous_row_values.append(candidate if 0 <= candidate < row else -1)
    previous_row = np.asarray(previous_row_values, dtype=np.int64)
    for row, source_index in enumerate(selected):
        geometry = cache[int(source_index)]
        patches = tuple(sorted(geometry.patches, key=lambda patch: patch.corrected_rail_point[0]))
        patch_count[row] = len(patches)
        for patch_index, patch in enumerate(patches):
            patch_mask[row, patch_index] = True
            targets[row, patch_index] = direct_patch_to_training_target(patch)
        previous = int(previous_indexes[source_index])
        if previous >= 0:
            history[row], history_mask[row] = geometry_to_history(cache[previous])

    dataset = ensure_supervision_metadata(NetworkA1DirectDataset(
        features=np.asarray(source.features[selected], dtype=float),
        history=history,
        history_mask=history_mask,
        patch_count=patch_count,
        patch_mask=patch_mask,
        targets=targets,
        previous_row=previous_row,
        metadata={key: np.asarray(value[selected]).copy() for key, value in source.metadata.items()},
    ))
    validate_direct_dataset(dataset)
    return dataset


def direct_patch_to_training_target(patch: object) -> np.ndarray:
    start = float(patch.start_y)
    end = float(patch.end_y)
    center = float(patch.corrected_rail_point[0])
    width = max(end - start, 2.0e-12)
    peak_fraction = np.clip((float(patch.peak_rail_point[0]) - start) / width, 1.0e-7, 1.0 - 1.0e-7)
    moments = np.clip(np.asarray(patch.shape_moments, dtype=float), 1.0e-7, 1.0 - 1.0e-7)
    peak_increment = max(
        float(patch.peak_vertical_penetration - patch.corrected_vertical_penetration),
        1.0e-12,
    )
    return np.array(
        [
            center,
            _softplus_inverse(max(center - start, 1.0e-12)),
            _softplus_inverse(max(end - center, 1.0e-12)),
            float(patch.corrected_wheel_point[0]),
            float(patch.corrected_rail_point[1]),
            float(patch.wheel_profile_lateral),
            _softplus_inverse(max(float(patch.corrected_vertical_penetration), 1.0e-12)),
            float(patch.contact_angle),
            _logit(peak_fraction),
            float(patch.peak_wheel_point[0]),
            float(patch.peak_rail_point[1]),
            _softplus_inverse(peak_increment),
            float(patch.peak_contact_angle),
            *(_logit(value) for value in moments),
        ],
        dtype=float,
    )


def validate_direct_dataset(dataset: NetworkA1DirectDataset) -> None:
    dataset = ensure_supervision_metadata(dataset)
    n = len(dataset)
    expected = {
        "features": (n, len(FEATURE_NAMES)),
        "history": (n, MAX_PATCHES, len(HISTORY_NAMES)),
        "history_mask": (n, MAX_PATCHES),
        "patch_count": (n,),
        "patch_mask": (n, MAX_PATCHES),
        "targets": (n, MAX_PATCHES, len(TARGET_NAMES)),
        "previous_row": (n,),
    }
    for name, shape in expected.items():
        if np.asarray(getattr(dataset, name)).shape != shape:
            raise ValueError(f"network-A1 Direct {name} has the wrong shape")
    if not np.isfinite(dataset.features).all():
        raise ValueError("network-A1 Direct features contain NaN or Inf")
    if not np.isfinite(dataset.history[dataset.history_mask]).all():
        raise ValueError("network-A1 Direct history contains NaN or Inf")
    if not np.isfinite(dataset.targets[dataset.patch_mask]).all():
        raise ValueError("network-A1 Direct targets contain NaN or Inf")
    if np.any(dataset.patch_count != np.sum(dataset.patch_mask, axis=1)):
        raise ValueError("network-A1 Direct patch count and mask disagree")
    if np.any(dataset.targets[~dataset.patch_mask] != 0.0):
        raise ValueError("network-A1 Direct inactive targets are not zero-filled")
    if np.any((dataset.previous_row < -1) | (dataset.previous_row >= n)):
        raise ValueError("network-A1 Direct previous-row reference is out of range")
    if np.any(dataset.previous_row >= np.arange(n)):
        raise ValueError("network-A1 Direct previous-row reference must point backward")
    sources = np.asarray(dataset.metadata.get("source", np.full((n,), "")), dtype=str)
    if np.any((sources == "sobol") & np.any(dataset.history_mask, axis=1)):
        raise ValueError("Sobol samples must not carry accepted-step history")
    for key in SUPERVISION_METADATA_FIELDS:
        if np.asarray(dataset.metadata[key]).shape != (n,):
            raise ValueError(f"network-A1 Direct {key} metadata has the wrong shape")
    accepted = np.asarray(dataset.metadata["accepted_step_label"], dtype=bool)
    teacher_relabel = np.asarray(dataset.metadata["teacher_relabel"], dtype=bool)
    topology_reference = np.asarray(
        dataset.metadata["topology_reference_available"], dtype=bool
    )
    topology_match = np.asarray(
        dataset.metadata["topology_matches_accepted_trajectory"], dtype=bool
    )
    dagger = dagger_source_mask(sources)
    if np.any(dagger & accepted):
        raise ValueError("DAgger rows cannot be accepted-step trajectory labels")
    if np.any(dagger & topology_match & ~(teacher_relabel & topology_reference)):
        raise ValueError(
            "runtime DAgger geometry supervision requires teacher relabel and topology reference"
        )


def stratified_pilot_indexes(
    dataset: NetworkADataset,
    count: int,
    *,
    seed: int = 20260721,
) -> np.ndarray:
    if count <= 0 or count > len(dataset):
        raise ValueError("pilot count must be positive and no larger than the source dataset")
    rng = np.random.default_rng(seed)
    desired = np.array([0.15, 0.75, 0.10]) * count
    quotas = np.floor(desired).astype(int)
    quotas[1] += count - int(np.sum(quotas))
    selected: list[np.ndarray] = []
    remaining = count
    for patch_count in (0, 2, 1):
        candidates = np.flatnonzero(dataset.patch_count == patch_count)
        take = min(int(quotas[patch_count]), candidates.size, remaining)
        if take:
            selected.append(rng.choice(candidates, size=take, replace=False))
            remaining -= take
    if remaining:
        used = np.concatenate(selected) if selected else np.zeros((0,), dtype=int)
        candidates = np.setdiff1d(np.arange(len(dataset)), used, assume_unique=False)
        selected.append(rng.choice(candidates, size=remaining, replace=False))
    return np.sort(np.concatenate(selected))


def save_direct_dataset(path: str | Path, dataset: NetworkA1DirectDataset) -> Path:
    dataset = ensure_supervision_metadata(dataset)
    validate_direct_dataset(dataset)
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, np.ndarray] = {
        "schema": np.array(SCHEMA_VERSION),
        "feature_names": np.asarray(FEATURE_NAMES),
        "history_names": np.asarray(HISTORY_NAMES),
        "target_names": np.asarray(TARGET_NAMES),
        "features": dataset.features,
        "history": dataset.history,
        "history_mask": dataset.history_mask,
        "patch_count": dataset.patch_count,
        "patch_mask": dataset.patch_mask,
        "targets": dataset.targets,
        "previous_row": dataset.previous_row,
    }
    payload.update({f"meta__{key}": value for key, value in dataset.metadata.items()})
    np.savez_compressed(output, **payload)
    return output


def load_direct_dataset(path: str | Path) -> NetworkA1DirectDataset:
    with np.load(path, allow_pickle=False) as payload:
        if str(payload["schema"].item()) != SCHEMA_VERSION:
            raise ValueError("unsupported network-A1 Direct dataset schema")
        if tuple(str(value) for value in payload["feature_names"]) != FEATURE_NAMES:
            raise ValueError("network-A1 Direct feature schema mismatch")
        if tuple(str(value) for value in payload["history_names"]) != HISTORY_NAMES:
            raise ValueError("network-A1 Direct history schema mismatch")
        if tuple(str(value) for value in payload["target_names"]) != TARGET_NAMES:
            raise ValueError("network-A1 Direct target schema mismatch")
        metadata = {
            key.removeprefix("meta__"): np.asarray(payload[key]).copy()
            for key in payload.files
            if key.startswith("meta__")
        }
        result = ensure_supervision_metadata(NetworkA1DirectDataset(
            features=np.asarray(payload["features"], dtype=float),
            history=np.asarray(payload["history"], dtype=float),
            history_mask=np.asarray(payload["history_mask"], dtype=bool),
            patch_count=np.asarray(payload["patch_count"], dtype=np.int8),
            patch_mask=np.asarray(payload["patch_mask"], dtype=bool),
            targets=np.asarray(payload["targets"], dtype=float),
            previous_row=np.asarray(payload["previous_row"], dtype=np.int64),
            metadata=metadata,
        ))
    validate_direct_dataset(result)
    return result


def generate_direct_dataset(
    *,
    source_dir: str | Path,
    output_dir: str | Path,
    repo_root: str | Path | None = None,
    mode: str = "pilot",
    seed: int = 20260721,
) -> Path:
    if mode not in {"pilot", "production"}:
        raise ValueError("mode must be 'pilot' or 'production'")
    counts = {"train": 1400, "validation": 300, "test": 300}
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, object] = {
        "schema": SCHEMA_VERSION,
        "mode": mode,
        "source_dir": str(Path(source_dir).resolve()),
        "history_rule": (
            "same trajectory/wheelset/side accepted steps ordered by "
            "Preload then Cal, with Cal-1 initialized from final Preload"
        ),
        "splits": {},
    }
    split_groups: dict[str, set[str]] = {}
    replay_samples = 0
    for offset, split in enumerate(("train", "validation", "test")):
        source = load_network_a_dataset(source_dir, split=split, dtype=np.float64)
        indexes = (
            stratified_pilot_indexes(source, min(counts[split], len(source)), seed=seed + offset)
            if mode == "pilot"
            else np.arange(len(source), dtype=int)
        )
        direct = build_direct_dataset(source, repo_root=repo_root, indexes=indexes)
        replay_samples += len(direct)
        split_groups[split] = set(np.asarray(direct.metadata["group_id"], dtype=str))
        path = save_direct_dataset(output / f"{split}.npz", direct)
        manifest["splits"][split] = {
            "path": path.name,
            "samples": len(direct),
            "topology_counts": {
                str(value): int(np.sum(direct.patch_count == value)) for value in range(3)
            },
            "history_valid": int(np.sum(np.any(direct.history_mask, axis=1))),
        }
    leakage: dict[str, list[str]] = {}
    split_names = tuple(split_groups)
    for left_index, left in enumerate(split_names):
        for right in split_names[left_index + 1 :]:
            overlap = sorted(split_groups[left] & split_groups[right])
            if overlap:
                leakage[f"{left}:{right}"] = overlap[:10]
    if leakage:
        raise ValueError(f"network-A1 Direct split group leakage: {leakage}")
    manifest["quality"] = {
        "passed": True,
        "teacher_replay_samples": int(replay_samples),
        "teacher_replay_failures": 0,
        "split_group_leakage": leakage,
        "note": "Every row was freshly replayed through the traditional final-patch teacher during construction.",
    }
    manifest_path = output / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return manifest_path


def _previous_accepted_indexes(dataset: NetworkADataset) -> np.ndarray:
    previous = np.full((len(dataset),), -1, dtype=int)
    source = np.asarray(dataset.metadata["source"], dtype=str)
    group = np.asarray(dataset.metadata["group_id"], dtype=str)
    wheelset = np.asarray(dataset.metadata["wheelset"], dtype=str)
    side = np.asarray(dataset.metadata["side"], dtype=str)
    step = np.asarray(dataset.metadata["step_index"], dtype=int)
    time = np.asarray(dataset.metadata["time_s"], dtype=float)
    stage = np.asarray(dataset.metadata["stage"], dtype=str)
    buckets: dict[tuple[str, str, str], list[int]] = {}
    for index in np.flatnonzero(source != "sobol"):
        buckets.setdefault((group[index], wheelset[index], side[index]), []).append(int(index))
    for indexes in buckets.values():
        # Stage-local time and step counters restart at Cal.  Sorting only by
        # time interleaves Preload-1, Cal-1, Preload-2, Cal-2 and initializes
        # Cal-1 from Preload-1 rather than the final accepted Preload state.
        stage_order = {"Preload": 0, "Cal": 1}
        ordered = sorted(
            indexes,
            key=lambda index: (
                stage_order.get(stage[index], 2),
                time[index],
                step[index],
                index,
            ),
        )
        for old, new in zip(ordered[:-1], ordered[1:], strict=True):
            previous[new] = old
    return previous


def _softplus_inverse(value: float) -> float:
    x = max(float(value), 1.0e-12)
    return float(x + np.log(-np.expm1(-x)))


def _logit(value: float) -> float:
    x = float(np.clip(value, 1.0e-7, 1.0 - 1.0e-7))
    return float(np.log(x) - np.log1p(-x))


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate WRCP-Net A1 Direct final-patch labels.")
    parser.add_argument("--source-dir", type=Path, default=Path("outputs/network_a_dataset_v1"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/network_a_direct_set_v1"))
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--mode", choices=("pilot", "production"), default="pilot")
    parser.add_argument("--seed", type=int, default=20260721)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    path = generate_direct_dataset(
        source_dir=args.source_dir,
        output_dir=args.output_dir,
        repo_root=args.repo_root,
        mode=args.mode,
        seed=args.seed,
    )
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
