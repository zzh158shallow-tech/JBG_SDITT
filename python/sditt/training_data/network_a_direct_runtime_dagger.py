from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from sditt.models.network_a1_direct_full_case import direct_geometry_from_traditional
from sditt.models.wrcp_net_a1_direct import HISTORY_NAMES, MAX_PATCHES, TARGET_NAMES
from sditt.training_data.network_a import (
    BOUNDARY_PENETRATION_M,
    build_network_a_teacher_context,
    teacher_geometry_from_features,
)
from sditt.training_data.network_a_direct import (
    NetworkA1DirectDataset,
    direct_patch_to_training_target,
    ensure_supervision_metadata,
    load_direct_dataset,
    save_direct_dataset,
    validate_direct_dataset,
)
from sditt.training_data.network_a_direct_fresh import concatenate_direct_datasets


RUNTIME_DAGGER_SCHEMA = "network-a-direct-runtime-dagger-v1"


def build_runtime_dagger_dataset(
    *,
    base_dataset_dir: str | Path,
    trace_dirs: Sequence[str | Path],
    output_dir: str | Path,
    repo_root: str | Path | None,
    maximum_step_index: int | None = None,
    minimum_step_index: int | None = None,
    allow_incomplete: bool = False,
    iteration_policy: str = "all",
    topology_reference_tolerance_m: float = 0.02,
) -> Path:
    if not trace_dirs:
        raise ValueError("at least one runtime trace directory is required")
    if iteration_policy not in {"all", "first"}:
        raise ValueError("iteration policy must be 'all' or 'first'")
    if topology_reference_tolerance_m <= 0.0:
        raise ValueError("topology reference tolerance must be positive")
    base_dir = Path(base_dataset_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    train = ensure_supervision_metadata(load_direct_dataset(base_dir / "train.npz"))
    validation = load_direct_dataset(base_dir / "validation.npz")
    test = load_direct_dataset(base_dir / "test.npz")
    context = build_network_a_teacher_context(repo_root)

    rows: list[dict[str, Any]] = []
    trace_records: list[dict[str, Any]] = []
    seen: set[tuple[str, int, str, str, int, int]] = set()
    for trace_value in trace_dirs:
        trace_dir = Path(trace_value)
        manifest_path = trace_dir / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("schema") != "network-a1-direct-runtime-trace-v1":
            raise ValueError(f"unsupported runtime trace schema: {trace_dir}")
        if not manifest.get("complete", False) and not allow_incomplete:
            raise ValueError(f"runtime trace is incomplete: {trace_dir}")
        irregularity = manifest.get("metadata", {}).get("track_irregularity", {})
        seed = int(irregularity.get("seed", -1))
        iteration_path = trace_dir / manifest["contents"]["iterations"]
        accepted = 0
        for line in iteration_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            record = json.loads(line)
            runtime = record["context"]
            if runtime.get("stage") != "Cal":
                continue
            if maximum_step_index is not None and int(runtime["step_index"]) > maximum_step_index:
                continue
            if minimum_step_index is not None and int(runtime["step_index"]) < minimum_step_index:
                continue
            if iteration_policy == "first" and int(runtime["iteration"]) != 1:
                continue
            key = (
                str(trace_dir.resolve()),
                seed,
                str(runtime["wheelset"]),
                str(runtime["side"]),
                int(runtime["step_index"]),
                int(runtime["iteration"]),
            )
            if key in seen:
                continue
            seen.add(key)
            rows.append({"trace": record, "seed": seed, "trace_dir": str(trace_dir.resolve())})
            accepted += 1
        trace_records.append(
            {
                "trace_dir": str(trace_dir.resolve()),
                "manifest_sha256": _sha256(manifest_path),
                "model_sha256": manifest.get("model_sha256"),
                "trace_complete": bool(manifest.get("complete", False)),
                "irregularity_seed": seed,
                "iteration_rows": accepted,
            }
        )

    topology_reference_path = base_dir / "accepted_topology_reference.npz"
    topology_reference = (
        _load_accepted_topology_reference(topology_reference_path)
        if topology_reference_path.exists()
        else _accepted_topology_reference(train)
    )
    runtime_candidates, label_records = _label_runtime_rows(
        rows,
        train,
        context,
        topology_reference_tolerance_m=topology_reference_tolerance_m,
        topology_reference=topology_reference,
    )
    eligible = (
        np.asarray(runtime_candidates.metadata["teacher_relabel"], dtype=bool)
        & np.asarray(
            runtime_candidates.metadata["topology_reference_available"], dtype=bool
        )
        & np.asarray(
            runtime_candidates.metadata["topology_matches_accepted_trajectory"], dtype=bool
        )
        & (runtime_candidates.patch_count > 0)
    )
    runtime_dataset = runtime_candidates.subset(np.flatnonzero(eligible))
    augmented = (
        concatenate_direct_datasets((train, runtime_dataset))
        if len(runtime_dataset)
        else train
    )
    validate_direct_dataset(augmented)
    _validate_group_isolation(augmented, validation, test)
    save_direct_dataset(destination / "train.npz", augmented)
    save_direct_dataset(destination / "validation.npz", validation)
    save_direct_dataset(destination / "test.npz", test)
    label_path = destination / "runtime_dagger_labels.jsonl"
    label_path.write_text(
        "".join(json.dumps(record, ensure_ascii=False) + "\n" for record in label_records),
        encoding="utf-8",
    )

    seeds = sorted(set(int(row["seed"]) for row in rows))
    validation_seeds = set(int(value) for value in validation.metadata["irregularity_seed"] if value >= 0)
    test_seeds = set(int(value) for value in test.metadata["irregularity_seed"] if value >= 0)
    manifest = {
        "schema": RUNTIME_DAGGER_SCHEMA,
        "base_dataset_dir": str(base_dir.resolve()),
        "repo_root": str(context.paths.root.resolve()),
        "source_kind": (
            "Cal nonlinear-iteration local supervision; these rows are not accepted-step trajectory labels"
        ),
        "teacher": "current traditional final-patch geometry replayed from each six-dimensional state",
        "history": "exact A1 runtime trace history, including final traditional Preload anchor",
        "maximum_step_index": maximum_step_index,
        "minimum_step_index": minimum_step_index,
        "allow_incomplete": bool(allow_incomplete),
        "iteration_policy": iteration_policy,
        "topology_reference_tolerance_m": float(topology_reference_tolerance_m),
        "topology_reference": {
            "path": str(topology_reference_path.resolve()) if topology_reference_path.exists() else None,
            "kind": "full accepted-step index" if topology_reference_path.exists() else "selected training rows",
            "sha256": _sha256(topology_reference_path) if topology_reference_path.exists() else None,
        },
        "irregularity_seeds": seeds,
        "base_train_samples": len(train),
        "runtime_dagger_candidates": len(runtime_candidates),
        "runtime_dagger_samples": len(runtime_dataset),
        "augmented_train_samples": len(augmented),
        "topology_counts": {
            str(value): int(np.sum(runtime_dataset.patch_count == value)) for value in range(3)
        },
        "geometry_supervision": {
            "teacher_relabelled_candidates": int(np.sum(
                runtime_candidates.metadata["teacher_relabel"]
            )),
            "reference_available_candidates": int(np.sum(
                runtime_candidates.metadata["topology_reference_available"]
            )),
            "topology_matches_accepted_trajectory_candidates": int(np.sum(
                runtime_candidates.metadata["topology_matches_accepted_trajectory"]
            )),
            "eligible_local_geometry_rows": len(runtime_dataset),
            "teacher_relabelled": int(np.sum(runtime_dataset.metadata["teacher_relabel"])),
            "reference_available": int(np.sum(
                runtime_dataset.metadata["topology_reference_available"]
            )),
            "topology_matches_accepted_trajectory": int(np.sum(
                runtime_dataset.metadata["topology_matches_accepted_trajectory"]
            )),
            "accepted_step_labels": int(np.sum(
                runtime_dataset.metadata["accepted_step_label"]
            )),
        },
        "wheel_side_counts": {
            f"{wheel}-{side}": int(np.sum(
                (runtime_dataset.metadata["wheelset"] == wheel)
                & (runtime_dataset.metadata["side"] == side)
            ))
            for wheel in ("FF", "FR", "RF", "RR")
            for side in ("L", "R")
        },
        "split_isolation": {
            "validation_seeds": sorted(validation_seeds),
            "test_seeds": sorted(test_seeds),
            "overlap": sorted(set(seeds) & (validation_seeds | test_seeds)),
        },
        "traces": trace_records,
        "files": {
            "train": "train.npz",
            "validation": "validation.npz",
            "test": "test.npz",
            "labels": label_path.name,
        },
    }
    if manifest["split_isolation"]["overlap"]:
        raise ValueError("runtime DAgger seed leakage detected")
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def _label_runtime_rows(
    rows: list[dict[str, Any]],
    base: NetworkA1DirectDataset,
    teacher_context: Any,
    *,
    topology_reference_tolerance_m: float,
    topology_reference: dict[tuple[int, str, str], tuple[np.ndarray, np.ndarray]] | None = None,
) -> tuple[NetworkA1DirectDataset, list[dict[str, Any]]]:
    base = ensure_supervision_metadata(base)
    if topology_reference is None:
        topology_reference = _accepted_topology_reference(base)
    n = len(rows)
    features = np.zeros((n, 6), dtype=float)
    history = np.zeros((n, MAX_PATCHES, len(HISTORY_NAMES)), dtype=float)
    history_mask = np.zeros((n, MAX_PATCHES), dtype=bool)
    patch_count = np.zeros((n,), dtype=np.int8)
    patch_mask = np.zeros((n, MAX_PATCHES), dtype=bool)
    targets = np.zeros((n, MAX_PATCHES, len(TARGET_NAMES)), dtype=float)
    metadata_lists: dict[str, list[Any]] = {key: [] for key in base.metadata}
    label_records: list[dict[str, Any]] = []

    for index, item in enumerate(rows):
        trace = item["trace"]
        runtime = trace["context"]
        values = np.asarray(trace["features"], dtype=float)
        traditional = teacher_geometry_from_features(teacher_context, values)
        direct = direct_geometry_from_traditional(traditional)
        patches = tuple(sorted(direct.patches, key=lambda patch: patch.corrected_rail_point[0]))
        features[index] = values
        history[index] = np.asarray(trace["history"], dtype=float)
        history_mask[index] = np.asarray(trace["history_mask"], dtype=bool)
        patch_count[index] = len(patches)
        for patch_index, patch in enumerate(patches):
            patch_mask[index, patch_index] = True
            targets[index, patch_index] = direct_patch_to_training_target(patch)
        maximum_penetration = max(
            (patch.corrected_vertical_penetration for patch in patches),
            default=0.0,
        )
        sample_class = (
            "no_contact"
            if not patches
            else "boundary" if maximum_penetration <= BOUNDARY_PENETRATION_M else "normal"
        )
        seed = int(item["seed"])
        reference_available, topology_matches, reference_count, reference_distance = (
            _match_accepted_topology(
                topology_reference,
                seed=seed,
                wheelset=str(runtime["wheelset"]),
                side=str(runtime["side"]),
                actual_mileage_m=float(runtime.get("actual_mileage_m", 0.0)),
                teacher_patch_count=len(patches),
                tolerance_m=topology_reference_tolerance_m,
            )
        )
        group_id = f"runtime-dagger-seed-{seed}"
        sample_id = (
            f"runtime-dagger:{seed}:{runtime['wheelset']}:{runtime['side']}:"
            f"{runtime['step_index']}:{runtime['iteration']}"
        )
        values_by_key: dict[str, Any] = {
            "source": "dagger_runtime_iteration",
            "sample_class": sample_class,
            "sample_id": sample_id,
            "group_id": group_id,
            "front_mileage_m": float(runtime.get("front_mileage_m", 0.0)),
            "actual_mileage_m": float(runtime.get("actual_mileage_m", 0.0)),
            "wheelset": str(runtime["wheelset"]),
            "side": str(runtime["side"]),
            "stage": "Cal",
            "step_index": int(runtime["step_index"]),
            "iteration": int(runtime["iteration"]),
            "time_s": float(runtime["time_s"]),
            "dt_s": float(runtime["dt_s"]),
            "converged": False,
            "irregularity_model": "china-ballastless",
            "irregularity_seed": seed,
            "accepted_step_label": False,
            "teacher_relabel": True,
            "topology_reference_available": reference_available,
            "topology_matches_accepted_trajectory": topology_matches,
        }
        for key in metadata_lists:
            metadata_lists[key].append(values_by_key[key])
        label_records.append(
            {
                "sample_id": sample_id,
                "trace_dir": item["trace_dir"],
                "teacher_patch_count": len(patches),
                "runtime_topology_probability": trace.get("topology_probability"),
                "runtime_feature_distance": trace.get("feature_distance"),
                "runtime_history_distance": trace.get("history_distance"),
                "accepted_step_label": False,
                "teacher_relabel": True,
                "topology_reference_available": reference_available,
                "accepted_topology_patch_count": reference_count,
                "accepted_topology_distance_m": reference_distance,
                "topology_matches_accepted_trajectory": topology_matches,
                "used_for_local_geometry_regression": bool(
                    reference_available and topology_matches and len(patches) > 0
                ),
                "formal_topology_weight": 0.0,
                "local_geometry_weight": (
                    1.0 if reference_available and topology_matches and len(patches) > 0 else 0.0
                ),
            }
        )

    metadata = {}
    for key, values in metadata_lists.items():
        base_dtype = np.asarray(base.metadata[key]).dtype
        metadata[key] = np.asarray(
            values,
            dtype=str if base_dtype.kind in {"U", "S"} else base_dtype,
        )
    result = NetworkA1DirectDataset(
        features=features,
        history=history,
        history_mask=history_mask,
        patch_count=patch_count,
        patch_mask=patch_mask,
        targets=targets,
        previous_row=np.full((n,), -1, dtype=np.int64),
        metadata=metadata,
    )
    validate_direct_dataset(result)
    return result, label_records


def _accepted_topology_reference(
    dataset: NetworkA1DirectDataset,
) -> dict[tuple[int, str, str], tuple[np.ndarray, np.ndarray]]:
    metadata = dataset.metadata
    source = np.asarray(metadata["source"], dtype=str)
    accepted = np.asarray(metadata["accepted_step_label"], dtype=bool)
    stage = np.asarray(metadata.get("stage", np.full((len(dataset),), "")), dtype=str)
    seed = np.asarray(metadata.get("irregularity_seed", np.full((len(dataset),), -1)), dtype=int)
    wheelset = np.asarray(metadata.get("wheelset", np.full((len(dataset),), "")), dtype=str)
    side = np.asarray(metadata.get("side", np.full((len(dataset),), "")), dtype=str)
    mileage = np.asarray(
        metadata.get("actual_mileage_m", np.zeros((len(dataset),))), dtype=float
    )
    eligible = accepted & (source == "full_cal_accepted") & (stage == "Cal") & (seed >= 0)
    reference: dict[tuple[int, str, str], tuple[np.ndarray, np.ndarray]] = {}
    for key in sorted(set(zip(seed[eligible], wheelset[eligible], side[eligible], strict=True))):
        indexes = np.flatnonzero(
            eligible
            & (seed == key[0])
            & (wheelset == key[1])
            & (side == key[2])
        )
        order = np.argsort(mileage[indexes], kind="stable")
        reference[(int(key[0]), str(key[1]), str(key[2]))] = (
            mileage[indexes][order],
            np.asarray(dataset.patch_count[indexes][order], dtype=np.int8),
        )
    return reference


def _load_accepted_topology_reference(
    path: str | Path,
) -> dict[tuple[int, str, str], tuple[np.ndarray, np.ndarray]]:
    with np.load(path, allow_pickle=False) as payload:
        if str(payload["schema"].item()) != "network-a1-accepted-topology-reference-v1":
            raise ValueError("unsupported accepted topology reference schema")
        seed = np.asarray(payload["irregularity_seed"], dtype=int)
        wheelset = np.asarray(payload["wheelset"], dtype=str)
        side = np.asarray(payload["side"], dtype=str)
        mileage = np.asarray(payload["actual_mileage_m"], dtype=float)
        patch_count = np.asarray(payload["patch_count"], dtype=np.int8)
    reference: dict[tuple[int, str, str], tuple[np.ndarray, np.ndarray]] = {}
    for key in sorted(set(zip(seed, wheelset, side, strict=True))):
        indexes = np.flatnonzero(
            (seed == key[0]) & (wheelset == key[1]) & (side == key[2])
        )
        order = np.argsort(mileage[indexes], kind="stable")
        reference[(int(key[0]), str(key[1]), str(key[2]))] = (
            mileage[indexes][order],
            patch_count[indexes][order],
        )
    return reference


def _match_accepted_topology(
    reference: dict[tuple[int, str, str], tuple[np.ndarray, np.ndarray]],
    *,
    seed: int,
    wheelset: str,
    side: str,
    actual_mileage_m: float,
    teacher_patch_count: int,
    tolerance_m: float,
) -> tuple[bool, bool, int | None, float | None]:
    values = reference.get((int(seed), str(wheelset), str(side)))
    if values is None:
        return False, False, None, None
    mileages, counts = values
    insertion = int(np.searchsorted(mileages, actual_mileage_m))
    candidates = [index for index in (insertion - 1, insertion) if 0 <= index < mileages.size]
    nearest = min(candidates, key=lambda index: abs(float(mileages[index]) - actual_mileage_m))
    distance = abs(float(mileages[nearest]) - float(actual_mileage_m))
    if distance > tolerance_m:
        return False, False, None, distance
    reference_count = int(counts[nearest])
    return True, reference_count == int(teacher_patch_count), reference_count, distance


def _validate_group_isolation(
    train: NetworkA1DirectDataset,
    validation: NetworkA1DirectDataset,
    test: NetworkA1DirectDataset,
) -> None:
    groups = {
        name: set(np.asarray(dataset.metadata["group_id"], dtype=str))
        for name, dataset in {"train": train, "validation": validation, "test": test}.items()
    }
    for left, right in (("train", "validation"), ("train", "test"), ("validation", "test")):
        overlap = groups[left] & groups[right]
        if overlap:
            raise ValueError(f"runtime DAgger group leakage: {left}/{right} {sorted(overlap)[:3]}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Teacher-label A1 Direct runtime iteration traces for local DAgger supervision."
    )
    parser.add_argument("--base-dataset-dir", type=Path, required=True)
    parser.add_argument("--trace-dir", type=Path, action="append", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=Path(".."))
    parser.add_argument("--maximum-step-index", type=int, default=None)
    parser.add_argument("--minimum-step-index", type=int, default=None)
    parser.add_argument("--iteration-policy", choices=("all", "first"), default="all")
    parser.add_argument("--topology-reference-tolerance-m", type=float, default=0.02)
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="accept fail-fast traces and retain their pre-failure and failure states",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    manifest = build_runtime_dagger_dataset(
        base_dataset_dir=args.base_dataset_dir,
        trace_dirs=args.trace_dir,
        output_dir=args.output_dir,
        repo_root=args.repo_root,
        maximum_step_index=args.maximum_step_index,
        minimum_step_index=args.minimum_step_index,
        allow_incomplete=args.allow_incomplete,
        iteration_policy=args.iteration_policy,
        topology_reference_tolerance_m=args.topology_reference_tolerance_m,
    )
    print(f"wrote {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
