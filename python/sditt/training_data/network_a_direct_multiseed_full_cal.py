from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Sequence

import numpy as np

from sditt.training_data.network_a import (
    NetworkADataset,
    load_network_a_dataset,
    save_network_a_archive,
)
from sditt.training_data.network_a_direct import (
    NetworkA1DirectDataset,
    build_direct_dataset,
    save_direct_dataset,
    validate_direct_dataset,
)
from sditt.training_data.network_a_direct_fresh import concatenate_direct_datasets
from sditt.training_data.network_a_direct_full_cal import (
    _collect_full_cal_source,
    _validate_replay_topology,
    final_preload_anchor_indexes,
    select_full_cal_indexes,
    validate_full_cal_source,
)


MULTISEED_FULL_CAL_SCHEMA = "network-a1-direct-multiseed-full-cal-v2"
DEFAULT_TRAINING_SEEDS = (
    20260716,
    20260717,
    20260718,
    20260719,
    20260720,
    20260723,
    20260724,
    20260725,
    20260726,
    20260727,
    20260728,
    20260729,
    20260730,
    20260731,
    20260732,
    20260733,
    20260734,
    20260735,
    20260736,
    20260737,
    20260738,
    20260739,
    20260740,
    20260741,
    20260742,
    20260743,
    20260744,
    20260745,
    20260746,
    20260747,
    20260748,
    20260749,
    20260757,
)
VALIDATION_SEED = 20260721
TEST_SEED = 20260722
DEFAULT_STAGE_END_MILEAGE_M = {"Preload": 47.6, "Cal": 130.0}


def prepare_multiseed_full_cal_dataset(
    *,
    output_dir: str | Path,
    repo_root: str | Path | None,
    training_seeds: Sequence[int] = DEFAULT_TRAINING_SEEDS,
    validation_seed: int = VALIDATION_SEED,
    test_seed: int = TEST_SEED,
    source_archives: dict[int, str | Path] | None = None,
    sampling_stride_steps: int = 64,
    rollout_window_steps: int = 8,
    transition_window_steps: int = 128,
    topology_context_steps: int = 4,
    cut_freq: float | None = None,
    stage_end_mileage: dict[str, float] | None = None,
) -> Path:
    """Build seed-isolated formal topology data from traditional accepted steps."""

    seeds = tuple(int(value) for value in training_seeds)
    if len(set(seeds)) < 2:
        raise ValueError("formal topology training requires at least two training seeds")
    if validation_seed == test_seed:
        raise ValueError("validation and test seeds must differ")
    forbidden = set(seeds) & {int(validation_seed), int(test_seed)}
    if forbidden:
        raise ValueError(f"training seeds overlap validation/test: {sorted(forbidden)}")
    if int(validation_seed) != VALIDATION_SEED or int(test_seed) != TEST_SEED:
        raise ValueError(
            f"locked evaluation seeds are validation={VALIDATION_SEED}, test={TEST_SEED}"
        )

    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    archives = {int(key): Path(value).resolve() for key, value in (source_archives or {}).items()}
    split_seeds = {
        "train": seeds,
        "validation": (int(validation_seed),),
        "test": (int(test_seed),),
    }
    parts: dict[str, list[NetworkA1DirectDataset]] = {name: [] for name in split_seeds}
    records: list[dict[str, object]] = []
    topology_reference_parts: dict[str, list[np.ndarray]] = {
        "irregularity_seed": [],
        "wheelset": [],
        "side": [],
        "actual_mileage_m": [],
        "patch_count": [],
    }

    for split, values in split_seeds.items():
        for seed in values:
            seed_dir = destination / "sources" / f"seed-{seed}"
            seed_dir.mkdir(parents=True, exist_ok=True)
            raw_path = seed_dir / "traditional_accepted_steps.npz"
            selected_path = seed_dir / "direct_selected.npz"
            source, provenance_path, reused = _load_or_collect_source(
                seed=seed,
                raw_path=raw_path,
                source_archive=archives.get(seed),
                repo_root=repo_root,
                cut_freq=cut_freq,
                stage_end_mileage=stage_end_mileage,
            )
            quality = validate_full_cal_source(source)
            _validate_stage_endpoints(source, stage_end_mileage)
            cal = np.asarray(source.metadata["stage"], dtype=str) == "Cal"
            topology_reference_parts["irregularity_seed"].append(
                np.asarray(source.metadata["irregularity_seed"], dtype=np.int64)[cal]
            )
            topology_reference_parts["wheelset"].append(
                np.asarray(source.metadata["wheelset"], dtype=str)[cal]
            )
            topology_reference_parts["side"].append(
                np.asarray(source.metadata["side"], dtype=str)[cal]
            )
            topology_reference_parts["actual_mileage_m"].append(
                np.asarray(source.metadata["actual_mileage_m"], dtype=float)[cal]
            )
            topology_reference_parts["patch_count"].append(
                np.asarray(source.patch_count, dtype=np.int8)[cal]
            )
            selected_indexes, selected_steps = select_full_cal_indexes(
                source,
                sampling_stride_steps=sampling_stride_steps,
                rollout_window_steps=rollout_window_steps,
                transition_window_steps=transition_window_steps,
                topology_context_steps=topology_context_steps,
            )
            selected_indexes = np.concatenate(
                (final_preload_anchor_indexes(source), selected_indexes)
            )
            direct = build_direct_dataset(source, repo_root=repo_root, indexes=selected_indexes)
            _validate_replay_topology(source, selected_indexes, direct)
            validate_direct_dataset(direct)
            save_direct_dataset(selected_path, direct)
            parts[split].append(direct)
            records.append(
                {
                    "split": split,
                    "seed": seed,
                    "source_archive": str(provenance_path),
                    "source_sha256": _sha256(provenance_path),
                    "source_reused": reused,
                    "accepted_cal_steps": quality.accepted_steps,
                    "accepted_cal_samples": quality.samples,
                    "selected_cal_steps": int(selected_steps.size),
                    "selected_samples": len(direct),
                    "selected_topology_counts": {
                        str(value): int(np.sum(direct.patch_count == value)) for value in range(3)
                    },
                    "front_mileage_start_m": quality.front_mileage_start_m,
                    "front_mileage_end_m": quality.front_mileage_end_m,
                    "selected_file": str(selected_path.relative_to(destination)),
                    "selected_sha256": _sha256(selected_path),
                }
            )

    datasets = {
        name: concatenate_direct_datasets(values)
        for name, values in parts.items()
    }
    _validate_locked_splits(datasets, seeds)
    for name, dataset in datasets.items():
        save_direct_dataset(destination / f"{name}.npz", dataset)
    topology_reference_path = destination / "accepted_topology_reference.npz"
    np.savez_compressed(
        topology_reference_path,
        schema=np.array("network-a1-accepted-topology-reference-v1"),
        **{
            key: np.concatenate(values)
            for key, values in topology_reference_parts.items()
        },
    )

    manifest = {
        "schema": MULTISEED_FULL_CAL_SCHEMA,
        "decision": (
            "formal topology supervision uses seed-isolated traditional full-Cal accepted steps; "
            "nonlinear iterations and retries are excluded"
        ),
        "repo_root": None if repo_root is None else str(Path(repo_root).resolve()),
        "full_modal_system": cut_freq is None,
        "cut_freq_hz": cut_freq,
        "training_seeds": list(seeds),
        "validation_seed": int(validation_seed),
        "test_seed": int(test_seed),
        "selection": {
            "sampling_stride_steps": int(sampling_stride_steps),
            "rollout_window_steps": int(rollout_window_steps),
            "transition_window_steps": int(transition_window_steps),
            "topology_context_steps": int(topology_context_steps),
            "accepted_step_label": True,
            "final_preload_anchor_per_seed": 8,
        },
        "splits": {
            name: {
                "samples": len(dataset),
                "topology_counts": {
                    str(value): int(np.sum(dataset.patch_count == value)) for value in range(3)
                },
                "sha256": _sha256(destination / f"{name}.npz"),
            }
            for name, dataset in datasets.items()
        },
        "runs": records,
        "accepted_topology_reference": {
            "path": topology_reference_path.name,
            "rows": int(sum(values.size for values in topology_reference_parts["patch_count"])),
            "sha256": _sha256(topology_reference_path),
        },
        "files": {
            "train": "train.npz",
            "validation": "validation.npz",
            "test": "test.npz",
            "accepted_topology_reference": topology_reference_path.name,
        },
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def _load_or_collect_source(
    *,
    seed: int,
    raw_path: Path,
    source_archive: Path | None,
    repo_root: str | Path | None,
    cut_freq: float | None,
    stage_end_mileage: dict[str, float] | None,
) -> tuple[NetworkADataset, Path, bool]:
    if source_archive is not None:
        source = load_network_a_dataset(source_archive, split="all", dtype=np.float64)
        return source, source_archive, True
    if raw_path.exists():
        source = load_network_a_dataset(raw_path, split="all", dtype=np.float64)
        return source, raw_path, True
    source = _collect_full_cal_source(
        repo_root=repo_root,
        irregularity_seed=seed,
        cut_freq=cut_freq,
        stage_end_mileage=stage_end_mileage,
    )
    save_network_a_archive(raw_path, source)
    return source, raw_path, False


def _validate_stage_endpoints(
    source: NetworkADataset,
    stage_end_mileage: dict[str, float] | None,
) -> None:
    """Reject legacy fixed-step archives that stopped short after a retry."""

    expected = DEFAULT_STAGE_END_MILEAGE_M if stage_end_mileage is None else stage_end_mileage
    stage = np.asarray(source.metadata["stage"], dtype=str)
    mileage = np.asarray(source.metadata["front_mileage_m"], dtype=float)
    step = np.asarray(source.metadata["step_index"], dtype=int)
    for name, endpoint in expected.items():
        rows = np.flatnonzero(stage == name)
        if not rows.size:
            raise ValueError(f"full-Cal source is missing stage {name}")
        final_step = int(np.max(step[rows]))
        final_rows = rows[step[rows] == final_step]
        actual = float(mileage[final_rows[0]])
        if not np.allclose(mileage[final_rows], actual, rtol=0.0, atol=1.0e-12):
            raise ValueError(f"{name} final accepted step has inconsistent wheel/side mileage")
        if not np.isclose(actual, float(endpoint), rtol=0.0, atol=1.0e-9):
            raise ValueError(
                f"{name} accepted trajectory stopped at {actual:.12f} m instead of "
                f"the required full-stage endpoint {float(endpoint):.12f} m"
            )


def _validate_locked_splits(
    datasets: dict[str, NetworkA1DirectDataset],
    training_seeds: Sequence[int],
) -> None:
    required = {
        "train": set(int(value) for value in training_seeds),
        "validation": {VALIDATION_SEED},
        "test": {TEST_SEED},
    }
    groups: dict[str, set[str]] = {}
    for name, dataset in datasets.items():
        validate_direct_dataset(dataset)
        seeds = set(int(value) for value in dataset.metadata["irregularity_seed"])
        if seeds != required[name]:
            raise ValueError(f"{name} seed set is {sorted(seeds)}, expected {sorted(required[name])}")
        if not np.all(np.asarray(dataset.metadata["accepted_step_label"], dtype=bool)):
            raise ValueError(f"{name} contains non-accepted-step labels")
        if not np.all(np.asarray(dataset.metadata["source"], dtype=str) == "full_cal_accepted"):
            raise ValueError(f"{name} contains non-full-Cal sources")
        groups[name] = set(np.asarray(dataset.metadata["group_id"], dtype=str))
    for left, right in (("train", "validation"), ("train", "test"), ("validation", "test")):
        overlap = groups[left] & groups[right]
        if overlap:
            raise ValueError(f"seed-isolated splits overlap: {left}/{right} {sorted(overlap)[:3]}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _archive_mapping(values: Sequence[str]) -> dict[int, Path]:
    result: dict[int, Path] = {}
    for value in values:
        seed_text, separator, path_text = value.partition("=")
        if not separator or not seed_text or not path_text:
            raise ValueError("--source-archive must use SEED=PATH")
        result[int(seed_text)] = Path(path_text)
    return result


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prepare seed-isolated multi-seed full-Cal accepted-step data."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/network_a_direct_multiseed_full_cal_v5_33seed_transition128"),
    )
    parser.add_argument("--repo-root", type=Path, default=Path(".."))
    parser.add_argument("--training-seeds", type=int, nargs="+", default=list(DEFAULT_TRAINING_SEEDS))
    parser.add_argument("--validation-seed", type=int, default=VALIDATION_SEED)
    parser.add_argument("--test-seed", type=int, default=TEST_SEED)
    parser.add_argument("--sampling-stride-steps", type=int, default=64)
    parser.add_argument("--rollout-window-steps", type=int, default=8)
    parser.add_argument("--transition-window-steps", type=int, default=128)
    parser.add_argument("--topology-context-steps", type=int, default=4)
    parser.add_argument("--cut-freq", type=float, default=None)
    parser.add_argument(
        "--source-archive",
        action="append",
        default=[],
        metavar="SEED=PATH",
        help="reuse one traditional accepted-step archive; may be repeated",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    manifest = prepare_multiseed_full_cal_dataset(
        output_dir=args.output_dir,
        repo_root=args.repo_root,
        training_seeds=args.training_seeds,
        validation_seed=args.validation_seed,
        test_seed=args.test_seed,
        source_archives=_archive_mapping(args.source_archive),
        sampling_stride_steps=args.sampling_stride_steps,
        rollout_window_steps=args.rollout_window_steps,
        transition_window_steps=args.transition_window_steps,
        topology_context_steps=args.topology_context_steps,
        cut_freq=args.cut_freq,
    )
    print(f"wrote {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
