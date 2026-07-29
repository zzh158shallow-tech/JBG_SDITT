from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np

from sditt.training_data.network_a import collect_coupled_samples
from sditt.training_data.network_a_direct import (
    NetworkA1DirectDataset,
    build_direct_dataset,
    ensure_supervision_metadata,
    load_direct_dataset,
    save_direct_dataset,
    validate_direct_dataset,
)


FRESH_SCHEMA = "network-a-direct-fresh-transitions-v1"
SAMPLES_PER_STAGE_STEP = 8


def build_fresh_transition_dataset(
    *,
    base_dataset_dir: str | Path,
    output_dir: str | Path,
    repo_root: str | Path | None,
    irregularity_seeds: Sequence[int],
    stage_steps: Sequence[int],
    cut_freq: float | None = None,
) -> Path:
    seeds = tuple(int(value) for value in irregularity_seeds)
    steps = tuple(int(value) for value in stage_steps)
    if not seeds or not steps or min(steps) <= 0:
        raise ValueError("fresh transition seeds and positive stage steps are required")

    base_dir = Path(base_dataset_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    train = load_direct_dataset(base_dir / "train.npz")
    validation = load_direct_dataset(base_dir / "validation.npz")
    test = load_direct_dataset(base_dir / "test.npz")
    validation_seeds = set(
        int(value) for value in np.asarray(validation.metadata["irregularity_seed"], dtype=int)
        if value >= 0
    )
    test_seeds = set(
        int(value) for value in np.asarray(test.metadata["irregularity_seed"], dtype=int)
        if value >= 0
    )
    forbidden = set(seeds) & (validation_seeds | test_seeds)
    if forbidden:
        raise ValueError(f"fresh transition seeds overlap validation/test: {sorted(forbidden)}")

    fresh_parts: list[NetworkA1DirectDataset] = []
    run_records: list[dict[str, object]] = []
    rejected_overflow: list[dict[str, object]] = []
    for seed in seeds:
        for n_steps in steps:
            group_id = f"fresh-irregular-seed-{seed}-stage-steps-{n_steps}"
            requested = 2 * SAMPLES_PER_STAGE_STEP * n_steps
            source, overflow = collect_coupled_samples(
                requested,
                repo_root=repo_root,
                group_id=group_id,
                irregularity_model="china-ballastless",
                irregularity_seed=seed,
                cut_freq=cut_freq,
            )
            direct = build_direct_dataset(source, repo_root=repo_root)
            cal_indexes = np.flatnonzero(np.asarray(direct.metadata["stage"], dtype=str) == "Cal")
            cal = direct.subset(cal_indexes)
            metadata = {key: np.asarray(values).copy() for key, values in cal.metadata.items()}
            metadata["source"] = np.asarray(
                ["fresh_coupled_irregular"] * len(cal),
                dtype=str,
            )
            metadata["sample_id"] = np.asarray(
                [f"fresh:{group_id}:{value}" for value in metadata["sample_id"]],
                dtype=str,
            )
            cal = NetworkA1DirectDataset(
                features=cal.features,
                history=cal.history,
                history_mask=cal.history_mask,
                patch_count=cal.patch_count,
                patch_mask=cal.patch_mask,
                targets=cal.targets,
                previous_row=cal.previous_row,
                metadata=metadata,
            )
            validate_direct_dataset(cal)
            fresh_parts.append(cal)
            rejected_overflow.extend(overflow)
            run_records.append(
                {
                    "group_id": group_id,
                    "irregularity_seed": seed,
                    "stage_steps": n_steps,
                    "cal_samples": len(cal),
                    "cal_topology_counts": {
                        str(value): int(np.sum(cal.patch_count == value)) for value in range(3)
                    },
                }
            )

    fresh = concatenate_direct_datasets(fresh_parts)
    augmented = concatenate_direct_datasets((train, fresh))
    validate_direct_dataset(augmented)
    _validate_group_isolation(augmented, validation, test)
    save_direct_dataset(destination / "train.npz", augmented)
    save_direct_dataset(destination / "validation.npz", validation)
    save_direct_dataset(destination / "test.npz", test)

    manifest = {
        "schema": FRESH_SCHEMA,
        "base_dataset_dir": str(base_dir.resolve()),
        "repo_root": str(Path(repo_root).resolve()) if repo_root is not None else None,
        "irregularity_seeds": list(seeds),
        "stage_steps": list(steps),
        "base_train_samples": len(train),
        "fresh_cal_samples": len(fresh),
        "augmented_train_samples": len(augmented),
        "fresh_topology_counts": {
            str(value): int(np.sum(fresh.patch_count == value)) for value in range(3)
        },
        "history_rule": (
            "each fresh Cal-1 retains the final accepted traditional Preload geometry; "
            "later Cal rows retain accepted-step links within the same fresh run"
        ),
        "split_isolation": {
            "validation_seeds": sorted(validation_seeds),
            "test_seeds": sorted(test_seeds),
            "overlap": [],
        },
        "runs": run_records,
        "rejected_overflow_count": len(rejected_overflow),
        "files": {"train": "train.npz", "validation": "validation.npz", "test": "test.npz"},
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def concatenate_direct_datasets(
    datasets: Sequence[NetworkA1DirectDataset],
) -> NetworkA1DirectDataset:
    items = tuple(ensure_supervision_metadata(dataset) for dataset in datasets)
    if not items:
        raise ValueError("at least one direct dataset is required")
    metadata_keys = set(items[0].metadata)
    if any(set(item.metadata) != metadata_keys for item in items[1:]):
        raise ValueError("direct datasets must have identical metadata fields")
    offsets = np.cumsum([0] + [len(item) for item in items[:-1]])
    previous_parts = []
    for item, offset in zip(items, offsets, strict=True):
        previous = item.previous_row.copy()
        previous[previous >= 0] += int(offset)
        previous_parts.append(previous)
    result = NetworkA1DirectDataset(
        features=np.concatenate([item.features for item in items], axis=0),
        history=np.concatenate([item.history for item in items], axis=0),
        history_mask=np.concatenate([item.history_mask for item in items], axis=0),
        patch_count=np.concatenate([item.patch_count for item in items], axis=0),
        patch_mask=np.concatenate([item.patch_mask for item in items], axis=0),
        targets=np.concatenate([item.targets for item in items], axis=0),
        previous_row=np.concatenate(previous_parts),
        metadata={
            key: np.concatenate([np.asarray(item.metadata[key]) for item in items], axis=0)
            for key in sorted(metadata_keys)
        },
    )
    validate_direct_dataset(result)
    return result


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
            raise ValueError(f"fresh transition group leakage: {left}/{right} {sorted(overlap)[:3]}")


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect fresh traditional Preload-to-Cal transitions for A1 Direct."
    )
    parser.add_argument(
        "--base-dataset-dir",
        type=Path,
        default=Path("outputs/network_a_direct_set_v2_history_fixed"),
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=Path(".."))
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    parser.add_argument("--stage-steps", type=int, nargs="+", default=[1, 2, 5])
    parser.add_argument("--cut-freq", type=float, default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    manifest = build_fresh_transition_dataset(
        base_dataset_dir=args.base_dataset_dir,
        output_dir=args.output_dir,
        repo_root=args.repo_root,
        irregularity_seeds=args.seeds,
        stage_steps=args.stage_steps,
        cut_freq=args.cut_freq,
    )
    print(f"wrote {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
