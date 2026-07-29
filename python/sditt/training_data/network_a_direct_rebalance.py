from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np

from sditt.training_data.network_a_direct import (
    NetworkA1DirectDataset,
    load_direct_dataset,
    save_direct_dataset,
    validate_direct_dataset,
)
from sditt.training_data.network_a_direct_fresh import concatenate_direct_datasets


REBALANCED_SCHEMA = "network-a-direct-rebalanced-dataset-v1"


def build_rebalanced_dataset(
    *,
    base_dataset_dir: str | Path,
    candidate_dataset_dir: str | Path,
    output_dir: str | Path,
    group_substring: str,
    retained_step_indexes: Sequence[int],
) -> Path:
    """Add selected accepted steps from a larger candidate dataset.

    This keeps a long-stage transition represented without letting every step
    of that trajectory overwhelm the earlier training distribution.
    """

    retained = tuple(sorted(set(int(value) for value in retained_step_indexes)))
    if not group_substring or not retained or retained[0] <= 0:
        raise ValueError("a group substring and positive retained step indexes are required")
    base_dir = Path(base_dataset_dir)
    candidate_dir = Path(candidate_dataset_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)

    train = load_direct_dataset(base_dir / "train.npz")
    validation = load_direct_dataset(base_dir / "validation.npz")
    test = load_direct_dataset(base_dir / "test.npz")
    candidate = load_direct_dataset(candidate_dir / "train.npz")
    group_ids = np.asarray(candidate.metadata["group_id"], dtype=str)
    step_index = np.asarray(candidate.metadata["step_index"], dtype=int)
    marker = np.fromiter(
        (group_substring in value for value in group_ids),
        dtype=bool,
        count=len(candidate),
    )
    selection = np.flatnonzero(marker & np.isin(step_index, retained))
    if not selection.size:
        raise ValueError("no candidate rows matched the requested group and step indexes")
    selected = candidate.subset(selection)
    augmented = concatenate_direct_datasets((train, selected))
    validate_direct_dataset(augmented)
    _validate_group_isolation(augmented, validation, test)

    save_direct_dataset(destination / "train.npz", augmented)
    save_direct_dataset(destination / "validation.npz", validation)
    save_direct_dataset(destination / "test.npz", test)
    manifest = {
        "schema": REBALANCED_SCHEMA,
        "base_dataset_dir": str(base_dir.resolve()),
        "candidate_dataset_dir": str(candidate_dir.resolve()),
        "group_substring": group_substring,
        "retained_step_indexes": list(retained),
        "base_train_samples": len(train),
        "selected_samples": len(selected),
        "augmented_train_samples": len(augmented),
        "selected_group_counts": {
            str(value): int(count)
            for value, count in zip(
                *np.unique(np.asarray(selected.metadata["group_id"], dtype=str), return_counts=True),
                strict=True,
            )
        },
        "selected_topology_counts": {
            str(value): int(np.sum(selected.patch_count == value)) for value in range(3)
        },
        "split_isolation": {"overlap": []},
        "files": {"train": "train.npz", "validation": "validation.npz", "test": "test.npz"},
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest_path


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
            raise ValueError(f"rebalanced dataset group leakage: {left}/{right} {sorted(overlap)[:3]}")


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Select sparse long-stage accepted steps without overwhelming A1 Direct training."
    )
    parser.add_argument("--base-dataset-dir", type=Path, required=True)
    parser.add_argument("--candidate-dataset-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--group-substring", required=True)
    parser.add_argument("--retained-step-indexes", type=int, nargs="+", required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    manifest = build_rebalanced_dataset(
        base_dataset_dir=args.base_dataset_dir,
        candidate_dataset_dir=args.candidate_dataset_dir,
        output_dir=args.output_dir,
        group_substring=args.group_substring,
        retained_step_indexes=args.retained_step_indexes,
    )
    print(f"wrote {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
