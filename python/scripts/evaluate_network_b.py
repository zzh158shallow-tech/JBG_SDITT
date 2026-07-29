from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np


PYTHON_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYTHON_ROOT))

from sditt.models.network_b_features import DIRECT_FEATURE_NAMES
from sditt.models.wrcp_net_b import force_metrics, load_wrcp_net_b
from sditt.training_data.network_b import (
    NetworkBDataset,
    load_network_b_dataset,
    validate_network_b_dataset,
)


def _resolve_path(value: Path) -> Path:
    return value if value.is_absolute() else (PYTHON_ROOT / value).resolve()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _load_test_dataset(
    directory: Path,
    seeds: list[int],
    *,
    require_direct_schema: bool,
) -> tuple[NetworkBDataset, list[dict[str, object]]]:
    datasets: list[NetworkBDataset] = []
    sources: list[dict[str, object]] = []
    for seed in seeds:
        path = directory / f"seed_{seed}.npz"
        if not path.is_file():
            raise FileNotFoundError(f"Network B test dataset does not exist: {path}")
        dataset = load_network_b_dataset(path)
        validate_network_b_dataset(
            dataset,
            expected_seed=seed,
            require_direct_schema=require_direct_schema,
            require_all_wheel_sides=True,
        )
        datasets.append(dataset)
        sources.append(
            {
                "seed": int(seed),
                "path": str(path),
                "rows": len(dataset),
                "sha256": _sha256_file(path),
            }
        )
    return NetworkBDataset.concatenate(datasets), sources


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate a locked WRCP-Net B model on a disjoint final test split. "
            "This command never trains or modifies the model."
        )
    )
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--test-seeds", nargs="+", type=int, required=True)
    parser.add_argument(
        "--training-metrics",
        type=Path,
        default=None,
        help="Defaults to metrics.json beside the model and proves seed isolation.",
    )
    parser.add_argument("--ood-threshold", type=float, default=4.0)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Explicitly replace an existing final-test report.",
    )
    args = parser.parse_args()
    if args.ood_threshold <= 0.0:
        parser.error("--ood-threshold must be positive")
    if len(set(args.test_seeds)) != len(args.test_seeds):
        parser.error("--test-seeds contains duplicates")

    model_path = _resolve_path(args.model)
    dataset_dir = _resolve_path(args.dataset_dir)
    training_metrics_path = (
        model_path.parent / "metrics.json"
        if args.training_metrics is None
        else _resolve_path(args.training_metrics)
    )
    if not model_path.is_file():
        parser.error(f"model does not exist: {model_path}")
    if not training_metrics_path.is_file():
        parser.error(f"training metrics do not exist: {training_metrics_path}")

    model_sha256 = _sha256_file(model_path)
    training_metrics = json.loads(training_metrics_path.read_text(encoding="utf-8"))
    if training_metrics.get("model_sha256") != model_sha256:
        parser.error("training metrics model hash does not match --model")
    if "test" in training_metrics:
        parser.error(
            "training metrics already contain test results; this is not a blind-test model"
        )
    split = training_metrics.get("split")
    if not isinstance(split, dict):
        parser.error("training metrics do not contain a valid split record")
    forbidden_seeds = {
        int(value)
        for name in ("train_seeds", "validation_seeds")
        for value in split.get(name, [])
    }
    overlap = forbidden_seeds & set(args.test_seeds)
    if overlap:
        parser.error(
            "test seeds overlap training or validation seeds: "
            + ", ".join(str(value) for value in sorted(overlap))
        )

    model = load_wrcp_net_b(model_path)
    try:
        test, dataset_sources = _load_test_dataset(
            dataset_dir,
            args.test_seeds,
            require_direct_schema=model.feature_names == DIRECT_FEATURE_NAMES,
        )
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    if test.feature_names != model.feature_names:
        parser.error("test dataset feature schema does not match the locked model")

    prediction = model.predict(test.features)
    in_distribution = model.in_distribution(
        test.features,
        threshold=float(args.ood_threshold),
    )
    normal_force = prediction[:, 0]
    creep_ratio = np.linalg.norm(prediction[:, 1:4], axis=1) / np.maximum(
        normal_force,
        1.0e-12,
    )
    report = {
        "schema": "wrcp-net-b-blind-test-v1",
        "model": {
            "path": str(model_path),
            "sha256": model_sha256,
            "model_schema": model.model_schema,
        },
        "training_metrics": {
            "path": str(training_metrics_path),
            "sha256": _sha256_file(training_metrics_path),
            "train_seeds": split.get("train_seeds", []),
            "validation_seeds": split.get("validation_seeds", []),
        },
        "test": {
            "dataset_dir": str(dataset_dir),
            "seeds": sorted(int(value) for value in args.test_seeds),
            "rows": len(test),
            "sources": dataset_sources,
        },
        "ood": {
            "threshold": float(args.ood_threshold),
            "in_distribution_rows": int(np.count_nonzero(in_distribution)),
            "out_of_distribution_rows": int(np.count_nonzero(~in_distribution)),
            "out_of_distribution_fraction": float(np.mean(~in_distribution)),
        },
        "physical_constraints": {
            "finite": bool(np.isfinite(prediction).all()),
            "negative_normal_force_rows": int(np.count_nonzero(normal_force < 0.0)),
            "friction_limit": float(model.friction_limit),
            "friction_limit_violation_rows": int(
                np.count_nonzero(creep_ratio > model.friction_limit + 1.0e-12)
            ),
            "maximum_creep_force_to_normal_ratio": float(np.max(creep_ratio)),
        },
        "force_metrics": force_metrics(test.targets, prediction),
    }

    seed_label = "_".join(str(value) for value in sorted(args.test_seeds))
    output_path = (
        model_path.parent / f"test_metrics_seed_{seed_label}.json"
        if args.output is None
        else _resolve_path(args.output)
    )
    if output_path.exists() and not args.overwrite:
        parser.error(
            f"test report already exists: {output_path}; use --overwrite explicitly"
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output_path)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
