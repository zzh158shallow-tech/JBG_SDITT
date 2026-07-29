from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


PYTHON_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYTHON_ROOT))

from sditt.models.wrcp_net_b import train_wrcp_net_b
from sditt.training_data.network_b import NetworkBDataset, load_network_b_dataset


def _load(directory: Path, seeds: list[int]) -> NetworkBDataset:
    return NetworkBDataset.concatenate(
        [load_network_b_dataset(str(directory / f"seed_{seed}.npz")) for seed in seeds]
    )


def _resolve_directory(value: Path) -> Path:
    return value if value.is_absolute() else (PYTHON_ROOT / value).resolve()


def _filter_training_match_distance(
    dataset: NetworkBDataset,
    maximum_um: float | None,
) -> NetworkBDataset:
    if maximum_um is None:
        return dataset
    if maximum_um <= 0.0:
        raise ValueError("--max-train-teacher-match-um must be positive")
    selected = np.flatnonzero(
        dataset.teacher_match_distance_m <= float(maximum_um) * 1.0e-6
    )
    if selected.size == 0:
        raise ValueError("teacher-match filter removed every training row")
    return dataset.subset(selected)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Train the WRCP-Net B patch-level MLP with seed-isolated training "
            "and validation splits. Final test data is intentionally unsupported "
            "by this command."
        )
    )
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        default=None,
        help=(
            "Legacy common directory used as a fallback for both training and "
            "validation data."
        ),
    )
    parser.add_argument("--train-dataset-dir", type=Path, default=None)
    parser.add_argument("--validation-dataset-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/wrcp_net_b"))
    parser.add_argument("--aux-dataset-dir", type=Path, default=None)
    parser.add_argument("--aux-train-seeds", nargs="+", type=int, default=[])
    parser.add_argument("--train-seeds", nargs="+", type=int, required=True)
    parser.add_argument("--validation-seeds", nargs="+", type=int, required=True)
    parser.add_argument("--epochs", type=int, default=800)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--patience", type=int, default=80)
    parser.add_argument("--learning-rate", type=float, default=8.0e-4)
    parser.add_argument("--weight-decay", type=float, default=1.0e-5)
    parser.add_argument("--huber-delta", type=float, default=0.5)
    parser.add_argument(
        "--loss-weights",
        nargs=7,
        type=float,
        default=None,
        metavar=(
            "FN",
            "FX",
            "FY",
            "FZ",
            "MX",
            "MY",
            "MZ",
        ),
    )
    parser.add_argument("--training-seed", type=int, default=20260720)
    parser.add_argument("--hidden-sizes", nargs="+", type=int, default=[64, 64])
    parser.add_argument(
        "--normal-force-mode",
        choices=("absolute_power", "hertz_residual", "hertz_fixed"),
        default="absolute_power",
    )
    parser.add_argument("--ood-training-quantile", type=float, default=0.9999)
    parser.add_argument("--ood-calibration-threshold", type=float, default=4.0)
    parser.add_argument(
        "--max-train-teacher-match-um",
        type=float,
        default=None,
        help=(
            "Optional sensitivity filter applied only to training rows; validation "
            "always retains its complete accepted-step distribution."
        ),
    )
    parser.add_argument(
        "--friction-limit",
        type=float,
        default=0.40,
        help="Maximum predicted creep-force magnitude divided by normal force.",
    )
    args = parser.parse_args()
    common_dir = (
        None if args.dataset_dir is None else _resolve_directory(args.dataset_dir)
    )
    train_dir = (
        common_dir
        if args.train_dataset_dir is None
        else _resolve_directory(args.train_dataset_dir)
    )
    validation_dir = (
        common_dir
        if args.validation_dataset_dir is None
        else _resolve_directory(args.validation_dataset_dir)
    )
    if train_dir is None:
        parser.error("--train-dataset-dir is required when --dataset-dir is omitted")
    if validation_dir is None:
        parser.error(
            "--validation-dataset-dir is required when --dataset-dir is omitted"
        )
    if args.friction_limit <= 0.0:
        parser.error("--friction-limit must be positive")
    if args.batch_size <= 0 or args.patience <= 0:
        parser.error("--batch-size and --patience must be positive")
    if args.weight_decay < 0.0:
        parser.error("--weight-decay must be non-negative")
    if args.huber_delta <= 0.0:
        parser.error("--huber-delta must be positive")
    if args.loss_weights is not None and (
        any(value < 0.0 for value in args.loss_weights)
        or not any(value > 0.0 for value in args.loss_weights)
    ):
        parser.error("--loss-weights must be non-negative with a positive sum")
    if not 0.0 < args.ood_training_quantile < 1.0:
        parser.error("--ood-training-quantile must be between 0 and 1")
    if args.ood_calibration_threshold <= 0.0:
        parser.error("--ood-calibration-threshold must be positive")
    output_dir = _resolve_directory(args.output_dir)
    train = _load(train_dir, args.train_seeds)
    if args.aux_dataset_dir is not None and args.aux_train_seeds:
        aux_dir = _resolve_directory(args.aux_dataset_dir)
        train = NetworkBDataset.concatenate([train, _load(aux_dir, args.aux_train_seeds)])
    unfiltered_train_rows = len(train)
    try:
        train = _filter_training_match_distance(
            train,
            args.max_train_teacher_match_um,
        )
    except ValueError as exc:
        parser.error(str(exc))
    validation = _load(validation_dir, args.validation_seeds)
    result = train_wrcp_net_b(
        train,
        validation,
        output_dir=output_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        patience=args.patience,
        learning_rate=args.learning_rate,
        weight_decay=args.weight_decay,
        huber_delta=args.huber_delta,
        loss_weights=args.loss_weights,
        seed=args.training_seed,
        hidden_sizes=tuple(args.hidden_sizes),
        friction_limit=float(args.friction_limit),
        normal_force_mode=args.normal_force_mode,
        ood_training_quantile=args.ood_training_quantile,
        ood_calibration_threshold=args.ood_calibration_threshold,
        training_metadata={
            "train_dataset_dir": str(train_dir),
            "validation_dataset_dir": str(validation_dir),
            "aux_dataset_dir": (
                None
                if args.aux_dataset_dir is None
                else str(_resolve_directory(args.aux_dataset_dir))
            ),
            "unfiltered_train_rows": int(unfiltered_train_rows),
            "max_train_teacher_match_um": args.max_train_teacher_match_um,
        },
    )
    print(json.dumps(result.metrics, indent=2, ensure_ascii=False))
    print(f"wrote {result.model_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
