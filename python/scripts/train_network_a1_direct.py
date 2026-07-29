from __future__ import annotations

import argparse
from pathlib import Path

from sditt.models.train_wrcp_net_a1_direct import train_wrcp_net_a1_direct
from sditt.models.wrcp_net_a1_direct import load_wrcp_net_a1_direct
from sditt.training_data.network_a_direct import load_direct_dataset


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train WRCP-Net A1 Direct Set.")
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        default=Path("outputs/network_a_direct_multiseed_full_cal_v5_33seed_transition128"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/wrcp_net_a1_direct_multiseed_topology_v21_shape_strict"),
    )
    parser.add_argument("--epochs", type=int, default=400)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--seed", type=int, default=20260721)
    parser.add_argument("--learning-rate", type=float, default=3.0e-4)
    parser.add_argument("--patience", type=int, default=40)
    parser.add_argument("--maximum-rollout-steps", type=int, default=32)
    parser.add_argument("--final-teacher-probability", type=float, default=0.1)
    parser.add_argument("--rollout-start-fraction", type=float, default=0.4)
    parser.add_argument("--rollout-refresh-epochs", type=int, default=5)
    parser.add_argument(
        "--moment-regression-weight",
        type=float,
        default=8.0,
        help="relative loss weight for the three dimensionless patch-shape moments",
    )
    parser.add_argument(
        "--initial-model",
        type=Path,
        default=None,
        help="warm-start weights and normalization from an existing A1 Direct artifact",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="repository root used to embed fixed LMA/L1/R1 projection tables",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    result = train_wrcp_net_a1_direct(
        load_direct_dataset(args.dataset_dir / "train.npz"),
        load_direct_dataset(args.dataset_dir / "validation.npz"),
        load_direct_dataset(args.dataset_dir / "test.npz"),
        output_dir=args.output_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        patience=args.patience,
        seed=args.seed,
        repo_root=args.repo_root,
        initial_model=(
            load_wrcp_net_a1_direct(args.initial_model)
            if args.initial_model is not None
            else None
        ),
        maximum_rollout_steps=args.maximum_rollout_steps,
        final_teacher_probability=args.final_teacher_probability,
        rollout_start_fraction=args.rollout_start_fraction,
        rollout_refresh_epochs=args.rollout_refresh_epochs,
        moment_regression_weight=args.moment_regression_weight,
    )
    print(f"wrote {result.model_path}")
    print(f"wrote {result.metrics_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
