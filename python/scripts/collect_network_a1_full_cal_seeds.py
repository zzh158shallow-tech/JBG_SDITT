from __future__ import annotations

import argparse
import json
from pathlib import Path

from sditt.training_data.network_a import save_network_a_archive
from sditt.training_data.network_a_direct_full_cal import (
    _collect_full_cal_source,
    validate_full_cal_source,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect resumable traditional full-Cal accepted-step archives by seed."
    )
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=Path(".."))
    parser.add_argument("--cut-freq", type=float, default=None)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)
    for seed in args.seeds:
        seed_dir = args.output_root / f"seed-{seed}"
        seed_dir.mkdir(parents=True, exist_ok=True)
        archive_path = seed_dir / "traditional_accepted_steps.npz"
        if archive_path.exists():
            print(f"reuse {archive_path}", flush=True)
            continue
        source = _collect_full_cal_source(
            repo_root=args.repo_root,
            irregularity_seed=seed,
            cut_freq=args.cut_freq,
            stage_end_mileage=None,
        )
        quality = validate_full_cal_source(source)
        save_network_a_archive(archive_path, source)
        record = {
            "seed": int(seed),
            "accepted_cal_steps": quality.accepted_steps,
            "accepted_cal_samples": quality.samples,
            "front_mileage_start_m": quality.front_mileage_start_m,
            "front_mileage_end_m": quality.front_mileage_end_m,
            "topology_counts": quality.topology_counts,
            "archive": archive_path.name,
        }
        (seed_dir / "collection.json").write_text(
            json.dumps(record, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        print(f"wrote {archive_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
