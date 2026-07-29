from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from sditt.training_data.network_a import LABEL_NAMES, save_network_a_archive
from sditt.training_data.network_a_direct_full_cal import _collect_full_cal_source


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Screen irregularity seeds with traditional Preload plus the first Cal transition "
            "window. Screening archives are not formal full-Cal training data."
        )
    )
    parser.add_argument("--seeds", type=int, nargs="+", required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/network_a1_transition_seed_screen"),
    )
    parser.add_argument("--repo-root", type=Path, default=Path(".."))
    parser.add_argument("--cal-end-mileage", type=float, default=48.3)
    parser.add_argument("--target-penetration-um", type=float, default=32.0)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    summary_path = destination / "screening_summary.json"
    records: list[dict[str, object]] = []
    penetration_index = LABEL_NAMES.index("corrected_vertical_penetration_m")
    for seed in args.seeds:
        source = _collect_full_cal_source(
            repo_root=args.repo_root,
            irregularity_seed=int(seed),
            cut_freq=None,
            stage_end_mileage={"Preload": 47.6, "Cal": float(args.cal_end_mileage)},
        )
        archive = destination / f"seed-{int(seed)}-transition.npz"
        save_network_a_archive(archive, source)
        stage = np.asarray(source.metadata["stage"], dtype=str)
        step = np.asarray(source.metadata["step_index"], dtype=int)
        selected = (stage == "Cal") & (step >= 1) & (step <= 64)
        active = source.patch_mask[selected]
        penetration = source.labels[selected, :, penetration_index][active]
        if not penetration.size:
            raise RuntimeError(f"seed {seed} has no active Cal patches in the first 64 steps")
        record = {
            "seed": int(seed),
            "archive": str(archive),
            "cal_steps": int(np.unique(step[stage == "Cal"]).size),
            "minimum_corrected_penetration_um": float(np.min(penetration) * 1.0e6),
            "maximum_corrected_penetration_um": float(np.max(penetration) * 1.0e6),
            "target_hit": bool(np.min(penetration) * 1.0e6 <= args.target_penetration_um),
            "formal_full_cal_training_data": False,
        }
        records.append(record)
        summary = {
            "schema": "network-a1-transition-seed-screen-v1",
            "decision": (
                "screening only; a target seed must be recollected to the 130 m Cal endpoint "
                "before it can enter formal training"
            ),
            "preload_end_mileage_m": 47.6,
            "cal_end_mileage_m": float(args.cal_end_mileage),
            "target_penetration_um": float(args.target_penetration_um),
            "records": records,
        }
        summary_path.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(record, ensure_ascii=False), flush=True)
    print(f"wrote {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
