from __future__ import annotations

import argparse
from pathlib import Path

from sditt.validation.full_case_short_run import write_full_case_short_run_report


DEFAULT_MODEL = Path(
    "outputs/wrcp_net_a1_direct_multiseed_topology_v22_dagger_strict_stabilized/model.npz"
)
DEFAULT_OUTPUT = Path("outputs/network_a1_direct_v22_strict_rollout600")
PRELOAD_END_M = 47.6
SPEED_MPS = 350.0 / 3.6
DT_S = 1.0e-4


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run full Preload followed by a bounded strict Network A1 Cal rollout."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--network-b-model", type=Path, default=None)
    parser.add_argument("--network-b-ood-threshold", type=float, default=4.0)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--irregularity-seed", type=int, default=20260722)
    parser.add_argument("--nominal-cal-steps", type=int, default=600)
    parser.add_argument("--enable-trace", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.nominal_cal_steps < 500:
        raise SystemExit("--nominal-cal-steps must be at least 500")
    model = args.model.resolve()
    if not model.is_file():
        raise SystemExit(f"model does not exist: {model}")
    network_b_model = (
        None if args.network_b_model is None else args.network_b_model.resolve()
    )
    if network_b_model is not None and not network_b_model.is_file():
        raise SystemExit(f"Network B model does not exist: {network_b_model}")
    if args.network_b_ood_threshold <= 0.0:
        raise SystemExit("--network-b-ood-threshold must be positive")
    output = args.output_dir.resolve()
    cal_end = PRELOAD_END_M + args.nominal_cal_steps * SPEED_MPS * DT_S
    trace_dir = output / "network_a1_trace" if args.enable_trace else None
    snapshot_path, report_path = write_full_case_short_run_report(
        output,
        cut_freq=None,
        dt=DT_S,
        n_steps_per_stage=1,
        use_sparse=True,
        use_matlab_mileage_endpoints=False,
        stage_end_mileage={"Preload": PRELOAD_END_M, "Cal": cal_end},
        save_progress=True,
        save_contact_key_data=True,
        preload_cache_dir=output / "preload_cache",
        history_retention_steps=700,
        checkpoint_dir=output / "checkpoints",
        save_checkpoints=True,
        resume_checkpoint=False,
        rail_layout="interval",
        track_irregularity="china-ballastless",
        irregularity_seed=args.irregularity_seed,
        contact_geometry_mode="network-a1-direct-after-preload",
        network_a_model_path=model,
        network_a_trace_dir=trace_dir,
        network_a_trace_mode="selective",
        network_a_trace_low_confidence=0.95,
        network_a_trace_sample_interval_m=1.0,
        network_a_force_mode=(
            "network-b" if network_b_model is not None else "hertz"
        ),
        network_b_model_path=network_b_model,
        network_b_ood_threshold=float(args.network_b_ood_threshold),
        network_b_ood_fallback="hertz",
        contact_force_tolerance=2.5e-3,
        snapshot_history_limit=1,
    )
    print(f"wrote {snapshot_path}")
    print(f"wrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
