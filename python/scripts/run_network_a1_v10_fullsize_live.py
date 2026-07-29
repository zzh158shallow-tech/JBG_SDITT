from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Iterable


PYTHON_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PYTHON_ROOT.parent
sys.path.insert(0, str(PYTHON_ROOT))

from sditt.validation.full_case_short_run import main as run_full_case


DEFAULT_SEED = 20260716
DEFAULT_MODEL = (
    PYTHON_ROOT
    / "outputs"
    / "wrcp_net_a1_direct_stageaware_v10_multilength_dagger_fixed_projection"
    / "model.npz"
)


def build_full_case_arguments(
    *,
    output_dir: Path,
    model_path: Path,
    irregularity_seed: int,
    preview_steps: int | None,
    live_window: bool,
    resume_checkpoint: bool,
    full_trace: bool,
) -> list[str]:
    """Build the locked V10 full-modal validation command."""

    checkpoint_dir = output_dir / "checkpoints"
    preload_cache_dir = output_dir / "preload_cache"
    trace_dir = output_dir / "network_a1_trace"
    arguments = [
        "--output-dir",
        str(output_dir),
        "--rail-layout",
        "interval",
        "--track-irregularity",
        "china-ballastless",
        "--irregularity-seed",
        str(irregularity_seed),
        "--contact-geometry-mode",
        "network-a1-direct-after-preload",
        "--network-a-model",
        str(model_path),
        "--network-a-force-mode",
        "hertz",
        "--network-b-ood-fallback",
        "hertz",
        "--network-a-trace-dir",
        str(trace_dir),
        "--network-a-trace-mode",
        "full" if full_trace else "selective",
        "--network-a-trace-low-confidence",
        "0.95",
        "--network-a-trace-sample-interval-m",
        "1.0",
        "--full-size",
        "--save-progress",
        "--save-contact-key-data",
        "--history-retention-steps",
        "256",
        "--preload-cache-dir",
        str(preload_cache_dir),
        "--checkpoint-dir",
        str(checkpoint_dir),
        "--save-checkpoints",
        "--resume-checkpoint" if resume_checkpoint else "--no-resume-checkpoint",
    ]
    if live_window:
        arguments.append("--live-window")
    if preview_steps is None:
        arguments.append("--matlab-mileage-endpoints")
    else:
        arguments.extend(("--steps", str(preview_steps)))
    return arguments


def _parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the experimental WRCP-Net A1 V10 direct-geometry model with the "
            "full FT-Modal system, interval L1/R1 rails, irregularity, and a live window."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory under python/outputs by default.",
    )
    parser.add_argument(
        "--network-a-model",
        type=Path,
        default=DEFAULT_MODEL,
        help="V10 fixed-profile projection artifact.",
    )
    parser.add_argument("--irregularity-seed", type=int, default=DEFAULT_SEED)
    parser.add_argument(
        "--preview-steps",
        type=int,
        default=None,
        help=(
            "Explicitly shorten the run to N accepted steps per stage. "
            "Omit this option for the default full MATLAB mileage."
        ),
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable the Tk realtime window for automated validation.",
    )
    parser.add_argument(
        "--resume-checkpoint",
        action="store_true",
        help="Resume from the latest compatible version-4 checkpoint.",
    )
    parser.add_argument(
        "--full-trace",
        action="store_true",
        help="Record every A1 nonlinear iteration; selective tracing is the default.",
    )
    return parser.parse_args(argv)


def _resolve_path(value: Path) -> Path:
    if value.is_absolute():
        return value.resolve()
    repo_candidate = (REPO_ROOT / value).resolve()
    python_candidate = (PYTHON_ROOT / value).resolve()
    if repo_candidate.exists() or not python_candidate.exists():
        return repo_candidate
    return python_candidate


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.preview_steps is not None and args.preview_steps <= 0:
        raise SystemExit("--preview-steps must be positive")
    model_path = _resolve_path(args.network_a_model)
    if not model_path.is_file():
        raise SystemExit(f"V10 model artifact does not exist: {model_path}")
    output_dir = (
        _resolve_path(args.output_dir)
        if args.output_dir is not None
        else PYTHON_ROOT
        / "outputs"
        / (
            f"network_a1_v10_fullsize_full_mileage_seed{args.irregularity_seed}"
            if args.preview_steps is None
            else f"network_a1_v10_fullsize_{args.preview_steps}steps_seed{args.irregularity_seed}"
        )
    )

    print("WRCP-Net A1 V10 full-size experimental run:")
    print("  rail layout: interval (L1 + R1)")
    print("  dynamics: full-size FT-Modal surrogate")
    print("  geometry: V10 A1 Direct fixed-profile projection after Preload")
    print("  force: Hertz + Kalker validation path (Network B is not enabled)")
    print("  irregularity: china-ballastless")
    print(f"  irregularity seed: {args.irregularity_seed}")
    print(
        "  mileage: "
        + (
            "full MATLAB endpoints"
            if args.preview_steps is None
            else f"{args.preview_steps} preview steps per stage"
        )
    )
    print(f"  model: {model_path}")
    print(f"  output: {output_dir}")
    print("  status: experimental; strict A1 confidence/OOD guards remain enabled")
    if not args.headless:
        print("The realtime window will open first; click '开始计算' to start.")

    return run_full_case(
        build_full_case_arguments(
            output_dir=output_dir,
            model_path=model_path,
            irregularity_seed=args.irregularity_seed,
            preview_steps=args.preview_steps,
            live_window=not args.headless,
            resume_checkpoint=args.resume_checkpoint,
            full_trace=args.full_trace,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
