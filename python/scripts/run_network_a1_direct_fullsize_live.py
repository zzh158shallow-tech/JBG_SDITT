"""Run the full-size SDITT case with the guarded WRCP-Net A1 Direct model.

The production boundary is intentionally fixed:

* Preload uses the traditional contact solver.
* Network A1 Direct replaces contact geometry only during Cal.
* Cal forces use Network B when ``--network-b-model`` is supplied; otherwise
  they use the analytical Hertz/Kalker path.
* Runtime tracing is disabled unless ``--enable-trace`` is supplied.

Run this script from either the repository root or ``python/``.  The default
run opens the realtime Tk window and covers the full MATLAB mileage endpoints.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Iterable


PYTHON_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PYTHON_ROOT.parent
sys.path.insert(0, str(PYTHON_ROOT))

from sditt.validation.full_case_short_run import main as run_full_case


DEFAULT_SEED = 20260721
DEFAULT_MODEL = (
    PYTHON_ROOT
    / "outputs"
    / "wrcp_net_a1_direct_multiseed_topology_v22_dagger_strict_stabilized"
    / "model.npz"
)
DEFAULT_NETWORK_B_MODEL = (
    PYTHON_ROOT
    / "outputs"
    / "wrcp_net_b_direct_round0_residual_128x128_weight010"
    / "model.npz"
)
DEFAULT_PRELOAD_CACHE_DIR = PYTHON_ROOT / "outputs" / "shared_preload_cache"
DEFAULT_OUTPUT_PREFIX = "network_a1_direct_v22_fullsize_live"


def build_full_case_arguments(
    *,
    output_dir: Path,
    model_path: Path,
    irregularity_seed: int,
    preview_steps: int | None,
    live_window: bool,
    resume_checkpoint: bool,
    cut_freq: float | None,
    enable_trace: bool,
    network_b_model_path: Path | None = None,
    network_b_ood_threshold: float = 4.0,
    network_b_ood_fallback: str = "hertz",
    preload_cache_dir: Path | None = None,
) -> list[str]:
    """Build arguments for the existing, tested full-case driver."""

    checkpoint_dir = output_dir / "checkpoints"
    resolved_preload_cache_dir = (
        output_dir / "preload_cache"
        if preload_cache_dir is None
        else preload_cache_dir
    )
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
        "network-b" if network_b_model_path is not None else "hertz",
        "--network-b-ood-fallback",
        network_b_ood_fallback,
        "--contact-force-tolerance",
        "0.0025",
        "--save-progress",
        "--save-contact-key-data",
        "--history-retention-steps",
        "256",
        "--snapshot-history-limit",
        "1",
        "--preload-cache-dir",
        str(resolved_preload_cache_dir),
        "--checkpoint-dir",
        str(checkpoint_dir),
        "--save-checkpoints",
        "--resume-checkpoint" if resume_checkpoint else "--no-resume-checkpoint",
    ]
    if network_b_model_path is not None:
        arguments.extend(
            (
                "--network-b-model",
                str(network_b_model_path),
                "--network-b-ood-threshold",
                f"{network_b_ood_threshold:g}",
            )
        )

    if live_window:
        arguments.append("--live-window")
    if cut_freq is None:
        arguments.append("--full-size")
    else:
        arguments.extend(("--cut-freq", str(cut_freq)))
    if preview_steps is None:
        arguments.append("--matlab-mileage-endpoints")
    else:
        arguments.extend(("--steps", str(preview_steps)))
    if enable_trace:
        arguments.extend(
            (
                "--network-a-trace-dir",
                str(output_dir / "network_a1_trace"),
                "--network-a-trace-mode",
                "selective",
                "--network-a-trace-low-confidence",
                "0.95",
                "--network-a-trace-sample-interval-m",
                "1.0",
            )
        )
    return arguments


def _parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the guarded WRCP-Net A1 Direct model in the full-size SDITT "
            "full case with a realtime window."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory; defaults to python/outputs/<case name>.",
    )
    parser.add_argument(
        "--preload-cache-dir",
        type=Path,
        default=DEFAULT_PRELOAD_CACHE_DIR,
        help=(
            "Shared Preload cache directory. Compatible configurations, including "
            "the same irregularity seed, reuse the saved accepted Preload state."
        ),
    )
    parser.add_argument(
        "--network-a-model",
        type=Path,
        default=DEFAULT_MODEL,
        help="Path to the guarded A1 Direct model.npz artifact.",
    )
    parser.add_argument(
        "--network-b-model",
        type=Path,
        default=None,
        help=(
            "Enable WRCP-Net B forces in Cal using this model artifact. "
            f"The current Round 0 model is {DEFAULT_NETWORK_B_MODEL}."
        ),
    )
    parser.add_argument(
        "--network-b-ood-threshold",
        type=float,
        default=4.0,
        help="Maximum calibrated Network B feature distance before fallback.",
    )
    parser.add_argument(
        "--network-b-ood-fallback",
        choices=("hertz", "traditional"),
        default="hertz",
        help="Force law used when Network B receives an out-of-distribution input.",
    )
    parser.add_argument(
        "--irregularity-seed",
        type=int,
        default=DEFAULT_SEED,
        help="Random seed for the Chinese ballastless-track irregularity.",
    )
    parser.add_argument(
        "--preview-steps",
        type=int,
        default=None,
        help=(
            "Run only N accepted steps per stage for a quick check. Omit this "
            "option to run to the full MATLAB mileage endpoints."
        ),
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable the Tk realtime window for automated diagnostics.",
    )
    parser.add_argument(
        "--cut-freq",
        type=float,
        default=None,
        help=(
            "Use a reduced modal cutoff frequency for diagnostics. Omit this "
            "option to use the full-size modal system."
        ),
    )
    checkpoint_group = parser.add_mutually_exclusive_group()
    checkpoint_group.add_argument(
        "--resume-checkpoint",
        dest="resume_checkpoint",
        action="store_true",
        default=True,
        help="Resume from the latest compatible checkpoint (default).",
    )
    checkpoint_group.add_argument(
        "--no-resume-checkpoint",
        dest="resume_checkpoint",
        action="store_false",
        help="Start from the beginning and ignore existing checkpoints.",
    )
    parser.add_argument(
        "--enable-trace",
        action="store_true",
        help=(
            "Enable selective Network A1 iteration tracing for diagnosis. "
            "Formal production runs leave tracing disabled."
        ),
    )
    return parser.parse_args(argv)


def _resolve_input_path(value: Path) -> Path:
    if value.is_absolute():
        return value.resolve()
    python_candidate = (PYTHON_ROOT / value).resolve()
    repo_candidate = (REPO_ROOT / value).resolve()
    if python_candidate.exists() or not repo_candidate.exists():
        return python_candidate
    return repo_candidate


def _resolve_output_path(value: Path) -> Path:
    if value.is_absolute():
        return value.resolve()
    if value.parts and value.parts[0] == "python":
        return (REPO_ROOT / value).resolve()
    return (PYTHON_ROOT / value).resolve()


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.preview_steps is not None and args.preview_steps <= 0:
        raise SystemExit("--preview-steps must be positive")
    if args.cut_freq is not None and args.cut_freq <= 0.0:
        raise SystemExit("--cut-freq must be positive")
    if args.network_b_ood_threshold <= 0.0:
        raise SystemExit("--network-b-ood-threshold must be positive")

    model_path = _resolve_input_path(args.network_a_model)
    if not model_path.is_file():
        raise SystemExit(f"Network A1 model artifact does not exist: {model_path}")
    network_b_model_path = (
        None
        if args.network_b_model is None
        else _resolve_input_path(args.network_b_model)
    )
    if network_b_model_path is not None and not network_b_model_path.is_file():
        raise SystemExit(
            f"Network B model artifact does not exist: {network_b_model_path}"
        )

    case_suffix = (
        f"full_mileage_seed{args.irregularity_seed}"
        if args.preview_steps is None
        else f"{args.preview_steps}steps_seed{args.irregularity_seed}"
    )
    output_dir = (
        _resolve_output_path(args.output_dir)
        if args.output_dir is not None
        else PYTHON_ROOT / "outputs" / f"{DEFAULT_OUTPUT_PREFIX}_{case_suffix}"
    )
    preload_cache_dir = _resolve_output_path(args.preload_cache_dir)

    print("WRCP-Net A1 Direct full-case configuration:")
    print("  Preload geometry: traditional solver")
    print("  Cal geometry: guarded Network A1 Direct")
    if network_b_model_path is None:
        print("  Cal contact force: Hertz + Kalker; Network B disabled")
    else:
        print("  Cal contact force: guarded Network B")
        print(f"  Network B model: {network_b_model_path}")
        print(f"  Network B OOD threshold: {args.network_b_ood_threshold:g}")
        print(f"  Network B OOD fallback: {args.network_b_ood_fallback}")
    print("  rail layout: interval L1 + R1")
    print("  irregularity: china-ballastless")
    print(f"  irregularity seed: {args.irregularity_seed}")
    print(
        "  structural dynamics: "
        + ("full-size FT-Modal" if args.cut_freq is None else f"cut_freq={args.cut_freq:g} Hz")
    )
    print(
        "  mileage: "
        + (
            "full MATLAB endpoints"
            if args.preview_steps is None
            else f"{args.preview_steps} accepted preview steps per stage"
        )
    )
    print(f"  model: {model_path}")
    print(f"  output: {output_dir}")
    print(f"  shared Preload cache: {preload_cache_dir}")
    print(f"  checkpoint resume: {args.resume_checkpoint}")
    print(f"  Network A1 trace: {'selective' if args.enable_trace else 'disabled'}")
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
            cut_freq=args.cut_freq,
            enable_trace=args.enable_trace,
            network_b_model_path=network_b_model_path,
            network_b_ood_threshold=args.network_b_ood_threshold,
            network_b_ood_fallback=args.network_b_ood_fallback,
            preload_cache_dir=preload_cache_dir,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
