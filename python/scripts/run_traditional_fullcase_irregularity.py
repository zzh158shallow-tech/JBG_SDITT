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


def build_full_case_arguments(
    *,
    output_dir: Path,
    irregularity_seed: int,
    resume_checkpoint: bool,
) -> list[str]:
    """Build the locked traditional-model full-case command-line arguments."""

    checkpoint_dir = output_dir / "checkpoints"
    preload_cache_dir = output_dir / "preload_cache"
    return [
        "--output-dir",
        str(output_dir),
        "--rail-layout",
        "interval",
        "--track-irregularity",
        "china-ballastless",
        "--irregularity-seed",
        str(irregularity_seed),
        "--contact-geometry-mode",
        "traditional",
        "--live-window",
        "--full-size",
        "--matlab-mileage-endpoints",
        "--save-progress",
        "--preload-cache-dir",
        str(preload_cache_dir),
        "--checkpoint-dir",
        str(checkpoint_dir),
        "--save-checkpoints",
        "--resume-checkpoint" if resume_checkpoint else "--no-resume-checkpoint",
    ]


def _parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run one full-mileage SDITT case with the traditional wheel-rail contact model, "
            "Chinese ballastless-track irregularity, complete wheel-force CSV output, and a live window."
        )
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory; defaults to python/outputs/traditional_fullcase_irregularity_seed<seed>.",
    )
    parser.add_argument(
        "--irregularity-seed",
        type=int,
        default=DEFAULT_SEED,
        help="Random-phase track-irregularity seed (default: 20260716).",
    )
    parser.add_argument(
        "--resume-checkpoint",
        action="store_true",
        help="Resume from the latest compatible checkpoint instead of starting a new full case.",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    output_dir = (
        args.output_dir
        if args.output_dir is not None
        else PYTHON_ROOT / "outputs" / f"traditional_fullcase_irregularity_seed{args.irregularity_seed}"
    )
    if not output_dir.is_absolute():
        output_dir = (REPO_ROOT / output_dir).resolve()

    print("SDITT traditional full-case configuration:")
    print("  rail layout: interval (L1 + R1)")
    print("  contact geometry: traditional")
    print("  track irregularity: china-ballastless")
    print(f"  irregularity seed: {args.irregularity_seed}")
    print("  modal system: full size")
    print("  mileage: full MATLAB endpoints")
    print(f"  output: {output_dir}")
    print("The realtime window will open first; click '开始计算' to start.")

    return run_full_case(
        build_full_case_arguments(
            output_dir=output_dir,
            irregularity_seed=args.irregularity_seed,
            resume_checkpoint=args.resume_checkpoint,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
