from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from sditt.config import ProjectPaths

from .modal import build_flexible_turnout_modal_matrices


def export_modal_track_matrices(
    output_path: str | Path,
    *,
    repo_root: str | Path | None = None,
    cut_freq: float = 2000.0,
    choose_turnout: str = "07(009)",
) -> Path:
    """Build and save flexible turnout modal M/K/C matrices."""

    paths = ProjectPaths.from_repo_root(repo_root)
    matrices = build_flexible_turnout_modal_matrices(
        paths.modal_turnout_mat,
        cut_freq=cut_freq,
        choose_turnout=choose_turnout,
        matlab_dir=paths.matlab_dir,
    )

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        M_track=matrices.M_track,
        K_track=matrices.K_track,
        C_track=matrices.C_track,
        DR=matrices.DR,
        ModeFreq_FT=matrices.mode_freq,
        omega=matrices.omega,
        cut_freq=np.array(matrices.cut_freq, dtype=float),
    )
    return path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export flexible turnout modal matrices reproduced from MATLAB.",
    )
    parser.add_argument(
        "output",
        nargs="?",
        default="modal_track_matrices_ft_230313.npz",
        help="Output .npz path.",
    )
    parser.add_argument("--repo-root", default=None, help="Repository root override.")
    parser.add_argument("--cut-freq", type=float, default=2000.0, help="Modal cutoff frequency in Hz.")
    parser.add_argument("--choose-turnout", default="07(009)", help="Turnout damping defaults.")
    args = parser.parse_args()

    output_path = export_modal_track_matrices(
        args.output,
        repo_root=args.repo_root,
        cut_freq=args.cut_freq,
        choose_turnout=args.choose_turnout,
    )
    with np.load(output_path) as data:
        print(f"saved={output_path}")
        for name in ("M_track", "K_track", "C_track", "DR"):
            matrix = data[name]
            print(f"{name}: shape={matrix.shape}, sum={matrix.sum():.12g}")


if __name__ == "__main__":
    main()
