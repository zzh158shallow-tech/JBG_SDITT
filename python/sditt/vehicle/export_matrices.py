from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from sditt.config import ProjectPaths

from .matrices import build_vehicle_matrices_rw_230409
from .parameters import load_vehicle_parameters


def export_vehicle_matrices(
    output_path: str | Path,
    *,
    repo_root: str | Path | None = None,
    vlc: float = 350 / 3.6,
    n_rv: int = 51,
) -> Path:
    """Build and save CRH380A vehicle matrices using MATLAB-compatible names."""

    paths = ProjectPaths.from_repo_root(repo_root)
    parameters = load_vehicle_parameters(paths.default_vehicle_parameters, vlc=vlc)
    matrices = build_vehicle_matrices_rw_230409(parameters, n_rv=n_rv)

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        M_vehicle=matrices.M_vehicle,
        K_vehicle=matrices.K_vehicle,
        C_vehicle=matrices.C_vehicle,
        Mlc=matrices.Mlc,
        Klc=matrices.Klc,
        Clc=matrices.Clc,
        Clc_0=matrices.Clc_0,
    )
    return path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export CRH380A vehicle M/K/C matrices reproduced from MATLAB.",
    )
    parser.add_argument(
        "output",
        nargs="?",
        default="vehicle_matrices_rw_230409.npz",
        help="Output .npz path.",
    )
    parser.add_argument("--repo-root", default=None, help="Repository root override.")
    parser.add_argument("--vlc", type=float, default=350 / 3.6, help="Vehicle speed in m/s.")
    parser.add_argument("--n-rv", type=int, default=51, help="Vehicle DOF count.")
    args = parser.parse_args()

    output_path = export_vehicle_matrices(
        args.output,
        repo_root=args.repo_root,
        vlc=args.vlc,
        n_rv=args.n_rv,
    )
    with np.load(output_path) as data:
        print(f"saved={output_path}")
        for name in ("M_vehicle", "K_vehicle", "C_vehicle", "Clc_0"):
            matrix = data[name]
            print(f"{name}: shape={matrix.shape}, sum={matrix.sum():.12g}")


if __name__ == "__main__":
    main()
