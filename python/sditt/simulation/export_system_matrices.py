from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from .system import build_default_modal_rw_system_matrices


def export_default_system_matrices(
    output_path: str | Path,
    *,
    repo_root: str | Path | None = None,
    cut_freq: float = 2000.0,
    vlc: float = 350 / 3.6,
) -> Path:
    """Build and save the main-script ``Mxt/Kxt/Cxt`` block system."""

    system, track, vehicle = build_default_modal_rw_system_matrices(
        repo_root=repo_root,
        cut_freq=cut_freq,
        vlc=vlc,
    )
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        Mxt=system.Mxt,
        Kxt=system.Kxt,
        Cxt=system.Cxt,
        n_track=np.array(system.layout.n_track, dtype=int),
        nm_fw=np.array(system.layout.nm_fw, dtype=int),
        n_wheels=np.array(system.layout.n_wheels, dtype=int),
        n_rv=np.array(system.layout.n_rv, dtype=int),
        M_track=track.M_track,
        K_track=track.K_track,
        C_track=track.C_track,
        M_vehicle=vehicle.M_vehicle,
        K_vehicle=vehicle.K_vehicle,
        C_vehicle=vehicle.C_vehicle,
    )
    return path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export full SDITT system matrices Mxt/Kxt/Cxt.",
    )
    parser.add_argument(
        "output",
        nargs="?",
        default="system_matrices_modal_rw.npz",
        help="Output .npz path.",
    )
    parser.add_argument("--repo-root", default=None, help="Repository root override.")
    parser.add_argument("--cut-freq", type=float, default=2000.0, help="Track modal cutoff frequency in Hz.")
    parser.add_argument("--vlc", type=float, default=350 / 3.6, help="Vehicle speed in m/s.")
    args = parser.parse_args()

    output_path = export_default_system_matrices(
        args.output,
        repo_root=args.repo_root,
        cut_freq=args.cut_freq,
        vlc=args.vlc,
    )
    with np.load(output_path) as data:
        print(f"saved={output_path}")
        print(
            "layout: "
            f"n_track={int(data['n_track'])}, "
            f"nm_fw={int(data['nm_fw'])}, "
            f"n_wheels={int(data['n_wheels'])}, "
            f"n_rv={int(data['n_rv'])}"
        )
        for name in ("Mxt", "Kxt", "Cxt"):
            matrix = data[name]
            print(f"{name}: shape={matrix.shape}, sum={matrix.sum():.12g}")


if __name__ == "__main__":
    main()
