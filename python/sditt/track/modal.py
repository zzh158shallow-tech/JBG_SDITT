from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import numpy as np
from scipy import sparse

from sditt.io.matlab import (
    MatFile,
    MatSummary,
    load_mat_file,
    load_mat_variables,
    summarize_mat_file,
)


@dataclass(frozen=True)
class ModalTrackMatrices:
    """Flexible turnout modal matrices and damping ratios."""

    mass_diag: np.ndarray
    stiffness_diag: np.ndarray
    damping_diag: np.ndarray
    DR: np.ndarray
    mode_freq: np.ndarray
    cut_freq: float

    @property
    def omega(self) -> np.ndarray:
        return 2.0 * np.pi * self.mode_freq[:, 1]

    @cached_property
    def M_track(self) -> np.ndarray:
        return np.diag(self.mass_diag)

    @cached_property
    def K_track(self) -> np.ndarray:
        return np.diag(self.stiffness_diag)

    @cached_property
    def C_track(self) -> np.ndarray:
        return np.diag(self.damping_diag)

    @cached_property
    def M_track_sparse(self) -> sparse.csr_matrix:
        return sparse.diags(self.mass_diag, format="csr")

    @cached_property
    def K_track_sparse(self) -> sparse.csr_matrix:
        return sparse.diags(self.stiffness_diag, format="csr")

    @cached_property
    def C_track_sparse(self) -> sparse.csr_matrix:
        return sparse.diags(self.damping_diag, format="csr")


def load_modal_turnout_data(path: str | Path) -> MatFile:
    """Load raw flexible turnout modal data from a MATLAB ``.mat`` file."""

    return load_mat_file(path)


def load_modal_turnout_frequencies(path: str | Path) -> np.ndarray:
    """Load ``ModeFreq.FT_All`` without expanding large mode-shape variables."""

    mat_file = load_mat_variables(path, {"ModeFreq"})
    mode_freq = mat_file.variables["ModeFreq"]
    return np.asarray(mode_freq["FT_All"], dtype=float)


def summarize_modal_turnout_data(path: str | Path) -> MatSummary:
    """Read variable names, shapes, and MATLAB classes without expanding arrays."""

    return summarize_mat_file(path)


def build_modal_track_matrices(
    mode_freq_all: np.ndarray,
    *,
    cut_freq: float = 2000.0,
    dr_normal: np.ndarray,
    dr_typical: np.ndarray | None = None,
) -> ModalTrackMatrices:
    """Reproduce the matrix part of ``Matrix_Modal_FT_230313.m``.

    The returned matrices follow the MATLAB modal formulation:
    ``M = I``, ``K = diag(omega^2)``, and ``C = diag(2 * xi * omega)``.
    Frequencies are read from column 2 of ``ModeFreq.FT_All`` in hertz.
    """

    mode_freq_all = np.asarray(mode_freq_all, dtype=float)
    bools = mode_freq_all[:, 1] <= cut_freq
    mode_freq = mode_freq_all[bools, :]
    omega = 2.0 * np.pi * mode_freq[:, 1]
    xi = _interpolate_damping_ratios(mode_freq[:, 1], dr_normal, dr_typical)

    n_track = mode_freq.shape[0]
    return ModalTrackMatrices(
        mass_diag=np.ones(n_track, dtype=float),
        stiffness_diag=omega**2,
        damping_diag=2.0 * xi * omega,
        DR=np.column_stack([mode_freq[:, 1], xi]),
        mode_freq=mode_freq,
        cut_freq=float(cut_freq),
    )


def build_flexible_turnout_modal_matrices(
    mat_path: str | Path,
    *,
    cut_freq: float = 2000.0,
    choose_turnout: str = "07(009)",
    matlab_dir: str | Path | None = None,
    dr_normal: np.ndarray | None = None,
    dr_typical: np.ndarray | None = None,
) -> ModalTrackMatrices:
    """Build default flexible turnout modal matrices from a modal ``.mat`` file."""

    mat_path = Path(mat_path)
    mode_freq_all = load_modal_turnout_frequencies(mat_path)
    n_track = int(np.count_nonzero(mode_freq_all[:, 1] <= cut_freq))
    if dr_normal is None:
        if choose_turnout == "07(009)":
            matlab_dir = Path(matlab_dir) if matlab_dir is not None else mat_path.parent
            dr_normal, default_typical = build_s8b_damping_tables(
                matlab_dir,
                mode_freq_all=mode_freq_all,
                n_track=n_track,
            )
            if dr_typical is None:
                dr_typical = default_typical
        elif choose_turnout == "CN18":
            dr_normal = np.array(
                [
                    [0, 0.2],
                    [50, 0.2],
                    [100, 0.1],
                    [150, 0.02],
                    [300, 0.02],
                    [1000, 0.02],
                    [2500, 0.02],
                ],
                dtype=float,
            )
        else:
            raise ValueError(f"unsupported turnout damping defaults: {choose_turnout}")

    return build_modal_track_matrices(
        mode_freq_all,
        cut_freq=cut_freq,
        dr_normal=dr_normal,
        dr_typical=dr_typical,
    )


def build_s8b_damping_tables(
    matlab_dir: str | Path,
    *,
    mode_freq_all: np.ndarray,
    n_track: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Reproduce the default ``07(009)`` S8b damping tables from the main script."""

    matlab_dir = Path(matlab_dir)
    mode_freq_all = np.asarray(mode_freq_all, dtype=float)

    dr_typical = _load_numeric_table(matlab_dir / "DR_S8b_v2_WeldAcc.txt")[:, :2]
    dr_typical = dr_typical[dr_typical[:, 0] <= n_track, :]
    dr_typical = _sort_by_first_column(dr_typical)
    x = mode_freq_all[dr_typical[:, 0].astype(int) - 1, 1]
    xy_dr = np.array(
        [
            [0, 0.30],
            [80, 0.30],
            [95, 0.05 * 4],
            [105, 0.05 * 4],
            [110, 0.70],
            [2500, 0.70],
        ],
        dtype=float,
    )
    dr_typical[:, 1] = np.interp(x, xy_dr[:, 0], xy_dr[:, 1])

    for filename, keep_columns in (
        ("DR_S8b_v1_Crossing.txt", None),
        ("DR_S8b_v5b_Crossing.txt", None),
        ("DR_S8b_v2_WeldAcc_ReCR.txt", 2),
    ):
        table = _load_numeric_table(matlab_dir / filename)
        if keep_columns is not None:
            table = table[:, :keep_columns]
        table = _sort_by_first_column(table)
        table = table[table[:, 0] <= n_track, :]
        dr_typical = _merge_typical_damping(dr_typical, table)

    dr_normal = np.array(
        [
            [0, 0.20],
            [80, 0.20],
            [95, 0.05 * 1],
            [105, 0.05 * 1],
            [110, 0.015],
            [200, 0.015],
            [250, 0.02],
            [350, 0.02],
            [375, 0.125],
            [450, 0.125],
            [475, 0.02],
            [600, 0.0175],
            [1000, 0.015],
            [2000, 0.01],
            [2500, 0.005],
        ],
        dtype=float,
    )
    return dr_normal, dr_typical


def _interpolate_damping_ratios(
    frequencies_hz: np.ndarray,
    dr_normal: np.ndarray,
    dr_typical: np.ndarray | None,
) -> np.ndarray:
    dr_normal = _sort_by_first_column(np.asarray(dr_normal, dtype=float))
    xi = np.interp(frequencies_hz, dr_normal[:, 0], dr_normal[:, 1])
    if dr_typical is None or np.asarray(dr_typical).size == 0:
        return xi

    dr_typical = _dedupe_typical_damping(np.asarray(dr_typical, dtype=float))
    for mode_index, damping_ratio in dr_typical[:, :2]:
        zero_based = int(mode_index) - 1
        if 0 <= zero_based < xi.size:
            xi[zero_based] = damping_ratio
    return xi


def _dedupe_typical_damping(table: np.ndarray) -> np.ndarray:
    table = _sort_by_first_column(table[:, :2])
    if table.shape[0] <= 1:
        return table
    _, keep_reversed = np.unique(table[::-1, 0], return_index=True)
    keep = table.shape[0] - 1 - keep_reversed
    return table[np.sort(keep), :]


def _merge_typical_damping(current: np.ndarray, updates: np.ndarray) -> np.ndarray:
    merged = {int(row[0]): row[:2].astype(float) for row in current}
    for row in updates[:, :2]:
        merged[int(row[0])] = row[:2].astype(float)
    return np.vstack([merged[key] for key in sorted(merged)])


def _sort_by_first_column(table: np.ndarray) -> np.ndarray:
    return table[np.argsort(table[:, 0]), :]


def _load_numeric_table(path: Path) -> np.ndarray:
    table = np.loadtxt(path, dtype=float)
    return np.atleast_2d(table)
