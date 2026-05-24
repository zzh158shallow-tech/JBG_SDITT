from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import numpy as np
from scipy import sparse

from sditt.config import MATLAB_FULL_DEFAULT_CASE, DefaultOperatingCase, ProjectPaths
from sditt.track import ModalTrackMatrices, build_flexible_turnout_modal_matrices
from sditt.vehicle import VehicleMatrices, build_vehicle_matrices_rw_230409, load_vehicle_parameters


@dataclass(frozen=True)
class SystemDofLayout:
    """System DOF slices matching the MATLAB ``Mxt/Kxt/Cxt`` block ordering."""

    n_track: int
    nm_fw: int
    n_wheels: int
    n_rv: int

    @property
    def n_flexible_wheel(self) -> int:
        return self.nm_fw * self.n_wheels

    @property
    def n_vehicle_block(self) -> int:
        return self.n_flexible_wheel + self.n_rv

    @property
    def total_dof(self) -> int:
        return self.n_track + self.n_vehicle_block

    @property
    def track(self) -> slice:
        return slice(0, self.n_track)

    @property
    def flexible_wheel(self) -> slice:
        start = self.n_track
        return slice(start, start + self.n_flexible_wheel)

    @property
    def rigid_vehicle(self) -> slice:
        start = self.n_track + self.n_flexible_wheel
        return slice(start, start + self.n_rv)

    @property
    def vehicle_block(self) -> slice:
        return slice(self.n_track, self.total_dof)


@dataclass(frozen=True)
class SystemMatrices:
    """Full system matrices assembled as in the main MATLAB script."""

    Mxt: np.ndarray
    Kxt: np.ndarray
    Cxt: np.ndarray
    layout: SystemDofLayout

    @cached_property
    def Mxt_sparse(self) -> sparse.csr_matrix:
        return sparse.csr_matrix(self.Mxt)

    @cached_property
    def Kxt_sparse(self) -> sparse.csr_matrix:
        return sparse.csr_matrix(self.Kxt)

    @cached_property
    def Cxt_sparse(self) -> sparse.csr_matrix:
        return sparse.csr_matrix(self.Cxt)


@dataclass(frozen=True)
class SparseSystemMatrices:
    """Sparse full-system matrices assembled without dense zero blocks."""

    Mxt: sparse.csr_matrix
    Kxt: sparse.csr_matrix
    Cxt: sparse.csr_matrix
    layout: SystemDofLayout

    @cached_property
    def Mxt_dense(self) -> np.ndarray:
        return self.Mxt.toarray()

    @cached_property
    def Kxt_dense(self) -> np.ndarray:
        return self.Kxt.toarray()

    @cached_property
    def Cxt_dense(self) -> np.ndarray:
        return self.Cxt.toarray()


def assemble_sparse_system_matrices(
    M_track: np.ndarray | sparse.spmatrix,
    K_track: np.ndarray | sparse.spmatrix,
    C_track: np.ndarray | sparse.spmatrix,
    M_vehicle: np.ndarray | sparse.spmatrix,
    K_vehicle: np.ndarray | sparse.spmatrix,
    C_vehicle: np.ndarray | sparse.spmatrix,
    *,
    nm_fw: int = 0,
    n_wheels: int = 4,
    n_rv: int | None = None,
) -> SparseSystemMatrices:
    """Assemble sparse block-diagonal ``Mxt/Kxt/Cxt`` without dense zero blocks."""

    M_track_sp = _as_square_sparse("M_track", M_track)
    K_track_sp = _as_square_sparse("K_track", K_track)
    C_track_sp = _as_square_sparse("C_track", C_track)
    M_vehicle_sp = _as_square_sparse("M_vehicle", M_vehicle)
    K_vehicle_sp = _as_square_sparse("K_vehicle", K_vehicle)
    C_vehicle_sp = _as_square_sparse("C_vehicle", C_vehicle)

    n_track = M_track_sp.shape[0]
    _require_shape("K_track", K_track_sp, (n_track, n_track))
    _require_shape("C_track", C_track_sp, (n_track, n_track))

    n_vehicle_block = M_vehicle_sp.shape[0]
    _require_shape("K_vehicle", K_vehicle_sp, (n_vehicle_block, n_vehicle_block))
    _require_shape("C_vehicle", C_vehicle_sp, (n_vehicle_block, n_vehicle_block))

    flexible_wheel_dof = int(nm_fw) * int(n_wheels)
    resolved_n_rv = n_vehicle_block - flexible_wheel_dof if n_rv is None else int(n_rv)
    if resolved_n_rv < 0:
        raise ValueError("n_rv cannot be negative")
    if flexible_wheel_dof + resolved_n_rv != n_vehicle_block:
        raise ValueError(
            "vehicle block size must equal nm_fw * n_wheels + n_rv "
            f"({n_vehicle_block} != {flexible_wheel_dof} + {resolved_n_rv})"
        )

    layout = SystemDofLayout(
        n_track=n_track,
        nm_fw=int(nm_fw),
        n_wheels=int(n_wheels),
        n_rv=resolved_n_rv,
    )
    return SparseSystemMatrices(
        Mxt=sparse.block_diag((M_track_sp, M_vehicle_sp), format="csr"),
        Kxt=sparse.block_diag((K_track_sp, K_vehicle_sp), format="csr"),
        Cxt=sparse.block_diag((C_track_sp, C_vehicle_sp), format="csr"),
        layout=layout,
    )


def assemble_system_matrices(
    M_track: np.ndarray,
    K_track: np.ndarray,
    C_track: np.ndarray,
    M_vehicle: np.ndarray,
    K_vehicle: np.ndarray,
    C_vehicle: np.ndarray,
    *,
    nm_fw: int = 0,
    n_wheels: int = 4,
    n_rv: int | None = None,
) -> SystemMatrices:
    """Assemble ``Mxt``, ``Kxt``, and ``Cxt`` without wheel-rail force coupling.

    MATLAB order:
    ``[track modal/FEM DOFs, flexible wheel modal DOFs, rigid vehicle DOFs]``.
    In the current RW vehicle route, ``nm_fw == 0`` and the vehicle block is the
    51-DOF CRH380A matrix directly after the track block.
    """

    M_track = _as_square("M_track", M_track)
    K_track = _as_square("K_track", K_track)
    C_track = _as_square("C_track", C_track)
    M_vehicle = _as_square("M_vehicle", M_vehicle)
    K_vehicle = _as_square("K_vehicle", K_vehicle)
    C_vehicle = _as_square("C_vehicle", C_vehicle)

    n_track = M_track.shape[0]
    _require_shape("K_track", K_track, (n_track, n_track))
    _require_shape("C_track", C_track, (n_track, n_track))

    n_vehicle_block = M_vehicle.shape[0]
    _require_shape("K_vehicle", K_vehicle, (n_vehicle_block, n_vehicle_block))
    _require_shape("C_vehicle", C_vehicle, (n_vehicle_block, n_vehicle_block))

    flexible_wheel_dof = int(nm_fw) * int(n_wheels)
    resolved_n_rv = n_vehicle_block - flexible_wheel_dof if n_rv is None else int(n_rv)
    if resolved_n_rv < 0:
        raise ValueError("n_rv cannot be negative")
    if flexible_wheel_dof + resolved_n_rv != n_vehicle_block:
        raise ValueError(
            "vehicle block size must equal nm_fw * n_wheels + n_rv "
            f"({n_vehicle_block} != {flexible_wheel_dof} + {resolved_n_rv})"
        )

    layout = SystemDofLayout(
        n_track=n_track,
        nm_fw=int(nm_fw),
        n_wheels=int(n_wheels),
        n_rv=resolved_n_rv,
    )
    Mxt = np.zeros((layout.total_dof, layout.total_dof), dtype=float)
    Kxt = np.zeros_like(Mxt)
    Cxt = np.zeros_like(Mxt)

    Mxt[layout.track, layout.track] = M_track
    Kxt[layout.track, layout.track] = K_track
    Cxt[layout.track, layout.track] = C_track

    Mxt[layout.vehicle_block, layout.vehicle_block] = M_vehicle
    Kxt[layout.vehicle_block, layout.vehicle_block] = K_vehicle
    Cxt[layout.vehicle_block, layout.vehicle_block] = C_vehicle

    return SystemMatrices(Mxt=Mxt, Kxt=Kxt, Cxt=Cxt, layout=layout)


def build_default_modal_rw_system_matrices(
    *,
    repo_root: str | Path | None = None,
    cut_freq: float | None = None,
    vlc: float | None = None,
    operating_case: DefaultOperatingCase = MATLAB_FULL_DEFAULT_CASE,
) -> tuple[SystemMatrices, ModalTrackMatrices, VehicleMatrices]:
    """Build the MATLAB full default route: 07(009), Face, 350 km/h, FT-Modal."""

    paths = ProjectPaths.from_repo_root(repo_root)
    resolved_cut_freq = operating_case.cut_freq_ft if cut_freq is None else float(cut_freq)
    resolved_vlc = operating_case.vlc if vlc is None else float(vlc)
    track = build_flexible_turnout_modal_matrices(
        paths.modal_turnout_mat,
        cut_freq=resolved_cut_freq,
        choose_turnout=operating_case.choose_turnout,
        matlab_dir=paths.matlab_dir,
    )
    vehicle_parameters = load_vehicle_parameters(paths.default_vehicle_parameters, vlc=resolved_vlc)
    vehicle = build_vehicle_matrices_rw_230409(vehicle_parameters, n_rv=operating_case.n_rv)
    system = assemble_system_matrices(
        track.M_track,
        track.K_track,
        track.C_track,
        vehicle.M_vehicle,
        vehicle.K_vehicle,
        vehicle.C_vehicle,
        nm_fw=operating_case.nm_fw,
        n_wheels=operating_case.n_wheels,
        n_rv=operating_case.n_rv,
    )
    return system, track, vehicle


def build_default_sparse_modal_rw_system_matrices(
    *,
    repo_root: str | Path | None = None,
    cut_freq: float | None = None,
    vlc: float | None = None,
    operating_case: DefaultOperatingCase = MATLAB_FULL_DEFAULT_CASE,
) -> tuple[SparseSystemMatrices, ModalTrackMatrices, VehicleMatrices]:
    """Build the MATLAB full default route using sparse block matrices."""

    paths = ProjectPaths.from_repo_root(repo_root)
    resolved_cut_freq = operating_case.cut_freq_ft if cut_freq is None else float(cut_freq)
    resolved_vlc = operating_case.vlc if vlc is None else float(vlc)
    track = build_flexible_turnout_modal_matrices(
        paths.modal_turnout_mat,
        cut_freq=resolved_cut_freq,
        choose_turnout=operating_case.choose_turnout,
        matlab_dir=paths.matlab_dir,
    )
    vehicle_parameters = load_vehicle_parameters(paths.default_vehicle_parameters, vlc=resolved_vlc)
    vehicle = build_vehicle_matrices_rw_230409(vehicle_parameters, n_rv=operating_case.n_rv)
    system = assemble_sparse_system_matrices(
        track.M_track_sparse,
        track.K_track_sparse,
        track.C_track_sparse,
        vehicle.M_vehicle,
        vehicle.K_vehicle,
        vehicle.C_vehicle,
        nm_fw=operating_case.nm_fw,
        n_wheels=operating_case.n_wheels,
        n_rv=operating_case.n_rv,
    )
    return system, track, vehicle


def _as_square(name: str, matrix: np.ndarray) -> np.ndarray:
    array = np.asarray(matrix, dtype=float)
    if array.ndim != 2 or array.shape[0] != array.shape[1]:
        raise ValueError(f"{name} must be a square 2D matrix, got {array.shape}")
    return array


def _as_square_sparse(name: str, matrix: np.ndarray | sparse.spmatrix) -> sparse.csr_matrix:
    array = sparse.csr_matrix(matrix, dtype=float) if sparse.issparse(matrix) else sparse.csr_matrix(np.asarray(matrix, dtype=float))
    if array.ndim != 2 or array.shape[0] != array.shape[1]:
        raise ValueError(f"{name} must be a square 2D matrix, got {array.shape}")
    return array


def _require_shape(name: str, matrix: np.ndarray, shape: tuple[int, int]) -> None:
    if matrix.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {matrix.shape}")
