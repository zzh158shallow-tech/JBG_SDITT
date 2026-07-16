from __future__ import annotations

import hashlib
import json
import pickle
import time
from copy import deepcopy
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np

from sditt.contact import (
    DefaultTrackContactParameters,
    FullCaseWheelRailContactResult,
    WheelPose2D,
    solve_default_wheel_rail_contact,
    trace_wheel_profile,
)
from sditt.config import DEFAULT_OPERATING_CASE, DefaultOperatingCase, ProjectPaths, SimulationStage
from sditt.integrators import LinearSecondOrderSystem
from sditt.profiles import (
    RailProfileSelector,
    WheelProfileSet,
    build_default_rail_profile_selector,
    build_wheel_profiles,
    rail_profile_numbers,
)
from sditt.track import (
    ModalBeamShapeFunctionContext,
    ModalFTGravityPreload,
    ModalTrackMatrices,
    build_default_07009_face_modal_beam_shape_function_context,
    build_modal_ft_gravity_preload,
    rail_dyn_modal_ft,
    wr_force_modal_ft,
)
from sditt.vehicle import (
    NonlinearDamperResponse,
    VehicleMatrices,
    build_nonlinear_damper_response,
    load_vehicle_parameters,
    wr_force_vehicle_sys_rotation_iii,
)

from .coupled import (
    CoupledIterationSettings,
    CoupledStepCallbacks,
    CoupledStepState,
    CoupledTimeIterationResult,
    StepDtCallback,
)
from .coupled import run_coupled_time_iteration
from .system import (
    SparseSystemMatrices,
    SystemMatrices,
    build_default_modal_rw_system_matrices,
    build_default_sparse_modal_rw_system_matrices,
)


@dataclass(frozen=True)
class MissingFullCaseStage:
    """A MATLAB main-loop stage that is not yet wired into the Python route."""

    name: str
    matlab_reference: str
    reason: str


@dataclass(frozen=True)
class FullCaseSideProfileSnapshot:
    """One wheel/rail side cross-section payload for progress displays."""

    side: str
    wheel_profile: np.ndarray
    rail_profile: np.ndarray
    wheel_contact_points: np.ndarray
    rail_contact_points: np.ndarray


@dataclass(frozen=True)
class FullCaseProfileSnapshot:
    """Lightweight left/right cross-section profile payload for progress displays."""

    wheelset: str
    sides: dict[str, FullCaseSideProfileSnapshot]


@dataclass(frozen=True)
class FullCaseProgressEvent:
    stage: SimulationStage
    step_index: int
    n_steps: int
    time: float
    dt: float
    front_mileage: float
    iterations: int
    normal_error: float
    normal_tangential_error: float
    contact_force_norm: float
    total_force_norm: float
    max_patch_force_z: float
    patch_force_labels: tuple[str, ...] = ()
    patch_force_magnitude: np.ndarray = field(default_factory=lambda: np.zeros((0,), dtype=float))
    patch_vertical_force_z: np.ndarray = field(default_factory=lambda: np.zeros((0,), dtype=float))
    profile_snapshot: FullCaseProfileSnapshot | None = None
    timing: Mapping[str, float] = field(default_factory=dict)
    step_wall_time: float = 0.0
    retry_count: int = 0
    damping_clip_count: int = 0
    damping_clip_max_delta: float = 0.0
    damping_clip_diagnostics: tuple[dict[str, Any], ...] = ()


@dataclass(frozen=True)
class FullDefaultCaseSettings:
    """Controls for the default full-case driver.

    The default straight FT-Modal route now runs as a physically complete
    Python migration of the main MATLAB path. Non-default branches such as
    curve/layout external force still remain guarded by ``missing_stages``.
    """

    cut_freq: float | None = None
    dt: float = 1.0e-4
    n_steps_per_stage: int = 1
    stage_end_mileage: Mapping[SimulationStage, float] | None = None
    use_matlab_mileage_endpoints: bool = False
    use_sparse: bool = True
    fail_on_missing_physics: bool = False
    progress_callback: Callable[[FullCaseProgressEvent], None] | None = None
    frozen_contact_input: "FullCaseFrozenContactInput | None" = None
    preload_cache_dir: str | Path | None = None
    history_retention_steps: int | None = 256
    checkpoint_dir: str | Path | None = None
    save_checkpoints: bool = False
    resume_checkpoint: bool = False
    resume_checkpoint_path: str | Path | None = None
    checkpoint_interval_m: float | None = 10.0
    iteration_settings: CoupledIterationSettings = CoupledIterationSettings(
        max_iterations=11,
        force_tolerance=2.5e-3,
        absolute_force_tolerance=1.0e-6,
        relaxation=1.0,
    )


@dataclass(frozen=True)
class FullCaseFrozenContactInput:
    """Frozen contact-force inputs used before the full contact routine is wired."""

    xlcs: int = 1
    pjcc: np.ndarray = field(default_factory=lambda: np.zeros((16, 1), dtype=float))
    pjch: np.ndarray = field(default_factory=lambda: np.zeros((16, 1), dtype=float))
    prhxf: np.ndarray = field(default_factory=lambda: np.zeros((16, 6), dtype=float))
    con_ws: Any = field(default_factory=lambda: {"FF": {}})


@dataclass(frozen=True)
class FullDefaultCasePreparation:
    """Prepared default system and migration status for the full-case route."""

    paths: ProjectPaths
    operating_case: DefaultOperatingCase
    settings: FullDefaultCaseSettings
    system: SystemMatrices | SparseSystemMatrices
    track: ModalTrackMatrices
    vehicle: VehicleMatrices
    vehicle_parameters: Any
    gravity_preload: ModalFTGravityPreload
    profile_selector: RailProfileSelector
    wheel_profiles: WheelProfileSet
    shape_function_context: ModalBeamShapeFunctionContext
    track_contact_parameters: DefaultTrackContactParameters
    stage_inp_par: dict[SimulationStage, dict[str, object]]
    missing_stages: tuple[MissingFullCaseStage, ...]

    @property
    def total_dof(self) -> int:
        return self.system.layout.total_dof

    @property
    def is_physical_complete(self) -> bool:
        return not self.missing_stages


@dataclass(frozen=True)
class FullCaseRailRecovery:
    """Rail-response recovery returned by the diagnostic callbacks."""

    stage: SimulationStage
    time: float
    front_mileage: float
    modal_displacement: np.ndarray
    modal_velocity: np.ndarray
    modal_acceleration: np.ndarray
    shape_function: dict[str, np.ndarray]
    rail_beam_motion: dict[str, dict[str, np.ndarray]]
    dis_rail: np.ndarray
    vel_rail: np.ndarray
    acc_rail: np.ndarray
    dyn_status_rail: dict[str, np.ndarray]
    physical_rail_recovered: bool
    missing_reason: str | None = None


@dataclass(frozen=True)
class FullCaseContactDiagnostics:
    """Contact-stage diagnostics returned by the diagnostic callbacks."""

    stage: SimulationStage
    time: float
    front_mileage: float
    rail_profile_numbers: dict[str, dict[str, int | None]]
    contact_force_enabled: bool
    missing_stages: tuple[str, ...]
    wheel_rail_contact: FullCaseWheelRailContactResult | None = None


@dataclass(frozen=True)
class FullCaseContactState:
    """Driver-held contact state passed across iterations, steps, and stages."""

    d0_by_wheelset: dict[str, float]
    relvel_max_by_wheelset: dict[str, dict[str, float]] = field(default_factory=dict)
    pjc: np.ndarray | None = None
    pjch: np.ndarray | None = None
    pjcc: np.ndarray | None = None
    prhx: np.ndarray | None = None
    prhxf: np.ndarray | None = None
    con_ws: Any | None = None


@dataclass(frozen=True)
class FullCaseIterationRecord:
    """One stored nonlinear iteration, similar to MATLAB ``Con_Int`` / ``ZP_Int``."""

    stage: SimulationStage
    step_index: int
    iteration: int
    time: float
    dt: float
    front_mileage: float
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    vehicle_displacement: np.ndarray
    vehicle_velocity: np.ndarray
    vehicle_acceleration: np.ndarray
    force_guess: np.ndarray
    contact_force: np.ndarray
    total_force: np.ndarray
    damper_equivalent_force: np.ndarray
    damper_response: NonlinearDamperResponse
    contact_state: FullCaseContactState | None
    rail_response: FullCaseRailRecovery
    rail_profile_numbers: dict[str, dict[str, int | None]]
    pjc: np.ndarray
    pjch: np.ndarray
    pjcc: np.ndarray
    prhx: np.ndarray
    prhxf: np.ndarray
    normal_error: float
    normal_tangential_error: float
    normal_error_per_patch: np.ndarray
    normal_tangential_error_per_patch: np.ndarray
    converged: bool = False


@dataclass(frozen=True)
class FullCaseOutputRow:
    """One accepted time-step output row, similar to MATLAB ``ZP_Dyn`` / ``ZP_Con``."""

    stage: SimulationStage
    step_index: int
    iterations: int
    time: float
    dt: float
    front_mileage: float
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    vehicle_displacement: np.ndarray
    vehicle_velocity: np.ndarray
    vehicle_acceleration: np.ndarray
    contact_force: np.ndarray
    total_force: np.ndarray
    damper_equivalent_force: np.ndarray
    contact_state: FullCaseContactState | None
    rail_response: FullCaseRailRecovery
    rail_profile_numbers: dict[str, dict[str, int | None]]
    pjc: np.ndarray
    pjch: np.ndarray
    pjcc: np.ndarray
    prhx: np.ndarray
    prhxf: np.ndarray
    normal_error: float
    normal_tangential_error: float
    patch_force_y: np.ndarray
    patch_force_z: np.ndarray
    wheelset_lateral_force: np.ndarray
    wheelset_vertical_force: np.ndarray


@dataclass(frozen=True)
class FullDefaultCaseStageResult:
    """One accepted `Preload` or `Cal` stage result."""

    stage: SimulationStage
    history: CoupledTimeIterationResult
    iteration_records: tuple[FullCaseIterationRecord, ...] = ()
    output_rows: tuple[FullCaseOutputRow, ...] = ()

    @property
    def convergence_history(self) -> np.ndarray:
        """Columns: step, iteration, time, dt, mileage, normal err, combined err."""

        if not self.iteration_records:
            return np.zeros((0, 7), dtype=float)
        return np.asarray(
            [
                [
                    float(record.step_index),
                    float(record.iteration),
                    record.time,
                    record.dt,
                    record.front_mileage,
                    record.normal_error,
                    record.normal_tangential_error,
                ]
                for record in self.iteration_records
            ],
            dtype=float,
        )

    @property
    def output_table(self) -> np.ndarray:
        """Columns: step, time, dt, mileage, iterations, normal err, combined err."""

        if not self.output_rows:
            return np.zeros((0, 7), dtype=float)
        return np.asarray(
            [
                [
                    float(row.step_index),
                    row.time,
                    row.dt,
                    row.front_mileage,
                    float(row.iterations),
                    row.normal_error,
                    row.normal_tangential_error,
                ]
                for row in self.output_rows
            ],
            dtype=float,
        )


@dataclass(frozen=True)
class FullDefaultCaseRunResult:
    """Two-stage default driver output and migration-completeness metadata."""

    preparation: FullDefaultCasePreparation
    stages: tuple[FullDefaultCaseStageResult, ...]
    preload_cache_status: str = "disabled"
    preload_cache_path: Path | None = None
    checkpoint_status: str = "disabled"
    checkpoint_path: Path | None = None
    resumed_from_checkpoint: bool = False
    resumed_checkpoint_mileage: float | None = None

    @property
    def is_physical_complete(self) -> bool:
        return self.preparation.is_physical_complete

    @property
    def iteration_records(self) -> tuple[FullCaseIterationRecord, ...]:
        return tuple(record for stage in self.stages for record in stage.iteration_records)

    @property
    def output_rows(self) -> tuple[FullCaseOutputRow, ...]:
        return tuple(row for stage in self.stages for row in stage.output_rows)


class MissingFullCasePhysicsError(RuntimeError):
    """Raised when a strict full-case run is requested before migration is complete."""


_PRELOAD_CACHE_VERSION = 1
_RUN_CHECKPOINT_VERSION = 1


def _preload_cache_path(preparation: FullDefaultCasePreparation) -> Path | None:
    cache_dir = preparation.settings.preload_cache_dir
    if cache_dir is None:
        return None
    key = _preload_cache_key(preparation)
    return Path(cache_dir) / f"preload_{key}.pkl"


def _preload_cache_key(preparation: FullDefaultCasePreparation) -> str:
    fingerprint = _preload_cache_fingerprint(preparation)
    payload = json.dumps(fingerprint, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()[:16]


def _run_checkpoint_dir(preparation: FullDefaultCasePreparation) -> Path | None:
    cache_dir = preparation.settings.checkpoint_dir
    if cache_dir is None:
        return None
    return Path(cache_dir)


def _run_checkpoint_path(preparation: FullDefaultCasePreparation) -> Path | None:
    checkpoint_dir = _run_checkpoint_dir(preparation)
    if checkpoint_dir is None:
        return None
    key = _run_checkpoint_key(preparation)
    return checkpoint_dir / f"checkpoint_{key}.pkl"


def _run_checkpoint_key(preparation: FullDefaultCasePreparation) -> str:
    fingerprint = _run_checkpoint_fingerprint(preparation)
    payload = json.dumps(fingerprint, sort_keys=True, separators=(",", ":"), default=str).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()[:16]


def _run_checkpoint_fingerprint(preparation: FullDefaultCasePreparation) -> dict[str, Any]:
    fingerprint = _preload_cache_fingerprint(preparation)
    fingerprint["version"] = _RUN_CHECKPOINT_VERSION
    fingerprint["stage_end_mileage"] = {
        stage: _stage_end_mileage(preparation, stage) for stage in preparation.operating_case.simulation_stages
    }
    return fingerprint


def _preload_cache_fingerprint(preparation: FullDefaultCasePreparation) -> dict[str, Any]:
    settings = preparation.settings
    operating_case = preparation.operating_case
    paths = preparation.paths
    return {
        "version": _PRELOAD_CACHE_VERSION,
        "operating_case": asdict(operating_case),
        "settings": {
            "cut_freq": settings.cut_freq,
            "dt": settings.dt,
            "n_steps_per_stage": settings.n_steps_per_stage,
            "stage_end_mileage": {str(key): value for key, value in (settings.stage_end_mileage or {}).items()},
            "use_matlab_mileage_endpoints": settings.use_matlab_mileage_endpoints,
            "use_sparse": settings.use_sparse,
            "iteration_settings": asdict(settings.iteration_settings),
        },
        "input_files": {
            "modal_turnout_mat": _file_fingerprint(paths.modal_turnout_mat),
            "rail_pro_mat": _file_fingerprint(paths.rail_pro_mat),
            "baseplate_pro_mat": _file_fingerprint(paths.baseplate_pro_mat),
            "vehicle_parameters": _file_fingerprint(paths.default_vehicle_parameters),
        },
        "stage_end_mileage": _stage_end_mileage(preparation, "Preload"),
        "total_dof": preparation.total_dof,
        "n_track": preparation.system.layout.n_track,
    }


def _file_fingerprint(path: Path) -> dict[str, Any]:
    resolved = Path(path).resolve()
    stat = resolved.stat()
    return {
        "path": str(resolved),
        "size": int(stat.st_size),
        "mtime_ns": int(stat.st_mtime_ns),
    }


def _load_preload_cache(preparation: FullDefaultCasePreparation, cache_path: Path) -> dict[str, Any] | None:
    if not cache_path.exists():
        return None
    try:
        with cache_path.open("rb") as handle:
            payload = pickle.load(handle)
    except Exception:
        return None
    if payload.get("fingerprint") != _preload_cache_fingerprint(preparation):
        return None
    data = payload.get("data")
    return data if isinstance(data, dict) else None


def _load_run_checkpoint(preparation: FullDefaultCasePreparation, checkpoint_path: Path) -> dict[str, Any] | None:
    if not checkpoint_path.exists():
        return None
    try:
        with checkpoint_path.open("rb") as handle:
            payload = pickle.load(handle)
    except Exception:
        return None
    if payload.get("fingerprint") != _run_checkpoint_fingerprint(preparation):
        return None
    data = payload.get("data")
    return data if isinstance(data, dict) else None


def _run_checkpoint_summary(checkpoint_path: Path, data: dict[str, Any]) -> dict[str, Any]:
    try:
        mtime_ns = int(checkpoint_path.stat().st_mtime_ns)
    except OSError:
        mtime_ns = 0
    return {
        "path": checkpoint_path,
        "stage": data.get("stage"),
        "step_index": data.get("step_index"),
        "time": data.get("time"),
        "front_mileage": data.get("front_mileage"),
        "mtime_ns": mtime_ns,
    }


def _run_checkpoint_sort_key(summary: dict[str, Any]) -> tuple[float, float, int, int]:
    return (
        float(summary.get("front_mileage") or 0.0),
        float(summary.get("time") or 0.0),
        int(summary.get("step_index") or 0),
        int(summary.get("mtime_ns") or 0),
    )


def _list_compatible_run_checkpoints(preparation: FullDefaultCasePreparation) -> tuple[dict[str, Any], ...]:
    checkpoint_dir = _run_checkpoint_dir(preparation)
    if checkpoint_dir is None or not checkpoint_dir.exists():
        return ()
    key = _run_checkpoint_key(preparation)
    summaries: list[dict[str, Any]] = []
    for checkpoint_path in checkpoint_dir.glob(f"checkpoint_{key}*.pkl"):
        data = _load_run_checkpoint(preparation, checkpoint_path)
        if data is not None:
            summaries.append(_run_checkpoint_summary(checkpoint_path, data))
    summaries.sort(key=_run_checkpoint_sort_key, reverse=True)
    return tuple(summaries)


def _selected_run_checkpoint(
    preparation: FullDefaultCasePreparation,
) -> tuple[Path, dict[str, Any]] | None:
    checkpoint_path = preparation.settings.resume_checkpoint_path
    if checkpoint_path is not None:
        selected_path = Path(checkpoint_path)
        data = _load_run_checkpoint(preparation, selected_path)
        return None if data is None else (selected_path, data)
    summaries = _list_compatible_run_checkpoints(preparation)
    if not summaries:
        return None
    selected_path = Path(summaries[0]["path"])
    data = _load_run_checkpoint(preparation, selected_path)
    return None if data is None else (selected_path, data)


def find_default_full_case_checkpoint(
    *,
    repo_root: str | Path | None = None,
    settings: FullDefaultCaseSettings | None = None,
    operating_case: DefaultOperatingCase = DEFAULT_OPERATING_CASE,
) -> dict[str, Any] | None:
    summaries = list_default_full_case_checkpoints(
        repo_root=repo_root,
        settings=settings,
        operating_case=operating_case,
    )
    return summaries[0] if summaries else None


def list_default_full_case_checkpoints(
    *,
    repo_root: str | Path | None = None,
    settings: FullDefaultCaseSettings | None = None,
    operating_case: DefaultOperatingCase = DEFAULT_OPERATING_CASE,
) -> tuple[dict[str, Any], ...]:
    preparation = prepare_default_full_case(repo_root=repo_root, settings=settings, operating_case=operating_case)
    return _list_compatible_run_checkpoints(preparation)


def load_default_full_case_checkpoint_summary(
    checkpoint_path: str | Path,
    *,
    repo_root: str | Path | None = None,
    settings: FullDefaultCaseSettings | None = None,
    operating_case: DefaultOperatingCase = DEFAULT_OPERATING_CASE,
) -> dict[str, Any] | None:
    preparation = prepare_default_full_case(repo_root=repo_root, settings=settings, operating_case=operating_case)
    path = Path(checkpoint_path)
    data = _load_run_checkpoint(preparation, path)
    return None if data is None else _run_checkpoint_summary(path, data)


def _run_checkpoint_archive_path(
    preparation: FullDefaultCasePreparation,
    *,
    stage: SimulationStage,
    accepted: Any,
    interval: float,
) -> Path | None:
    checkpoint_dir = _run_checkpoint_dir(preparation)
    if checkpoint_dir is None:
        return None
    rail = accepted.rail_response
    key = _run_checkpoint_key(preparation)
    bucket = _checkpoint_mileage_bucket(float(rail.front_mileage), interval)
    stage_label = "".join(char if char.isalnum() else "_" for char in str(stage))
    mileage_label = f"{float(rail.front_mileage):.3f}".replace("-", "neg").replace(".", "p")
    step_label = f"{int(accepted.step_index):08d}"
    timestamp_label = str(time.time_ns())
    return (
        checkpoint_dir
        / f"checkpoint_{key}_{stage_label}_bucket{bucket:06d}_m{mileage_label}_step{step_label}_{timestamp_label}.pkl"
    )


def _write_run_checkpoint(
    preparation: FullDefaultCasePreparation,
    checkpoint_path: Path,
    *,
    stage: SimulationStage,
    original_stage_start_front_mileage: float,
    accepted: Any,
    progress_events: tuple[FullCaseProgressEvent, ...] = (),
) -> None:
    rail = accepted.rail_response
    geometry = accepted.contact_geometry
    contact = geometry.wheel_rail_contact if geometry is not None else None
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "fingerprint": _run_checkpoint_fingerprint(preparation),
        "data": {
            "stage": stage,
            "original_stage_start_front_mileage": float(original_stage_start_front_mileage),
            "time": float(accepted.time),
            "step_index": int(accepted.step_index),
            "front_mileage": float(rail.front_mileage),
            "displacement": accepted.displacement.copy(),
            "velocity": accepted.velocity.copy(),
            "acceleration": accepted.acceleration.copy(),
            "contact_force": accepted.contact_force.copy(),
            "contact_state": _contact_state_from_contact_result(contact),
            "displacement_history_seed": _copy_cached_array(accepted.displacement_history_seed),
            "velocity_history_seed": _copy_cached_array(accepted.velocity_history_seed),
            "acceleration_history_seed": _copy_cached_array(accepted.acceleration_history_seed),
            "progress_events": tuple(progress_events),
        },
    }
    with checkpoint_path.open("wb") as handle:
        pickle.dump(payload, handle, protocol=pickle.HIGHEST_PROTOCOL)


def _write_preload_cache(
    preparation: FullDefaultCasePreparation,
    cache_path: Path,
    *,
    displacement: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    contact_force: np.ndarray,
    stage_start_front_mileage: float,
    contact_state: FullCaseContactState | None,
    displacement_history_seed: np.ndarray | None,
    velocity_history_seed: np.ndarray | None,
    acceleration_history_seed: np.ndarray | None,
    preload_events: tuple[FullCaseProgressEvent, ...] = (),
) -> None:
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "fingerprint": _preload_cache_fingerprint(preparation),
        "data": {
            "displacement": displacement.copy(),
            "velocity": velocity.copy(),
            "acceleration": acceleration.copy(),
            "contact_force": contact_force.copy(),
            "stage_start_front_mileage": float(stage_start_front_mileage),
            "contact_state": _clone_contact_state(contact_state),
            "displacement_history_seed": _copy_cached_array(displacement_history_seed),
            "velocity_history_seed": _copy_cached_array(velocity_history_seed),
            "acceleration_history_seed": _copy_cached_array(acceleration_history_seed),
            "preload_events": tuple(preload_events),
        },
    }
    with cache_path.open("wb") as handle:
        pickle.dump(payload, handle, protocol=pickle.HIGHEST_PROTOCOL)


def _copy_cached_array(value: np.ndarray | None) -> np.ndarray | None:
    if value is None:
        return None
    return np.asarray(value, dtype=float).copy()


def _append_progress_event_history(events: list[FullCaseProgressEvent], event: FullCaseProgressEvent) -> None:
    if events and events[-1].profile_snapshot is not None:
        events[-1] = replace(events[-1], profile_snapshot=None)
    events.append(event)


def prepare_default_full_case(
    *,
    repo_root: str | Path | None = None,
    settings: FullDefaultCaseSettings | None = None,
    operating_case: DefaultOperatingCase = DEFAULT_OPERATING_CASE,
) -> FullDefaultCasePreparation:
    """Build the default MATLAB case matrices and report remaining migration gaps."""

    settings = settings or FullDefaultCaseSettings()
    paths = ProjectPaths.from_repo_root(repo_root)
    if settings.use_sparse:
        system, track, vehicle = build_default_sparse_modal_rw_system_matrices(
            repo_root=paths.root,
            cut_freq=settings.cut_freq,
            operating_case=operating_case,
        )
    else:
        system, track, vehicle = build_default_modal_rw_system_matrices(
            repo_root=paths.root,
            cut_freq=settings.cut_freq,
            operating_case=operating_case,
        )
    vehicle_parameters = load_vehicle_parameters(
        paths.default_vehicle_parameters,
        vlc=operating_case.vlc,
    )
    gravity_preload = build_modal_ft_gravity_preload(
        modal_mat_path=paths.modal_turnout_mat,
        rail_pro_path=paths.rail_pro_mat,
        baseplate_pro_path=paths.baseplate_pro_mat,
        vehicle_parameters=vehicle_parameters,
        n_track=system.layout.n_track,
        cut_freq=operating_case.cut_freq_ft if settings.cut_freq is None else settings.cut_freq,
        choose_turnout=operating_case.choose_turnout,
        type_rail_all=operating_case.rail_types_all,
        type_baseplate=operating_case.baseplate_types,
        nm_fw=operating_case.nm_fw,
        n_wheels=operating_case.n_wheels,
        n_rv=operating_case.n_rv,
    )
    profile_selector = build_default_rail_profile_selector(
        repo_root=paths.root,
        vehicle_parameters=vehicle_parameters,
        operating_case=operating_case,
    )
    wheel_profiles = build_wheel_profiles(
        paths.wheel_profile_dir,
        vehicle_type="CRH380A",
        drc=float(vehicle_parameters.values["Drc"]),
        dlb=float(vehicle_parameters.values["Dlb"]),
        r0=float(vehicle_parameters.values["R0"]),
    )
    shape_function_context = build_default_07009_face_modal_beam_shape_function_context(
        modal_mat_path=paths.modal_turnout_mat,
        vehicle_parameters=vehicle_parameters,
        cut_freq=operating_case.cut_freq_ft if settings.cut_freq is None else settings.cut_freq,
        operating_case=operating_case,
    )
    shared_inp_par = operating_case.to_inp_par()
    shared_inp_par.update(shape_function_context.inp_par_fields())

    return FullDefaultCasePreparation(
        paths=paths,
        operating_case=operating_case,
        settings=settings,
        system=system,
        track=track,
        vehicle=vehicle,
        vehicle_parameters=vehicle_parameters,
        gravity_preload=gravity_preload,
        profile_selector=profile_selector,
        wheel_profiles=wheel_profiles,
        shape_function_context=shape_function_context,
        track_contact_parameters=_default_contact_track_parameters(),
        stage_inp_par={
            stage: {
                **shared_inp_par,
                "Type_simulation": stage,
            }
            for stage in operating_case.simulation_stages
        },
        missing_stages=_default_missing_stages(operating_case),
    )


def run_default_full_case_driver(
    *,
    repo_root: str | Path | None = None,
    settings: FullDefaultCaseSettings | None = None,
    operating_case: DefaultOperatingCase = DEFAULT_OPERATING_CASE,
) -> FullDefaultCaseRunResult:
    """Run the current Python full-case driver through `Preload -> Cal`.

    The default straight MATLAB route now carries wheel-rail contact,
    nonlinear damper updates, iteration storage, and accepted-step output
    through the coupled loop. Set ``fail_on_missing_physics=True`` to reject
    non-default branches that still list missing stages.
    """

    preparation = prepare_default_full_case(
        repo_root=repo_root,
        settings=settings,
        operating_case=operating_case,
    )
    if preparation.missing_stages and preparation.settings.fail_on_missing_physics:
        names = ", ".join(stage.name for stage in preparation.missing_stages)
        raise MissingFullCasePhysicsError(f"default full-case physics is incomplete: {names}")

    dynamics_system = _linear_system(preparation.system)
    displacement = _initial_preload_displacement(preparation)
    velocity = np.zeros_like(displacement)
    acceleration: np.ndarray | None = None
    stage_start_front_mileage = _initial_front_mileage(preparation, preparation.operating_case.simulation_stages[0])
    contact_state: FullCaseContactState | None = _initial_preload_contact_state(preparation)
    contact_force = _contact_force_from_contact_state(
        preparation,
        preparation.operating_case.simulation_stages[0],
        displacement,
        velocity,
        stage_start_front_mileage + preparation.operating_case.vlc * preparation.settings.dt,
        contact_state,
    )
    stages: list[FullDefaultCaseStageResult] = []
    displacement_history_seed: np.ndarray | None = None
    velocity_history_seed: np.ndarray | None = None
    acceleration_history_seed: np.ndarray | None = None
    preload_cache_path = _preload_cache_path(preparation)
    preload_cache_status = "disabled" if preload_cache_path is None else "miss"
    checkpoint_dir = _run_checkpoint_dir(preparation)
    checkpoint_save_enabled = (
        checkpoint_dir is not None
        and preparation.settings.save_checkpoints
        and preparation.settings.frozen_contact_input is None
    )
    checkpoint_path: Path | None = None
    checkpoint_status = "miss" if checkpoint_save_enabled or preparation.settings.resume_checkpoint else "disabled"
    resumed_from_checkpoint = False
    resumed_checkpoint_mileage: float | None = None
    resume_checkpoint_data: dict[str, Any] | None = None
    stages_to_run = tuple(preparation.operating_case.simulation_stages)
    preload_progress_events: list[FullCaseProgressEvent] = []
    checkpoint_progress_events: list[FullCaseProgressEvent] = []
    if preparation.settings.resume_checkpoint and preparation.settings.frozen_contact_input is None:
        selected_checkpoint = _selected_run_checkpoint(preparation)
        if selected_checkpoint is not None:
            checkpoint_path, checkpoint = selected_checkpoint
            resume_checkpoint_data = checkpoint
            checkpoint_status = "hit"
            resumed_from_checkpoint = True
            resumed_checkpoint_mileage = float(checkpoint["front_mileage"])
            resume_stage = str(checkpoint["stage"])
            displacement = checkpoint["displacement"].copy()
            velocity = checkpoint["velocity"].copy()
            acceleration = checkpoint["acceleration"].copy()
            contact_force = checkpoint["contact_force"].copy()
            stage_start_front_mileage = float(checkpoint["original_stage_start_front_mileage"])
            contact_state = _clone_contact_state(checkpoint["contact_state"])
            displacement_history_seed = _copy_cached_array(checkpoint["displacement_history_seed"])
            velocity_history_seed = _copy_cached_array(checkpoint["velocity_history_seed"])
            acceleration_history_seed = _copy_cached_array(checkpoint["acceleration_history_seed"])
            checkpoint_progress_events = list(tuple(checkpoint.get("progress_events", ())))
            if preparation.settings.progress_callback is not None:
                for event in tuple(checkpoint_progress_events):
                    preparation.settings.progress_callback(event)
            if resume_stage == "Preload":
                stages_to_run = tuple(preparation.operating_case.simulation_stages)
            else:
                stages_to_run = _stages_from(preparation.operating_case.simulation_stages, resume_stage)
    if (
        not resumed_from_checkpoint
        and preload_cache_path is not None
        and preparation.settings.frozen_contact_input is None
    ):
        cached = _load_preload_cache(preparation, preload_cache_path)
        if cached is not None:
            preload_cache_status = "hit"
            displacement = cached["displacement"].copy()
            velocity = cached["velocity"].copy()
            acceleration = cached["acceleration"].copy()
            contact_force = cached["contact_force"].copy()
            stage_start_front_mileage = float(cached["stage_start_front_mileage"])
            contact_state = _clone_contact_state(cached["contact_state"])
            displacement_history_seed = _copy_cached_array(cached["displacement_history_seed"])
            velocity_history_seed = _copy_cached_array(cached["velocity_history_seed"])
            acceleration_history_seed = _copy_cached_array(cached["acceleration_history_seed"])
            if preparation.settings.progress_callback is not None:
                for event in tuple(cached.get("preload_events", ())):
                    preparation.settings.progress_callback(event)
            for event in tuple(cached.get("preload_events", ())):
                _append_progress_event_history(checkpoint_progress_events, event)
            stages_to_run = tuple(stage for stage in stages_to_run if stage != "Preload")

    for stage in stages_to_run:
        stage_step_index0 = 0
        stage_time0 = 0.0
        original_stage_start_front_mileage = stage_start_front_mileage
        if resumed_from_checkpoint and resume_checkpoint_data is not None and checkpoint_status == "hit":
            if str(resume_checkpoint_data["stage"]) == stage:
                stage_step_index0 = int(resume_checkpoint_data["step_index"])
                stage_time0 = float(resume_checkpoint_data["time"])
                original_stage_start_front_mileage = float(resume_checkpoint_data["original_stage_start_front_mileage"])
                stage_start_front_mileage = original_stage_start_front_mileage
        stage_wall_start = time.perf_counter()
        current_front_mileage = stage_start_front_mileage + preparation.operating_case.vlc * stage_time0
        callbacks, build_stage_storage, extract_contact_state = _diagnostic_callbacks(
            preparation,
            stage,
            contact_state=contact_state,
            stage_start_front_mileage=stage_start_front_mileage,
        )
        step_dt_callback = _stage_step_dt_callback(
            preparation,
            stage,
            original_stage_start_front_mileage=original_stage_start_front_mileage,
        )
        if stage_step_index0 == 0:
            first_step_dt = step_dt_callback(stage_step_index0 + 1, stage_time0, preparation.settings.dt)
            contact_force = _contact_force_from_contact_state(
                preparation,
                stage,
                displacement,
                velocity,
                current_front_mileage + preparation.operating_case.vlc * first_step_dt,
                contact_state,
            )
        stage_steps = _stage_step_count(preparation, stage, current_front_mileage)
        stage_display_steps = stage_step_index0 + stage_steps
        if stage_steps == 0:
            stage_start_front_mileage = current_front_mileage
            continue
        mirror_checkpoint_progress = (
            checkpoint_save_enabled
            and preparation.settings.checkpoint_interval_m is not None
            and preparation.settings.checkpoint_interval_m > 0.0
        )
        mirror_preload_progress = (
            stage == "Preload"
            and preload_cache_path is not None
            and preparation.settings.frozen_contact_input is None
        )
        progress_mirror: Callable[[FullCaseProgressEvent], None] | None = None
        if mirror_checkpoint_progress or mirror_preload_progress:

            def progress_mirror(event: FullCaseProgressEvent) -> None:
                if mirror_checkpoint_progress:
                    _append_progress_event_history(checkpoint_progress_events, event)
                if mirror_preload_progress:
                    _append_progress_event_history(preload_progress_events, event)

        progress_callback = _progress_callback(
            preparation,
            stage,
            stage_display_steps,
            build_stage_storage,
            extra_callback=progress_mirror,
        )
        checkpoint_callback = _run_checkpoint_callback(
            preparation,
            stage=stage,
            original_stage_start_front_mileage=original_stage_start_front_mileage,
            initial_front_mileage=current_front_mileage,
            progress_events=lambda: tuple(checkpoint_progress_events),
        )

        def accepted_callback(accepted: Any) -> None:
            nonlocal checkpoint_path, checkpoint_status
            if progress_callback is not None:
                progress_callback(accepted)
            saved_checkpoint_path = None if checkpoint_callback is None else checkpoint_callback(accepted)
            if saved_checkpoint_path is not None:
                checkpoint_path = saved_checkpoint_path
                checkpoint_status = "saved"

        history = run_coupled_time_iteration(
            dynamics_system,
            callbacks,
            dt=preparation.settings.dt,
            n_steps=stage_steps,
            displacement0=displacement,
            velocity0=velocity,
            acceleration0=acceleration,
            displacement_history0=displacement_history_seed,
            velocity_history0=velocity_history_seed,
            acceleration_history0=acceleration_history_seed,
            contact_force0=contact_force,
            settings=preparation.settings.iteration_settings,
            accepted_step_callback=accepted_callback,
            history_retention_steps=preparation.settings.history_retention_steps,
            time0=stage_time0,
            step_index0=stage_step_index0,
            step_dt_callback=step_dt_callback,
        )
        iteration_records, output_rows = build_stage_storage(history)
        contact_state = extract_contact_state(history)
        stages.append(
            FullDefaultCaseStageResult(
                stage=stage,
                history=history,
                iteration_records=iteration_records,
                output_rows=output_rows,
            )
        )
        displacement = history.displacement[-1].copy()
        velocity = history.velocity[-1].copy()
        acceleration = history.acceleration[-1].copy()
        contact_force = history.contact_force[-1].copy()
        stage_start_front_mileage = float(history.rail_response[-1].front_mileage)
        if preparation.total_dof > 1000:
            displacement_history_seed = _park_stage_seed(history.displacement)
            velocity_history_seed = _park_stage_seed(history.velocity)
            acceleration_history_seed = _park_stage_seed(history.acceleration)
        else:
            displacement_history_seed = None
            velocity_history_seed = None
            acceleration_history_seed = None
        history_timing = dict(history.timing)
        history_timing["stage_wall_time"] = time.perf_counter() - stage_wall_start
        object.__setattr__(history, "timing", history_timing)
        if stage == "Preload" and preload_cache_path is not None and preparation.settings.frozen_contact_input is None:
            _write_preload_cache(
                preparation,
                preload_cache_path,
                displacement=displacement,
                velocity=velocity,
                acceleration=acceleration,
                contact_force=contact_force,
                stage_start_front_mileage=stage_start_front_mileage,
                contact_state=contact_state,
                displacement_history_seed=displacement_history_seed,
                velocity_history_seed=velocity_history_seed,
                acceleration_history_seed=acceleration_history_seed,
                preload_events=tuple(preload_progress_events),
            )
            preload_cache_status = "saved"

    return FullDefaultCaseRunResult(
        preparation=preparation,
        stages=tuple(stages),
        preload_cache_status=preload_cache_status,
        preload_cache_path=preload_cache_path,
        checkpoint_status=checkpoint_status,
        checkpoint_path=checkpoint_path,
        resumed_from_checkpoint=resumed_from_checkpoint,
        resumed_checkpoint_mileage=resumed_checkpoint_mileage,
    )


def _linear_system(system: SystemMatrices | SparseSystemMatrices) -> LinearSecondOrderSystem:
    use_sparse = isinstance(system, SparseSystemMatrices)
    return LinearSecondOrderSystem(system.Mxt, system.Cxt, system.Kxt, use_sparse=use_sparse)


def _stages_from(stages: tuple[SimulationStage, ...], start_stage: str) -> tuple[SimulationStage, ...]:
    for index, stage in enumerate(stages):
        if stage == start_stage:
            return stages[index:]
    return stages


def _run_checkpoint_callback(
    preparation: FullDefaultCasePreparation,
    *,
    stage: SimulationStage,
    original_stage_start_front_mileage: float,
    initial_front_mileage: float,
    progress_events: Callable[[], tuple[FullCaseProgressEvent, ...]] | None = None,
) -> Callable[[Any], Path | None] | None:
    interval = preparation.settings.checkpoint_interval_m
    if (
        not preparation.settings.save_checkpoints
        or preparation.settings.checkpoint_dir is None
        or preparation.settings.frozen_contact_input is not None
        or interval is None
        or interval <= 0.0
    ):
        return None
    last_bucket = _checkpoint_mileage_bucket(initial_front_mileage, interval)

    def save_if_crossed(accepted: Any) -> Path | None:
        nonlocal last_bucket
        rail = accepted.rail_response
        current_bucket = _checkpoint_mileage_bucket(float(rail.front_mileage), interval)
        if current_bucket == last_bucket:
            return None
        last_bucket = current_bucket
        checkpoint_path = _run_checkpoint_archive_path(
            preparation,
            stage=stage,
            accepted=accepted,
            interval=interval,
        )
        if checkpoint_path is None:
            return None
        _write_run_checkpoint(
            preparation,
            checkpoint_path,
            stage=stage,
            original_stage_start_front_mileage=original_stage_start_front_mileage,
            accepted=accepted,
            progress_events=() if progress_events is None else progress_events(),
        )
        return checkpoint_path

    return save_if_crossed


def _checkpoint_mileage_bucket(mileage: float, interval: float) -> int:
    return int(np.floor(float(mileage) / float(interval)))


def _stage_step_count(preparation: FullDefaultCasePreparation, stage: SimulationStage, start_mileage: float) -> int:
    end_mileage = _stage_end_mileage(preparation, stage)
    if end_mileage is None:
        return preparation.settings.n_steps_per_stage
    speed = preparation.operating_case.vlc
    if speed == 0.0 or preparation.settings.dt == 0.0:
        raise ValueError("vehicle speed and dt must produce nonzero mileage increment")
    current = float(start_mileage)
    target = float(end_mileage)
    if (speed > 0.0 and current >= target - 1.0e-12) or (speed < 0.0 and current <= target + 1.0e-12):
        return 0
    if (speed > 0.0 and target < current - 1.0e-12) or (speed < 0.0 and target > current + 1.0e-12):
        raise ValueError(
            f"{stage} end mileage {end_mileage} is behind start mileage {start_mileage} for current direction"
        )

    count = 0
    max_count = int(abs((target - current) / (speed * preparation.settings.dt))) * 16 + 1024
    while (speed > 0.0 and current < target - 1.0e-12) or (speed < 0.0 and current > target + 1.0e-12):
        step_dt = _matlab_mileage_step_dt(preparation, current, preparation.settings.dt)
        next_mileage = current + speed * step_dt
        if (speed > 0.0 and next_mileage > target) or (speed < 0.0 and next_mileage < target):
            next_mileage = target
        if abs(next_mileage - current) <= 1.0e-15:
            raise RuntimeError("stage step-count estimation stopped making mileage progress")
        current = next_mileage
        count += 1
        if count > max_count:
            raise RuntimeError("stage step-count estimation exceeded its safety limit")
    return count


def _stage_step_dt_callback(
    preparation: FullDefaultCasePreparation,
    stage: SimulationStage,
    *,
    original_stage_start_front_mileage: float,
) -> StepDtCallback:
    end_mileage = _stage_end_mileage(preparation, stage)

    def callback(_step_index: int, time_current: float, nominal_dt: float) -> float:
        current_mileage = float(original_stage_start_front_mileage) + preparation.operating_case.vlc * float(time_current)
        step_dt = _matlab_mileage_step_dt(preparation, current_mileage, nominal_dt)
        if end_mileage is None:
            return step_dt
        speed = preparation.operating_case.vlc
        next_mileage = current_mileage + speed * step_dt
        if speed > 0.0 and next_mileage > float(end_mileage):
            return max((float(end_mileage) - current_mileage) / speed, np.finfo(float).eps)
        if speed < 0.0 and next_mileage < float(end_mileage):
            return max((float(end_mileage) - current_mileage) / speed, np.finfo(float).eps)
        return step_dt

    return callback


def _matlab_mileage_step_dt(
    preparation: FullDefaultCasePreparation,
    front_mileage: float,
    nominal_dt: float,
) -> float:
    """Apply MATLAB's `Pos_WS` small-step windows around the crossing region."""

    if preparation.operating_case.rail_layout != "turnout":
        return float(nominal_dt)
    if preparation.operating_case.choose_turnout != "07(009)":
        return float(nominal_dt)
    vehicle = preparation.vehicle_parameters.values
    distances = np.array(
        [
            0.0,
            2.0 * float(vehicle["Ll1"]),
            2.0 * float(vehicle["Ll2"]),
            2.0 * (float(vehicle["Ll1"]) + float(vehicle["Ll2"])),
        ],
        dtype=float,
    )
    wheel_positions = float(front_mileage) - distances
    dt = float(nominal_dt)
    if np.any((wheel_positions >= 60.0) & (wheel_positions < 104.0)):
        dt = min(dt, float(nominal_dt) / 2.0)
    if np.any((wheel_positions >= 104.0) & (wheel_positions < 105.0)):
        dt = min(dt, float(nominal_dt) / 4.0)
    return dt


def _stage_end_mileage(preparation: FullDefaultCasePreparation, stage: SimulationStage) -> float | None:
    explicit = preparation.settings.stage_end_mileage
    if explicit is not None and stage in explicit:
        return float(explicit[stage])
    if not preparation.settings.use_matlab_mileage_endpoints:
        return None
    vehicle = preparation.vehicle_parameters.values
    ll_total = float(vehicle["Ll1"]) + float(vehicle["Ll2"])
    if preparation.operating_case.vehicle_direction == "Face":
        if stage == "Preload":
            return 47.5 + 0.1
        if stage == "Cal":
            return 110.0 + 2.0 * ll_total
    else:
        if stage == "Preload":
            return 110.0 + 2.0 * ll_total - 0.1
        if stage == "Cal":
            return 45.0
    return None


def _progress_callback(
    preparation: FullDefaultCasePreparation,
    stage: SimulationStage,
    stage_steps: int,
    build_stage_storage: Callable[[CoupledTimeIterationResult], Any],
    *,
    extra_callback: Callable[[FullCaseProgressEvent], None] | None = None,
) -> Callable[[Any], None] | None:
    del build_stage_storage
    callback = preparation.settings.progress_callback
    if callback is None and extra_callback is None:
        return None

    def emit(accepted: Any) -> None:
        rail = accepted.rail_response
        geometry = accepted.contact_geometry
        contact = geometry.wheel_rail_contact if geometry is not None else None
        if contact is None:
            patch_force_y = np.zeros((0,), dtype=float)
            patch_force_z = np.zeros((0,), dtype=float)
        else:
            patch_force_y = -contact.prhxf[:, 1] - contact.pjch[:, 0]
            patch_force_z = -contact.prhxf[:, 2] - contact.pjcc[:, 0]
        damping_clip_diagnostics = tuple(contact.damping_clip_diagnostics) if contact is not None else ()
        damping_clip_max_delta = max(
            (
                abs(float(item["raw_damping_force"]) - float(item["clipped_damping_force"]))
                for item in damping_clip_diagnostics
            ),
            default=0.0,
        )
        patch_force_magnitude = np.hypot(patch_force_y, patch_force_z)
        patch_force_labels = _progress_patch_force_labels(preparation.stage_inp_par[stage])
        profile_snapshot = _progress_profile_snapshot(preparation, stage, accepted, contact)
        event = FullCaseProgressEvent(
            stage=stage,
            step_index=int(getattr(accepted, "step_index", 0) or 0),
            n_steps=stage_steps,
            time=float(accepted.time),
            dt=float(getattr(accepted, "dt", preparation.settings.dt)),
            front_mileage=float(rail.front_mileage),
            iterations=int(accepted.iterations),
            normal_error=float("nan"),
            normal_tangential_error=float("nan"),
            contact_force_norm=float(np.linalg.norm(accepted.contact_force)),
            total_force_norm=float(np.linalg.norm(accepted.total_force)),
            max_patch_force_z=float(np.max(np.abs(patch_force_z), initial=0.0)),
            patch_force_labels=patch_force_labels,
            patch_force_magnitude=np.asarray(patch_force_magnitude, dtype=float).copy(),
            patch_vertical_force_z=np.asarray(patch_force_z, dtype=float).copy(),
            profile_snapshot=profile_snapshot,
            timing=dict(getattr(accepted, "timing", {}) or {}),
            step_wall_time=float(getattr(accepted, "step_wall_time", 0.0) or 0.0),
            retry_count=int(getattr(accepted, "retry_count", 0) or 0),
            damping_clip_count=len(damping_clip_diagnostics),
            damping_clip_max_delta=damping_clip_max_delta,
            damping_clip_diagnostics=damping_clip_diagnostics,
        )
        if callback is not None:
            callback(event)
        if extra_callback is not None:
            extra_callback(event)

    return emit


def _progress_patch_force_labels(inp_par: Mapping[str, object]) -> tuple[str, ...]:
    wheelsets = tuple(str(wheelset) for wheelset in inp_par.get("Exp_WS", ()))
    dummy_rails = tuple(str(dummy_rail) for dummy_rail in inp_par.get("Exp_DummyRail", ()))
    return tuple(f"{wheelset}-{dummy_rail}" for wheelset in wheelsets for dummy_rail in dummy_rails)


def _progress_profile_snapshot(
    preparation: FullDefaultCasePreparation,
    stage: SimulationStage,
    accepted: Any,
    contact: FullCaseWheelRailContactResult | None,
) -> FullCaseProfileSnapshot | None:
    if contact is None:
        return None

    selection = _progress_profile_selection(contact)
    if selection is None:
        return None
    wheelset, wheel_index = selection

    inp_par = preparation.stage_inp_par[stage]
    pose = _progress_wheel_pose(np.asarray(accepted.displacement, dtype=float), inp_par, wheel_index)
    d0 = float(contact.d0_by_wheelset.get(wheelset, 0.0))

    track_profile = contact.track_profiles.get(wheelset)
    if track_profile is None:
        return None

    con_ws = contact.con_ws.get(wheelset, {}) if isinstance(contact.con_ws, dict) else {}
    sides: dict[str, FullCaseSideProfileSnapshot] = {}
    for side in ("L", "R"):
        if side == "L":
            wheel_profile_source = preparation.wheel_profiles.left
            angle_source = preparation.wheel_profiles.contact_angle_left
        else:
            wheel_profile_source = preparation.wheel_profiles.right
            angle_source = preparation.wheel_profiles.contact_angle_right

        trace = trace_wheel_profile(
            wheel_profile_source,
            angle_source,
            pose,
            dlb=float(preparation.vehicle_parameters.values.get("Dlb", 0.0)),
        )
        wheel_profile = trace.track_points[:, 1:3].copy()
        wheel_profile[:, 1] += pose.vertical + d0
        rail_profile = np.asarray(track_profile.profile.get(side, np.zeros((0, 2), dtype=float)), dtype=float)
        if wheel_profile.size == 0 or rail_profile.size == 0:
            continue

        wheel_contacts = _progress_wheel_contact_points(con_ws, side, pose, d0)
        rail_contacts = _progress_side_array(con_ws, "Con_rail_1", side, columns=2)
        contact_count = min(wheel_contacts.shape[0], rail_contacts.shape[0])
        sides[side] = FullCaseSideProfileSnapshot(
            side=side,
            wheel_profile=_decimate_profile(wheel_profile),
            rail_profile=_decimate_profile(rail_profile),
            wheel_contact_points=wheel_contacts[:contact_count, :],
            rail_contact_points=rail_contacts[:contact_count, :],
        )

    if not sides:
        return None
    return FullCaseProfileSnapshot(wheelset=wheelset, sides=sides)


def _progress_profile_selection(contact: FullCaseWheelRailContactResult) -> tuple[str, int] | None:
    wheelsets = [wheelset for wheelset in contact.track_profiles if wheelset in contact.con_ws]
    if "FF" in wheelsets:
        return "FF", wheelsets.index("FF")

    candidates: list[tuple[float, str, int]] = []
    for wheel_index, wheelset in enumerate(wheelsets):
        con_ws = contact.con_ws.get(wheelset, {})
        if not isinstance(con_ws, dict):
            continue
        wheelset_force = 0.0
        for side in ("L", "R"):
            normal_force = _progress_side_array(con_ws, "Normal_Force", side, columns=4)
            wheelset_force = max(wheelset_force, float(np.max(np.abs(normal_force[:, 0]), initial=0.0)))
        candidates.append((wheelset_force, wheelset, wheel_index))
    if not candidates:
        return None
    _, wheelset, wheel_index = max(candidates, key=lambda item: item[0])
    return wheelset, wheel_index


def _progress_wheel_pose(displacement: np.ndarray, inp_par: Mapping[str, Any], wheel_index: int) -> WheelPose2D:
    base = int(inp_par["N_track"]) + int(inp_par["NM_FW"]) * int(inp_par["Nw"]) + 5 * wheel_index
    return WheelPose2D(
        lateral=_progress_state_value(displacement, base + 1),
        vertical=_progress_state_value(displacement, base + 0),
        roll=_progress_state_value(displacement, base + 2),
        yaw=_progress_state_value(displacement, base + 4),
    )


def _progress_state_value(state: np.ndarray, index: int) -> float:
    if state.ndim == 1:
        return float(state[index])
    if state.ndim == 2 and state.shape[1] > 3:
        return float(state[index, 3])
    if state.ndim == 2 and state.shape[1] == 1:
        return float(state[index, 0])
    raise ValueError("state arrays must be vectors, single-column arrays, or MATLAB-style arrays with column 4")


def _progress_wheel_contact_points(con_ws: Mapping[str, Any], side: str, pose: WheelPose2D, d0: float) -> np.ndarray:
    local = _progress_side_array(con_ws, "Con_wheel_2_full", side, columns=3)
    if local.size == 0:
        local = _progress_side_array(con_ws, "Con_wheel_2", side, columns=3)
    if local.size == 0:
        return np.zeros((0, 2), dtype=float)
    orientation = _progress_wheelset_orientation(pose.roll, pose.yaw)
    track = local[:, :3] @ orientation + np.array([0.0, pose.lateral, 0.0], dtype=float)
    points = track[:, 1:3].copy()
    points[:, 1] += pose.vertical + d0
    return points


def _progress_side_array(con_ws: Mapping[str, Any], key: str, side: str, *, columns: int) -> np.ndarray:
    container = con_ws.get(key, {}) if isinstance(con_ws, Mapping) else {}
    value = container.get(side, np.zeros((0, columns), dtype=float)) if isinstance(container, Mapping) else container
    array = np.asarray(value, dtype=float)
    if array.size == 0:
        return np.zeros((0, columns), dtype=float)
    return array.reshape((-1, array.shape[-1]))[:, :columns]


def _progress_wheelset_orientation(roll: float, yaw: float) -> np.ndarray:
    return np.array(
        [
            [np.cos(yaw), np.sin(yaw), 0.0],
            [-np.cos(roll) * np.sin(yaw), np.cos(roll) * np.cos(yaw), np.sin(roll)],
            [np.sin(roll) * np.sin(yaw), -np.sin(roll) * np.cos(yaw), np.cos(roll)],
        ],
        dtype=float,
    )


def _decimate_profile(profile: np.ndarray, *, max_points: int = 600) -> np.ndarray:
    points = np.asarray(profile, dtype=float)
    if points.shape[0] <= max_points:
        return points.copy()
    indexes = np.linspace(0, points.shape[0] - 1, max_points).astype(int)
    return points[indexes, :].copy()


def _park_stage_seed(history: np.ndarray) -> np.ndarray:
    values = np.asarray(history, dtype=float)
    if values.shape[0] >= 3:
        return values[-3:, :].copy()
    return np.repeat(values[-1:, :], 3, axis=0)


def _diagnostic_callbacks(
    preparation: FullDefaultCasePreparation,
    stage: SimulationStage,
    *,
    contact_state: FullCaseContactState | None,
    stage_start_front_mileage: float,
) -> tuple[
    CoupledStepCallbacks,
    Callable[[CoupledTimeIterationResult], tuple[tuple[FullCaseIterationRecord, ...], tuple[FullCaseOutputRow, ...]]],
    Callable[[CoupledTimeIterationResult], FullCaseContactState | None],
]:
    layout = preparation.system.layout
    missing_names = tuple(item.name for item in preparation.missing_stages)
    iteration_records: list[FullCaseIterationRecord] = []
    last_accepted_contact_state: FullCaseContactState | None = None
    previous_step_key: tuple[int, float] | None = None
    previous_normal_force: np.ndarray | None = None
    previous_combined_force_norm: np.ndarray | None = None
    step_contact_state: dict[int, FullCaseContactState] = {}
    step_result_contact_state: dict[int, FullCaseContactState] = {}
    previous_iteration_pjc: dict[int, np.ndarray] = {}
    reuse_contact_state_within_step = preparation.settings.cut_freq is None
    history_retention_steps = preparation.settings.history_retention_steps

    def prune_step_contact_working_state(step_index: int) -> None:
        for key in tuple(step_contact_state):
            if key < step_index:
                del step_contact_state[key]
        for key in tuple(step_result_contact_state):
            if key < step_index - 1:
                del step_result_contact_state[key]
        for key in tuple(previous_iteration_pjc):
            if key < step_index:
                del previous_iteration_pjc[key]
        if history_retention_steps is not None:
            first_kept_step = step_index - int(history_retention_steps) + 1
            if first_kept_step > 1:
                del iteration_records[: next(
                    (index for index, record in enumerate(iteration_records) if record.step_index >= first_kept_step),
                    len(iteration_records),
                )]

    def recover_track_response(state: CoupledStepState) -> FullCaseRailRecovery:
        front_mileage = stage_start_front_mileage + preparation.operating_case.vlc * state.time
        shape_function, rail_beam_motion = preparation.shape_function_context.evaluate(front_mileage)
        dis_rail, vel_rail, acc_rail, dyn_status_rail = rail_dyn_modal_ft(
            preparation.stage_inp_par[stage],
            state.displacement[layout.track],
            state.velocity[layout.track],
            state.acceleration[layout.track],
            shape_function,
        )
        return FullCaseRailRecovery(
            stage=stage,
            time=state.time,
            front_mileage=front_mileage,
            modal_displacement=state.displacement[layout.track].copy(),
            modal_velocity=state.velocity[layout.track].copy(),
            modal_acceleration=state.acceleration[layout.track].copy(),
            shape_function=shape_function,
            rail_beam_motion=rail_beam_motion,
            dis_rail=dis_rail,
            vel_rail=vel_rail,
            acc_rail=acc_rail,
            dyn_status_rail=dyn_status_rail,
            physical_rail_recovered=True,
        )

    def contact_geometry(state: CoupledStepState, rail: FullCaseRailRecovery) -> FullCaseContactDiagnostics:
        prune_step_contact_working_state(state.step_index)
        selected_profiles = preparation.profile_selector.select(rail.front_mileage)
        wheel_rail_contact = None
        if preparation.settings.frozen_contact_input is None:
            if state.step_index not in step_contact_state:
                step_contact_state[state.step_index] = (
                    _clone_contact_state(step_result_contact_state.get(state.step_index - 1))
                    or _clone_contact_state(contact_state)
                    or FullCaseContactState(d0_by_wheelset={})
                )
            step_input_state = _clone_contact_state(step_contact_state[state.step_index]) or FullCaseContactState(
                d0_by_wheelset={}
            )
            previous_relvel_max_by_wheelset = (
                step_input_state.relvel_max_by_wheelset or None
                if step_input_state is not None
                else None
            )
            wheel_rail_contact = solve_default_wheel_rail_contact(
                preparation.stage_inp_par[stage],
                preparation.vehicle_parameters.values,
                preparation.wheel_profiles,
                selected_profiles,
                rail,
                state.displacement,
                state.velocity,
                front_mileage=rail.front_mileage,
                track_parameters=preparation.track_contact_parameters,
                fixed_d0_by_wheelset=step_input_state.d0_by_wheelset or None,
                previous_relvel_max_by_wheelset=previous_relvel_max_by_wheelset,
                use_cal_d0_trace=preparation.total_dof > 1000,
            )
            prior_pjc = previous_iteration_pjc.get(state.step_index)
            if prior_pjc is None and step_input_state.pjc is not None:
                prior_pjc = np.asarray(step_input_state.pjc, dtype=float)
            if prior_pjc is not None and prior_pjc.shape == wheel_rail_contact.pjc.shape:
                wheel_rail_contact.pjc[:, 0] = prior_pjc[:, 1]
            previous_iteration_pjc[state.step_index] = wheel_rail_contact.pjc.copy()
            step_result_contact_state[state.step_index] = FullCaseContactState(
                d0_by_wheelset={
                    wheelset: float(value) for wheelset, value in wheel_rail_contact.d0_by_wheelset.items()
                },
                relvel_max_by_wheelset={
                    wheelset: {dummy_rail: float(value) for dummy_rail, value in relvel.items()}
                    for wheelset, relvel in wheel_rail_contact.relvel_max_by_wheelset.items()
                },
                pjc=wheel_rail_contact.pjc.copy(),
                pjch=wheel_rail_contact.pjch.copy(),
                pjcc=wheel_rail_contact.pjcc.copy(),
                prhx=wheel_rail_contact.prhx.copy(),
                prhxf=wheel_rail_contact.prhxf.copy(),
                con_ws=deepcopy(wheel_rail_contact.con_ws),
            )
            if not step_input_state.d0_by_wheelset or not reuse_contact_state_within_step:
                step_contact_state[state.step_index] = FullCaseContactState(
                    d0_by_wheelset={
                        wheelset: float(value)
                        for wheelset, value in step_result_contact_state[state.step_index].d0_by_wheelset.items()
                    },
                    relvel_max_by_wheelset={
                        wheelset: {dummy_rail: float(value) for dummy_rail, value in relvel.items()}
                        for wheelset, relvel in step_input_state.relvel_max_by_wheelset.items()
                    },
                    pjc=_copy_array_or_none(step_input_state.pjc),
                    pjch=_copy_array_or_none(step_input_state.pjch),
                    pjcc=_copy_array_or_none(step_input_state.pjcc),
                    prhx=_copy_array_or_none(step_input_state.prhx),
                    prhxf=_copy_array_or_none(step_input_state.prhxf),
                    con_ws=deepcopy(step_input_state.con_ws),
                )
        return FullCaseContactDiagnostics(
            stage=stage,
            time=state.time,
            front_mileage=rail.front_mileage,
            rail_profile_numbers=rail_profile_numbers(selected_profiles),
            contact_force_enabled=True,
            wheel_rail_contact=wheel_rail_contact,
            missing_stages=missing_names,
        )

    def contact_force(
        state: CoupledStepState,
        rail: FullCaseRailRecovery,
        geometry: FullCaseContactDiagnostics,
    ) -> np.ndarray:
        nonlocal previous_step_key, previous_normal_force, previous_combined_force_norm
        frozen_input = preparation.settings.frozen_contact_input
        contact_input = geometry.wheel_rail_contact
        if frozen_input is None and contact_input is None:
            return np.zeros(preparation.total_dof, dtype=float)
        if frozen_input is None:
            frozen_input = FullCaseFrozenContactInput(
                xlcs=contact_input.xlcs,
                pjcc=contact_input.pjcc,
                pjch=contact_input.pjch,
                prhxf=contact_input.prhxf,
                con_ws=contact_input.con_ws,
            )

        damper_response = build_nonlinear_damper_response(
            preparation.stage_inp_par[stage],
            preparation.vehicle_parameters.values,
            preparation.vehicle,
            state.displacement,
            state.velocity,
        )
        pxt = damper_response.equivalent_force.reshape(-1, 1)
        pxt, _ = wr_force_modal_ft(
            preparation.stage_inp_par[stage],
            int(frozen_input.xlcs),
            pxt,
            frozen_input.pjcc,
            frozen_input.pjch,
            frozen_input.prhxf,
            frozen_input.con_ws,
            rail.shape_function,
        )
        pxt, _ = wr_force_vehicle_sys_rotation_iii(
            preparation.stage_inp_par[stage],
            preparation.vehicle_parameters.values,
            _default_track_force_mapping_parameters(),
            {},
            pxt,
            np.zeros_like(state.displacement),
            frozen_input.pjcc,
            frozen_input.pjch,
            frozen_input.prhxf,
            frozen_input.con_ws,
        )
        contact_force_vector = pxt[:, 0]

        step_key = (state.step_index, state.dt)
        if previous_step_key != step_key:
            previous_step_key = step_key
            seed_contact_state = step_contact_state.get(state.step_index)
            previous_normal_force = _contact_state_normal_force(seed_contact_state)
            previous_combined_force_norm = _contact_state_combined_force_norm(seed_contact_state)

        pjc, pjch, pjcc, prhx, prhxf = _storage_contact_arrays(contact_input, frozen_input)
        current_normal_force = _storage_normal_force(pjc, pjcc)
        current_combined_force_norm = _storage_combined_force_norm(pjch, pjcc, prhxf)
        normal_error, normal_error_per_patch = _storage_relative_error(previous_normal_force, current_normal_force)
        normal_tan_error, normal_tan_error_per_patch = _storage_relative_error(
            previous_combined_force_norm,
            current_combined_force_norm,
        )
        previous_normal_force = current_normal_force.copy()
        previous_combined_force_norm = current_combined_force_norm.copy()

        iteration_records.append(
            FullCaseIterationRecord(
                stage=stage,
                step_index=state.step_index,
                iteration=state.iteration,
                time=state.time,
                dt=state.dt,
                front_mileage=rail.front_mileage,
                displacement=state.displacement.copy(),
                velocity=state.velocity.copy(),
                acceleration=state.acceleration.copy(),
                vehicle_displacement=state.displacement[layout.rigid_vehicle].copy(),
                vehicle_velocity=state.velocity[layout.rigid_vehicle].copy(),
                vehicle_acceleration=state.acceleration[layout.rigid_vehicle].copy(),
                force_guess=state.force_guess.copy(),
                contact_force=contact_force_vector.copy(),
                total_force=preparation.gravity_preload.pxt_gravity + contact_force_vector,
                damper_equivalent_force=damper_response.equivalent_force.copy(),
                damper_response=damper_response,
                contact_state=_clone_contact_state(step_result_contact_state.get(state.step_index)),
                rail_response=rail,
                rail_profile_numbers=_copy_rail_profile_numbers(geometry.rail_profile_numbers),
                pjc=pjc.copy(),
                pjch=pjch.copy(),
                pjcc=pjcc.copy(),
                prhx=prhx.copy(),
                prhxf=prhxf.copy(),
                normal_error=normal_error,
                normal_tangential_error=normal_tan_error,
                normal_error_per_patch=normal_error_per_patch.copy(),
                normal_tangential_error_per_patch=normal_tan_error_per_patch.copy(),
            )
        )
        return contact_force_vector

    def converged(
        state: CoupledStepState,
        _rail: FullCaseRailRecovery,
        _geometry: FullCaseContactDiagnostics,
        _contact_force: np.ndarray,
        _force_guess: np.ndarray,
        settings: CoupledIterationSettings,
    ) -> bool:
        if not iteration_records:
            return False
        record = iteration_records[-1]
        if record.step_index != state.step_index or record.iteration != state.iteration or record.dt != state.dt:
            return False
        return (
            record.normal_error <= settings.force_tolerance
            and record.normal_tangential_error <= settings.force_tolerance
        ) or (
            record.normal_error <= settings.absolute_force_tolerance
            and record.normal_tangential_error <= settings.absolute_force_tolerance
        )

    def build_stage_storage(
        history: CoupledTimeIterationResult,
    ) -> tuple[tuple[FullCaseIterationRecord, ...], tuple[FullCaseOutputRow, ...]]:
        nonlocal last_accepted_contact_state
        accepted_keys = {
            (
                int(step_index),
                int(iterations),
                float(dt),
                float(time_value),
            )
            for step_index, iterations, dt, time_value in zip(
                history.step_index,
                history.iterations,
                history.dt,
                history.time,
                strict=True,
            )
            if int(step_index) != 0
        }
        stored_iterations: list[FullCaseIterationRecord] = []
        accepted_by_step: dict[int, FullCaseIterationRecord] = {}
        for record in iteration_records:
            key = (record.step_index, record.iteration, record.dt, record.time)
            if key in accepted_keys:
                record = replace(record, converged=True)
                accepted_by_step[record.step_index] = record
            stored_iterations.append(record)

        output_rows = tuple(
            _build_output_row(
                preparation,
                accepted_by_step[step_index],
                iterations=int(iterations),
            )
            for step_index, iterations in zip(history.step_index, history.iterations, strict=True)
            if int(step_index) != 0 and int(step_index) in accepted_by_step
        )
        last_accepted_contact_state = _clone_contact_state(output_rows[-1].contact_state) if output_rows else None
        return tuple(stored_iterations), output_rows

    def extract_contact_state(history: CoupledTimeIterationResult) -> FullCaseContactState | None:
        del history
        return _clone_contact_state(last_accepted_contact_state or contact_state)

    return (
        CoupledStepCallbacks(
            recover_track_response=recover_track_response,
            contact_geometry=contact_geometry,
            contact_force=contact_force,
            external_force=lambda _time: preparation.gravity_preload.pxt_gravity.copy(),
            converged=converged,
        ),
        build_stage_storage,
        extract_contact_state,
    )


def _default_missing_stages(operating_case: DefaultOperatingCase) -> tuple[MissingFullCaseStage, ...]:
    missing: list[MissingFullCaseStage] = []
    if operating_case.layout_type != "Straight":
        missing.append(
            MissingFullCaseStage(
                name="curve_external_force",
                matlab_reference="External_Force_Curve_210227_v4.m",
                reason="Curve/layout external force terms are not yet part of the Python driver for non-straight layouts.",
            )
        )
    return tuple(missing)


def _initial_front_mileage(
    preparation: FullDefaultCasePreparation,
    stage: SimulationStage,
) -> float:
    if stage == "Preload" and preparation.operating_case.vehicle_direction == "Face":
        start = 32.0 - 2.0 * (preparation.operating_case.vlc * 3.6 - 350.0) / 50.0
    else:
        start = 32.0
    return float(start)


def _default_track_force_mapping_parameters() -> dict[str, float]:
    return {"Br": 0.7175}


def _default_contact_track_parameters() -> DefaultTrackContactParameters:
    return DefaultTrackContactParameters()


def _initial_preload_contact_state(preparation: FullDefaultCasePreparation) -> FullCaseContactState:
    operating_case = preparation.operating_case
    n_patches = operating_case.n_contact_patch * operating_case.n_wheels
    pjc = np.zeros((n_patches, 2), dtype=float)
    pjch = np.zeros((n_patches, 1), dtype=float)
    pjcc = np.zeros((n_patches, 1), dtype=float)
    prhx = np.zeros((n_patches, 3), dtype=float)
    prhxf = np.zeros((n_patches, 6), dtype=float)
    seeded = np.zeros((n_patches,), dtype=bool)
    seeded[0::operating_case.n_contact_patch] = True
    if operating_case.vehicle_direction == "Face":
        seeded[1::operating_case.n_contact_patch] = True
    else:
        seeded[operating_case.n_contact_patch - 1 :: operating_case.n_contact_patch] = True
    wheel_load = _nominal_static_wheel_load(preparation.vehicle_parameters.values)
    pjc[seeded, :] = wheel_load
    pjcc[seeded, 0] = -wheel_load
    return FullCaseContactState(
        d0_by_wheelset=_initial_unloaded_d0_by_wheelset(preparation),
        pjc=pjc,
        pjch=pjch,
        pjcc=pjcc,
        prhx=prhx,
        prhxf=prhxf,
        con_ws={"FF": {}},
    )


def _initial_unloaded_d0_by_wheelset(preparation: FullDefaultCasePreparation) -> dict[str, float]:
    stage = preparation.operating_case.simulation_stages[0]
    front_mileage = _initial_front_mileage(preparation, stage)
    shape_function, rail_beam_motion = preparation.shape_function_context.evaluate(front_mileage)
    zero_track = np.zeros(preparation.system.layout.n_track, dtype=float)
    dis_rail, vel_rail, acc_rail, dyn_status_rail = rail_dyn_modal_ft(
        preparation.stage_inp_par[stage],
        zero_track,
        zero_track,
        zero_track,
        shape_function,
    )
    rail_response = FullCaseRailRecovery(
        stage=stage,
        time=0.0,
        front_mileage=front_mileage,
        modal_displacement=zero_track.copy(),
        modal_velocity=zero_track.copy(),
        modal_acceleration=zero_track.copy(),
        shape_function=shape_function,
        rail_beam_motion=rail_beam_motion,
        dis_rail=dis_rail,
        vel_rail=vel_rail,
        acc_rail=acc_rail,
        dyn_status_rail=dyn_status_rail,
        physical_rail_recovered=True,
    )
    zero_state = np.zeros(preparation.total_dof, dtype=float)
    contact = solve_default_wheel_rail_contact(
        preparation.stage_inp_par[stage],
        preparation.vehicle_parameters.values,
        preparation.wheel_profiles,
        preparation.profile_selector.select(front_mileage),
        rail_response,
        zero_state,
        zero_state,
        front_mileage=front_mileage,
        track_parameters=preparation.track_contact_parameters,
    )
    return {wheelset: float(value) for wheelset, value in contact.d0_by_wheelset.items()}


def _initial_preload_displacement(preparation: FullDefaultCasePreparation) -> np.ndarray:
    """Seed the same static vehicle deflections as MATLAB's no-Pre branch."""

    displacement = np.zeros(preparation.total_dof, dtype=float)
    vehicle_parameters = preparation.vehicle_parameters.values
    operating_case = preparation.operating_case
    layout = preparation.system.layout
    base = layout.rigid_vehicle.start
    wheel_load = _nominal_static_wheel_load(vehicle_parameters)
    zw = wheel_load ** (2.0 / 3.0) * 3.86e-8 * float(vehicle_parameters["R0"]) ** (-0.115)
    zt = (
        (float(vehicle_parameters["Mb"]) / 2.0 + float(vehicle_parameters["Mc"]) / 4.0)
        * 9.81
        / (2.0 * float(vehicle_parameters["K1z"]))
        + zw
    )
    zc = float(vehicle_parameters["Mc"]) / 4.0 * 9.81 / float(vehicle_parameters["K2z"]) + zt

    for offset in (0, 5, 10, 15):
        _set_if_in_bounds(displacement, base + offset, zw)
    for offset in (20, 25):
        _set_if_in_bounds(displacement, base + offset, zt)
    _set_if_in_bounds(displacement, base + 30, zc)
    for offset in range(35, 43):
        _set_if_in_bounds(displacement, base + offset, zw)
    if operating_case.vehicle_type == "CR400BF":
        for offset in range(55, 63):
            _set_if_in_bounds(displacement, base + offset, zt)
    return displacement


def _set_if_in_bounds(array: np.ndarray, index: int, value: float) -> None:
    if 0 <= index < array.size:
        array[index] = value


def _nominal_static_wheel_load(vehicle_parameters: Any) -> float:
    return 9.81 * (
        float(vehicle_parameters["Mw"]) / 2.0
        + float(vehicle_parameters["Mb"]) / 4.0
        + float(vehicle_parameters["Mc"]) / 8.0
    )


def _contact_force_from_contact_state(
    preparation: FullDefaultCasePreparation,
    stage: SimulationStage,
    displacement: np.ndarray,
    velocity: np.ndarray,
    front_mileage: float,
    contact_state: FullCaseContactState | None,
) -> np.ndarray:
    if contact_state is None or contact_state.pjcc is None or contact_state.pjch is None or contact_state.prhxf is None:
        return np.zeros(preparation.total_dof, dtype=float)
    shape_function, _ = preparation.shape_function_context.evaluate(front_mileage)
    damper_response = build_nonlinear_damper_response(
        preparation.stage_inp_par[stage],
        preparation.vehicle_parameters.values,
        preparation.vehicle,
        displacement,
        velocity,
    )
    pxt = damper_response.equivalent_force.reshape(-1, 1)
    pxt, _ = wr_force_modal_ft(
        preparation.stage_inp_par[stage],
        1,
        pxt,
        contact_state.pjcc,
        contact_state.pjch,
        contact_state.prhxf,
        contact_state.con_ws or {"FF": {}},
        shape_function,
    )
    pxt, _ = wr_force_vehicle_sys_rotation_iii(
        preparation.stage_inp_par[stage],
        preparation.vehicle_parameters.values,
        _default_track_force_mapping_parameters(),
        {},
        pxt,
        np.zeros_like(displacement),
        contact_state.pjcc,
        contact_state.pjch,
        contact_state.prhxf,
        contact_state.con_ws or {"FF": {}},
    )
    return pxt[:, 0]


def _clone_contact_state(contact_state: FullCaseContactState | None) -> FullCaseContactState | None:
    if contact_state is None:
        return None
    return FullCaseContactState(
        d0_by_wheelset={wheelset: float(value) for wheelset, value in contact_state.d0_by_wheelset.items()},
        relvel_max_by_wheelset={
            wheelset: {dummy_rail: float(value) for dummy_rail, value in relvel.items()}
            for wheelset, relvel in contact_state.relvel_max_by_wheelset.items()
        },
        pjc=_copy_array_or_none(contact_state.pjc),
        pjch=_copy_array_or_none(contact_state.pjch),
        pjcc=_copy_array_or_none(contact_state.pjcc),
        prhx=_copy_array_or_none(contact_state.prhx),
        prhxf=_copy_array_or_none(contact_state.prhxf),
        con_ws=deepcopy(contact_state.con_ws),
    )


def _contact_state_from_contact_result(contact: FullCaseWheelRailContactResult | None) -> FullCaseContactState | None:
    if contact is None:
        return None
    return FullCaseContactState(
        d0_by_wheelset={wheelset: float(value) for wheelset, value in contact.d0_by_wheelset.items()},
        relvel_max_by_wheelset={
            wheelset: {dummy_rail: float(value) for dummy_rail, value in relvel.items()}
            for wheelset, relvel in contact.relvel_max_by_wheelset.items()
        },
        pjc=np.asarray(contact.pjc, dtype=float).copy(),
        pjch=np.asarray(contact.pjch, dtype=float).copy(),
        pjcc=np.asarray(contact.pjcc, dtype=float).copy(),
        prhx=np.asarray(contact.prhx, dtype=float).copy(),
        prhxf=np.asarray(contact.prhxf, dtype=float).copy(),
        con_ws=deepcopy(contact.con_ws),
    )


def _copy_array_or_none(value: np.ndarray | None) -> np.ndarray | None:
    if value is None:
        return None
    return np.asarray(value, dtype=float).copy()


def _contact_state_normal_force(contact_state: FullCaseContactState | None) -> np.ndarray | None:
    if contact_state is None or contact_state.pjcc is None:
        return None
    pjc = np.zeros((contact_state.pjcc.shape[0], 2), dtype=float) if contact_state.pjc is None else contact_state.pjc
    return _storage_normal_force(np.asarray(pjc, dtype=float), np.asarray(contact_state.pjcc, dtype=float)).copy()


def _contact_state_combined_force_norm(contact_state: FullCaseContactState | None) -> np.ndarray | None:
    if contact_state is None or contact_state.pjch is None or contact_state.pjcc is None or contact_state.prhxf is None:
        return None
    return _storage_combined_force_norm(
        np.asarray(contact_state.pjch, dtype=float),
        np.asarray(contact_state.pjcc, dtype=float),
        np.asarray(contact_state.prhxf, dtype=float),
    ).copy()


def _storage_contact_arrays(
    contact_input: FullCaseWheelRailContactResult | None,
    frozen_input: FullCaseFrozenContactInput,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if contact_input is not None:
        return (
            np.asarray(contact_input.pjc, dtype=float),
            np.asarray(contact_input.pjch, dtype=float),
            np.asarray(contact_input.pjcc, dtype=float),
            np.asarray(contact_input.prhx, dtype=float),
            np.asarray(contact_input.prhxf, dtype=float),
        )
    patch_count = np.asarray(frozen_input.pjcc, dtype=float).shape[0]
    return (
        np.zeros((patch_count, 2), dtype=float),
        np.asarray(frozen_input.pjch, dtype=float),
        np.asarray(frozen_input.pjcc, dtype=float),
        np.zeros((patch_count, 3), dtype=float),
        np.asarray(frozen_input.prhxf, dtype=float),
    )


def _storage_normal_force(pjc: np.ndarray, pjcc: np.ndarray) -> np.ndarray:
    if pjc.ndim == 2 and pjc.shape[1] >= 2 and np.any(np.abs(pjc[:, 1]) > 0.0):
        return np.asarray(pjc[:, 1], dtype=float)
    if pjcc.ndim == 2 and pjcc.shape[1] >= 1:
        return np.asarray(pjcc[:, 0], dtype=float)
    return np.zeros((0,), dtype=float)


def _storage_combined_force_norm(pjch: np.ndarray, pjcc: np.ndarray, prhxf: np.ndarray) -> np.ndarray:
    if prhxf.size == 0:
        return np.zeros((0,), dtype=float)
    combined = np.zeros((prhxf.shape[0], 3), dtype=float)
    combined[:, 0] = prhxf[:, 0]
    combined[:, 1] = prhxf[:, 1] + np.asarray(pjch[:, 0], dtype=float)
    combined[:, 2] = prhxf[:, 2] + np.asarray(pjcc[:, 0], dtype=float)
    return np.linalg.norm(combined, axis=1)


def _storage_relative_error(
    previous: np.ndarray | None,
    current: np.ndarray,
) -> tuple[float, np.ndarray]:
    current = np.asarray(current, dtype=float).reshape(-1)
    if current.size == 0:
        return 0.0, current.copy()
    if previous is None:
        errors = np.ones_like(current, dtype=float)
        return 1.0, errors

    previous = np.asarray(previous, dtype=float).reshape(-1)
    denom = np.maximum(np.abs(current), np.finfo(float).eps)
    errors = np.abs(current - previous) / denom
    both_zero = (np.abs(current) <= np.finfo(float).eps) & (np.abs(previous) <= np.finfo(float).eps)
    errors[both_zero] = 0.0
    return float(np.max(errors, initial=0.0)), errors


def _copy_rail_profile_numbers(
    values: dict[str, dict[str, int | None]],
) -> dict[str, dict[str, int | None]]:
    return {
        rail: {wheelset: profile_num for wheelset, profile_num in by_station.items()}
        for rail, by_station in values.items()
    }


def _build_output_row(
    preparation: FullDefaultCasePreparation,
    record: FullCaseIterationRecord,
    *,
    iterations: int,
) -> FullCaseOutputRow:
    patch_force_y, patch_force_z, wheelset_lateral_force, wheelset_vertical_force = _wheelset_force_summary(
        preparation.operating_case,
        record.pjch,
        record.pjcc,
        record.prhxf,
    )
    return FullCaseOutputRow(
        stage=record.stage,
        step_index=record.step_index,
        iterations=iterations,
        time=record.time,
        dt=record.dt,
        front_mileage=record.front_mileage,
        displacement=record.displacement.copy(),
        velocity=record.velocity.copy(),
        acceleration=record.acceleration.copy(),
        vehicle_displacement=record.vehicle_displacement.copy(),
        vehicle_velocity=record.vehicle_velocity.copy(),
        vehicle_acceleration=record.vehicle_acceleration.copy(),
        contact_force=record.force_guess.copy(),
        total_force=preparation.gravity_preload.pxt_gravity + record.force_guess,
        damper_equivalent_force=record.damper_equivalent_force.copy(),
        contact_state=record.contact_state,
        rail_response=record.rail_response,
        rail_profile_numbers=_copy_rail_profile_numbers(record.rail_profile_numbers),
        pjc=record.pjc.copy(),
        pjch=record.pjch.copy(),
        pjcc=record.pjcc.copy(),
        prhx=record.prhx.copy(),
        prhxf=record.prhxf.copy(),
        normal_error=record.normal_error,
        normal_tangential_error=record.normal_tangential_error,
        patch_force_y=patch_force_y,
        patch_force_z=patch_force_z,
        wheelset_lateral_force=wheelset_lateral_force,
        wheelset_vertical_force=wheelset_vertical_force,
    )


def _wheelset_force_summary(
    operating_case: DefaultOperatingCase,
    pjch: np.ndarray,
    pjcc: np.ndarray,
    prhxf: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    patch_force_y = -np.asarray(prhxf[:, 1], dtype=float) - np.asarray(pjch[:, 0], dtype=float)
    patch_force_z = -np.asarray(prhxf[:, 2], dtype=float) - np.asarray(pjcc[:, 0], dtype=float)
    wheelset_lateral_force = np.zeros((operating_case.n_wheels, 2), dtype=float)
    wheelset_vertical_force = np.zeros((operating_case.n_wheels, 2), dtype=float)
    for wheel_index in range(operating_case.n_wheels):
        start = wheel_index * operating_case.n_contact_patch
        stop = start + operating_case.n_contact_patch
        patch_y = patch_force_y[start:stop]
        patch_z = patch_force_z[start:stop]
        if patch_y.size == 0:
            continue
        wheelset_lateral_force[wheel_index, 0] = patch_y[0]
        wheelset_vertical_force[wheel_index, 0] = patch_z[0]
        wheelset_lateral_force[wheel_index, 1] = np.sum(patch_y[1:])
        wheelset_vertical_force[wheel_index, 1] = np.sum(patch_z[1:])
    return patch_force_y, patch_force_z, wheelset_lateral_force, wheelset_vertical_force
