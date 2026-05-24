from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Callable, Mapping

import numpy as np

from sditt.contact import (
    DefaultTrackContactParameters,
    FullCaseWheelRailContactResult,
    solve_default_wheel_rail_contact,
)
from sditt.config import MATLAB_FULL_DEFAULT_CASE, DefaultOperatingCase, ProjectPaths, SimulationStage
from sditt.integrators import LinearSecondOrderSystem
from sditt.profiles import (
    DefaultRailProfileSelector,
    WheelProfileSet,
    build_default_07009_face_profile_selector,
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

from .coupled import CoupledIterationSettings, CoupledStepCallbacks, CoupledStepState, CoupledTimeIterationResult
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
    iteration_settings: CoupledIterationSettings = CoupledIterationSettings(
        max_iterations=30,
        force_tolerance=1.0e-3,
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
    profile_selector: DefaultRailProfileSelector
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


def prepare_default_full_case(
    *,
    repo_root: str | Path | None = None,
    settings: FullDefaultCaseSettings | None = None,
    operating_case: DefaultOperatingCase = MATLAB_FULL_DEFAULT_CASE,
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
    profile_selector = build_default_07009_face_profile_selector(
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
    operating_case: DefaultOperatingCase = MATLAB_FULL_DEFAULT_CASE,
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

    for stage in preparation.operating_case.simulation_stages:
        callbacks, build_stage_storage, extract_contact_state = _diagnostic_callbacks(
            preparation,
            stage,
            contact_state=contact_state,
            stage_start_front_mileage=stage_start_front_mileage,
        )
        contact_force = _contact_force_from_contact_state(
            preparation,
            stage,
            displacement,
            velocity,
            stage_start_front_mileage + preparation.operating_case.vlc * preparation.settings.dt,
            contact_state,
        )
        stage_steps = _stage_step_count(preparation, stage, stage_start_front_mileage)
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
            accepted_step_callback=_progress_callback(preparation, stage, stage_steps, build_stage_storage),
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

    return FullDefaultCaseRunResult(
        preparation=preparation,
        stages=tuple(stages),
    )


def _linear_system(system: SystemMatrices | SparseSystemMatrices) -> LinearSecondOrderSystem:
    use_sparse = isinstance(system, SparseSystemMatrices)
    return LinearSecondOrderSystem(system.Mxt, system.Cxt, system.Kxt, use_sparse=use_sparse)


def _stage_step_count(preparation: FullDefaultCasePreparation, stage: SimulationStage, start_mileage: float) -> int:
    end_mileage = _stage_end_mileage(preparation, stage)
    if end_mileage is None:
        return preparation.settings.n_steps_per_stage
    step_distance = preparation.operating_case.vlc * preparation.settings.dt
    if step_distance == 0.0:
        raise ValueError("vehicle speed and dt must produce nonzero mileage increment")
    steps_float = (float(end_mileage) - float(start_mileage)) / step_distance
    if steps_float < -1.0e-12:
        raise ValueError(
            f"{stage} end mileage {end_mileage} is behind start mileage {start_mileage} for current direction"
        )
    return max(0, int(np.ceil(max(0.0, steps_float) - 1.0e-12)))


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
) -> Callable[[Any], None] | None:
    del build_stage_storage
    callback = preparation.settings.progress_callback
    if callback is None:
        return None

    def emit(accepted: Any) -> None:
        rail = accepted.rail_response
        geometry = accepted.contact_geometry
        contact = geometry.wheel_rail_contact if geometry is not None else None
        patch_force_z = np.zeros((0,), dtype=float) if contact is None else -contact.prhxf[:, 2] - contact.pjcc[:, 0]
        callback(
            FullCaseProgressEvent(
                stage=stage,
                step_index=int(getattr(accepted, "step_index", 0) or 0),
                n_steps=stage_steps,
                time=float(accepted.time),
                dt=float(preparation.settings.dt),
                front_mileage=float(rail.front_mileage),
                iterations=int(accepted.iterations),
                normal_error=float("nan"),
                normal_tangential_error=float("nan"),
                contact_force_norm=float(np.linalg.norm(accepted.contact_force)),
                total_force_norm=float(np.linalg.norm(accepted.total_force)),
                max_patch_force_z=float(np.max(np.abs(patch_force_z), initial=0.0)),
            )
        )

    return emit


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
                fixed_d0_by_wheelset=(
                    step_input_state.d0_by_wheelset or None if reuse_contact_state_within_step else None
                ),
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
                step_index,
                int(history.iterations[step_index]),
                float(history.dt[step_index]),
                float(history.time[step_index]),
            )
            for step_index in range(1, history.time.size)
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
                iterations=int(history.iterations[step_index]),
            )
            for step_index in range(1, history.time.size)
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
