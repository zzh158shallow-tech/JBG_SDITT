from __future__ import annotations

import time
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Mapping

import numpy as np

from sditt.integrators import (
    LinearSecondOrderSystem,
    PreparedLinearStepper,
    initial_acceleration,
    newmark_step,
    park_step,
)


StateCallback = Callable[["CoupledStepState"], Any]
ContactForceCallback = Callable[["CoupledStepState", Any, Any], np.ndarray]
ExternalForceCallback = Callable[[float], np.ndarray]
ConvergenceCallback = Callable[["CoupledStepState", Any, Any, np.ndarray, np.ndarray, "CoupledIterationSettings"], bool]
StepDtCallback = Callable[[int, float, float], float]


@dataclass(frozen=True)
class CoupledIterationSettings:
    """Controls for force-iteration and adaptive retry."""

    max_iterations: int = 20
    force_tolerance: float = 1e-4
    absolute_force_tolerance: float = 1e-8
    relaxation: float = 1.0
    min_dt: float = 1e-7
    shrink_factor: float = 0.5
    reset_dt_after_success: bool = True

    def __post_init__(self) -> None:
        if self.max_iterations <= 0:
            raise ValueError("max_iterations must be positive")
        if self.force_tolerance < 0.0 or self.absolute_force_tolerance < 0.0:
            raise ValueError("force tolerances must be non-negative")
        if not 0.0 < self.relaxation <= 1.0:
            raise ValueError("relaxation must be in (0, 1]")
        if self.min_dt <= 0.0:
            raise ValueError("min_dt must be positive")
        if not 0.0 < self.shrink_factor < 1.0:
            raise ValueError("shrink_factor must be in (0, 1)")


@dataclass(frozen=True)
class CoupledStepCallbacks:
    """Pluggable physics stages for the coupled wheel-rail loop."""

    recover_track_response: StateCallback
    contact_geometry: Callable[["CoupledStepState", Any], Any]
    contact_force: ContactForceCallback
    external_force: ExternalForceCallback | None = None
    converged: ConvergenceCallback | None = None


@dataclass(frozen=True)
class CoupledStepState:
    """Candidate state inside one nonlinear force iteration."""

    step_index: int
    iteration: int
    time: float
    dt: float
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    force_guess: np.ndarray


@dataclass(frozen=True)
class CoupledTimeIterationResult:
    """Accepted coupled response history and iteration metadata."""

    step_index: np.ndarray
    time: np.ndarray
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    contact_force: np.ndarray
    total_force: np.ndarray
    iterations: np.ndarray
    dt: np.ndarray
    retry_count: np.ndarray
    rail_response: tuple[Any, ...]
    contact_geometry: tuple[Any, ...]
    timing: Mapping[str, float] = field(default_factory=dict)


def run_coupled_time_iteration(
    system: LinearSecondOrderSystem,
    callbacks: CoupledStepCallbacks,
    *,
    dt: float,
    n_steps: int,
    displacement0: np.ndarray | None = None,
    velocity0: np.ndarray | None = None,
    acceleration0: np.ndarray | None = None,
    displacement_history0: np.ndarray | None = None,
    velocity_history0: np.ndarray | None = None,
    acceleration_history0: np.ndarray | None = None,
    contact_force0: np.ndarray | None = None,
    settings: CoupledIterationSettings | None = None,
    accepted_step_callback: Callable[[Any], None] | None = None,
    history_retention_steps: int | None = None,
    time0: float = 0.0,
    step_index0: int = 0,
    step_dt_callback: StepDtCallback | None = None,
    time_end: float | None = None,
) -> CoupledTimeIterationResult:
    """Run the coupled main loop with force convergence and step-size retry.

    Per accepted step:
    ``integrate -> recover track -> contact geometry -> contact force ->
    force convergence``. If the contact force does not converge within
    ``max_iterations``, the same step is retried with a smaller ``dt``.
    """

    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if n_steps < 0:
        raise ValueError("n_steps cannot be negative")
    if history_retention_steps is not None and history_retention_steps <= 0:
        raise ValueError("history_retention_steps must be positive")
    if step_index0 < 0:
        raise ValueError("step_index0 cannot be negative")
    if time_end is not None and (not np.isfinite(time_end) or time_end < time0):
        raise ValueError("time_end must be finite and no earlier than time0")
    settings = settings or CoupledIterationSettings()
    if settings.min_dt > dt:
        raise ValueError("min_dt cannot exceed dt")

    q_seed = _as_history_seed(displacement_history0, system.ndof)
    v_seed = _as_history_seed(velocity_history0, system.ndof)
    a_seed = _as_history_seed(acceleration_history0, system.ndof)
    if (q_seed is None) != (v_seed is None) or (q_seed is None) != (a_seed is None):
        raise ValueError("displacement, velocity, and acceleration history seeds must be supplied together")

    q0 = (
        q_seed[-1].copy()
        if q_seed is not None
        else np.zeros(system.ndof, dtype=float)
        if displacement0 is None
        else _as_vector(displacement0, system.ndof)
    )
    v0 = (
        v_seed[-1].copy()
        if v_seed is not None
        else np.zeros(system.ndof, dtype=float)
        if velocity0 is None
        else _as_vector(velocity0, system.ndof)
    )
    f_contact = (
        np.zeros(system.ndof, dtype=float) if contact_force0 is None else _as_vector(contact_force0, system.ndof)
    )
    f_ext0 = _external_force(callbacks.external_force, 0.0, system.ndof)
    a0 = (
        a_seed[-1].copy()
        if a_seed is not None
        else _as_vector(acceleration0, system.ndof)
        if acceleration0 is not None
        else initial_acceleration(system, q0, v0, f_ext0 + f_contact)
    )

    step_index_history = [int(step_index0)]
    time_history = [float(time0)]
    q_history = [q0]
    v_history = [v0]
    a_history = [a0]
    contact_force_history = [f_contact]
    total_force_history = [f_ext0 + f_contact]
    iteration_history = [0]
    dt_history = [0.0]
    retry_count_history = [0]
    rail_history: list[Any] = [None]
    geometry_history: list[Any] = [None]
    integration_q_history = [row.copy() for row in q_seed] if q_seed is not None else [q0]
    integration_v_history = [row.copy() for row in v_seed] if v_seed is not None else [v0]
    integration_a_history = [row.copy() for row in a_seed] if a_seed is not None else [a0]

    current_dt = float(dt)
    stepper_cache: dict[float, PreparedLinearStepper] = {}
    timing: dict[str, float] = {}
    accepted_steps = 0
    retry_count = 0
    accepted_step_wall_start = time.perf_counter()
    maximum_accepted_steps = max(int(n_steps) * 16 + 1024, int(n_steps))
    time_tolerance = 1.0e-12 if time_end is None else max(
        1.0e-12,
        abs(float(time_end)) * np.finfo(float).eps * 8.0,
    )
    while accepted_steps < n_steps or (
        time_end is not None and time_history[-1] < float(time_end) - time_tolerance
    ):
        if accepted_steps >= maximum_accepted_steps:
            raise RuntimeError("coupled iteration did not reach time_end within its safety limit")
        if retry_count == 0:
            current_dt = _scheduled_dt(
                step_dt_callback,
                int(step_index0) + accepted_steps + 1,
                time_history[-1],
                dt,
            )
            accepted_step_wall_start = time.perf_counter()
        stepper = stepper_cache.get(current_dt)
        if stepper is None:
            start = time.perf_counter()
            stepper = PreparedLinearStepper(system, dt=current_dt)
            _add_timing(timing, "linear_stepper_setup", start)
            stepper_cache[current_dt] = stepper
        try:
            accepted = _attempt_coupled_step(
                system,
                callbacks,
                step_index=int(step_index0) + accepted_steps + 1,
                time_next=time_history[-1] + current_dt,
                dt=current_dt,
                q_history=integration_q_history,
                v_history=integration_v_history,
                a_history=integration_a_history,
                previous_contact_force=contact_force_history[-1],
                settings=settings,
                stepper=stepper,
                timing=timing,
            )
        except _StepDidNotConverge:
            retry_count += 1
            current_dt *= settings.shrink_factor
            if current_dt < settings.min_dt:
                raise RuntimeError(
                    f"coupled force iteration did not converge before dt fell below {settings.min_dt}"
                ) from None
            continue

        accepted = replace(
            accepted,
            retry_count=retry_count,
            step_wall_time=time.perf_counter() - accepted_step_wall_start,
        )
        time_history.append(accepted.time)
        q_history.append(accepted.displacement)
        v_history.append(accepted.velocity)
        a_history.append(accepted.acceleration)
        contact_force_history.append(accepted.contact_force)
        total_force_history.append(accepted.total_force)
        iteration_history.append(accepted.iterations)
        dt_history.append(current_dt)
        retry_count_history.append(retry_count)
        rail_history.append(accepted.rail_response)
        geometry_history.append(accepted.contact_geometry)
        integration_q_history.append(accepted.displacement)
        integration_v_history.append(accepted.velocity)
        integration_a_history.append(accepted.acceleration)
        if len(integration_q_history) > 3:
            del integration_q_history[:-3]
            del integration_v_history[:-3]
            del integration_a_history[:-3]
        if accepted_step_callback is not None:
            accepted_step_callback(
                replace(
                    accepted,
                    displacement_history_seed=_park_history_seed(integration_q_history),
                    velocity_history_seed=_park_history_seed(integration_v_history),
                    acceleration_history_seed=_park_history_seed(integration_a_history),
                )
            )
        accepted_steps += 1
        step_index_history.append(accepted.step_index)
        if history_retention_steps is not None:
            max_rows = int(history_retention_steps) + 1
            if len(time_history) > max_rows:
                drop = len(time_history) - max_rows
                del step_index_history[:drop]
                del time_history[:drop]
                del q_history[:drop]
                del v_history[:drop]
                del a_history[:drop]
                del contact_force_history[:drop]
                del total_force_history[:drop]
                del iteration_history[:drop]
                del dt_history[:drop]
                del retry_count_history[:drop]
                del rail_history[:drop]
                del geometry_history[:drop]
        if settings.reset_dt_after_success:
            current_dt = float(dt)
        retry_count = 0

    return CoupledTimeIterationResult(
        step_index=np.asarray(step_index_history, dtype=int),
        time=np.asarray(time_history, dtype=float),
        displacement=np.vstack(q_history),
        velocity=np.vstack(v_history),
        acceleration=np.vstack(a_history),
        contact_force=np.vstack(contact_force_history),
        total_force=np.vstack(total_force_history),
        iterations=np.asarray(iteration_history, dtype=int),
        dt=np.asarray(dt_history, dtype=float),
        retry_count=np.asarray(retry_count_history, dtype=int),
        rail_response=tuple(rail_history),
        contact_geometry=tuple(geometry_history),
        timing=dict(timing),
    )


@dataclass(frozen=True)
class _AcceptedStep:
    step_index: int
    time: float
    dt: float
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    contact_force: np.ndarray
    total_force: np.ndarray
    iterations: int
    rail_response: Any
    contact_geometry: Any
    timing: Mapping[str, float] = field(default_factory=dict)
    step_wall_time: float = 0.0
    retry_count: int = 0
    displacement_history_seed: np.ndarray | None = None
    velocity_history_seed: np.ndarray | None = None
    acceleration_history_seed: np.ndarray | None = None


class _StepDidNotConverge(Exception):
    pass


def _attempt_coupled_step(
    system: LinearSecondOrderSystem,
    callbacks: CoupledStepCallbacks,
    *,
    step_index: int,
    time_next: float,
    dt: float,
    q_history: list[np.ndarray],
    v_history: list[np.ndarray],
    a_history: list[np.ndarray],
    previous_contact_force: np.ndarray,
    settings: CoupledIterationSettings,
    stepper: PreparedLinearStepper | None = None,
    timing: dict[str, float] | None = None,
) -> _AcceptedStep:
    step_wall_start = time.perf_counter()
    force_guess = previous_contact_force.copy()
    external = _external_force(callbacks.external_force, time_next, system.ndof)
    last_q = last_v = last_a = None
    last_rail = last_geometry = None
    last_contact = None
    stepper = stepper or PreparedLinearStepper(system, dt=dt)

    for iteration in range(1, settings.max_iterations + 1):
        total_force = external + force_guess
        start = time.perf_counter()
        q_next, v_next, a_next = _integrate_candidate(
            system,
            q_history,
            v_history,
            a_history,
            total_force,
            dt,
            stepper,
        )
        _add_timing(timing, "integrate_candidate", start)
        state = CoupledStepState(
            step_index=step_index,
            iteration=iteration,
            time=time_next,
            dt=dt,
            displacement=q_next,
            velocity=v_next,
            acceleration=a_next,
            force_guess=force_guess,
        )
        start = time.perf_counter()
        rail_response = callbacks.recover_track_response(state)
        _add_timing(timing, "recover_track_response", start)
        start = time.perf_counter()
        contact_geometry = callbacks.contact_geometry(state, rail_response)
        _add_timing(timing, "contact_geometry", start)
        _add_nested_timing(
            timing,
            getattr(contact_geometry, "timing", None),
            prefix="contact_geometry.",
        )
        start = time.perf_counter()
        contact_force = _as_vector(callbacks.contact_force(state, rail_response, contact_geometry), system.ndof)
        _add_timing(timing, "contact_force", start)

        last_q, last_v, last_a = q_next, v_next, a_next
        last_rail, last_geometry = rail_response, contact_geometry
        last_contact = contact_force

        start = time.perf_counter()
        converged = _step_converged(callbacks, state, rail_response, contact_geometry, contact_force, force_guess, settings)
        _add_timing(timing, "convergence_check", start)
        if converged:
            return _AcceptedStep(
                step_index=step_index,
                time=time_next,
                dt=dt,
                displacement=q_next,
                velocity=v_next,
                acceleration=a_next,
                contact_force=contact_force,
                total_force=external + contact_force,
                iterations=iteration,
                rail_response=rail_response,
                contact_geometry=contact_geometry,
                timing=dict(timing or {}),
                step_wall_time=time.perf_counter() - step_wall_start,
            )
        force_guess = (1.0 - settings.relaxation) * force_guess + settings.relaxation * contact_force

    raise _StepDidNotConverge()


def _add_timing(timing: dict[str, float] | None, name: str, start: float) -> None:
    if timing is None:
        return
    timing[name] = timing.get(name, 0.0) + (time.perf_counter() - start)


def _add_nested_timing(
    timing: dict[str, float] | None,
    nested: Mapping[str, float] | None,
    *,
    prefix: str,
) -> None:
    if timing is None or not nested:
        return
    for name, seconds in nested.items():
        key = f"{prefix}{name}"
        timing[key] = timing.get(key, 0.0) + float(seconds)


def _scheduled_dt(callback: StepDtCallback | None, step_index: int, time_current: float, nominal_dt: float) -> float:
    if callback is None:
        return float(nominal_dt)
    scheduled = float(callback(step_index, float(time_current), float(nominal_dt)))
    if scheduled <= 0.0:
        raise ValueError("step_dt_callback must return a positive dt")
    return scheduled


def _integrate_candidate(
    system: LinearSecondOrderSystem,
    q_history: list[np.ndarray],
    v_history: list[np.ndarray],
    a_history: list[np.ndarray],
    total_force: np.ndarray,
    dt: float,
    stepper: PreparedLinearStepper | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if stepper is not None:
        if len(q_history) < 3:
            return stepper.newmark_step(q_history[-1], v_history[-1], a_history[-1], total_force)
        return stepper.park_step(np.vstack(q_history[-3:]), np.vstack(v_history[-3:]), total_force)
    if len(q_history) < 3:
        return newmark_step(system, q_history[-1], v_history[-1], a_history[-1], total_force, dt)
    return park_step(system, np.vstack(q_history[-3:]), np.vstack(v_history[-3:]), total_force, dt)


def _force_converged(contact_force: np.ndarray, force_guess: np.ndarray, settings: CoupledIterationSettings) -> bool:
    delta = np.linalg.norm(contact_force - force_guess, ord=np.inf)
    scale = max(np.linalg.norm(contact_force, ord=np.inf), np.linalg.norm(force_guess, ord=np.inf), 1.0)
    return delta <= settings.absolute_force_tolerance or delta / scale <= settings.force_tolerance


def _step_converged(
    callbacks: CoupledStepCallbacks,
    state: CoupledStepState,
    rail_response: Any,
    contact_geometry: Any,
    contact_force: np.ndarray,
    force_guess: np.ndarray,
    settings: CoupledIterationSettings,
) -> bool:
    if callbacks.converged is not None:
        return bool(callbacks.converged(state, rail_response, contact_geometry, contact_force, force_guess, settings))
    return _force_converged(contact_force, force_guess, settings)


def _external_force(callback: ExternalForceCallback | None, time: float, ndof: int) -> np.ndarray:
    if callback is None:
        return np.zeros(ndof, dtype=float)
    return _as_vector(callback(time), ndof)


def _as_vector(value: np.ndarray, ndof: int) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (ndof,):
        raise ValueError(f"expected vector with shape ({ndof},), got {array.shape}")
    return array


def _as_history_seed(value: np.ndarray | None, ndof: int) -> np.ndarray | None:
    if value is None:
        return None
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[1] != ndof or array.shape[0] < 1:
        raise ValueError(f"expected history seed with shape (n, {ndof}), got {array.shape}")
    return array.copy()


def _park_history_seed(history: list[np.ndarray]) -> np.ndarray:
    values = np.vstack(history)
    if values.shape[0] >= 3:
        return values[-3:, :].copy()
    return np.repeat(values[-1:, :], 3, axis=0)
