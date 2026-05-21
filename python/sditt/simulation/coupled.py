from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from sditt.integrators import (
    LinearSecondOrderSystem,
    initial_acceleration,
    newmark_step,
    park_step,
)


StateCallback = Callable[["CoupledStepState"], Any]
ContactForceCallback = Callable[["CoupledStepState", Any, Any], np.ndarray]
ExternalForceCallback = Callable[[float], np.ndarray]


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

    time: np.ndarray
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    contact_force: np.ndarray
    total_force: np.ndarray
    iterations: np.ndarray
    dt: np.ndarray
    rail_response: tuple[Any, ...]
    contact_geometry: tuple[Any, ...]


def run_coupled_time_iteration(
    system: LinearSecondOrderSystem,
    callbacks: CoupledStepCallbacks,
    *,
    dt: float,
    n_steps: int,
    displacement0: np.ndarray | None = None,
    velocity0: np.ndarray | None = None,
    acceleration0: np.ndarray | None = None,
    contact_force0: np.ndarray | None = None,
    settings: CoupledIterationSettings | None = None,
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
    settings = settings or CoupledIterationSettings()
    if settings.min_dt > dt:
        raise ValueError("min_dt cannot exceed dt")

    q0 = np.zeros(system.ndof, dtype=float) if displacement0 is None else _as_vector(displacement0, system.ndof)
    v0 = np.zeros(system.ndof, dtype=float) if velocity0 is None else _as_vector(velocity0, system.ndof)
    f_contact = (
        np.zeros(system.ndof, dtype=float) if contact_force0 is None else _as_vector(contact_force0, system.ndof)
    )
    f_ext0 = _external_force(callbacks.external_force, 0.0, system.ndof)
    a0 = (
        _as_vector(acceleration0, system.ndof)
        if acceleration0 is not None
        else initial_acceleration(system, q0, v0, f_ext0 + f_contact)
    )

    time_history = [0.0]
    q_history = [q0]
    v_history = [v0]
    a_history = [a0]
    contact_force_history = [f_contact]
    total_force_history = [f_ext0 + f_contact]
    iteration_history = [0]
    dt_history = [0.0]
    rail_history: list[Any] = [None]
    geometry_history: list[Any] = [None]

    current_dt = float(dt)
    accepted_steps = 0
    while accepted_steps < n_steps:
        try:
            accepted = _attempt_coupled_step(
                system,
                callbacks,
                step_index=accepted_steps + 1,
                time_next=time_history[-1] + current_dt,
                dt=current_dt,
                q_history=q_history,
                v_history=v_history,
                a_history=a_history,
                previous_contact_force=contact_force_history[-1],
                settings=settings,
            )
        except _StepDidNotConverge:
            current_dt *= settings.shrink_factor
            if current_dt < settings.min_dt:
                raise RuntimeError(
                    f"coupled force iteration did not converge before dt fell below {settings.min_dt}"
                ) from None
            continue

        time_history.append(accepted.time)
        q_history.append(accepted.displacement)
        v_history.append(accepted.velocity)
        a_history.append(accepted.acceleration)
        contact_force_history.append(accepted.contact_force)
        total_force_history.append(accepted.total_force)
        iteration_history.append(accepted.iterations)
        dt_history.append(current_dt)
        rail_history.append(accepted.rail_response)
        geometry_history.append(accepted.contact_geometry)
        accepted_steps += 1
        if settings.reset_dt_after_success:
            current_dt = float(dt)

    return CoupledTimeIterationResult(
        time=np.asarray(time_history, dtype=float),
        displacement=np.vstack(q_history),
        velocity=np.vstack(v_history),
        acceleration=np.vstack(a_history),
        contact_force=np.vstack(contact_force_history),
        total_force=np.vstack(total_force_history),
        iterations=np.asarray(iteration_history, dtype=int),
        dt=np.asarray(dt_history, dtype=float),
        rail_response=tuple(rail_history),
        contact_geometry=tuple(geometry_history),
    )


@dataclass(frozen=True)
class _AcceptedStep:
    time: float
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    contact_force: np.ndarray
    total_force: np.ndarray
    iterations: int
    rail_response: Any
    contact_geometry: Any


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
) -> _AcceptedStep:
    force_guess = previous_contact_force.copy()
    external = _external_force(callbacks.external_force, time_next, system.ndof)
    last_q = last_v = last_a = None
    last_rail = last_geometry = None
    last_contact = None

    for iteration in range(1, settings.max_iterations + 1):
        total_force = external + force_guess
        q_next, v_next, a_next = _integrate_candidate(
            system,
            q_history,
            v_history,
            a_history,
            total_force,
            dt,
        )
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
        rail_response = callbacks.recover_track_response(state)
        contact_geometry = callbacks.contact_geometry(state, rail_response)
        contact_force = _as_vector(callbacks.contact_force(state, rail_response, contact_geometry), system.ndof)

        last_q, last_v, last_a = q_next, v_next, a_next
        last_rail, last_geometry = rail_response, contact_geometry
        last_contact = contact_force

        if _force_converged(contact_force, force_guess, settings):
            return _AcceptedStep(
                time=time_next,
                displacement=q_next,
                velocity=v_next,
                acceleration=a_next,
                contact_force=contact_force,
                total_force=external + contact_force,
                iterations=iteration,
                rail_response=rail_response,
                contact_geometry=contact_geometry,
            )
        force_guess = (1.0 - settings.relaxation) * force_guess + settings.relaxation * contact_force

    raise _StepDidNotConverge()


def _integrate_candidate(
    system: LinearSecondOrderSystem,
    q_history: list[np.ndarray],
    v_history: list[np.ndarray],
    a_history: list[np.ndarray],
    total_force: np.ndarray,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(q_history) < 3:
        return newmark_step(system, q_history[-1], v_history[-1], a_history[-1], total_force, dt)
    return park_step(system, np.vstack(q_history[-3:]), np.vstack(v_history[-3:]), total_force, dt)


def _force_converged(contact_force: np.ndarray, force_guess: np.ndarray, settings: CoupledIterationSettings) -> bool:
    delta = np.linalg.norm(contact_force - force_guess, ord=np.inf)
    scale = max(np.linalg.norm(contact_force, ord=np.inf), np.linalg.norm(force_guess, ord=np.inf), 1.0)
    return delta <= settings.absolute_force_tolerance or delta / scale <= settings.force_tolerance


def _external_force(callback: ExternalForceCallback | None, time: float, ndof: int) -> np.ndarray:
    if callback is None:
        return np.zeros(ndof, dtype=float)
    return _as_vector(callback(time), ndof)


def _as_vector(value: np.ndarray, ndof: int) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (ndof,):
        raise ValueError(f"expected vector with shape ({ndof},), got {array.shape}")
    return array
