from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from sditt.integrators import LinearSecondOrderSystem
from sditt.simulation import (
    CoupledIterationSettings,
    CoupledStepCallbacks,
    CoupledStepState,
    run_coupled_time_iteration,
)


def test_coupled_time_iteration_repeats_step_until_contact_force_converges() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[1.0]]),
        damping=np.array([[0.0]]),
        stiffness=np.array([[10.0]]),
    )
    events: list[tuple[str, int, float]] = []

    def recover(state: CoupledStepState) -> dict[str, float]:
        events.append(("recover", state.iteration, state.force_guess[0]))
        return {"rail_displacement": float(state.displacement[0])}

    def geometry(state: CoupledStepState, rail: dict[str, float]) -> dict[str, float]:
        events.append(("geometry", state.iteration, rail["rail_displacement"]))
        return {"gap": -rail["rail_displacement"]}

    def force(state: CoupledStepState, rail: dict[str, float], geom: dict[str, float]) -> np.ndarray:
        events.append(("force", state.iteration, geom["gap"]))
        return np.array([5.0])

    result = run_coupled_time_iteration(
        system,
        CoupledStepCallbacks(recover, geometry, force),
        dt=0.01,
        n_steps=1,
        settings=CoupledIterationSettings(max_iterations=3, force_tolerance=0.0, absolute_force_tolerance=0.0),
    )

    assert result.iterations.tolist() == [0, 2]
    assert np.allclose(result.contact_force[-1], [5.0])
    assert np.allclose(result.total_force[-1], [5.0])
    assert [event[0] for event in events] == ["recover", "geometry", "force", "recover", "geometry", "force"]
    assert events[0][2] == 0.0
    assert events[3][2] == 5.0
    assert result.displacement.shape == (2, 1)
    assert result.rail_response[-1]["rail_displacement"] == result.displacement[-1, 0]
    assert result.contact_geometry[-1]["gap"] == -result.displacement[-1, 0]


def test_coupled_time_iteration_shrinks_dt_and_retries_unconverged_step() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[1.0]]),
        damping=np.array([[0.0]]),
        stiffness=np.array([[1.0]]),
    )
    attempted_dt: list[float] = []

    def recover(state: CoupledStepState) -> dict[str, float]:
        return {"dt": state.dt}

    def geometry(state: CoupledStepState, rail: dict[str, float]) -> dict[str, float]:
        return rail

    def force(state: CoupledStepState, rail: dict[str, float], geom: dict[str, float]) -> np.ndarray:
        attempted_dt.append(state.dt)
        if state.dt > 0.005:
            return np.array([1.0])
        return state.force_guess.copy()

    result = run_coupled_time_iteration(
        system,
        CoupledStepCallbacks(recover, geometry, force),
        dt=0.01,
        n_steps=1,
        settings=CoupledIterationSettings(
            max_iterations=1,
            force_tolerance=0.0,
            absolute_force_tolerance=0.0,
            min_dt=0.0025,
            shrink_factor=0.5,
        ),
    )

    assert attempted_dt == [0.01, 0.005]
    assert np.isclose(result.dt[-1], 0.005)
    assert np.isclose(result.time[-1], 0.005)
    assert result.iterations[-1] == 1
    assert result.retry_count.tolist() == [0, 1]


def test_coupled_time_iteration_extends_after_retry_until_time_end() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[1.0]]),
        damping=np.array([[0.0]]),
        stiffness=np.array([[1.0]]),
    )
    fail_nominal_once = True

    def force(state: CoupledStepState, _rail: object, _geometry: object) -> np.ndarray:
        nonlocal fail_nominal_once
        if fail_nominal_once and state.dt > 0.005:
            fail_nominal_once = False
            return np.array([1.0])
        return state.force_guess.copy()

    result = run_coupled_time_iteration(
        system,
        CoupledStepCallbacks(
            lambda state: {"dt": state.dt},
            lambda _state, rail: rail,
            force,
        ),
        dt=0.01,
        n_steps=2,
        settings=CoupledIterationSettings(
            max_iterations=1,
            force_tolerance=0.0,
            absolute_force_tolerance=0.0,
            min_dt=0.0025,
            shrink_factor=0.5,
        ),
        step_dt_callback=lambda _step, time, nominal: min(nominal, 0.02 - time),
        time_end=0.02,
    )

    assert result.step_index.tolist() == [0, 1, 2, 3]
    assert np.allclose(result.dt, [0.0, 0.005, 0.01, 0.005])
    assert result.time[-1] == 0.02


def test_coupled_time_iteration_uses_scheduled_dt_for_each_new_step() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[1.0]]),
        damping=np.array([[0.0]]),
        stiffness=np.array([[0.0]]),
    )
    attempted: list[tuple[int, float, float]] = []

    def recover(state: CoupledStepState) -> dict[str, float]:
        attempted.append((state.step_index, state.time, state.dt))
        return {"dt": state.dt}

    def geometry(state: CoupledStepState, rail: dict[str, float]) -> dict[str, float]:
        return rail

    def force(state: CoupledStepState, rail: dict[str, float], geom: dict[str, float]) -> np.ndarray:
        return state.force_guess.copy()

    result = run_coupled_time_iteration(
        system,
        CoupledStepCallbacks(recover, geometry, force),
        dt=0.01,
        n_steps=3,
        step_dt_callback=lambda step_index, _time_current, nominal_dt: nominal_dt / step_index,
    )

    assert [item[2] for item in attempted] == [0.01, 0.005, 0.01 / 3.0]
    assert np.allclose(result.dt, [0.0, 0.01, 0.005, 0.01 / 3.0])
    assert np.allclose(result.time, [0.0, 0.01, 0.015, 0.015 + 0.01 / 3.0])


def test_coupled_time_iteration_uses_external_force_with_converged_contact() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[2.0]]),
        damping=np.array([[0.0]]),
        stiffness=np.array([[0.0]]),
    )
    callbacks = CoupledStepCallbacks(
        recover_track_response=lambda state: {"q": state.displacement.copy()},
        contact_geometry=lambda state, rail: {"rail": rail},
        contact_force=lambda state, rail, geom: np.zeros(1),
        external_force=lambda time: np.array([4.0 * time]),
    )

    result = run_coupled_time_iteration(
        system,
        callbacks,
        dt=0.1,
        n_steps=2,
        settings=CoupledIterationSettings(max_iterations=2, force_tolerance=0.0, absolute_force_tolerance=0.0),
    )

    assert np.allclose(result.contact_force, 0.0)
    assert np.allclose(result.total_force[:, 0], [0.0, 0.4, 0.8])
    assert np.all(result.iterations[1:] == 1)


def test_coupled_time_iteration_accumulates_nested_contact_timing() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[1.0]]),
        damping=np.array([[0.0]]),
        stiffness=np.array([[0.0]]),
    )
    callbacks = CoupledStepCallbacks(
        recover_track_response=lambda state: state.displacement.copy(),
        contact_geometry=lambda state, rail: SimpleNamespace(
            timing={"network_a1.predict_total": 0.125}
        ),
        contact_force=lambda state, rail, geometry: np.zeros((1,)),
    )

    result = run_coupled_time_iteration(
        system,
        callbacks,
        dt=0.01,
        n_steps=2,
    )

    assert result.timing["contact_geometry.network_a1.predict_total"] == 0.25
