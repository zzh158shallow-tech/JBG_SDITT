from __future__ import annotations

import numpy as np

from sditt.integrators import (
    LinearSecondOrderSystem,
    NewmarkCoefficients,
    PreparedLinearStepper,
    initial_acceleration,
    integrate_park_newmark,
    newmark_step,
    park_step,
)
from sditt.simulation import (
    HarmonicForce,
    assemble_system_matrices,
    run_default_no_contact_smoke,
    run_no_contact_forced_response,
)


def test_newmark_coefficients_match_matlab_setup() -> None:
    coeffs = NewmarkCoefficients(dt=0.01)

    assert np.isclose(coeffs.a1, 40000.0)
    assert np.isclose(coeffs.a2, 200.0)
    assert np.isclose(coeffs.a3, 400.0)
    assert np.isclose(coeffs.a4, 1.0)
    assert np.isclose(coeffs.a5, 0.0)
    assert np.isclose(coeffs.a6, 1.0)


def test_newmark_step_satisfies_end_step_equilibrium() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[2.0]]),
        damping=np.array([[0.4]]),
        stiffness=np.array([[50.0]]),
    )
    q0 = np.array([0.1])
    v0 = np.array([0.0])
    p1 = np.array([1.5])
    a0 = initial_acceleration(system, q0, v0, np.array([1.0]))

    q1, v1, a1 = newmark_step(system, q0, v0, a0, p1, dt=0.005)

    residual = system.mass @ a1 + system.damping @ v1 + system.stiffness @ q1 - p1
    assert np.allclose(residual, 0.0, atol=1e-10)


def test_park_step_matches_direct_matlab_formula() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[2.0]]),
        damping=np.array([[0.4]]),
        stiffness=np.array([[50.0]]),
    )
    qh = np.array([[0.1000], [0.0998], [0.0992]])
    vh = np.array([[0.0000], [-0.0400], [-0.0795]])
    p_next = np.array([0.0])
    dt = 0.01

    q4, v4, a4 = park_step(system, qh, vh, p_next, dt)

    r = 10.0 / (6.0 * dt)
    bw = (
        (-15.0 / (6.0 * dt)) * qh[2]
        + (1.0 / dt) * qh[1]
        - (1.0 / (6.0 * dt)) * qh[0]
    )
    bs = (
        (-15.0 / (6.0 * dt)) * vh[2]
        + (1.0 / dt) * vh[1]
        - (1.0 / (6.0 * dt)) * vh[0]
    )
    effective = r * r * system.mass + r * system.damping + system.stiffness
    rhs = p_next - r * (system.mass @ bw) - system.mass @ bs - system.damping @ bw
    expected_q4 = np.linalg.solve(effective, rhs)
    expected_v4 = r * expected_q4 + bw
    expected_a4 = r * expected_v4 + bs

    assert np.allclose(q4, expected_q4)
    assert np.allclose(v4, expected_v4)
    assert np.allclose(a4, expected_a4)


def test_park_newmark_sdof_free_vibration_tracks_analytic_solution() -> None:
    mass = 1.0
    stiffness = 100.0
    damping_ratio = 0.05
    damping = 2.0 * damping_ratio * np.sqrt(stiffness * mass)
    system = LinearSecondOrderSystem(
        mass=np.array([[mass]]),
        damping=np.array([[damping]]),
        stiffness=np.array([[stiffness]]),
    )

    history = integrate_park_newmark(
        system,
        force=np.array([0.0]),
        displacement0=np.array([0.1]),
        velocity0=np.array([0.0]),
        dt=0.001,
        n_steps=2000,
    )

    omega_n = np.sqrt(stiffness / mass)
    omega_d = omega_n * np.sqrt(1.0 - damping_ratio**2)
    expected = 0.1 * np.exp(-damping_ratio * omega_n * history.time) * (
        np.cos(omega_d * history.time)
        + damping_ratio / np.sqrt(1.0 - damping_ratio**2) * np.sin(omega_d * history.time)
    )

    assert history.startup_steps == 2
    assert np.all(np.isfinite(history.displacement))
    assert np.max(np.abs(history.displacement[:, 0] - expected)) < 2e-3


def test_integrator_accepts_assembled_vehicle_track_style_matrices() -> None:
    track_mass = np.eye(2)
    track_stiffness = np.diag([100.0, 400.0])
    track_damping = np.diag([0.8, 1.2])
    vehicle_mass = np.diag([3.0, 4.0])
    vehicle_stiffness = np.diag([60.0, 80.0])
    vehicle_damping = np.diag([0.5, 0.6])
    matrices = assemble_system_matrices(
        track_mass,
        track_stiffness,
        track_damping,
        vehicle_mass,
        vehicle_stiffness,
        vehicle_damping,
        nm_fw=0,
        n_wheels=4,
        n_rv=2,
    )
    system = LinearSecondOrderSystem(matrices.Mxt, matrices.Cxt, matrices.Kxt)
    q0 = np.zeros(system.ndof)
    q0[0] = 1e-3

    history = integrate_park_newmark(
        system,
        force=lambda _time: np.zeros(system.ndof),
        displacement0=q0,
        velocity0=np.zeros(system.ndof),
        dt=1e-4,
        n_steps=20,
    )

    assert history.displacement.shape == (21, 4)
    assert history.velocity.shape == (21, 4)
    assert history.acceleration.shape == (21, 4)
    assert np.all(np.isfinite(history.displacement))


def test_prepared_stepper_matches_standalone_steps() -> None:
    system = LinearSecondOrderSystem(
        mass=np.array([[2.0, 0.0], [0.0, 3.0]]),
        damping=np.array([[0.4, 0.0], [0.0, 0.6]]),
        stiffness=np.array([[50.0, -5.0], [-5.0, 80.0]]),
    )
    stepper = PreparedLinearStepper(system, dt=0.005)
    q0 = np.array([0.1, -0.02])
    v0 = np.array([0.0, 0.01])
    a0 = initial_acceleration(system, q0, v0, np.array([1.0, 0.5]))
    p1 = np.array([1.5, -0.2])

    assert np.allclose(stepper.newmark_step(q0, v0, a0, p1), newmark_step(system, q0, v0, a0, p1, 0.005))


def test_sparse_linear_system_integrates_like_dense() -> None:
    dense_system = LinearSecondOrderSystem(
        mass=np.diag([2.0, 3.0]),
        damping=np.diag([0.4, 0.5]),
        stiffness=np.diag([50.0, 70.0]),
    )
    sparse_system = LinearSecondOrderSystem(
        mass=dense_system.mass,
        damping=dense_system.damping,
        stiffness=dense_system.stiffness,
        use_sparse=True,
    )
    force = np.array([1.0, -0.5])
    kwargs = dict(force=force, displacement0=np.array([0.1, 0.0]), velocity0=np.zeros(2), dt=0.001, n_steps=20)

    dense_history = integrate_park_newmark(dense_system, **kwargs)
    sparse_history = integrate_park_newmark(sparse_system, **kwargs)

    assert np.allclose(sparse_history.displacement, dense_history.displacement)
    assert np.allclose(sparse_history.velocity, dense_history.velocity)
    assert np.allclose(sparse_history.acceleration, dense_history.acceleration)


def test_no_contact_forced_response_advances_qva_histories() -> None:
    matrices = assemble_system_matrices(
        np.eye(1),
        np.array([[100.0]]),
        np.array([[1.0]]),
        np.diag([2.0, 3.0]),
        np.diag([40.0, 60.0]),
        np.diag([0.4, 0.5]),
        nm_fw=0,
        n_wheels=4,
        n_rv=2,
    )
    force = HarmonicForce(
        ndof=matrices.layout.total_dof,
        dof=0,
        amplitude=500.0,
        frequency_hz=5.0,
    )

    result = run_no_contact_forced_response(matrices, force, dt=1e-3, n_steps=100)

    assert result.history.displacement.shape == (101, 3)
    assert result.history.velocity.shape == (101, 3)
    assert result.history.acceleration.shape == (101, 3)
    assert result.force.shape == (101, 3)
    assert np.all(np.isfinite(result.history.displacement))
    assert np.max(np.abs(result.force[:, 0])) > 0.0
    assert np.max(np.abs(result.history.displacement[:, 0])) > 0.0
    assert np.max(np.abs(result.history.velocity[:, 0])) > 0.0
    assert np.max(np.abs(result.history.acceleration[:, 0])) > 0.0
    assert np.allclose(result.history.displacement[:, 1:], 0.0)


def test_default_no_contact_smoke_uses_vehicle_track_matrices() -> None:
    result = run_default_no_contact_smoke(cut_freq=50.0, dt=1e-4, n_steps=20)

    assert result.history.displacement.shape == (21, 56)
    assert result.force.shape == (21, 56)
    assert np.all(np.isfinite(result.history.displacement))
    assert np.max(np.abs(result.history.acceleration[:, 0])) > 0.0
