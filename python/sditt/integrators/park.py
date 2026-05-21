from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np


ArrayLikeForce = np.ndarray | Callable[[float], np.ndarray]


@dataclass(frozen=True)
class NewmarkCoefficients:
    """Average-acceleration Newmark constants used by ``Integration_Park.m``."""

    dt: float
    alpha: float = 0.5
    beta: float = 0.25

    @property
    def a1(self) -> float:
        return 1.0 / (self.beta * self.dt * self.dt)

    @property
    def a2(self) -> float:
        return self.alpha / (self.beta * self.dt)

    @property
    def a3(self) -> float:
        return 1.0 / (self.beta * self.dt)

    @property
    def a4(self) -> float:
        return 1.0 / (2.0 * self.beta) - 1.0

    @property
    def a5(self) -> float:
        return (self.dt / 2.0) * (self.alpha / self.beta - 2.0)

    @property
    def a6(self) -> float:
        return self.alpha / self.beta - 1.0


@dataclass(frozen=True)
class LinearSecondOrderSystem:
    """Linear system ``M qdd + C qd + K q = p(t)``."""

    mass: np.ndarray
    damping: np.ndarray
    stiffness: np.ndarray

    def __post_init__(self) -> None:
        mass = _as_square("mass", self.mass)
        damping = _as_square("damping", self.damping)
        stiffness = _as_square("stiffness", self.stiffness)
        _require_shape("damping", damping, mass.shape)
        _require_shape("stiffness", stiffness, mass.shape)
        object.__setattr__(self, "mass", mass)
        object.__setattr__(self, "damping", damping)
        object.__setattr__(self, "stiffness", stiffness)

    @property
    def ndof(self) -> int:
        return self.mass.shape[0]


@dataclass(frozen=True)
class TimeHistory:
    """Integrated response history with rows ordered by time."""

    time: np.ndarray
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    startup_steps: int


def initial_acceleration(
    system: LinearSecondOrderSystem,
    displacement: np.ndarray,
    velocity: np.ndarray,
    force: np.ndarray,
) -> np.ndarray:
    """Compute initial acceleration from dynamic equilibrium."""

    q = _as_vector("displacement", displacement, system.ndof)
    v = _as_vector("velocity", velocity, system.ndof)
    p = _as_vector("force", force, system.ndof)
    return np.linalg.solve(system.mass, p - system.damping @ v - system.stiffness @ q)


def newmark_step(
    system: LinearSecondOrderSystem,
    displacement: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    force_next: np.ndarray,
    dt: float,
    *,
    alpha: float = 0.5,
    beta: float = 0.25,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Advance one step with the MATLAB Newmark startup formula."""

    if dt <= 0.0:
        raise ValueError("dt must be positive")
    coeffs = NewmarkCoefficients(dt=dt, alpha=alpha, beta=beta)
    q = _as_vector("displacement", displacement, system.ndof)
    v = _as_vector("velocity", velocity, system.ndof)
    a = _as_vector("acceleration", acceleration, system.ndof)
    p = _as_vector("force_next", force_next, system.ndof)

    rhs = (
        p
        + system.mass @ (coeffs.a1 * q + coeffs.a3 * v + coeffs.a4 * a)
        + system.damping @ (coeffs.a2 * q + coeffs.a6 * v + coeffs.a5 * a)
    )
    effective = system.stiffness + coeffs.a1 * system.mass + coeffs.a2 * system.damping
    q_next = np.linalg.solve(effective, rhs)
    a_next = coeffs.a1 * (q_next - q) - coeffs.a3 * v - coeffs.a4 * a
    v_next = v + (1.0 - coeffs.alpha) * dt * a + coeffs.alpha * dt * a_next
    return q_next, v_next, a_next


def park_step(
    system: LinearSecondOrderSystem,
    displacement_history: np.ndarray,
    velocity_history: np.ndarray,
    force_next: np.ndarray,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Advance one Park step from three previous states.

    ``displacement_history`` and ``velocity_history`` must contain the three
    previous columns used by MATLAB as ``Zwy(:,1:3)`` and ``Zsd(:,1:3)``.
    """

    if dt <= 0.0:
        raise ValueError("dt must be positive")
    qh = _as_history("displacement_history", displacement_history, system.ndof)
    vh = _as_history("velocity_history", velocity_history, system.ndof)
    p = _as_vector("force_next", force_next, system.ndof)

    q1, q2, q3 = qh
    v1, v2, v3 = vh
    r = 10.0 / (6.0 * dt)
    bw = (
        (-15.0 / (6.0 * dt)) * q3
        + (1.0 / dt) * q2
        - (1.0 / (6.0 * dt)) * q1
    )
    bs = (
        (-15.0 / (6.0 * dt)) * v3
        + (1.0 / dt) * v2
        - (1.0 / (6.0 * dt)) * v1
    )
    rhs = p - r * (system.mass @ bw) - system.mass @ bs - system.damping @ bw
    effective = r * r * system.mass + r * system.damping + system.stiffness

    q_next = np.linalg.solve(effective, rhs)
    v_next = r * q_next + bw
    a_next = r * v_next + bs
    return q_next, v_next, a_next


def integrate_park_newmark(
    system: LinearSecondOrderSystem,
    force: ArrayLikeForce,
    displacement0: np.ndarray,
    velocity0: np.ndarray,
    dt: float,
    n_steps: int,
    *,
    acceleration0: np.ndarray | None = None,
    startup_steps: int = 2,
    alpha: float = 0.5,
    beta: float = 0.25,
) -> TimeHistory:
    """Integrate a linear system using Newmark startup followed by Park.

    The default two Newmark startup steps create the three response states
    needed by the Park recurrence. Requests below two startup steps are raised
    to two when enough time steps are present. Set ``startup_steps=3`` to mimic
    the preload branch that fills MATLAB columns 2, 3, and 4 with Newmark.
    """

    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if n_steps < 0:
        raise ValueError("n_steps cannot be negative")
    if startup_steps < 0:
        raise ValueError("startup_steps cannot be negative")

    q = np.zeros((n_steps + 1, system.ndof), dtype=float)
    v = np.zeros_like(q)
    a = np.zeros_like(q)
    t = np.arange(n_steps + 1, dtype=float) * dt

    q[0] = _as_vector("displacement0", displacement0, system.ndof)
    v[0] = _as_vector("velocity0", velocity0, system.ndof)
    a[0] = (
        _as_vector("acceleration0", acceleration0, system.ndof)
        if acceleration0 is not None
        else initial_acceleration(system, q[0], v[0], _force_at(force, t[0], system.ndof))
    )

    actual_startup = min(max(startup_steps, 2), n_steps) if n_steps else 0
    for i in range(actual_startup):
        q[i + 1], v[i + 1], a[i + 1] = newmark_step(
            system,
            q[i],
            v[i],
            a[i],
            _force_at(force, t[i + 1], system.ndof),
            dt,
            alpha=alpha,
            beta=beta,
        )

    for i in range(actual_startup, n_steps):
        q[i + 1], v[i + 1], a[i + 1] = park_step(
            system,
            q[i - 2 : i + 1],
            v[i - 2 : i + 1],
            _force_at(force, t[i + 1], system.ndof),
            dt,
        )

    return TimeHistory(
        time=t,
        displacement=q,
        velocity=v,
        acceleration=a,
        startup_steps=actual_startup,
    )


def _force_at(force: ArrayLikeForce, time: float, ndof: int) -> np.ndarray:
    value = force(time) if callable(force) else force
    return _as_vector("force", value, ndof)


def _as_square(name: str, matrix: np.ndarray) -> np.ndarray:
    array = np.asarray(matrix, dtype=float)
    if array.ndim != 2 or array.shape[0] != array.shape[1]:
        raise ValueError(f"{name} must be a square 2D matrix, got {array.shape}")
    return array


def _as_vector(name: str, vector: np.ndarray, ndof: int) -> np.ndarray:
    array = np.asarray(vector, dtype=float).reshape(-1)
    if array.shape != (ndof,):
        raise ValueError(f"{name} must have shape ({ndof},), got {array.shape}")
    return array


def _as_history(name: str, history: np.ndarray, ndof: int) -> np.ndarray:
    array = np.asarray(history, dtype=float)
    if array.shape == (3, ndof):
        return array
    if array.shape == (ndof, 3):
        array = array.T
    if array.shape != (3, ndof):
        raise ValueError(
            f"{name} must have shape (3, {ndof}) or ({ndof}, 3), got {array.shape}"
        )
    return array


def _require_shape(name: str, matrix: np.ndarray, shape: tuple[int, int]) -> None:
    if matrix.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {matrix.shape}")
