from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
from scipy import sparse
from scipy.sparse import linalg as sparse_linalg


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
    use_sparse: bool = False

    def __post_init__(self) -> None:
        mass = _as_square("mass", self.mass, use_sparse=self.use_sparse)
        damping = _as_square("damping", self.damping, use_sparse=self.use_sparse)
        stiffness = _as_square("stiffness", self.stiffness, use_sparse=self.use_sparse)
        _require_shape("damping", damping, mass.shape)
        _require_shape("stiffness", stiffness, mass.shape)
        object.__setattr__(self, "mass", mass)
        object.__setattr__(self, "damping", damping)
        object.__setattr__(self, "stiffness", stiffness)

    @property
    def ndof(self) -> int:
        return self.mass.shape[0]

    def matvec_mass(self, vector: np.ndarray) -> np.ndarray:
        return np.asarray(self.mass @ vector, dtype=float).reshape(-1)

    def matvec_damping(self, vector: np.ndarray) -> np.ndarray:
        return np.asarray(self.damping @ vector, dtype=float).reshape(-1)

    def matvec_stiffness(self, vector: np.ndarray) -> np.ndarray:
        return np.asarray(self.stiffness @ vector, dtype=float).reshape(-1)

    def solve(self, matrix: np.ndarray | sparse.spmatrix, rhs: np.ndarray) -> np.ndarray:
        if sparse.issparse(matrix):
            return np.asarray(sparse_linalg.spsolve(matrix.tocsc(), rhs), dtype=float).reshape(-1)
        return np.linalg.solve(np.asarray(matrix, dtype=float), rhs)


@dataclass(frozen=True)
class TimeHistory:
    """Integrated response history with rows ordered by time."""

    time: np.ndarray
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    startup_steps: int


@dataclass(frozen=True)
class PreparedLinearStepper:
    """Cached effective matrices/factorizations for repeated Newmark/Park steps."""

    system: LinearSecondOrderSystem
    dt: float
    alpha: float = 0.5
    beta: float = 0.25

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        coeffs = NewmarkCoefficients(dt=self.dt, alpha=self.alpha, beta=self.beta)
        r = 10.0 / (6.0 * self.dt)
        newmark_effective = self.system.stiffness + coeffs.a1 * self.system.mass + coeffs.a2 * self.system.damping
        park_effective = r * r * self.system.mass + r * self.system.damping + self.system.stiffness
        object.__setattr__(self, "coeffs", coeffs)
        object.__setattr__(self, "park_r", r)
        object.__setattr__(self, "_newmark_solve", _factor_solver(newmark_effective))
        object.__setattr__(self, "_park_solve", _factor_solver(park_effective))

    def initial_acceleration(
        self,
        displacement: np.ndarray,
        velocity: np.ndarray,
        force: np.ndarray,
    ) -> np.ndarray:
        q = _as_vector("displacement", displacement, self.system.ndof)
        v = _as_vector("velocity", velocity, self.system.ndof)
        p = _as_vector("force", force, self.system.ndof)
        rhs = p - self.system.matvec_damping(v) - self.system.matvec_stiffness(q)
        return _factor_solver(self.system.mass)(rhs)

    def newmark_step(
        self,
        displacement: np.ndarray,
        velocity: np.ndarray,
        acceleration: np.ndarray,
        force_next: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        q = _as_vector("displacement", displacement, self.system.ndof)
        v = _as_vector("velocity", velocity, self.system.ndof)
        a = _as_vector("acceleration", acceleration, self.system.ndof)
        p = _as_vector("force_next", force_next, self.system.ndof)
        coeffs = self.coeffs

        rhs = (
            p
            + self.system.matvec_mass(coeffs.a1 * q + coeffs.a3 * v + coeffs.a4 * a)
            + self.system.matvec_damping(coeffs.a2 * q + coeffs.a6 * v + coeffs.a5 * a)
        )
        q_next = self._newmark_solve(rhs)
        a_next = coeffs.a1 * (q_next - q) - coeffs.a3 * v - coeffs.a4 * a
        v_next = v + (1.0 - coeffs.alpha) * self.dt * a + coeffs.alpha * self.dt * a_next
        return q_next, v_next, a_next

    def park_step(
        self,
        displacement_history: np.ndarray,
        velocity_history: np.ndarray,
        force_next: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        qh = _as_history("displacement_history", displacement_history, self.system.ndof)
        vh = _as_history("velocity_history", velocity_history, self.system.ndof)
        p = _as_vector("force_next", force_next, self.system.ndof)

        q1, q2, q3 = qh
        v1, v2, v3 = vh
        r = self.park_r
        bw = (-15.0 / (6.0 * self.dt)) * q3 + (1.0 / self.dt) * q2 - (1.0 / (6.0 * self.dt)) * q1
        bs = (-15.0 / (6.0 * self.dt)) * v3 + (1.0 / self.dt) * v2 - (1.0 / (6.0 * self.dt)) * v1
        rhs = p - r * self.system.matvec_mass(bw) - self.system.matvec_mass(bs) - self.system.matvec_damping(bw)

        q_next = self._park_solve(rhs)
        v_next = r * q_next + bw
        a_next = r * v_next + bs
        return q_next, v_next, a_next


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
    return system.solve(system.mass, p - system.matvec_damping(v) - system.matvec_stiffness(q))


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

    rhs = p + system.matvec_mass(coeffs.a1 * q + coeffs.a3 * v + coeffs.a4 * a) + system.matvec_damping(
        coeffs.a2 * q + coeffs.a6 * v + coeffs.a5 * a
    )
    effective = system.stiffness + coeffs.a1 * system.mass + coeffs.a2 * system.damping
    q_next = system.solve(effective, rhs)
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
    rhs = p - r * system.matvec_mass(bw) - system.matvec_mass(bs) - system.matvec_damping(bw)
    effective = r * r * system.mass + r * system.damping + system.stiffness

    q_next = system.solve(effective, rhs)
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
    stepper = PreparedLinearStepper(system, dt=dt, alpha=alpha, beta=beta)
    for i in range(actual_startup):
        q[i + 1], v[i + 1], a[i + 1] = stepper.newmark_step(
            q[i],
            v[i],
            a[i],
            _force_at(force, t[i + 1], system.ndof),
        )

    for i in range(actual_startup, n_steps):
        q[i + 1], v[i + 1], a[i + 1] = stepper.park_step(
            q[i - 2 : i + 1],
            v[i - 2 : i + 1],
            _force_at(force, t[i + 1], system.ndof),
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


def _as_square(name: str, matrix: np.ndarray | sparse.spmatrix, *, use_sparse: bool = False) -> np.ndarray | sparse.csr_matrix:
    array = sparse.csr_matrix(matrix, dtype=float) if use_sparse or sparse.issparse(matrix) else np.asarray(matrix, dtype=float)
    if array.ndim != 2 or array.shape[0] != array.shape[1]:
        raise ValueError(f"{name} must be a square 2D matrix, got {array.shape}")
    return array


def _factor_solver(matrix: np.ndarray | sparse.spmatrix) -> Callable[[np.ndarray], np.ndarray]:
    if sparse.issparse(matrix):
        solve = sparse_linalg.factorized(matrix.tocsc())

        def sparse_solve(rhs: np.ndarray) -> np.ndarray:
            return np.asarray(solve(rhs), dtype=float).reshape(-1)

        return sparse_solve

    dense = np.asarray(matrix, dtype=float)

    def dense_solve(rhs: np.ndarray) -> np.ndarray:
        return np.linalg.solve(dense, rhs)

    return dense_solve


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
