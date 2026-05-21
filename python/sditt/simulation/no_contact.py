from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from sditt.integrators import LinearSecondOrderSystem, TimeHistory, integrate_park_newmark

from .system import SystemMatrices, build_default_modal_rw_system_matrices


ForceFunction = Callable[[float], np.ndarray]


@dataclass(frozen=True)
class HarmonicForce:
    """Single-DOF harmonic external force vector for no-contact smoke runs."""

    ndof: int
    dof: int
    amplitude: float
    frequency_hz: float
    phase: float = 0.0
    offset: float = 0.0

    def __post_init__(self) -> None:
        if self.ndof <= 0:
            raise ValueError("ndof must be positive")
        if not 0 <= self.dof < self.ndof:
            raise ValueError(f"dof must be in [0, {self.ndof}), got {self.dof}")
        if self.frequency_hz < 0.0:
            raise ValueError("frequency_hz cannot be negative")

    def __call__(self, time: float) -> np.ndarray:
        force = np.zeros(self.ndof, dtype=float)
        force[self.dof] = self.offset + self.amplitude * np.sin(
            2.0 * np.pi * self.frequency_hz * time + self.phase
        )
        return force


@dataclass(frozen=True)
class NoContactSimulationResult:
    """No-contact forced-response result and the force sampled at each time."""

    history: TimeHistory
    force: np.ndarray


def run_no_contact_forced_response(
    matrices: SystemMatrices,
    force: ForceFunction,
    *,
    dt: float,
    n_steps: int,
    displacement0: np.ndarray | None = None,
    velocity0: np.ndarray | None = None,
    startup_steps: int = 2,
) -> NoContactSimulationResult:
    """Run the first linear dynamics loop without wheel-rail contact.

    This integrates ``Mxt qdd + Cxt qd + Kxt q = P(t)`` with a prescribed force
    only. It is intentionally a small bridge between assembled system matrices
    and the Park/Newmark integrator before adding contact-force iteration.
    """

    system = LinearSecondOrderSystem(matrices.Mxt, matrices.Cxt, matrices.Kxt)
    q0 = np.zeros(system.ndof, dtype=float) if displacement0 is None else displacement0
    v0 = np.zeros(system.ndof, dtype=float) if velocity0 is None else velocity0

    history = integrate_park_newmark(
        system,
        force=force,
        displacement0=q0,
        velocity0=v0,
        dt=dt,
        n_steps=n_steps,
        startup_steps=startup_steps,
    )
    sampled_force = np.vstack([force(time) for time in history.time])
    return NoContactSimulationResult(history=history, force=sampled_force)


def run_default_no_contact_smoke(
    *,
    repo_root: str | Path | None = None,
    cut_freq: float = 50.0,
    force_dof: int = 0,
    force_amplitude: float = 1_000.0,
    force_frequency_hz: float = 5.0,
    dt: float = 1e-4,
    n_steps: int = 100,
) -> NoContactSimulationResult:
    """Build a small modal RW system and run a harmonic no-contact response."""

    matrices, _track, _vehicle = build_default_modal_rw_system_matrices(
        repo_root=repo_root,
        cut_freq=cut_freq,
    )
    force = HarmonicForce(
        ndof=matrices.layout.total_dof,
        dof=force_dof,
        amplitude=force_amplitude,
        frequency_hz=force_frequency_hz,
    )
    return run_no_contact_forced_response(matrices, force, dt=dt, n_steps=n_steps)
