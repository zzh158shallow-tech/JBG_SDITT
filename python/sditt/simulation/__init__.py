"""Simulation orchestration and full-system matrix assembly."""

from .no_contact import (
    HarmonicForce,
    NoContactSimulationResult,
    run_default_no_contact_smoke,
    run_no_contact_forced_response,
)
from .coupled import (
    CoupledIterationSettings,
    CoupledStepCallbacks,
    CoupledStepState,
    CoupledTimeIterationResult,
    run_coupled_time_iteration,
)
from .small_case import (
    SmallRigidWheelsetCase,
    SmallRigidWheelsetResult,
    export_small_rigid_wheelset_curve,
    run_small_rigid_wheelset_case,
)
from .system import (
    SystemDofLayout,
    SystemMatrices,
    assemble_system_matrices,
    build_default_modal_rw_system_matrices,
)

__all__ = [
    "HarmonicForce",
    "CoupledIterationSettings",
    "CoupledStepCallbacks",
    "CoupledStepState",
    "CoupledTimeIterationResult",
    "NoContactSimulationResult",
    "SmallRigidWheelsetCase",
    "SmallRigidWheelsetResult",
    "SystemDofLayout",
    "SystemMatrices",
    "assemble_system_matrices",
    "build_default_modal_rw_system_matrices",
    "run_coupled_time_iteration",
    "run_default_no_contact_smoke",
    "run_no_contact_forced_response",
    "run_small_rigid_wheelset_case",
    "export_small_rigid_wheelset_curve",
]
