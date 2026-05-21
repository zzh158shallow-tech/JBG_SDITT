"""Simulation orchestration and full-system matrix assembly."""

from .system import (
    SystemDofLayout,
    SystemMatrices,
    assemble_system_matrices,
    build_default_modal_rw_system_matrices,
)

__all__ = [
    "SystemDofLayout",
    "SystemMatrices",
    "assemble_system_matrices",
    "build_default_modal_rw_system_matrices",
]
