from __future__ import annotations

from dataclasses import replace

import numpy as np

from sditt.models.network_a_continuity import (
    NetworkAContinuityController,
    NetworkAContinuitySettings,
)
from sditt.training_data.network_a import (
    build_network_a_teacher_context,
    teacher_geometry_from_features,
)


def _context(step: int, iteration: int, dt: float = 1.0e-4) -> dict[str, object]:
    return {
        "stage": "Cal",
        "step_index": step,
        "iteration": iteration,
        "time_s": step * dt,
        "dt_s": dt,
        "front_mileage_m": 48.0,
        "actual_mileage_m": 48.0,
        "wheelset": "FF",
        "side": "L",
    }


def _geometry():
    context = build_network_a_teacher_context()
    return teacher_geometry_from_features(
        context,
        np.array([0.0, 0.0, 0.001, 0.0, 0.0, 0.1702], dtype=float),
    )


def test_penetration_anchor_updates_only_after_accepted_step() -> None:
    controller = NetworkAContinuityController(
        NetworkAContinuitySettings(
            penetration_alpha=0.3,
            penetration_rate_m_per_s=np.full((2, 2), 0.22),
            residual_rate_m_per_s=np.full((2, 2), 0.22),
        )
    )
    original = _geometry()
    first, _ = controller.process(
        original,
        bounded_residual_m=np.zeros(4),
        side="L",
        roll=0.0,
        diagnostic_context=_context(1, 2),
    )
    controller.mark_accepted(stage="Cal", step_index=1, iteration=2, time_s=1.0e-4, dt_s=1.0e-4)
    anchor = first.patches[0].corrected_vertical_penetration
    raised_patch = replace(
        original.patches[0],
        corrected_vertical_penetration=anchor + 1.0e-4,
        peak_vertical_penetration=original.patches[0].peak_vertical_penetration + 1.0e-4,
    )
    raised = replace(original, patches=(raised_patch,))

    candidate, diagnostics = controller.process(
        raised,
        bounded_residual_m=np.zeros(4),
        side="L",
        roll=0.0,
        diagnostic_context=_context(2, 3),
    )
    retry, _ = controller.process(
        raised,
        bounded_residual_m=np.zeros(4),
        side="L",
        roll=0.0,
        diagnostic_context=_context(2, 4, dt=2.5e-5),
    )

    assert np.isclose(candidate.patches[0].corrected_vertical_penetration, anchor + 2.2e-5)
    assert np.isclose(retry.patches[0].corrected_vertical_penetration, anchor + 5.5e-6)
    assert diagnostics.penetration_limited[0]


def test_large_shallow_branch_jump_is_held_by_hysteresis() -> None:
    controller = NetworkAContinuityController()
    original = _geometry()
    first, _ = controller.process(
        original,
        bounded_residual_m=np.zeros(4),
        side="L",
        roll=0.0,
        diagnostic_context=_context(1, 2),
    )
    controller.mark_accepted(stage="Cal", step_index=1, iteration=2, time_s=1.0e-4, dt_s=1.0e-4)
    old = first.patches[0]
    jumped_patch = replace(
        old,
        corrected_rail_point=old.corrected_rail_point + np.array([8.0e-4, 0.0]),
        corrected_wheel_point=old.corrected_wheel_point + np.array([0.0, 8.0e-4, 0.0]),
        corrected_vertical_penetration=max(old.corrected_vertical_penetration - 5.0e-5, 0.0),
    )
    jumped = replace(original, patches=(jumped_patch,))

    used, diagnostics = controller.process(
        jumped,
        bounded_residual_m=np.zeros(4),
        side="L",
        roll=0.0,
        diagnostic_context=_context(2, 2),
    )

    assert diagnostics.branch_held[0]
    assert np.isclose(used.patches[0].corrected_rail_point[0], old.corrected_rail_point[0])
