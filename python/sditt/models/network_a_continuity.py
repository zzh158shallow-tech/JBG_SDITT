from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Mapping

import numpy as np
from scipy.optimize import linear_sum_assignment

from sditt.contact.geometry import ContactPatch, MultiPointContactGeometry


@dataclass(frozen=True)
class NetworkAContinuitySettings:
    """Numerical continuity guard for strict neural contact geometry."""

    penetration_alpha: float = 0.30
    penetration_rate_m_per_s: np.ndarray = field(
        default_factory=lambda: np.array([[0.22, 0.22], [0.06, 0.06]], dtype=float)
    )
    residual_rate_m_per_s: np.ndarray = field(
        default_factory=lambda: np.array([[0.22, 0.22], [0.06, 0.06]], dtype=float)
    )
    contact_location_rate_m_per_s: np.ndarray = field(
        default_factory=lambda: np.array([6.5, 5.5], dtype=float)
    )
    switch_distance_m: float = 5.0e-5
    maximum_match_distance_m: float = 5.0e-4
    distance_scale_m: float = 2.0e-4
    angle_scale_rad: float = 2.0e-2
    penetration_scale_m: float = 2.0e-5
    switch_hysteresis: float = 0.25

    def __post_init__(self) -> None:
        if not 0.0 < float(self.penetration_alpha) <= 1.0:
            raise ValueError("penetration_alpha must be in (0, 1]")
        for name in ("penetration_rate_m_per_s", "residual_rate_m_per_s"):
            values = np.asarray(getattr(self, name), dtype=float)
            if values.shape != (2, 2) or not np.isfinite(values).all() or np.any(values <= 0.0):
                raise ValueError(f"{name} must have shape (2, 2) with positive finite values")
        location_rate = np.asarray(self.contact_location_rate_m_per_s, dtype=float)
        if location_rate.shape != (2,) or np.any(location_rate <= 0.0):
            raise ValueError("contact_location_rate_m_per_s must have shape (2,)")


@dataclass(frozen=True)
class ContactContinuityDiagnostics:
    base_penetration_m: np.ndarray
    bounded_residual_m: np.ndarray
    used_residual_m: np.ndarray
    network_penetration_m: np.ndarray
    used_penetration_m: np.ndarray
    residual_limited: np.ndarray
    penetration_limited: np.ndarray
    branch_held: np.ndarray


@dataclass(frozen=True)
class _AcceptedContactState:
    patches: tuple[ContactPatch, ...]
    residual_m: np.ndarray


@dataclass(frozen=True)
class _PendingContactState:
    patches: tuple[ContactPatch, ...]
    residual_m: np.ndarray


class NetworkAContinuityController:
    """Anchor every nonlinear candidate to the preceding accepted step."""

    def __init__(self, settings: NetworkAContinuitySettings | None = None) -> None:
        self.settings = settings or NetworkAContinuitySettings()
        self._accepted: dict[str, _AcceptedContactState] = {}
        self._pending: dict[tuple[str, int, int, float, float], dict[str, _PendingContactState]] = {}

    def accepted_patches(self, contact_key: str) -> tuple[ContactPatch, ...]:
        state = self._accepted.get(contact_key)
        return () if state is None else state.patches

    def process(
        self,
        geometry: MultiPointContactGeometry,
        *,
        bounded_residual_m: np.ndarray,
        side: str,
        roll: float,
        diagnostic_context: Mapping[str, Any] | None,
    ) -> tuple[MultiPointContactGeometry, ContactContinuityDiagnostics]:
        side_index = 0 if side == "L" else 1
        contact_key = _contact_key(side, diagnostic_context)
        previous = self._accepted.get(contact_key)
        matched, branch_held = _match_and_apply_hysteresis(
            geometry.patches,
            () if previous is None else previous.patches,
            self.settings,
            side_index=side_index,
            dt=_context_dt(diagnostic_context),
        )
        geometry = replace(geometry, patches=matched, has_contact=bool(matched))
        base = _penetration_vector(matched)
        bounded = _as_patch_correction(bounded_residual_m, len(matched))
        dt = _context_dt(diagnostic_context)
        residual_limited = np.zeros((4,), dtype=bool)
        used_residual = bounded.copy()
        if previous is not None and len(previous.patches) == len(matched):
            previous_residual = _as_patch_correction(previous.residual_m, len(matched))
            rate = np.asarray(self.settings.residual_rate_m_per_s, dtype=float)[side_index]
            allowance = np.tile(rate, len(matched))[: 2 * len(matched)] * dt
            proposal = bounded[: 2 * len(matched)] - previous_residual[: 2 * len(matched)]
            clipped = np.clip(proposal, -allowance, allowance)
            residual_limited[: 2 * len(matched)] = np.abs(clipped - proposal) > 1.0e-15
            used_residual[: 2 * len(matched)] = previous_residual[: 2 * len(matched)] + clipped
        network_geometry = _apply_patch_penetration_delta(geometry, used_residual, roll=roll)
        network_penetration = _penetration_vector(network_geometry.patches)
        penetration_limited = np.zeros((4,), dtype=bool)
        used_geometry = network_geometry
        if previous is not None and len(previous.patches) == len(network_geometry.patches):
            previous_penetration = _penetration_vector(previous.patches)
            rate = np.asarray(self.settings.penetration_rate_m_per_s, dtype=float)[side_index]
            allowance = np.tile(rate, len(matched))[: 2 * len(matched)] * dt
            difference = (
                network_penetration[: 2 * len(matched)]
                - previous_penetration[: 2 * len(matched)]
            )
            # Preserve the teacher's ordinary dynamics exactly. Under-relaxation
            # is activated only for changes outside the calibrated teacher
            # envelope; applying alpha unconditionally creates phase lag.
            proposal = np.where(
                np.abs(difference) > allowance,
                float(self.settings.penetration_alpha) * difference,
                difference,
            )
            clipped = np.clip(proposal, -allowance, allowance)
            penetration_limited[: 2 * len(matched)] = np.abs(clipped - proposal) > 1.0e-15
            used = network_penetration.copy()
            used[: 2 * len(matched)] = previous_penetration[: 2 * len(matched)] + clipped
            used_geometry = _set_patch_penetrations(network_geometry, used, roll=roll)
        used_penetration = _penetration_vector(used_geometry.patches)
        if diagnostic_context is not None:
            solver_key = _solver_key(diagnostic_context)
            self._pending.setdefault(solver_key, {})[contact_key] = _PendingContactState(
                patches=tuple(used_geometry.patches),
                residual_m=used_residual.copy(),
            )
            self._discard_stale_pending(solver_key)
        diagnostics = ContactContinuityDiagnostics(
            base_penetration_m=base,
            bounded_residual_m=bounded,
            used_residual_m=used_residual,
            network_penetration_m=network_penetration,
            used_penetration_m=used_penetration,
            residual_limited=residual_limited,
            penetration_limited=penetration_limited,
            branch_held=branch_held,
        )
        return used_geometry, diagnostics

    def mark_accepted(self, **values: Any) -> None:
        solver_key = _solver_key(values)
        pending = self._pending.get(solver_key)
        if pending is None:
            return
        for contact_key, state in pending.items():
            self._accepted[contact_key] = _AcceptedContactState(
                patches=tuple(state.patches),
                residual_m=np.asarray(state.residual_m, dtype=float).copy(),
            )
        self._pending.clear()

    def export_state(self) -> dict[str, Any]:
        return {
            key: {
                "patches": state.patches,
                "residual_m": state.residual_m.copy(),
            }
            for key, state in self._accepted.items()
        }

    def import_state(self, payload: Mapping[str, Any] | None) -> None:
        self._accepted.clear()
        self._pending.clear()
        for key, values in dict(payload or {}).items():
            self._accepted[str(key)] = _AcceptedContactState(
                patches=tuple(values["patches"]),
                residual_m=np.asarray(values["residual_m"], dtype=float).copy(),
            )

    def _discard_stale_pending(self, current: tuple[str, int, int, float, float]) -> None:
        stage, step, *_ = current
        for key in tuple(self._pending):
            if key[0] != stage or key[1] < step:
                del self._pending[key]


def _contact_key(side: str, context: Mapping[str, Any] | None) -> str:
    wheelset = "single" if context is None else str(context.get("wheelset", "single"))
    return f"{wheelset}:{side}"


def _solver_key(values: Mapping[str, Any]) -> tuple[str, int, int, float, float]:
    return (
        str(values["stage"]),
        int(values["step_index"]),
        int(values["iteration"]),
        round(float(values["time_s"]), 15),
        round(float(values["dt_s"]), 15),
    )


def _context_dt(context: Mapping[str, Any] | None) -> float:
    if context is None:
        return 1.0e-4
    return max(float(context.get("dt_s", 1.0e-4)), np.finfo(float).eps)


def _match_and_apply_hysteresis(
    current: tuple[ContactPatch, ...],
    previous: tuple[ContactPatch, ...],
    settings: NetworkAContinuitySettings,
    *,
    side_index: int,
    dt: float,
) -> tuple[tuple[ContactPatch, ...], np.ndarray]:
    held = np.zeros((2,), dtype=bool)
    if not current or not previous or len(current) != len(previous):
        return tuple(current), held
    count = len(current)
    cost = np.empty((count, count), dtype=float)
    for i, old in enumerate(previous):
        for j, new in enumerate(current):
            distance = abs(float(new.corrected_rail_point[0] - old.corrected_rail_point[0]))
            angle = abs(float(new.contact_angle - old.contact_angle))
            cost[i, j] = distance / settings.distance_scale_m + angle / settings.angle_scale_rad
    row_indexes, column_indexes = linear_sum_assignment(cost)
    assignment = np.empty((count,), dtype=int)
    assignment[row_indexes] = column_indexes
    matched: list[ContactPatch] = []
    switch_distance = min(
        settings.maximum_match_distance_m,
        max(
            settings.switch_distance_m,
            float(np.asarray(settings.contact_location_rate_m_per_s)[side_index]) * dt,
        ),
    )
    for i, j in enumerate(assignment):
        old = previous[i]
        new = current[j]
        distance = abs(float(new.corrected_rail_point[0] - old.corrected_rail_point[0]))
        angle = abs(float(new.contact_angle - old.contact_angle))
        penetration_advantage = (
            float(new.corrected_vertical_penetration - old.corrected_vertical_penetration)
            / settings.penetration_scale_m
        )
        score = (
            penetration_advantage
            - distance / settings.distance_scale_m
            - angle / settings.angle_scale_rad
        )
        unreasonable = distance > settings.maximum_match_distance_m
        should_hold = distance > switch_distance and (
            unreasonable or score <= settings.switch_hysteresis
        )
        if should_hold:
            matched.append(old)
            held[i] = True
        else:
            matched.append(new)
    return tuple(matched), held


def _penetration_vector(patches: tuple[ContactPatch, ...]) -> np.ndarray:
    result = np.zeros((4,), dtype=float)
    for index, patch in enumerate(patches[:2]):
        result[2 * index] = float(patch.corrected_vertical_penetration)
        result[2 * index + 1] = float(patch.peak_vertical_penetration)
    return result


def _as_patch_correction(values: np.ndarray, patch_count: int) -> np.ndarray:
    result = np.zeros((4,), dtype=float)
    source = np.asarray(values, dtype=float).reshape(-1)
    count = min(2 * patch_count, source.size, 4)
    result[:count] = source[:count]
    return result


def _apply_patch_penetration_delta(
    geometry: MultiPointContactGeometry,
    delta: np.ndarray,
    *,
    roll: float,
) -> MultiPointContactGeometry:
    current = _penetration_vector(geometry.patches)
    current[: 2 * len(geometry.patches)] += np.asarray(delta, dtype=float)[: 2 * len(geometry.patches)]
    return _set_patch_penetrations(geometry, current, roll=roll)


def _set_patch_penetrations(
    geometry: MultiPointContactGeometry,
    values: np.ndarray,
    *,
    roll: float,
) -> MultiPointContactGeometry:
    penetration = np.asarray(values, dtype=float)
    patches: list[ContactPatch] = []
    for index, patch in enumerate(geometry.patches):
        corrected = max(0.0, float(penetration[2 * index]))
        peak = max(0.0, float(penetration[2 * index + 1]))
        patches.append(
            replace(
                patch,
                corrected_vertical_penetration=corrected,
                corrected_normal_penetration=corrected
                / max(np.cos(patch.contact_angle + roll), np.finfo(float).eps),
                peak_vertical_penetration=peak,
                peak_normal_penetration=peak
                / max(np.cos(patch.peak_contact_angle + roll), np.finfo(float).eps),
            )
        )
    return replace(geometry, patches=tuple(patches), has_contact=bool(patches))
