from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from sditt.contact.geometry import (
    ContactPatch,
    DirectContactGeometry,
    DirectContactPatch,
    MultiPointContactGeometry,
    WheelPose2D,
)
from sditt.models.network_a1_direct_runtime_trace import NetworkA1DirectRuntimeTraceWriter
from sditt.models.wrcp_net_a1_direct import (
    WRCPNetA1DirectSet,
    geometry_to_history,
    load_wrcp_net_a1_direct,
    shift_direct_geometry,
)


@dataclass
class NetworkA1DirectContactGeometryAdapter:
    """Strict Cal-stage adapter for final patch-set inference."""

    model: WRCPNetA1DirectSet
    model_path: Path
    trace_writer: NetworkA1DirectRuntimeTraceWriter | None = None
    _accepted: dict[str, DirectContactGeometry] = field(default_factory=dict)
    _pending: dict[tuple[str, int, int, float, float], dict[str, DirectContactGeometry]] = field(
        default_factory=dict
    )

    @classmethod
    def load(
        cls,
        model_path: str | Path,
        *,
        trace_output_dir: str | Path | None = None,
        trace_metadata: Mapping[str, Any] | None = None,
        **_: Any,
    ) -> "NetworkA1DirectContactGeometryAdapter":
        path = Path(model_path).resolve()
        writer = (
            None
            if trace_output_dir is None
            else NetworkA1DirectRuntimeTraceWriter(
                trace_output_dir,
                model_path=path,
                metadata=trace_metadata,
            )
        )
        return cls(model=load_wrcp_net_a1_direct(path), model_path=path, trace_writer=writer)

    def solve(
        self,
        *,
        side: str,
        pose: WheelPose2D,
        d0: float,
        rail_shift_yz: np.ndarray,
        diagnostic_context: Mapping[str, Any] | None = None,
        timing: dict[str, float] | None = None,
    ) -> DirectContactGeometry:
        total_started = time.perf_counter() if timing is not None else None
        if side not in {"L", "R"}:
            raise ValueError("network-A1 Direct side must be 'L' or 'R'")
        if diagnostic_context is None:
            raise RuntimeError("network-A1 Direct requires solver diagnostic context")
        shift = np.asarray(rail_shift_yz, dtype=float).reshape(2)
        if not np.isfinite(shift).all():
            raise ValueError("network-A1 Direct rail shift contains NaN or Inf")
        phase_started = time.perf_counter() if timing is not None else None
        contact_key = f"{diagnostic_context.get('wheelset', 'single')}:{side}"
        accepted_geometry = self._accepted.get(contact_key)
        history, history_mask = geometry_to_history(accepted_geometry)
        features = np.array(
            [
                0.0 if side == "L" else 1.0,
                float(pose.lateral) - shift[0],
                float(pose.vertical) - shift[1],
                float(pose.roll),
                float(pose.yaw),
                float(d0),
            ],
            dtype=float,
        )
        _record_timing(timing, "network_a1.adapter_input", phase_started)
        try:
            prediction = self.model.predict(
                features=features,
                history=history,
                history_mask=history_mask,
                topology_prior_available=contact_key in self._accepted,
                previous_patch_count=(
                    0 if accepted_geometry is None else len(accepted_geometry.patches)
                ),
                timing=timing,
            )
        except (RuntimeError, ValueError) as error:
            if self.trace_writer is not None:
                feature_distance, history_distance = _distribution_distances(
                    self.model,
                    features,
                    history,
                    history_mask,
                )
                topology_probability, query_probability, decoded_targets = _raw_diagnostics(
                    self.model,
                    features,
                    history,
                    history_mask,
                )
                self.trace_writer.record_failure(
                    context=diagnostic_context,
                    features=features,
                    history=history,
                    history_mask=history_mask,
                    topology_probability=topology_probability,
                    query_probability=query_probability,
                    decoded_targets=decoded_targets,
                    feature_distance=feature_distance,
                    history_distance=history_distance,
                    in_distribution=bool(
                        feature_distance <= self.model.ood_threshold
                        and history_distance <= self.model.ood_threshold
                    ),
                    error=error,
                )
            raise
        phase_started = time.perf_counter() if timing is not None else None
        solver_key = _solver_key(diagnostic_context)
        self._discard_stale_pending(solver_key)
        self._pending.setdefault(solver_key, {})[contact_key] = prediction.geometry
        if self.trace_writer is not None:
            self.trace_writer.record(
                context=diagnostic_context,
                features=features,
                history=history,
                history_mask=history_mask,
                topology_probability=prediction.topology_probability,
                query_probability=prediction.query_probability,
                decoded_targets=prediction.decoded_targets,
                geometry=prediction.geometry,
                feature_distance=prediction.normalized_feature_distance,
                history_distance=prediction.normalized_history_distance,
            )
        shifted = shift_direct_geometry(prediction.geometry, shift)
        _record_timing(timing, "network_a1.adapter_finalize", phase_started)
        _record_timing(timing, "network_a1.adapter_total", total_started)
        return shifted

    def solve_many(
        self,
        requests: tuple[Mapping[str, Any], ...] | list[Mapping[str, Any]],
        *,
        timing: dict[str, float] | None = None,
    ) -> tuple[DirectContactGeometry, ...]:
        """Solve one coupled iteration with a single batched network forward pass."""

        total_started = time.perf_counter() if timing is not None else None
        if not requests:
            _record_timing(timing, "network_a1.adapter_total", total_started)
            return ()

        phase_started = time.perf_counter() if timing is not None else None
        prepared: list[
            tuple[
                str,
                np.ndarray,
                Mapping[str, Any],
                str,
                np.ndarray,
                np.ndarray,
                np.ndarray,
                bool,
                int,
            ]
        ] = []
        for request in requests:
            side = str(request["side"])
            pose = request["pose"]
            d0 = float(request["d0"])
            diagnostic_context = request.get("diagnostic_context")
            if side not in {"L", "R"}:
                raise ValueError("network-A1 Direct side must be 'L' or 'R'")
            if not isinstance(pose, WheelPose2D):
                raise TypeError("network-A1 Direct batch pose must be WheelPose2D")
            if diagnostic_context is None:
                raise RuntimeError("network-A1 Direct requires solver diagnostic context")
            shift = np.asarray(request["rail_shift_yz"], dtype=float).reshape(2)
            if not np.isfinite(shift).all():
                raise ValueError("network-A1 Direct rail shift contains NaN or Inf")
            contact_key = f"{diagnostic_context.get('wheelset', 'single')}:{side}"
            accepted_geometry = self._accepted.get(contact_key)
            history, history_mask = geometry_to_history(accepted_geometry)
            features = np.array(
                [
                    0.0 if side == "L" else 1.0,
                    float(pose.lateral) - shift[0],
                    float(pose.vertical) - shift[1],
                    float(pose.roll),
                    float(pose.yaw),
                    d0,
                ],
                dtype=float,
            )
            prepared.append(
                (
                    side,
                    shift,
                    diagnostic_context,
                    contact_key,
                    features,
                    history,
                    history_mask,
                    contact_key in self._accepted,
                    0 if accepted_geometry is None else len(accepted_geometry.patches),
                )
            )
        _record_timing(timing, "network_a1.adapter_input", phase_started)

        try:
            predictions = self.model.predict_batch(
                features=np.stack([value[4] for value in prepared]),
                history=np.stack([value[5] for value in prepared]),
                history_mask=np.stack([value[6] for value in prepared]),
                topology_prior_available=np.asarray(
                    [value[7] for value in prepared],
                    dtype=bool,
                ),
                previous_patch_count=np.asarray(
                    [value[8] for value in prepared],
                    dtype=int,
                ),
                timing=timing,
            )
        except (RuntimeError, ValueError):
            # Preserve the scalar path's precise failure trace and error message.
            return tuple(
                self.solve(
                    side=str(request["side"]),
                    pose=request["pose"],
                    d0=float(request["d0"]),
                    rail_shift_yz=np.asarray(request["rail_shift_yz"], dtype=float),
                    diagnostic_context=request.get("diagnostic_context"),
                    timing=timing,
                )
                for request in requests
            )

        phase_started = time.perf_counter() if timing is not None else None
        shifted_geometries: list[DirectContactGeometry] = []
        for values, prediction in zip(prepared, predictions, strict=True):
            (
                _,
                shift,
                diagnostic_context,
                contact_key,
                features,
                history,
                history_mask,
                _,
                _,
            ) = values
            solver_key = _solver_key(diagnostic_context)
            self._discard_stale_pending(solver_key)
            self._pending.setdefault(solver_key, {})[contact_key] = prediction.geometry
            if self.trace_writer is not None:
                self.trace_writer.record(
                    context=diagnostic_context,
                    features=features,
                    history=history,
                    history_mask=history_mask,
                    topology_probability=prediction.topology_probability,
                    query_probability=prediction.query_probability,
                    decoded_targets=prediction.decoded_targets,
                    geometry=prediction.geometry,
                    feature_distance=prediction.normalized_feature_distance,
                    history_distance=prediction.normalized_history_distance,
                )
            shifted_geometries.append(
                shift_direct_geometry(prediction.geometry, shift)
            )
        _record_timing(timing, "network_a1.adapter_finalize", phase_started)
        _record_timing(timing, "network_a1.adapter_total", total_started)
        return tuple(shifted_geometries)

    def seed_from_traditional_snapshot(self, snapshot: Any) -> None:
        """Initialize first-Cal history from the final accepted Preload state."""

        if snapshot is None:
            return
        for wheelset, by_side in snapshot.wheel_rail_contact.geometry_by_wheelset_side.items():
            for side, geometry in by_side.items():
                if geometry is None:
                    continue
                shift = np.asarray(
                    snapshot.effective_rail_displacement_by_wheelset_side[wheelset][side],
                    dtype=float,
                ).reshape(2)
                self._accepted[f"{wheelset}:{side}"] = direct_geometry_from_traditional(
                    geometry,
                    rail_shift_yz=shift,
                )

    def mark_accepted(self, **values: Any) -> None:
        key = _solver_key(values)
        pending = self._pending.get(key)
        if pending is not None:
            self._accepted.update(pending)
        self._pending.clear()
        if self.trace_writer is not None:
            self.trace_writer.mark_accepted(**values)

    def export_continuity_state(self) -> dict[str, Any]:
        return {"schema": "network-a1-direct-accepted-state-v1", "accepted": dict(self._accepted)}

    def import_continuity_state(self, payload: Mapping[str, Any] | None) -> None:
        self._accepted.clear()
        self._pending.clear()
        if payload is None:
            return
        if payload.get("schema") != "network-a1-direct-accepted-state-v1":
            raise ValueError("incompatible network-A1 Direct accepted-state checkpoint")
        self._accepted.update(dict(payload.get("accepted", {})))

    def close_trace(self) -> None:
        if self.trace_writer is not None:
            self.trace_writer.close()

    def _discard_stale_pending(self, current: tuple[str, int, int, float, float]) -> None:
        stage, step, *_ = current
        for key in tuple(self._pending):
            if key[0] != stage or key[1] < step:
                del self._pending[key]


def _record_timing(
    timing: dict[str, float] | None,
    name: str,
    started: float | None,
) -> None:
    if timing is None or started is None:
        return
    timing[name] = timing.get(name, 0.0) + (time.perf_counter() - started)


def direct_geometry_from_traditional(
    geometry: MultiPointContactGeometry,
    *,
    rail_shift_yz: np.ndarray | None = None,
) -> DirectContactGeometry:
    shift = (
        np.zeros((2,), dtype=float)
        if rail_shift_yz is None
        else np.asarray(rail_shift_yz, dtype=float).reshape(2)
    )
    patches = tuple(
        _direct_patch_from_traditional(geometry, patch, shift=shift)
        for patch in geometry.patches
    )
    probability = np.zeros((3,), dtype=float)
    probability[min(len(patches), 2)] = 1.0
    return DirectContactGeometry(
        has_contact=bool(patches),
        patches=patches,
        topology_probability=probability,
        in_distribution=True,
    )


def _direct_patch_from_traditional(
    geometry: MultiPointContactGeometry,
    patch: ContactPatch,
    *,
    shift: np.ndarray,
) -> DirectContactPatch:
    elastic = np.asarray(geometry.elastic_penetration, dtype=float)
    start = int(np.clip(patch.start_index, 0, elastic.shape[0] - 1))
    end = int(np.clip(patch.end_index, start, elastic.shape[0] - 1))
    y = elastic[start : end + 1, 0]
    penetration = np.maximum(elastic[start : end + 1, 1], 0.0)
    width = max(float(y[-1] - y[0]), np.finfo(float).eps)
    peak = max(float(patch.peak_vertical_penetration), np.finfo(float).eps)
    moments = np.array(
        [
            _normalized_penetration_moment(y, penetration, width=width, peak=peak, power=1.0),
            _normalized_penetration_moment(y, penetration, width=width, peak=peak, power=1.5),
            _normalized_penetration_moment(y, penetration, width=width, peak=peak, power=2.0),
        ],
        dtype=float,
    )
    moments = np.clip(moments, 1.0e-7, 1.0 - 1.0e-7)

    def wheel_point(value: np.ndarray) -> np.ndarray:
        result = np.asarray(value, dtype=float).copy()
        result[1] -= shift[0]
        return result

    def rail_point(value: np.ndarray) -> np.ndarray:
        return np.asarray(value, dtype=float) - shift

    return DirectContactPatch(
        start_y=float(y[0] - shift[0]),
        end_y=float(y[-1] - shift[0]),
        peak_wheel_point=wheel_point(patch.peak_wheel_point),
        peak_rail_point=rail_point(patch.peak_rail_point),
        corrected_wheel_point=wheel_point(patch.corrected_wheel_point),
        corrected_rail_point=rail_point(patch.corrected_rail_point),
        wheel_profile_lateral=float(patch.wheel_profile_lateral),
        peak_vertical_penetration=float(patch.peak_vertical_penetration),
        peak_normal_penetration=float(patch.peak_normal_penetration),
        peak_contact_angle=float(patch.peak_contact_angle),
        corrected_vertical_penetration=float(patch.corrected_vertical_penetration),
        corrected_normal_penetration=float(patch.corrected_normal_penetration),
        contact_angle=float(patch.contact_angle),
        shape_moments=moments,
    )


def _normalized_penetration_moment(
    y: np.ndarray,
    penetration: np.ndarray,
    *,
    width: float,
    peak: float,
    power: float,
) -> float:
    if y.size < 2:
        return 1.0
    integral = float(np.trapezoid(np.maximum(penetration, 0.0) ** power, y))
    return integral / max(width * peak**power, np.finfo(float).eps)


def _solver_key(values: Mapping[str, Any]) -> tuple[str, int, int, float, float]:
    return (
        str(values["stage"]),
        int(values["step_index"]),
        int(values["iteration"]),
        round(float(values["time_s"]), 15),
        round(float(values["dt_s"]), 15),
    )


def _distribution_distances(
    model: WRCPNetA1DirectSet,
    features: np.ndarray,
    history: np.ndarray,
    history_mask: np.ndarray,
) -> tuple[float, float]:
    if not np.isfinite(features).all() or not np.isfinite(history[history_mask]).all():
        maximum = float(np.finfo(float).max)
        return maximum, maximum
    feature_distance = float(
        np.max(
            np.abs((features - model.feature_mean) / model.feature_scale)
            / (
                np.ones_like(model.feature_mean)
                if model.feature_ood_scale is None
                else model.feature_ood_scale
            )
        )
    )
    history_distance = (
        0.0
        if not np.any(history_mask)
        else float(
            np.max(
                np.abs(
                    (history[history_mask] - model.history_mean)
                    / model.history_scale
                    / (
                        np.ones_like(model.history_mean)
                        if model.history_ood_scale is None
                        else model.history_ood_scale
                    )
                )
            )
        )
    )
    return feature_distance, history_distance


def _raw_diagnostics(
    model: WRCPNetA1DirectSet,
    features: np.ndarray,
    history: np.ndarray,
    history_mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not np.isfinite(features).all() or not np.isfinite(history[history_mask]).all():
        empty = np.zeros((0,), dtype=float)
        return empty, empty, np.zeros((0, 0), dtype=float)
    topology_logits, query_raw = model.forward_raw(features, history, history_mask)
    shifted = topology_logits - np.max(topology_logits)
    topology_probability = np.exp(shifted) / np.sum(np.exp(shifted))
    query_probability = 1.0 / (1.0 + np.exp(-np.clip(query_raw[:, 0], -60.0, 60.0)))
    decoded_targets = query_raw[:, 1:] * model.target_scale + model.target_mean
    return topology_probability, query_probability, decoded_targets
