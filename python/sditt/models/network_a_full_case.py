from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

from sditt.contact.geometry import BoundaryExtrema, ContactPatch, MultiPointContactGeometry, WheelPose2D
from sditt.models.wrcp_net_a2g import (
    WRCPNetA2G,
    _geometry_from_predicted_field,
    load_wrcp_net_a2g,
    predict_wrcp_net_a2g,
)
from sditt.models.network_a_runtime_trace import NetworkARuntimeTraceWriter
from sditt.models.network_a_continuity import (
    NetworkAContinuityController,
    NetworkAContinuitySettings,
)
from sditt.models.wrcp_net_a2r import (
    PenetrationResidualHeads,
    load_penetration_residual_heads,
)
from sditt.training_data.network_a import NetworkATeacherContext, build_network_a_teacher_context


@dataclass
class NetworkAContactGeometryAdapter:
    """Run WRCP-Net A2G as a strict full replacement for patch search."""

    model: WRCPNetA2G
    normalization: dict[str, np.ndarray]
    context: NetworkATeacherContext
    model_path: Path
    trace_writer: NetworkARuntimeTraceWriter | None = None
    residual_heads: PenetrationResidualHeads | None = None
    continuity: NetworkAContinuityController = field(
        default_factory=NetworkAContinuityController
    )

    @classmethod
    def load(
        cls,
        model_path: str | Path,
        *,
        repo_root: str | Path | None = None,
        trace_output_dir: str | Path | None = None,
        trace_metadata: dict[str, Any] | None = None,
        trace_mode: str = "selective",
        trace_low_confidence_threshold: float = 0.95,
        trace_sample_interval_m: float | None = 1.0,
    ) -> "NetworkAContactGeometryAdapter":
        path = Path(model_path).resolve()
        model, normalization = load_wrcp_net_a2g(path)
        context = build_network_a_teacher_context(repo_root)
        trace_writer = (
            None
            if trace_output_dir is None
            else NetworkARuntimeTraceWriter(
                trace_output_dir,
                model_path=path,
                metadata=trace_metadata,
                mode=trace_mode,
                low_confidence_threshold=trace_low_confidence_threshold,
                sample_interval_m=trace_sample_interval_m,
            )
        )
        residual_heads = load_penetration_residual_heads(path)
        continuity = NetworkAContinuityController(_continuity_settings_from_model(path))
        return cls(
            model=model,
            normalization=normalization,
            context=context,
            model_path=path,
            trace_writer=trace_writer,
            residual_heads=residual_heads,
            continuity=continuity,
        )

    def solve(
        self,
        *,
        side: str,
        pose: WheelPose2D,
        d0: float,
        rail_shift_yz: np.ndarray,
        diagnostic_context: dict[str, Any] | None = None,
    ) -> MultiPointContactGeometry:
        if side not in ("L", "R"):
            raise ValueError(f"network-A side must be 'L' or 'R', got {side!r}")
        shift = np.asarray(rail_shift_yz, dtype=float)
        if shift.shape != (2,) or not np.isfinite(shift).all():
            raise ValueError("network-A rail_shift_yz must contain two finite values")
        features = np.array(
            [
                0.0 if side == "L" else 1.0,
                float(pose.lateral) - shift[0],
                float(pose.vertical) - shift[1],
                float(pose.roll),
                float(pose.yaw),
                float(d0),
            ],
            dtype=np.float32,
        )
        if not np.isfinite(features).all():
            raise ValueError("network-A input contains NaN or Inf")
        prediction = predict_wrcp_net_a2g(self.model, features, self.normalization)
        desired_count = int(prediction.patch_count[0])
        wheelset = "single" if diagnostic_context is None else str(
            diagnostic_context.get("wheelset", "single")
        )
        geometry = _geometry_from_predicted_field(
            self.context,
            features.astype(float),
            self.normalization["canonical_y_m"],
            prediction.gap_field_m[0],
            desired_count,
            profile_refinement=False,
            candidate_profile_refinement=True,
            anchor_patches=self.continuity.accepted_patches(f"{wheelset}:{side}"),
        )
        if len(geometry.patches) != desired_count:
            raise RuntimeError(
                "WRCP-Net A2G topology mismatch without fallback: "
                f"predicted {desired_count} patches but reconstructed {len(geometry.patches)}"
            )
        base_penetration = _patch_penetration_vector(geometry)
        bounded_residual = (
            np.zeros((4,), dtype=float)
            if self.residual_heads is None
            else self.residual_heads.predict(features, base_penetration[None, :])[0].astype(float)
        )
        geometry, continuity_diagnostics = self.continuity.process(
            geometry,
            bounded_residual_m=bounded_residual,
            side=side,
            roll=float(features[3]),
            diagnostic_context=diagnostic_context,
        )
        if self.trace_writer is not None:
            if diagnostic_context is None:
                raise RuntimeError("network-A runtime tracing requires diagnostic context")
            self.trace_writer.record(
                context=diagnostic_context,
                features=features,
                class_probability=prediction.class_probability[0],
                predicted_patch_count=desired_count,
                geometry=geometry,
                gap_field_m=prediction.gap_field_m[0],
                continuity_diagnostics=continuity_diagnostics,
            )
        shifted = _shift_geometry_to_runtime_coordinates(geometry, shift)
        _validate_geometry(shifted)
        return shifted

    def mark_accepted(self, **values: Any) -> None:
        self.continuity.mark_accepted(**values)
        if self.trace_writer is not None:
            self.trace_writer.mark_accepted(**values)

    def export_continuity_state(self) -> dict[str, Any]:
        return self.continuity.export_state()

    def import_continuity_state(self, payload: dict[str, Any] | None) -> None:
        self.continuity.import_state(payload)

    def close_trace(self) -> None:
        if self.trace_writer is not None:
            self.trace_writer.close()


def _shift_geometry_to_runtime_coordinates(
    geometry: MultiPointContactGeometry,
    rail_shift_yz: np.ndarray,
) -> MultiPointContactGeometry:
    shift_y, shift_z = (float(value) for value in np.asarray(rail_shift_yz, dtype=float))
    elastic = np.asarray(geometry.elastic_penetration, dtype=float).copy()
    if elastic.size:
        elastic[:, 0] += shift_y
    wheel = np.asarray(geometry.wheel_interp, dtype=float).copy()
    if wheel.size:
        wheel[:, 1] += shift_y
    rail = np.asarray(geometry.rail_interp, dtype=float).copy()
    if rail.size:
        rail[:, 0] += shift_y
        rail[:, 1] += shift_z

    def shift_boundary(values: np.ndarray) -> np.ndarray:
        result = np.asarray(values, dtype=float).copy()
        if result.size:
            result[:, 1] += shift_y
        return result

    boundaries = BoundaryExtrema(
        extrema=shift_boundary(geometry.boundaries.extrema),
        positive_extrema=shift_boundary(geometry.boundaries.positive_extrema),
        starts=shift_boundary(geometry.boundaries.starts),
        ends=shift_boundary(geometry.boundaries.ends),
    )

    def shift_wheel(point: np.ndarray) -> np.ndarray:
        result = np.asarray(point, dtype=float).copy()
        result[1] += shift_y
        return result

    def shift_rail(point: np.ndarray) -> np.ndarray:
        result = np.asarray(point, dtype=float).copy()
        result[0] += shift_y
        result[1] += shift_z
        return result

    patches: tuple[ContactPatch, ...] = tuple(
        replace(
            patch,
            peak_wheel_point=shift_wheel(patch.peak_wheel_point),
            peak_rail_point=shift_rail(patch.peak_rail_point),
            corrected_wheel_point=shift_wheel(patch.corrected_wheel_point),
            corrected_rail_point=shift_rail(patch.corrected_rail_point),
        )
        for patch in geometry.patches
    )
    return MultiPointContactGeometry(
        has_contact=bool(patches),
        elastic_penetration=elastic,
        wheel_interp=wheel,
        rail_interp=rail,
        contact_angles=np.asarray(geometry.contact_angles, dtype=float).copy(),
        wheel_profile_lateral=np.asarray(geometry.wheel_profile_lateral, dtype=float).copy(),
        boundaries=boundaries,
        patches=patches,
    )


def _validate_geometry(geometry: MultiPointContactGeometry) -> None:
    arrays: tuple[Any, ...] = (
        geometry.elastic_penetration,
        geometry.wheel_interp,
        geometry.rail_interp,
        geometry.contact_angles,
        geometry.wheel_profile_lateral,
    )
    if any(not np.isfinite(np.asarray(value, dtype=float)).all() for value in arrays):
        raise RuntimeError("WRCP-Net A2G produced non-finite geometry without fallback")
    if len(geometry.patches) > 2:
        raise RuntimeError("WRCP-Net A2G produced more than two patches without fallback")


def _patch_penetration_vector(geometry: MultiPointContactGeometry) -> np.ndarray:
    values = np.zeros((4,), dtype=np.float32)
    for index, patch in enumerate(geometry.patches[:2]):
        values[2 * index] = float(patch.corrected_vertical_penetration)
        values[2 * index + 1] = float(patch.peak_vertical_penetration)
    return values


def _continuity_settings_from_model(path: Path) -> NetworkAContinuitySettings:
    defaults = NetworkAContinuitySettings()
    with np.load(path, allow_pickle=False) as archive:
        penetration_rate = (
            archive["a2r_penetration_rate_m_per_s"].copy()
            if "a2r_penetration_rate_m_per_s" in archive.files
            else defaults.penetration_rate_m_per_s
        )
        residual_rate = (
            archive["a2r_residual_rate_m_per_s"].copy()
            if "a2r_residual_rate_m_per_s" in archive.files
            else penetration_rate
        )
        alpha = (
            float(archive["a2r_continuity_alpha"])
            if "a2r_continuity_alpha" in archive.files
            else defaults.penetration_alpha
        )
        contact_location_rate = (
            archive["a2r_contact_location_rate_m_per_s"].copy()
            if "a2r_contact_location_rate_m_per_s" in archive.files
            else defaults.contact_location_rate_m_per_s
        )
    return NetworkAContinuitySettings(
        penetration_alpha=alpha,
        penetration_rate_m_per_s=np.asarray(penetration_rate, dtype=float),
        residual_rate_m_per_s=np.asarray(residual_rate, dtype=float),
        contact_location_rate_m_per_s=np.asarray(contact_location_rate, dtype=float),
    )
