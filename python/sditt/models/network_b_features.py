from __future__ import annotations

import numpy as np

from typing import Any


FEATURE_NAMES = (
    "side_id", "patch_id", "pose_lateral_m", "pose_vertical_m", "pose_roll_rad", "pose_yaw_rad", "d0_m",
    "corrected_wheel_x_m", "corrected_wheel_y_m", "corrected_wheel_z_m",
    "corrected_vertical_penetration_m", "corrected_normal_penetration_m", "corrected_contact_angle_rad",
    "peak_wheel_x_m", "peak_wheel_y_m", "peak_wheel_z_m", "peak_vertical_penetration_m",
    "peak_normal_penetration_m", "peak_contact_angle_rad", "corrected_rail_y_m", "corrected_rail_z_m",
    "peak_rail_y_m", "peak_rail_z_m", "relative_velocity_z_mps", "relative_velocity_ratio",
    "wheel_slip_x_mps", "wheel_slip_y_mps", "wheel_slip_z_mps", "rail_slip_x_mps", "rail_slip_y_mps",
    "rail_slip_z_mps", "rolling_speed_mps", "patch_start_y_m", "patch_end_y_m", "patch_width_m",
    "log_corrected_normal_penetration", "log_peak_normal_penetration", "corrected_penetration_power_1p5",
    "peak_penetration_power_1p5", "wheel_rolling_curvature_1pm", "wheel_lateral_curvature_1pm",
    "rail_lateral_curvature_1pm", "rolling_radius_sum_curvature_1pm", "hertz_m", "hertz_n",
    "elastic_permeability_m_per_N_2over3", "hertz_con_a", "hertz_con_b", "hertz_con_r",
    "penetration_area_m2", "penetration_1p5_area_m2p5", "penetration_mean_m",
    "penetration_std_m", "penetration_sample_count",
)
DIRECT_FEATURE_NAMES = FEATURE_NAMES[:-1]


def build_network_b_features(
    *,
    side: str,
    pose: Any,
    d0: float,
    patch_ids: np.ndarray,
    con_wheel_2: np.ndarray,
    con_wheel_2_peak: np.ndarray,
    con_rail_1: np.ndarray,
    con_rail_1_peak: np.ndarray,
    con_rel_vel: np.ndarray,
    vsdc: np.ndarray,
    vjsdc: np.ndarray,
    vgd: np.ndarray,
    patch_bounds: np.ndarray,
    curvature_inputs: np.ndarray,
    patch_shape: np.ndarray,
    feature_names: tuple[str, ...] = FEATURE_NAMES,
) -> np.ndarray:
    """Build the exact feature matrix shared by teacher collection and runtime."""
    if side not in {"L", "R"}:
        raise ValueError("side must be 'L' or 'R'")
    patch_ids = np.asarray(patch_ids, dtype=float).reshape(-1)
    n = patch_ids.size
    arrays = {
        "con_wheel_2": np.asarray(con_wheel_2, dtype=float),
        "con_wheel_2_peak": np.asarray(con_wheel_2_peak, dtype=float),
        "con_rail_1": np.asarray(con_rail_1, dtype=float),
        "con_rail_1_peak": np.asarray(con_rail_1_peak, dtype=float),
        "con_rel_vel": np.asarray(con_rel_vel, dtype=float),
        "vsdc": np.asarray(vsdc, dtype=float),
        "vjsdc": np.asarray(vjsdc, dtype=float),
        "patch_bounds": np.asarray(patch_bounds, dtype=float),
        "curvature_inputs": np.asarray(curvature_inputs, dtype=float),
        "patch_shape": np.asarray(patch_shape, dtype=float),
    }
    if feature_names not in {FEATURE_NAMES, DIRECT_FEATURE_NAMES}:
        raise ValueError("unsupported network-B feature schema")
    shape_width = 5 if feature_names == FEATURE_NAMES else 4
    widths = {"con_wheel_2": 6, "con_wheel_2_peak": 6, "con_rail_1": 2, "con_rail_1_peak": 2,
              "con_rel_vel": 2, "vsdc": 3, "vjsdc": 3, "patch_bounds": 3, "curvature_inputs": 10,
              "patch_shape": shape_width}
    for name, width in widths.items():
        if arrays[name].shape != (n, width):
            raise ValueError(f"{name} must have shape ({n}, {width})")
    speed = np.asarray(vgd, dtype=float).reshape(-1)
    if speed.shape != (n,):
        raise ValueError(f"vgd must have shape ({n},)")
    pose_columns = np.tile(np.array([pose.lateral, pose.vertical, pose.roll, pose.yaw, d0], dtype=float), (n, 1))
    curvature_features = arrays["curvature_inputs"].copy()
    curvature_features[:, :4] = np.divide(
        1.0, curvature_features[:, :4], out=np.zeros_like(curvature_features[:, :4]),
        where=np.isfinite(curvature_features[:, :4]) & (np.abs(curvature_features[:, :4]) > 1.0e-12),
    )
    result = np.column_stack((
        np.full((n,), 0.0 if side == "L" else 1.0), patch_ids, pose_columns, arrays["con_wheel_2"],
        arrays["con_wheel_2_peak"], arrays["con_rail_1"], arrays["con_rail_1_peak"], arrays["con_rel_vel"],
        arrays["vsdc"], arrays["vjsdc"], speed, arrays["patch_bounds"],
        np.log(np.maximum(arrays["con_wheel_2"][:, 4], 1.0e-12)),
        np.log(np.maximum(arrays["con_wheel_2_peak"][:, 4], 1.0e-12)),
        np.maximum(arrays["con_wheel_2"][:, 4], 0.0) ** 1.5,
        np.maximum(arrays["con_wheel_2_peak"][:, 4], 0.0) ** 1.5,
        curvature_features, arrays["patch_shape"],
    ))
    if result.shape != (n, len(feature_names)):
        raise RuntimeError("network-B feature layout is inconsistent with FEATURE_NAMES")
    if not np.isfinite(result).all():
        bad = sorted({feature_names[index] for index in np.argwhere(~np.isfinite(result))[:, 1]})
        raise ValueError(f"network-B features contain NaN or Inf in {bad}")
    return result


def network_b_patch_shape_features(
    geometry: Any,
    *,
    include_sample_count: bool = True,
) -> np.ndarray:
    """Return smooth per-patch penetration-shape descriptors available before force evaluation."""

    elastic = np.asarray(geometry.elastic_penetration, dtype=float)
    rows: list[np.ndarray] = []
    for patch in geometry.patches:
        start = int(np.clip(patch.start_index, 0, elastic.shape[0] - 1))
        end = int(np.clip(patch.end_index, start, elastic.shape[0] - 1))
        y = elastic[start : end + 1, 0]
        penetration = np.maximum(elastic[start : end + 1, 1], 0.0)
        area = float(np.trapezoid(penetration, y)) if y.size > 1 else 0.0
        power_area = float(np.trapezoid(penetration**1.5, y)) if y.size > 1 else 0.0
        row = [
            area,
            power_area,
            float(np.mean(penetration)),
            float(np.std(penetration)),
        ]
        if include_sample_count:
            row.append(float(penetration.size))
        rows.append(np.asarray(row, dtype=float))
    width = 5 if include_sample_count else 4
    return np.stack(rows) if rows else np.zeros((0, width), dtype=float)


def direct_network_b_patch_shape_features(geometry: Any) -> np.ndarray:
    """Build four grid-independent shape descriptors from direct patch moments."""

    rows: list[np.ndarray] = []
    for patch in geometry.patches:
        width = max(float(patch.end_y - patch.start_y), np.finfo(float).eps)
        peak = max(float(patch.peak_vertical_penetration), np.finfo(float).eps)
        moment_1, moment_1p5, moment_2 = np.asarray(patch.shape_moments, dtype=float)
        area = width * peak * moment_1
        power_area = width * peak**1.5 * moment_1p5
        mean = peak * moment_1
        variance = max(peak**2 * moment_2 - mean**2, 0.0)
        rows.append(np.array([area, power_area, mean, np.sqrt(variance)], dtype=float))
    return np.stack(rows) if rows else np.zeros((0, 4), dtype=float)
