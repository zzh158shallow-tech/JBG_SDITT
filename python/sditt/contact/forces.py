from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, overload

import numpy as np

from sditt.profiles.geometry import contact_tables


@dataclass(frozen=True)
class StripePatchResult:
    """STRIPES normal-force details for one contact patch."""

    wheel_points: np.ndarray
    rail_points: np.ndarray
    curvature: np.ndarray
    stripes: np.ndarray
    normal_force: float
    area: float


@dataclass(frozen=True)
class StripesNormalForceResult:
    """Result shaped like the elastic-force part of ``NF_STRIPES_230623.m``."""

    normal_force: np.ndarray
    area: np.ndarray
    epsilon: np.ndarray
    patches: tuple[StripePatchResult, ...]


@dataclass(frozen=True)
class KalkerTangentialForceResult:
    """Kalker linear creep forces with MATLAB's active saturation correction."""

    semi_axis_a: np.ndarray
    semi_axis_b: np.ndarray
    kalker_coefficients: np.ndarray
    creep_stiffness: np.ndarray
    linear_force: np.ndarray
    saturated_force: np.ndarray
    linear_tangent_norm: np.ndarray
    saturated_tangent_norm: np.ndarray
    saturation_scale: np.ndarray
    force_track: np.ndarray | None = None


@overload
def hertz_normal_force(
    normal_penetration: float,
    elastic_permeability: float,
    *,
    minimum_force: float = 0.0,
) -> float: ...


@overload
def hertz_normal_force(
    normal_penetration: np.ndarray,
    elastic_permeability: float | np.ndarray,
    *,
    minimum_force: float = 0.0,
) -> np.ndarray: ...


def hertz_normal_force(
    normal_penetration: float | np.ndarray,
    elastic_permeability: float | np.ndarray,
    *,
    minimum_force: float = 0.0,
) -> float | np.ndarray:
    """Return the Hertz normal contact force.

    This mirrors the MATLAB baseline in ``Multi_Con_250812.m``:
    ``Normal_Force(:,5) = (normal_penetration / elastic_permeability_Unit)^(3/2)``.

    Parameters
    ----------
    normal_penetration:
        Normal elastic penetration, in metres. Non-positive values are treated
        as open contact and produce ``minimum_force``.
    elastic_permeability:
        Hertz elastic permeability for unit force, in metres per N^(2/3).
    minimum_force:
        Optional lower bound for closed/open contacts. The Python default is
        zero; MATLAB later floors active normal forces to ``1e-3`` N.
    """

    penetration = np.asarray(normal_penetration, dtype=float)
    permeability = np.asarray(elastic_permeability, dtype=float)
    floor = float(minimum_force)
    if floor < 0.0:
        raise ValueError("minimum_force must be non-negative")
    if not np.all(np.isfinite(permeability)) or np.any(permeability <= 0.0):
        raise ValueError("elastic_permeability must be positive and finite")

    active_penetration = np.maximum(penetration, 0.0)
    force = np.where(active_penetration > 0.0, (active_penetration / permeability) ** 1.5, floor)
    if np.isscalar(normal_penetration) and np.isscalar(elastic_permeability):
        return float(force)
    return force


def stripes_normal_force(
    wheel_radius_profile: np.ndarray,
    rail_radius_profile: np.ndarray,
    contact_wheel_track: np.ndarray,
    contact_wheel_local: np.ndarray,
    contact_rail_track: np.ndarray,
    wheel_interp: np.ndarray,
    rail_interp: np.ndarray,
    *,
    wheel_lateral: float,
    wheel_to_track: np.ndarray,
    contact_to_track: tuple[np.ndarray, ...] | np.ndarray,
    penetration_peaks: np.ndarray,
    m: np.ndarray,
    n: np.ndarray,
    con_a: np.ndarray,
    con_b: np.ndarray,
    con_r: np.ndarray,
    elastic_modulus: float,
    poisson_ratio: float,
    stripe_count: int = 51,
    correction: Literal["A", "AB"] = "AB",
    bgmn: np.ndarray | None = None,
) -> StripesNormalForceResult:
    """Compute STRIPES normal elastic forces.

    This ports the normal elastic-force core of MATLAB ``NF_STRIPES_230623.m``.
    The returned ``normal_force[:, 5]`` equivalent is column 4 of
    ``result.normal_force``; columns 0-3 and 5 are left as zeros for later
    vector decomposition and damping.
    """

    wheel_radius = _sort_points(wheel_radius_profile)
    rail_radius = _sort_points(rail_radius_profile)
    con_wheel_track = np.asarray(contact_wheel_track, dtype=float)
    con_wheel_local = np.asarray(contact_wheel_local, dtype=float)
    con_rail_track = np.asarray(contact_rail_track, dtype=float)
    wheel = _sort_points_by_column(np.asarray(wheel_interp, dtype=float), 1)
    rail = _sort_points(np.asarray(rail_interp, dtype=float))
    peaks = np.asarray(penetration_peaks, dtype=float)
    wheel_to_track = np.asarray(wheel_to_track, dtype=float)
    transforms = _as_transform_tuple(contact_to_track, con_wheel_track.shape[0])
    bgmn_table = contact_tables().bgmn if bgmn is None else np.asarray(bgmn, dtype=float)

    patch_count = peaks.shape[0]
    if patch_count == 0:
        return StripesNormalForceResult(
            normal_force=np.zeros((0, 6), dtype=float),
            area=np.zeros((0,), dtype=float),
            epsilon=np.zeros((0,), dtype=float),
            patches=(),
        )

    m = _as_patch_vector("m", m, patch_count)
    n = _as_patch_vector("n", n, patch_count)
    con_a = _as_patch_vector("con_a", con_a, patch_count)
    con_b = _as_patch_vector("con_b", con_b, patch_count)
    con_r = _as_patch_vector("con_r", con_r, patch_count)

    if correction == "A":
        epsilon = n**2 / con_r / (1.0 + con_a / con_b)
    elif correction == "AB":
        epsilon = n**2 / con_r / (1.0 + (n / m) ** 2)
    else:
        raise ValueError("correction must be 'A' or 'AB'")

    h0 = epsilon * con_wheel_local[:, 4]
    normal_force = np.zeros((patch_count, 6), dtype=float)
    area = np.zeros((patch_count,), dtype=float)
    patch_results: list[StripePatchResult] = []
    previous_penetration_end_track: np.ndarray | None = None

    for i in range(patch_count):
        transform = transforms[i]
        wheel_scope = _patch_wheel_scope(wheel[:, 1], peaks[:, 1], i)
        wheel_con = _right_matrix_divide(wheel[wheel_scope, :] - con_wheel_track[i, :3], transform)
        rail_track_3d = np.column_stack((np.zeros(rail.shape[0]), rail[:, 0], rail[:, 1]))
        rail_origin_3d = np.array([0.0, con_rail_track[i, 0], con_rail_track[i, 1]], dtype=float)
        rail_con_all = _right_matrix_divide(rail_track_3d - rail_origin_3d, transform)

        limit = 0.005 if abs(peaks[i, 1]) < 0.72 else 0.015
        near = np.abs(wheel_con[:, 1]) <= limit
        wheel_near = wheel_con[near, :]
        if wheel_near.shape[0] == 0:
            patch_results.append(_empty_stripe_patch())
            continue

        rail_z_near = _interp_columns(rail_con_all[:, 1], rail_con_all[:, [2]], wheel_near[:, 1])[:, 0]
        normal_gap = rail_z_near - wheel_near[:, 2]
        virtual_penetration = np.column_stack((wheel_near[:, 1], h0[i] - normal_gap))
        penetration_window, previous_penetration_end_track = _trim_virtual_penetration(
            virtual_penetration,
            wheel_near,
            transform,
            con_wheel_track[i, :3],
            previous_penetration_end_track,
        )
        positive = penetration_window[:, 1] > 0.0
        if not np.any(positive):
            patch_results.append(_empty_stripe_patch())
            continue

        stripe_y, stripe_penetration, dy = _stripe_samples(penetration_window, positive, stripe_count)
        stripe_count_i = stripe_y.size

        wheel_con_stripe = np.empty((stripe_count_i, 3), dtype=float)
        wheel_con_stripe[:, 1] = stripe_y
        wheel_con_stripe[:, [0, 2]] = _interp_columns(wheel_con[:, 1], wheel_con[:, [0, 2]], stripe_y)
        wheel_track_stripe = wheel_con_stripe @ transform + con_wheel_track[i, :3]
        wheel_local_stripe = _right_matrix_divide(
            wheel_track_stripe - np.array([0.0, wheel_lateral, 0.0], dtype=float),
            wheel_to_track,
        )

        rail_con_stripe = np.empty((stripe_count_i, 3), dtype=float)
        rail_con_stripe[:, 1] = stripe_y
        rail_con_stripe[:, [0, 2]] = _interp_columns(rail_con_all[:, 1], rail_con_all[:, [0, 2]], stripe_y)
        rail_track_stripe = rail_con_stripe @ transform
        rail_patch = rail_track_stripe[:, 1:3] + con_rail_track[i, :2]

        r_yy_w, r_xx_w, r_xx_r, rou = _contact_radii(
            wheel_radius,
            rail_radius,
            wheel_local_stripe,
            rail_patch,
        )
        beta = np.arccos(np.clip(rou / 4.0 * np.abs(1.0 / r_yy_w - 1.0 / r_xx_w - 1.0 / r_xx_r), -1.0, 1.0))
        m_j = np.interp(beta, bgmn_table[:, 0], bgmn_table[:, 1])
        n_j = np.interp(beta, bgmn_table[:, 0], bgmn_table[:, 2])
        a_j = 0.5 / r_yy_w
        b_j_ori = 0.5 * (1.0 / r_xx_w + 1.0 / r_xx_r)
        b_j = _smooth_five_point(b_j_ori) if stripe_count_i > 1 else b_j_ori

        active = stripe_penetration >= 0.0
        if correction == "A":
            a_cj = b_j * (n_j / m_j) ** 2
            stiffness = elastic_modulus * (1.0 + a_j / b_j) * dy / (2.0 * (1.0 - poisson_ratio**2) * n_j**3)
        else:
            a_cj = (a_j + b_j) / (1.0 + (m_j / n_j) ** 2)
            stiffness = elastic_modulus * (1.0 + (n_j / m_j) ** 2) * dy / (
                2.0 * (1.0 - poisson_ratio**2) * n_j**3
            )
        stiffness = np.where(active, stiffness, 0.0)
        stripe_force = stiffness * stripe_penetration
        semi_axis = np.zeros_like(stripe_penetration)
        semi_axis[active] = np.sqrt(np.maximum(stripe_penetration[active] / a_cj[active], 0.0))

        normal_force[i, 4] = float(np.sum(stripe_force))
        area[i] = float(np.sum(2.0 * semi_axis * dy))
        curvature = np.column_stack((r_yy_w, r_xx_w, r_xx_r, rou, beta, m_j, n_j, a_j, b_j, stiffness))
        stripes = np.column_stack((stiffness, stripe_penetration, stripe_force))
        patch_results.append(
            StripePatchResult(
                wheel_points=wheel_local_stripe,
                rail_points=rail_patch,
                curvature=curvature,
                stripes=stripes,
                normal_force=normal_force[i, 4],
                area=area[i],
            )
        )

    return StripesNormalForceResult(
        normal_force=normal_force,
        area=area,
        epsilon=epsilon,
        patches=tuple(patch_results),
    )


def hu_guo_normal_damping_force(
    stiffness: float | np.ndarray,
    penetration: float | np.ndarray,
    relative_velocity_ratio: float | np.ndarray,
    restitution_coefficient: float,
) -> float | np.ndarray:
    """Return Hu-Guo normal damping force components.

    MATLAB ``Multi_Con_250812.m`` uses this for STRIPES damping:
    ``K * (3*(1-e)/(2*e) * rel_vel_ratio) * penetration``.
    """

    e = float(restitution_coefficient)
    if e <= 0.0:
        raise ValueError("restitution_coefficient must be positive")
    stiffness_arr = np.asarray(stiffness, dtype=float)
    penetration_arr = np.asarray(penetration, dtype=float)
    rel_vel_ratio_arr = np.asarray(relative_velocity_ratio, dtype=float)
    damping = stiffness_arr * (3.0 * (1.0 - e) / (2.0 * e) * rel_vel_ratio_arr) * penetration_arr
    if np.isscalar(stiffness) and np.isscalar(penetration) and np.isscalar(relative_velocity_ratio):
        return float(damping)
    return damping


def add_hu_guo_stripes_damping(
    result: StripesNormalForceResult,
    relative_velocity_ratio: float | np.ndarray,
    restitution_coefficient: float,
    *,
    window: float = 1.0,
) -> StripesNormalForceResult:
    """Add Hu-Guo normal damping to a STRIPES elastic-force result.

    The output mirrors MATLAB ``Normal_Force`` columns: column 0 is total
    normal force, column 4 elastic force, and column 5 damping force.
    """

    win = float(window)
    normal_force = result.normal_force.copy()
    ratios = np.asarray(relative_velocity_ratio, dtype=float)
    if ratios.ndim == 0:
        ratios = np.full(normal_force.shape[0], float(ratios), dtype=float)
    if ratios.shape != (normal_force.shape[0],):
        raise ValueError("relative_velocity_ratio must be scalar or one value per contact patch")

    for i, patch in enumerate(result.patches):
        if patch.stripes.size == 0:
            normal_force[i, 5] = 0.0
        else:
            stripe_damping = hu_guo_normal_damping_force(
                patch.stripes[:, 0],
                patch.stripes[:, 1],
                ratios[i],
                restitution_coefficient,
            )
            normal_force[i, 5] = float(np.sum(stripe_damping) * win)
        normal_force[i, 0] = normal_force[i, 4] + normal_force[i, 5]

    return StripesNormalForceResult(
        normal_force=normal_force,
        area=result.area.copy(),
        epsilon=result.epsilon.copy(),
        patches=result.patches,
    )


def normal_damping_window(mileage: float, vehicle_direction: Literal["Face", "Trail"]) -> float:
    """Return the damping window used around turnout entry/exit in MATLAB 6.4."""

    x = float(mileage)
    if vehicle_direction == "Face":
        return float(np.interp(x, [0.0, 28.5, 40.0, 1000.0], [0.0, 0.0, 1.0, 1.0]))
    if vehicle_direction == "Trail":
        return float(np.interp(x, [-1000.0, 135.0, 150.0, 1000.0], [1.0, 1.0, 0.0, 0.0]))
    raise ValueError("vehicle_direction must be 'Face' or 'Trail'")


def kalker_linear_saturated_creep_force(
    normal_force: np.ndarray,
    creepage: np.ndarray,
    rolling_radius_sum: np.ndarray,
    wheel_rolling_radius: np.ndarray,
    m: np.ndarray,
    n: np.ndarray,
    *,
    elastic_modulus: float,
    poisson_ratio: float,
    friction_coefficient: float,
    vehicle_speed: float,
    contact_to_track: tuple[np.ndarray, ...] | np.ndarray | None = None,
    bgc1: np.ndarray | None = None,
    bgc2: np.ndarray | None = None,
) -> KalkerTangentialForceResult:
    """Compute Kalker linear tangential creep force with saturation.

    This follows the active MATLAB path in ``Multi_Con_250812.m`` sections
    6.6-6.8. FASTSIM is intentionally not used here.
    """

    normal = np.asarray(normal_force, dtype=float).reshape(-1)
    creepage = np.asarray(creepage, dtype=float)
    if creepage.shape != (normal.size, 3):
        raise ValueError("creepage must have shape (n_contact, 3)")
    rou = _as_patch_vector("rolling_radius_sum", rolling_radius_sum, normal.size)
    r_yy_w = _as_patch_vector("wheel_rolling_radius", wheel_rolling_radius, normal.size)
    m = _as_patch_vector("m", m, normal.size)
    n = _as_patch_vector("n", n, normal.size)
    if np.any(normal < 0.0):
        raise ValueError("normal_force must be non-negative")
    if friction_coefficient < 0.0:
        raise ValueError("friction_coefficient must be non-negative")

    tables = contact_tables()
    bgc1_table = tables.bgc1 if bgc1 is None else np.asarray(bgc1, dtype=float)
    bgc2_table = tables.bgc2 if bgc2 is None else np.asarray(bgc2, dtype=float)
    patch_count = normal.size

    semi_a = np.zeros(patch_count, dtype=float)
    semi_b = np.zeros(patch_count, dtype=float)
    coefficients = np.zeros((patch_count, 4), dtype=float)

    for i in range(patch_count):
        if normal[i] <= 0.0:
            continue
        if rou[i] / r_yy_w[i] <= 2.0:
            semi_a[i] = 0.1506e-3 * m[i] * (rou[i] * normal[i]) ** (1.0 / 3.0)
            semi_b[i] = 0.1506e-3 * n[i] * (rou[i] * normal[i]) ** (1.0 / 3.0)
            aspect = semi_b[i] / semi_a[i]
            coefficients[i, :] = _kalker_coefficients_b_over_a(aspect, poisson_ratio, bgc2_table)
        else:
            semi_a[i] = 0.1506e-3 * n[i] * (rou[i] * normal[i]) ** (1.0 / 3.0)
            semi_b[i] = 0.1506e-3 * m[i] * (rou[i] * normal[i]) ** (1.0 / 3.0)
            aspect = semi_a[i] / semi_b[i]
            coefficients[i, :] = _kalker_coefficients_a_over_b(aspect, poisson_ratio, bgc1_table)

    shear_modulus = elastic_modulus / (2.0 * (1.0 + poisson_ratio))
    ab = semi_a * semi_b
    stiffness = np.column_stack(
        (
            shear_modulus * ab * coefficients[:, 0],
            shear_modulus * ab * coefficients[:, 1],
            shear_modulus * ab ** 1.5 * coefficients[:, 2],
            shear_modulus * ab**2 * coefficients[:, 3],
        )
    )

    linear = np.zeros((patch_count, 3), dtype=float)
    linear[:, 0] = -stiffness[:, 0] * creepage[:, 0]
    if vehicle_speed >= 0.0:
        linear[:, 1] = -stiffness[:, 1] * creepage[:, 1] - stiffness[:, 2] * creepage[:, 2]
        linear[:, 2] = stiffness[:, 2] * creepage[:, 1] - stiffness[:, 3] * creepage[:, 2]
    else:
        linear[:, 1] = -stiffness[:, 1] * creepage[:, 1] + stiffness[:, 2] * creepage[:, 2]
        linear[:, 2] = stiffness[:, 2] * creepage[:, 1] + stiffness[:, 3] * creepage[:, 2]

    linear_norm = np.linalg.norm(linear[:, :2], axis=1)
    saturated_norm = _saturated_tangent_norm(linear_norm, normal, friction_coefficient)
    scale = np.divide(saturated_norm, linear_norm, out=np.zeros_like(saturated_norm), where=linear_norm > 0.0)
    saturated = linear * scale[:, np.newaxis]

    force_track = None
    if contact_to_track is not None:
        transforms = _as_transform_tuple(contact_to_track, patch_count)
        force_track = np.zeros((patch_count, 6), dtype=float)
        for i, transform in enumerate(transforms):
            force_track[i, :3] = np.array([saturated[i, 0], saturated[i, 1], 0.0]) @ transform
            force_track[i, 3:6] = np.array([0.0, 0.0, saturated[i, 2]]) @ transform

    return KalkerTangentialForceResult(
        semi_axis_a=semi_a,
        semi_axis_b=semi_b,
        kalker_coefficients=coefficients,
        creep_stiffness=stiffness,
        linear_force=linear,
        saturated_force=saturated,
        linear_tangent_norm=linear_norm,
        saturated_tangent_norm=saturated_norm,
        saturation_scale=scale,
        force_track=force_track,
    )


def contact_to_track_matrix(yaw: float, roll: float, contact_angle: float) -> np.ndarray:
    """Return the contact-CS to track-CS matrix used before ``NF_STRIPES_230623``."""

    angle = contact_angle + roll
    return np.array(
        [
            [np.cos(yaw), np.sin(yaw), 0.0],
            [-np.cos(angle) * np.sin(yaw), np.cos(angle) * np.cos(yaw), np.sin(angle)],
            [np.sin(angle) * np.sin(yaw), -np.sin(angle) * np.cos(yaw), np.cos(angle)],
        ],
        dtype=float,
    )


def _kalker_coefficients_b_over_a(aspect: float, poisson_ratio: float, bgc2: np.ndarray) -> np.ndarray:
    gg = float(aspect)
    gg_a = np.log(16.0 / gg**2)
    c11 = 2.0 * np.pi / (gg_a - 2.0 * poisson_ratio) / gg * (1.0 + (3.0 - np.log(4.0)) / (gg_a - 2.0 * poisson_ratio))
    c22 = (
        2.0
        * np.pi
        * (1.0 + (1.0 - poisson_ratio) * (3.0 - np.log(4.0)) / ((1.0 - poisson_ratio) * gg_a + 2.0 * poisson_ratio))
        / ((1.0 - poisson_ratio) * gg_a + 2.0 * poisson_ratio)
        / gg
    )
    c23 = 2.0 * np.pi / (3.0 * gg * np.sqrt(gg)) / ((1.0 - poisson_ratio) * gg_a - 2.0 + 4.0 * poisson_ratio)
    c33 = np.pi / 4.0 * (1.0 - (poisson_ratio * gg_a - 2.0) / ((1.0 - poisson_ratio) * gg_a - 2.0 + 4.0 * poisson_ratio))
    table = np.vstack((np.array([0.0, c11, c22, c23, c33], dtype=float), bgc2))
    return np.array([np.interp(gg, table[:, 0], table[:, col]) for col in range(1, 5)], dtype=float)


def _kalker_coefficients_a_over_b(aspect: float, poisson_ratio: float, bgc1: np.ndarray) -> np.ndarray:
    gg = float(aspect)
    c11 = np.pi**2 / 4.0 / (1.0 - poisson_ratio)
    c22 = np.pi**2 / 4.0
    c23 = np.pi * np.sqrt(gg) / 3.0 / (1.0 - poisson_ratio) * (
        1.0 + poisson_ratio * (0.5 * np.log(16.0 / gg**2) + np.log(4.0) - 5.0)
    )
    c33 = np.pi**2 / 16.0 / (1.0 - poisson_ratio) / gg
    table = np.vstack((np.array([0.0, c11, c22, c23, c33], dtype=float), bgc1))
    return np.array([np.interp(gg, table[:, 0], table[:, col]) for col in range(1, 5)], dtype=float)


def _saturated_tangent_norm(linear_norm: np.ndarray, normal_force: np.ndarray, friction_coefficient: float) -> np.ndarray:
    limit = friction_coefficient * normal_force
    threshold = 3.0 * limit
    saturated = np.zeros_like(linear_norm)
    active = (linear_norm > 0.0) & (limit > 0.0)
    polynomial = active & (linear_norm <= threshold)
    temp = np.divide(linear_norm, limit, out=np.zeros_like(linear_norm), where=limit > 0.0)
    saturated[polynomial] = limit[polynomial] * (
        temp[polynomial] - temp[polynomial] ** 2 / 3.0 + temp[polynomial] ** 3 / 27.0
    )
    saturated[active & ~polynomial] = limit[active & ~polynomial]
    return saturated


def _contact_radii(
    wheel_radius_profile: np.ndarray,
    rail_radius_profile: np.ndarray,
    wheel_points: np.ndarray,
    rail_points: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    r_yy_w = wheel_points[:, 2]
    r_xx_w = np.interp(wheel_points[:, 1], wheel_radius_profile[:, 0], wheel_radius_profile[:, 1])
    r_xx_r = np.interp(rail_points[:, 0], rail_radius_profile[:, 0], rail_radius_profile[:, 1])
    adjust = (r_xx_w < 0.0) & (np.abs(r_xx_w) * 0.9 <= np.abs(r_xx_r))
    r_xx_r = np.where(adjust, np.abs(r_xx_w) * 0.9, r_xx_r)
    r_xx_r = np.where(np.abs(r_xx_r) > 1.0, 1.0, r_xx_r)
    rou = 4.0 / (1.0 / r_yy_w + 1.0 / r_xx_w + 1.0 / r_xx_r)
    return r_yy_w, r_xx_w, r_xx_r, rou


def _patch_wheel_scope(y: np.ndarray, peak_y: np.ndarray, index: int, margin: float = 1e-3) -> np.ndarray:
    if peak_y.size <= 1:
        return np.ones_like(y, dtype=bool)
    if index == 0:
        return y <= peak_y[index + 1] - margin
    if index == peak_y.size - 1:
        return y >= peak_y[index - 1] + margin
    return (y >= peak_y[index - 1] + margin) & (y <= peak_y[index + 1] - margin)


def _trim_virtual_penetration(
    virtual_penetration: np.ndarray,
    wheel_near: np.ndarray,
    transform: np.ndarray,
    contact_wheel_track: np.ndarray,
    previous_end_track: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray | None]:
    positive = virtual_penetration[:, 1] > 0.0
    if not np.any(positive):
        return virtual_penetration[:0, :], previous_end_track

    positive_indexes = np.flatnonzero(positive)
    limits_con = wheel_near[[positive_indexes[0], positive_indexes[-1]], :]
    limits_track = limits_con @ transform + contact_wheel_track
    trimmed = virtual_penetration
    if previous_end_track is not None and limits_track[0, 1] <= previous_end_track[1]:
        left_con = _right_matrix_divide(previous_end_track - contact_wheel_track, transform)
        trimmed = trimmed[trimmed[:, 0] > left_con[1], :]

    if trimmed.shape[0] > 1:
        starts = np.flatnonzero((trimmed[:-1, 1] <= 0.0) & (trimmed[1:, 1] >= 0.0))
        ends = np.flatnonzero((trimmed[:-1, 1] >= 0.0) & (trimmed[1:, 1] <= 0.0))
        if starts.size == ends.size and starts.size > 0 and np.any(trimmed[starts, 0] > trimmed[ends, 0]):
            trimmed = trimmed[(trimmed[:, 0] < trimmed[starts[-1], 0]) & (trimmed[:, 0] > trimmed[ends[0], 0]), :]
        elif starts.size > ends.size and starts.size > 0:
            trimmed = trimmed[trimmed[:, 0] < trimmed[starts[-1], 0], :]
        elif starts.size < ends.size and ends.size > 0:
            trimmed = trimmed[trimmed[:, 0] > trimmed[ends[0], 0], :]

    positive = trimmed[:, 1] > 0.0
    if np.any(positive):
        indexes = np.flatnonzero(positive)
        end_y = trimmed[indexes[-1], 0]
        end_xz = _interp_columns(wheel_near[:, 1], wheel_near[:, [0, 2]], np.array([end_y]))[0]
        previous_end_track = np.array([end_xz[0], end_y, end_xz[1]]) @ transform + contact_wheel_track
    return trimmed, previous_end_track


def _stripe_samples(
    penetration: np.ndarray,
    positive: np.ndarray,
    stripe_count: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if stripe_count < 0:
        raise ValueError("stripe_count must be non-negative")
    if stripe_count == 0:
        y = penetration[positive, 0]
        p = penetration[positive, 1]
        positive_indexes = np.flatnonzero(positive)
        include = positive.copy()
        if positive_indexes[0] > 0:
            include[positive_indexes[0] - 1] = True
        if positive_indexes[-1] + 1 < include.size:
            include[positive_indexes[-1] + 1] = True
        support_y = penetration[include, 0]
        if y.size == 1:
            dy = np.array([0.0], dtype=float)
        else:
            dy = np.diff(support_y[:-1]) / 2.0 + np.diff(support_y[1:]) / 2.0
        return y, p, dy
    if stripe_count == 1:
        y = np.array([np.mean(penetration[positive, 0])], dtype=float)
        return y, np.interp(y, penetration[:, 0], penetration[:, 1]), np.array([0.0], dtype=float)
    y_positive = penetration[positive, 0]
    y = np.linspace(y_positive[0], y_positive[-1], stripe_count)
    p = np.interp(y, penetration[:, 0], penetration[:, 1])
    dy = np.full(stripe_count, (y[-1] - y[0]) / (stripe_count - 1), dtype=float)
    return y, p, dy


def _right_matrix_divide(values: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return np.linalg.solve(np.asarray(matrix, dtype=float).T, np.asarray(values, dtype=float).T).T


def _interp_columns(x: np.ndarray, y: np.ndarray, x_new: np.ndarray) -> np.ndarray:
    order = np.argsort(x)
    x_sorted = np.asarray(x, dtype=float)[order]
    y_sorted = np.asarray(y, dtype=float)[order]
    unique_x, unique_index = np.unique(x_sorted, return_index=True)
    y_unique = y_sorted[unique_index]
    if y_unique.ndim == 1:
        y_unique = y_unique[:, np.newaxis]
    return np.column_stack([np.interp(x_new, unique_x, y_unique[:, col]) for col in range(y_unique.shape[1])])


def _smooth_five_point(values: np.ndarray) -> np.ndarray:
    data = np.asarray(values, dtype=float)
    if data.size < 5:
        return data.copy()
    out = data.copy()
    for i in range(data.size):
        start = max(0, i - 2)
        end = min(data.size, i + 3)
        out[i] = float(np.mean(data[start:end]))
    return out


def _sort_points(points: np.ndarray) -> np.ndarray:
    data = np.asarray(points, dtype=float)
    if data.size == 0:
        return data.reshape(0, 2)
    return data[np.argsort(data[:, 0])]


def _sort_points_by_column(points: np.ndarray, column: int) -> np.ndarray:
    data = np.asarray(points, dtype=float)
    if data.size == 0:
        return data.reshape(0, 3)
    return data[np.argsort(data[:, column])]


def _as_patch_vector(name: str, values: np.ndarray, patch_count: int) -> np.ndarray:
    vector = np.asarray(values, dtype=float).reshape(-1)
    if vector.size != patch_count:
        raise ValueError(f"{name} must have one value per contact patch")
    return vector


def _as_transform_tuple(transforms: tuple[np.ndarray, ...] | np.ndarray, patch_count: int) -> tuple[np.ndarray, ...]:
    if isinstance(transforms, tuple):
        if len(transforms) != patch_count:
            raise ValueError("contact_to_track must have one matrix per contact patch")
        return tuple(np.asarray(matrix, dtype=float) for matrix in transforms)
    data = np.asarray(transforms, dtype=float)
    if data.shape == (3, 3):
        return tuple(data.copy() for _ in range(patch_count))
    if data.shape == (patch_count, 3, 3):
        return tuple(data[i, :, :] for i in range(patch_count))
    raise ValueError("contact_to_track must be a 3x3 matrix or an array of per-patch 3x3 matrices")


def _empty_stripe_patch() -> StripePatchResult:
    return StripePatchResult(
        wheel_points=np.empty((0, 3), dtype=float),
        rail_points=np.empty((0, 2), dtype=float),
        curvature=np.empty((0, 10), dtype=float),
        stripes=np.empty((0, 3), dtype=float),
        normal_force=0.0,
        area=0.0,
    )
