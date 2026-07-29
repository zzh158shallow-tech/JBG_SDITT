from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline


@dataclass(frozen=True)
class WheelPose2D:
    """Rigid wheel pose used by the single wheel/rail contact geometry."""

    lateral: float = 0.0
    vertical: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class WheelTrace:
    """Wheel profile points transformed into the track coordinate system."""

    wheel_lateral: np.ndarray
    track_points: np.ndarray
    contact_angles: np.ndarray


@dataclass(frozen=True)
class PreparedWheelTraceProfile:
    """Wheel profile samples that do not depend on wheel pose."""

    x_profile: np.ndarray
    rolling_radius: np.ndarray
    contact_angles: np.ndarray


@dataclass(frozen=True)
class PreparedContactProfileGeometry:
    """Pose-dependent wheel/rail interpolants before contact-point search."""

    wheel_interp: np.ndarray
    rail_interp: np.ndarray
    contact_angles: np.ndarray
    wheel_profile_lateral: np.ndarray


@dataclass(frozen=True)
class PreparedRailProfileInterpolator:
    """Sorted fixed rail profile and its reusable cubic interpolator."""

    rail: np.ndarray
    spline: CubicSpline | None


@dataclass(frozen=True)
class SinglePointContact:
    """One Hertz-style normal contact geometry result."""

    has_contact: bool
    rail_point: np.ndarray
    wheel_point: np.ndarray
    wheel_profile_lateral: float
    vertical_gap: float
    vertical_penetration: float
    normal_penetration: float
    contact_angle: float


@dataclass(frozen=True)
class ContactPatch:
    """One positive-penetration contact patch before and after correction."""

    peak_index: int
    start_index: int
    end_index: int
    peak_wheel_point: np.ndarray
    peak_rail_point: np.ndarray
    corrected_wheel_point: np.ndarray
    corrected_rail_point: np.ndarray
    wheel_profile_lateral: float
    peak_vertical_penetration: float
    peak_normal_penetration: float
    peak_contact_angle: float
    corrected_vertical_penetration: float
    corrected_normal_penetration: float
    contact_angle: float


@dataclass(frozen=True)
class DirectContactPatch:
    """One final contact patch predicted without a reconstructed profile grid.

    ``shape_moments`` contains the dimensionless first, 1.5-order and second
    moments of the positive penetration distribution, normalized by patch
    width and peak penetration.  They let a force surrogate consume smooth
    patch-shape information without depending on a 501/1001-point grid.
    """

    start_y: float
    end_y: float
    peak_wheel_point: np.ndarray
    peak_rail_point: np.ndarray
    corrected_wheel_point: np.ndarray
    corrected_rail_point: np.ndarray
    wheel_profile_lateral: float
    peak_vertical_penetration: float
    peak_normal_penetration: float
    peak_contact_angle: float
    corrected_vertical_penetration: float
    corrected_normal_penetration: float
    contact_angle: float
    shape_moments: np.ndarray


@dataclass(frozen=True)
class DirectContactGeometry:
    """Final patch geometry produced directly by WRCP-Net A1 Direct Set.

    Unlike :class:`MultiPointContactGeometry`, this contract deliberately has
    no interpolated profiles, penetration field, or sample indexes.
    """

    has_contact: bool
    patches: tuple[DirectContactPatch, ...]
    topology_probability: np.ndarray
    in_distribution: bool = True


@dataclass(frozen=True)
class BoundaryExtrema:
    """Positive penetration intervals and extrema from ``Extreme_Boundary.m``."""

    extrema: np.ndarray
    positive_extrema: np.ndarray
    starts: np.ndarray
    ends: np.ndarray


@dataclass(frozen=True)
class MultiPointContactGeometry:
    """Candidate contact patches found from one wheel profile and one rail profile."""

    has_contact: bool
    elastic_penetration: np.ndarray
    wheel_interp: np.ndarray
    rail_interp: np.ndarray
    contact_angles: np.ndarray
    wheel_profile_lateral: np.ndarray
    boundaries: BoundaryExtrema
    patches: tuple[ContactPatch, ...]


def trace_wheel_profile(
    wheel_profile: np.ndarray,
    contact_angle_table: np.ndarray,
    pose: WheelPose2D | None = None,
    *,
    dlb: float | None = None,
    discrete_len_flange: float = 0.5e-5,
    discrete_len_tread: float = 2.5e-5,
) -> WheelTrace:
    """Transform a wheel profile into the track cross-section.

    This is the single-rigid-wheel subset of MATLAB ``TracePrinciple.m``. The
    output columns are ``x, y, z`` in track coordinates; only ``y`` and ``z``
    are needed for the current cross-section contact geometry.
    """

    pose = pose or WheelPose2D()
    prepared = _prepared_wheel_trace_profile(
        wheel_profile,
        contact_angle_table,
        dlb=dlb,
        discrete_len_flange=discrete_len_flange,
        discrete_len_tread=discrete_len_tread,
    )
    x_profile = prepared.x_profile
    rolling_radius = prepared.rolling_radius
    contact_angles = prepared.contact_angles

    lx = -np.cos(pose.roll) * np.sin(pose.yaw)
    ly = np.cos(pose.roll) * np.cos(pose.yaw)
    lz = np.sin(pose.roll)
    tan_angle = np.tan(contact_angles)
    m_value = np.sqrt(np.maximum(0.0, 1.0 - lx**2 * (1.0 + tan_angle**2)))
    denominator = 1.0 - lx**2

    x_base = x_profile * lx
    y_base = x_profile * ly + pose.lateral
    # MATLAB TracePrinciple.m leaves Zw_DW out of the traced wheel profile;
    # vertical wheel motion is added later when forming Elastic_pen.
    z_base = x_profile * lz

    x_track = x_base + lx * rolling_radius * tan_angle
    y_track = y_base - rolling_radius * (lx**2 * ly * tan_angle + lz * m_value) / denominator
    z_track = z_base - rolling_radius * (lx**2 * lz * tan_angle - ly * m_value) / denominator
    return WheelTrace(
        wheel_lateral=x_profile,
        track_points=np.column_stack((x_track, y_track, z_track)),
        contact_angles=contact_angles,
    )


def single_point_contact_geometry(
    wheel_profile: np.ndarray,
    contact_angle_table: np.ndarray,
    rail_profile: np.ndarray,
    *,
    pose: WheelPose2D | None = None,
    min_overlap_margin: float = 0.0,
    dlb: float | None = None,
    discrete_len_flange: float = 0.5e-5,
    discrete_len_tread: float = 2.5e-5,
) -> SinglePointContact:
    """Compute the simplest single-point wheel/rail normal contact geometry.

    The contact candidate is the wheel-trace point with the largest vertical
    penetration against the interpolated rail profile. Positive penetration
    means the transformed wheel profile lies below/into the rail profile in the
    same vertical coordinate convention used by the MATLAB migration.
    """

    trace = trace_wheel_profile(
        wheel_profile,
        contact_angle_table,
        pose,
        dlb=dlb,
        discrete_len_flange=discrete_len_flange,
        discrete_len_tread=discrete_len_tread,
    )
    rail = _sort_points(rail_profile)
    if trace.track_points.size == 0 or rail.size == 0:
        return _empty_contact()

    y_wheel = trace.track_points[:, 1]
    lower = max(float(np.min(y_wheel)), float(np.min(rail[:, 0]))) + min_overlap_margin
    upper = min(float(np.max(y_wheel)), float(np.max(rail[:, 0]))) - min_overlap_margin
    if lower > upper:
        return _empty_contact()

    mask = (y_wheel >= lower) & (y_wheel <= upper)
    if not np.any(mask):
        return _empty_contact()

    wheel_points = trace.track_points[mask]
    wheel_lateral = trace.wheel_lateral[mask]
    contact_angles = trace.contact_angles[mask]
    vertical_offset = 0.0 if pose is None else pose.vertical
    rail_z = _matlab_rail_interp(rail, wheel_points[:, 1])
    shifted_wheel_z = wheel_points[:, 2] + vertical_offset
    vertical_gap = rail_z - shifted_wheel_z
    index = int(np.argmin(vertical_gap))

    gap = float(vertical_gap[index])
    vertical_penetration = max(0.0, -gap)
    contact_angle = float(contact_angles[index])
    roll = 0.0 if pose is None else pose.roll
    normal_penetration = vertical_penetration / max(np.cos(contact_angle + roll), np.finfo(float).eps)
    rail_point = np.array([wheel_points[index, 1], rail_z[index]], dtype=float)
    wheel_point = np.array([wheel_points[index, 1], shifted_wheel_z[index]], dtype=float)
    return SinglePointContact(
        has_contact=vertical_penetration > 0.0,
        rail_point=rail_point,
        wheel_point=wheel_point,
        wheel_profile_lateral=float(wheel_lateral[index]),
        vertical_gap=gap,
        vertical_penetration=vertical_penetration,
        normal_penetration=float(normal_penetration),
        contact_angle=contact_angle,
    )


def multi_point_contact_geometry(
    wheel_profile: np.ndarray,
    contact_angle_table: np.ndarray,
    rail_profile: np.ndarray,
    *,
    pose: WheelPose2D | None = None,
    penetration_offset: float = 0.0,
    min_overlap_margin: float = 0.0,
    correction_theta: float = 2e-5,
    dlb: float | None = None,
) -> MultiPointContactGeometry:
    """Find all single-rail contact candidates and apply quasi-elastic correction.

    This reproduces the ``Multi_Con_250812.m`` contact-point finding subset:
    trace wheel profile, interpolate rail height, build the elastic penetration
    curve, find positive penetration intervals, then move each peak contact
    point to the weighted quasi-elastic center of its interval.
    """

    prepared = prepare_contact_profile_geometry(
        wheel_profile,
        contact_angle_table,
        rail_profile,
        pose=pose,
        min_overlap_margin=min_overlap_margin,
        dlb=dlb,
    )
    wheel_interp = prepared.wheel_interp
    rail_interp = prepared.rail_interp
    angles = prepared.contact_angles
    wheel_lateral = prepared.wheel_profile_lateral
    if wheel_interp.size == 0:
        empty = np.empty((0, 2), dtype=float)
        boundaries = extreme_boundary(empty)
        return MultiPointContactGeometry(False, empty, wheel_interp, rail_interp, angles, wheel_lateral, boundaries, ())

    vertical_offset = 0.0 if pose is None else pose.vertical
    elastic_penetration = np.column_stack(
        (wheel_interp[:, 1], wheel_interp[:, 2] - rail_interp[:, 1] + vertical_offset + penetration_offset)
    )
    boundaries = extreme_boundary(elastic_penetration, opt="max")
    patches = quasi_elastic_correction(
        elastic_penetration,
        boundaries.positive_extrema,
        boundaries.starts,
        boundaries.ends,
        wheel_interp,
        rail_interp,
        angles,
        wheel_lateral,
        contact_angle_table=contact_angle_table,
        yaw=0.0 if pose is None else pose.yaw,
        lateral=0.0 if pose is None else pose.lateral,
        roll=0.0 if pose is None else pose.roll,
        theta=correction_theta,
    )
    return MultiPointContactGeometry(
        has_contact=bool(patches),
        elastic_penetration=elastic_penetration,
        wheel_interp=wheel_interp,
        rail_interp=rail_interp,
        contact_angles=angles,
        wheel_profile_lateral=wheel_lateral,
        boundaries=boundaries,
        patches=patches,
    )


def prepare_contact_profile_geometry(
    wheel_profile: np.ndarray,
    contact_angle_table: np.ndarray,
    rail_profile: np.ndarray,
    *,
    pose: WheelPose2D | None = None,
    min_overlap_margin: float = 0.0,
    dlb: float | None = None,
    prepared_rail: PreparedRailProfileInterpolator | None = None,
) -> PreparedContactProfileGeometry:
    """Build native profile interpolants without searching for contact patches."""

    trace = trace_wheel_profile(wheel_profile, contact_angle_table, pose, dlb=dlb)
    wheel_interp, rail_interp, angles, wheel_lateral = _overlap_interpolants(
        trace,
        rail_profile,
        min_overlap_margin=min_overlap_margin,
        prepared_rail=prepared_rail,
    )
    return PreparedContactProfileGeometry(
        wheel_interp=wheel_interp,
        rail_interp=rail_interp,
        contact_angles=angles,
        wheel_profile_lateral=wheel_lateral,
    )


def prepare_rail_profile_interpolator(rail_profile: np.ndarray) -> PreparedRailProfileInterpolator:
    """Precompute the pose-independent rail sorting and cubic coefficients."""

    rail = _sort_points(rail_profile)
    unique_y, unique_index = np.unique(rail[:, 0], return_index=True)
    unique_z = rail[unique_index, 1]
    spline = CubicSpline(unique_y, unique_z) if unique_y.size >= 4 else None
    return PreparedRailProfileInterpolator(rail=rail, spline=spline)


def extreme_boundary(ver_dis: np.ndarray, opt: str = "max") -> BoundaryExtrema:
    """Find extrema and positive intervals following MATLAB ``Extreme_Boundary.m``.

    Returned indexes are zero-based Python sample indexes. Array columns are
    ``index, y, value, first_derivative``.
    """

    data = _sort_points(ver_dis)
    if data.shape[0] < 2:
        empty = np.empty((0, 4), dtype=float)
        return BoundaryExtrema(empty, empty, empty, empty)

    derivative = np.gradient(data[:, 1], data[:, 0])
    sign_change = ((derivative[:-1] < 0) & (derivative[1:] >= 0)) | (
        (derivative[:-1] >= 0) & (derivative[1:] < 0)
    )
    extrema_mask = (data[:-1, 1] >= 0) & sign_change
    extrema_indexes = np.flatnonzero(extrema_mask)
    extrema = _boundary_rows(data, derivative, extrema_indexes)

    start_indexes = np.flatnonzero((data[:-1, 1] < 0) & (data[1:, 1] >= 0)) + 1
    end_indexes = np.flatnonzero((data[:-1, 1] >= 0) & (data[1:, 1] < 0))

    if data[0, 1] >= 0:
        start_indexes = np.concatenate(([0], start_indexes))
    if data[-1, 1] >= 0:
        end_indexes = np.concatenate((end_indexes, [data.shape[0] - 1]))

    pair_count = min(start_indexes.size, end_indexes.size)
    start_indexes = start_indexes[:pair_count]
    end_indexes = end_indexes[:pair_count]
    starts = _boundary_rows(data, derivative, start_indexes)
    ends = _boundary_rows(data, derivative, end_indexes)

    if pair_count == 0:
        positive = np.empty((0, 4), dtype=float)
    elif opt == "min":
        min_mask = (data[:-1, 1] >= 0) & (derivative[:-1] < 0) & (derivative[1:] >= 0)
        positive = _boundary_rows(data, derivative, np.flatnonzero(min_mask))
    elif opt == "max":
        peaks = []
        for start, end in zip(start_indexes, end_indexes, strict=True):
            if end < start:
                continue
            local = start + int(np.argmax(data[start : end + 1, 1]))
            peaks.append(local)
        positive = _boundary_rows(data, derivative, np.array(peaks, dtype=int))
    else:
        raise ValueError("opt must be 'max' or 'min'")
    return BoundaryExtrema(extrema=extrema, positive_extrema=positive, starts=starts, ends=ends)


def quasi_elastic_correction(
    elastic_penetration: np.ndarray,
    positive_peaks: np.ndarray,
    positive_starts: np.ndarray,
    positive_ends: np.ndarray,
    wheel_interp: np.ndarray,
    rail_interp: np.ndarray,
    contact_angles: np.ndarray,
    wheel_profile_lateral: np.ndarray | None = None,
    *,
    contact_angle_table: np.ndarray | None = None,
    yaw: float = 0.0,
    lateral: float = 0.0,
    roll: float = 0.0,
    theta: float = 2e-5,
    assume_sorted: bool = False,
) -> tuple[ContactPatch, ...]:
    """Apply MATLAB ``Quasi_Elastic_Correction.m`` to candidate patches."""

    if positive_peaks.size == 0:
        return ()

    elastic = np.asarray(elastic_penetration, dtype=float) if assume_sorted else _sort_points(elastic_penetration)
    wheel_raw = np.asarray(wheel_interp, dtype=float)
    wheel_order = (
        np.arange(wheel_raw.shape[0], dtype=int)
        if assume_sorted and wheel_raw.size
        else np.argsort(wheel_raw[:, 1], kind="mergesort")
        if wheel_raw.size
        else np.array([], dtype=int)
    )
    wheel = wheel_raw[wheel_order] if wheel_raw.size else wheel_raw.reshape(0, 3)
    rail = np.asarray(rail_interp, dtype=float) if assume_sorted else _sort_points(rail_interp)
    angles_raw = np.asarray(contact_angles, dtype=float)
    angles = angles_raw[wheel_order] if wheel_order.size else angles_raw
    if wheel_profile_lateral is not None:
        wheel_lateral_raw = np.asarray(wheel_profile_lateral, dtype=float)
        wheel_lateral = wheel_lateral_raw[wheel_order] if wheel_order.size else wheel_lateral_raw
    else:
        wheel_lateral = wheel[:, 1]
    if contact_angle_table is not None:
        angle_profile = np.asarray(contact_angle_table, dtype=float) if assume_sorted else _sort_points(contact_angle_table)
    else:
        angle_profile = _sort_points(np.column_stack((wheel_lateral, angles)))
    wheel_to_track = _wheelset_orientation(roll, yaw)
    patches: list[ContactPatch] = []

    for peak_row, start_row, end_row in zip(positive_peaks, positive_starts, positive_ends, strict=True):
        peak_index = int(peak_row[0])
        start = int(start_row[0])
        end = int(end_row[0])
        if end < start:
            continue

        peak_penetration = float(peak_row[2])
        center_y = _weighted_patch_center(elastic, start, end, peak_penetration, theta)
        corrected_penetration = float(np.interp(center_y, elastic[:, 0], elastic[:, 1]))
        corrected_wheel = np.array(
            [
                np.interp(center_y, wheel[:, 1], wheel[:, 0]),
                center_y,
                np.interp(center_y, wheel[:, 1], wheel[:, 2]),
            ],
            dtype=float,
        )
        corrected_rail = np.array([center_y, np.interp(center_y, rail[:, 0], rail[:, 1])], dtype=float)
        corrected_local = _right_matrix_divide(
            corrected_wheel[np.newaxis, :] - np.array([0.0, lateral, 0.0], dtype=float),
            wheel_to_track,
        )[0]
        corrected_lateral = float(corrected_local[1])
        corrected_angle = float(np.interp(corrected_lateral, angle_profile[:, 0], angle_profile[:, 1]))
        normal_penetration = corrected_penetration / max(np.cos(corrected_angle + roll), np.finfo(float).eps)
        peak_angle = float(angles_raw[peak_index])
        peak_normal_penetration = peak_penetration / max(np.cos(peak_angle + roll), np.finfo(float).eps)

        peak_wheel = wheel[peak_index, :]
        peak_rail = rail[peak_index, :]
        patches.append(
            ContactPatch(
                peak_index=peak_index,
                start_index=start,
                end_index=end,
                peak_wheel_point=peak_wheel.copy(),
                peak_rail_point=peak_rail.copy(),
                corrected_wheel_point=corrected_wheel,
                corrected_rail_point=corrected_rail,
                wheel_profile_lateral=corrected_lateral,
                peak_vertical_penetration=peak_penetration,
                peak_normal_penetration=float(peak_normal_penetration),
                peak_contact_angle=peak_angle,
                corrected_vertical_penetration=corrected_penetration,
                corrected_normal_penetration=float(normal_penetration),
                contact_angle=corrected_angle,
            )
        )
    return tuple(patches)


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


def _wheelset_orientation(roll: float, yaw: float) -> np.ndarray:
    return np.array(
        [
            [np.cos(yaw), np.sin(yaw), 0.0],
            [-np.cos(roll) * np.sin(yaw), np.cos(roll) * np.cos(yaw), np.sin(roll)],
            [np.sin(roll) * np.sin(yaw), -np.sin(roll) * np.cos(yaw), np.cos(roll)],
        ],
        dtype=float,
    )


def _right_matrix_divide(values: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return np.linalg.solve(np.asarray(matrix, dtype=float).T, np.asarray(values, dtype=float).T).T


_WHEEL_TRACE_CACHE_MAX = 16
_wheel_trace_cache: dict[tuple[int, int, tuple[int, ...], tuple[int, ...], float | None, float, float], PreparedWheelTraceProfile] = {}


def _prepared_wheel_trace_profile(
    wheel_profile: np.ndarray,
    contact_angle_table: np.ndarray,
    *,
    dlb: float | None,
    discrete_len_flange: float,
    discrete_len_tread: float,
) -> PreparedWheelTraceProfile:
    profile_array = np.asarray(wheel_profile, dtype=float)
    angle_array = np.asarray(contact_angle_table, dtype=float)
    cache_key = (
        id(wheel_profile),
        id(contact_angle_table),
        profile_array.shape,
        angle_array.shape,
        None if dlb is None else float(dlb),
        float(discrete_len_flange),
        float(discrete_len_tread),
    )
    cached = _wheel_trace_cache.get(cache_key)
    if cached is not None:
        return cached

    profile = _sort_points(profile_array)
    angle_table = _sort_points(angle_array)
    if dlb is not None:
        profile = _densify_wheel_profile_for_trace(
            profile,
            dlb=float(dlb),
            discrete_len_flange=discrete_len_flange,
            discrete_len_tread=discrete_len_tread,
        )

    prepared = PreparedWheelTraceProfile(
        x_profile=profile[:, 0],
        rolling_radius=profile[:, 1],
        contact_angles=_spline_interp(angle_table[:, 0], angle_table[:, 1], profile[:, 0]),
    )
    if len(_wheel_trace_cache) >= _WHEEL_TRACE_CACHE_MAX:
        _wheel_trace_cache.pop(next(iter(_wheel_trace_cache)))
    _wheel_trace_cache[cache_key] = prepared
    return prepared


def _densify_wheel_profile_for_trace(
    profile: np.ndarray,
    *,
    dlb: float,
    discrete_len_flange: float,
    discrete_len_tread: float,
) -> np.ndarray:
    """Match MATLAB ``TracePrinciple.m`` wheel-profile resampling."""

    x = profile[:, 0]
    is_right = float(np.mean(x)) >= 0.0
    if is_right:
        flange = x <= dlb + 38e-3
    else:
        flange = x >= -(dlb + 38e-3)
    if not np.any(flange) or np.all(flange):
        dense_x = _matlab_colon(float(x[0]), discrete_len_tread, float(x[-1]))
    else:
        flange_x = _matlab_colon(float(np.min(x[flange])), discrete_len_flange, float(np.max(x[flange])))
        tread_x = _matlab_colon(float(np.min(x[~flange])), discrete_len_tread, float(np.max(x[~flange])))
        dense_x = np.sort(np.concatenate((flange_x, tread_x)))
    dense_x = np.unique(dense_x)
    radius = _spline_interp(profile[:, 0], profile[:, 1], dense_x)
    return np.column_stack((dense_x, radius))


def _matlab_colon(start: float, step: float, stop: float) -> np.ndarray:
    if step <= 0.0:
        raise ValueError("step must be positive")
    if start > stop:
        return np.empty((0,), dtype=float)
    count = int(np.floor((stop - start) / step + 1e-12)) + 1
    return start + step * np.arange(count, dtype=float)


def _spline_interp(x: np.ndarray, y: np.ndarray, x_new: np.ndarray) -> np.ndarray:
    order = np.argsort(x)
    x_sorted = np.asarray(x, dtype=float)[order]
    y_sorted = np.asarray(y, dtype=float)[order]
    unique_x, unique_index = np.unique(x_sorted, return_index=True)
    y_unique = y_sorted[unique_index]
    if unique_x.size < 4:
        return np.interp(x_new, unique_x, y_unique)
    return CubicSpline(unique_x, y_unique)(x_new)


def _overlap_interpolants(
    trace: WheelTrace,
    rail_profile: np.ndarray,
    *,
    min_overlap_margin: float,
    prepared_rail: PreparedRailProfileInterpolator | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rail = _sort_points(rail_profile) if prepared_rail is None else prepared_rail.rail
    if trace.track_points.size == 0 or rail.size == 0:
        return (
            np.empty((0, 3), dtype=float),
            np.empty((0, 2), dtype=float),
            np.empty((0,), dtype=float),
            np.empty((0,), dtype=float),
        )

    y_wheel = trace.track_points[:, 1]
    lower = max(float(np.min(y_wheel)), float(np.min(rail[:, 0]))) + min_overlap_margin
    upper = min(float(np.max(y_wheel)), float(np.max(rail[:, 0]))) - min_overlap_margin
    if lower > upper:
        return (
            np.empty((0, 3), dtype=float),
            np.empty((0, 2), dtype=float),
            np.empty((0,), dtype=float),
            np.empty((0,), dtype=float),
        )

    mask = (y_wheel >= lower) & (y_wheel <= upper)
    if not np.any(mask):
        return (
            np.empty((0, 3), dtype=float),
            np.empty((0, 2), dtype=float),
            np.empty((0,), dtype=float),
            np.empty((0,), dtype=float),
        )

    sort_index = np.argsort(trace.track_points[mask, 1])
    wheel = trace.track_points[mask][sort_index]
    angles = trace.contact_angles[mask][sort_index]
    wheel_lateral = trace.wheel_lateral[mask][sort_index]
    rail_z = _matlab_rail_interp(
        rail,
        wheel[:, 1],
        spline=None if prepared_rail is None else prepared_rail.spline,
    )
    rail_interp = np.column_stack((wheel[:, 1], rail_z))
    return wheel, rail_interp, angles, wheel_lateral


def _matlab_rail_interp(
    rail: np.ndarray,
    y: np.ndarray,
    *,
    spline: CubicSpline | None = None,
) -> np.ndarray:
    spline_z = _spline_interp(rail[:, 0], rail[:, 1], y) if spline is None else spline(y)
    linear_z = np.interp(y, rail[:, 0], rail[:, 1])
    return np.where(spline_z > 0.6 + 8e-3, linear_z, spline_z)


def _boundary_rows(data: np.ndarray, derivative: np.ndarray, indexes: np.ndarray) -> np.ndarray:
    if indexes.size == 0:
        return np.empty((0, 4), dtype=float)
    return np.column_stack((indexes, data[indexes, 0], data[indexes, 1], derivative[indexes]))


def _weighted_patch_center(
    elastic: np.ndarray,
    start: int,
    end: int,
    peak_penetration: float,
    theta: float,
) -> float:
    y = elastic[start : end + 1, 0]
    penetration = elastic[start : end + 1, 1]
    if y.size == 1:
        return float(y[0])
    weights = np.exp((penetration - peak_penetration) / theta)
    if end + 1 < elastic.shape[0]:
        widths = np.diff(elastic[start : end + 2, 0])
    else:
        widths = np.gradient(y)
    weighted_widths = weights * widths
    denominator = float(np.sum(weighted_widths))
    if denominator == 0.0:
        return float(y[int(np.argmax(penetration))])
    return float(np.sum(y * weighted_widths) / denominator)


def _empty_contact() -> SinglePointContact:
    point = np.array([np.nan, np.nan], dtype=float)
    return SinglePointContact(
        has_contact=False,
        rail_point=point,
        wheel_point=point,
        wheel_profile_lateral=np.nan,
        vertical_gap=np.inf,
        vertical_penetration=0.0,
        normal_penetration=0.0,
        contact_angle=np.nan,
    )
