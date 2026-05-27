from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from math import comb
from pathlib import Path
from typing import Callable, Iterable, Mapping

import numpy as np
from scipy.interpolate import make_smoothing_spline

from sditt.profiles.loaders import load_profile_file


WHEEL_STATIONS = ("FF", "FR", "RF", "RR")
_RADIUS_CACHE_MAX = 2048
_radius_profile_cache: dict[tuple[tuple[int, ...], str, float, bytes], np.ndarray] = {}


@dataclass(frozen=True)
class WheelProfileSet:
    """Wheel profiles in the profile coordinate system."""

    right: np.ndarray
    left: np.ndarray
    contact_angle_right: np.ndarray
    contact_angle_left: np.ndarray
    radius_right: np.ndarray
    radius_left: np.ndarray


@dataclass(frozen=True)
class ContactTables:
    """Lookup tables assembled by MATLAB ``Par_Con.m``."""

    bgmn: np.ndarray
    bgc1: np.ndarray
    bgc2: np.ndarray


@dataclass(frozen=True)
class MileageProfileEntry:
    """One measured rail profile indexed by longitudinal mileage."""

    mileage: float
    profile_file: Path


@dataclass(frozen=True)
class BezierProfileData:
    """Bezier interpolation cache matching ``Create_BezierIntData.m``."""

    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    divisions: np.ndarray
    x_to_t: tuple[np.ndarray, ...]


@dataclass(frozen=True)
class RailProfileRecord:
    """Interpolated profile information for one wheel station."""

    profile_num: int | None
    profile: np.ndarray
    front_profile: np.ndarray
    rear_profile: np.ndarray
    front_extreme: np.ndarray
    rear_extreme: np.ndarray
    radius: np.ndarray


@dataclass(frozen=True)
class RailProfileSet:
    """Profile-coordinate rail sections keyed by wheel station."""

    by_station: dict[str, RailProfileRecord]


@dataclass(frozen=True)
class OffsetProfileRecord:
    """One dummy rail after conversion to the track coordinate system."""

    profile: np.ndarray
    radius: np.ndarray
    front_profile: np.ndarray
    rear_profile: np.ndarray
    front_extreme: np.ndarray
    rear_extreme: np.ndarray
    d_y: float
    d_z: float


@dataclass(frozen=True)
class TrackProfileSet:
    """Track-coordinate profiles keyed by dummy rail plus merged left/right rails."""

    profile: dict[str, np.ndarray] = field(default_factory=dict)
    radius: dict[str, np.ndarray] = field(default_factory=dict)
    front_profile: dict[str, np.ndarray] = field(default_factory=dict)
    rear_profile: dict[str, np.ndarray] = field(default_factory=dict)
    front_extreme: dict[str, np.ndarray] = field(default_factory=dict)
    rear_extreme: dict[str, np.ndarray] = field(default_factory=dict)
    offsets: dict[str, tuple[float, float]] = field(default_factory=dict)


def contact_tables() -> ContactTables:
    """Return Hertz/contact lookup tables from ``Par_Con.m``."""

    bgmn = np.array(
        [
            [0, 1e10, 0],
            [1, 36.89, 0.131],
            [10, 6.612, 0.319],
            [12.84, 5.22, 0.352],
            [14.07, 5, 0.361],
            [15.2, 4.79, 0.369],
            [16.26, 4.57, 0.378],
            [17.25, 4.36, 0.386],
            [18.2, 4.14, 0.395],
            [18.88, 4.01, 0.401],
            [19.95, 3.8, 0.412],
            [21.57, 3.61, 0.425],
            [23.07, 3.41, 0.438],
            [24.5, 3.24, 0.45],
            [25.84, 3.06, 0.462],
            [27.44, 2.94, 0.473],
            [28.96, 2.82, 0.485],
            [30.4, 2.7, 0.496],
            [31.79, 2.57, 0.509],
            [34.41, 2.43, 0.526],
            [36.87, 2.3, 0.545],
            [39.2, 2.18, 0.561],
            [41.41, 2.07, 0.577],
            [43.53, 1.99, 0.593],
            [45.57, 1.91, 0.608],
            [47.55, 1.84, 0.622],
            [49.46, 1.77, 0.637],
            [51.23, 1.72, 0.651],
            [51.32, 1.72, 0.651],
            [53.13, 1.66, 0.664],
            [54.9, 1.61, 0.677],
            [56.63, 1.57, 0.691],
            [58.33, 1.53, 0.704],
            [60, 1.49, 0.717],
            [63.26, 1.42, 0.744],
            [66.42, 1.35, 0.769],
            [69.51, 1.29, 0.797],
            [72.54, 1.24, 0.824],
            [75.52, 1.2, 0.851],
            [78.46, 1.15, 0.878],
            [81.37, 1.11, 0.907],
            [84.26, 1.07, 0.936],
            [87.13, 1.04, 0.968],
            [90, 1, 1],
        ],
        dtype=float,
    )
    bgmn[:, 0] *= np.pi / 180.0
    bgc1 = np.array(
        [
            [0.1, 3.31, 2.52, 0.473, 8.28],
            [0.2, 3.37, 2.63, 0.603, 4.27],
            [0.3, 3.44, 2.75, 0.715, 2.96],
            [0.4, 3.53, 2.88, 0.823, 2.32],
            [0.5, 3.62, 3.01, 0.929, 1.93],
            [0.6, 3.72, 3.14, 1.030, 1.68],
            [0.7, 3.81, 3.28, 1.140, 1.50],
            [0.8, 3.91, 3.41, 1.250, 1.37],
            [0.9, 4.01, 3.54, 1.360, 1.27],
            [1.0, 4.12, 3.67, 1.470, 1.19],
        ],
        dtype=float,
    )
    bgc2 = np.flipud(
        np.array(
            [
                [1.0, 4.12, 3.67, 1.47, 1.190],
                [0.9, 4.22, 3.81, 1.59, 1.110],
                [0.8, 4.36, 3.99, 1.75, 1.040],
                [0.7, 4.54, 4.21, 1.95, 0.965],
                [0.6, 4.78, 4.50, 2.23, 0.892],
                [0.5, 5.10, 4.90, 2.62, 0.819],
                [0.4, 5.57, 5.48, 3.24, 0.747],
                [0.3, 6.34, 6.40, 4.32, 0.674],
                [0.2, 7.78, 8.14, 6.63, 0.601],
                [0.1, 11.7, 12.8, 14.6, 0.526],
            ],
            dtype=float,
        )
    )
    return ContactTables(bgmn=bgmn, bgc1=bgc1, bgc2=bgc2)


def build_wheel_profiles(
    wheel_dir: str | Path,
    *,
    vehicle_type: str = "CRH380A",
    drc: float = 0.0,
    dlb: float = 0.0,
    r0: float = 0.430,
) -> WheelProfileSet:
    """Read wheel profile text data and derive mirrored side, angle, and radius arrays."""

    wheel_path = Path(wheel_dir)
    if vehicle_type == "CR400BF":
        raw = load_profile_file(wheel_path / "LMB10.txt").points[:, :2]
        right = np.column_stack((raw[:, 0] + drc + dlb, raw[:, 1] + r0))
    else:
        raw = load_profile_file(wheel_path / "LMA_UnitMM.txt").points[:, :2]
        right = np.column_stack((raw[:, 0] / 1000.0 + drc + dlb, raw[:, 1] / 1000.0 + r0))

    right = sort_points(right)
    left = sort_points(np.column_stack((-right[:, 0], right[:, 1])))
    contact_angle_right = contact_angles(right)
    contact_angle_left = sort_points(np.column_stack((-contact_angle_right[:, 0], -contact_angle_right[:, 1])))
    radius_right = wheel_curvature_radius(right, vehicle_type=vehicle_type)
    radius_left = sort_points(np.column_stack((-radius_right[:, 0], radius_right[:, 1])))
    return WheelProfileSet(
        right=right,
        left=left,
        contact_angle_right=contact_angle_right,
        contact_angle_left=contact_angle_left,
        radius_right=radius_right,
        radius_left=radius_left,
    )


def contact_angles(profile: np.ndarray) -> np.ndarray:
    """Replicate the finite-difference contact-angle table from ``Radius_wheel.m``."""

    points = sort_points(profile)
    slopes = np.diff(points[:, 1]) / np.diff(points[:, 0])
    angles = np.arctan(slopes)
    step = points[-1, 0] - points[-2, 0]
    x = np.concatenate(([points[0, 0]], points[1:, 0], [points[-1, 0] + step]))
    y = np.concatenate(([angles[0]], angles, [angles[-1]]))
    return np.column_stack((x, y))


def curvature_radius(profile: np.ndarray) -> np.ndarray:
    """Estimate signed profile curvature radius with NumPy gradients."""

    points = sort_points(profile)
    x = points[:, 0]
    z = points[:, 1]
    with np.errstate(divide="ignore", invalid="ignore"):
        dz_dx = np.gradient(z, x)
        d2z_dx2 = np.gradient(dz_dx, x)
        radius = ((1.0 + dz_dx**2) ** 1.5) / d2z_dx2
    return np.column_stack((x, radius))


def wheel_curvature_radius(profile: np.ndarray, *, vehicle_type: str = "CRH380A") -> np.ndarray:
    """Wheel rolling-radius table with MATLAB's side-specific masking conventions."""

    smoothing = 1.0 - (5.0e-10 if vehicle_type == "CR400BF" else 5.0e-12)
    radius = radius_profile_v3(
        profile,
        smoothing=smoothing,
        smooth_profile=True,
        smooth_radius=True,
    )
    radius[:, 1] *= -1.0
    if vehicle_type == "CR400BF":
        x_new = np.arange(np.nanmin(radius[:, 0]), np.nanmax(radius[:, 0]) + 0.0001 / 2, 0.0001)
        y_new = np.interp(x_new, radius[:, 0], radius[:, 1])
        radius = np.column_stack((x_new, y_new))
        mask = (radius[:, 0] > 0.76) | (np.abs(radius[:, 1]) > 0.3)
        radius[mask, 1] = -np.inf
    else:
        radius[radius[:, 0] > 0.7512, 1] = -np.inf
    return sort_points(radius)


def rail_curvature_radius(profile: np.ndarray) -> np.ndarray:
    """Rail radius table used by profile interpolation, returned as absolute radius."""

    radius = radius_profile_v3(profile, smoothing=1.0 - 5.0e-8, smooth_profile=True, smooth_radius=True)
    radius[:, 1] = np.abs(radius[:, 1])
    return radius


def radius_profile_v3(
    profile: np.ndarray,
    *,
    smoothing: float,
    smooth_profile: bool,
    smooth_radius: bool,
) -> np.ndarray:
    """Approximate MATLAB ``Radius_profile_v3(profile, p, Choose_1, Choose_2)``."""

    points = sort_points(profile)
    if points.shape[0] < 3:
        return curvature_radius(points)

    x = points[:, 0]
    z = points[:, 1]
    if smooth_profile:
        z0 = _csaps_values(x, z, smoothing)
        dz_dx = np.gradient(z0, x)
        dz_dx = _csaps_values(x, dz_dx, smoothing)
        d2z_dx2 = np.gradient(dz_dx, x)
        d2z_dx2 = _csaps_values(x, d2z_dx2, smoothing)
        with np.errstate(divide="ignore", invalid="ignore"):
            curvature = d2z_dx2 / ((1.0 + dz_dx**2) ** 1.5)
            if smooth_radius:
                curvature = _csaps_values(x, curvature, smoothing)
            radius = 1.0 / curvature
        return np.column_stack((x, radius))

    radius = curvature_radius(points)
    if smooth_radius:
        radius[:, 1] = _csaps_values(radius[:, 0], radius[:, 1], smoothing)
    return radius


def _csaps_values(x: np.ndarray, y: np.ndarray, smoothing: float) -> np.ndarray:
    finite = np.isfinite(x) & np.isfinite(y)
    if np.count_nonzero(finite) < 3:
        return np.asarray(y, dtype=float).copy()
    p = float(np.clip(smoothing, np.finfo(float).eps, 1.0))
    lam = (1.0 - p) / p
    spline = make_smoothing_spline(x[finite], y[finite], lam=lam)
    out = np.asarray(y, dtype=float).copy()
    out[finite] = spline(x[finite])
    return out


def extreme_points(profile: np.ndarray, *, kind: str | None = None) -> np.ndarray:
    """Find derivative sign-change points like MATLAB ``Extreme_point.m``."""

    if profile.size == 0:
        return np.empty((0, 4), dtype=float)
    points = sort_points(profile)
    slope = np.gradient(points[:, 1], points[:, 0])
    if kind == "min":
        mask = (slope[:-1] < 0) & (slope[1:] >= 0)
    elif kind == "max":
        mask = (slope[:-1] >= 0) & (slope[1:] < 0)
    else:
        mask = ((slope[:-1] < 0) & (slope[1:] >= 0)) | ((slope[:-1] >= 0) & (slope[1:] < 0))
    indexes = np.flatnonzero(mask)
    if indexes.size == 0:
        return np.empty((0, 4), dtype=float)
    return np.column_stack((indexes + 1, points[indexes, 0], points[indexes, 1], slope[indexes]))


def read_mileage_profile_file(
    path: str | Path,
    *,
    profile_base_dir: str | Path | None = None,
    skip_first: int = 0,
    skip_last: int = 0,
) -> tuple[MileageProfileEntry, ...]:
    """Read MATLAB mileage/profile lists such as ``07(009)-Mileage-zjg_zgyg.txt``."""

    file_path = Path(path)
    base_dir = Path(profile_base_dir) if profile_base_dir is not None else file_path.parent
    entries: list[MileageProfileEntry] = []
    for raw_line in file_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith(("#", "%", "!")):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        try:
            mileage = float(parts[0])
        except ValueError:
            continue
        entries.append(MileageProfileEntry(mileage=mileage, profile_file=base_dir / parts[1]))
    if skip_last:
        return tuple(entries[skip_first:-skip_last])
    return tuple(entries[skip_first:])


def create_bezier_profile_data(
    mileage_entries: Iterable[MileageProfileEntry],
    divisions: np.ndarray,
    *,
    num_interp: int = 1000,
) -> BezierProfileData:
    """Build the longitudinal Bezier interpolation cache used by ``Create_prrFile.m``."""

    entries = tuple(mileage_entries)
    divisions_out = np.asarray(divisions, dtype=float).copy()
    if divisions_out.ndim == 1:
        divisions_out = divisions_out.reshape(1, -1)
    if divisions_out.shape[1] == 2:
        divisions_out = np.column_stack((divisions_out, np.zeros(divisions_out.shape[0])))

    all_x: list[float] = []
    all_y: list[np.ndarray] = []
    all_z: list[np.ndarray] = []
    x_to_t: list[np.ndarray] = []
    sample_t = np.linspace(0.0, 1.0, 10001)

    for div_index, div in enumerate(divisions_out):
        segment = [entry for entry in entries if div[0] <= entry.mileage <= div[1]]
        if len(segment) >= 2 and segment[-2].mileage == segment[-1].mileage:
            segment = segment[:-1]
        elif len(segment) >= 2 and segment[0].mileage == segment[1].mileage:
            segment = segment[1:]
        divisions_out[div_index, 2] = len(segment)

        for entry in segment:
            profile = load_profile_file(entry.profile_file).points[:, :2]
            profile = profile[profile[:, 1] <= 23e-3]
            y = np.linspace(profile[0, 0], profile[-1, 0], num_interp)
            z = np.interp(y, profile[:, 0], profile[:, 1])
            all_x.append(entry.mileage)
            all_y.append(y)
            all_z.append(z)

        x_values = np.array([entry.mileage for entry in segment], dtype=float)
        if len(x_values) == 0:
            x_to_t.append(np.empty((0, 2), dtype=float))
        elif len(x_values) == 1:
            x_to_t.append(np.array([[x_values[0], 0.0]], dtype=float))
        else:
            x_curve = _bezier_values(x_values, sample_t)
            x_to_t.append(np.column_stack((x_curve, sample_t)))
    if all_y:
        y_data = np.column_stack(all_y)
        z_data = np.column_stack(all_z)
    else:
        y_data = np.empty((num_interp, 0), dtype=float)
        z_data = np.empty((num_interp, 0), dtype=float)
    return BezierProfileData(x=np.array(all_x), y=y_data, z=z_data, divisions=divisions_out, x_to_t=tuple(x_to_t))


def interpolate_rail_profiles(
    j1: float,
    distance_vehicle: Mapping[str, float] | Iterable[float],
    mileage_entries: Iterable[MileageProfileEntry],
    *,
    same_front_profile: bool,
    same_rear_profile: bool,
    bezier: BezierProfileData | None = None,
    num_interp: int = 1000,
    radius_smoothing: float | Callable[[float], float] = 1.0 - 5.0e-8,
) -> RailProfileSet:
    """Generate per-wheel rail profiles, mirroring MATLAB ``Create_prrFile.m`` behavior."""

    entries = tuple(mileage_entries)
    station_distances = _station_distances(distance_vehicle)
    records: dict[str, RailProfileRecord] = {}
    for station in WHEEL_STATIONS:
        mileage = j1 - station_distances[station]
        smoothing = radius_smoothing(mileage) if callable(radius_smoothing) else radius_smoothing
        records[station] = _interpolate_one_rail_profile(
            mileage,
            entries,
            same_front_profile=same_front_profile,
            same_rear_profile=same_rear_profile,
            bezier=bezier,
            num_interp=num_interp,
            radius_smoothing=float(smoothing),
        )
    return RailProfileSet(by_station=records)


def offset_profile_to_track(
    rail_profile: RailProfileRecord,
    *,
    wheel_side: str,
    ori_prr: float,
    dis_rail_y: float = 0.0,
    dis_rail_z: float = 0.0,
    irregularity_y: float = 0.0,
    irregularity_z: float = 0.0,
    gauge: float = 1.435,
    vertical_offset: float = 0.6,
) -> OffsetProfileRecord:
    """Convert one profile-coordinate rail section into the track coordinate system."""

    sign = -1.0 if wheel_side == "L" else 1.0
    d_y = dis_rail_y + irregularity_y
    d_z = vertical_offset + dis_rail_z + irregularity_z
    lateral_origin = gauge / 2.0 + ori_prr

    def offset(points: np.ndarray, *, include_z: bool = True) -> np.ndarray:
        if points.size == 0:
            return np.empty((0, points.shape[1] if points.ndim == 2 else 0), dtype=float)
        out = points.copy()
        out[:, 0] = sign * (out[:, 0] + lateral_origin) + d_y
        if include_z and out.shape[1] > 1:
            out[:, 1] = out[:, 1] + d_z
        return out

    radius = offset(rail_profile.radius, include_z=False)
    front_extreme = rail_profile.front_extreme.copy()
    rear_extreme = rail_profile.rear_extreme.copy()
    if front_extreme.size:
        front_extreme[:, 2] = sign * (front_extreme[:, 2] + lateral_origin) + d_y
    if rear_extreme.size:
        rear_extreme[:, 2] = sign * (rear_extreme[:, 2] + lateral_origin) + d_y
    return OffsetProfileRecord(
        profile=offset(rail_profile.profile),
        radius=radius,
        front_profile=offset(rail_profile.front_profile),
        rear_profile=offset(rail_profile.rear_profile),
        front_extreme=front_extreme,
        rear_extreme=rear_extreme,
        d_y=d_y,
        d_z=d_z,
    )


def build_track_profiles(
    dummy_profiles: Mapping[str, OffsetProfileRecord],
    *,
    left_dummy_rails: Iterable[str],
    right_dummy_rails: Iterable[str],
) -> TrackProfileSet:
    """Merge dummy rails into left/right track-coordinate profiles as in ``Get_Profile_P2_v2.m``."""

    profiles = {key: value.profile for key, value in dummy_profiles.items()}
    radii = {key: value.radius for key, value in dummy_profiles.items()}
    fronts = {key: value.front_profile for key, value in dummy_profiles.items()}
    rears = {key: value.rear_profile for key, value in dummy_profiles.items()}
    front_extreme = {key: value.front_extreme for key, value in dummy_profiles.items()}
    rear_extreme = {key: value.rear_extreme for key, value in dummy_profiles.items()}
    offsets = {key: (value.d_y, value.d_z) for key, value in dummy_profiles.items()}

    profiles["L"], radii["L"] = _merge_side(dummy_profiles, tuple(left_dummy_rails), is_left=True)
    profiles["R"], radii["R"] = _merge_side(dummy_profiles, tuple(right_dummy_rails), is_left=False)
    return TrackProfileSet(
        profile=profiles,
        radius=radii,
        front_profile=fronts,
        rear_profile=rears,
        front_extreme=front_extreme,
        rear_extreme=rear_extreme,
        offsets=offsets,
    )


def sort_points(points: np.ndarray) -> np.ndarray:
    data = np.asarray(points, dtype=float)
    if data.size == 0:
        return data.reshape(0, 2)
    return data[np.argsort(data[:, 0])]


def _interpolate_one_rail_profile(
    mileage: float,
    entries: tuple[MileageProfileEntry, ...],
    *,
    same_front_profile: bool,
    same_rear_profile: bool,
    bezier: BezierProfileData | None,
    num_interp: int,
    radius_smoothing: float,
) -> RailProfileRecord:
    first = entries[0]
    last = entries[-1]
    if mileage < first.mileage:
        if not same_front_profile:
            return _empty_rail_profile()
        profile = sort_points(load_profile_file(first.profile_file).points[:, :2])
        return _rail_record(1, profile, profile, profile, radius_smoothing=radius_smoothing)
    if mileage > last.mileage:
        if not same_rear_profile:
            return _empty_rail_profile()
        profile = sort_points(load_profile_file(last.profile_file).points[:, :2])
        return _rail_record(len(entries), profile, profile, profile, radius_smoothing=radius_smoothing)

    mileage_values = np.array([entry.mileage for entry in entries], dtype=float)
    profile_index = int(np.searchsorted(mileage_values, mileage, side="left"))
    if profile_index == 0:
        front_entry = entries[0]
        rear_entry = entries[0]
    else:
        front_entry = entries[profile_index - 1]
        rear_entry = entries[profile_index]
    front = sort_points(load_profile_file(front_entry.profile_file).points[:, :2])
    rear = sort_points(load_profile_file(rear_entry.profile_file).points[:, :2])
    profile = _bezier_profile_at(mileage, bezier, num_interp) if bezier is not None else None
    if profile is None:
        profile = _linear_profile_between(front, rear, mileage, front_entry.mileage, rear_entry.mileage, num_interp)
    return _rail_record(profile_index + 1, sort_points(profile), front, rear, radius_smoothing=radius_smoothing)


def _rail_record(
    profile_num: int | None,
    profile: np.ndarray,
    front: np.ndarray,
    rear: np.ndarray,
    *,
    radius_smoothing: float = 1.0 - 5.0e-8,
) -> RailProfileRecord:
    return RailProfileRecord(
        profile_num=profile_num,
        profile=profile,
        front_profile=front,
        rear_profile=rear,
        front_extreme=extreme_points(front),
        rear_extreme=extreme_points(rear),
        radius=_cached_absolute_radius_profile_v3(profile, radius_smoothing),
    )


def _cached_absolute_radius_profile_v3(profile: np.ndarray, radius_smoothing: float) -> np.ndarray:
    profile_array = np.asarray(profile, dtype=float)
    if profile_array.size == 0:
        return np.empty((0, 2), dtype=float)
    contiguous = np.ascontiguousarray(profile_array)
    digest = hashlib.blake2b(contiguous.view(np.uint8), digest_size=16).digest()
    cache_key = (contiguous.shape, contiguous.dtype.str, float(radius_smoothing), digest)
    cached = _radius_profile_cache.get(cache_key)
    if cached is not None:
        return cached.copy()
    radius = _absolute_radius_profile_v3(contiguous, radius_smoothing)
    if len(_radius_profile_cache) >= _RADIUS_CACHE_MAX:
        _radius_profile_cache.pop(next(iter(_radius_profile_cache)))
    _radius_profile_cache[cache_key] = radius.copy()
    return radius


def _absolute_radius_profile_v3(profile: np.ndarray, radius_smoothing: float) -> np.ndarray:
    radius = radius_profile_v3(
        profile,
        smoothing=radius_smoothing,
        smooth_profile=True,
        smooth_radius=True,
    )
    radius[:, 1] = np.abs(radius[:, 1])
    return radius


def _empty_rail_profile() -> RailProfileRecord:
    empty = np.empty((0, 2), dtype=float)
    empty_extreme = np.empty((0, 4), dtype=float)
    return RailProfileRecord(None, empty, empty, empty, empty_extreme, empty_extreme, empty)


def _linear_profile_between(
    front: np.ndarray,
    rear: np.ndarray,
    mileage: float,
    front_mileage: float,
    rear_mileage: float,
    num_interp: int,
) -> np.ndarray:
    x_min = max(float(np.min(front[:, 0])), float(np.min(rear[:, 0])))
    x_max = min(float(np.max(front[:, 0])), float(np.max(rear[:, 0])))
    x = np.linspace(x_min, x_max, num_interp)
    z_front = np.interp(x, front[:, 0], front[:, 1])
    z_rear = np.interp(x, rear[:, 0], rear[:, 1])
    ratio = 0.0 if rear_mileage == front_mileage else (mileage - front_mileage) / (rear_mileage - front_mileage)
    return np.column_stack((x, z_front + (z_rear - z_front) * ratio))


def _bezier_profile_at(mileage: float, bezier: BezierProfileData | None, num_interp: int) -> np.ndarray | None:
    if bezier is None:
        return None
    matches = np.flatnonzero((mileage >= bezier.divisions[:, 0]) & (mileage <= bezier.divisions[:, 1]))
    if matches.size == 0:
        return None
    div_index = int(matches[0])
    x_to_t = bezier.x_to_t[div_index]
    if x_to_t.size == 0:
        return None
    t = float(np.interp(mileage, x_to_t[:, 0], x_to_t[:, 1]))
    start = int(np.sum(bezier.divisions[:div_index, 2]))
    stop = int(np.sum(bezier.divisions[: div_index + 1, 2]))
    if stop <= start:
        return None
    y_columns = bezier.y[:num_interp, start:stop]
    z_columns = bezier.z[:num_interp, start:stop]
    weights = _bezier_weights(y_columns.shape[1], t)
    y = y_columns @ weights
    z = z_columns @ weights
    return np.column_stack((y, z))


def _bezier_values(values: np.ndarray, t: np.ndarray) -> np.ndarray:
    weights = np.column_stack([_bezier_weights(len(values), value) for value in t])
    return values @ weights


def _bezier_weights(n_points: int, t: float) -> np.ndarray:
    degree = n_points - 1
    indexes = np.arange(n_points)
    coefficients = np.array([comb(degree, int(index)) for index in indexes], dtype=float)
    return coefficients * (1.0 - t) ** (degree - indexes) * t**indexes


def _station_distances(distance_vehicle: Mapping[str, float] | Iterable[float]) -> dict[str, float]:
    if isinstance(distance_vehicle, Mapping):
        return {station: float(distance_vehicle[station]) for station in WHEEL_STATIONS}
    values = tuple(float(value) for value in distance_vehicle)
    if len(values) != len(WHEEL_STATIONS):
        raise ValueError(f"distance_vehicle must provide {len(WHEEL_STATIONS)} distances")
    return dict(zip(WHEEL_STATIONS, values, strict=True))


def _merge_side(
    dummy_profiles: Mapping[str, OffsetProfileRecord],
    dummy_rails: tuple[str, ...],
    *,
    is_left: bool,
) -> tuple[np.ndarray, np.ndarray]:
    nonempty = [name for name in dummy_rails if name in dummy_profiles and dummy_profiles[name].profile.size]
    if not nonempty:
        return np.empty((0, 2), dtype=float), np.empty((0, 2), dtype=float)
    if len(nonempty) == 1:
        item = dummy_profiles[nonempty[0]]
        return sort_points(item.profile), sort_points(item.radius)

    masks = {name: np.ones(dummy_profiles[name].profile.shape[0], dtype=bool) for name in nonempty}
    for outside, inside in zip(nonempty[:-1], nonempty[1:], strict=False):
        outside_profile = dummy_profiles[outside].profile
        inside_profile = dummy_profiles[inside].profile
        if is_left:
            masks[outside] &= outside_profile[:, 0] < min(np.max(outside_profile[:, 0]), np.min(inside_profile[:, 0]))
            masks[inside] &= inside_profile[:, 0] > max(np.max(outside_profile[:, 0]), np.min(inside_profile[:, 0]))
        else:
            masks[outside] &= outside_profile[:, 0] > max(np.min(outside_profile[:, 0]), np.max(inside_profile[:, 0]))
            masks[inside] &= inside_profile[:, 0] < min(np.min(outside_profile[:, 0]), np.max(inside_profile[:, 0]))

    merged_profiles = []
    merged_radii = []
    for name in nonempty:
        mask = masks[name]
        merged_profiles.append(dummy_profiles[name].profile[mask, :])
        if dummy_profiles[name].radius.shape[0] == mask.shape[0]:
            merged_radii.append(dummy_profiles[name].radius[mask, :])
        else:
            merged_radii.append(dummy_profiles[name].radius)
    return sort_points(np.vstack(merged_profiles)), sort_points(np.vstack(merged_radii))
