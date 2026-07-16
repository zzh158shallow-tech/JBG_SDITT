from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Iterable, Mapping, Protocol

import numpy as np

from sditt.config import DEFAULT_OPERATING_CASE, MATLAB_FULL_DEFAULT_CASE, DefaultOperatingCase, ProjectPaths
from sditt.vehicle import VehicleParameters

from .geometry import (
    BezierProfileData,
    MileageProfileEntry,
    RailProfileRecord,
    RailProfileSet,
    create_bezier_profile_data,
    extreme_points,
    interpolate_rail_profiles,
    rail_curvature_radius,
    sort_points,
)
from .loaders import load_profile_file


DummyRailProfiles = dict[str, RailProfileSet]


class RailProfileSelector(Protocol):
    """Common interface for mileage-varying turnout and constant interval profiles."""

    def select(self, j1: float, profile_num_interp: Iterable[str] | None = None) -> DummyRailProfiles: ...


@dataclass(frozen=True)
class ConstantBasicRailProfileSelector:
    """Return one measured standard basic-rail section on both track sides."""

    profile_file: Path
    rail_names: tuple[str, ...] = ("L1", "R1")

    @cached_property
    def record(self) -> RailProfileRecord:
        profile = sort_points(load_profile_file(self.profile_file).points[:, :2])
        return RailProfileRecord(
            profile_num=1,
            profile=profile,
            front_profile=profile,
            rear_profile=profile,
            front_extreme=extreme_points(profile),
            rear_extreme=extreme_points(profile),
            radius=rail_curvature_radius(profile),
        )

    @cached_property
    def profile_set(self) -> RailProfileSet:
        return RailProfileSet(by_station={station: self.record for station in ("FF", "FR", "RF", "RR")})

    def select(
        self,
        j1: float,
        profile_num_interp: Iterable[str] | None = None,
    ) -> DummyRailProfiles:
        """Return the same L1/R1 basic-rail profile regardless of mileage."""

        del j1
        requested = self.rail_names if profile_num_interp is None else tuple(profile_num_interp)
        unsupported = tuple(rail for rail in requested if rail not in self.rail_names)
        if unsupported:
            raise ValueError(f"unsupported interval dummy rail profiles: {unsupported}")
        return {rail: self.profile_set for rail in requested}


@dataclass(frozen=True)
class DefaultRailProfileSelector:
    """Default ``07(009)`` / Face rail profile selector in profile coordinates."""

    mileage_entries: Mapping[str, tuple[MileageProfileEntry, ...]]
    bezier_profiles: Mapping[str, BezierProfileData]
    distance_vehicle: Mapping[str, float]
    num_interp: int = 1000
    _select_cache: dict[tuple[float, tuple[str, ...]], DummyRailProfiles] = field(
        default_factory=dict,
        init=False,
        repr=False,
        compare=False,
    )

    def select(
        self,
        j1: float,
        profile_num_interp: Iterable[str] | None = None,
    ) -> DummyRailProfiles:
        """Construct MATLAB-style ``RailPro_ProCS`` records for one front mileage."""

        requested = ("L1", "R1", "R2", "R3") if profile_num_interp is None else tuple(profile_num_interp)
        cache_key = (float(j1), requested)
        cached = self._select_cache.get(cache_key)
        if cached is not None:
            return cached

        profiles: DummyRailProfiles = {}
        for rail in requested:
            if rail == "L1":
                profiles[rail] = self._select_l1()
            elif rail == "R1":
                profiles[rail] = interpolate_rail_profiles(
                    j1,
                    self.distance_vehicle,
                    self.mileage_entries["R1"],
                    same_front_profile=True,
                    same_rear_profile=False,
                    bezier=self.bezier_profiles["R1"],
                    num_interp=self.num_interp,
                    radius_smoothing=1.0 - 5.0e-8,
                )
            elif rail == "R2":
                profiles[rail] = interpolate_rail_profiles(
                    j1,
                    self.distance_vehicle,
                    self.mileage_entries["R2"],
                    same_front_profile=False,
                    same_rear_profile=False,
                    bezier=self.bezier_profiles["R2"],
                    num_interp=self.num_interp,
                    radius_smoothing=_r2_radius_smoothing,
                )
            elif rail == "R3":
                profiles[rail] = interpolate_rail_profiles(
                    j1,
                    self.distance_vehicle,
                    self.mileage_entries["R3"],
                    same_front_profile=False,
                    same_rear_profile=True,
                    bezier=self.bezier_profiles["R3"],
                    num_interp=self.num_interp,
                    radius_smoothing=1.0 - 5.0e-9,
                )
            else:
                raise ValueError(f"unsupported default dummy rail profile: {rail}")
        if len(self._select_cache) >= 32:
            self._select_cache.pop(next(iter(self._select_cache)))
        self._select_cache[cache_key] = profiles
        return profiles

    @cached_property
    def l1_record(self) -> RailProfileRecord:
        profile = sort_points(load_profile_file(self.mileage_entries["R1"][0].profile_file).points[:, :2])
        return RailProfileRecord(
            profile_num=1,
            profile=profile,
            front_profile=profile,
            rear_profile=profile,
            front_extreme=extreme_points(profile),
            rear_extreme=extreme_points(profile),
            radius=rail_curvature_radius(profile),
        )

    def _select_l1(self) -> RailProfileSet:
        return RailProfileSet(
            by_station={
                station: self.l1_record
                for station in ("FF", "FR", "RF", "RR")
            }
        )


def build_default_07009_face_profile_selector(
    *,
    repo_root: str | Path | None = None,
    vehicle_parameters: VehicleParameters | Mapping[str, object] | None = None,
    operating_case: DefaultOperatingCase = MATLAB_FULL_DEFAULT_CASE,
    num_interp: int = 1000,
) -> DefaultRailProfileSelector:
    """Build the default ``Get_Profile_P1_Through_210624`` profile selector."""

    if operating_case.choose_turnout != "07(009)" or operating_case.vehicle_direction != "Face":
        raise ValueError("only the default 07(009) / Face profile selector is implemented")

    paths = ProjectPaths.from_repo_root(repo_root)
    profile_root = paths.profile_dir
    mileage_root = paths.turnout_mileage_dir
    profile_index = _profile_file_index(profile_root)

    entries = {
        "R1": _read_default_mileage_entries(
            mileage_root / "07(009)-Mileage-qjbg.txt",
            profile_index=profile_index,
            skip_first=2,
            skip_last=1,
        ),
        "R2": _read_default_mileage_entries(
            mileage_root / "07(009)-Mileage-zjg_zgyg_250722.txt",
            profile_index=profile_index,
        ),
        "R3": _read_default_mileage_entries(
            mileage_root / "07(009)-Mileage-cxg.txt",
            profile_index=profile_index,
            skip_last=1,
        ),
    }
    bezier = {
        "R1": create_bezier_profile_data(
            entries["R1"],
            np.array([[entries["R1"][0].mileage, entries["R1"][-1].mileage]], dtype=float),
            num_interp=num_interp,
        ),
        "R2": create_bezier_profile_data(
            entries["R2"],
            np.array(
                [
                    [entries["R2"][0].mileage, entries["R2"][72].mileage],
                    [entries["R2"][72].mileage, entries["R2"][77].mileage],
                    [entries["R2"][78].mileage, entries["R2"][-1].mileage],
                ],
                dtype=float,
            ),
            num_interp=num_interp,
        ),
        "R3": create_bezier_profile_data(
            entries["R3"],
            np.array([[entries["R3"][0].mileage, entries["R3"][-1].mileage]], dtype=float),
            num_interp=num_interp,
        ),
    }
    return DefaultRailProfileSelector(
        mileage_entries=entries,
        bezier_profiles=bezier,
        distance_vehicle=_distance_vehicle(vehicle_parameters),
        num_interp=num_interp,
    )


def build_interval_basic_rail_profile_selector(
    *,
    repo_root: str | Path | None = None,
) -> ConstantBasicRailProfileSelector:
    """Build the mileage-invariant two-basic-rail interval selector."""

    paths = ProjectPaths.from_repo_root(repo_root)
    if not paths.standard_basic_rail_profile.exists():
        raise FileNotFoundError(f"standard basic rail profile was not found: {paths.standard_basic_rail_profile}")
    return ConstantBasicRailProfileSelector(profile_file=paths.standard_basic_rail_profile)


def build_default_rail_profile_selector(
    *,
    repo_root: str | Path | None = None,
    vehicle_parameters: VehicleParameters | Mapping[str, object] | None = None,
    operating_case: DefaultOperatingCase = DEFAULT_OPERATING_CASE,
    num_interp: int = 1000,
) -> RailProfileSelector:
    """Build the selector matching the requested interval/turnout contact layout."""

    if operating_case.rail_layout == "interval":
        return build_interval_basic_rail_profile_selector(repo_root=repo_root)
    return build_default_07009_face_profile_selector(
        repo_root=repo_root,
        vehicle_parameters=vehicle_parameters,
        operating_case=operating_case,
        num_interp=num_interp,
    )


def rail_profile_numbers(profiles: Mapping[str, RailProfileSet]) -> dict[str, dict[str, int | None]]:
    """Compact profile-number view of a selected ``RailPro_ProCS`` structure."""

    return {
        rail: {
            station: record.profile_num
            for station, record in rail_profiles.by_station.items()
        }
        for rail, rail_profiles in profiles.items()
    }


def _read_default_mileage_entries(
    path: Path,
    *,
    profile_index: Mapping[str, Path],
    skip_first: int = 0,
    skip_last: int = 0,
) -> tuple[MileageProfileEntry, ...]:
    entries: list[MileageProfileEntry] = []
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
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
        filename = parts[1]
        try:
            profile_file = profile_index[filename]
        except KeyError as exc:
            raise FileNotFoundError(f"profile file {filename!r} referenced by {path} was not found") from exc
        entries.append(MileageProfileEntry(mileage=mileage, profile_file=profile_file))
    if skip_last:
        entries = entries[skip_first:-skip_last]
    else:
        entries = entries[skip_first:]
    return tuple(entries)


def _profile_file_index(profile_root: Path) -> dict[str, Path]:
    index: dict[str, Path] = {}
    for path in profile_root.rglob("*.txt"):
        if "Mileage" in path.parts:
            continue
        index.setdefault(path.name, path)
    return index


def _distance_vehicle(
    vehicle_parameters: VehicleParameters | Mapping[str, object] | None,
) -> dict[str, float]:
    if vehicle_parameters is None:
        ll1 = 2.50 / 2.0
        ll2 = 17.50 / 2.0
    else:
        values = vehicle_parameters.values if isinstance(vehicle_parameters, VehicleParameters) else vehicle_parameters
        ll1 = float(values["Ll1"])
        ll2 = float(values["Ll2"])
    return {
        "FF": 0.0,
        "FR": 2.0 * ll1,
        "RF": 2.0 * ll2,
        "RR": 2.0 * (ll1 + ll2),
    }


def _r2_radius_smoothing(mileage: float) -> float:
    if 0.0 <= mileage <= 75.0:
        return 1.0 - 1.0e-11
    if 75.0 <= mileage <= 200.0:
        return 1.0 - 5.0e-10
    return 1.0 - 1.0e-11
