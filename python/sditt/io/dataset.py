from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from sditt.config import ProjectPaths
from sditt.io.matlab import MatSummary
from sditt.io.text import NumericTextData, load_numeric_text as load_numeric_text_file
from sditt.profiles import ProfileData, discover_profile_files, load_profile_file
from sditt.track import load_modal_turnout_frequencies, summarize_modal_turnout_data
from sditt.vehicle import VehicleParameters, load_vehicle_parameters


@dataclass(frozen=True)
class RawInputManifest:
    """Lightweight index of SDITT raw input files."""

    paths: ProjectPaths
    modal_turnout_summary: MatSummary
    modal_frequencies_shape: tuple[int, ...]
    vehicle_parameter_count: int
    profile_files: tuple[Path, ...]
    wheel_profile_files: tuple[Path, ...]
    rail_profile_files: tuple[Path, ...]
    numeric_text_files: tuple[Path, ...]


@dataclass(frozen=True)
class SDITTRawInputs:
    """Raw SDITT inputs loaded from MATLAB, text, and profile files.

    This is a data-read layer only. It intentionally performs no contact,
    vehicle, track, integration, or dynamics calculations.
    """

    paths: ProjectPaths
    modal_turnout_summary: MatSummary
    modal_frequencies: np.ndarray
    vehicle_parameters: VehicleParameters
    profile_files: tuple[Path, ...]
    wheel_profile_files: tuple[Path, ...]
    rail_profile_files: tuple[Path, ...]
    wheel_profiles: dict[Path, ProfileData]
    rail_profiles: dict[Path, ProfileData]
    numeric_text_tables: dict[Path, NumericTextData]


def inspect_raw_inputs(root: str | Path | None = None) -> RawInputManifest:
    """Return a lightweight manifest for the default SDITT raw inputs."""

    paths = ProjectPaths.from_repo_root(root)
    modal_summary = summarize_modal_turnout_data(paths.modal_turnout_mat)
    modal_frequencies = load_modal_turnout_frequencies(paths.modal_turnout_mat)
    vehicle = load_vehicle_parameters(paths.default_vehicle_parameters)
    profile_files = tuple(discover_profile_files(paths.profile_dir))
    wheel_files, rail_files = _split_profile_files(profile_files, paths)

    return RawInputManifest(
        paths=paths,
        modal_turnout_summary=modal_summary,
        modal_frequencies_shape=tuple(int(part) for part in modal_frequencies.shape),
        vehicle_parameter_count=len(vehicle.values),
        profile_files=profile_files,
        wheel_profile_files=wheel_files,
        rail_profile_files=rail_files,
        numeric_text_files=tuple(_discover_numeric_text_files(paths)),
    )


def load_sditt_raw_inputs(
    root: str | Path | None = None,
    *,
    load_profiles: bool = True,
    max_profiles_per_kind: int | None = None,
    load_text_tables: bool = True,
) -> SDITTRawInputs:
    """Load the default raw data needed by the Python migration scaffold.

    Set ``max_profiles_per_kind`` during smoke tests to avoid reading thousands
    of profile files. Leave it as ``None`` to load all discovered wheel and rail
    profile files.
    """

    paths = ProjectPaths.from_repo_root(root)
    modal_summary = summarize_modal_turnout_data(paths.modal_turnout_mat)
    modal_frequencies = load_modal_turnout_frequencies(paths.modal_turnout_mat)
    vehicle = load_vehicle_parameters(paths.default_vehicle_parameters)
    profile_files = tuple(discover_profile_files(paths.profile_dir))
    wheel_files, rail_files = _split_profile_files(profile_files, paths)

    wheel_profiles: dict[Path, ProfileData] = {}
    rail_profiles: dict[Path, ProfileData] = {}
    if load_profiles:
        wheel_profiles = _load_profiles(wheel_files, paths.profile_dir, max_profiles_per_kind)
        rail_profiles = _load_profiles(rail_files, paths.profile_dir, max_profiles_per_kind)

    numeric_text_tables: dict[Path, NumericTextData] = {}
    if load_text_tables:
        for path in _discover_numeric_text_files(paths):
            numeric_text_tables[path.relative_to(paths.matlab_dir)] = load_numeric_text_file(path)

    return SDITTRawInputs(
        paths=paths,
        modal_turnout_summary=modal_summary,
        modal_frequencies=modal_frequencies,
        vehicle_parameters=vehicle,
        profile_files=profile_files,
        wheel_profile_files=wheel_files,
        rail_profile_files=rail_files,
        wheel_profiles=wheel_profiles,
        rail_profiles=rail_profiles,
        numeric_text_tables=numeric_text_tables,
    )


def _split_profile_files(
    profile_files: tuple[Path, ...],
    paths: ProjectPaths,
) -> tuple[tuple[Path, ...], tuple[Path, ...]]:
    wheel_root = paths.wheel_profile_dir.resolve()
    wheel_files: list[Path] = []
    rail_files: list[Path] = []
    for path in profile_files:
        resolved = path.resolve()
        if resolved == wheel_root or wheel_root in resolved.parents:
            wheel_files.append(path)
        else:
            rail_files.append(path)
    return tuple(wheel_files), tuple(rail_files)


def _load_profiles(
    files: tuple[Path, ...],
    root: Path,
    limit: int | None,
) -> dict[Path, ProfileData]:
    selected = files if limit is None else files[:limit]
    return {path.relative_to(root): load_profile_file(path) for path in selected}


def _discover_numeric_text_files(paths: ProjectPaths) -> list[Path]:
    candidates = [
        paths.matlab_dir / "Mileage_prr_2.txt",
        *sorted(paths.matlab_dir.glob("DR_S8b_*.txt")),
        *sorted(paths.turnout_mileage_dir.glob("*.txt")),
    ]
    return [path for path in candidates if path.exists()]
