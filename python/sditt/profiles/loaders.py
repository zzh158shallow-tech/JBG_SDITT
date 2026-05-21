from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from sditt.io.text import load_numeric_text


@dataclass(frozen=True)
class ProfileData:
    """Raw wheel/rail profile points from a text or SIMPACK ``.prr`` file."""

    path: Path
    points: np.ndarray
    metadata: dict[str, str]


def discover_profile_files(root: str | Path) -> list[Path]:
    """Find profile source files below ``root``."""

    root_path = Path(root)
    return sorted(
        path
        for path in root_path.rglob("*")
        if path.is_file() and _is_profile_candidate(path)
    )


def load_profile_directory(root: str | Path) -> dict[Path, ProfileData]:
    """Load all profile-like files under ``root`` keyed by relative path."""

    root_path = Path(root)
    profiles: dict[Path, ProfileData] = {}
    for path in discover_profile_files(root_path):
        profiles[path.relative_to(root_path)] = load_profile_file(path)
    return profiles


def load_profile_file(path: str | Path) -> ProfileData:
    file_path = Path(path)
    if file_path.suffix.lower() in {".prr", ".prw"}:
        return _load_prr_profile(file_path)
    return _load_txt_profile(file_path)


def _load_txt_profile(path: Path) -> ProfileData:
    data = load_numeric_text(path)
    return ProfileData(path=path, points=data.data, metadata={"format": "txt"})


def _is_profile_candidate(path: Path) -> bool:
    if path.suffix.lower() not in {".txt", ".prr", ".prw"}:
        return False
    return not any("mileage" in part.lower() for part in path.parts)


def _load_prr_profile(path: Path) -> ProfileData:
    metadata: dict[str, str] = {"format": path.suffix.lower().lstrip(".")}
    numeric_lines: list[str] = []

    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        content = line.split("!", 1)[0].strip()
        if "=" in content:
            key, value = content.split("=", 1)
            metadata[key.strip()] = value.strip().strip("'")
            continue
        if re.match(r"^[+\-0-9.eE,\s]+$", content):
            numeric_lines.append(content)

    rows: list[list[float]] = []
    for line in numeric_lines:
        parts = line.replace(",", " ").split()
        if len(parts) >= 2:
            rows.append([float(part) for part in parts])

    points = np.array(rows, dtype=float) if rows else np.empty((0, 0), dtype=float)
    return ProfileData(path=path, points=points, metadata=metadata)
