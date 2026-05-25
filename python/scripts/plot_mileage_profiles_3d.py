from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sditt.config import ProjectPaths
from sditt.profiles.loaders import load_profile_file


DEFAULT_MILEAGE_FILE = (
    "SDITT-RW-FT-250728/WRProfile-07(009)-1_18/07(009)-Mileage/"
    "07(009)-Mileage-zjg_zgyg_250722.txt"
)
DEFAULT_OUTPUT = "python/outputs/profile_3d/r2_zjg_zgyg_profiles_top.svg"


@dataclass(frozen=True)
class MileageProfile:
    index: int
    mileage: float
    name: str
    path: Path
    points: np.ndarray


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    repo_root = ProjectPaths.from_repo_root(args.repo_root).root
    mileage_file = _resolve_input_path(repo_root, args.mileage_file)
    profile_root = _resolve_input_path(repo_root, args.profile_root) if args.profile_root else mileage_file.parents[1]
    output = _resolve_output_path(repo_root, args.output)
    manifest_output = output.with_suffix(".manifest.json")

    profiles = read_mileage_profiles(
        mileage_file,
        profile_root=profile_root,
        points_per_profile=args.points_per_profile,
        z_max=args.z_max,
        mirror_z=args.mirror_z,
        limit=args.limit,
    )
    if not profiles:
        raise SystemExit(f"no profiles were loaded from {mileage_file}")

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_profile_svg(profiles, width=args.width, height=args.height), encoding="utf-8")
    manifest_output.write_text(
        json.dumps(
            {
                "mileage_file": str(mileage_file),
                "profile_root": str(profile_root),
                "output": str(output),
                "view": "top",
                "mirror_z": bool(args.mirror_z),
                "count": len(profiles),
                "profiles": [
                    {
                        "index": profile.index,
                        "mileage": profile.mileage,
                        "name": profile.name,
                        "path": str(profile.path),
                        "point_count": int(profile.points.shape[0]),
                    }
                    for profile in profiles
                ],
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"wrote {output}")
    print(f"wrote {manifest_output}")
    return 0


def read_mileage_profiles(
    mileage_file: Path,
    *,
    profile_root: Path,
    points_per_profile: int,
    z_max: float | None,
    mirror_z: bool,
    limit: int | None,
) -> list[MileageProfile]:
    profile_index = _profile_file_index(profile_root)
    rows = _read_mileage_rows(mileage_file)
    if limit is not None:
        rows = rows[:limit]

    profiles: list[MileageProfile] = []
    for row_index, (mileage, filename) in enumerate(rows, start=1):
        path = _resolve_profile_path(filename, mileage_file=mileage_file, profile_index=profile_index)
        raw = load_profile_file(path).points[:, :2]
        points = _prepare_profile_points(raw, points_per_profile=points_per_profile, z_max=z_max, mirror_z=mirror_z)
        if points.size == 0:
            continue
        profiles.append(
            MileageProfile(
                index=row_index,
                mileage=float(mileage),
                name=filename,
                path=path,
                points=points,
            )
        )
    return profiles


def render_profile_svg(profiles: list[MileageProfile], *, width: int, height: int) -> str:
    mileage_values = np.asarray([profile.mileage for profile in profiles], dtype=float)
    lateral_values = np.concatenate([profile.points[:, 0] for profile in profiles])
    vertical_values = np.concatenate([profile.points[:, 1] for profile in profiles])

    ranges = {
        "mileage": _range_with_padding(mileage_values, pad_ratio=0.02),
        "lateral": _range_with_padding(lateral_values, pad_ratio=0.08),
        "vertical": _range_with_padding(vertical_values, pad_ratio=0.08),
    }
    project = _Projector(width=width, height=height, ranges=ranges)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,Helvetica,sans-serif}.axis{stroke:#333;stroke-width:1.2}.grid{stroke:#ddd;stroke-width:.7}.profile{fill:none;stroke-width:1.05;stroke-linecap:round;stroke-linejoin:round}</style>',
        f'<text x="24" y="30" font-size="18" fill="#202020">Top View Rail Profiles by Mileage ({len(profiles)} sections)</text>',
    ]
    parts.extend(_axis_svg(project))

    for profile_index, profile in enumerate(profiles):
        color = _color_ramp(profile_index / max(len(profiles) - 1, 1))
        points = " ".join(
            f"{x:.2f},{y:.2f}"
            for x, y in (
                project.point(profile.mileage, float(lateral), float(vertical))
                for lateral, vertical in profile.points
            )
        )
        opacity = 0.34 if len(profiles) > 80 else 0.52
        parts.append(f'<polyline class="profile" points="{points}" stroke="{color}" opacity="{opacity:.2f}"/>')

    legend_x = width - 220
    legend_y = 56
    parts.extend(
        [
            f'<text x="{legend_x}" y="{legend_y}" font-size="12" fill="#404040">Mileage color scale</text>',
            _gradient_bar_svg(legend_x, legend_y + 12),
            f'<text x="{legend_x}" y="{legend_y + 48}" font-size="11" fill="#555">{mileage_values[0]:.3f} m</text>',
            f'<text x="{legend_x + 150}" y="{legend_y + 48}" font-size="11" fill="#555" text-anchor="end">{mileage_values[-1]:.3f} m</text>',
        ]
    )
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


class _Projector:
    def __init__(self, *, width: int, height: int, ranges: dict[str, tuple[float, float]]) -> None:
        self.width = width
        self.height = height
        self.ranges = ranges
        self.origin = np.array([110.0, height - 120.0], dtype=float)
        self.axis_mileage = np.array([width - 250.0, 0.0], dtype=float)
        self.axis_lateral = np.array([0.0, -(height - 255.0)], dtype=float)
        self.axis_vertical = np.array([38.0, -28.0], dtype=float)

    def point(self, mileage: float, lateral: float, vertical: float) -> tuple[float, float]:
        u = _normalize(mileage, self.ranges["mileage"])
        v = _normalize(lateral, self.ranges["lateral"])
        w = _normalize(vertical, self.ranges["vertical"])
        point = self.origin + u * self.axis_mileage + v * self.axis_lateral + w * self.axis_vertical
        return float(point[0]), float(point[1])


def _axis_svg(project: _Projector) -> list[str]:
    parts: list[str] = []
    origin = project.point(project.ranges["mileage"][0], project.ranges["lateral"][0], project.ranges["vertical"][0])
    x_end = project.point(project.ranges["mileage"][1], project.ranges["lateral"][0], project.ranges["vertical"][0])
    y_end = project.point(project.ranges["mileage"][0], project.ranges["lateral"][1], project.ranges["vertical"][0])
    z_end = project.point(project.ranges["mileage"][0], project.ranges["lateral"][0], project.ranges["vertical"][1])
    parts.extend(
        [
            _line(origin, x_end, "axis"),
            _line(origin, y_end, "axis"),
            _line(origin, z_end, "axis"),
            f'<text x="{x_end[0] + 8:.2f}" y="{x_end[1] + 4:.2f}" font-size="12" fill="#202020">Mileage (m)</text>',
            f'<text x="{y_end[0] - 8:.2f}" y="{y_end[1] - 6:.2f}" font-size="12" fill="#202020" text-anchor="end">Lateral Y (m)</text>',
            f'<text x="{z_end[0] + 6:.2f}" y="{z_end[1] - 6:.2f}" font-size="12" fill="#202020">Mirrored Z (m)</text>',
        ]
    )
    for tick in np.linspace(*project.ranges["mileage"], 6):
        p0 = project.point(float(tick), project.ranges["lateral"][0], project.ranges["vertical"][0])
        p1 = project.point(float(tick), project.ranges["lateral"][1], project.ranges["vertical"][0])
        parts.append(_line(p0, p1, "grid"))
        parts.append(f'<text x="{p0[0]:.2f}" y="{p0[1] + 18:.2f}" font-size="10" fill="#555" text-anchor="middle">{tick:.1f}</text>')
    for tick in np.linspace(*project.ranges["lateral"], 5):
        p0 = project.point(project.ranges["mileage"][0], float(tick), project.ranges["vertical"][0])
        p1 = project.point(project.ranges["mileage"][1], float(tick), project.ranges["vertical"][0])
        parts.append(_line(p0, p1, "grid"))
    return parts


def _line(start: tuple[float, float], end: tuple[float, float], class_name: str) -> str:
    return f'<line class="{class_name}" x1="{start[0]:.2f}" y1="{start[1]:.2f}" x2="{end[0]:.2f}" y2="{end[1]:.2f}"/>'


def _gradient_bar_svg(x: int, y: int) -> str:
    rects = []
    steps = 40
    for index in range(steps):
        color = _color_ramp(index / (steps - 1))
        rects.append(f'<rect x="{x + index * 150 / steps:.2f}" y="{y}" width="{150 / steps + 0.4:.2f}" height="16" fill="{color}"/>')
    return "\n".join(rects)


def _read_mileage_rows(path: Path) -> list[tuple[float, str]]:
    rows: list[tuple[float, str]] = []
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith(("#", "%", "!")):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        try:
            rows.append((float(parts[0]), parts[1]))
        except ValueError:
            continue
    return sorted(rows, key=lambda item: item[0])


def _profile_file_index(root: Path) -> dict[str, Path]:
    index: dict[str, Path] = {}
    for path in sorted(root.rglob("*.txt")):
        if any("mileage" in part.lower() for part in path.parts):
            continue
        index.setdefault(path.name, path)
    return index


def _resolve_profile_path(filename: str, *, mileage_file: Path, profile_index: dict[str, Path]) -> Path:
    path = Path(filename)
    if path.is_absolute() and path.exists():
        return path
    relative_to_mileage = (mileage_file.parent / path).resolve()
    if relative_to_mileage.exists():
        return relative_to_mileage
    try:
        return profile_index[path.name]
    except KeyError as exc:
        raise FileNotFoundError(f"profile file {filename!r} referenced by {mileage_file} was not found") from exc


def _prepare_profile_points(
    raw: np.ndarray,
    *,
    points_per_profile: int,
    z_max: float | None,
    mirror_z: bool,
) -> np.ndarray:
    points = np.asarray(raw, dtype=float)
    points = points[np.all(np.isfinite(points[:, :2]), axis=1), :2]
    if z_max is not None:
        points = points[points[:, 1] <= float(z_max)]
    if points.shape[0] < 2:
        return np.zeros((0, 2), dtype=float)
    points = points.copy()
    if mirror_z:
        points[:, 1] = -points[:, 1]
    points = points[np.argsort(points[:, 0], kind="mergesort")]
    if points_per_profile <= 0 or points.shape[0] <= points_per_profile:
        return points
    y = np.linspace(float(points[0, 0]), float(points[-1, 0]), points_per_profile)
    z = np.interp(y, points[:, 0], points[:, 1])
    return np.column_stack((y, z))


def _range_with_padding(values: np.ndarray, *, pad_ratio: float) -> tuple[float, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return 0.0, 1.0
    low = float(np.min(finite))
    high = float(np.max(finite))
    if math.isclose(low, high):
        pad = max(abs(low) * 0.05, 1.0)
    else:
        pad = (high - low) * pad_ratio
    return low - pad, high + pad


def _normalize(value: float, value_range: tuple[float, float]) -> float:
    low, high = value_range
    return 0.0 if high == low else (float(value) - low) / (high - low)


def _color_ramp(t: float) -> str:
    t = float(np.clip(t, 0.0, 1.0))
    stops = np.array(
        [
            [37, 52, 148],
            [44, 127, 184],
            [65, 182, 196],
            [161, 218, 180],
            [255, 255, 191],
            [254, 178, 76],
            [215, 48, 39],
        ],
        dtype=float,
    )
    position = t * (len(stops) - 1)
    index = min(int(position), len(stops) - 2)
    ratio = position - index
    color = (1.0 - ratio) * stops[index] + ratio * stops[index + 1]
    return "#{:02x}{:02x}{:02x}".format(*(int(round(part)) for part in color))


def _resolve_input_path(repo_root: Path, value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    return (repo_root / path).resolve()


def _resolve_output_path(repo_root: Path, value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    return (repo_root / path).resolve()


def _parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render mileage-ordered rail profile sections as a 3D SVG wireframe.")
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--mileage-file", type=Path, default=Path(DEFAULT_MILEAGE_FILE))
    parser.add_argument("--profile-root", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=Path(DEFAULT_OUTPUT))
    parser.add_argument("--points-per-profile", type=int, default=140)
    parser.add_argument("--z-max", type=float, default=23.0e-3)
    parser.add_argument("--no-mirror-z", dest="mirror_z", action="store_false")
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=860)
    parser.set_defaults(mirror_z=True)
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(main())
