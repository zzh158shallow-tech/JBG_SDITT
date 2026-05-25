from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sditt.config import ProjectPaths


DEFAULT_OUTPUT_DIR = "python/outputs/profile_3d/combined_right_07009"
DEFAULT_START_MILEAGE = 32.0
DEFAULT_END_MILEAGE = 130.0
DEFAULT_MILEAGE_STEP = 0.1
GAUGE = 1.435
ORI_PRR = 0.0355
VERTICAL_OFFSET = 0.6


@dataclass(frozen=True)
class MileageEntry:
    mileage: float
    profile_file: Path


@dataclass(frozen=True)
class BezierData:
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    divisions: np.ndarray
    x_to_t: tuple[np.ndarray, ...]


@dataclass(frozen=True)
class CombinedSection:
    index: int
    mileage: float
    points: np.ndarray
    sources: tuple[str, ...]


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    paths = ProjectPaths.from_repo_root(args.repo_root)
    output_dir = _resolve_output_path(paths.root, args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    entries = _default_entries(paths)
    bezier = _default_bezier(entries, num_interp=args.points_per_profile)
    mileages = _mileage_grid(args.start_mileage, args.end_mileage, args.mileage_step)
    sections = [
        _combined_right_section(
            index=index,
            mileage=float(mileage),
            entries=entries,
            bezier=bezier,
            num_interp=args.points_per_profile,
            mirror_z=args.mirror_z,
        )
        for index, mileage in enumerate(mileages, start=1)
    ]
    sections = [section for section in sections if section.points.size]
    if not sections:
        raise SystemExit("no combined right-rail profiles were generated")

    csv_path = output_dir / "combined_right_R1_R2_R3_3d.csv"
    xyz_path = output_dir / "combined_right_R1_R2_R3_3d.xyz"
    ply_path = output_dir / "combined_right_R1_R2_R3_3d.ply"
    svg_path = output_dir / "combined_right_R1_R2_R3_top_view.svg"
    manifest_path = output_dir / "combined_right_R1_R2_R3_manifest.json"

    _write_csv(csv_path, sections)
    _write_xyz(xyz_path, sections)
    _write_ply(ply_path, sections)
    svg_path.write_text(_render_top_view_svg(sections, width=args.width, height=args.height), encoding="utf-8")
    manifest_path.write_text(
        json.dumps(
            {
                "turnout": "07(009)",
                "rail": "right combined R = R1 + R2 + R3",
                "coordinate_system": "track coordinates",
                "x": "mileage_m",
                "y": "lateral_track_m",
                "z": "mirrored_vertical_track_m" if args.mirror_z else "vertical_track_m",
                "mirror_z": bool(args.mirror_z),
                "dynamic_displacement": "not applied",
                "track_irregularity": "not applied",
                "gauge_m": GAUGE,
                "ori_prr_m": ORI_PRR,
                "vertical_offset_m": VERTICAL_OFFSET,
                "start_mileage_m": float(args.start_mileage),
                "end_mileage_m": float(args.end_mileage),
                "mileage_step_m": float(args.mileage_step),
                "section_count": len(sections),
                "point_count": int(sum(section.points.shape[0] for section in sections)),
                "outputs": {
                    "csv": str(csv_path),
                    "xyz": str(xyz_path),
                    "ply": str(ply_path),
                    "top_view_svg": str(svg_path),
                },
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"wrote {csv_path}")
    print(f"wrote {xyz_path}")
    print(f"wrote {ply_path}")
    print(f"wrote {svg_path}")
    print(f"wrote {manifest_path}")
    return 0


def _default_entries(paths: ProjectPaths) -> dict[str, tuple[MileageEntry, ...]]:
    profile_index = _profile_file_index(paths.profile_dir)
    mileage_root = paths.turnout_mileage_dir
    return {
        "R1": _read_mileage_entries(
            mileage_root / "07(009)-Mileage-qjbg.txt",
            profile_index=profile_index,
            skip_first=2,
            skip_last=1,
        ),
        "R2": _read_mileage_entries(
            mileage_root / "07(009)-Mileage-zjg_zgyg_250722.txt",
            profile_index=profile_index,
        ),
        "R3": _read_mileage_entries(
            mileage_root / "07(009)-Mileage-cxg.txt",
            profile_index=profile_index,
            skip_last=1,
        ),
    }


def _default_bezier(entries: Mapping[str, tuple[MileageEntry, ...]], *, num_interp: int) -> dict[str, BezierData]:
    return {
        "R1": _create_bezier_data(
            entries["R1"],
            np.array([[entries["R1"][0].mileage, entries["R1"][-1].mileage]], dtype=float),
            num_interp=num_interp,
        ),
        "R2": _create_bezier_data(
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
        "R3": _create_bezier_data(
            entries["R3"],
            np.array([[entries["R3"][0].mileage, entries["R3"][-1].mileage]], dtype=float),
            num_interp=num_interp,
        ),
    }


def _combined_right_section(
    *,
    index: int,
    mileage: float,
    entries: Mapping[str, tuple[MileageEntry, ...]],
    bezier: Mapping[str, BezierData],
    num_interp: int,
    mirror_z: bool,
) -> CombinedSection:
    dummy_profiles = {
        "R1": _offset_right_profile(
            _interpolate_profile(
                mileage,
                entries["R1"],
                same_front_profile=True,
                same_rear_profile=False,
                bezier=bezier["R1"],
                num_interp=num_interp,
            )
        ),
        "R2": _offset_right_profile(
            _interpolate_profile(
                mileage,
                entries["R2"],
                same_front_profile=False,
                same_rear_profile=False,
                bezier=bezier["R2"],
                num_interp=num_interp,
            )
        ),
        "R3": _offset_right_profile(
            _interpolate_profile(
                mileage,
                entries["R3"],
                same_front_profile=False,
                same_rear_profile=True,
                bezier=bezier["R3"],
                num_interp=num_interp,
            )
        ),
    }
    points, sources = _merge_right_profiles(dummy_profiles, ("R1", "R2", "R3"))
    if mirror_z and points.size:
        points = points.copy()
        points[:, 1] *= -1.0
    return CombinedSection(index=index, mileage=mileage, points=points, sources=sources)


def _interpolate_profile(
    mileage: float,
    entries: tuple[MileageEntry, ...],
    *,
    same_front_profile: bool,
    same_rear_profile: bool,
    bezier: BezierData,
    num_interp: int,
) -> np.ndarray:
    first = entries[0]
    last = entries[-1]
    if mileage < first.mileage:
        if not same_front_profile:
            return np.empty((0, 2), dtype=float)
        return _sorted_profile(first.profile_file)
    if mileage > last.mileage:
        if not same_rear_profile:
            return np.empty((0, 2), dtype=float)
        return _sorted_profile(last.profile_file)

    mileage_values = np.array([entry.mileage for entry in entries], dtype=float)
    profile_index = int(np.searchsorted(mileage_values, mileage, side="left"))
    if profile_index == 0:
        front_entry = entries[0]
        rear_entry = entries[0]
    else:
        front_entry = entries[profile_index - 1]
        rear_entry = entries[profile_index]
    profile = _bezier_profile_at(mileage, bezier, num_interp)
    if profile is not None:
        return _sort_points(profile)
    return _linear_profile_between(
        _sorted_profile(front_entry.profile_file),
        _sorted_profile(rear_entry.profile_file),
        mileage,
        front_entry.mileage,
        rear_entry.mileage,
        num_interp,
    )


def _offset_right_profile(profile: np.ndarray) -> np.ndarray:
    if profile.size == 0:
        return np.empty((0, 2), dtype=float)
    out = profile.copy()
    lateral_origin = GAUGE / 2.0 + ORI_PRR
    out[:, 0] = out[:, 0] + lateral_origin
    out[:, 1] = out[:, 1] + VERTICAL_OFFSET
    return out


def _merge_right_profiles(
    dummy_profiles: Mapping[str, np.ndarray],
    dummy_rails: tuple[str, ...],
) -> tuple[np.ndarray, tuple[str, ...]]:
    nonempty = [name for name in dummy_rails if dummy_profiles[name].size]
    if not nonempty:
        return np.empty((0, 2), dtype=float), ()
    if len(nonempty) == 1:
        profile = _sort_points(dummy_profiles[nonempty[0]])
        return profile, tuple(nonempty[0] for _ in range(profile.shape[0]))

    masks = {name: np.ones(dummy_profiles[name].shape[0], dtype=bool) for name in nonempty}
    for outside, inside in zip(nonempty[:-1], nonempty[1:], strict=False):
        outside_profile = dummy_profiles[outside]
        inside_profile = dummy_profiles[inside]
        masks[outside] &= outside_profile[:, 0] > max(np.min(outside_profile[:, 0]), np.max(inside_profile[:, 0]))
        masks[inside] &= inside_profile[:, 0] < min(np.min(outside_profile[:, 0]), np.max(inside_profile[:, 0]))

    merged = []
    sources = []
    for name in nonempty:
        selected = dummy_profiles[name][masks[name], :]
        merged.append(selected)
        sources.extend([name] * selected.shape[0])
    points = np.vstack(merged)
    order = np.argsort(points[:, 0], kind="mergesort")
    return points[order], tuple(np.asarray(sources, dtype=object)[order].tolist())


def _create_bezier_data(
    mileage_entries: tuple[MileageEntry, ...],
    divisions: np.ndarray,
    *,
    num_interp: int,
) -> BezierData:
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
        segment = [entry for entry in mileage_entries if div[0] <= entry.mileage <= div[1]]
        if len(segment) >= 2 and segment[-2].mileage == segment[-1].mileage:
            segment = segment[:-1]
        elif len(segment) >= 2 and segment[0].mileage == segment[1].mileage:
            segment = segment[1:]
        divisions_out[div_index, 2] = len(segment)

        for entry in segment:
            profile = _sorted_profile(entry.profile_file)
            profile = profile[profile[:, 1] <= 23.0e-3]
            y = np.linspace(float(profile[0, 0]), float(profile[-1, 0]), num_interp)
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
            x_to_t.append(np.column_stack((_bezier_values(x_values, sample_t), sample_t)))

    y_data = np.column_stack(all_y) if all_y else np.empty((num_interp, 0), dtype=float)
    z_data = np.column_stack(all_z) if all_z else np.empty((num_interp, 0), dtype=float)
    return BezierData(x=np.array(all_x), y=y_data, z=z_data, divisions=divisions_out, x_to_t=tuple(x_to_t))


def _bezier_profile_at(mileage: float, bezier: BezierData, num_interp: int) -> np.ndarray | None:
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
    return np.column_stack((y_columns @ weights, z_columns @ weights))


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


def _write_csv(path: Path, sections: list[CombinedSection]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "section_index",
                "mileage_m",
                "point_index",
                "source_rail",
                "x_mileage_m",
                "y_lateral_track_m",
                "z_vertical_mirrored_track_m",
            ]
        )
        for section in sections:
            for point_index, (point, source) in enumerate(zip(section.points, section.sources, strict=True), start=1):
                writer.writerow(
                    [
                        section.index,
                        f"{section.mileage:.6f}",
                        point_index,
                        source,
                        f"{section.mileage:.6f}",
                        f"{point[0]:.9f}",
                        f"{point[1]:.9f}",
                    ]
                )


def _write_xyz(path: Path, sections: list[CombinedSection]) -> None:
    with path.open("w", encoding="utf-8") as stream:
        for section in sections:
            for point in section.points:
                stream.write(f"{section.mileage:.9f} {point[0]:.9f} {point[1]:.9f}\n")


def _write_ply(path: Path, sections: list[CombinedSection]) -> None:
    vertex_count = sum(section.points.shape[0] for section in sections)
    edge_count = sum(max(section.points.shape[0] - 1, 0) for section in sections)
    with path.open("w", encoding="utf-8") as stream:
        stream.write("ply\n")
        stream.write("format ascii 1.0\n")
        stream.write("comment 07(009) combined right rail profile R1+R2+R3\n")
        stream.write("comment x=mileage_m y=lateral_track_m z=mirrored_vertical_track_m\n")
        stream.write(f"element vertex {vertex_count}\n")
        stream.write("property float x\n")
        stream.write("property float y\n")
        stream.write("property float z\n")
        stream.write(f"element edge {edge_count}\n")
        stream.write("property int vertex1\n")
        stream.write("property int vertex2\n")
        stream.write("end_header\n")
        for section in sections:
            for point in section.points:
                stream.write(f"{section.mileage:.9f} {point[0]:.9f} {point[1]:.9f}\n")
        vertex_offset = 0
        for section in sections:
            for local_index in range(section.points.shape[0] - 1):
                stream.write(f"{vertex_offset + local_index} {vertex_offset + local_index + 1}\n")
            vertex_offset += section.points.shape[0]


def _render_top_view_svg(sections: list[CombinedSection], *, width: int, height: int) -> str:
    mileages = np.asarray([section.mileage for section in sections], dtype=float)
    lateral = np.concatenate([section.points[:, 0] for section in sections])
    vertical = np.concatenate([section.points[:, 1] for section in sections])
    x_range = _range_with_padding(mileages, pad_ratio=0.02)
    y_range = _range_with_padding(lateral, pad_ratio=0.05)
    z_range = _range_with_padding(vertical, pad_ratio=0.02)
    left_margin = 72.0
    right_margin = 160.0
    top_margin = 72.0
    bottom_margin = 72.0
    plot_width = width - left_margin - right_margin
    plot_height = height - top_margin - bottom_margin

    def project(mileage: float, lateral_value: float) -> tuple[float, float]:
        x = left_margin + _normalize(mileage, x_range) * plot_width
        y = height - bottom_margin - _normalize(lateral_value, y_range) * plot_height
        return x, y

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,Helvetica,sans-serif}.axis{stroke:#202020;stroke-width:1.2}.grid{stroke:#dddddd;stroke-width:.7}.profile{fill:none;stroke-width:.85;stroke-linecap:round}</style>',
        f'<text x="24" y="34" font-size="18" fill="#202020">07(009) Right Combined Rail Profile, Top View, Colored by Z ({len(sections)} sections)</text>',
    ]
    x0, y0 = project(x_range[0], y_range[0])
    x1, _ = project(x_range[1], y_range[0])
    _, y1 = project(x_range[0], y_range[1])
    parts.append(f'<line class="axis" x1="{x0:.2f}" y1="{y0:.2f}" x2="{x1:.2f}" y2="{y0:.2f}"/>')
    parts.append(f'<line class="axis" x1="{x0:.2f}" y1="{y0:.2f}" x2="{x0:.2f}" y2="{y1:.2f}"/>')
    for tick in np.linspace(x_range[0], x_range[1], 8):
        tx0, ty0 = project(float(tick), y_range[0])
        tx1, ty1 = project(float(tick), y_range[1])
        parts.append(f'<line class="grid" x1="{tx0:.2f}" y1="{ty0:.2f}" x2="{tx1:.2f}" y2="{ty1:.2f}"/>')
        parts.append(f'<text x="{tx0:.2f}" y="{ty0 + 20:.2f}" font-size="10" fill="#555" text-anchor="middle">{tick:.1f}</text>')
    for tick in np.linspace(y_range[0], y_range[1], 6):
        tx0, ty0 = project(x_range[0], float(tick))
        tx1, ty1 = project(x_range[1], float(tick))
        parts.append(f'<line class="grid" x1="{tx0:.2f}" y1="{ty0:.2f}" x2="{tx1:.2f}" y2="{ty1:.2f}"/>')
        parts.append(f'<text x="{tx0 - 8:.2f}" y="{ty0 + 4:.2f}" font-size="10" fill="#555" text-anchor="end">{tick:.3f}</text>')
    parts.append(f'<text x="{width / 2:.2f}" y="{height - 22:.2f}" font-size="12" fill="#202020" text-anchor="middle">Mileage (m)</text>')
    parts.append(f'<text x="18" y="{height / 2:.2f}" font-size="12" fill="#202020" transform="rotate(-90 18 {height / 2:.2f})" text-anchor="middle">Lateral track Y (m)</text>')

    total_segments = sum(max(section.points.shape[0] - 1, 0) for section in sections)
    stride = max(1, int(math.ceil(total_segments / 120000)))
    for section in sections:
        projected = [project(section.mileage, point[0]) for point in section.points]
        for point_index in range(0, section.points.shape[0] - 1, stride):
            p0 = projected[point_index]
            p1 = projected[point_index + 1]
            z_value = 0.5 * (section.points[point_index, 1] + section.points[point_index + 1, 1])
            color = _color_ramp(_normalize(float(z_value), z_range))
            parts.append(
                f'<line class="profile" x1="{p0[0]:.2f}" y1="{p0[1]:.2f}" x2="{p1[0]:.2f}" y2="{p1[1]:.2f}" stroke="{color}" opacity="0.58"/>'
            )
    parts.extend(_z_colorbar_svg(x=width - 104, y=104, height=height - 228, z_range=z_range))
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def _z_colorbar_svg(*, x: float, y: float, height: float, z_range: tuple[float, float]) -> list[str]:
    width = 20.0
    steps = 80
    parts = [f'<text x="{x - 8:.2f}" y="{y - 18:.2f}" font-size="12" fill="#202020">Z (m)</text>']
    for index in range(steps):
        t0 = index / steps
        t1 = (index + 1) / steps
        color = _color_ramp(1.0 - (t0 + t1) * 0.5)
        rect_y = y + t0 * height
        rect_h = height / steps + 0.4
        parts.append(f'<rect x="{x:.2f}" y="{rect_y:.2f}" width="{width:.2f}" height="{rect_h:.2f}" fill="{color}"/>')
    parts.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{width:.2f}" height="{height:.2f}" fill="none" stroke="#202020" stroke-width="0.8"/>')
    for tick in np.linspace(z_range[0], z_range[1], 6):
        tick_y = y + (1.0 - _normalize(float(tick), z_range)) * height
        parts.append(f'<line x1="{x + width:.2f}" y1="{tick_y:.2f}" x2="{x + width + 5:.2f}" y2="{tick_y:.2f}" stroke="#202020" stroke-width="0.8"/>')
        parts.append(f'<text x="{x + width + 9:.2f}" y="{tick_y + 4:.2f}" font-size="10" fill="#555">{tick:.3f}</text>')
    return parts


def _read_mileage_entries(
    path: Path,
    *,
    profile_index: Mapping[str, Path],
    skip_first: int = 0,
    skip_last: int = 0,
) -> tuple[MileageEntry, ...]:
    entries: list[MileageEntry] = []
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        parts = raw_line.split()
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
        entries.append(MileageEntry(mileage=mileage, profile_file=profile_file))
    if skip_last:
        entries = entries[skip_first:-skip_last]
    else:
        entries = entries[skip_first:]
    return tuple(entries)


def _profile_file_index(root: Path) -> dict[str, Path]:
    index: dict[str, Path] = {}
    for path in sorted(root.rglob("*.txt")):
        if any("mileage" in part.lower() for part in path.parts):
            continue
        index.setdefault(path.name, path)
    return index


def _sorted_profile(path: Path) -> np.ndarray:
    return _sort_points(_load_profile_file(path)[:, :2])


def _load_profile_file(path: str | Path) -> np.ndarray:
    file_path = Path(path)
    if file_path.suffix.lower() in {".prr", ".prw"}:
        return _load_prr_profile(file_path)
    return _load_numeric_text(file_path)


def _load_numeric_text(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith(("!", "#", "%")):
            continue
        if "!" in line:
            line = line.split("!", 1)[0].strip()
        if "%" in line:
            line = line.split("%", 1)[0].strip()
        parts = line.replace(",", " ").split()
        try:
            rows.append([float(part) for part in parts])
        except ValueError:
            numeric_prefix: list[float] = []
            for part in parts:
                try:
                    numeric_prefix.append(float(part))
                except ValueError:
                    break
            if numeric_prefix:
                rows.append(numeric_prefix)
    if not rows:
        return np.empty((0, 0), dtype=float)
    width = max(len(row) for row in rows)
    data = np.full((len(rows), width), np.nan, dtype=float)
    for row_index, row in enumerate(rows):
        data[row_index, : len(row)] = row
    return data


def _load_prr_profile(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        content = raw_line.split("!", 1)[0].strip()
        if not content or "=" in content:
            continue
        if not re.match(r"^[+\-0-9.eE,\s]+$", content):
            continue
        parts = content.replace(",", " ").split()
        if len(parts) >= 2:
            rows.append([float(part) for part in parts])
    return np.array(rows, dtype=float) if rows else np.empty((0, 0), dtype=float)


def _sort_points(points: np.ndarray) -> np.ndarray:
    data = np.asarray(points, dtype=float)
    data = data[np.all(np.isfinite(data[:, :2]), axis=1), :2]
    if data.size == 0:
        return np.empty((0, 2), dtype=float)
    return data[np.argsort(data[:, 0], kind="mergesort")]


def _bezier_values(values: np.ndarray, t: np.ndarray) -> np.ndarray:
    weights = np.column_stack([_bezier_weights(len(values), value) for value in t])
    return values @ weights


def _bezier_weights(n_points: int, t: float) -> np.ndarray:
    degree = n_points - 1
    indexes = np.arange(n_points)
    coefficients = np.array([math.comb(degree, int(index)) for index in indexes], dtype=float)
    return coefficients * (1.0 - t) ** (degree - indexes) * t**indexes


def _mileage_grid(start: float, end: float, step: float) -> np.ndarray:
    if step <= 0.0:
        raise ValueError("mileage step must be positive")
    count = int(np.floor((end - start) / step)) + 1
    values = start + np.arange(count, dtype=float) * step
    if values.size == 0 or not np.isclose(values[-1], end):
        values = np.append(values, end)
    return values


def _range_with_padding(values: np.ndarray, *, pad_ratio: float) -> tuple[float, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return 0.0, 1.0
    low = float(np.min(finite))
    high = float(np.max(finite))
    pad = max((high - low) * pad_ratio, 1.0e-9)
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


def _resolve_output_path(repo_root: Path, value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    return (repo_root / path).resolve()


def _parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export 07(009) R1+R2+R3 combined right rail profiles as 3D files.")
    parser.add_argument("--repo-root", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=Path(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--start-mileage", type=float, default=DEFAULT_START_MILEAGE)
    parser.add_argument("--end-mileage", type=float, default=DEFAULT_END_MILEAGE)
    parser.add_argument("--mileage-step", type=float, default=DEFAULT_MILEAGE_STEP)
    parser.add_argument("--points-per-profile", type=int, default=180)
    parser.add_argument("--no-mirror-z", dest="mirror_z", action="store_false")
    parser.add_argument("--width", type=int, default=1400)
    parser.add_argument("--height", type=int, default=900)
    parser.set_defaults(mirror_z=True)
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(main())
