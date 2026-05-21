from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from sditt.contact import hertz_normal_force
from sditt.integrators import LinearSecondOrderSystem

from .coupled import CoupledIterationSettings, CoupledStepCallbacks, CoupledStepState, run_coupled_time_iteration


@dataclass(frozen=True)
class SmallRigidWheelsetCase:
    """Deterministic one-wheelset case for coupled-loop verification."""

    speed: float = 25.0
    distance: float = 0.5
    dt: float = 1.0e-4
    static_wheel_load: float = 70_000.0
    static_penetration: float = 4.0e-5
    irregularity_amplitude: float = 1.5e-5
    irregularity_start: float = 0.15
    irregularity_length: float = 0.20
    rail_mass: float = 1_500.0
    rail_stiffness: float = 8.0e7
    rail_damping: float = 1.2e5
    wheelset_mass: float = 1_600.0
    primary_stiffness: float = 1.2e6
    primary_damping: float = 2.0e4


@dataclass(frozen=True)
class SmallRigidWheelsetResult:
    """Small-case response and wheel-rail normal-force curve."""

    mileage: np.ndarray
    normal_force: np.ndarray
    penetration: np.ndarray
    rail_irregularity: np.ndarray
    iterations: np.ndarray
    displacement: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


def run_small_rigid_wheelset_case(case: SmallRigidWheelsetCase | None = None) -> SmallRigidWheelsetResult:
    """Run one wheelset over a short deterministic profile with no random roughness."""

    case = case or SmallRigidWheelsetCase()
    n_steps = int(np.ceil(case.distance / (case.speed * case.dt)))
    permeability = case.static_penetration / case.static_wheel_load ** (2.0 / 3.0)
    system = LinearSecondOrderSystem(
        mass=np.diag([case.rail_mass, case.wheelset_mass]),
        damping=np.array(
            [
                [case.rail_damping + case.primary_damping, -case.primary_damping],
                [-case.primary_damping, case.primary_damping],
            ],
            dtype=float,
        ),
        stiffness=np.array(
            [
                [case.rail_stiffness + case.primary_stiffness, -case.primary_stiffness],
                [-case.primary_stiffness, case.primary_stiffness],
            ],
            dtype=float,
        ),
    )

    def recover_track_response(state: CoupledStepState) -> dict[str, float]:
        return {
            "rail_vertical": float(state.displacement[0]),
            "rail_velocity": float(state.velocity[0]),
        }

    def contact_geometry(state: CoupledStepState, rail: dict[str, float]) -> dict[str, float]:
        mileage = case.speed * state.time
        irregularity = _half_sine_irregularity(
            mileage,
            start=case.irregularity_start,
            length=case.irregularity_length,
            amplitude=case.irregularity_amplitude,
        )
        penetration = case.static_penetration + irregularity + state.displacement[1] - rail["rail_vertical"]
        return {
            "mileage": mileage,
            "irregularity": irregularity,
            "penetration": penetration,
        }

    def contact_force(state: CoupledStepState, rail: dict[str, float], geometry: dict[str, float]) -> np.ndarray:
        normal = hertz_normal_force(geometry["penetration"], permeability)
        delta_normal = normal - case.static_wheel_load
        return np.array([delta_normal, -delta_normal], dtype=float)

    result = run_coupled_time_iteration(
        system,
        CoupledStepCallbacks(recover_track_response, contact_geometry, contact_force),
        dt=case.dt,
        n_steps=n_steps,
        settings=CoupledIterationSettings(
            max_iterations=30,
            force_tolerance=1.0e-5,
            absolute_force_tolerance=1.0e-2,
            relaxation=0.6,
            min_dt=case.dt / 16.0,
            shrink_factor=0.5,
        ),
    )

    mileage = case.speed * result.time
    penetration = np.empty_like(mileage)
    irregularity = np.empty_like(mileage)
    normal_force = np.empty_like(mileage)
    for i, time in enumerate(result.time):
        rail_vertical = float(result.displacement[i, 0])
        wheel_vertical = float(result.displacement[i, 1])
        irregularity[i] = _half_sine_irregularity(
            mileage[i],
            start=case.irregularity_start,
            length=case.irregularity_length,
            amplitude=case.irregularity_amplitude,
        )
        penetration[i] = case.static_penetration + irregularity[i] + wheel_vertical - rail_vertical
        normal_force[i] = hertz_normal_force(penetration[i], permeability)

    return SmallRigidWheelsetResult(
        mileage=mileage,
        normal_force=normal_force,
        penetration=penetration,
        rail_irregularity=irregularity,
        iterations=result.iterations,
        displacement=result.displacement,
        velocity=result.velocity,
        acceleration=result.acceleration,
    )


def export_small_rigid_wheelset_curve(
    output_dir: str | Path,
    case: SmallRigidWheelsetCase | None = None,
) -> tuple[SmallRigidWheelsetResult, Path, Path]:
    """Run the small case and export CSV plus an SVG normal-force curve."""

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    result = run_small_rigid_wheelset_case(case)
    csv_path = output_path / "small_rigid_wheelset_force.csv"
    svg_path = output_path / "small_rigid_wheelset_force.svg"
    data = np.column_stack(
        (
            result.mileage,
            result.normal_force,
            result.penetration,
            result.rail_irregularity,
            result.iterations,
        )
    )
    np.savetxt(
        csv_path,
        data,
        delimiter=",",
        header="mileage_m,normal_force_N,penetration_m,rail_irregularity_m,iterations",
        comments="",
    )
    _write_force_svg(svg_path, result.mileage, result.normal_force)
    return result, csv_path, svg_path


def _half_sine_irregularity(mileage: float, *, start: float, length: float, amplitude: float) -> float:
    if mileage < start or mileage > start + length:
        return 0.0
    phase = (mileage - start) / length
    return float(amplitude * np.sin(np.pi * phase))


def _write_force_svg(path: Path, mileage: np.ndarray, normal_force: np.ndarray) -> None:
    width, height = 900, 420
    margin_left, margin_right, margin_top, margin_bottom = 70, 24, 24, 56
    x0, x1 = float(mileage.min()), float(mileage.max())
    y0 = float(np.floor(normal_force.min() / 1000.0) * 1000.0)
    y1 = float(np.ceil(normal_force.max() / 1000.0) * 1000.0)
    if y0 == y1:
        y0 -= 1.0
        y1 += 1.0

    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    def sx(x: float) -> float:
        return margin_left + (x - x0) / (x1 - x0) * plot_w

    def sy(y: float) -> float:
        return margin_top + (y1 - y) / (y1 - y0) * plot_h

    points = " ".join(f"{sx(float(x)):.2f},{sy(float(y)):.2f}" for x, y in zip(mileage, normal_force))
    y_ticks = np.linspace(y0, y1, 6)
    x_ticks = np.linspace(x0, x1, 6)
    grid = []
    labels = []
    for value in y_ticks:
        y = sy(float(value))
        grid.append(f'<line x1="{margin_left}" y1="{y:.2f}" x2="{width - margin_right}" y2="{y:.2f}" class="grid"/>')
        labels.append(f'<text x="{margin_left - 10}" y="{y + 4:.2f}" text-anchor="end">{value / 1000:.1f}</text>')
    for value in x_ticks:
        x = sx(float(value))
        grid.append(f'<line x1="{x:.2f}" y1="{margin_top}" x2="{x:.2f}" y2="{height - margin_bottom}" class="grid"/>')
        labels.append(f'<text x="{x:.2f}" y="{height - 25}" text-anchor="middle">{value:.2f}</text>')

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<style>
  text {{ font-family: Arial, sans-serif; font-size: 13px; fill: #263238; }}
  .title {{ font-size: 18px; font-weight: 700; }}
  .grid {{ stroke: #d9e0e3; stroke-width: 1; }}
  .axis {{ stroke: #53666f; stroke-width: 1.4; }}
  .curve {{ fill: none; stroke: #1f77b4; stroke-width: 2.4; }}
</style>
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="{margin_left}" y="18" class="title">Small rigid wheelset normal force</text>
{''.join(grid)}
<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{height - margin_bottom}" class="axis"/>
<line x1="{margin_left}" y1="{height - margin_bottom}" x2="{width - margin_right}" y2="{height - margin_bottom}" class="axis"/>
<polyline points="{points}" class="curve"/>
{''.join(labels)}
<text x="{width / 2:.1f}" y="{height - 6}" text-anchor="middle">Mileage (m)</text>
<text x="18" y="{height / 2:.1f}" text-anchor="middle" transform="rotate(-90 18 {height / 2:.1f})">Normal force (kN)</text>
</svg>
"""
    path.write_text(svg, encoding="utf-8")
