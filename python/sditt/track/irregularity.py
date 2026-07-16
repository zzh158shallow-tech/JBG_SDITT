from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Mapping

import numpy as np


TrackIrregularityModel = Literal["none", "china-ballastless"]
IRREGULARITY_COMPONENTS = ("vertical", "alignment", "cross_level", "gauge")
IRREGULARITY_COMPONENT_LABELS = {
    "vertical": "高低",
    "alignment": "轨向",
    "cross_level": "水平",
    "gauge": "轨距",
}


@dataclass(frozen=True)
class PiecewisePowerSpectrum:
    """One-sided spatial PSD expressed in mm²/(1/m)."""

    boundaries: tuple[float, ...]
    coefficients: tuple[float, ...]
    exponents: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.boundaries) != len(self.coefficients) + 1:
            raise ValueError("spectrum boundaries must contain one more value than segments")
        if len(self.coefficients) != len(self.exponents):
            raise ValueError("spectrum coefficients and exponents must have equal length")
        if any(right <= left for left, right in zip(self.boundaries[:-1], self.boundaries[1:], strict=True)):
            raise ValueError("spectrum boundaries must be strictly increasing")

    def values(self, spatial_frequency: float | np.ndarray) -> np.ndarray:
        frequencies = np.asarray(spatial_frequency, dtype=float)
        values = np.zeros_like(frequencies, dtype=float)
        for index, (left, right, coefficient, exponent) in enumerate(
            zip(self.boundaries[:-1], self.boundaries[1:], self.coefficients, self.exponents, strict=True)
        ):
            include_right = index == len(self.coefficients) - 1
            mask = (frequencies >= left) & ((frequencies <= right) if include_right else (frequencies < right))
            values[mask] = coefficient / frequencies[mask] ** exponent
        return values

    def variance_mm2(self, *, lower: float | None = None, upper: float | None = None) -> float:
        variance = 0.0
        for left, right, coefficient, exponent in zip(
            self.boundaries[:-1], self.boundaries[1:], self.coefficients, self.exponents, strict=True
        ):
            left = max(left, self.boundaries[0] if lower is None else float(lower))
            right = min(right, self.boundaries[-1] if upper is None else float(upper))
            if right <= left:
                continue
            if np.isclose(exponent, 1.0):
                variance += coefficient * np.log(right / left)
            else:
                variance += coefficient * (right ** (1.0 - exponent) - left ** (1.0 - exponent)) / (
                    1.0 - exponent
                )
        return float(variance)


CHINA_BALLASTLESS_SPECTRA: Mapping[str, PiecewisePowerSpectrum] = {
    "vertical": PiecewisePowerSpectrum(
        boundaries=(0.005, 0.0187, 0.0474, 0.1533, 0.5),
        coefficients=(1.0544e-5, 3.5588e-3, 1.9784e-2, 3.9488e-4),
        exponents=(3.3891, 1.9271, 1.3643, 3.4516),
    ),
    "alignment": PiecewisePowerSpectrum(
        boundaries=(0.005, 0.0450, 0.1234, 0.5),
        coefficients=(3.9513e-3, 1.1047e-2, 7.5633e-4),
        exponents=(1.8670, 1.5354, 2.8171),
    ),
    "cross_level": PiecewisePowerSpectrum(
        boundaries=(0.005, 0.0258, 0.1163, 0.5),
        coefficients=(3.6148e-3, 4.3685e-2, 4.5867e-3),
        exponents=(1.7278, 1.0461, 2.0939),
    ),
    "gauge": PiecewisePowerSpectrum(
        boundaries=(0.005, 0.1090, 0.2938, 0.5),
        coefficients=(5.4978e-2, 5.0701e-3, 1.8778e-4),
        exponents=(0.8282, 1.9037, 4.5948),
    ),
}


@dataclass(frozen=True)
class TrackIrregularitySettings:
    """Configuration for deterministic random-phase track irregularity."""

    model: TrackIrregularityModel = "none"
    seed: int = 20260716
    min_wavelength_m: float = 2.0
    max_wavelength_m: float = 200.0
    spatial_frequency_step: float = 0.001

    def __post_init__(self) -> None:
        if self.model not in ("none", "china-ballastless"):
            raise ValueError(f"unsupported track irregularity model {self.model!r}")
        if self.min_wavelength_m <= 0.0 or self.max_wavelength_m <= self.min_wavelength_m:
            raise ValueError("irregularity wavelengths must satisfy 0 < min < max")
        if self.spatial_frequency_step <= 0.0:
            raise ValueError("spatial_frequency_step must be positive")

    @property
    def min_spatial_frequency(self) -> float:
        return 1.0 / self.max_wavelength_m

    @property
    def max_spatial_frequency(self) -> float:
        return 1.0 / self.min_wavelength_m


@dataclass(frozen=True)
class TrackIrregularitySample:
    """Irregularity components and combined left/right rail values at one mileage."""

    mileage: float
    components_m: Mapping[str, float]
    component_slopes: Mapping[str, float]
    rail_displacement_m: Mapping[str, tuple[float, float]]
    rail_slopes: Mapping[str, tuple[float, float]]

    def rail_velocity(self, side: str, speed: float) -> tuple[float, float]:
        slope_y, slope_z = self.rail_slopes[side]
        return float(speed * slope_y), float(speed * slope_z)


@dataclass(frozen=True)
class TrackIrregularityProfile:
    """Continuous random-phase trigonometric-series realization."""

    settings: TrackIrregularitySettings
    spatial_frequency: np.ndarray
    frequency_width: np.ndarray
    target_psd_mm2_per_inv_m: Mapping[str, np.ndarray]
    amplitude_m: Mapping[str, np.ndarray]
    phase_rad: Mapping[str, np.ndarray]

    def component_values(self, mileage: float | np.ndarray) -> dict[str, np.ndarray]:
        x = np.atleast_1d(np.asarray(mileage, dtype=float))
        angle_base = 2.0 * np.pi * x[:, np.newaxis] * self.spatial_frequency[np.newaxis, :]
        return {
            component: np.sum(
                amplitude[np.newaxis, :] * np.cos(angle_base + self.phase_rad[component][np.newaxis, :]),
                axis=1,
            )
            for component, amplitude in self.amplitude_m.items()
        }

    def component_slopes(self, mileage: float | np.ndarray) -> dict[str, np.ndarray]:
        x = np.atleast_1d(np.asarray(mileage, dtype=float))
        angle_base = 2.0 * np.pi * x[:, np.newaxis] * self.spatial_frequency[np.newaxis, :]
        angular_frequency = 2.0 * np.pi * self.spatial_frequency
        return {
            component: np.sum(
                -amplitude[np.newaxis, :]
                * angular_frequency[np.newaxis, :]
                * np.sin(angle_base + self.phase_rad[component][np.newaxis, :]),
                axis=1,
            )
            for component, amplitude in self.amplitude_m.items()
        }

    def sample(self, mileage: float) -> TrackIrregularitySample:
        values = {name: float(value[0]) for name, value in self.component_values(mileage).items()}
        slopes = {name: float(value[0]) for name, value in self.component_slopes(mileage).items()}
        rail_displacement, rail_slopes = _combine_rail_values(values, slopes)
        return TrackIrregularitySample(
            mileage=float(mileage),
            components_m=values,
            component_slopes=slopes,
            rail_displacement_m=rail_displacement,
            rail_slopes=rail_slopes,
        )

    def theoretical_rms_mm(self) -> dict[str, float]:
        return {
            name: float(
                np.sqrt(
                    spectrum.variance_mm2(
                        lower=self.settings.min_spatial_frequency,
                        upper=self.settings.max_spatial_frequency,
                    )
                )
            )
            for name, spectrum in CHINA_BALLASTLESS_SPECTRA.items()
        }

    def discrete_rms_mm(self) -> dict[str, float]:
        return {
            name: float(np.sqrt(np.sum(psd * self.frequency_width)))
            for name, psd in self.target_psd_mm2_per_inv_m.items()
        }

    def summary(self) -> dict[str, object]:
        return {
            "model": self.settings.model,
            "seed": self.settings.seed,
            "min_wavelength_m": self.settings.min_wavelength_m,
            "max_wavelength_m": self.settings.max_wavelength_m,
            "min_spatial_frequency_1_per_m": self.settings.min_spatial_frequency,
            "max_spatial_frequency_1_per_m": self.settings.max_spatial_frequency,
            "spatial_frequency_step_1_per_m": self.settings.spatial_frequency_step,
            "theoretical_rms_mm": self.theoretical_rms_mm(),
            "discrete_rms_mm": self.discrete_rms_mm(),
        }


def build_track_irregularity_profile(
    settings: TrackIrregularitySettings,
) -> TrackIrregularityProfile | None:
    """Build one reproducible Chinese ballastless-track spectrum realization."""

    if settings.model == "none":
        return None
    f_min = settings.min_spatial_frequency
    f_max = settings.max_spatial_frequency
    left_edges = np.arange(f_min, f_max, settings.spatial_frequency_step, dtype=float)
    right_edges = np.minimum(left_edges + settings.spatial_frequency_step, f_max)
    widths = right_edges - left_edges
    frequencies = 0.5 * (left_edges + right_edges)
    rng = np.random.default_rng(settings.seed)
    target_psd: dict[str, np.ndarray] = {}
    amplitude_m: dict[str, np.ndarray] = {}
    phase_rad: dict[str, np.ndarray] = {}
    for component in IRREGULARITY_COMPONENTS:
        psd = CHINA_BALLASTLESS_SPECTRA[component].values(frequencies)
        target_psd[component] = psd
        amplitude_m[component] = np.sqrt(2.0 * psd * widths) / 1000.0
        phase_rad[component] = rng.uniform(0.0, 2.0 * np.pi, size=frequencies.size)
    return TrackIrregularityProfile(
        settings=settings,
        spatial_frequency=frequencies,
        frequency_width=widths,
        target_psd_mm2_per_inv_m=target_psd,
        amplitude_m=amplitude_m,
        phase_rad=phase_rad,
    )


def export_track_irregularity(
    profile: TrackIrregularityProfile,
    output_dir: str | Path,
    *,
    mileage_start: float = 0.0,
    mileage_end: float = 150.0,
    mileage_step: float = 0.05,
) -> tuple[Path, Path, Path]:
    """Write the realized line, target/discrete spectrum, and a four-panel SVG."""

    if mileage_end <= mileage_start or mileage_step <= 0.0:
        raise ValueError("invalid irregularity export mileage range")
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    mileage = np.arange(mileage_start, mileage_end + mileage_step / 2.0, mileage_step, dtype=float)
    values = profile.component_values(mileage)
    rail = _combine_rail_arrays(values)

    csv_path = output_path / "track_irregularity.csv"
    columns = [mileage]
    headers = ["mileage_m"]
    for component in IRREGULARITY_COMPONENTS:
        columns.extend((values[component], values[component] * 1000.0))
        headers.extend((f"{component}_m", f"{component}_mm"))
    for side in ("L", "R"):
        for axis in ("y", "z"):
            series = rail[f"{side}_{axis}"]
            columns.extend((series, series * 1000.0))
            headers.extend((f"rail_{side}_{axis}_m", f"rail_{side}_{axis}_mm"))
    np.savetxt(csv_path, np.column_stack(columns), delimiter=",", header=",".join(headers), comments="")

    spectrum_path = output_path / "track_irregularity_spectrum.csv"
    spectrum_columns = [profile.spatial_frequency, 1.0 / profile.spatial_frequency, profile.frequency_width]
    spectrum_headers = ["spatial_frequency_1_per_m", "wavelength_m", "frequency_width_1_per_m"]
    for component in IRREGULARITY_COMPONENTS:
        target = profile.target_psd_mm2_per_inv_m[component]
        discrete = profile.amplitude_m[component] ** 2 * 1.0e6 / (2.0 * profile.frequency_width)
        spectrum_columns.extend((target, discrete))
        spectrum_headers.extend((f"{component}_target_psd_mm2_per_1_per_m", f"{component}_discrete_psd_mm2_per_1_per_m"))
    np.savetxt(
        spectrum_path,
        np.column_stack(spectrum_columns),
        delimiter=",",
        header=",".join(spectrum_headers),
        comments="",
    )

    svg_path = output_path / "track_irregularity.svg"
    svg_path.write_text(_irregularity_svg(mileage, values), encoding="utf-8")
    return csv_path, spectrum_path, svg_path


def _combine_rail_values(
    values: Mapping[str, float],
    slopes: Mapping[str, float],
) -> tuple[dict[str, tuple[float, float]], dict[str, tuple[float, float]]]:
    def combine(source: Mapping[str, float]) -> dict[str, tuple[float, float]]:
        return {
            "L": (
                source["alignment"] - source["gauge"] / 2.0,
                source["vertical"] - source["cross_level"] / 2.0,
            ),
            "R": (
                source["alignment"] + source["gauge"] / 2.0,
                source["vertical"] + source["cross_level"] / 2.0,
            ),
        }

    return combine(values), combine(slopes)


def _combine_rail_arrays(values: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {
        "L_y": values["alignment"] - values["gauge"] / 2.0,
        "R_y": values["alignment"] + values["gauge"] / 2.0,
        "L_z": values["vertical"] - values["cross_level"] / 2.0,
        "R_z": values["vertical"] + values["cross_level"] / 2.0,
    }


def _irregularity_svg(mileage: np.ndarray, values: Mapping[str, np.ndarray]) -> str:
    width, height = 1120, 760
    margin_left, margin_right, margin_top, margin_bottom = 76, 24, 36, 48
    gap = 24
    panel_height = (height - margin_top - margin_bottom - 3 * gap) / 4.0
    plot_width = width - margin_left - margin_right
    x0, x1 = float(mileage[0]), float(mileage[-1])
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#263238}.axis{stroke:#53666f;stroke-width:1}.grid{stroke:#e1e7ea;stroke-width:1}.curve{fill:none;stroke:#1f77b4;stroke-width:1.4}</style>',
    ]
    for panel, component in enumerate(IRREGULARITY_COMPONENTS):
        y = np.asarray(values[component], dtype=float) * 1000.0
        y_abs = max(float(np.max(np.abs(y), initial=0.0)), 1.0e-9)
        y0 = margin_top + panel * (panel_height + gap)

        def sx(value: float) -> float:
            return margin_left + (value - x0) / max(x1 - x0, np.finfo(float).eps) * plot_width

        def sy(value: float) -> float:
            return y0 + panel_height / 2.0 - value / y_abs * panel_height * 0.44

        points = " ".join(f"{sx(float(x)):.2f},{sy(float(v)):.2f}" for x, v in zip(mileage, y, strict=True))
        parts.extend(
            [
                f'<line x1="{margin_left}" y1="{sy(0.0):.2f}" x2="{width-margin_right}" y2="{sy(0.0):.2f}" class="grid"/>',
                f'<line x1="{margin_left}" y1="{y0:.2f}" x2="{margin_left}" y2="{y0+panel_height:.2f}" class="axis"/>',
                f'<polyline points="{points}" class="curve"/>',
                f'<text x="14" y="{y0+18:.2f}" font-size="14">{IRREGULARITY_COMPONENT_LABELS[component]} (mm)</text>',
                f'<text x="{margin_left-8}" y="{sy(y_abs)+4:.2f}" text-anchor="end" font-size="11">{y_abs:.2f}</text>',
                f'<text x="{margin_left-8}" y="{sy(-y_abs)+4:.2f}" text-anchor="end" font-size="11">{-y_abs:.2f}</text>',
            ]
        )
    parts.append(f'<text x="{width/2:.1f}" y="{height-12}" text-anchor="middle" font-size="14">Mileage (m)</text>')
    parts.append("</svg>")
    return "".join(parts)
