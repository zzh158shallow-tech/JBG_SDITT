from __future__ import annotations

import numpy as np
import pytest

from sditt.track import (
    CHINA_BALLASTLESS_SPECTRA,
    IRREGULARITY_COMPONENTS,
    TrackIrregularitySettings,
    build_track_irregularity_profile,
    export_track_irregularity,
)


def test_china_ballastless_piecewise_spectra_use_published_segments() -> None:
    expected = {
        "vertical": ((0.005, 0.0187, 0.0474, 0.1533, 0.5), (1.0544e-5, 3.5588e-3, 1.9784e-2, 3.9488e-4)),
        "alignment": ((0.005, 0.0450, 0.1234, 0.5), (3.9513e-3, 1.1047e-2, 7.5633e-4)),
        "cross_level": ((0.005, 0.0258, 0.1163, 0.5), (3.6148e-3, 4.3685e-2, 4.5867e-3)),
        "gauge": ((0.005, 0.1090, 0.2938, 0.5), (5.4978e-2, 5.0701e-3, 1.8778e-4)),
    }
    for component, (boundaries, coefficients) in expected.items():
        spectrum = CHINA_BALLASTLESS_SPECTRA[component]
        assert spectrum.boundaries == boundaries
        assert spectrum.coefficients == coefficients
        boundary = boundaries[1]
        assert spectrum.values(boundary - 1.0e-9) == pytest.approx(
            coefficients[0] / (boundary - 1.0e-9) ** spectrum.exponents[0]
        )
        assert spectrum.values(boundary) == pytest.approx(coefficients[1] / boundary ** spectrum.exponents[1])


def test_china_ballastless_theoretical_rms_matches_reference_values() -> None:
    profile = build_track_irregularity_profile(TrackIrregularitySettings(model="china-ballastless"))
    assert profile is not None

    assert profile.theoretical_rms_mm() == pytest.approx(
        {"vertical": 1.2203026, "alignment": 0.6677926, "cross_level": 0.5235391, "gauge": 0.3437111},
        rel=1.0e-6,
    )
    assert profile.discrete_rms_mm() == pytest.approx(profile.theoretical_rms_mm(), rel=1.0e-2)


def test_random_phase_profile_is_reproducible_and_seed_sensitive() -> None:
    first = build_track_irregularity_profile(TrackIrregularitySettings(model="china-ballastless", seed=12))
    same = build_track_irregularity_profile(TrackIrregularitySettings(model="china-ballastless", seed=12))
    different = build_track_irregularity_profile(TrackIrregularitySettings(model="china-ballastless", seed=13))
    assert first is not None and same is not None and different is not None
    x = np.linspace(0.0, 150.0, 301)

    for component in IRREGULARITY_COMPONENTS:
        assert np.array_equal(first.component_values(x)[component], same.component_values(x)[component])
        assert not np.array_equal(first.component_values(x)[component], different.component_values(x)[component])
        assert np.array_equal(
            first.target_psd_mm2_per_inv_m[component],
            different.target_psd_mm2_per_inv_m[component],
        )


def test_analytic_irregularity_slope_matches_centered_difference() -> None:
    profile = build_track_irregularity_profile(TrackIrregularitySettings(model="china-ballastless", seed=42))
    assert profile is not None
    x = 54.0
    dx = 1.0e-5
    values_before = profile.component_values(x - dx)
    values_after = profile.component_values(x + dx)
    slopes = profile.component_slopes(x)

    for component in IRREGULARITY_COMPONENTS:
        finite_difference = (values_after[component][0] - values_before[component][0]) / (2.0 * dx)
        assert slopes[component][0] == pytest.approx(finite_difference, rel=2.0e-8, abs=1.0e-10)


def test_left_right_rail_combination_recovers_four_source_components() -> None:
    profile = build_track_irregularity_profile(TrackIrregularitySettings(model="china-ballastless", seed=7))
    assert profile is not None
    sample = profile.sample(88.0)
    left_y, left_z = sample.rail_displacement_m["L"]
    right_y, right_z = sample.rail_displacement_m["R"]

    assert (left_z + right_z) / 2.0 == pytest.approx(sample.components_m["vertical"])
    assert right_z - left_z == pytest.approx(sample.components_m["cross_level"])
    assert (left_y + right_y) / 2.0 == pytest.approx(sample.components_m["alignment"])
    assert right_y - left_y == pytest.approx(sample.components_m["gauge"])


def test_track_irregularity_export_writes_finite_csv_spectrum_and_svg(tmp_path) -> None:
    profile = build_track_irregularity_profile(TrackIrregularitySettings(model="china-ballastless", seed=9))
    assert profile is not None
    csv_path, spectrum_path, svg_path = export_track_irregularity(
        profile,
        tmp_path,
        mileage_start=30.0,
        mileage_end=35.0,
        mileage_step=0.05,
    )

    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    spectrum = np.loadtxt(spectrum_path, delimiter=",", skiprows=1)
    assert np.all(np.isfinite(data))
    assert np.all(np.isfinite(spectrum))
    assert "vertical_mm" in csv_path.read_text(encoding="utf-8").splitlines()[0]
    assert "vertical_target_psd" in spectrum_path.read_text(encoding="utf-8").splitlines()[0]
    assert "高低" in svg_path.read_text(encoding="utf-8")
