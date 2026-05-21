from __future__ import annotations

import numpy as np

from sditt.config import ProjectPaths
from sditt.contact import (
    WheelPose2D,
    extreme_boundary,
    multi_point_contact_geometry,
    quasi_elastic_correction,
    single_point_contact_geometry,
    trace_wheel_profile,
)
from sditt.profiles import build_wheel_profiles


def test_trace_wheel_profile_zero_pose_matches_matlab_subset() -> None:
    wheel = np.array([[-0.01, 0.50], [0.00, 0.52], [0.01, 0.50]], dtype=float)
    angles = np.column_stack((wheel[:, 0], np.zeros(3)))

    trace = trace_wheel_profile(wheel, angles)

    assert np.allclose(trace.track_points[:, 0], 0.0)
    assert np.allclose(trace.track_points[:, 1], wheel[:, 0])
    assert np.allclose(trace.track_points[:, 2], wheel[:, 1])
    assert np.allclose(trace.contact_angles, 0.0)


def test_single_point_contact_reports_penetration_and_angle() -> None:
    wheel = np.array([[-0.02, 0.98], [0.00, 1.02], [0.02, 0.98]], dtype=float)
    angles = np.column_stack((wheel[:, 0], np.zeros(3)))
    rail = np.array([[-0.03, 1.00], [0.03, 1.00]], dtype=float)

    contact = single_point_contact_geometry(wheel, angles, rail)

    assert contact.has_contact
    assert np.allclose(contact.rail_point, [0.0, 1.0])
    assert np.allclose(contact.wheel_point, [0.0, 1.02])
    assert np.isclose(contact.vertical_gap, -0.02)
    assert np.isclose(contact.vertical_penetration, 0.02)
    assert np.isclose(contact.normal_penetration, 0.02)
    assert np.isclose(contact.contact_angle, 0.0)


def test_single_point_contact_converts_vertical_to_normal_penetration() -> None:
    angle = np.deg2rad(20.0)
    wheel = np.array([[0.0, 1.01]], dtype=float)
    angles = np.array([[0.0, angle]], dtype=float)
    rail = np.array([[-0.02, 1.00], [0.02, 1.00]], dtype=float)

    contact = single_point_contact_geometry(wheel, angles, rail)

    assert contact.has_contact
    assert np.isclose(contact.vertical_penetration, 0.01)
    assert np.isclose(contact.normal_penetration, 0.01 / np.cos(angle))
    assert np.isclose(contact.contact_angle, angle)


def test_single_point_contact_no_overlap_returns_no_contact() -> None:
    wheel = np.array([[0.20, 1.01]], dtype=float)
    angles = np.array([[0.20, 0.0]], dtype=float)
    rail = np.array([[-0.02, 1.00], [0.02, 1.00]], dtype=float)

    contact = single_point_contact_geometry(wheel, angles, rail)

    assert not contact.has_contact
    assert contact.vertical_penetration == 0.0
    assert contact.normal_penetration == 0.0
    assert np.isinf(contact.vertical_gap)


def test_single_point_contact_real_wheel_profile_smoke() -> None:
    paths = ProjectPaths.from_repo_root()
    wheel = build_wheel_profiles(paths.wheel_profile_dir, r0=0.430)
    rail = np.array([[-0.08, 0.600], [0.08, 0.600]], dtype=float)

    no_contact = single_point_contact_geometry(
        wheel.right,
        wheel.contact_angle_right,
        rail,
        pose=WheelPose2D(vertical=0.0),
    )
    contact = single_point_contact_geometry(
        wheel.right,
        wheel.contact_angle_right,
        rail,
        pose=WheelPose2D(vertical=0.25),
    )

    assert not no_contact.has_contact
    assert contact.has_contact
    assert contact.vertical_penetration > 0.0
    assert contact.normal_penetration >= contact.vertical_penetration


def test_extreme_boundary_finds_multiple_positive_patches() -> None:
    elastic = np.array(
        [
            [-0.04, -0.001],
            [-0.03, 0.002],
            [-0.02, 0.006],
            [-0.01, 0.001],
            [0.00, -0.001],
            [0.01, 0.003],
            [0.02, 0.008],
            [0.03, 0.002],
            [0.04, -0.001],
        ],
        dtype=float,
    )

    boundaries = extreme_boundary(elastic, opt="max")

    assert boundaries.starts[:, 0].astype(int).tolist() == [1, 5]
    assert boundaries.ends[:, 0].astype(int).tolist() == [3, 7]
    assert boundaries.positive_extrema[:, 0].astype(int).tolist() == [2, 6]
    assert np.allclose(boundaries.positive_extrema[:, 2], [0.006, 0.008])


def test_quasi_elastic_correction_returns_corrected_patch_centers() -> None:
    elastic = np.array(
        [
            [-0.03, -0.001],
            [-0.02, 0.002],
            [-0.01, 0.006],
            [0.00, 0.001],
            [0.01, -0.001],
        ],
        dtype=float,
    )
    boundaries = extreme_boundary(elastic, opt="max")
    wheel_interp = np.column_stack((np.zeros(elastic.shape[0]), elastic[:, 0], 1.0 + elastic[:, 1]))
    rail_interp = np.column_stack((elastic[:, 0], np.ones(elastic.shape[0])))
    angles = np.linspace(0.0, 0.04, elastic.shape[0])
    wheel_lateral = np.linspace(-0.04, 0.04, elastic.shape[0])

    patches = quasi_elastic_correction(
        elastic,
        boundaries.positive_extrema,
        boundaries.starts,
        boundaries.ends,
        wheel_interp,
        rail_interp,
        angles,
        wheel_lateral,
        theta=2e-5,
    )

    assert len(patches) == 1
    patch = patches[0]
    assert patch.peak_index == 2
    assert boundaries.starts[0, 1] <= patch.corrected_wheel_point[1] <= boundaries.ends[0, 1]
    assert patch.corrected_vertical_penetration > 0.0
    assert np.isclose(
        patch.corrected_normal_penetration,
        patch.corrected_vertical_penetration / np.cos(patch.contact_angle),
    )
    assert np.isclose(patch.corrected_rail_point[1], 1.0)


def test_multi_point_contact_geometry_finds_two_contact_candidates() -> None:
    y = np.linspace(-0.05, 0.05, 101)
    penetration = (
        0.009 * np.exp(-((y + 0.025) / 0.006) ** 2)
        + 0.007 * np.exp(-((y - 0.022) / 0.007) ** 2)
        - 0.001
    )
    rail = np.column_stack((y, np.ones_like(y)))
    wheel = np.column_stack((y, 1.0 + penetration))
    angles = np.column_stack((y, np.zeros_like(y)))

    result = multi_point_contact_geometry(wheel, angles, rail)

    assert result.has_contact
    assert len(result.patches) == 2
    assert result.boundaries.starts.shape[0] == 2
    assert np.all([patch.corrected_vertical_penetration > 0.0 for patch in result.patches])
    assert result.patches[0].corrected_wheel_point[1] < result.patches[1].corrected_wheel_point[1]
