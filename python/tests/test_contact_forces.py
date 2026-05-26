from __future__ import annotations

import numpy as np
import pytest

from sditt.contact import (
    add_hu_guo_stripes_damping,
    contact_to_track_matrix,
    hertz_normal_force,
    hu_guo_normal_damping_force,
    kalker_linear_saturated_creep_force,
    normal_damping_window,
    stripes_normal_force,
)


def test_hertz_normal_force_matches_matlab_formula() -> None:
    normal_penetration = 2.5e-5
    elastic_permeability = 1.0e-8

    force = hertz_normal_force(normal_penetration, elastic_permeability)

    assert np.isclose(force, (normal_penetration / elastic_permeability) ** 1.5)


def test_hertz_normal_force_clamps_open_contact_to_zero() -> None:
    elastic_permeability = 1.0e-8

    force = hertz_normal_force(np.array([-1.0e-6, 0.0, 4.0e-6]), elastic_permeability)

    assert np.allclose(force[:2], 0.0)
    assert force[2] > 0.0


def test_hertz_normal_force_accepts_vector_permeability() -> None:
    penetration = np.array([1.0e-6, 8.0e-6])
    permeability = np.array([1.0e-8, 2.0e-8])

    force = hertz_normal_force(penetration, permeability)

    assert np.allclose(force, (penetration / permeability) ** 1.5)


def test_hertz_normal_force_can_apply_debug_floor() -> None:
    force = hertz_normal_force(0.0, 1.0e-8, minimum_force=1.0e-3)

    assert force == 1.0e-3


def test_hertz_normal_force_rejects_invalid_parameters() -> None:
    with pytest.raises(ValueError, match="elastic_permeability"):
        hertz_normal_force(1.0e-6, 0.0)

    with pytest.raises(ValueError, match="minimum_force"):
        hertz_normal_force(1.0e-6, 1.0e-8, minimum_force=-1.0)


def test_contact_to_track_matrix_matches_identity_at_zero_angles() -> None:
    assert np.allclose(contact_to_track_matrix(0.0, 0.0, 0.0), np.eye(3))


def test_stripes_normal_force_returns_elastic_force_for_simple_patch() -> None:
    y = np.linspace(-0.02, 0.02, 81)
    wheel_interp = np.column_stack((np.zeros_like(y), y, np.full_like(y, 0.43)))
    rail_interp = np.column_stack((y, np.full_like(y, 0.43)))
    wheel_radius_profile = np.array([[-0.03, 0.43], [0.03, 0.43]], dtype=float)
    rail_radius_profile = np.array([[-0.03, 1.0], [0.03, 1.0]], dtype=float)
    contact_wheel_track = np.array([[0.0, 0.0, 0.43]], dtype=float)
    contact_wheel_local = np.array([[0.0, 0.0, 0.43, 0.0, 1.0e-5, 0.0]], dtype=float)
    contact_rail_track = np.array([[0.0, 0.43]], dtype=float)
    penetration_peaks = np.array([[40.0, 0.0, 1.0e-5, 0.0]], dtype=float)

    result = stripes_normal_force(
        wheel_radius_profile,
        rail_radius_profile,
        contact_wheel_track,
        contact_wheel_local,
        contact_rail_track,
        wheel_interp,
        rail_interp,
        wheel_lateral=0.0,
        wheel_to_track=np.eye(3),
        contact_to_track=np.eye(3),
        penetration_peaks=penetration_peaks,
        m=np.array([2.0]),
        n=np.array([0.5]),
        con_a=np.array([1.0]),
        con_b=np.array([2.0]),
        con_r=np.array([0.8]),
        elastic_modulus=2.06e11,
        poisson_ratio=0.3,
        stripe_count=11,
        correction="AB",
    )

    assert result.normal_force.shape == (1, 6)
    assert result.normal_force[0, 4] > 0.0
    assert result.area[0] > 0.0
    assert len(result.patches) == 1
    assert result.patches[0].stripes.shape == (11, 3)
    assert np.isclose(result.patches[0].normal_force, result.normal_force[0, 4])
    assert np.all(result.patches[0].stripes[:, 0] > 0.0)
    assert np.all(result.patches[0].stripes[:, 1] > 0.0)
    assert np.allclose(result.patches[0].stripes[:, 2], result.patches[0].stripes[:, 0] * result.patches[0].stripes[:, 1])


def test_stripes_normal_force_uses_ab_epsilon_correction() -> None:
    y = np.linspace(-0.01, 0.01, 21)
    result = stripes_normal_force(
        np.array([[-0.02, 0.43], [0.02, 0.43]], dtype=float),
        np.array([[-0.02, 1.0], [0.02, 1.0]], dtype=float),
        np.array([[0.0, 0.0, 0.43]], dtype=float),
        np.array([[0.0, 0.0, 0.43, 0.0, 1.0e-5, 0.0]], dtype=float),
        np.array([[0.0, 0.43]], dtype=float),
        np.column_stack((np.zeros_like(y), y, np.full_like(y, 0.43))),
        np.column_stack((y, np.full_like(y, 0.43))),
        wheel_lateral=0.0,
        wheel_to_track=np.eye(3),
        contact_to_track=np.eye(3),
        penetration_peaks=np.array([[10.0, 0.0, 1.0e-5, 0.0]], dtype=float),
        m=np.array([2.0]),
        n=np.array([0.5]),
        con_a=np.array([1.0]),
        con_b=np.array([2.0]),
        con_r=np.array([0.8]),
        elastic_modulus=2.06e11,
        poisson_ratio=0.3,
        stripe_count=5,
        correction="AB",
    )

    assert np.allclose(result.epsilon, (0.5**2) / 0.8 / (1.0 + (0.5 / 2.0) ** 2))


def test_hu_guo_normal_damping_force_matches_matlab_formula() -> None:
    stiffness = np.array([1.0e8, 2.0e8])
    penetration = np.array([1.0e-5, 2.0e-5])
    rel_vel_ratio = 0.25
    restitution = 0.6

    damping = hu_guo_normal_damping_force(stiffness, penetration, rel_vel_ratio, restitution)

    expected = stiffness * (3.0 * (1.0 - restitution) / (2.0 * restitution) * rel_vel_ratio) * penetration
    assert np.allclose(damping, expected)


def test_add_hu_guo_stripes_damping_sets_elastic_damping_and_total_columns() -> None:
    y = np.linspace(-0.02, 0.02, 81)
    elastic = stripes_normal_force(
        np.array([[-0.03, 0.43], [0.03, 0.43]], dtype=float),
        np.array([[-0.03, 1.0], [0.03, 1.0]], dtype=float),
        np.array([[0.0, 0.0, 0.43]], dtype=float),
        np.array([[0.0, 0.0, 0.43, 0.0, 1.0e-5, 0.0]], dtype=float),
        np.array([[0.0, 0.43]], dtype=float),
        np.column_stack((np.zeros_like(y), y, np.full_like(y, 0.43))),
        np.column_stack((y, np.full_like(y, 0.43))),
        wheel_lateral=0.0,
        wheel_to_track=np.eye(3),
        contact_to_track=np.eye(3),
        penetration_peaks=np.array([[40.0, 0.0, 1.0e-5, 0.0]], dtype=float),
        m=np.array([2.0]),
        n=np.array([0.5]),
        con_a=np.array([1.0]),
        con_b=np.array([2.0]),
        con_r=np.array([0.8]),
        elastic_modulus=2.06e11,
        poisson_ratio=0.3,
        stripe_count=11,
        correction="AB",
    )

    damped = add_hu_guo_stripes_damping(
        elastic,
        relative_velocity_ratio=0.2,
        restitution_coefficient=0.5,
        window=0.75,
    )

    expected_damping = np.sum(
        elastic.patches[0].stripes[:, 0]
        * (3.0 * (1.0 - 0.5) / (2.0 * 0.5) * 0.2)
        * elastic.patches[0].stripes[:, 1]
    ) * 0.75
    assert np.isclose(damped.normal_force[0, 4], elastic.normal_force[0, 4])
    assert np.isclose(damped.normal_force[0, 5], expected_damping)
    assert np.isclose(damped.normal_force[0, 0], damped.normal_force[0, 4] + damped.normal_force[0, 5])


def test_add_hu_guo_stripes_damping_clips_negative_damping_to_elastic_force() -> None:
    y = np.linspace(-0.02, 0.02, 81)
    elastic = stripes_normal_force(
        np.array([[-0.03, 0.43], [0.03, 0.43]], dtype=float),
        np.array([[-0.03, 1.0], [0.03, 1.0]], dtype=float),
        np.array([[0.0, 0.0, 0.43]], dtype=float),
        np.array([[0.0, 0.0, 0.43, 0.0, 1.0e-5, 0.0]], dtype=float),
        np.array([[0.0, 0.43]], dtype=float),
        np.column_stack((np.zeros_like(y), y, np.full_like(y, 0.43))),
        np.column_stack((y, np.full_like(y, 0.43))),
        wheel_lateral=0.0,
        wheel_to_track=np.eye(3),
        contact_to_track=np.eye(3),
        penetration_peaks=np.array([[40.0, 0.0, 1.0e-5, 0.0]], dtype=float),
        m=np.array([2.0]),
        n=np.array([0.5]),
        con_a=np.array([1.0]),
        con_b=np.array([2.0]),
        con_r=np.array([0.8]),
        elastic_modulus=2.06e11,
        poisson_ratio=0.3,
        stripe_count=11,
        correction="AB",
    )

    damped = add_hu_guo_stripes_damping(
        elastic,
        relative_velocity_ratio=-10.0,
        restitution_coefficient=0.5,
    )

    assert np.isclose(damped.normal_force[0, 5], -elastic.normal_force[0, 4])
    assert np.isclose(damped.normal_force[0, 0], 0.0)
    assert len(damped.damping_clip_diagnostics) == 1
    diagnostic = damped.damping_clip_diagnostics[0]
    assert diagnostic.patch_index == 0
    assert diagnostic.raw_damping_force < -diagnostic.elastic_force
    assert np.isclose(diagnostic.clipped_damping_force, -diagnostic.elastic_force)


def test_normal_damping_window_matches_face_and_trail_tables() -> None:
    assert normal_damping_window(20.0, "Face") == 0.0
    assert normal_damping_window(100.0, "Face") == 1.0
    assert np.isclose(normal_damping_window(34.25, "Face"), 0.5)

    assert normal_damping_window(100.0, "Trail") == 1.0
    assert normal_damping_window(200.0, "Trail") == 0.0
    assert np.isclose(normal_damping_window(142.5, "Trail"), 0.5)


def test_kalker_linear_saturated_creep_force_matches_linear_region() -> None:
    result = kalker_linear_saturated_creep_force(
        normal_force=np.array([1.0e5]),
        creepage=np.array([[1.0e-5, 2.0e-5, 3.0e-4]]),
        rolling_radius_sum=np.array([0.5]),
        wheel_rolling_radius=np.array([0.43]),
        m=np.array([2.0]),
        n=np.array([0.5]),
        elastic_modulus=2.14e11,
        poisson_ratio=0.3,
        friction_coefficient=0.4,
        vehicle_speed=80.0,
        contact_to_track=np.eye(3),
    )

    expected_x = -result.creep_stiffness[0, 0] * 1.0e-5
    expected_y = -result.creep_stiffness[0, 1] * 2.0e-5 - result.creep_stiffness[0, 2] * 3.0e-4
    expected_mz = result.creep_stiffness[0, 2] * 2.0e-5 - result.creep_stiffness[0, 3] * 3.0e-4
    linear_norm = np.hypot(expected_x, expected_y)
    limit = 0.4 * 1.0e5
    temp = linear_norm / limit
    expected_norm = limit * (temp - temp**2 / 3.0 + temp**3 / 27.0)
    expected_scale = expected_norm / linear_norm

    assert result.semi_axis_a[0] > 0.0
    assert result.semi_axis_b[0] > 0.0
    assert np.all(result.kalker_coefficients[0] > 0.0)
    assert np.allclose(result.linear_force[0], [expected_x, expected_y, expected_mz])
    assert np.isclose(result.saturation_scale[0], expected_scale)
    assert np.allclose(result.saturated_force[0], result.linear_force[0] * expected_scale)
    assert np.allclose(result.force_track[0, :3], [result.saturated_force[0, 0], result.saturated_force[0, 1], 0.0])
    assert np.allclose(result.force_track[0, 3:], [0.0, 0.0, result.saturated_force[0, 2]])


def test_kalker_linear_saturated_creep_force_limits_large_creep_to_friction() -> None:
    result = kalker_linear_saturated_creep_force(
        normal_force=np.array([1.0e5]),
        creepage=np.array([[0.5, 0.25, 0.0]]),
        rolling_radius_sum=np.array([0.5]),
        wheel_rolling_radius=np.array([0.43]),
        m=np.array([2.0]),
        n=np.array([0.5]),
        elastic_modulus=2.14e11,
        poisson_ratio=0.3,
        friction_coefficient=0.4,
        vehicle_speed=80.0,
    )

    assert result.linear_tangent_norm[0] > 3.0 * 0.4 * 1.0e5
    assert np.isclose(result.saturated_tangent_norm[0], 0.4 * 1.0e5)
    assert np.isclose(np.linalg.norm(result.saturated_force[0, :2]), 0.4 * 1.0e5)


def test_kalker_linear_saturated_creep_force_uses_vehicle_speed_sign_for_spin_coupling() -> None:
    positive = kalker_linear_saturated_creep_force(
        normal_force=np.array([1.0e5]),
        creepage=np.array([[0.0, 1.0e-5, 1.0e-5]]),
        rolling_radius_sum=np.array([0.5]),
        wheel_rolling_radius=np.array([0.43]),
        m=np.array([2.0]),
        n=np.array([0.5]),
        elastic_modulus=2.14e11,
        poisson_ratio=0.3,
        friction_coefficient=0.4,
        vehicle_speed=80.0,
    )
    negative = kalker_linear_saturated_creep_force(
        normal_force=np.array([1.0e5]),
        creepage=np.array([[0.0, 1.0e-5, 1.0e-5]]),
        rolling_radius_sum=np.array([0.5]),
        wheel_rolling_radius=np.array([0.43]),
        m=np.array([2.0]),
        n=np.array([0.5]),
        elastic_modulus=2.14e11,
        poisson_ratio=0.3,
        friction_coefficient=0.4,
        vehicle_speed=-80.0,
    )

    assert positive.linear_force[0, 1] < negative.linear_force[0, 1]
    assert positive.linear_force[0, 2] < negative.linear_force[0, 2]
