from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from sditt.contact.full_case import _creepage_inputs, _wheel_pose, _wheel_rate
from sditt.simulation.full_case import _progress_profile_selection


def test_rigid_wheelset_pose_and_rate_follow_matlab_dof_layout() -> None:
    inp_par = {"N_track": 5, "NM_FW": 0, "Nw": 4, "Vlc": 12.0}
    vehicle_parameters = {"R0": 0.43}
    displacement = np.zeros((25,), dtype=float)
    velocity = np.zeros((25,), dtype=float)
    base = 5
    displacement[base + 0] = 0.012
    displacement[base + 1] = -0.021
    displacement[base + 2] = 0.034
    displacement[base + 4] = -0.055
    velocity[base + 0] = 0.11
    velocity[base + 1] = -0.22
    velocity[base + 2] = 0.33
    velocity[base + 3] = -0.44
    velocity[base + 4] = 0.55

    pose = _wheel_pose(displacement, inp_par, vehicle_parameters, wheel_index=0)
    rate = _wheel_rate(velocity, inp_par, vehicle_parameters, wheel_index=0)

    assert pose.vertical == pytest.approx(0.012)
    assert pose.lateral == pytest.approx(-0.021)
    assert pose.roll == pytest.approx(0.034)
    assert pose.yaw == pytest.approx(-0.055)
    assert rate["base"] == 5
    assert np.allclose(rate["trans"], np.array([12.0, -0.22, 0.11]))
    assert rate["roll_rate"] == pytest.approx(0.33)
    assert rate["spin_rate_relative"] == pytest.approx(-0.44)
    assert rate["spin_rate"] == pytest.approx(-12.0 / 0.43 - 0.44)
    assert rate["yaw_rate"] == pytest.approx(0.55)


def test_creepage_inputs_include_rigid_wheel_kinematics_and_r2_rail_beam_velocity() -> None:
    inp_par = {
        "N_track": 5,
        "NM_FW": 0,
        "Nw": 4,
        "N_ConPatch": 4,
        "Exp_DummyRail": ["L1", "R1", "R2", "R3"],
    }
    pose = SimpleNamespace(roll=0.0, yaw=0.0)
    wheel_rate = {
        "trans": np.array([12.0, 0.5, -0.2], dtype=float),
        "roll_rate": 0.3,
        "spin_rate": -4.0,
        "yaw_rate": 0.2,
    }
    con_wheel_2 = np.array([[0.0, 0.2, 0.43]], dtype=float)
    transforms = np.array([np.eye(3)], dtype=float)
    patch_ids = np.array([3], dtype=int)
    rail_response = SimpleNamespace(
        vel_rail=np.zeros((16, 6), dtype=float),
        rail_beam_motion={"Vel_Z": {"R2": np.array([[1.5], [0.0], [0.0], [0.0]], dtype=float)}},
    )
    rail_response.vel_rail[2, :3] = np.array([0.7, -0.2, 0.4], dtype=float)

    vjd, vjd_r, vsdc, vjsdc = _creepage_inputs(
        rail_response,
        inp_par,
        {"R0": 0.43},
        wheel_rate,
        pose,
        con_wheel_2,
        transforms,
        patch_ids,
        wheel_index=0,
    )

    expected_vjd = np.array([[10.24, 0.371, -0.14]], dtype=float)
    expected_vjd_r = np.array([[0.7, -0.2, 1.9]], dtype=float)
    expected_vsdc = expected_vjd - expected_vjd_r
    expected_vjsdc = np.array([[0.3, -4.0, 0.2]], dtype=float)

    assert np.allclose(vjd, expected_vjd)
    assert np.allclose(vjd_r, expected_vjd_r)
    assert np.allclose(vsdc, expected_vsdc)
    assert np.allclose(vjsdc, expected_vjsdc)


def test_progress_profile_selection_prefers_front_wheelset_for_stable_live_display() -> None:
    contact = SimpleNamespace(
        track_profiles={"FF": object(), "FR": object(), "RF": object(), "RR": object()},
        con_ws={
            "FF": {"Normal_Force": {"L": np.zeros((0, 4)), "R": np.zeros((0, 4))}},
            "FR": {"Normal_Force": {"L": np.zeros((0, 4)), "R": np.array([[10.0, 0.0, 0.0, 2.0]])}},
            "RF": {"Normal_Force": {"L": np.zeros((0, 4)), "R": np.array([[20.0, 0.0, 0.0, 3.0]])}},
            "RR": {"Normal_Force": {"L": np.zeros((0, 4)), "R": np.array([[30.0, 0.0, 0.0, 4.0]])}},
        },
    )

    assert _progress_profile_selection(contact) == ("FF", 0)


def test_progress_profile_selection_fallback_keeps_actual_wheelset_index() -> None:
    contact = SimpleNamespace(
        track_profiles={"FR": object(), "RF": object(), "RR": object()},
        con_ws={
            "FR": {"Normal_Force": {"L": np.zeros((0, 4)), "R": np.array([[10.0, 0.0, 0.0, 2.0]])}},
            "RF": {"Normal_Force": {"L": np.zeros((0, 4)), "R": np.array([[30.0, 0.0, 0.0, 3.0]])}},
            "RR": {"Normal_Force": {"L": np.zeros((0, 4)), "R": np.array([[20.0, 0.0, 0.0, 4.0]])}},
        },
    )

    assert _progress_profile_selection(contact) == ("RF", 1)
