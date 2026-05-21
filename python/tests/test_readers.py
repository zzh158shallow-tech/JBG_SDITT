from __future__ import annotations

import numpy as np

from sditt.config import ProjectPaths
from sditt.io import inspect_raw_inputs, load_sditt_raw_inputs
from sditt.io.text import load_numeric_text
from sditt.profiles import (
    build_track_profiles,
    build_wheel_profiles,
    contact_tables,
    create_bezier_profile_data,
    discover_profile_files,
    interpolate_rail_profiles,
    load_profile_file,
    offset_profile_to_track,
    read_mileage_profile_file,
)
from sditt.simulation import assemble_system_matrices, build_default_modal_rw_system_matrices
from sditt.track import (
    build_flexible_turnout_modal_matrices,
    build_modal_track_matrices,
    load_modal_turnout_frequencies,
    summarize_modal_turnout_data,
)
from sditt.vehicle import build_vehicle_matrices_rw_230409, load_vehicle_parameters


def test_load_modal_turnout_mat_file() -> None:
    paths = ProjectPaths.from_repo_root()
    mat_file = summarize_modal_turnout_data(paths.modal_turnout_mat)

    assert mat_file.path.exists()
    assert "MATLAB 5.0 MAT-file" in mat_file.header
    assert mat_file.variables


def test_load_modal_turnout_frequencies_only() -> None:
    paths = ProjectPaths.from_repo_root()
    mode_freq = load_modal_turnout_frequencies(paths.modal_turnout_mat)

    assert mode_freq.shape[1] == 2
    assert np.all(mode_freq[:, 1] > 0)
    assert np.isclose(mode_freq[0, 1], 47.754)


def test_build_modal_track_matrices_formula() -> None:
    mode_freq_all = np.array(
        [
            [1, 10.0],
            [2, 20.0],
            [3, 30.0],
        ],
        dtype=float,
    )
    dr_normal = np.array([[0, 0.01], [30, 0.04]], dtype=float)
    dr_typical = np.array([[2, 0.10]], dtype=float)

    matrices = build_modal_track_matrices(
        mode_freq_all,
        cut_freq=20.0,
        dr_normal=dr_normal,
        dr_typical=dr_typical,
    )
    omega = 2 * np.pi * mode_freq_all[:2, 1]

    assert np.allclose(matrices.M_track, np.eye(2))
    assert np.allclose(np.diag(matrices.K_track), omega**2)
    assert np.allclose(np.diag(matrices.C_track), 2 * matrices.DR[:, 1] * omega)
    assert np.isclose(matrices.DR[1, 1], 0.10)


def test_build_flexible_turnout_modal_matrices_low_cutoff() -> None:
    paths = ProjectPaths.from_repo_root()
    matrices = build_flexible_turnout_modal_matrices(
        paths.modal_turnout_mat,
        cut_freq=50.0,
        matlab_dir=paths.matlab_dir,
    )
    omega = matrices.omega

    assert matrices.M_track.shape == (5, 5)
    assert np.allclose(matrices.M_track, np.eye(5))
    assert np.allclose(np.diag(matrices.K_track), omega**2)
    assert np.allclose(np.diag(matrices.C_track), 2 * matrices.DR[:, 1] * omega)
    assert np.isclose(matrices.mode_freq[0, 1], 47.754)


def test_load_vehicle_parameters_from_matlab_script() -> None:
    paths = ProjectPaths.from_repo_root()
    vehicle = load_vehicle_parameters(paths.default_vehicle_parameters)

    assert vehicle.unsupported_statements == ()
    assert vehicle.values["Mw"] == 1901.8
    assert vehicle.values["R0"] == 0.430
    assert np.isclose(vehicle.values["Omiga"], (350 / 3.6) / 0.430)
    assert vehicle.values["C_DPz"].shape == (2, 1)
    assert vehicle.values["K_STy_Table"].shape == (13, 2)
    assert np.all(np.diff(vehicle.values["K_STy_Table"][:, 0]) >= 0)


def test_build_vehicle_matrices_rw_230409() -> None:
    paths = ProjectPaths.from_repo_root()
    vehicle = load_vehicle_parameters(paths.default_vehicle_parameters)
    matrices = build_vehicle_matrices_rw_230409(vehicle)

    assert matrices.M_vehicle.shape == (51, 51)
    assert matrices.K_vehicle.shape == (51, 51)
    assert matrices.C_vehicle.shape == (51, 51)
    assert matrices.C_vehicle_linear.shape == (51, 51)

    assert np.isclose(matrices.M_vehicle[0, 0], vehicle.values["Mw"])
    assert np.isclose(matrices.M_vehicle[20, 20], vehicle.values["Mb"])
    assert np.isclose(matrices.M_vehicle[30, 30], vehicle.values["Mc"])
    assert np.isclose(matrices.M_vehicle[50, 50], 1e-8)

    assert np.allclose(matrices.K_vehicle, matrices.K_vehicle.T)
    assert np.allclose(matrices.C_vehicle, matrices.C_vehicle.T)
    assert np.isclose(matrices.K_vehicle[0, 0], 12152000.0)
    assert np.isclose(matrices.C_vehicle[35, 35], 19600.0)
    assert np.isclose(matrices.C_vehicle_linear[35, 35], 0.0)


def test_assemble_system_matrices_block_structure() -> None:
    M_track = np.eye(2)
    K_track = np.diag([10.0, 20.0])
    C_track = np.diag([1.0, 2.0])
    M_vehicle = np.diag([3.0, 4.0, 5.0])
    K_vehicle = np.diag([30.0, 40.0, 50.0])
    C_vehicle = np.diag([6.0, 7.0, 8.0])

    system = assemble_system_matrices(
        M_track,
        K_track,
        C_track,
        M_vehicle,
        K_vehicle,
        C_vehicle,
        nm_fw=1,
        n_wheels=1,
        n_rv=2,
    )

    assert system.layout.total_dof == 5
    assert system.layout.track == slice(0, 2)
    assert system.layout.flexible_wheel == slice(2, 3)
    assert system.layout.rigid_vehicle == slice(3, 5)
    assert np.allclose(system.Mxt[:2, :2], M_track)
    assert np.allclose(system.Kxt[2:, 2:], K_vehicle)
    assert np.allclose(system.Cxt[2:, 2:], C_vehicle)
    assert np.count_nonzero(system.Mxt[:2, 2:]) == 0
    assert np.count_nonzero(system.Kxt[2:, :2]) == 0
    assert np.count_nonzero(system.Cxt[:2, 2:]) == 0


def test_build_default_modal_rw_system_matrices_low_cutoff() -> None:
    system, track, vehicle = build_default_modal_rw_system_matrices(cut_freq=50.0)

    assert system.layout.n_track == track.M_track.shape[0] == 5
    assert system.layout.nm_fw == 0
    assert system.layout.n_wheels == 4
    assert system.layout.n_rv == 51
    assert system.Mxt.shape == (56, 56)
    assert np.allclose(system.Mxt[system.layout.track, system.layout.track], track.M_track)
    assert np.allclose(system.Kxt[system.layout.track, system.layout.track], track.K_track)
    assert np.allclose(system.Cxt[system.layout.track, system.layout.track], track.C_track)
    assert np.allclose(
        system.Mxt[system.layout.vehicle_block, system.layout.vehicle_block],
        vehicle.M_vehicle,
    )
    assert np.allclose(
        system.Kxt[system.layout.vehicle_block, system.layout.vehicle_block],
        vehicle.K_vehicle,
    )
    assert np.allclose(
        system.Cxt[system.layout.vehicle_block, system.layout.vehicle_block],
        vehicle.C_vehicle,
    )


def test_load_profile_text_file() -> None:
    paths = ProjectPaths.from_repo_root()
    files = discover_profile_files(paths.profile_dir)
    assert files

    profile = load_profile_file(files[0])
    assert profile.path.exists()
    assert profile.points.ndim == 2


def test_build_wheel_profiles_from_matlab_inputs() -> None:
    paths = ProjectPaths.from_repo_root()
    wheel = build_wheel_profiles(paths.wheel_profile_dir, r0=0.430)

    assert wheel.right.shape[1] == 2
    assert wheel.left.shape == wheel.right.shape
    assert wheel.contact_angle_right.shape[1] == 2
    assert wheel.contact_angle_left.shape == wheel.contact_angle_right.shape
    assert wheel.radius_right.shape == wheel.right.shape
    assert np.allclose(wheel.left[:, 0], np.sort(-wheel.right[:, 0]))
    assert np.isclose(np.max(wheel.right[:, 1]), 0.430 + 0.0280248, atol=1e-7)


def test_contact_lookup_tables_match_matlab_shapes() -> None:
    tables = contact_tables()

    assert tables.bgmn.shape == (44, 3)
    assert tables.bgc1.shape == (10, 5)
    assert tables.bgc2.shape == (10, 5)
    assert np.isclose(tables.bgmn[-1, 0], np.pi / 2)
    assert np.isclose(tables.bgc2[0, 0], 0.1)


def test_read_mileage_and_interpolate_rail_profiles() -> None:
    paths = ProjectPaths.from_repo_root()
    profile_dir = paths.profile_dir / "07(009)-zjg-20200418"
    mileage_entries = read_mileage_profile_file(
        profile_dir / "Mileage_prr.txt",
        profile_base_dir=profile_dir,
    )

    profiles = interpolate_rail_profiles(
        mileage_entries[10].mileage + 0.03,
        [0.0, 1.0, 2.0, 3.0],
        mileage_entries,
        same_front_profile=True,
        same_rear_profile=False,
        num_interp=128,
    )
    ff = profiles.by_station["FF"]

    assert len(mileage_entries) > 20
    assert ff.profile.shape == (128, 2)
    assert ff.front_profile.shape[1] == 2
    assert ff.rear_profile.shape[1] == 2
    assert ff.radius.shape == (128, 2)
    assert ff.profile_num is not None


def test_bezier_profile_data_and_track_coordinate_offsets() -> None:
    paths = ProjectPaths.from_repo_root()
    profile_dir = paths.profile_dir / "07(009)-zjg-20200418"
    mileage_entries = read_mileage_profile_file(
        profile_dir / "Mileage_prr.txt",
        profile_base_dir=profile_dir,
    )[:5]
    divisions = np.array([[mileage_entries[0].mileage, mileage_entries[-1].mileage]], dtype=float)
    bezier = create_bezier_profile_data(mileage_entries, divisions, num_interp=64)

    rail = interpolate_rail_profiles(
        mileage_entries[2].mileage,
        {"FF": 0.0, "FR": 0.0, "RF": 0.0, "RR": 0.0},
        mileage_entries,
        same_front_profile=True,
        same_rear_profile=True,
        bezier=bezier,
        num_interp=64,
    ).by_station["FF"]
    offset = offset_profile_to_track(rail, wheel_side="R", ori_prr=0.0, dis_rail_y=0.01, dis_rail_z=0.02)
    track = build_track_profiles({"R1": offset}, left_dummy_rails=(), right_dummy_rails=("R1",))

    assert bezier.y.shape == (64, 5)
    assert rail.profile.shape == (64, 2)
    assert np.allclose(offset.profile[:, 0], rail.profile[:, 0] + 1.435 / 2 + 0.01)
    assert np.allclose(offset.profile[:, 1], rail.profile[:, 1] + 0.62)
    assert np.array_equal(track.profile["R"], offset.profile)


def test_load_mileage_text_file() -> None:
    paths = ProjectPaths.from_repo_root()
    data = load_numeric_text(paths.matlab_dir / "Mileage_prr_2.txt")

    assert data.data.ndim == 2
    assert data.data.size > 0


def test_inspect_raw_input_manifest() -> None:
    manifest = inspect_raw_inputs()

    assert manifest.modal_frequencies_shape == (6204, 2)
    assert manifest.vehicle_parameter_count >= 70
    assert manifest.wheel_profile_files
    assert manifest.rail_profile_files
    assert manifest.numeric_text_files
    assert not any("mileage" in str(path).lower() for path in manifest.profile_files)


def test_load_sditt_raw_input_samples() -> None:
    raw = load_sditt_raw_inputs(max_profiles_per_kind=2)

    assert raw.modal_frequencies.shape == (6204, 2)
    assert raw.vehicle_parameters.unsupported_statements == ()
    assert len(raw.wheel_profiles) == 2
    assert len(raw.rail_profiles) == 2
    assert raw.numeric_text_tables
    assert all(profile.points.ndim == 2 for profile in raw.wheel_profiles.values())
    assert all(profile.points.ndim == 2 for profile in raw.rail_profiles.values())
