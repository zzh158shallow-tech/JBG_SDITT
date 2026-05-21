from __future__ import annotations

import numpy as np

from sditt.config import ProjectPaths
from sditt.io.text import load_numeric_text
from sditt.profiles import discover_profile_files, load_profile_file
from sditt.track import summarize_modal_turnout_data
from sditt.vehicle import build_vehicle_matrices_rw_230409, load_vehicle_parameters


def test_load_modal_turnout_mat_file() -> None:
    paths = ProjectPaths.from_repo_root()
    mat_file = summarize_modal_turnout_data(paths.modal_turnout_mat)

    assert mat_file.path.exists()
    assert "MATLAB 5.0 MAT-file" in mat_file.header
    assert mat_file.variables


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


def test_load_profile_text_file() -> None:
    paths = ProjectPaths.from_repo_root()
    files = discover_profile_files(paths.profile_dir)
    assert files

    profile = load_profile_file(files[0])
    assert profile.path.exists()
    assert profile.points.ndim == 2


def test_load_mileage_text_file() -> None:
    paths = ProjectPaths.from_repo_root()
    data = load_numeric_text(paths.matlab_dir / "Mileage_prr_2.txt")

    assert data.data.ndim == 2
    assert data.data.size > 0
