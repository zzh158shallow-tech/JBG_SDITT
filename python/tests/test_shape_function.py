from __future__ import annotations

import numpy as np
import pytest

from sditt.config import MATLAB_FULL_DEFAULT_CASE, ProjectPaths
from sditt.track import build_default_07009_face_modal_beam_shape_function_context
from sditt.vehicle import load_vehicle_parameters


def test_build_default_modal_beam_shape_function_context_low_cutoff() -> None:
    paths = ProjectPaths.from_repo_root()
    vehicle = load_vehicle_parameters(paths.default_vehicle_parameters, vlc=MATLAB_FULL_DEFAULT_CASE.vlc)

    context = build_default_07009_face_modal_beam_shape_function_context(
        modal_mat_path=paths.modal_turnout_mat,
        vehicle_parameters=vehicle,
        cut_freq=50.0,
    )

    inp_par = context.inp_par_fields()
    assert context.n_track == 5
    assert inp_par["ModeShape"]["qjbg"].shape == (4084, 5)
    assert inp_par["ModeShape"]["zjg_zgyg"].shape == (1756, 5)
    assert inp_par["N_Node"]["cxg"] == 390

    pos_z = inp_par["RailBeam"]["Pos_Z"]["R2_zjg"]
    vel_z = inp_par["RailBeam"]["Vel_Z"]["R2_zjg"]
    assert pos_z.shape == (1001, 2)
    assert vel_z.shape == pos_z.shape
    assert pos_z[0, 0] == pytest.approx(50.0)
    assert pos_z[0, 1] == pytest.approx(23.0e-3)
    assert pos_z[-1, 0] == pytest.approx(150.0)
    assert pos_z[-1, 1] == pytest.approx(0.0)


def test_cal_shape_function_beam188_fwv_matches_default_qjbg_segment_formula() -> None:
    paths = ProjectPaths.from_repo_root()
    vehicle = load_vehicle_parameters(paths.default_vehicle_parameters, vlc=MATLAB_FULL_DEFAULT_CASE.vlc)
    context = build_default_07009_face_modal_beam_shape_function_context(
        modal_mat_path=paths.modal_turnout_mat,
        vehicle_parameters=vehicle,
        cut_freq=50.0,
    )

    shape_function, rail_beam_motion = context.evaluate(54.0)

    qjbg_nodes = context.pos_node["qjbg"]
    qjbg_lengths = context.pos_node["qjbg_Length"].reshape(-1)
    mileage = 54.0
    m = int(np.searchsorted(qjbg_nodes[:, 1], mileage, side="right") - 1)
    sigma = (mileage - qjbg_nodes[m, 1]) / qjbg_lengths[m]

    expected_y = np.array(
        [
            1.0 - 3.0 * sigma**2 + 2.0 * sigma**3,
            (sigma - 2.0 * sigma**2 + sigma**3) * qjbg_lengths[m],
            sigma**2 * (3.0 - 2.0 * sigma),
            (sigma**3 - sigma**2) * qjbg_lengths[m],
        ]
    )
    expected_z = np.array(
        [
            1.0 - 3.0 * sigma**2 + 2.0 * sigma**3,
            -(sigma - 2.0 * sigma**2 + sigma**3) * qjbg_lengths[m],
            sigma**2 * (3.0 - 2.0 * sigma),
            -(sigma**3 - sigma**2) * qjbg_lengths[m],
        ]
    )

    assert shape_function["FF_qjbg_Y"] == pytest.approx(expected_y)
    assert shape_function["FF_qjbg_Z"] == pytest.approx(expected_z)
    assert shape_function["FF_qjbg_ROTY"] == pytest.approx([1.0 - sigma, sigma])
    assert shape_function["FF_qjbg_ROTZ"] == pytest.approx([1.0 - sigma, sigma])
    assert shape_function["FF_qjbg_Mapping_DynStatus_Y"].tolist() == [m * 4 + 1, m * 4 + 4, m * 4 + 5, m * 4 + 8]
    assert shape_function["FF_qjbg_Mapping_DynStatus_Z"].tolist() == [m * 4 + 2, m * 4 + 3, m * 4 + 6, m * 4 + 7]
    assert shape_function["FF_qjbg_Mapping_DynStatus_ROTY"].tolist() == [m * 4 + 3, m * 4 + 7]
    assert shape_function["FF_qjbg_Mapping_DynStatus_ROTZ"].tolist() == [m * 4 + 4, m * 4 + 8]

    assert shape_function["RF_cxg_Y"].size == 0
    assert shape_function["RF_cxg_Mapping_DynStatus_Y"].size == 0

    pos_z_table = context.rail_beam.pos_z["R2_zjg"]
    vel_z_table = context.rail_beam.vel_z["R2_zjg"]
    assert rail_beam_motion["Pos_Z"]["R2"][0, 0] == pytest.approx(np.interp(54.0, pos_z_table[:, 0], pos_z_table[:, 1]))
    assert rail_beam_motion["Vel_Z"]["R2"][0, 0] == pytest.approx(np.interp(54.0, vel_z_table[:, 0], vel_z_table[:, 1]))
    assert np.isnan(rail_beam_motion["Pos_Z"]["R2"][2, 0])
    assert np.isnan(rail_beam_motion["Vel_Z"]["R2"][2, 0])
