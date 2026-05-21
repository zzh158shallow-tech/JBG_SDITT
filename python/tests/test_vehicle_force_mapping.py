from __future__ import annotations

import numpy as np

from sditt.vehicle import wr_force_vehicle_sys_rotation_iii


def test_rotation_iii_maps_legacy_contact_to_rigid_wheel_and_tread_nodes() -> None:
    inp = {
        "N_track": 2,
        "NM_FW": 2,
        "Nw": 1,
        "N_ConPatch": 1,
        "Vlc": 10.0,
        "Exp_DummyRail_WheelSide": ["L"],
        "Pos_Node": {"Tread": {"L": {"Mid": np.array([[0.0, 0.0, -1.0], [0.0, 0.0, -0.5]])}}},
        "ModeShape": {
            "FW": np.array(
                [
                    [1.0, 0.0],
                    [0.0, 1.0],
                    [2.0, 0.0],
                    [0.0, 2.0],
                    [3.0, 0.0],
                    [0.0, 3.0],
                ]
            )
        },
    }
    par_vehicle = {"R0": 0.5}
    par_track = {"Br": 0.7}
    par_fw = {
        "Matrix_L": np.array([2.0, 3.0]),
        "DOF_pos": {"Tread_L_Mid": np.array([[1, 2, 3], [4, 5, 6]])},
    }
    pxt = np.zeros((2 + 2 + 5, 1))
    zwy = np.zeros((pxt.shape[0], 4))
    zwy[2 + 2 + 4, 3] = 0.2
    pjcc = np.array([[100.0]])
    pjch = np.array([[20.0]])
    prhxf = np.array([[3.0, 4.0, 5.0, 0.0, 7.0, 8.0]])

    out, q_temp = wr_force_vehicle_sys_rotation_iii(
        inp,
        par_vehicle,
        par_track,
        par_fw,
        pxt,
        zwy,
        pjcc,
        pjch,
        prhxf,
        {"FF": {}},
    )

    sigma = (-0.753 - -1.0) / 0.5
    shape = np.array([1.0 - 3.0 * sigma**2 + 2.0 * sigma**3, sigma**2 * (3.0 - 2.0 * sigma)])
    q_contact = np.array([3.0, 24.0, 105.0])
    q_nodes = np.concatenate((q_contact * shape[0], q_contact * shape[1]))
    expected_fw = (10.0 / 0.5) ** 2 * np.array([2.0, 3.0]) + inp["ModeShape"]["FW"].T @ q_nodes

    assert np.allclose(q_temp["L"], q_nodes)
    assert np.allclose(out[2:4, 0], expected_fw)
    assert np.isclose(out[4, 0], 105.0)
    assert np.isclose(out[5, 0], 24.0)
    assert np.isclose(out[6, 0], -100.0 * 0.7 - 20.0 * 0.5 - 5.0 * 0.7 - 4.0 * 0.5)
    assert np.isclose(out[7, 0], 3.0 * 0.5 + 7.0)
    assert np.isclose(out[8, 0], 20.0 * 0.7 * 0.2 + 3.0 * 0.7 + 4.0 * 0.7 * 0.2 + 8.0)


def test_rotation_iii_maps_multi_contact_shape_functions_to_flexible_wheel() -> None:
    inp = {
        "N_track": 1,
        "NM_FW": 2,
        "Nw": 1,
        "N_ConPatch": 1,
        "Vlc": 0.0,
        "Exp_WS": ["WS1"],
        "Type_Side": ["L", "R"],
        "ModeShape": {"FW": np.arange(1.0, 25.0).reshape(12, 2)},
    }
    par_vehicle = {"R0": 0.5}
    par_track = {"Br": 0.7}
    par_fw = {"Matrix_L": np.zeros(2)}
    pxt = np.zeros((1 + 2 + 5, 1))
    zwy = np.zeros((pxt.shape[0], 4))
    con_ws = {
        "FF": {"Normal_Force": True},
        "WS1": {
            "Normal_Force": {"L": np.array([[0.0, 20.0, 100.0, 1.0]]), "R": np.zeros((0, 4))},
            "Con_wheel_2": {"L": np.array([[0.0, -0.72, 0.43]]), "R": np.zeros((0, 3))},
            "Prhxf_T": {"L": np.array([[3.0, 4.0, 5.0, 0.0, 7.0, 8.0]]), "R": np.zeros((0, 6))},
            "ShapeFun_FW": {
                "L": {"XOY": [np.array([0.1, 0.2, 0.3, 0.4])], "Z": [np.array([0.6, 0.4])]},
                "R": {"XOY": [], "Z": []},
            },
            "DOF_pos_FW": {
                "L": {
                    "Node_Around_XOY": [np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]])],
                    "Node_Around_Z": [np.array([[1, 2, 3], [4, 5, 6]])],
                },
                "R": {"Node_Around_XOY": [], "Node_Around_Z": []},
            },
        },
    }

    out, q_temp = wr_force_vehicle_sys_rotation_iii(
        inp,
        par_vehicle,
        par_track,
        par_fw,
        pxt,
        zwy,
        np.zeros((1, 1)),
        np.zeros((1, 1)),
        np.zeros((1, 6)),
        con_ws,
    )

    q_contact = np.array([3.0, 24.0, 105.0])
    xoy_shape = np.array([0.1, 0.2, 0.3, 0.4])
    z_shape = np.array([0.6, 0.4])
    q_column = np.concatenate((q_contact[0] * xoy_shape, q_contact[1] * xoy_shape, q_contact[2] * z_shape))
    dof_pos = np.array([1, 4, 7, 10, 2, 5, 8, 11, 3, 6]) - 1
    expected_fw = inp["ModeShape"]["FW"][dof_pos, :].T @ q_column

    assert np.allclose(q_temp["L"][:, 0], q_column)
    assert np.allclose(out[1:3, 0], expected_fw)
    assert np.isclose(out[3, 0], 105.0)
    assert np.isclose(out[4, 0], 24.0)
