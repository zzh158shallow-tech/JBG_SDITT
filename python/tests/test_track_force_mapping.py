from __future__ import annotations

import numpy as np

from sditt.track import wr_force_modal_ft


def test_wr_force_modal_ft_maps_legacy_face_contacts_to_modal_coordinates() -> None:
    inp = {
        "N_track": 2,
        "Nw": 1,
        "N_ConPatch": 2,
        "VehicleDir": "Face",
        "Type_Rail": ["R1", "R2"],
        "Exp_WS": ["WS1"],
        "DOF_Node": {"R1": 4, "R2": 4},
        "ModeShape": {
            "R1": np.array([[1.0, 0.0], [0.0, 1.0], [2.0, 0.0], [0.0, 2.0]]),
            "R2": np.array([[0.5, 0.0], [0.0, 0.5], [1.0, 0.0], [0.0, 1.0]]),
        },
    }
    shape_function = {
        "WS1_R1_Y": np.array([0.25, 0.75]),
        "WS1_R1_Z": np.array([0.4, 0.6]),
        "WS1_R1_Mapping_DynStatus_Y": np.array([1, 3]),
        "WS1_R1_Mapping_DynStatus_Z": np.array([2, 4]),
        "WS1_R2_Y": np.array([0.2, 0.8]),
        "WS1_R2_Z": np.array([0.3, 0.7]),
        "WS1_R2_Mapping_DynStatus_Y": np.array([1, 3]),
        "WS1_R2_Mapping_DynStatus_Z": np.array([2, 4]),
    }
    pxt = np.zeros((2, 1))
    pjcc = np.array([[100.0], [200.0]])
    pjch = np.array([[10.0], [20.0]])
    prhxf = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])

    out, pxt_track = wr_force_modal_ft(inp, 1, pxt, pjcc, pjch, prhxf, {"FF": {}}, shape_function)

    expected_r1 = np.array([-(10.0 + 2.0) * 0.25, -(100.0 + 3.0) * 0.4, -(10.0 + 2.0) * 0.75, -(100.0 + 3.0) * 0.6])
    expected_r2 = np.array([-(20.0 + 5.0) * 0.2, -(200.0 + 6.0) * 0.3, -(20.0 + 5.0) * 0.8, -(200.0 + 6.0) * 0.7])
    expected_modal = inp["ModeShape"]["R1"].T @ expected_r1 + inp["ModeShape"]["R2"].T @ expected_r2

    assert np.allclose(pxt_track["R1"][:, 0], expected_r1)
    assert np.allclose(pxt_track["R2"][:, 0], expected_r2)
    assert np.allclose(out[:, 0], expected_modal)


def test_wr_force_modal_ft_maps_detailed_contacts_using_normal_force_patch_id() -> None:
    inp = {
        "N_track": 2,
        "Nw": 1,
        "N_ConPatch": 2,
        "VehicleDir": "Face",
        "Type_Rail": ["R1", "R2"],
        "Type_Side": ["L", "R"],
        "Exp_WS": ["WS1"],
        "DOF_Node": {"R1": 3, "R2": 3},
        "ModeShape": {
            "R1": np.eye(3, 2),
            "R2": np.array([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]]),
        },
    }
    shape_function = {
        "WS1_R2_Y": np.array([0.25, 0.75]),
        "WS1_R2_Z": np.array([1.0]),
        "WS1_R2_Mapping_DynStatus_Y": np.array([1, 3]),
        "WS1_R2_Mapping_DynStatus_Z": np.array([2]),
    }
    con_ws = {
        "FF": {"Normal_Force": True},
        "WS1": {
            "Normal_Force": {
                "L": np.array([[0.0, 20.0, 100.0, 2.0]]),
                "R": np.zeros((0, 4)),
            },
            "Prhxf_T": {
                "L": np.array([[3.0, 4.0, 5.0]]),
                "R": np.zeros((0, 3)),
            },
        },
    }

    out, pxt_track = wr_force_modal_ft(
        inp,
        4,
        np.zeros((2, 1)),
        np.zeros((2, 1)),
        np.zeros((2, 1)),
        np.zeros((2, 3)),
        con_ws,
        shape_function,
    )

    expected_r2 = np.array([-(20.0 + 4.0) * 0.25, -(100.0 + 5.0), -(20.0 + 4.0) * 0.75])
    expected_modal = inp["ModeShape"]["R2"].T @ expected_r2

    assert np.allclose(pxt_track["R1"][:, 0], 0.0)
    assert np.allclose(pxt_track["R2"][:, 0], expected_r2)
    assert np.allclose(out[:, 0], expected_modal)
