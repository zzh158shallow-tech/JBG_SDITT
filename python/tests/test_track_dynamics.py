from __future__ import annotations

import numpy as np

from sditt.track import rail_dyn_modal_ft


def test_rail_dyn_modal_ft_recovers_node_status_from_modal_coordinates() -> None:
    inp = {
        "N_track": 2,
        "Nw": 1,
        "N_ConPatch": 1,
        "Type_Rail": ["R1"],
        "Exp_WS": ["WS1"],
        "N_Node": {"R1": 2},
        "ModeShape": {
            "R1": np.array(
                [
                    [1.0, 0.0],
                    [0.0, 1.0],
                    [2.0, 0.0],
                    [0.0, 2.0],
                    [3.0, 0.0],
                    [0.0, 3.0],
                    [4.0, 0.0],
                    [0.0, 4.0],
                ]
            )
        },
    }
    shape_function = {
        "WS1_R1_Y": np.array([0.25, 0.75]),
        "WS1_R1_Z": np.array([0.4, 0.6]),
        "WS1_R1_ROTY": np.array([0.3, 0.7]),
        "WS1_R1_ROTZ": np.array([0.2, 0.8]),
        "WS1_R1_Mapping_DynStatus_Y": np.array([1, 5]),
        "WS1_R1_Mapping_DynStatus_Z": np.array([2, 6]),
        "WS1_R1_Mapping_DynStatus_ROTY": np.array([3, 7]),
        "WS1_R1_Mapping_DynStatus_ROTZ": np.array([4, 8]),
    }
    zwy = _matlab_state([10.0, 20.0])
    zsd = _matlab_state([1.0, 2.0])
    zjsd = _matlab_state([0.1, 0.2])

    dis_rail, vel_rail, acc_rail, dyn_status = rail_dyn_modal_ft(inp, zwy, zsd, zjsd, shape_function)

    dyn_dis = inp["ModeShape"]["R1"] @ np.array([10.0, 20.0])
    dyn_vel = inp["ModeShape"]["R1"] @ np.array([1.0, 2.0])
    dyn_acc = inp["ModeShape"]["R1"] @ np.array([0.1, 0.2])
    expected_status = np.zeros((2, 6))
    expected_status[0, [1, 2, 4, 5]] = dyn_dis[:4]
    expected_status[1, [1, 2, 4, 5]] = dyn_dis[4:8]

    assert np.allclose(dyn_status["R1_Dis"], expected_status)
    assert np.allclose(dyn_status["R1_Vel"][0, [1, 2, 4, 5]], dyn_vel[:4])
    assert np.allclose(dyn_status["R1_Acc"][1, [1, 2, 4, 5]], dyn_acc[4:8])

    expected_contact_dis = np.zeros(6)
    expected_contact_dis[1] = np.array([0.25, 0.75]) @ dyn_dis[[0, 4]]
    expected_contact_dis[2] = np.array([0.4, 0.6]) @ dyn_dis[[1, 5]]
    expected_contact_dis[4] = np.array([0.3, 0.7]) @ dyn_dis[[2, 6]]
    expected_contact_dis[5] = np.array([0.2, 0.8]) @ dyn_dis[[3, 7]]

    assert np.allclose(dis_rail[0, :], expected_contact_dis)
    assert np.isclose(vel_rail[0, 1], np.array([0.25, 0.75]) @ dyn_vel[[0, 4]])
    assert np.isclose(acc_rail[0, 2], np.array([0.4, 0.6]) @ dyn_acc[[1, 5]])


def test_rail_dyn_modal_ft_skips_empty_shape_function_contact() -> None:
    inp = {
        "N_track": 1,
        "Nw": 1,
        "N_ConPatch": 1,
        "Type_Rail": ["R1"],
        "Exp_WS": ["WS1"],
        "N_Node": {"R1": 1},
        "ModeShape": {"R1": np.ones((4, 1))},
    }
    shape_function = {
        "WS1_R1_Y": np.array([]),
        "WS1_R1_Z": np.array([]),
        "WS1_R1_ROTY": np.array([]),
        "WS1_R1_ROTZ": np.array([]),
        "WS1_R1_Mapping_DynStatus_Y": np.array([]),
        "WS1_R1_Mapping_DynStatus_Z": np.array([]),
        "WS1_R1_Mapping_DynStatus_ROTY": np.array([]),
        "WS1_R1_Mapping_DynStatus_ROTZ": np.array([]),
    }

    dis_rail, vel_rail, acc_rail, dyn_status = rail_dyn_modal_ft(
        inp,
        np.array([2.0]),
        np.array([3.0]),
        np.array([4.0]),
        shape_function,
    )

    assert np.allclose(dis_rail, 0.0)
    assert np.allclose(vel_rail, 0.0)
    assert np.allclose(acc_rail, 0.0)
    assert np.allclose(dyn_status["R1_Dis"][0, [1, 2, 4, 5]], 2.0)


def _matlab_state(values: list[float]) -> np.ndarray:
    state = np.zeros((len(values), 4), dtype=float)
    state[:, 3] = values
    return state
