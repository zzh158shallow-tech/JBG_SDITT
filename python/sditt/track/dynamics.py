from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ShapeFunctionEntry:
    shape_y: np.ndarray
    shape_z: np.ndarray
    shape_roty: np.ndarray
    shape_rotz: np.ndarray
    map_y: np.ndarray
    map_z: np.ndarray
    map_roty: np.ndarray
    map_rotz: np.ndarray


def rail_dyn_modal_ft(
    inp_par: Any,
    zwy: np.ndarray,
    zsd: np.ndarray,
    zjsd: np.ndarray,
    shape_function: Any,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    """Recover flexible-turnout rail dynamic response from modal coordinates.

    This reproduces MATLAB ``RailDyn_ModalFT.m`` for the 4-DOF rail-beam
    dynamic status used by the modal flexible turnout route. The recovered
    node status keeps MATLAB's six-column convention: columns 2, 3, 5, and 6
    contain ``Y``, ``Z``, ``ROTY``, and ``ROTZ`` respectively.
    """

    n_track = int(_field(inp_par, "N_track"))
    n_wheels = int(_field(inp_par, "Nw"))
    n_contact_patch = int(_field(inp_par, "N_ConPatch"))
    type_rail = _cell_values(_field(inp_par, "Type_Rail"))
    exp_ws = _cell_values(_field(inp_par, "Exp_WS"))

    dis_rail = np.zeros((n_wheels * n_contact_patch, 6), dtype=float)
    vel_rail = np.zeros_like(dis_rail)
    acc_rail = np.zeros_like(dis_rail)

    modal_state = np.column_stack(
        (
            _state_column(zwy, n_track),
            _state_column(zsd, n_track),
            _state_column(zjsd, n_track),
        )
    )
    dyn_track: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    dyn_status_rail: dict[str, np.ndarray] = {}

    dof_num = 4
    dof_columns = np.array([1, 2, 4, 5], dtype=int)
    shape_cache: dict[tuple[str, str], ShapeFunctionEntry] = {}

    for rail in type_rail:
        mode_shape = np.asarray(_field(_field(inp_par, "ModeShape"), rail), dtype=float)
        recovered = mode_shape @ modal_state
        dyn_dis = recovered[:, 0]
        dyn_vel = recovered[:, 1]
        dyn_acc = recovered[:, 2]
        dyn_track[rail] = (dyn_dis, dyn_vel, dyn_acc)

        node_count = int(_field(_field(inp_par, "N_Node"), rail))
        for suffix, vector in (("Dis", dyn_dis), ("Vel", dyn_vel), ("Acc", dyn_acc)):
            status = np.zeros((node_count, 6), dtype=float)
            for node in range(node_count):
                pos = slice(node * dof_num, (node + 1) * dof_num)
                status[node, dof_columns] = vector[pos]
            dyn_status_rail[f"{rail}_{suffix}"] = status

    for wheel in range(n_wheels):
        wheelset = exp_ws[wheel]
        for patch in range(n_contact_patch):
            rail = type_rail[patch]
            contact = n_contact_patch * wheel + patch
            shape_entry = _shape_function_entry(shape_function, wheelset, rail, shape_cache)
            if shape_entry.shape_y.size == 0:
                continue

            dyn_dis, dyn_vel, dyn_acc = dyn_track[rail]
            _fill_contact_response(dis_rail[contact, :], dyn_dis, shape_entry)
            _fill_contact_response(vel_rail[contact, :], dyn_vel, shape_entry)
            _fill_contact_response(acc_rail[contact, :], dyn_acc, shape_entry)

    return dis_rail, vel_rail, acc_rail, dyn_status_rail


def _fill_contact_response(
    row: np.ndarray,
    vector: np.ndarray,
    shape: ShapeFunctionEntry,
) -> None:
    row[1] = shape.shape_y @ vector[shape.map_y]
    row[2] = shape.shape_z @ vector[shape.map_z]
    row[4] = shape.shape_roty @ vector[shape.map_roty]
    row[5] = shape.shape_rotz @ vector[shape.map_rotz]


def _shape_function_entry(
    shape_function: Any,
    wheelset: str,
    rail: str,
    cache: dict[tuple[str, str], ShapeFunctionEntry],
) -> ShapeFunctionEntry:
    key = (wheelset, rail)
    if key not in cache:
        cache[key] = ShapeFunctionEntry(
            shape_y=np.asarray(_field(shape_function, f"{wheelset}_{rail}_Y"), dtype=float).reshape(-1),
            shape_z=np.asarray(_field(shape_function, f"{wheelset}_{rail}_Z"), dtype=float).reshape(-1),
            shape_roty=np.asarray(_field(shape_function, f"{wheelset}_{rail}_ROTY"), dtype=float).reshape(-1),
            shape_rotz=np.asarray(_field(shape_function, f"{wheelset}_{rail}_ROTZ"), dtype=float).reshape(-1),
            map_y=_matlab_rows_to_python(_field(shape_function, f"{wheelset}_{rail}_Mapping_DynStatus_Y")),
            map_z=_matlab_rows_to_python(_field(shape_function, f"{wheelset}_{rail}_Mapping_DynStatus_Z")),
            map_roty=_matlab_rows_to_python(_field(shape_function, f"{wheelset}_{rail}_Mapping_DynStatus_ROTY")),
            map_rotz=_matlab_rows_to_python(_field(shape_function, f"{wheelset}_{rail}_Mapping_DynStatus_ROTZ")),
        )
    return cache[key]


def _state_column(state: np.ndarray, n_track: int) -> np.ndarray:
    array = np.asarray(state, dtype=float)
    if array.ndim == 1:
        return array[:n_track]
    if array.ndim == 2 and array.shape[1] > 3:
        return array[:n_track, 3]
    if array.ndim == 2 and array.shape[1] == 1:
        return array[:n_track, 0]
    raise ValueError("state arrays must be vectors, single-column arrays, or MATLAB-style arrays with column 4")


def _field(obj: Any, name: str) -> Any:
    if isinstance(obj, dict):
        return obj[name]
    return getattr(obj, name)


def _cell_values(cell: Any) -> list[Any]:
    if isinstance(cell, np.ndarray):
        values = list(cell.reshape(-1, order="F"))
    else:
        values = list(cell)
    return [value.item() if isinstance(value, np.ndarray) and value.shape == () else value for value in values]


def _matlab_rows_to_python(rows: Any) -> np.ndarray:
    row_array = np.asarray(rows, dtype=int).reshape(-1)
    if row_array.size == 0:
        return row_array
    if np.min(row_array) >= 1:
        return row_array - 1
    return row_array
