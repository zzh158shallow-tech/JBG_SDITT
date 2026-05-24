from __future__ import annotations

from typing import Any

import numpy as np


def wr_force_vehicle_sys_rotation_iii(
    inp_par: Any,
    par_vehicle: Any,
    par_track: Any,
    par_fw: Any,
    pxt: np.ndarray,
    zwy: np.ndarray,
    pjcc: np.ndarray,
    pjch: np.ndarray,
    prhxf: np.ndarray,
    con_ws: Any,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Map wheel-rail contact forces into the coupled vehicle-system vector.

    This reproduces MATLAB ``WR_Force_VehicleSys_RotationIII.m``. The system
    ordering is ``[track, flexible-wheel modal coordinates, rigid wheelsets]``.
    DOF position arrays copied from MATLAB are accepted as one-based indices
    and are converted when used to index NumPy arrays.
    """

    br = float(_field(par_track, "Br"))
    r0 = float(_field(par_vehicle, "R0"))
    n_track = int(_field(inp_par, "N_track"))
    nm_fw = int(_field(inp_par, "NM_FW"))
    n_wheels = int(_field(inp_par, "Nw"))
    pos_rv = n_track + nm_fw * n_wheels

    out = np.array(pxt, dtype=float, copy=True)
    zwy = np.asarray(zwy, dtype=float)
    pjcc = _as_column(pjcc)
    pjch = _as_column(pjch)
    prhxf = np.asarray(prhxf, dtype=float)
    q_temp: dict[str, np.ndarray] = {}

    if not _has_field(_field(con_ws, "FF"), "Normal_Force"):
        for wheel in range(n_wheels):
            pos_nm_fw = slice(n_track + nm_fw * wheel, n_track + nm_fw * (wheel + 1))
            if nm_fw > 0:
                out[pos_nm_fw, 0] += (float(_field(inp_par, "Vlc")) / r0) ** 2 * _as_column(
                    _field(par_fw, "Matrix_L")
                )

            for patch in range(int(_field(inp_par, "N_ConPatch"))):
                side = _cell_value(_field(inp_par, "Exp_DummyRail_WheelSide"), patch)
                wheelside = _wheelside_number(side)
                sign = (-1.0) ** wheelside
                contact = int(_field(inp_par, "N_ConPatch")) * wheel + patch
                _add_rigid_wheel_force(
                    out,
                    zwy,
                    pos_rv,
                    wheel,
                    wheelside,
                    br,
                    r0,
                    normal_y=float(pjch[contact]),
                    normal_z=float(pjcc[contact]),
                    creep=prhxf[contact, :],
                )

                if nm_fw > 0:
                    con_pos_y = 0.753 - 0.753 * 2.0 * (side == "L")
                    tread_mid = np.asarray(_field(_field(_field(_field(inp_par, "Pos_Node"), "Tread"), side), "Mid"))
                    m = _last_nonnegative_index(con_pos_y - tread_mid[:, 2])
                    lengths = np.diff(tread_mid[:, 2])
                    sigma = (con_pos_y - tread_mid[m, 2]) / lengths[m]
                    shape = np.array([1.0 - 3.0 * sigma**2 + 2.0 * sigma**3, sigma**2 * (3.0 - 2.0 * sigma)])
                    dof_name = f"Tread_{side}_Mid"
                    dof_pos = np.asarray(_field(_field(par_fw, "DOF_pos"), dof_name))
                    dof_load = np.concatenate((dof_pos[m, :], dof_pos[m + 1, :]))
                    dof_load = _matlab_dof_to_python(dof_load, _field(_field(inp_par, "ModeShape"), "FW"))
                    q_contact = np.array(
                        [prhxf[contact, 0], prhxf[contact, 1] + pjch[contact], prhxf[contact, 2] + pjcc[contact]],
                        dtype=float,
                    )
                    q_nodes = np.concatenate((q_contact * shape[0], q_contact * shape[1]))
                    q_temp[side] = q_nodes
                    mode_shape = np.asarray(_field(_field(inp_par, "ModeShape"), "FW"), dtype=float)
                    out[pos_nm_fw, 0] += mode_shape[dof_load, :].T @ q_nodes
        return out, q_temp

    for wheel in range(n_wheels):
        exp_ws = _cell_value(_field(inp_par, "Exp_WS"), wheel)
        con_str = _field(con_ws, exp_ws)
        pos_nm_fw = slice(n_track + nm_fw * wheel, n_track + nm_fw * (wheel + 1))
        if nm_fw > 0:
            out[pos_nm_fw, 0] += (float(_field(inp_par, "Vlc")) / r0) ** 2 * _as_column(_field(par_fw, "Matrix_L"))

        for wheelside in (1, 2):
            side = _cell_value(_field(inp_par, "Type_Side"), wheelside - 1)
            normal_force = np.asarray(_field(_field(con_str, "Normal_Force"), side), dtype=float)
            con_wheel_2 = np.asarray(_field(_field(con_str, "Con_wheel_2"), side), dtype=float)
            prhxf_temp = np.asarray(_field(_field(con_str, "Prhxf_T"), side), dtype=float)

            for k in range(normal_force.shape[0]):
                br_temp = abs(float(con_wheel_2[k, 1]))
                r_temp = float(con_wheel_2[k, 2])
                _add_rigid_wheel_force(
                    out,
                    zwy,
                    pos_rv,
                    wheel,
                    wheelside,
                    br_temp,
                    r_temp,
                    normal_y=float(normal_force[k, 1]),
                    normal_z=float(normal_force[k, 2]),
                    creep=prhxf_temp[k, :],
                )

                if nm_fw > 0:
                    q_contact = np.array(
                        [
                            prhxf_temp[k, 0],
                            prhxf_temp[k, 1] + normal_force[k, 1],
                            prhxf_temp[k, 2] + normal_force[k, 2],
                        ],
                        dtype=float,
                    )
                    shape_fw = _field(_field(_field(con_str, "ShapeFun_FW"), side), "XOY")
                    shape_z = _field(_field(_field(con_str, "ShapeFun_FW"), side), "Z")
                    xoy = np.asarray(_cell_value(shape_fw, k), dtype=float).reshape(-1)
                    z_shape = np.asarray(_cell_value(shape_z, k), dtype=float).reshape(-1)
                    q_column = np.concatenate((q_contact[0] * xoy, q_contact[1] * xoy, q_contact[2] * z_shape))
                    q_temp[side] = _store_column(q_temp.get(side), q_column, k, normal_force.shape[0])

                    dof_side = _field(_field(_field(con_str, "DOF_pos_FW"), side), "Node_Around_XOY")
                    dof_z_side = _field(_field(_field(con_str, "DOF_pos_FW"), side), "Node_Around_Z")
                    dof_xoy = np.asarray(_cell_value(dof_side, k))
                    dof_z = np.asarray(_cell_value(dof_z_side, k))
                    dof_pos = np.concatenate((dof_xoy[:, 0], dof_xoy[:, 1], dof_z[:, 2]))
                    mode_shape = np.asarray(_field(_field(inp_par, "ModeShape"), "FW"), dtype=float)
                    dof_pos = _matlab_dof_to_python(dof_pos, mode_shape)
                    out[pos_nm_fw, 0] += mode_shape[dof_pos, :].T @ q_column

    return out, q_temp


def _add_rigid_wheel_force(
    pxt: np.ndarray,
    zwy: np.ndarray,
    pos_rv: int,
    wheel: int,
    wheelside: int,
    br: float,
    radius: float,
    *,
    normal_y: float,
    normal_z: float,
    creep: np.ndarray,
) -> None:
    sign = (-1.0) ** wheelside
    base = pos_rv + 5 * wheel
    yaw_displacement = _state_value(zwy, base + 4)
    pxt[base + 0, 0] += normal_z
    pxt[base + 1, 0] += normal_y
    pxt[base + 2, 0] += normal_z * sign * br
    pxt[base + 2, 0] -= normal_y * radius
    pxt[base + 4, 0] -= normal_y * br * yaw_displacement * sign

    pxt[base + 0, 0] += creep[2]
    pxt[base + 1, 0] += creep[1]
    pxt[base + 2, 0] += creep[2] * sign * br
    pxt[base + 2, 0] -= creep[1] * radius
    pxt[base + 3, 0] += creep[0] * radius
    pxt[base + 3, 0] += creep[4]
    pxt[base + 4, 0] -= creep[0] * br * sign
    pxt[base + 4, 0] -= creep[1] * br * yaw_displacement * sign
    pxt[base + 4, 0] += creep[5]


def _field(obj: Any, name: str) -> Any:
    if isinstance(obj, dict):
        return obj[name]
    return getattr(obj, name)


def _has_field(obj: Any, name: str) -> bool:
    if isinstance(obj, dict):
        return name in obj
    return hasattr(obj, name)


def _cell_value(cell: Any, index: int) -> Any:
    if isinstance(cell, np.ndarray):
        value = cell.reshape(-1, order="F")[index]
    else:
        value = cell[index]
    if isinstance(value, np.ndarray) and value.shape == ():
        return value.item()
    return value


def _as_column(array: Any) -> np.ndarray:
    return np.asarray(array, dtype=float).reshape(-1)


def _state_value(state: Any, index: int) -> float:
    array = np.asarray(state, dtype=float)
    if array.ndim == 1:
        return float(array[index])
    if array.ndim == 2 and array.shape[1] > 3:
        return float(array[index, 3])
    if array.ndim == 2 and array.shape[1] == 1:
        return float(array[index, 0])
    raise ValueError("state arrays must be vectors, single-column arrays, or MATLAB-style arrays with column 4")


def _wheelside_number(side: str) -> int:
    if side == "L":
        return 1
    if side == "R":
        return 2
    raise ValueError(f"unknown wheel side {side!r}; expected 'L' or 'R'")


def _last_nonnegative_index(values: np.ndarray) -> int:
    matches = np.flatnonzero(np.asarray(values) >= 0.0)
    if matches.size == 0:
        raise ValueError("contact position is before the first tread node")
    return int(matches[-1])


def _matlab_dof_to_python(dof_pos: np.ndarray, mode_shape: np.ndarray) -> np.ndarray:
    dof = np.asarray(dof_pos, dtype=int).reshape(-1)
    if dof.size == 0:
        return dof
    if np.min(dof) >= 1 and np.max(dof) <= np.asarray(mode_shape).shape[0]:
        return dof - 1
    return dof


def _store_column(existing: np.ndarray | None, column: np.ndarray, index: int, width: int) -> np.ndarray:
    if existing is None:
        existing = np.zeros((column.size, width), dtype=float)
    if existing.shape[0] != column.size:
        raise ValueError("Q_temp column length changed within the same wheel side")
    existing[:, index] = column
    return existing
