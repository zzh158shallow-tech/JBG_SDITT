from __future__ import annotations

from typing import Any

import numpy as np


def wr_force_modal_ft(
    inp_par: Any,
    xlcs: int,
    pxt: np.ndarray,
    pjcc: np.ndarray,
    pjch: np.ndarray,
    prhxf: np.ndarray,
    con_ws: Any,
    shape_function: Any,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Map wheel-rail forces into flexible-turnout modal coordinates.

    This reproduces MATLAB ``WR_Force_ModalFT.m``. Contact forces are first
    accumulated on each rail's physical dynamic-status DOFs through beam shape
    functions, then projected into the first ``InpPar.N_track`` modal
    coordinates by ``ModeShape.<rail>.T``.
    """

    n_track = int(_field(inp_par, "N_track"))
    n_wheels = int(_field(inp_par, "Nw"))
    n_contact_patch = int(_field(inp_par, "N_ConPatch"))
    type_rail = _cell_values(_field(inp_par, "Type_Rail"))
    exp_ws = _cell_values(_field(inp_par, "Exp_WS"))
    type_side = _cell_values(_field(inp_par, "Type_Side")) if _has_field(inp_par, "Type_Side") else ["L", "R"]

    out = np.array(pxt, dtype=float, copy=True)
    pjcc = _as_column(pjcc)
    pjch = _as_column(pjch)
    prhxf = np.asarray(prhxf, dtype=float)

    pxt_track = {
        rail: np.zeros((int(_field(_field(inp_par, "DOF_Node"), rail)), 1), dtype=float) for rail in type_rail
    }
    has_detailed_normal_force = _has_field(con_ws, "FF") and _has_field(_field(con_ws, "FF"), "Normal_Force")

    for wheel in range(n_wheels):
        if int(xlcs) <= 3 and not has_detailed_normal_force:
            range_i2 = _legacy_contact_range(_field(inp_par, "VehicleDir"))
            for rail_index in range_i2:
                contact = n_contact_patch * wheel + rail_index
                rail = type_rail[rail_index]
                _accumulate_physical_rail_force(
                    pxt_track[rail],
                    exp_ws[wheel],
                    rail,
                    normal_y=float(pjch[contact]),
                    normal_z=float(pjcc[contact]),
                    creep=prhxf[contact, :],
                    shape_function=shape_function,
                )
        else:
            con_str = _field(con_ws, exp_ws[wheel])
            for side in type_side:
                normal_force = np.asarray(_field(_field(con_str, "Normal_Force"), side), dtype=float)
                prhxf_temp = np.asarray(_field(_field(con_str, "Prhxf_T"), side), dtype=float)
                for k in range(normal_force.shape[0]):
                    rail_index = int(normal_force[k, 3]) - 1
                    rail = type_rail[rail_index]
                    _accumulate_physical_rail_force(
                        pxt_track[rail],
                        exp_ws[wheel],
                        rail,
                        normal_y=float(normal_force[k, 1]),
                        normal_z=float(normal_force[k, 2]),
                        creep=prhxf_temp[k, :],
                        shape_function=shape_function,
                    )

    for rail in type_rail:
        mode_shape = np.asarray(_field(_field(inp_par, "ModeShape"), rail), dtype=float)
        out[:n_track, 0] += mode_shape.T @ pxt_track[rail][:, 0]

    return out, pxt_track


def _accumulate_physical_rail_force(
    pxt_track_rail: np.ndarray,
    wheelset_name: str,
    rail: str,
    *,
    normal_y: float,
    normal_z: float,
    creep: np.ndarray,
    shape_function: Any,
) -> None:
    force_y = -(normal_y + float(creep[1]))
    force_z = -(normal_z + float(creep[2]))
    shape_y = np.asarray(_field(shape_function, f"{wheelset_name}_{rail}_Y"), dtype=float).reshape(-1)
    shape_z = np.asarray(_field(shape_function, f"{wheelset_name}_{rail}_Z"), dtype=float).reshape(-1)
    row_y = _matlab_rows_to_python(_field(shape_function, f"{wheelset_name}_{rail}_Mapping_DynStatus_Y"))
    row_z = _matlab_rows_to_python(_field(shape_function, f"{wheelset_name}_{rail}_Mapping_DynStatus_Z"))

    if row_y.size != shape_y.size:
        raise ValueError(f"{wheelset_name}_{rail}_Y and Mapping_DynStatus_Y lengths differ")
    if row_z.size != shape_z.size:
        raise ValueError(f"{wheelset_name}_{rail}_Z and Mapping_DynStatus_Z lengths differ")

    pxt_track_rail[row_y, 0] += force_y * shape_y
    pxt_track_rail[row_z, 0] += force_z * shape_z


def _legacy_contact_range(vehicle_dir: str) -> tuple[int, ...]:
    if vehicle_dir == "Face":
        return (0, 1)
    if vehicle_dir == "Trail":
        return (0, 3)
    raise ValueError(f"unknown VehicleDir {vehicle_dir!r}; expected 'Face' or 'Trail'")


def _field(obj: Any, name: str) -> Any:
    if isinstance(obj, dict):
        return obj[name]
    return getattr(obj, name)


def _has_field(obj: Any, name: str) -> bool:
    if isinstance(obj, dict):
        return name in obj
    return hasattr(obj, name)


def _cell_values(cell: Any) -> list[Any]:
    if isinstance(cell, np.ndarray):
        values = list(cell.reshape(-1, order="F"))
    else:
        values = list(cell)
    return [value.item() if isinstance(value, np.ndarray) and value.shape == () else value for value in values]


def _as_column(array: Any) -> np.ndarray:
    return np.asarray(array, dtype=float).reshape(-1)


def _matlab_rows_to_python(rows: Any) -> np.ndarray:
    row_array = np.asarray(rows, dtype=int).reshape(-1)
    if row_array.size == 0:
        return row_array
    if np.min(row_array) >= 1:
        return row_array - 1
    return row_array
