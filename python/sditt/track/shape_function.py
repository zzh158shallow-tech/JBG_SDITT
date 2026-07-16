from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from sditt.config import MATLAB_FULL_DEFAULT_CASE, DefaultOperatingCase
from sditt.io.matlab import load_mat_variables
from sditt.vehicle import VehicleParameters


@dataclass(frozen=True)
class RailBeamData:
    """Static default ``RailBeam`` tables used by the MATLAB FT route."""

    pos_y: Mapping[str, np.ndarray] = field(default_factory=dict)
    vel_y: Mapping[str, np.ndarray] = field(default_factory=dict)
    win: Mapping[str, np.ndarray] = field(default_factory=dict)
    pos_z: Mapping[str, np.ndarray] = field(default_factory=dict)
    vel_z: Mapping[str, np.ndarray] = field(default_factory=dict)

    def to_inp_par(self) -> dict[str, dict[str, np.ndarray]]:
        return {
            "Pos_Y": dict(self.pos_y),
            "Vel_Y": dict(self.vel_y),
            "Win": dict(self.win),
            "Pos_Z": dict(self.pos_z),
            "Vel_Z": dict(self.vel_z),
        }


@dataclass(frozen=True)
class ModalBeamShapeFunctionContext:
    """Prepared modal-track inputs for ``Cal_ShapeFunction_Beam188_FWV``."""

    n_track: int
    wheelsets: tuple[str, ...]
    rail_types: tuple[str, ...]
    dummy_rails: tuple[str, ...]
    vlc: float
    pos_node: Mapping[str, np.ndarray]
    n_node: Mapping[str, int]
    dof_node: Mapping[str, int]
    mode_shape: Mapping[str, np.ndarray]
    rail_beam: RailBeamData
    distance_vehicle: Mapping[str, float]

    def inp_par_fields(self) -> dict[str, object]:
        return {
            "N_track": self.n_track,
            "Nw": len(self.wheelsets),
            "N_ConPatch": len(self.rail_types),
            "Type_Rail": self.rail_types,
            "Exp_WS": self.wheelsets,
            "Exp_DummyRail": self.dummy_rails,
            "Vlc": self.vlc,
            "Pos_Node": self.pos_node,
            "N_Node": self.n_node,
            "DOF_Node": self.dof_node,
            "ModeShape": self.mode_shape,
            "RailBeam": self.rail_beam.to_inp_par(),
        }

    def par_vehicle_fields(self) -> dict[str, np.ndarray]:
        return {
            "Distance_Vehicle": np.array(
                [self.distance_vehicle[wheelset] for wheelset in self.wheelsets],
                dtype=float,
            )
        }

    def evaluate(self, j1: float) -> tuple[dict[str, np.ndarray], dict[str, dict[str, np.ndarray]]]:
        return cal_shape_function_beam188_fwv(
            self.inp_par_fields(),
            self.par_vehicle_fields(),
            j1,
        )


def build_default_07009_face_modal_beam_shape_function_context(
    *,
    modal_mat_path: str | Path,
    vehicle_parameters: VehicleParameters | Mapping[str, Any],
    cut_freq: float,
    operating_case: DefaultOperatingCase = MATLAB_FULL_DEFAULT_CASE,
    choose_zjg_uneven: int = 0,
) -> ModalBeamShapeFunctionContext:
    """Prepare the default FT modal inputs needed by the beam shape-function step."""

    if operating_case.choose_turnout != "07(009)" or operating_case.vehicle_direction != "Face":
        raise ValueError("only the default 07(009) / Face beam-shape route is implemented")

    values = vehicle_parameters.values if isinstance(vehicle_parameters, VehicleParameters) else vehicle_parameters
    mode_data = _load_modal_shape_function_inputs(str(Path(modal_mat_path).resolve()), float(cut_freq))
    rail_beam = (
        RailBeamData()
        if operating_case.rail_layout == "interval"
        else build_default_07009_face_rail_beam(
            vlc=operating_case.vlc,
            choose_turnout=operating_case.choose_turnout,
            choose_zjg_uneven=choose_zjg_uneven,
        )
    )
    return ModalBeamShapeFunctionContext(
        n_track=mode_data["n_track"],
        wheelsets=tuple(operating_case.wheelsets),
        rail_types=tuple(operating_case.rail_types),
        dummy_rails=tuple(operating_case.dummy_rails),
        vlc=operating_case.vlc,
        pos_node=mode_data["pos_node"],
        n_node=mode_data["n_node"],
        dof_node=mode_data["dof_node"],
        mode_shape=mode_data["mode_shape"],
        rail_beam=rail_beam,
        distance_vehicle=_distance_vehicle(values),
    )


def build_default_07009_face_rail_beam(
    *,
    vlc: float,
    choose_turnout: str = "07(009)",
    choose_zjg_uneven: int = 0,
) -> RailBeamData:
    """Reproduce the active default ``Cal_RailBeam_230518`` tables."""

    if choose_turnout != "07(009)":
        raise ValueError(f"unsupported turnout for default RailBeam tables: {choose_turnout!r}")

    mileage_height_red = np.array(
        [
            [0.000, 23.000],
            [0.578, 14.000],
            [2.889, 3.000],
            [5.280, 1.375],
            [6.569, 0.500],
            [7.304, 0.000],
            [10.962, 0.000],
            [100.000, 0.000],
        ],
        dtype=float,
    )
    if int(choose_zjg_uneven) == 1:
        xy = np.array(
            [
                [0.000, 0.9],
                [0.578, -0.4],
                [2.889, 1.9],
                [7.304, 1.7],
                [10.962, 0.0],
                [1000.000, 0.0],
            ],
            dtype=float,
        )
        mileage_height_red[:, 1] += np.interp(mileage_height_red[:, 0], xy[:, 0], xy[:, 1])

    mileage_height_red[:, 0] += 50.0
    mileage_height_red[:, 1] /= 1000.0
    x = _matlab_colon(mileage_height_red[0, 0], 0.1, mileage_height_red[-1, 0])
    y = np.interp(x, mileage_height_red[:, 0], mileage_height_red[:, 1])
    vel = np.gradient(y) / np.gradient(x) * float(vlc)

    return RailBeamData(
        pos_z={"R2_zjg": np.column_stack((x, y))},
        vel_z={"R2_zjg": np.column_stack((x, vel))},
    )


def cal_shape_function_beam188_fwv(
    inp_par: Mapping[str, Any],
    par_vehicle: VehicleParameters | Mapping[str, Any],
    j1: float,
) -> tuple[dict[str, np.ndarray], dict[str, dict[str, np.ndarray]]]:
    """Reproduce ``Cal_ShapeFunction_Beam188_FWV.m`` for the default FT route."""

    n_wheels = int(inp_par["Nw"])
    n_contact_patch = int(inp_par["N_ConPatch"])
    type_rail = _cell_values(inp_par["Type_Rail"])
    exp_ws = _cell_values(inp_par["Exp_WS"])
    exp_dummy_rail = _cell_values(inp_par["Exp_DummyRail"])
    distance_vehicle = _distance_vehicle_vector(par_vehicle, exp_ws)
    rail_beam = inp_par.get("RailBeam", {})

    shape_function: dict[str, np.ndarray] = {}
    rail_beam_motion: dict[str, dict[str, np.ndarray]] = {
        "Pos_Y": {},
        "Vel_Y": {},
        "Win": {},
        "Pos_Z": {},
        "Vel_Z": {},
    }

    for wheel_index, wheelset in enumerate(exp_ws):
        mileage = float(j1) - distance_vehicle[wheel_index]
        for rail_index in range(n_contact_patch):
            rail = type_rail[rail_index]
            dummy_rail = exp_dummy_rail[rail_index]

            pos_node_temp = np.asarray(inp_par["Pos_Node"][rail], dtype=float)
            pos_node_length_temp = np.asarray(inp_par["Pos_Node"][f"{rail}_Length"], dtype=float).reshape(-1)
            node_count = int(inp_par["N_Node"][rail])
            m = int(np.searchsorted(pos_node_temp[:, 1], mileage, side="right") - 1)

            prefix = f"{wheelset}_{rail}"
            if 0 <= m < node_count - 1:
                x = mileage - float(pos_node_temp[m, 1])
                a = float(pos_node_length_temp[m])
                sigma = x / a

                shape_function[f"{prefix}_Y"] = np.array(
                    [
                        1.0 - 3.0 * sigma**2 + 2.0 * sigma**3,
                        (sigma - 2.0 * sigma**2 + sigma**3) * a,
                        sigma**2 * (3.0 - 2.0 * sigma),
                        (sigma**3 - sigma**2) * a,
                    ],
                    dtype=float,
                )
                shape_function[f"{prefix}_Z"] = np.array(
                    [
                        1.0 - 3.0 * sigma**2 + 2.0 * sigma**3,
                        -(sigma - 2.0 * sigma**2 + sigma**3) * a,
                        sigma**2 * (3.0 - 2.0 * sigma),
                        -(sigma**3 - sigma**2) * a,
                    ],
                    dtype=float,
                )
                shape_function[f"{prefix}_ROTY"] = np.array([1.0 - sigma, sigma], dtype=float)
                shape_function[f"{prefix}_ROTZ"] = np.array([1.0 - sigma, sigma], dtype=float)
                shape_function[f"{prefix}_Mapping_DynStatus_Y"] = np.array(
                    [m * 4 + 1, m * 4 + 4, m * 4 + 5, m * 4 + 8],
                    dtype=int,
                )
                shape_function[f"{prefix}_Mapping_DynStatus_Z"] = np.array(
                    [m * 4 + 2, m * 4 + 3, m * 4 + 6, m * 4 + 7],
                    dtype=int,
                )
                shape_function[f"{prefix}_Mapping_DynStatus_ROTY"] = np.array([m * 4 + 3, m * 4 + 7], dtype=int)
                shape_function[f"{prefix}_Mapping_DynStatus_ROTZ"] = np.array([m * 4 + 4, m * 4 + 8], dtype=int)

                if dummy_rail == "R2":
                    pos_table = _nested_field(rail_beam, "Pos_Z", "R2_zjg")
                    vel_table = _nested_field(rail_beam, "Vel_Z", "R2_zjg")
                    _assign_motion_value(
                        rail_beam_motion["Pos_Z"],
                        dummy_rail,
                        wheel_index,
                        n_wheels,
                        _interp1_linear_nan(pos_table[:, 0], pos_table[:, 1], mileage),
                    )
                    _assign_motion_value(
                        rail_beam_motion["Vel_Z"],
                        dummy_rail,
                        wheel_index,
                        n_wheels,
                        _interp1_linear_nan(vel_table[:, 0], vel_table[:, 1], mileage),
                    )
            else:
                shape_function[f"{prefix}_Y"] = np.array([], dtype=float)
                shape_function[f"{prefix}_Z"] = np.array([], dtype=float)
                shape_function[f"{prefix}_ROTY"] = np.array([], dtype=float)
                shape_function[f"{prefix}_ROTZ"] = np.array([], dtype=float)
                shape_function[f"{prefix}_Mapping_DynStatus_Y"] = np.array([], dtype=int)
                shape_function[f"{prefix}_Mapping_DynStatus_Z"] = np.array([], dtype=int)
                shape_function[f"{prefix}_Mapping_DynStatus_ROTY"] = np.array([], dtype=int)
                shape_function[f"{prefix}_Mapping_DynStatus_ROTZ"] = np.array([], dtype=int)

    return shape_function, rail_beam_motion


@lru_cache(maxsize=4)
def _load_modal_shape_function_inputs(
    modal_mat_path: str,
    cut_freq: float,
) -> dict[str, object]:
    modal = load_mat_variables(
        modal_mat_path,
        {"ModeFreq", "ModeShape", "Pos_Node", "N_Node", "DOF_Node"},
    ).variables
    mode_mask = np.asarray(modal["ModeFreq"]["FT_All"], dtype=float)[:, 1] <= float(cut_freq)
    n_track = int(np.count_nonzero(mode_mask))
    rail_types = ("zjbg", "qjbg", "zjg_zgyg", "cxg")

    pos_node: dict[str, np.ndarray] = {}
    n_node: dict[str, int] = {}
    dof_node: dict[str, int] = {}
    mode_shape: dict[str, np.ndarray] = {}
    for rail in rail_types:
        pos_node[rail] = np.asarray(modal["Pos_Node"][rail], dtype=float)
        pos_node[f"{rail}_Length"] = np.asarray(modal["Pos_Node"][f"{rail}_Length"], dtype=float)
        n_node[rail] = _scalar_int(modal["N_Node"][rail])
        dof_node[rail] = _scalar_int(modal["DOF_Node"][rail])
        mode_shape[rail] = np.asarray(modal["ModeShape"][rail], dtype=float)[:, mode_mask][:, :n_track]

    return {
        "n_track": n_track,
        "pos_node": pos_node,
        "n_node": n_node,
        "dof_node": dof_node,
        "mode_shape": mode_shape,
    }


def _distance_vehicle(values: Mapping[str, Any]) -> dict[str, float]:
    ll1 = float(values["Ll1"])
    ll2 = float(values["Ll2"])
    return {
        "FF": 0.0,
        "FR": 2.0 * ll1,
        "RF": 2.0 * ll2,
        "RR": 2.0 * (ll1 + ll2),
    }


def _distance_vehicle_vector(
    par_vehicle: VehicleParameters | Mapping[str, Any],
    wheelsets: list[str],
) -> np.ndarray:
    if isinstance(par_vehicle, VehicleParameters):
        return np.array([_distance_vehicle(par_vehicle.values)[wheelset] for wheelset in wheelsets], dtype=float)

    if "Distance_Vehicle" in par_vehicle:
        distances = np.asarray(par_vehicle["Distance_Vehicle"], dtype=float).reshape(-1)
        if distances.size != len(wheelsets):
            raise ValueError("Distance_Vehicle length does not match wheelset count")
        return distances

    return np.array([_distance_vehicle(par_vehicle)[wheelset] for wheelset in wheelsets], dtype=float)


def _cell_values(cell: Any) -> list[Any]:
    if isinstance(cell, np.ndarray):
        values = list(cell.reshape(-1, order="F"))
    else:
        values = list(cell)
    return [value.item() if isinstance(value, np.ndarray) and value.shape == () else value for value in values]


def _nested_field(struct: Mapping[str, Any], field_name: str, key: str) -> np.ndarray:
    table = struct.get(field_name, {})
    if key not in table:
        raise KeyError(f"missing RailBeam.{field_name}.{key}")
    return np.asarray(table[key], dtype=float)


def _assign_motion_value(
    target: dict[str, np.ndarray],
    rail: str,
    wheel_index: int,
    n_wheels: int,
    value: float,
) -> None:
    if rail not in target:
        target[rail] = np.full((n_wheels, 1), np.nan, dtype=float)
    target[rail][wheel_index, 0] = float(value)


def _interp1_linear_nan(x: np.ndarray, y: np.ndarray, xq: float) -> float:
    if float(xq) < float(x[0]) or float(xq) > float(x[-1]):
        return float("nan")
    return float(np.interp(float(xq), x, y))


def _matlab_colon(start: float, step: float, stop: float) -> np.ndarray:
    count = int(np.floor((stop - start) / step + 1.0e-12)) + 1
    return start + step * np.arange(count, dtype=float)


def _scalar_int(value: Any) -> int:
    return int(np.asarray(value, dtype=int).reshape(-1)[0])
