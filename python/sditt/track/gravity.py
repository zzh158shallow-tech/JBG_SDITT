from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from sditt.io.matlab import load_mat_file, load_mat_variables
from sditt.vehicle import VehicleParameters


@dataclass(frozen=True)
class ModalFTGravityPreload:
    """Static gravity preload for the modal flexible-turnout route."""

    pxt_gravity: np.ndarray
    pxt_track: Mapping[str, Any]


def build_modal_ft_gravity_preload(
    *,
    modal_mat_path: str | Path,
    rail_pro_path: str | Path,
    baseplate_pro_path: str | Path,
    vehicle_parameters: VehicleParameters | Mapping[str, Any],
    n_track: int,
    cut_freq: float,
    choose_turnout: str = "07(009)",
    type_rail_all: tuple[str, ...],
    type_baseplate: tuple[str, ...],
    nm_fw: int = 0,
    n_wheels: int = 4,
    n_rv: int = 51,
) -> ModalFTGravityPreload:
    """Reproduce ``Gravity_Load_ModalFT.m`` for the modal FT default route."""

    values = vehicle_parameters.values if isinstance(vehicle_parameters, VehicleParameters) else vehicle_parameters
    return _build_modal_ft_gravity_preload_cached(
        str(Path(modal_mat_path).resolve()),
        str(Path(rail_pro_path).resolve()),
        str(Path(baseplate_pro_path).resolve()),
        float(values["Mw"]),
        float(values["Mb"]),
        float(values["Mc"]),
        int(n_track),
        float(cut_freq),
        str(choose_turnout),
        tuple(type_rail_all),
        tuple(type_baseplate),
        int(nm_fw),
        int(n_wheels),
        int(n_rv),
    )


@lru_cache(maxsize=4)
def _build_modal_ft_gravity_preload_cached(
    modal_mat_path: str,
    rail_pro_path: str,
    baseplate_pro_path: str,
    mw: float,
    mb: float,
    mc: float,
    n_track: int,
    cut_freq: float,
    choose_turnout: str,
    type_rail_all: tuple[str, ...],
    type_baseplate: tuple[str, ...],
    nm_fw: int,
    n_wheels: int,
    n_rv: int,
) -> ModalFTGravityPreload:
    modal = load_mat_variables(
        modal_mat_path,
        {"ModeFreq", "ModeShape", "Pos_Node", "N_Node", "DOF_Node", "Type_SpaceIron"},
    ).variables
    rail_pro = load_mat_file(rail_pro_path).variables["RailPro"]
    baseplate_pro = load_mat_file(baseplate_pro_path).variables["BaseplatePro"]

    mode_shape = modal["ModeShape"]
    pos_node = modal["Pos_Node"]
    n_node = modal["N_Node"]
    dof_node = modal["DOF_Node"]
    type_space_iron = tuple(str(item) for item in modal["Type_SpaceIron"])
    mode_mask = np.asarray(modal["ModeFreq"]["FT_All"], dtype=float)[:, 1] <= cut_freq

    total_dof = n_track + nm_fw * n_wheels + n_rv
    pxt_gravity = np.zeros(total_dof, dtype=float)
    pxt_track: dict[str, Any] = {}

    _add_vehicle_body_gravity(pxt_gravity, n_track + nm_fw * n_wheels, mw=mw, mb=mb, mc=mc)
    _add_rail_gravity(
        pxt_gravity,
        pxt_track,
        rail_pro=rail_pro,
        mode_shape=mode_shape,
        pos_node=pos_node,
        n_node=n_node,
        dof_node=dof_node,
        mode_mask=mode_mask,
        n_track=n_track,
        choose_turnout=choose_turnout,
        type_rail_all=type_rail_all,
    )
    _add_baseplate_gravity(
        pxt_gravity,
        pxt_track,
        baseplate_pro=baseplate_pro,
        mode_shape=mode_shape,
        pos_node=pos_node,
        mode_mask=mode_mask,
        n_track=n_track,
        type_baseplate=type_baseplate,
    )
    _add_space_iron_gravity(
        pxt_gravity,
        pxt_track,
        mode_shape=mode_shape,
        pos_node=pos_node,
        n_node=n_node,
        mode_mask=mode_mask,
        n_track=n_track,
        type_space_iron=type_space_iron,
    )

    return ModalFTGravityPreload(pxt_gravity=pxt_gravity, pxt_track=pxt_track)


def _add_vehicle_body_gravity(
    pxt_gravity: np.ndarray,
    offset: int,
    *,
    mw: float,
    mb: float,
    mc: float,
) -> None:
    pxt_gravity[offset + np.array([0, 5, 10, 15])] = 9.81 * mw
    pxt_gravity[offset + np.array([20, 25])] = 9.81 * mb
    pxt_gravity[offset + 30] = 9.81 * mc


def _add_rail_gravity(
    pxt_gravity: np.ndarray,
    pxt_track: dict[str, Any],
    *,
    rail_pro: Mapping[str, np.ndarray],
    mode_shape: Mapping[str, Any],
    pos_node: Mapping[str, Any],
    n_node: Mapping[str, Any],
    dof_node: Mapping[str, Any],
    mode_mask: np.ndarray,
    n_track: int,
    choose_turnout: str,
    type_rail_all: tuple[str, ...],
) -> None:
    density = 7850.0
    for rail_type in _rail_range(choose_turnout, type_rail_all):
        len_nodes = _scalar_int(n_node[rail_type])
        area = np.asarray(rail_pro[rail_type], dtype=float)[:, 4]
        if choose_turnout == "CN18" and type_rail_all.index(rail_type) == 4:
            add_type = type_rail_all[5]
            area = np.concatenate([area, np.asarray(rail_pro[add_type], dtype=float)[1:, 4]])

        tributary_length = _tributary_lengths(np.asarray(pos_node[f"{rail_type}_Length"], dtype=float).reshape(-1))
        mass = area * tributary_length * density

        track_force = np.zeros(_scalar_int(dof_node[rail_type]), dtype=float)
        track_force[1 + 4 * np.arange(len_nodes)] = mass * 9.81
        pxt_track[rail_type] = track_force
        pxt_gravity[:n_track] += _modal_project(mode_shape[rail_type], track_force, mode_mask, n_track)


def _add_baseplate_gravity(
    pxt_gravity: np.ndarray,
    pxt_track: dict[str, Any],
    *,
    baseplate_pro: Mapping[str, Any],
    mode_shape: Mapping[str, Any],
    pos_node: Mapping[str, Any],
    mode_mask: np.ndarray,
    n_track: int,
    type_baseplate: tuple[str, ...],
) -> None:
    if not type_baseplate or type_baseplate[0] not in mode_shape:
        return

    density = 7850.0
    baseplate_nd = baseplate_pro["ND"]
    for baseplate_type in type_baseplate:
        cells = _cell_table(pos_node[f"{baseplate_type}_cell"], n_columns=5)
        div_lengths = pos_node[f"{baseplate_type}_Div_Length"]
        pxt_track[baseplate_type] = {}
        node_offset = 0

        for row in cells:
            number = _scalar_int(row[0])
            part_name = f"B_m{abs(number)}" if number < 0 else f"B_{number}"
            n_part = _scalar_int(row[3])
            node_slice = slice(node_offset, node_offset + n_part)

            tributary_length = _tributary_lengths(np.asarray(div_lengths[part_name], dtype=float).reshape(-1))
            nd = np.asarray(baseplate_nd[baseplate_type], dtype=float)
            height = np.empty(n_part, dtype=float)
            height[0] = nd[node_slice.start, 4]
            height[-1] = nd[node_slice.stop - 1, 4]
            if n_part > 2:
                height[1:-1] = (nd[node_slice.start + 1 : node_slice.stop - 1, 4] + nd[node_slice.start : node_slice.stop - 2, 4]) / 2.0
            width = nd[node_slice, 5]
            mass = height * width * tributary_length * density

            track_force = np.zeros(n_part * 2, dtype=float)
            track_force[0 : 2 * n_part : 2] = mass * 9.81
            pxt_track[baseplate_type][part_name] = track_force
            pxt_gravity[:n_track] += _modal_project(
                mode_shape[baseplate_type][part_name],
                track_force,
                mode_mask,
                n_track,
            )
            node_offset += n_part


def _add_space_iron_gravity(
    pxt_gravity: np.ndarray,
    pxt_track: dict[str, Any],
    *,
    mode_shape: Mapping[str, Any],
    pos_node: Mapping[str, Any],
    n_node: Mapping[str, Any],
    mode_mask: np.ndarray,
    n_track: int,
    type_space_iron: tuple[str, ...],
) -> None:
    if not type_space_iron or type_space_iron[0] not in mode_shape:
        return

    density = 7850.0
    for iron_type in type_space_iron:
        pxt_track[iron_type] = {}
        for part_name in _sorted_part_names(pos_node[iron_type]):
            n_part = _scalar_int(n_node[iron_type][part_name])
            nodes = np.asarray(pos_node[iron_type][part_name], dtype=float)

            tributary_length = np.empty(n_part, dtype=float)
            tributary_length[0] = (nodes[1, 2] - nodes[0, 2]) / 2.0
            tributary_length[-1] = (nodes[-1, 2] - nodes[-2, 2]) / 2.0
            if n_part > 2:
                tributary_length[1:-1] = (nodes[2:, 2] - nodes[:-2, 2]) / 2.0
            mass = nodes[:, 4] * nodes[:, 5] * tributary_length * density

            track_len = n_part * 6 - 4
            track_force = np.zeros(track_len, dtype=float)
            positions = np.concatenate(
                [
                    np.array([1], dtype=int),
                    6 + 6 * np.arange(max(n_part - 2, 0), dtype=int),
                    np.array([track_len - 3], dtype=int),
                ]
            )
            track_force[positions] = mass * 9.81
            pxt_track[iron_type][part_name] = track_force
            pxt_gravity[:n_track] += _modal_project(
                mode_shape[iron_type][part_name],
                track_force,
                mode_mask,
                n_track,
            )


def _rail_range(choose_turnout: str, type_rail_all: tuple[str, ...]) -> tuple[str, ...]:
    if choose_turnout == "07(009)":
        return type_rail_all
    if choose_turnout == "CN18":
        return tuple(type_rail_all[index] for index in (0, 1, 2, 3, 4, 6, 7))
    raise ValueError(f"unsupported turnout for modal FT gravity preload: {choose_turnout}")


def _tributary_lengths(segment_lengths: np.ndarray) -> np.ndarray:
    lengths = np.asarray(segment_lengths, dtype=float).reshape(-1)
    result = np.empty(lengths.size + 1, dtype=float)
    result[0] = lengths[0] / 2.0
    result[-1] = lengths[-1] / 2.0
    if result.size > 2:
        result[1:-1] = (lengths[:-1] + lengths[1:]) / 2.0
    return result


def _modal_project(
    shape: np.ndarray,
    physical_force: np.ndarray,
    mode_mask: np.ndarray,
    n_track: int,
) -> np.ndarray:
    shape_array = np.asarray(shape, dtype=float)
    if shape_array.shape[1] == mode_mask.size:
        active_shape = shape_array[:, mode_mask]
    elif shape_array.shape[1] == n_track:
        active_shape = shape_array
    elif shape_array.shape[1] > n_track:
        active_shape = shape_array[:, :n_track]
    else:
        raise ValueError(f"mode shape has {shape_array.shape[1]} modes, cannot project to {n_track}")
    if active_shape.shape[0] != physical_force.size:
        raise ValueError(f"mode shape rows {active_shape.shape[0]} do not match force size {physical_force.size}")
    return active_shape.T @ physical_force


def _cell_table(cells: list[Any], *, n_columns: int) -> list[list[Any]]:
    if len(cells) % n_columns != 0:
        raise ValueError(f"cell array length {len(cells)} is not divisible by {n_columns}")
    n_rows = len(cells) // n_columns
    return [[cells[col * n_rows + row] for col in range(n_columns)] for row in range(n_rows)]


def _sorted_part_names(parts: Mapping[str, Any]) -> tuple[str, ...]:
    return tuple(sorted(parts, key=lambda name: int(name.split("_", 1)[1])))


def _scalar_int(value: Any) -> int:
    return int(np.asarray(value).reshape(-1)[0])
