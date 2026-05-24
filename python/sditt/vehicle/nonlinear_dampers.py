from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from .matrices import VehicleMatrices


@dataclass(frozen=True)
class NonlinearDamperBranch:
    """One MATLAB ``DamperNL`` branch."""

    damp_velocity: np.ndarray | None
    spring_length: np.ndarray
    mark: np.ndarray


@dataclass(frozen=True)
class NonlinearDamperState:
    """State returned by MATLAB ``Judge_DamperNL`` / ``NonLinear_DampingForce``."""

    dpz: NonlinearDamperBranch
    sx: NonlinearDamperBranch
    dpy: NonlinearDamperBranch
    sty: NonlinearDamperBranch
    fz_dpz_damp: np.ndarray
    fx_sx_damp: np.ndarray
    fy_dpy_damp: np.ndarray
    fy_sty_stiff: np.ndarray


@dataclass(frozen=True)
class NonlinearDamperResponse:
    """Nonlinear damper update for one frozen full-system state."""

    state: NonlinearDamperState
    c_vehicle: np.ndarray
    pxt_force: np.ndarray
    equivalent_force: np.ndarray


def build_nonlinear_damper_response(
    inp_par: Mapping[str, object],
    vehicle_parameters: Mapping[str, Any],
    vehicle_matrices: VehicleMatrices,
    displacement: np.ndarray,
    velocity: np.ndarray,
) -> NonlinearDamperResponse:
    """Reproduce the default CRH380A_v6 nonlinear damper stage.

    MATLAB updates ``Cxt`` and adds a piecewise-linear damper offset into
    ``Pxt``. The Python full-case driver keeps the base linear ``Cxt`` fixed and
    returns an equivalent force correction:

    ``(C_vehicle_base - C_vehicle_nl) @ v_vehicle + Pxt_nl``

    so the integrated dynamics remain equivalent without mutating the stepper's
    system matrices inside the force-iteration loop.
    """

    vehicle_slice = _vehicle_block_slice(inp_par, vehicle_matrices, displacement.shape[0])
    q_vehicle = np.asarray(displacement[vehicle_slice], dtype=float).reshape(-1)
    v_vehicle = np.asarray(velocity[vehicle_slice], dtype=float).reshape(-1)

    state = _judge_damper_nl(inp_par, vehicle_parameters, q_vehicle, v_vehicle)
    c_vehicle = _create_c_vehicle_nl(inp_par, vehicle_parameters, state, vehicle_matrices.C_vehicle_linear)
    pxt_vehicle = _nonlinear_damping_force(inp_par, vehicle_parameters, state, q_vehicle.shape[0])

    pxt_force = np.zeros_like(displacement, dtype=float)
    pxt_force[vehicle_slice] = pxt_vehicle

    equivalent_force = np.zeros_like(displacement, dtype=float)
    equivalent_force[vehicle_slice] = (vehicle_matrices.C_vehicle - c_vehicle) @ v_vehicle + pxt_vehicle

    return NonlinearDamperResponse(
        state=state,
        c_vehicle=c_vehicle,
        pxt_force=pxt_force,
        equivalent_force=equivalent_force,
    )


def _judge_damper_nl(
    inp_par: Mapping[str, object],
    vehicle_parameters: Mapping[str, Any],
    q_vehicle: np.ndarray,
    v_vehicle: np.ndarray,
) -> NonlinearDamperState:
    nm_fw = int(inp_par["NM_FW"])
    if nm_fw != 0:
        raise NotImplementedError("nonlinear vehicle dampers currently support only the RW route with NM_FW == 0")

    p = vehicle_parameters
    range_i3_dpz = np.array([1, 2, 1, 2, 1, 2, 1, 2], dtype=int)
    range_i2_dpz = np.array([1, 1, 2, 2, 1, 1, 2, 2], dtype=int)
    range_i1_dpy = np.array([1, 1, 2, 2], dtype=int)
    range_i2_dpy = np.array([1, 2, 1, 2], dtype=int)
    range_i3_sx = np.array([1, 2, 1, 2], dtype=int)
    range_i1_sty = np.array([1, 2], dtype=int)

    bg_z_dpz = _idx1([21, 21, 21, 21, 26, 26, 26, 26])
    bg_rx_dpz = _idx1([23, 23, 23, 23, 28, 28, 28, 28])
    bg_ry_dpz = _idx1([24, 24, 24, 24, 29, 29, 29, 29])
    ws_z_dpz = _idx1([1, 1, 6, 6, 11, 11, 16, 16])
    ws_rx_dpz = _idx1([3, 3, 8, 8, 13, 13, 18, 18])
    ws_ry_dpz = _idx1([4, 4, 9, 9, 14, 14, 19, 19])
    pz_idx = _idx1(range(36, 44))

    dpz_damp_velocity = (
        v_vehicle[bg_z_dpz]
        - v_vehicle[pz_idx]
        + ((-1.0) ** range_i3_dpz) * float(p["Lb_DPz"]) * v_vehicle[bg_rx_dpz]
        + ((-1.0) ** range_i2_dpz) * float(p["Ll_DPzt"]) * v_vehicle[bg_ry_dpz]
    )
    dpz_spring_length = (
        q_vehicle[pz_idx]
        - q_vehicle[ws_z_dpz]
        - ((-1.0) ** range_i3_dpz) * float(p["Lb_DPz"]) * q_vehicle[ws_rx_dpz]
        - ((-1.0) ** range_i2_dpz) * float(p["Ll_DPzw"]) * q_vehicle[ws_ry_dpz]
    )
    dpz_mark = np.abs(dpz_damp_velocity) > float(p["C_DPz_V0"])

    cb_x_sx = _idx1([34, 34, 34, 34])
    cb_yaw_sx = _idx1([35, 35, 35, 35])
    sn_x_idx = _idx1([44, 45, 46, 47])
    bg_pitch_sx = _idx1([24, 24, 29, 29])
    bg_yaw_sx = _idx1([25, 25, 30, 30])

    sx_damp_velocity = (
        float(p["H_cS"]) * v_vehicle[cb_x_sx]
        - v_vehicle[sn_x_idx]
        + ((-1.0) ** (range_i3_sx + 1)) * float(p["Lb_S"]) * v_vehicle[cb_yaw_sx]
    )
    sx_spring_length = (
        q_vehicle[sn_x_idx]
        + float(p["H_St"]) * q_vehicle[bg_pitch_sx]
        - ((-1.0) ** (range_i3_sx + 1)) * float(p["Lb_S"]) * q_vehicle[bg_yaw_sx]
    )
    sx_mark = np.abs(sx_damp_velocity) > float(p["C_Sx_V0"])

    cb_y_dpy = _idx1([32, 32, 32, 32])
    cb_roll_dpy = _idx1([33, 33, 33, 33])
    cb_yaw_dpy = _idx1([35, 35, 35, 35])
    py_idx = _idx1([48, 49, 50, 51])
    bg_y_dpy = _idx1([22, 22, 27, 27])
    bg_roll_dpy = _idx1([23, 23, 28, 28])
    bg_yaw_dpy = _idx1([25, 25, 30, 30])

    ll_dpyc = np.array(
        [
            float(p["Ll2"]) + float(p["Ll_DPyt"]),
            float(p["Ll2"]) - float(p["Ll_DPyt"]),
            float(p["Ll2"]) - float(p["Ll_DPyt"]),
            float(p["Ll2"]) + float(p["Ll_DPyt"]),
        ],
        dtype=float,
    )
    dpy_damp_velocity = (
        v_vehicle[cb_y_dpy]
        - v_vehicle[py_idx]
        - float(p["H_cDPy"]) * v_vehicle[cb_roll_dpy]
        + ((-1.0) ** (range_i1_dpy + 1)) * ll_dpyc * v_vehicle[cb_yaw_dpy]
    )
    dpy_spring_length = (
        q_vehicle[py_idx]
        - q_vehicle[bg_y_dpy]
        - float(p["H_DPyt"]) * q_vehicle[bg_roll_dpy]
        - ((-1.0) ** (range_i2_dpy + 1)) * float(p["Ll_DPyt"]) * q_vehicle[bg_yaw_dpy]
    )
    dpy_mark = np.abs(dpy_damp_velocity) > float(p["C_DPy_V0"])

    cb_y_sty = _idx1([32, 32])
    cb_roll_sty = _idx1([33, 33])
    cb_yaw_sty = _idx1([35, 35])
    bg_y_sty = _idx1([22, 27])
    bg_roll_sty = _idx1([23, 28])

    sty_spring_length = (
        q_vehicle[cb_y_sty]
        - q_vehicle[bg_y_sty]
        - float(p["H_cST"]) * q_vehicle[cb_roll_sty]
        - float(p["H_STt"]) * q_vehicle[bg_roll_sty]
        + ((-1.0) ** (range_i1_sty + 1)) * float(p["Ll_ST"]) * q_vehicle[cb_yaw_sty]
    )
    sty_mark = np.abs(sty_spring_length) > float(p["L0_ST"])

    fz_dpz_damp = np.zeros(8, dtype=float)
    active = dpz_mark
    fz_dpz_damp[active] = (
        np.sign(dpz_damp_velocity[active])
        * (float(np.asarray(p["C_DPz"], dtype=float).ravel()[0]) - float(np.asarray(p["C_DPz"], dtype=float).ravel()[1]))
        * float(p["C_DPz_V0"])
    )

    fx_sx_damp = np.zeros(4, dtype=float)
    active = sx_mark
    fx_sx_damp[active] = (
        np.sign(sx_damp_velocity[active])
        * (float(np.asarray(p["C_Sx"], dtype=float).ravel()[0]) - float(np.asarray(p["C_Sx"], dtype=float).ravel()[1]))
        * float(p["C_Sx_V0"])
    )

    fy_dpy_damp = np.zeros(4, dtype=float)
    active = dpy_mark
    fy_dpy_damp[active] = (
        np.sign(dpy_damp_velocity[active])
        * (float(np.asarray(p["C_DPy"], dtype=float).ravel()[0]) - float(np.asarray(p["C_DPy"], dtype=float).ravel()[1]))
        * float(p["C_DPy_V0"])
    )

    fy_sty_stiff = np.interp(
        sty_spring_length,
        np.asarray(p["K_STy_Table"], dtype=float)[:, 0],
        np.asarray(p["K_STy_Table"], dtype=float)[:, 1],
    )

    return NonlinearDamperState(
        dpz=NonlinearDamperBranch(
            damp_velocity=dpz_damp_velocity,
            spring_length=dpz_spring_length,
            mark=dpz_mark,
        ),
        sx=NonlinearDamperBranch(
            damp_velocity=sx_damp_velocity,
            spring_length=sx_spring_length,
            mark=sx_mark,
        ),
        dpy=NonlinearDamperBranch(
            damp_velocity=dpy_damp_velocity,
            spring_length=dpy_spring_length,
            mark=dpy_mark,
        ),
        sty=NonlinearDamperBranch(
            damp_velocity=None,
            spring_length=sty_spring_length,
            mark=sty_mark,
        ),
        fz_dpz_damp=fz_dpz_damp,
        fx_sx_damp=fx_sx_damp,
        fy_dpy_damp=fy_dpy_damp,
        fy_sty_stiff=fy_sty_stiff,
    )


def _create_c_vehicle_nl(
    inp_par: Mapping[str, object],
    vehicle_parameters: Mapping[str, Any],
    state: NonlinearDamperState,
    c_vehicle_linear: np.ndarray,
) -> np.ndarray:
    p = vehicle_parameters
    clc = np.array(c_vehicle_linear, dtype=float, copy=True)
    dof_fw = int(inp_par["Nw"]) * int(inp_par["NM_FW"])
    c_dpz_values = np.asarray(p["C_DPz"], dtype=float).ravel()
    c_sx_values = np.asarray(p["C_Sx"], dtype=float).ravel()
    c_dpy_values = np.asarray(p["C_DPy"], dtype=float).ravel()

    for i1 in range(1, 3):
        for i2 in range(1, 3):
            for i3 in range(1, 3):
                pos = 4 * (i1 - 1) + 2 * (i2 - 1) + i3
                pos_p = dof_fw + 35 + pos
                pos_b = dof_fw + 20 + 5 * (i1 - 1)
                c_dpz = c_dpz_values[0] if not state.dpz.mark[pos - 1] else c_dpz_values[1]
                _add(clc, pos_b + 1, pos_b + 1, c_dpz)
                _add(clc, pos_p, pos_b + 1, -c_dpz)
                _add(clc, pos_b + 3, pos_b + 1, ((-1) ** i3) * float(p["Lb_DPz"]) * c_dpz)
                _add(clc, pos_b + 4, pos_b + 1, ((-1) ** i2) * float(p["Ll_DPzt"]) * c_dpz)
                _add(clc, pos_b + 1, pos_p, -c_dpz)
                _add(clc, pos_p, pos_p, c_dpz)
                _add(clc, pos_b + 3, pos_p, -(((-1) ** i3) * float(p["Lb_DPz"]) * c_dpz))
                _add(clc, pos_b + 4, pos_p, -(((-1) ** i2) * float(p["Ll_DPzt"]) * c_dpz))
                _add(clc, pos_b + 1, pos_b + 3, ((-1) ** i3) * float(p["Lb_DPz"]) * c_dpz)
                _add(clc, pos_p, pos_b + 3, -(((-1) ** i3) * float(p["Lb_DPz"]) * c_dpz))
                _add(clc, pos_b + 3, pos_b + 3, float(p["Lb_DPz"]) * float(p["Lb_DPz"]) * c_dpz)
                _add(
                    clc,
                    pos_b + 4,
                    pos_b + 3,
                    ((-1) ** i2) * float(p["Ll_DPzt"]) * (((-1) ** i3) * float(p["Lb_DPz"])) * c_dpz,
                )
                _add(clc, pos_b + 1, pos_b + 4, ((-1) ** i2) * float(p["Ll_DPzt"]) * c_dpz)
                _add(clc, pos_p, pos_b + 4, -(((-1) ** i2) * float(p["Ll_DPzt"]) * c_dpz))
                _add(
                    clc,
                    pos_b + 3,
                    pos_b + 4,
                    ((-1) ** i3) * float(p["Lb_DPz"]) * (((-1) ** i2) * float(p["Ll_DPzt"])) * c_dpz,
                )
                _add(clc, pos_b + 4, pos_b + 4, float(p["Ll_DPzt"]) * float(p["Ll_DPzt"]) * c_dpz)

    for i1 in range(1, 3):
        for i3 in range(1, 3):
            pos = 2 * (i1 - 1) + i3
            pos_p = dof_fw + 35 + 8 + pos
            pos_c = dof_fw + 30
            c_sx = c_sx_values[0] if not state.sx.mark[pos - 1] else c_sx_values[1]
            _add(clc, pos_c + 4, pos_c + 4, float(p["H_cS"]) * float(p["H_cS"]) * c_sx)
            _add(clc, pos_c + 5, pos_c + 4, float(p["H_cS"]) * ((-1) ** (i3 + 1)) * float(p["Lb_S"]) * c_sx)
            _add(clc, pos_p, pos_c + 4, -float(p["H_cS"]) * c_sx)
            _add(clc, pos_c + 4, pos_c + 5, ((-1) ** (i3 + 1)) * float(p["Lb_S"]) * float(p["H_cS"]) * c_sx)
            _add(clc, pos_c + 5, pos_c + 5, float(p["Lb_S"]) * float(p["Lb_S"]) * c_sx)
            _add(clc, pos_p, pos_c + 5, -(((-1) ** (i3 + 1)) * float(p["Lb_S"]) * c_sx))
            _add(clc, pos_c + 4, pos_p, -float(p["H_cS"]) * c_sx)
            _add(clc, pos_c + 5, pos_p, -(((-1) ** (i3 + 1)) * float(p["Lb_S"]) * c_sx))
            _add(clc, pos_p, pos_p, c_sx)

    for i1 in range(1, 3):
        for i2 in range(1, 3):
            pos = 2 * (i1 - 1) + i2
            pos_p = dof_fw + 35 + 8 + 4 + pos
            pos_c = dof_fw + 30
            c_dpy = c_dpy_values[0] if not state.dpy.mark[pos - 1] else c_dpy_values[1]
            if (i1 == 1 and i2 == 1) or (i1 == 2 and i2 == 2):
                ll_dpyc = float(p["Ll2"]) + float(p["Ll_DPyt"])
            else:
                ll_dpyc = float(p["Ll2"]) - float(p["Ll_DPyt"])
            _add(clc, pos_c + 2, pos_c + 2, c_dpy)
            _add(clc, pos_p, pos_c + 2, -c_dpy)
            _add(clc, pos_c + 3, pos_c + 2, -float(p["H_cDPy"]) * c_dpy)
            _add(clc, pos_c + 5, pos_c + 2, ((-1) ** (i1 + 1)) * ll_dpyc * c_dpy)
            _add(clc, pos_c + 2, pos_p, -c_dpy)
            _add(clc, pos_p, pos_p, c_dpy)
            _add(clc, pos_c + 3, pos_p, float(p["H_cDPy"]) * c_dpy)
            _add(clc, pos_c + 5, pos_p, -(((-1) ** (i1 + 1)) * ll_dpyc * c_dpy))
            _add(clc, pos_c + 2, pos_c + 3, -float(p["H_cDPy"]) * c_dpy)
            _add(clc, pos_p, pos_c + 3, float(p["H_cDPy"]) * c_dpy)
            _add(clc, pos_c + 3, pos_c + 3, float(p["H_cDPy"]) * float(p["H_cDPy"]) * c_dpy)
            _add(clc, pos_c + 5, pos_c + 3, -float(p["H_cDPy"]) * (((-1) ** (i1 + 1)) * ll_dpyc) * c_dpy)
            _add(clc, pos_c + 2, pos_c + 5, ((-1) ** (i1 + 1)) * ll_dpyc * c_dpy)
            _add(clc, pos_p, pos_c + 5, -(((-1) ** (i1 + 1)) * ll_dpyc * c_dpy))
            _add(clc, pos_c + 3, pos_c + 5, -(((-1) ** (i1 + 1)) * ll_dpyc * float(p["H_cDPy"]) * c_dpy))
            _add(clc, pos_c + 5, pos_c + 5, ll_dpyc * ll_dpyc * c_dpy)

    return clc


def _nonlinear_damping_force(
    inp_par: Mapping[str, object],
    vehicle_parameters: Mapping[str, Any],
    state: NonlinearDamperState,
    n_vehicle_dof: int,
) -> np.ndarray:
    p = vehicle_parameters
    pxt = np.zeros(n_vehicle_dof, dtype=float)
    range_i3_dpz = np.array([1, 2, 1, 2, 1, 2, 1, 2], dtype=int)
    range_i2_dpz = np.array([1, 1, 2, 2, 1, 1, 2, 2], dtype=int)
    range_i1_dpy = np.array([1, 1, 2, 2], dtype=int)
    range_i3_sx = np.array([1, 2, 1, 2], dtype=int)
    range_i1_sty = np.array([1, 2], dtype=int)

    bg_base = np.array([20, 20, 20, 20, 25, 25, 25, 25], dtype=int)
    pz_idx = np.array([36, 37, 38, 39, 40, 41, 42, 43], dtype=int)
    for kk in range(8):
        force = state.fz_dpz_damp[kk]
        pos_bg = bg_base[kk]
        pxt[pos_bg] -= force
        pxt[pz_idx[kk] - 1] += force
        pxt[pos_bg + 2] += ((-1) ** (range_i3_dpz[kk] + 1)) * float(p["Lb_DPz"]) * force
        pxt[pos_bg + 3] += ((-1) ** (range_i2_dpz[kk] + 1)) * float(p["Ll_DPzt"]) * force

    cb_base = 30
    sn_idx = np.array([44, 45, 46, 47], dtype=int)
    for kk in range(4):
        force = state.fx_sx_damp[kk]
        pxt[sn_idx[kk] - 1] += force
        pxt[cb_base + 3] -= float(p["H_cS"]) * force
        pxt[cb_base + 4] += ((-1) ** range_i3_sx[kk]) * float(p["Lb_S"]) * force

    py_idx = np.array([48, 49, 50, 51], dtype=int)
    for kk in range(4):
        force = state.fy_dpy_damp[kk]
        pxt[cb_base + 1] -= force
        pxt[py_idx[kk] - 1] += force
        pxt[cb_base + 2] += float(p["H_cDPy"]) * force
        if kk in (0, 3):
            ll_dpyc = float(p["Ll2"]) + float(p["Ll_DPyt"])
        else:
            ll_dpyc = float(p["Ll2"]) - float(p["Ll_DPyt"])
        pxt[cb_base + 4] -= ((-1) ** (range_i1_dpy[kk] + 1)) * ll_dpyc * force

    bg_y_idx = np.array([22, 27], dtype=int)
    for kk in range(2):
        force = state.fy_sty_stiff[kk]
        pxt[cb_base + 1] -= force
        pxt[bg_y_idx[kk] - 1] += force
        pxt[cb_base + 2] += float(p["H_cST"]) * force
        pxt[bg_y_idx[kk]] += float(p["H_STt"]) * force
        pxt[cb_base + 4] -= ((-1) ** (range_i1_sty[kk] + 1)) * float(p["Ll_ST"]) * force

    return pxt


def _vehicle_block_slice(
    inp_par: Mapping[str, object],
    vehicle_matrices: VehicleMatrices,
    ndof: int,
) -> slice:
    n_track = int(inp_par["N_track"])
    n_vehicle = vehicle_matrices.M_vehicle.shape[0]
    vehicle_slice = slice(n_track, n_track + n_vehicle)
    if vehicle_slice.stop > ndof:
        raise ValueError("vehicle block exceeds supplied system state length")
    return vehicle_slice


def _idx1(values: range | list[int]) -> np.ndarray:
    return np.asarray(list(values), dtype=int) - 1


def _add(matrix: np.ndarray, row_1based: int, col_1based: int, value: float) -> None:
    matrix[row_1based - 1, col_1based - 1] += float(value)

