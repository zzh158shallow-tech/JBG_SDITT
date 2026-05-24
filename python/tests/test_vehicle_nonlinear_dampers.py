from __future__ import annotations

import numpy as np
import pytest

from sditt.simulation import FullDefaultCaseSettings, prepare_default_full_case
from sditt.vehicle import build_nonlinear_damper_response


def test_nonlinear_vehicle_damper_response_matches_crh380a_v6_piecewise_terms() -> None:
    preparation = prepare_default_full_case(settings=FullDefaultCaseSettings(cut_freq=50.0))
    layout = preparation.system.layout
    params = preparation.vehicle_parameters.values

    q = np.zeros(preparation.total_dof, dtype=float)
    v = np.zeros_like(q)
    offset = layout.n_track

    q[offset + 31] = 0.03
    q[offset + 21] = 0.03
    v[offset + 20] = 0.2
    v[offset + 36] = 0.2
    v[offset + 37] = 0.2
    v[offset + 38] = 0.2
    v[offset + 44] = -0.01
    v[offset + 50] = -0.2

    response = build_nonlinear_damper_response(
        preparation.stage_inp_par["Cal"],
        params,
        preparation.vehicle,
        q,
        v,
    )

    assert response.state.dpz.mark.tolist() == [True, False, False, False, False, False, False, False]
    assert response.state.sx.mark.tolist() == [False, True, False, False]
    assert response.state.dpy.mark.tolist() == [False, False, False, True]
    assert response.state.sty.mark.tolist() == [False, True]

    c_dpz = np.asarray(params["C_DPz"], dtype=float).ravel()
    c_sx = np.asarray(params["C_Sx"], dtype=float).ravel()
    c_dpy = np.asarray(params["C_DPy"], dtype=float).ravel()
    delta_dpz = float(c_dpz[0] - c_dpz[1])
    delta_sx = float(c_sx[0] - c_sx[1])
    delta_dpy = float(c_dpy[0] - c_dpy[1])
    expected_fz = delta_dpz * float(params["C_DPz_V0"])
    expected_fx = delta_sx * float(params["C_Sx_V0"])
    expected_fy = delta_dpy * float(params["C_DPy_V0"])

    assert response.state.fz_dpz_damp[0] == pytest.approx(expected_fz)
    assert np.count_nonzero(response.state.fz_dpz_damp) == 1
    assert response.state.fx_sx_damp[1] == pytest.approx(expected_fx)
    assert np.count_nonzero(response.state.fx_sx_damp) == 1
    assert response.state.fy_dpy_damp[3] == pytest.approx(expected_fy)
    assert np.count_nonzero(response.state.fy_dpy_damp) == 1
    assert np.allclose(response.state.sty.spring_length, [0.0, 0.03])
    assert response.state.fy_sty_stiff[1] == pytest.approx(4000.0)

    assert response.c_vehicle[35, 35] == pytest.approx(preparation.vehicle.C_vehicle[35, 35] - delta_dpz)
    assert response.c_vehicle[44, 44] == pytest.approx(preparation.vehicle.C_vehicle[44, 44] - delta_sx)
    assert response.c_vehicle[50, 50] == pytest.approx(preparation.vehicle.C_vehicle[50, 50] - delta_dpy)

    assert response.pxt_force[offset + 35] == pytest.approx(expected_fz)
    assert response.pxt_force[offset + 20] == pytest.approx(-expected_fz)
    assert response.pxt_force[offset + 44] == pytest.approx(expected_fx)
    assert response.pxt_force[offset + 50] == pytest.approx(expected_fy)
    assert response.pxt_force[offset + 26] == pytest.approx(4000.0)


def test_nonlinear_vehicle_damper_equivalent_force_matches_matrix_switch_identity() -> None:
    preparation = prepare_default_full_case(settings=FullDefaultCaseSettings(cut_freq=50.0))
    rng = np.random.default_rng(1234)
    q = rng.normal(scale=0.02, size=preparation.total_dof)
    v = rng.normal(scale=0.2, size=preparation.total_dof)

    response = build_nonlinear_damper_response(
        preparation.stage_inp_par["Cal"],
        preparation.vehicle_parameters.values,
        preparation.vehicle,
        q,
        v,
    )

    vehicle_slice = slice(preparation.system.layout.n_track, preparation.total_dof)
    base_residual = preparation.vehicle.C_vehicle @ v[vehicle_slice] - response.equivalent_force[vehicle_slice]
    nl_residual = response.c_vehicle @ v[vehicle_slice] - response.pxt_force[vehicle_slice]
    assert np.allclose(base_residual, nl_residual)
