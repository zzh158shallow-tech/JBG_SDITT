from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from .parameters import VehicleParameters


@dataclass(frozen=True)
class VehicleMatrices:
    """Vehicle mass, stiffness, and damping matrices from RW MATLAB model."""

    M_vehicle: np.ndarray
    K_vehicle: np.ndarray
    C_vehicle: np.ndarray
    C_vehicle_linear: np.ndarray

    @property
    def Mlc(self) -> np.ndarray:
        return self.M_vehicle

    @property
    def Klc(self) -> np.ndarray:
        return self.K_vehicle

    @property
    def Clc(self) -> np.ndarray:
        return self.C_vehicle

    @property
    def Clc_0(self) -> np.ndarray:
        return self.C_vehicle_linear


def build_vehicle_matrices_rw_230409(
    parameters: VehicleParameters | Mapping[str, Any],
    *,
    n_rv: int = 51,
) -> VehicleMatrices:
    """Reproduce ``Matrix_Vehicle_RW_230409.m`` for CRH380A vehicle matrices.

    MATLAB uses 1-based DOF indices throughout the original assembly. This port
    deliberately keeps those formulae 1-based at the call sites and translates
    them only in ``_get``/``_set`` to preserve the source script structure.
    """

    p = parameters.values if isinstance(parameters, VehicleParameters) else parameters
    Mw = p["Mw"]
    Jwx = p["Jwx"]
    Jwy = p["Jwy"]
    Jwz = p["Jwz"]
    Mb = p["Mb"]
    Jbx = p["Jbx"]
    Jby = p["Jby"]
    Jbz = p["Jbz"]
    Mc = p["Mc"]
    Jcx = p["Jcx"]
    Jcy = p["Jcy"]
    Jcz = p["Jcz"]
    K1x = p["K1x"]
    C1x = p["C1x"]
    K1y = p["K1y"]
    C1y = p["C1y"]
    K1z = p["K1z"]
    C1z = p["C1z"]
    K2x = p["K2x"]
    C2x = p["C2x"]
    K2y = p["K2y"]
    C2y = p["C2y"]
    K2z = p["K2z"]
    C2z = p["C2z"]
    K_Jx = p["K_Jx"]
    K_Jy = p["K_Jy"]
    K_DPz = p["K_DPz"]
    C_DPz = _mat_scalar(p["C_DPz"], 1)
    K_Sx = p["K_Sx"]
    C_Sx = _mat_scalar(p["C_Sx"], 1)
    K_DPy = p["K_DPy"]
    C_DPy = _mat_scalar(p["C_DPy"], 1)
    K_Tx = p["K_Tx"]
    K_ROTx = p["K_ROTx"]
    Ll1 = p["Ll1"]
    Ll2 = p["Ll2"]
    Ll_Jt = p["Ll_Jt"]
    Ll_Jw = p["Ll_Jw"]
    Ll_DPzt = p["Ll_DPzt"]
    Ll_DPzw = p["Ll_DPzw"]
    Ll_DPyt = p["Ll_DPyt"]
    Ll_ST = p["Ll_ST"]
    Lb1 = p["Lb1"]
    Lb2 = p["Lb2"]
    Lb_J = p["Lb_J"]
    Lb_DPz = p["Lb_DPz"]
    Lb_S = p["Lb_S"]
    H_tw = p["H_tw"]
    H_Bt = p["H_Bt"]
    H_cB = p["H_cB"]
    H_tJ = p["H_tJ"]
    H_Jw = p["H_Jw"]
    H_St = p["H_St"]
    H_cS = p["H_cS"]
    H_Tt = p["H_Tt"]
    H_cT = p["H_cT"]
    H_DPyt = p["H_DPyt"]
    H_cDPy = p["H_cDPy"]
    H_STt = p["H_STt"]
    H_cST = p["H_cST"]
    Mlc = np.zeros((n_rv, n_rv), dtype=float)
    i1 = 0
    while i1<4:
        _set(Mlc, 5*i1+1, 5*i1+1, Mw)
        _set(Mlc, 5*i1+2, 5*i1+2, Mw)
        _set(Mlc, 5*i1+3, 5*i1+3, Jwx)
        _set(Mlc, 5*i1+4, 5*i1+4, Jwy)
        _set(Mlc, 5*i1+5, 5*i1+5, Jwz)
        i1 = i1+1
    i1 = 0
    while i1<2:
        _set(Mlc, 20+5*i1+1, 20+5*i1+1, Mb)
        _set(Mlc, 20+5*i1+2, 20+5*i1+2, Mb)
        _set(Mlc, 20+5*i1+3, 20+5*i1+3, Jbx)
        _set(Mlc, 20+5*i1+4, 20+5*i1+4, Jby)
        _set(Mlc, 20+5*i1+5, 20+5*i1+5, Jbz)
        i1 = i1+1
    _set(Mlc, 30+1, 30+1, Mc)
    _set(Mlc, 30+2, 30+2, Mc)
    _set(Mlc, 30+3, 30+3, Jcx)
    _set(Mlc, 30+4, 30+4, Jcy)
    _set(Mlc, 30+5, 30+5, Jcz)
    for kk in range(36, int(n_rv) + 1):
        _set(Mlc, kk, kk, 1e-8)
    Klc = np.zeros((n_rv, n_rv), dtype=float)
    i1 = 0
    while i1<8:
        i1g1 = math.trunc((i1+1+3)/4)
        i1g2 = math.trunc((i1+1+1)/2)
        i1g3 = math.trunc((3*(i1+1)+1)/2)
        i1g4 = math.trunc((i1+1-1)/2)
        pos_B = 20+5*(i1g1-1)
        pos_W = 5*(i1g2-1)
        _set(Klc, pos_B+1, pos_B+1, _get(Klc, pos_B+1, pos_B+1)+K1z)
        _set(Klc, pos_B+3, pos_B+1, _get(Klc, pos_B+3, pos_B+1)+((-1)**(i1+1))*Lb1*K1z)
        _set(Klc, pos_B+4, pos_B+1, _get(Klc, pos_B+4, pos_B+1)+((-1)**i1g2)*Ll1*K1z)
        _set(Klc, pos_W+1, pos_B+1, _get(Klc, pos_W+1, pos_B+1)-K1z)
        _set(Klc, pos_W+3, pos_B+1, _get(Klc, pos_W+3, pos_B+1)-((-1)**(i1+1))*Lb1*K1z)
        _set(Klc, pos_B+1, pos_W+1, _get(Klc, pos_B+1, pos_W+1)-K1z)
        _set(Klc, pos_B+3, pos_W+1, _get(Klc, pos_B+3, pos_W+1)-((-1)**(i1+1))*Lb1*K1z)
        _set(Klc, pos_B+4, pos_W+1, _get(Klc, pos_B+4, pos_W+1)-((-1)**i1g2)*Ll1*K1z)
        _set(Klc, pos_W+1, pos_W+1, _get(Klc, pos_W+1, pos_W+1)+K1z)
        _set(Klc, pos_W+3, pos_W+1, _get(Klc, pos_W+3, pos_W+1)+((-1)**(i1+1))*Lb1*K1z)
        _set(Klc, pos_B+1, pos_B+3, _get(Klc, pos_B+1, pos_B+3)+((-1)**(i1+1))*Lb1*K1z)
        _set(Klc, pos_B+3, pos_B+3, _get(Klc, pos_B+3, pos_B+3)+Lb1*Lb1*K1z)
        _set(Klc, pos_B+4, pos_B+3, _get(Klc, pos_B+4, pos_B+3)+((-1)**i1g3)*Lb1*Ll1*K1z)
        _set(Klc, pos_W+1, pos_B+3, _get(Klc, pos_W+1, pos_B+3)-((-1)**(i1+1))*Lb1*K1z)
        _set(Klc, pos_W+3, pos_B+3, _get(Klc, pos_W+3, pos_B+3)-Lb1*Lb1*K1z)
        _set(Klc, pos_B+1, pos_B+4, _get(Klc, pos_B+1, pos_B+4)+((-1)**i1g2)*Ll1*K1z)
        _set(Klc, pos_B+3, pos_B+4, _get(Klc, pos_B+3, pos_B+4)+((-1)**i1g3)*Lb1*Ll1*K1z)
        _set(Klc, pos_B+4, pos_B+4, _get(Klc, pos_B+4, pos_B+4)+Ll1*Ll1*K1z)
        _set(Klc, pos_W+1, pos_B+4, _get(Klc, pos_W+1, pos_B+4)-((-1)**i1g2)*Ll1*K1z)
        _set(Klc, pos_W+3, pos_B+4, _get(Klc, pos_W+3, pos_B+4)-((-1)**i1g3)*Lb1*Ll1*K1z)
        _set(Klc, pos_B+1, pos_W+3, _get(Klc, pos_B+1, pos_W+3)-((-1)**(i1+1))*Lb1*K1z)
        _set(Klc, pos_B+3, pos_W+3, _get(Klc, pos_B+3, pos_W+3)-Lb1*Lb1*K1z)
        _set(Klc, pos_B+4, pos_W+3, _get(Klc, pos_B+4, pos_W+3)-((-1)**i1g3)*Lb1*Ll1*K1z)
        _set(Klc, pos_W+1, pos_W+3, _get(Klc, pos_W+1, pos_W+3)+((-1)**(i1+1))*Lb1*K1z)
        _set(Klc, pos_W+3, pos_W+3, _get(Klc, pos_W+3, pos_W+3)+Lb1*Lb1*K1z)
        _set(Klc, pos_B+2, pos_B+2, _get(Klc, pos_B+2, pos_B+2)+K1y)
        _set(Klc, pos_B+2, pos_B+3, _get(Klc, pos_B+2, pos_B+3)-H_tw*K1y)
        _set(Klc, pos_B+2, pos_B+5, _get(Klc, pos_B+2, pos_B+5)+((-1)**i1g4)*Ll1*K1y)
        _set(Klc, pos_B+2, pos_W+2, _get(Klc, pos_B+2, pos_W+2)-K1y)
        _set(Klc, pos_B+3, pos_B+2, _get(Klc, pos_B+3, pos_B+2)-H_tw*K1y)
        _set(Klc, pos_B+3, pos_B+3, _get(Klc, pos_B+3, pos_B+3)+H_tw*H_tw*K1y)
        _set(Klc, pos_B+3, pos_B+5, _get(Klc, pos_B+3, pos_B+5)-((-1)**i1g4)*Ll1*H_tw*K1y)
        _set(Klc, pos_B+3, pos_W+2, _get(Klc, pos_B+3, pos_W+2)+H_tw*K1y)
        _set(Klc, pos_B+5, pos_B+2, _get(Klc, pos_B+5, pos_B+2)+((-1)**i1g4)*Ll1*K1y)
        _set(Klc, pos_B+5, pos_B+3, _get(Klc, pos_B+5, pos_B+3)-((-1)**i1g4)*Ll1*H_tw*K1y)
        _set(Klc, pos_B+5, pos_B+5, _get(Klc, pos_B+5, pos_B+5)+Ll1*Ll1*K1y)
        _set(Klc, pos_B+5, pos_W+2, _get(Klc, pos_B+5, pos_W+2)-((-1)**i1g4)*Ll1*K1y)
        _set(Klc, pos_W+2, pos_B+2, _get(Klc, pos_W+2, pos_B+2)-K1y)
        _set(Klc, pos_W+2, pos_B+3, _get(Klc, pos_W+2, pos_B+3)+H_tw*K1y)
        _set(Klc, pos_W+2, pos_B+5, _get(Klc, pos_W+2, pos_B+5)-((-1)**i1g4)*Ll1*K1y)
        _set(Klc, pos_W+2, pos_W+2, _get(Klc, pos_W+2, pos_W+2)+K1y)
        _set(Klc, pos_B+4, pos_B+4, _get(Klc, pos_B+4, pos_B+4)+H_tw*H_tw*K1x)
        _set(Klc, pos_B+4, pos_B+5, _get(Klc, pos_B+4, pos_B+5)+((-1)**(i1+1+1))*Lb1*H_tw*K1x)
        _set(Klc, pos_B+4, pos_W+5, _get(Klc, pos_B+4, pos_W+5)-((-1)**(i1+1+1))*Lb1*H_tw*K1x)
        _set(Klc, pos_B+5, pos_B+4, _get(Klc, pos_B+5, pos_B+4)+((-1)**(i1+1+1))*Lb1*H_tw*K1x)
        _set(Klc, pos_B+5, pos_B+5, _get(Klc, pos_B+5, pos_B+5)+Lb1*Lb1*K1x)
        _set(Klc, pos_B+5, pos_W+5, _get(Klc, pos_B+5, pos_W+5)-Lb1*Lb1*K1x)
        _set(Klc, pos_W+5, pos_B+4, _get(Klc, pos_W+5, pos_B+4)-((-1)**(i1+1+1))*Lb1*H_tw*K1x)
        _set(Klc, pos_W+5, pos_B+5, _get(Klc, pos_W+5, pos_B+5)-Lb1*Lb1*K1x)
        _set(Klc, pos_W+5, pos_W+5, _get(Klc, pos_W+5, pos_W+5)+Lb1*Lb1*K1x)
        i1 = i1+1
    i1 = 0
    while i1<4:
        i1g2 = math.trunc((i1+1+1)/2)
        i1g3 = math.trunc((3*(i1+1)+1)/2)
        i1g4 = math.trunc((i1+1-1)/2)
        pos_BG = 20+5*(i1g2-1)
        pos_CB = 30
        _set(Klc, pos_CB+1, pos_CB+1, _get(Klc, pos_CB+1, pos_CB+1)+K2z)
        _set(Klc, pos_CB+3, pos_CB+1, _get(Klc, pos_CB+3, pos_CB+1)+((-1)**(i1+1))*Lb2*K2z)
        _set(Klc, pos_CB+4, pos_CB+1, _get(Klc, pos_CB+4, pos_CB+1)+((-1)**i1g2)*Ll2*K2z)
        _set(Klc, pos_BG+1, pos_CB+1, _get(Klc, pos_BG+1, pos_CB+1)-K2z)
        _set(Klc, pos_BG+3, pos_CB+1, _get(Klc, pos_BG+3, pos_CB+1)-((-1)**(i1+1))*Lb2*K2z)
        _set(Klc, pos_CB+1, pos_CB+3, _get(Klc, pos_CB+1, pos_CB+3)+((-1)**(i1+1))*Lb2*K2z)
        _set(Klc, pos_CB+3, pos_CB+3, _get(Klc, pos_CB+3, pos_CB+3)+Lb2*Lb2*K2z)
        _set(Klc, pos_CB+4, pos_CB+3, _get(Klc, pos_CB+4, pos_CB+3)+((-1)**i1g3)*Lb2*Ll2*K2z)
        _set(Klc, pos_BG+1, pos_CB+3, _get(Klc, pos_BG+1, pos_CB+3)-((-1)**(i1+1))*Lb2*K2z)
        _set(Klc, pos_BG+3, pos_CB+3, _get(Klc, pos_BG+3, pos_CB+3)-Lb2*Lb2*K2z)
        _set(Klc, pos_CB+1, pos_CB+4, _get(Klc, pos_CB+1, pos_CB+4)+((-1)**i1g2)*Ll2*K2z)
        _set(Klc, pos_CB+3, pos_CB+4, _get(Klc, pos_CB+3, pos_CB+4)+((-1)**i1g3)*Lb2*Ll2*K2z)
        _set(Klc, pos_CB+4, pos_CB+4, _get(Klc, pos_CB+4, pos_CB+4)+Ll2*Ll2*K2z)
        _set(Klc, pos_BG+1, pos_CB+4, _get(Klc, pos_BG+1, pos_CB+4)-((-1)**i1g2)*Ll2*K2z)
        _set(Klc, pos_BG+3, pos_CB+4, _get(Klc, pos_BG+3, pos_CB+4)-((-1)**i1g3)*Lb2*Ll2*K2z)
        _set(Klc, pos_CB+1, pos_BG+1, _get(Klc, pos_CB+1, pos_BG+1)-K2z)
        _set(Klc, pos_CB+3, pos_BG+1, _get(Klc, pos_CB+3, pos_BG+1)-((-1)**(i1+1))*Lb2*K2z)
        _set(Klc, pos_CB+4, pos_BG+1, _get(Klc, pos_CB+4, pos_BG+1)-((-1)**i1g2)*Ll2*K2z)
        _set(Klc, pos_BG+1, pos_BG+1, _get(Klc, pos_BG+1, pos_BG+1)+K2z)
        _set(Klc, pos_BG+3, pos_BG+1, _get(Klc, pos_BG+3, pos_BG+1)+((-1)**(i1+1))*Lb2*K2z)
        _set(Klc, pos_CB+1, pos_BG+3, _get(Klc, pos_CB+1, pos_BG+3)-((-1)**(i1+1))*Lb2*K2z)
        _set(Klc, pos_CB+3, pos_BG+3, _get(Klc, pos_CB+3, pos_BG+3)-Lb2*Lb2*K2z)
        _set(Klc, pos_CB+4, pos_BG+3, _get(Klc, pos_CB+4, pos_BG+3)-((-1)**i1g3)*Lb2*Ll2*K2z)
        _set(Klc, pos_BG+1, pos_BG+3, _get(Klc, pos_BG+1, pos_BG+3)+((-1)**(i1+1))*Lb2*K2z)
        _set(Klc, pos_BG+3, pos_BG+3, _get(Klc, pos_BG+3, pos_BG+3)+Lb2*Lb2*K2z)
        _set(Klc, pos_CB+2, pos_CB+2, _get(Klc, pos_CB+2, pos_CB+2)+K2y)
        _set(Klc, pos_CB+2, pos_CB+3, _get(Klc, pos_CB+2, pos_CB+3)-H_cB*K2y)
        _set(Klc, pos_CB+2, pos_CB+5, _get(Klc, pos_CB+2, pos_CB+5)+((-1)**i1g4)*Ll2*K2y)
        _set(Klc, pos_CB+2, pos_BG+2, _get(Klc, pos_CB+2, pos_BG+2)-K2y)
        _set(Klc, pos_CB+2, pos_BG+3, _get(Klc, pos_CB+2, pos_BG+3)-H_Bt*K2y)
        _set(Klc, pos_CB+3, pos_CB+2, _get(Klc, pos_CB+3, pos_CB+2)-H_cB*K2y)
        _set(Klc, pos_CB+3, pos_CB+3, _get(Klc, pos_CB+3, pos_CB+3)+H_cB*H_cB*K2y)
        _set(Klc, pos_CB+3, pos_CB+5, _get(Klc, pos_CB+3, pos_CB+5)-((-1)**i1g4)*Ll2*H_cB*K2y)
        _set(Klc, pos_CB+3, pos_BG+2, _get(Klc, pos_CB+3, pos_BG+2)+H_cB*K2y)
        _set(Klc, pos_CB+3, pos_BG+3, _get(Klc, pos_CB+3, pos_BG+3)+H_cB*H_Bt*K2y)
        _set(Klc, pos_CB+5, pos_CB+2, _get(Klc, pos_CB+5, pos_CB+2)+((-1)**i1g4)*Ll2*K2y)
        _set(Klc, pos_CB+5, pos_CB+3, _get(Klc, pos_CB+5, pos_CB+3)-((-1)**i1g4)*Ll2*H_cB*K2y)
        _set(Klc, pos_CB+5, pos_CB+5, _get(Klc, pos_CB+5, pos_CB+5)+Ll2*Ll2*K2y)
        _set(Klc, pos_CB+5, pos_BG+2, _get(Klc, pos_CB+5, pos_BG+2)-((-1)**i1g4)*Ll2*K2y)
        _set(Klc, pos_CB+5, pos_BG+3, _get(Klc, pos_CB+5, pos_BG+3)-((-1)**i1g4)*Ll2*H_Bt*K2y)
        _set(Klc, pos_BG+2, pos_CB+2, _get(Klc, pos_BG+2, pos_CB+2)-K2y)
        _set(Klc, pos_BG+2, pos_CB+3, _get(Klc, pos_BG+2, pos_CB+3)+H_cB*K2y)
        _set(Klc, pos_BG+2, pos_CB+5, _get(Klc, pos_BG+2, pos_CB+5)-((-1)**i1g4)*Ll2*K2y)
        _set(Klc, pos_BG+2, pos_BG+2, _get(Klc, pos_BG+2, pos_BG+2)+K2y)
        _set(Klc, pos_BG+2, pos_BG+3, _get(Klc, pos_BG+2, pos_BG+3)+H_Bt*K2y)
        _set(Klc, pos_BG+3, pos_CB+2, _get(Klc, pos_BG+3, pos_CB+2)-H_Bt*K2y)
        _set(Klc, pos_BG+3, pos_CB+3, _get(Klc, pos_BG+3, pos_CB+3)+H_cB*H_Bt*K2y)
        _set(Klc, pos_BG+3, pos_CB+5, _get(Klc, pos_BG+3, pos_CB+5)-((-1)**i1g4)*Ll2*H_Bt*K2y)
        _set(Klc, pos_BG+3, pos_BG+2, _get(Klc, pos_BG+3, pos_BG+2)+H_Bt*K2y)
        _set(Klc, pos_BG+3, pos_BG+3, _get(Klc, pos_BG+3, pos_BG+3)+H_Bt*H_Bt*K2y)
        _set(Klc, pos_CB+4, pos_CB+4, _get(Klc, pos_CB+4, pos_CB+4)+H_cB*H_cB*K2x)
        _set(Klc, pos_CB+4, pos_CB+5, _get(Klc, pos_CB+4, pos_CB+5)+((-1)**(i1+1+1))*Lb2*H_cB*K2x)
        _set(Klc, pos_CB+4, pos_BG+5, _get(Klc, pos_CB+4, pos_BG+5)-((-1)**(i1+1+1))*Lb2*H_cB*K2x)
        _set(Klc, pos_CB+4, pos_BG+4, _get(Klc, pos_CB+4, pos_BG+4)+H_cB*H_Bt*K2x)
        _set(Klc, pos_CB+5, pos_CB+4, _get(Klc, pos_CB+5, pos_CB+4)+((-1)**(i1+1+1))*Lb2*H_cB*K2x)
        _set(Klc, pos_CB+5, pos_CB+5, _get(Klc, pos_CB+5, pos_CB+5)+Lb2*Lb2*K2x)
        _set(Klc, pos_CB+5, pos_BG+5, _get(Klc, pos_CB+5, pos_BG+5)-Lb2*Lb2*K2x)
        _set(Klc, pos_CB+5, pos_BG+4, _get(Klc, pos_CB+5, pos_BG+4)+((-1)**(i1+1+1))*Lb2*H_Bt*K2x)
        _set(Klc, pos_BG+5, pos_CB+4, _get(Klc, pos_BG+5, pos_CB+4)-((-1)**(i1+1+1))*Lb2*H_cB*K2x)
        _set(Klc, pos_BG+5, pos_CB+5, _get(Klc, pos_BG+5, pos_CB+5)-Lb2*Lb2*K2x)
        _set(Klc, pos_BG+5, pos_BG+5, _get(Klc, pos_BG+5, pos_BG+5)+Lb2*Lb2*K2x)
        _set(Klc, pos_BG+5, pos_BG+4, _get(Klc, pos_BG+5, pos_BG+4)-((-1)**(i1+1+1))*Lb2*H_Bt*K2x)
        _set(Klc, pos_BG+4, pos_CB+4, _get(Klc, pos_BG+4, pos_CB+4)+H_cB*H_Bt*K2x)
        _set(Klc, pos_BG+4, pos_CB+5, _get(Klc, pos_BG+4, pos_CB+5)+((-1)**(i1+1+1))*Lb2*H_Bt*K2x)
        _set(Klc, pos_BG+4, pos_BG+5, _get(Klc, pos_BG+4, pos_BG+5)-((-1)**(i1+1+1))*Lb2*H_Bt*K2x)
        _set(Klc, pos_BG+4, pos_BG+4, _get(Klc, pos_BG+4, pos_BG+4)+H_Bt*H_Bt*K2x)
        i1 = i1+1
    Clc = np.zeros((n_rv, n_rv), dtype=float)
    i1 = 0
    while i1<8:
        i1g1 = math.trunc((i1+1+3)/4)
        i1g2 = math.trunc((i1+1+1)/2)
        i1g3 = math.trunc((3*(i1+1)+1)/2)
        i1g4 = math.trunc((i1+1-1)/2)
        pos_B = 20+5*(i1g1-1)
        pos_W = 5*(i1g2-1)
        _set(Clc, pos_B+1, pos_B+1, _get(Clc, pos_B+1, pos_B+1)+C1z)
        _set(Clc, pos_B+3, pos_B+1, _get(Clc, pos_B+3, pos_B+1)+((-1)**(i1+1))*Lb1*C1z)
        _set(Clc, pos_B+4, pos_B+1, _get(Clc, pos_B+4, pos_B+1)+((-1)**i1g2)*Ll1*C1z)
        _set(Clc, pos_W+1, pos_B+1, _get(Clc, pos_W+1, pos_B+1)-C1z)
        _set(Clc, pos_W+3, pos_B+1, _get(Clc, pos_W+3, pos_B+1)-((-1)**(i1+1))*Lb1*C1z)
        _set(Clc, pos_B+1, pos_W+1, _get(Clc, pos_B+1, pos_W+1)-C1z)
        _set(Clc, pos_B+3, pos_W+1, _get(Clc, pos_B+3, pos_W+1)-((-1)**(i1+1))*Lb1*C1z)
        _set(Clc, pos_B+4, pos_W+1, _get(Clc, pos_B+4, pos_W+1)-((-1)**i1g2)*Ll1*C1z)
        _set(Clc, pos_W+1, pos_W+1, _get(Clc, pos_W+1, pos_W+1)+C1z)
        _set(Clc, pos_W+3, pos_W+1, _get(Clc, pos_W+3, pos_W+1)+((-1)**(i1+1))*Lb1*C1z)
        _set(Clc, pos_B+1, pos_B+3, _get(Clc, pos_B+1, pos_B+3)+((-1)**(i1+1))*Lb1*C1z)
        _set(Clc, pos_B+3, pos_B+3, _get(Clc, pos_B+3, pos_B+3)+Lb1*Lb1*C1z)
        _set(Clc, pos_B+4, pos_B+3, _get(Clc, pos_B+4, pos_B+3)+((-1)**i1g3)*Lb1*Ll1*C1z)
        _set(Clc, pos_W+1, pos_B+3, _get(Clc, pos_W+1, pos_B+3)-((-1)**(i1+1))*Lb1*C1z)
        _set(Clc, pos_W+3, pos_B+3, _get(Clc, pos_W+3, pos_B+3)-Lb1*Lb1*C1z)
        _set(Clc, pos_B+1, pos_B+4, _get(Clc, pos_B+1, pos_B+4)+((-1)**i1g2)*Ll1*C1z)
        _set(Clc, pos_B+3, pos_B+4, _get(Clc, pos_B+3, pos_B+4)+((-1)**i1g3)*Lb1*Ll1*C1z)
        _set(Clc, pos_B+4, pos_B+4, _get(Clc, pos_B+4, pos_B+4)+Ll1*Ll1*C1z)
        _set(Clc, pos_W+1, pos_B+4, _get(Clc, pos_W+1, pos_B+4)-((-1)**i1g2)*Ll1*C1z)
        _set(Clc, pos_W+3, pos_B+4, _get(Clc, pos_W+3, pos_B+4)-((-1)**i1g3)*Lb1*Ll1*C1z)
        _set(Clc, pos_B+1, pos_W+3, _get(Clc, pos_B+1, pos_W+3)-((-1)**(i1+1))*Lb1*C1z)
        _set(Clc, pos_B+3, pos_W+3, _get(Clc, pos_B+3, pos_W+3)-Lb1*Lb1*C1z)
        _set(Clc, pos_B+4, pos_W+3, _get(Clc, pos_B+4, pos_W+3)-((-1)**i1g3)*Lb1*Ll1*C1z)
        _set(Clc, pos_W+1, pos_W+3, _get(Clc, pos_W+1, pos_W+3)+((-1)**(i1+1))*Lb1*C1z)
        _set(Clc, pos_W+3, pos_W+3, _get(Clc, pos_W+3, pos_W+3)+Lb1*Lb1*C1z)
        _set(Clc, pos_B+2, pos_B+2, _get(Clc, pos_B+2, pos_B+2)+C1y)
        _set(Clc, pos_B+2, pos_B+3, _get(Clc, pos_B+2, pos_B+3)-H_tw*C1y)
        _set(Clc, pos_B+2, pos_B+5, _get(Clc, pos_B+2, pos_B+5)+((-1)**i1g4)*Ll1*C1y)
        _set(Clc, pos_B+2, pos_W+2, _get(Clc, pos_B+2, pos_W+2)-C1y)
        _set(Clc, pos_B+3, pos_B+2, _get(Clc, pos_B+3, pos_B+2)-H_tw*C1y)
        _set(Clc, pos_B+3, pos_B+3, _get(Clc, pos_B+3, pos_B+3)+H_tw*H_tw*C1y)
        _set(Clc, pos_B+3, pos_B+5, _get(Clc, pos_B+3, pos_B+5)-((-1)**i1g4)*Ll1*H_tw*C1y)
        _set(Clc, pos_B+3, pos_W+2, _get(Clc, pos_B+3, pos_W+2)+H_tw*C1y)
        _set(Clc, pos_B+5, pos_B+2, _get(Clc, pos_B+5, pos_B+2)+((-1)**i1g4)*Ll1*C1y)
        _set(Clc, pos_B+5, pos_B+3, _get(Clc, pos_B+5, pos_B+3)-((-1)**i1g4)*Ll1*H_tw*C1y)
        _set(Clc, pos_B+5, pos_B+5, _get(Clc, pos_B+5, pos_B+5)+Ll1*Ll1*C1y)
        _set(Clc, pos_B+5, pos_W+2, _get(Clc, pos_B+5, pos_W+2)-((-1)**i1g4)*Ll1*C1y)
        _set(Clc, pos_W+2, pos_B+2, _get(Clc, pos_W+2, pos_B+2)-C1y)
        _set(Clc, pos_W+2, pos_B+3, _get(Clc, pos_W+2, pos_B+3)+H_tw*C1y)
        _set(Clc, pos_W+2, pos_B+5, _get(Clc, pos_W+2, pos_B+5)-((-1)**i1g4)*Ll1*C1y)
        _set(Clc, pos_W+2, pos_W+2, _get(Clc, pos_W+2, pos_W+2)+C1y)
        _set(Clc, pos_B+4, pos_B+4, _get(Clc, pos_B+4, pos_B+4)+H_tw*H_tw*C1x)
        _set(Clc, pos_B+4, pos_B+5, _get(Clc, pos_B+4, pos_B+5)+((-1)**(i1+1+1))*Lb1*H_tw*C1x)
        _set(Clc, pos_B+4, pos_W+5, _get(Clc, pos_B+4, pos_W+5)-((-1)**(i1+1+1))*Lb1*H_tw*C1x)
        _set(Clc, pos_B+5, pos_B+4, _get(Clc, pos_B+5, pos_B+4)+((-1)**(i1+1+1))*Lb1*H_tw*C1x)
        _set(Clc, pos_B+5, pos_B+5, _get(Clc, pos_B+5, pos_B+5)+Lb1*Lb1*C1x)
        _set(Clc, pos_B+5, pos_W+5, _get(Clc, pos_B+5, pos_W+5)-Lb1*Lb1*C1x)
        _set(Clc, pos_W+5, pos_B+4, _get(Clc, pos_W+5, pos_B+4)-((-1)**(i1+1+1))*Lb1*H_tw*C1x)
        _set(Clc, pos_W+5, pos_B+5, _get(Clc, pos_W+5, pos_B+5)-Lb1*Lb1*C1x)
        _set(Clc, pos_W+5, pos_W+5, _get(Clc, pos_W+5, pos_W+5)+Lb1*Lb1*C1x)
        i1 = i1+1
    i1 = 0
    while i1<4:
        i1g2 = math.trunc((i1+1+1)/2)
        i1g3 = math.trunc((3*(i1+1)+1)/2)
        i1g4 = math.trunc((i1+1-1)/2)
        pos_BG = 20+5*(i1g2-1)
        pos_CB = 30
        _set(Clc, pos_CB+1, pos_CB+1, _get(Clc, pos_CB+1, pos_CB+1)+C2z)
        _set(Clc, pos_CB+3, pos_CB+1, _get(Clc, pos_CB+3, pos_CB+1)+((-1)**(i1+1))*Lb2*C2z)
        _set(Clc, pos_CB+4, pos_CB+1, _get(Clc, pos_CB+4, pos_CB+1)+((-1)**i1g2)*Ll2*C2z)
        _set(Clc, pos_BG+1, pos_CB+1, _get(Clc, pos_BG+1, pos_CB+1)-C2z)
        _set(Clc, pos_BG+3, pos_CB+1, _get(Clc, pos_BG+3, pos_CB+1)-((-1)**(i1+1))*Lb2*C2z)
        _set(Clc, pos_CB+1, pos_CB+3, _get(Clc, pos_CB+1, pos_CB+3)+((-1)**(i1+1))*Lb2*C2z)
        _set(Clc, pos_CB+3, pos_CB+3, _get(Clc, pos_CB+3, pos_CB+3)+Lb2*Lb2*C2z)
        _set(Clc, pos_CB+4, pos_CB+3, _get(Clc, pos_CB+4, pos_CB+3)+((-1)**i1g3)*Lb2*Ll2*C2z)
        _set(Clc, pos_BG+1, pos_CB+3, _get(Clc, pos_BG+1, pos_CB+3)-((-1)**(i1+1))*Lb2*C2z)
        _set(Clc, pos_BG+3, pos_CB+3, _get(Clc, pos_BG+3, pos_CB+3)-Lb2*Lb2*C2z)
        _set(Clc, pos_CB+1, pos_CB+4, _get(Clc, pos_CB+1, pos_CB+4)+((-1)**i1g2)*Ll2*C2z)
        _set(Clc, pos_CB+3, pos_CB+4, _get(Clc, pos_CB+3, pos_CB+4)+((-1)**i1g3)*Lb2*Ll2*C2z)
        _set(Clc, pos_CB+4, pos_CB+4, _get(Clc, pos_CB+4, pos_CB+4)+Ll2*Ll2*C2z)
        _set(Clc, pos_BG+1, pos_CB+4, _get(Clc, pos_BG+1, pos_CB+4)-((-1)**i1g2)*Ll2*C2z)
        _set(Clc, pos_BG+3, pos_CB+4, _get(Clc, pos_BG+3, pos_CB+4)-((-1)**i1g3)*Lb2*Ll2*C2z)
        _set(Clc, pos_CB+1, pos_BG+1, _get(Clc, pos_CB+1, pos_BG+1)-C2z)
        _set(Clc, pos_CB+3, pos_BG+1, _get(Clc, pos_CB+3, pos_BG+1)-((-1)**(i1+1))*Lb2*C2z)
        _set(Clc, pos_CB+4, pos_BG+1, _get(Clc, pos_CB+4, pos_BG+1)-((-1)**i1g2)*Ll2*C2z)
        _set(Clc, pos_BG+1, pos_BG+1, _get(Clc, pos_BG+1, pos_BG+1)+C2z)
        _set(Clc, pos_BG+3, pos_BG+1, _get(Clc, pos_BG+3, pos_BG+1)+((-1)**(i1+1))*Lb2*C2z)
        _set(Clc, pos_CB+1, pos_BG+3, _get(Clc, pos_CB+1, pos_BG+3)-((-1)**(i1+1))*Lb2*C2z)
        _set(Clc, pos_CB+3, pos_BG+3, _get(Clc, pos_CB+3, pos_BG+3)-Lb2*Lb2*C2z)
        _set(Clc, pos_CB+4, pos_BG+3, _get(Clc, pos_CB+4, pos_BG+3)-((-1)**i1g3)*Lb2*Ll2*C2z)
        _set(Clc, pos_BG+1, pos_BG+3, _get(Clc, pos_BG+1, pos_BG+3)+((-1)**(i1+1))*Lb2*C2z)
        _set(Clc, pos_BG+3, pos_BG+3, _get(Clc, pos_BG+3, pos_BG+3)+Lb2*Lb2*C2z)
        _set(Clc, pos_CB+2, pos_CB+2, _get(Clc, pos_CB+2, pos_CB+2)+C2y)
        _set(Clc, pos_CB+2, pos_CB+3, _get(Clc, pos_CB+2, pos_CB+3)-H_cB*C2y)
        _set(Clc, pos_CB+2, pos_CB+5, _get(Clc, pos_CB+2, pos_CB+5)+((-1)**i1g4)*Ll2*C2y)
        _set(Clc, pos_CB+2, pos_BG+2, _get(Clc, pos_CB+2, pos_BG+2)-C2y)
        _set(Clc, pos_CB+2, pos_BG+3, _get(Clc, pos_CB+2, pos_BG+3)-H_Bt*C2y)
        _set(Clc, pos_CB+3, pos_CB+2, _get(Clc, pos_CB+3, pos_CB+2)-H_cB*C2y)
        _set(Clc, pos_CB+3, pos_CB+3, _get(Clc, pos_CB+3, pos_CB+3)+H_cB*H_cB*C2y)
        _set(Clc, pos_CB+3, pos_CB+5, _get(Clc, pos_CB+3, pos_CB+5)-((-1)**i1g4)*Ll2*H_cB*C2y)
        _set(Clc, pos_CB+3, pos_BG+2, _get(Clc, pos_CB+3, pos_BG+2)+H_cB*C2y)
        _set(Clc, pos_CB+3, pos_BG+3, _get(Clc, pos_CB+3, pos_BG+3)+H_cB*H_Bt*C2y)
        _set(Clc, pos_CB+5, pos_CB+2, _get(Clc, pos_CB+5, pos_CB+2)+((-1)**i1g4)*Ll2*C2y)
        _set(Clc, pos_CB+5, pos_CB+3, _get(Clc, pos_CB+5, pos_CB+3)-((-1)**i1g4)*Ll2*H_cB*C2y)
        _set(Clc, pos_CB+5, pos_CB+5, _get(Clc, pos_CB+5, pos_CB+5)+Ll2*Ll2*C2y)
        _set(Clc, pos_CB+5, pos_BG+2, _get(Clc, pos_CB+5, pos_BG+2)-((-1)**i1g4)*Ll2*C2y)
        _set(Clc, pos_CB+5, pos_BG+3, _get(Clc, pos_CB+5, pos_BG+3)-((-1)**i1g4)*Ll2*H_Bt*C2y)
        _set(Clc, pos_BG+2, pos_CB+2, _get(Clc, pos_BG+2, pos_CB+2)-C2y)
        _set(Clc, pos_BG+2, pos_CB+3, _get(Clc, pos_BG+2, pos_CB+3)+H_cB*C2y)
        _set(Clc, pos_BG+2, pos_CB+5, _get(Clc, pos_BG+2, pos_CB+5)-((-1)**i1g4)*Ll2*C2y)
        _set(Clc, pos_BG+2, pos_BG+2, _get(Clc, pos_BG+2, pos_BG+2)+C2y)
        _set(Clc, pos_BG+2, pos_BG+3, _get(Clc, pos_BG+2, pos_BG+3)+H_Bt*C2y)
        _set(Clc, pos_BG+3, pos_CB+2, _get(Clc, pos_BG+3, pos_CB+2)-H_Bt*C2y)
        _set(Clc, pos_BG+3, pos_CB+3, _get(Clc, pos_BG+3, pos_CB+3)+H_cB*H_Bt*C2y)
        _set(Clc, pos_BG+3, pos_CB+5, _get(Clc, pos_BG+3, pos_CB+5)-((-1)**i1g4)*Ll2*H_Bt*C2y)
        _set(Clc, pos_BG+3, pos_BG+2, _get(Clc, pos_BG+3, pos_BG+2)+H_Bt*C2y)
        _set(Clc, pos_BG+3, pos_BG+3, _get(Clc, pos_BG+3, pos_BG+3)+H_Bt*H_Bt*C2y)
        _set(Clc, pos_CB+4, pos_CB+4, _get(Clc, pos_CB+4, pos_CB+4)+H_cB*H_cB*C2x)
        _set(Clc, pos_CB+4, pos_CB+5, _get(Clc, pos_CB+4, pos_CB+5)+((-1)**(i1+1+1))*Lb2*H_cB*C2x)
        _set(Clc, pos_CB+4, pos_BG+5, _get(Clc, pos_CB+4, pos_BG+5)-((-1)**(i1+1+1))*Lb2*H_cB*C2x)
        _set(Clc, pos_CB+4, pos_BG+4, _get(Clc, pos_CB+4, pos_BG+4)+H_cB*H_Bt*C2x)
        _set(Clc, pos_CB+5, pos_CB+4, _get(Clc, pos_CB+5, pos_CB+4)+((-1)**(i1+1+1))*Lb2*H_cB*C2x)
        _set(Clc, pos_CB+5, pos_CB+5, _get(Clc, pos_CB+5, pos_CB+5)+Lb2*Lb2*C2x)
        _set(Clc, pos_CB+5, pos_BG+5, _get(Clc, pos_CB+5, pos_BG+5)-Lb2*Lb2*C2x)
        _set(Clc, pos_CB+5, pos_BG+4, _get(Clc, pos_CB+5, pos_BG+4)+((-1)**(i1+1+1))*Lb2*H_Bt*C2x)
        _set(Clc, pos_BG+5, pos_CB+4, _get(Clc, pos_BG+5, pos_CB+4)-((-1)**(i1+1+1))*Lb2*H_cB*C2x)
        _set(Clc, pos_BG+5, pos_CB+5, _get(Clc, pos_BG+5, pos_CB+5)-Lb2*Lb2*C2x)
        _set(Clc, pos_BG+5, pos_BG+5, _get(Clc, pos_BG+5, pos_BG+5)+Lb2*Lb2*C2x)
        _set(Clc, pos_BG+5, pos_BG+4, _get(Clc, pos_BG+5, pos_BG+4)-((-1)**(i1+1+1))*Lb2*H_Bt*C2x)
        _set(Clc, pos_BG+4, pos_CB+4, _get(Clc, pos_BG+4, pos_CB+4)+H_cB*H_Bt*C2x)
        _set(Clc, pos_BG+4, pos_CB+5, _get(Clc, pos_BG+4, pos_CB+5)+((-1)**(i1+1+1))*Lb2*H_Bt*C2x)
        _set(Clc, pos_BG+4, pos_BG+5, _get(Clc, pos_BG+4, pos_BG+5)-((-1)**(i1+1+1))*Lb2*H_Bt*C2x)
        _set(Clc, pos_BG+4, pos_BG+4, _get(Clc, pos_BG+4, pos_BG+4)+H_Bt*H_Bt*C2x)
        i1 = i1+1
    Clc_0 = Clc.copy()
    for i1 in range(1, 3):
        for i2 in range(1, 3):
            for i3 in range(1, 3):
                pos_W = 10*(i1-1)+5*(i2-1)
                pos_B = 20+5*(i1-1)
                _set(Klc, pos_B+4, pos_B+4, _get(Klc, pos_B+4, pos_B+4) + H_tJ*H_tJ*K_Jx)
                _set(Klc, pos_B+5, pos_B+4, _get(Klc, pos_B+5, pos_B+4) + H_tJ*((-1)**(i3+1))*Lb_J*K_Jx)
                _set(Klc, pos_W+4, pos_B+4, _get(Klc, pos_W+4, pos_B+4) + H_tJ*H_Jw*K_Jx)
                _set(Klc, pos_W+5, pos_B+4, _get(Klc, pos_W+5, pos_B+4) - H_tJ*((-1)**(i3+1))*Lb_J*K_Jx)
                _set(Klc, pos_B+4, pos_B+5, _get(Klc, pos_B+4, pos_B+5) + ((-1)**(i3+1))*Lb_J*H_tJ*K_Jx)
                _set(Klc, pos_B+5, pos_B+5, _get(Klc, pos_B+5, pos_B+5) + Lb_J*Lb_J*K_Jx)
                _set(Klc, pos_W+4, pos_B+5, _get(Klc, pos_W+4, pos_B+5) + ((-1)**(i3+1))*Lb_J*H_Jw*K_Jx)
                _set(Klc, pos_W+5, pos_B+5, _get(Klc, pos_W+5, pos_B+5) - Lb_J*Lb_J*K_Jx)
                _set(Klc, pos_B+4, pos_W+4, _get(Klc, pos_B+4, pos_W+4) + H_Jw*H_tJ*K_Jx)
                _set(Klc, pos_B+5, pos_W+4, _get(Klc, pos_B+5, pos_W+4) + H_Jw*((-1)**(i3+1))*Lb_J*K_Jx)
                _set(Klc, pos_W+4, pos_W+4, _get(Klc, pos_W+4, pos_W+4) + H_Jw*H_Jw*K_Jx)
                _set(Klc, pos_W+5, pos_W+4, _get(Klc, pos_W+5, pos_W+4) - H_Jw*((-1)**(i3+1))*Lb_J*K_Jx)
                _set(Klc, pos_B+4, pos_W+5, _get(Klc, pos_B+4, pos_W+5) - ((-1)**(i3+1))*Lb_J*H_tJ*K_Jx)
                _set(Klc, pos_B+5, pos_W+5, _get(Klc, pos_B+5, pos_W+5) - Lb_J*Lb_J*K_Jx)
                _set(Klc, pos_W+4, pos_W+5, _get(Klc, pos_W+4, pos_W+5) - ((-1)**(i3+1))*Lb_J*H_Jw*K_Jx)
                _set(Klc, pos_W+5, pos_W+5, _get(Klc, pos_W+5, pos_W+5) + Lb_J*Lb_J*K_Jx)
                _set(Klc, pos_B+2, pos_B+2, _get(Klc, pos_B+2, pos_B+2) + K_Jy)
                _set(Klc, pos_B+3, pos_B+2, _get(Klc, pos_B+3, pos_B+2) - H_tJ*K_Jy)
                _set(Klc, pos_B+5, pos_B+2, _get(Klc, pos_B+5, pos_B+2) + ((-1)**(i2+1))*Ll_Jt*K_Jy)
                _set(Klc, pos_W+2, pos_B+2, _get(Klc, pos_W+2, pos_B+2) - K_Jy)
                _set(Klc, pos_W+3, pos_B+2, _get(Klc, pos_W+3, pos_B+2) - H_Jw*K_Jy)
                _set(Klc, pos_W+5, pos_B+2, _get(Klc, pos_W+5, pos_B+2) - ((-1)**(i2+1))*Ll_Jw*K_Jy)
                _set(Klc, pos_B+2, pos_B+3, _get(Klc, pos_B+2, pos_B+3) - H_tJ*K_Jy)
                _set(Klc, pos_B+3, pos_B+3, _get(Klc, pos_B+3, pos_B+3) + H_tJ*H_tJ*K_Jy)
                _set(Klc, pos_B+5, pos_B+3, _get(Klc, pos_B+5, pos_B+3) - H_tJ*((-1)**(i2+1))*Ll_Jt*K_Jy)
                _set(Klc, pos_W+2, pos_B+3, _get(Klc, pos_W+2, pos_B+3) + H_tJ*K_Jy)
                _set(Klc, pos_W+3, pos_B+3, _get(Klc, pos_W+3, pos_B+3) + H_tJ*H_Jw*K_Jy)
                _set(Klc, pos_W+5, pos_B+3, _get(Klc, pos_W+5, pos_B+3) + H_tJ*((-1)**(i2+1))*Ll_Jw*K_Jy)
                _set(Klc, pos_B+2, pos_B+5, _get(Klc, pos_B+2, pos_B+5) + ((-1)**(i2+1))*Ll_Jt*K_Jy)
                _set(Klc, pos_B+3, pos_B+5, _get(Klc, pos_B+3, pos_B+5) - ((-1)**(i2+1))*Ll_Jt*H_tJ*K_Jy)
                _set(Klc, pos_B+5, pos_B+5, _get(Klc, pos_B+5, pos_B+5) + Ll_Jt*Ll_Jt*K_Jy)
                _set(Klc, pos_W+2, pos_B+5, _get(Klc, pos_W+2, pos_B+5) - ((-1)**(i2+1))*Ll_Jt*K_Jy)
                _set(Klc, pos_W+3, pos_B+5, _get(Klc, pos_W+3, pos_B+5) - ((-1)**(i2+1))*Ll_Jt*H_Jw*K_Jy)
                _set(Klc, pos_W+5, pos_B+5, _get(Klc, pos_W+5, pos_B+5) - ((-1)**(i2+1))*Ll_Jt*((-1)**(i2+1))*Ll_Jw*K_Jy)
                _set(Klc, pos_B+2, pos_W+2, _get(Klc, pos_B+2, pos_W+2) - K_Jy)
                _set(Klc, pos_B+3, pos_W+2, _get(Klc, pos_B+3, pos_W+2) + H_tJ*K_Jy)
                _set(Klc, pos_B+5, pos_W+2, _get(Klc, pos_B+5, pos_W+2) - ((-1)**(i2+1))*Ll_Jt*K_Jy)
                _set(Klc, pos_W+2, pos_W+2, _get(Klc, pos_W+2, pos_W+2) + K_Jy)
                _set(Klc, pos_W+3, pos_W+2, _get(Klc, pos_W+3, pos_W+2) + H_Jw*K_Jy)
                _set(Klc, pos_W+5, pos_W+2, _get(Klc, pos_W+5, pos_W+2) + ((-1)**(i2+1))*Ll_Jw*K_Jy)
                _set(Klc, pos_B+2, pos_W+3, _get(Klc, pos_B+2, pos_W+3) - H_Jw*K_Jy)
                _set(Klc, pos_B+3, pos_W+3, _get(Klc, pos_B+3, pos_W+3) + H_Jw*H_tJ*K_Jy)
                _set(Klc, pos_B+5, pos_W+3, _get(Klc, pos_B+5, pos_W+3) - H_Jw*((-1)**(i2+1))*Ll_Jt*K_Jy)
                _set(Klc, pos_W+2, pos_W+3, _get(Klc, pos_W+2, pos_W+3) + H_Jw*K_Jy)
                _set(Klc, pos_W+3, pos_W+3, _get(Klc, pos_W+3, pos_W+3) + H_Jw*H_Jw*K_Jy)
                _set(Klc, pos_W+5, pos_W+3, _get(Klc, pos_W+5, pos_W+3) + H_Jw*((-1)**(i2+1))*Ll_Jw*K_Jy)
                _set(Klc, pos_B+2, pos_W+5, _get(Klc, pos_B+2, pos_W+5) - ((-1)**(i2+1))*Ll_Jw*K_Jy)
                _set(Klc, pos_B+3, pos_W+5, _get(Klc, pos_B+3, pos_W+5) + ((-1)**(i2+1))*Ll_Jw*H_tJ*K_Jy)
                _set(Klc, pos_B+5, pos_W+5, _get(Klc, pos_B+5, pos_W+5) - Ll_Jw*Ll_Jt*K_Jy)
                _set(Klc, pos_W+2, pos_W+5, _get(Klc, pos_W+2, pos_W+5) + ((-1)**(i2+1))*Ll_Jw*K_Jy)
                _set(Klc, pos_W+3, pos_W+5, _get(Klc, pos_W+3, pos_W+5) + ((-1)**(i2+1))*Ll_Jw*H_Jw*K_Jy)
                _set(Klc, pos_W+5, pos_W+5, _get(Klc, pos_W+5, pos_W+5) + Ll_Jw*Ll_Jw*K_Jy)
    for i1 in range(1, 3):
        for i2 in range(1, 3):
            for i3 in range(1, 3):
                pos_P = 35+4*(i1-1)+2*(i2-1)+i3
                pos_W = 10*(i1-1)+5*(i2-1)
                _set(Klc, pos_P, pos_P, _get(Klc, pos_P, pos_P) + K_DPz)
                _set(Klc, pos_W+1, pos_P, _get(Klc, pos_W+1, pos_P) - K_DPz)
                _set(Klc, pos_W+3, pos_P, _get(Klc, pos_W+3, pos_P) - ((-1)**i3)*Lb_DPz*K_DPz)
                _set(Klc, pos_W+4, pos_P, _get(Klc, pos_W+4, pos_P) - ((-1)**i2)*Ll_DPzw*K_DPz)
                _set(Klc, pos_P, pos_W+1, _get(Klc, pos_P, pos_W+1) - K_DPz)
                _set(Klc, pos_W+1, pos_W+1, _get(Klc, pos_W+1, pos_W+1) + K_DPz)
                _set(Klc, pos_W+3, pos_W+1, _get(Klc, pos_W+3, pos_W+1) + ((-1)**i3)*Lb_DPz*K_DPz)
                _set(Klc, pos_W+4, pos_W+1, _get(Klc, pos_W+4, pos_W+1) + ((-1)**i2)*Ll_DPzw*K_DPz)
                _set(Klc, pos_P, pos_W+3, _get(Klc, pos_P, pos_W+3) - ((-1)**i3)*Lb_DPz*K_DPz)
                _set(Klc, pos_W+1, pos_W+3, _get(Klc, pos_W+1, pos_W+3) + ((-1)**i3)*Lb_DPz*K_DPz)
                _set(Klc, pos_W+3, pos_W+3, _get(Klc, pos_W+3, pos_W+3) + Lb_DPz*Lb_DPz*K_DPz)
                _set(Klc, pos_W+4, pos_W+3, _get(Klc, pos_W+4, pos_W+3) + ((-1)**i3)*Lb_DPz*((-1)**i2)*Ll_DPzw*K_DPz)
                _set(Klc, pos_P, pos_W+4, _get(Klc, pos_P, pos_W+4) - ((-1)**i2)*Ll_DPzw*K_DPz)
                _set(Klc, pos_W+1, pos_W+4, _get(Klc, pos_W+1, pos_W+4) + ((-1)**i2)*Ll_DPzw*K_DPz)
                _set(Klc, pos_W+3, pos_W+4, _get(Klc, pos_W+3, pos_W+4) + ((-1)**i2)*Ll_DPzw*((-1)**i3)*Lb_DPz*K_DPz)
                _set(Klc, pos_W+4, pos_W+4, _get(Klc, pos_W+4, pos_W+4) + Ll_DPzw*Ll_DPzw*K_DPz)
    for i1 in range(1, 3):
        for i2 in range(1, 3):
            for i3 in range(1, 3):
                pos_P = 35+4*(i1-1)+2*(i2-1)+i3
                pos_B = 20+5*(i1-1)
                _set(Clc, pos_B+1, pos_B+1, _get(Clc, pos_B+1, pos_B+1) + C_DPz)
                _set(Clc, pos_P, pos_B+1, _get(Clc, pos_P, pos_B+1) - C_DPz)
                _set(Clc, pos_B+3, pos_B+1, _get(Clc, pos_B+3, pos_B+1) + ((-1)**i3)*Lb_DPz*C_DPz)
                _set(Clc, pos_B+4, pos_B+1, _get(Clc, pos_B+4, pos_B+1) + ((-1)**i2)*Ll_DPzt* C_DPz)
                _set(Clc, pos_B+1, pos_P, _get(Clc, pos_B+1, pos_P) - C_DPz)
                _set(Clc, pos_P, pos_P, _get(Clc, pos_P, pos_P) + C_DPz)
                _set(Clc, pos_B+3, pos_P, _get(Clc, pos_B+3, pos_P) - ((-1)**i3)*Lb_DPz*C_DPz)
                _set(Clc, pos_B+4, pos_P, _get(Clc, pos_B+4, pos_P) - ((-1)**i2)*Ll_DPzt* C_DPz)
                _set(Clc, pos_B+1, pos_B+3, _get(Clc, pos_B+1, pos_B+3) + ((-1)**i3)*Lb_DPz*C_DPz)
                _set(Clc, pos_P, pos_B+3, _get(Clc, pos_P, pos_B+3) - ((-1)**i3)*Lb_DPz*C_DPz)
                _set(Clc, pos_B+3, pos_B+3, _get(Clc, pos_B+3, pos_B+3) + Lb_DPz*Lb_DPz*C_DPz)
                _set(Clc, pos_B+4, pos_B+3, _get(Clc, pos_B+4, pos_B+3) + ((-1)**i2)*Ll_DPzt*((-1)**i3)*Lb_DPz*C_DPz)
                _set(Clc, pos_B+1, pos_B+4, _get(Clc, pos_B+1, pos_B+4) + ((-1)**i2)*Ll_DPzt* C_DPz)
                _set(Clc, pos_P, pos_B+4, _get(Clc, pos_P, pos_B+4) - ((-1)**i2)*Ll_DPzt* C_DPz)
                _set(Clc, pos_B+3, pos_B+4, _get(Clc, pos_B+3, pos_B+4) + ((-1)**i3)*Lb_DPz*((-1)**i2)*Ll_DPzt*C_DPz)
                _set(Clc, pos_B+4, pos_B+4, _get(Clc, pos_B+4, pos_B+4) + Ll_DPzt*Ll_DPzt*C_DPz)
    for i1 in range(1, 3):
        for i3 in range(1, 3):
            pos_P = 35+8+2*(i1-1)+i3
            pos_B = 20+5*(i1-1)
            _set(Klc, pos_P, pos_P, _get(Klc, pos_P, pos_P) + K_Sx)
            _set(Klc, pos_B+4, pos_P, _get(Klc, pos_B+4, pos_P) + H_St*K_Sx)
            _set(Klc, pos_B+5, pos_P, _get(Klc, pos_B+5, pos_P) - ((-1)**(i3+1))*Lb_S*K_Sx)
            _set(Klc, pos_P, pos_B+4, _get(Klc, pos_P, pos_B+4) + H_St*K_Sx)
            _set(Klc, pos_B+4, pos_B+4, _get(Klc, pos_B+4, pos_B+4) + H_St*H_St*K_Sx)
            _set(Klc, pos_B+5, pos_B+4, _get(Klc, pos_B+5, pos_B+4) - H_St*((-1)**(i3+1))*Lb_S*K_Sx)
            _set(Klc, pos_P, pos_B+5, _get(Klc, pos_P, pos_B+5) - ((-1)**(i3+1))*Lb_S*K_Sx)
            _set(Klc, pos_B+4, pos_B+5, _get(Klc, pos_B+4, pos_B+5) - ((-1)**(i3+1))*Lb_S*H_St*K_Sx)
            _set(Klc, pos_B+5, pos_B+5, _get(Klc, pos_B+5, pos_B+5) + Lb_S*Lb_S*K_Sx)
    for i1 in range(1, 3):
        for i3 in range(1, 3):
            pos_P = 35+8+2*(i1-1)+i3
            pos_C = 30
            _set(Clc, pos_C+4, pos_C+4, _get(Clc, pos_C+4, pos_C+4) + H_cS*H_cS*C_Sx)
            _set(Clc, pos_C+5, pos_C+4, _get(Clc, pos_C+5, pos_C+4) + H_cS*((-1)**(i3+1))*Lb_S*C_Sx)
            _set(Clc, pos_P, pos_C+4, _get(Clc, pos_P, pos_C+4) - H_cS*C_Sx)
            _set(Clc, pos_C+4, pos_C+5, _get(Clc, pos_C+4, pos_C+5) + ((-1)**(i3+1))*Lb_S*H_cS*C_Sx)
            _set(Clc, pos_C+5, pos_C+5, _get(Clc, pos_C+5, pos_C+5) + Lb_S*Lb_S*C_Sx)
            _set(Clc, pos_P, pos_C+5, _get(Clc, pos_P, pos_C+5) - ((-1)**(i3+1))*Lb_S*C_Sx)
            _set(Clc, pos_C+4, pos_P, _get(Clc, pos_C+4, pos_P) - H_cS*C_Sx)
            _set(Clc, pos_C+5, pos_P, _get(Clc, pos_C+5, pos_P) - ((-1)**(i3+1))*Lb_S*C_Sx)
            _set(Clc, pos_P, pos_P, _get(Clc, pos_P, pos_P) + C_Sx)
    for i1 in range(1, 3):
        for i2 in range(1, 3):
            pos_P = 35+8+4+2*(i1-1)+i2
            pos_B = 20+5*(i1-1)
            _set(Klc, pos_P, pos_P, _get(Klc, pos_P, pos_P) + K_DPy)
            _set(Klc, pos_B+2, pos_P, _get(Klc, pos_B+2, pos_P) - K_DPy)
            _set(Klc, pos_B+3, pos_P, _get(Klc, pos_B+3, pos_P) - H_DPyt*K_DPy)
            _set(Klc, pos_B+5, pos_P, _get(Klc, pos_B+5, pos_P) - ((-1)**(i2+1))*Ll_DPyt*K_DPy)
            _set(Klc, pos_P, pos_B+2, _get(Klc, pos_P, pos_B+2) - K_DPy)
            _set(Klc, pos_B+2, pos_B+2, _get(Klc, pos_B+2, pos_B+2) + K_DPy)
            _set(Klc, pos_B+3, pos_B+2, _get(Klc, pos_B+3, pos_B+2) + H_DPyt*K_DPy)
            _set(Klc, pos_B+5, pos_B+2, _get(Klc, pos_B+5, pos_B+2) + ((-1)**(i2+1))*Ll_DPyt*K_DPy)
            _set(Klc, pos_P, pos_B+3, _get(Klc, pos_P, pos_B+3) - H_DPyt*K_DPy)
            _set(Klc, pos_B+2, pos_B+3, _get(Klc, pos_B+2, pos_B+3) + H_DPyt*K_DPy)
            _set(Klc, pos_B+3, pos_B+3, _get(Klc, pos_B+3, pos_B+3) + H_DPyt*H_DPyt*K_DPy)
            _set(Klc, pos_B+5, pos_B+3, _get(Klc, pos_B+5, pos_B+3) + H_DPyt*((-1)**(i2+1))*Ll_DPyt*K_DPy)
            _set(Klc, pos_P, pos_B+5, _get(Klc, pos_P, pos_B+5) - ((-1)**(i2+1))*Ll_DPyt*K_DPy)
            _set(Klc, pos_B+2, pos_B+5, _get(Klc, pos_B+2, pos_B+5) + ((-1)**(i2+1))*Ll_DPyt*K_DPy)
            _set(Klc, pos_B+3, pos_B+5, _get(Klc, pos_B+3, pos_B+5) + ((-1)**(i2+1))*Ll_DPyt*H_DPyt*K_DPy)
            _set(Klc, pos_B+5, pos_B+5, _get(Klc, pos_B+5, pos_B+5) + Ll_DPyt*Ll_DPyt*K_DPy)
    for i1 in range(1, 3):
        for i2 in range(1, 3):
            pos_P = 35+8+4+2*(i1-1)+i2
            pos_C = 30
            if (i1==1 and i2==1)  or  (i1==2 and i2==2):
                Ll_DPyc = p["Ll2"]+p["Ll_DPyt"]
            else:
                Ll_DPyc = p["Ll2"]-p["Ll_DPyt"]
            _set(Clc, pos_C+2, pos_C+2, _get(Clc, pos_C+2, pos_C+2) + C_DPy)
            _set(Clc, pos_P, pos_C+2, _get(Clc, pos_P, pos_C+2) - C_DPy)
            _set(Clc, pos_C+3, pos_C+2, _get(Clc, pos_C+3, pos_C+2) - H_cDPy*C_DPy)
            _set(Clc, pos_C+5, pos_C+2, _get(Clc, pos_C+5, pos_C+2) + ((-1)**(i1+1))*Ll_DPyc*C_DPy)
            _set(Clc, pos_C+2, pos_P, _get(Clc, pos_C+2, pos_P) - C_DPy)
            _set(Clc, pos_P, pos_P, _get(Clc, pos_P, pos_P) + C_DPy)
            _set(Clc, pos_C+3, pos_P, _get(Clc, pos_C+3, pos_P) + H_cDPy*C_DPy)
            _set(Clc, pos_C+5, pos_P, _get(Clc, pos_C+5, pos_P) - ((-1)**(i1+1))*Ll_DPyc*C_DPy)
            _set(Clc, pos_C+2, pos_C+3, _get(Clc, pos_C+2, pos_C+3) - H_cDPy*C_DPy)
            _set(Clc, pos_P, pos_C+3, _get(Clc, pos_P, pos_C+3) + H_cDPy*C_DPy)
            _set(Clc, pos_C+3, pos_C+3, _get(Clc, pos_C+3, pos_C+3) + H_cDPy*H_cDPy*C_DPy)
            _set(Clc, pos_C+5, pos_C+3, _get(Clc, pos_C+5, pos_C+3) - H_cDPy*((-1)**(i1+1))*Ll_DPyc*C_DPy)
            _set(Clc, pos_C+2, pos_C+5, _get(Clc, pos_C+2, pos_C+5) + ((-1)**(i1+1))*Ll_DPyc*C_DPy)
            _set(Clc, pos_P, pos_C+5, _get(Clc, pos_P, pos_C+5) - ((-1)**(i1+1))*Ll_DPyc*C_DPy)
            _set(Clc, pos_C+3, pos_C+5, _get(Clc, pos_C+3, pos_C+5) - ((-1)**(i1+1))*Ll_DPyc*H_cDPy*C_DPy)
            _set(Clc, pos_C+5, pos_C+5, _get(Clc, pos_C+5, pos_C+5) + Ll_DPyc*Ll_DPyc*C_DPy)
    for i1 in range(1, 3):
        pos_B = 20+5*(i1-1)
        pos_C = 30
        _set(Klc, pos_C+4, pos_C+4, _get(Klc, pos_C+4, pos_C+4) + H_cT*H_cT*K_Tx)
        _set(Klc, pos_B+4, pos_C+4, _get(Klc, pos_B+4, pos_C+4) + H_cT*H_Tt*K_Tx)
        _set(Klc, pos_C+4, pos_B+4, _get(Klc, pos_C+4, pos_B+4) + H_Tt*H_cT*K_Tx)
        _set(Klc, pos_B+4, pos_B+4, _get(Klc, pos_B+4, pos_B+4) + H_Tt*H_Tt*K_Tx)
    for i1 in range(1, 3):
        pos_B = 20+5*(i1-1)
        pos_C = 30
        _set(Klc, pos_C+3, pos_C+3, _get(Klc, pos_C+3, pos_C+3) + K_ROTx)
        _set(Klc, pos_B+3, pos_C+3, _get(Klc, pos_B+3, pos_C+3) - K_ROTx)
        _set(Klc, pos_C+3, pos_B+3, _get(Klc, pos_C+3, pos_B+3) - K_ROTx)
        _set(Klc, pos_B+3, pos_B+3, _get(Klc, pos_B+3, pos_B+3) + K_ROTx)
    return VehicleMatrices(
        M_vehicle=Mlc,
        K_vehicle=Klc,
        C_vehicle=Clc,
        C_vehicle_linear=Clc_0,
    )


def _get(array: np.ndarray, row: float, col: float) -> float:
    return float(array[int(row) - 1, int(col) - 1])


def _set(array: np.ndarray, row: float, col: float, value: float) -> None:
    array[int(row) - 1, int(col) - 1] = float(value)


def _mat_scalar(value: Any, matlab_index: int) -> float:
    return float(np.asarray(value, dtype=float).ravel()[matlab_index - 1])
