from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


VehicleDirection = Literal["Face", "Trail"]
TrackType = Literal["FT-Modal", "FT-FEM", "Co-Running", "Flexible Track"]
LayoutType = Literal["Straight", "Curve"]
NormalContactType = Literal["Hertz", "Hertz&ConDamp", "SIMHertz&ConDamp", "STRIPES&ConDamp"]
ContactDampingType = Literal["Hu-Guo", "Ref", "Lankarani-CNikravesh"]
IntegrationMethod = Literal["Park", "Newmark", "Zhai_Predict", "Houbolt"]
SimulationStage = Literal["Preload", "Cal"]


@dataclass(frozen=True)
class DefaultOperatingCase:
    """MATLAB main-script defaults for the 07(009) Face FT-Modal route."""

    choose_turnout: str = "07(009)"
    track_type: TrackType = "FT-Modal"
    layout_type: LayoutType = "Straight"
    vehicle_direction: VehicleDirection = "Face"
    speed_kmh: float = 350.0
    vehicle_type: str = "CRH380A_v6"
    normal_contact_type: NormalContactType = "STRIPES&ConDamp"
    contact_damping: ContactDampingType = "Hu-Guo"
    contact_damping_coefficient: float = 0.83
    integration_method: IntegrationMethod = "Park"
    cut_freq_ft: float = 2000.0
    n_wheels: int = 4
    nm_fw: int = 0
    n_rigid_vehicle_base: int = 35
    n_rigid_vehicle_extra: int = 16
    contact_patch_left: int = 1
    contact_patch_right: int = 3
    type_side: tuple[str, ...] = ("L", "R")
    wheelsets: tuple[str, ...] = ("FF", "FR", "RF", "RR")
    dummy_rails: tuple[str, ...] = ("L1", "R1", "R2", "R3")
    dummy_rails_left: tuple[str, ...] = ("L1",)
    dummy_rails_right: tuple[str, ...] = ("R1", "R2", "R3")
    dummy_rail_wheel_side: tuple[str, ...] = ("L", "R", "R", "R")
    dof_types: tuple[str, ...] = ("UX", "UY", "UZ", "ROTX", "ROTY", "ROTZ")
    rail_types_all: tuple[str, ...] = (
        "zjbg",
        "qjg_cgyg",
        "zjg_zgyg",
        "qjbg",
        "cxg",
        "dxg",
        "cgjg",
        "hg",
    )
    rail_types: tuple[str, ...] = ("zjbg", "qjbg", "zjg_zgyg", "cxg")
    baseplate_types: tuple[str, ...] = (
        "Switch_L",
        "Switch_R",
        "Closure_zjbg",
        "Closure_qjg_cgyg",
        "Closure_zjg_zgyg",
        "Closure_qjbg",
        "Crossing_L",
        "Crossing_Center",
        "Crossing_R",
        "Plain_Thr_L",
        "Plain_Thr_R",
        "Plain_Div_L",
        "Plain_Div_R",
    )
    simulation_stages: tuple[SimulationStage, ...] = ("Preload", "Cal")

    @property
    def vlc(self) -> float:
        """Signed vehicle speed in m/s, matching MATLAB Trail sign convention."""

        speed = self.speed_kmh / 3.6
        return -speed if self.vehicle_direction == "Trail" else speed

    @property
    def n_rv(self) -> int:
        """CRH380A_v6 rigid vehicle DOF count after MATLAB's extra DOF block."""

        return self.n_rigid_vehicle_base + self.n_rigid_vehicle_extra

    @property
    def n_contact_patch(self) -> int:
        return self.contact_patch_left + self.contact_patch_right

    def stage_inp_par(self, stage: SimulationStage) -> dict[str, object]:
        """Return MATLAB-style ``InpPar`` fields for one main-loop stage."""

        if stage not in self.simulation_stages:
            raise ValueError(f"unsupported simulation stage {stage!r}")
        inp_par = self.to_inp_par()
        inp_par["Type_simulation"] = stage
        return inp_par

    def to_inp_par(self) -> dict[str, object]:
        """Return the default case using MATLAB ``InpPar`` field names."""

        return {
            "Choose_Turnout": self.choose_turnout,
            "Type_Side": self.type_side,
            "Exp_WS": self.wheelsets,
            "Exp_DummyRail": self.dummy_rails,
            "Exp_DummyRail_L": self.dummy_rails_left,
            "Exp_DummyRail_R": self.dummy_rails_right,
            "Exp_DummyRail_WheelSide": self.dummy_rail_wheel_side,
            "N_ConPatch_L": self.contact_patch_left,
            "N_ConPatch_R": self.contact_patch_right,
            "N_ConPatch": self.n_contact_patch,
            "Type_DOF": self.dof_types,
            "Type_Rail_All": self.rail_types_all,
            "Type_Rail": self.rail_types,
            "Type_Baseplate": self.baseplate_types,
            "Type_Normal": self.normal_contact_type,
            "ConDamp": self.contact_damping,
            "ConDamp_Coff": self.contact_damping_coefficient,
            "Int_Method": self.integration_method,
            "Type_Track": self.track_type,
            "Type_Layout": self.layout_type,
            "Vlc": self.vlc,
            "VehicleDir": self.vehicle_direction,
            "Type_Vehicle": self.vehicle_type,
            "N_RV": self.n_rv,
            "Nw": self.n_wheels,
            "NM_FW": self.nm_fw,
            "CutFreq_FT": self.cut_freq_ft,
        }


MATLAB_FULL_DEFAULT_CASE = DefaultOperatingCase()
