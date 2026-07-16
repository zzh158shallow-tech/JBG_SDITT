from __future__ import annotations

from dataclasses import dataclass
from math import pi
from typing import Any, Mapping

import numpy as np

from sditt.contact.forces import (
    NormalDampingClipDiagnostic,
    add_hu_guo_stripes_damping,
    contact_to_track_matrix,
    hertz_normal_force,
    kalker_linear_saturated_creep_force,
    normal_damping_window,
    stripes_normal_force,
)
from sditt.contact.geometry import WheelPose2D, multi_point_contact_geometry, single_point_contact_geometry
from sditt.profiles.geometry import TrackProfileSet, WheelProfileSet, build_track_profiles, contact_tables, offset_profile_to_track
from sditt.track.irregularity import TrackIrregularityProfile, TrackIrregularitySample


_ELLIPTIC_INTEGRAL_PHI = np.linspace(0.0, pi / 2.0, 2001)
_ELLIPTIC_INTEGRAL_SIN2 = np.sin(_ELLIPTIC_INTEGRAL_PHI) ** 2


@dataclass(frozen=True)
class DefaultTrackContactParameters:
    br: float = 0.7175
    fr: float = 0.40
    er: float = 214.0e9
    vr: float = 0.30
    ori_prr: float = 0.0355
    gauge: float = 1.435
    vertical_offset: float = 0.6


@dataclass(frozen=True)
class FullCaseWheelRailContactResult:
    xlcs: int
    con_ws: dict[str, Any]
    pjc: np.ndarray
    pjch: np.ndarray
    pjcc: np.ndarray
    prhx: np.ndarray
    prhxf: np.ndarray
    track_profiles: dict[str, TrackProfileSet]
    d0_by_wheelset: dict[str, float]
    relvel_max_by_wheelset: dict[str, dict[str, float]]
    damping_clip_diagnostics: tuple[dict[str, Any], ...] = ()


def solve_default_wheel_rail_contact(
    inp_par: Mapping[str, Any],
    vehicle_parameters: Mapping[str, Any],
    wheel_profiles: WheelProfileSet,
    rail_profile_selection: Mapping[str, Any],
    rail_response: Any,
    displacement: np.ndarray,
    velocity: np.ndarray,
    *,
    front_mileage: float,
    track_parameters: DefaultTrackContactParameters = DefaultTrackContactParameters(),
    xlcs: int = 1,
    fixed_d0_by_wheelset: Mapping[str, float] | None = None,
    previous_relvel_max_by_wheelset: Mapping[str, Mapping[str, float]] | None = None,
    use_cal_d0_trace: bool = True,
    track_irregularity: TrackIrregularityProfile | None = None,
) -> FullCaseWheelRailContactResult:
    """Assemble the default rigid-wheel full-case contact route.

    This is the Python default-route subset of ``Multi_Con_250812.m``. It
    uses the migrated geometry, STRIPES/Hertz, damping, and Kalker helpers to
    generate the MATLAB-style per-wheelset contact structure plus the
    aggregated force arrays consumed by the existing force-mapping stage.
    """

    n_wheels = int(inp_par["Nw"])
    n_contact_patch = int(inp_par["N_ConPatch"])
    exp_ws = list(inp_par["Exp_WS"])
    exp_dummy_rail = list(inp_par["Exp_DummyRail"])
    exp_dummy_rail_side = list(inp_par["Exp_DummyRail_WheelSide"])
    left_dummy_rails = tuple(inp_par["Exp_DummyRail_L"])
    right_dummy_rails = tuple(inp_par["Exp_DummyRail_R"])
    wheel_distances = _wheelset_distances(vehicle_parameters, exp_ws)

    displacement = np.asarray(displacement, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    pjc = np.zeros((n_wheels * n_contact_patch, 2), dtype=float)
    pjch = np.zeros((n_wheels * n_contact_patch, 1), dtype=float)
    pjcc = np.zeros((n_wheels * n_contact_patch, 1), dtype=float)
    prhx = np.zeros((n_wheels * n_contact_patch, 3), dtype=float)
    prhxf = np.zeros((n_wheels * n_contact_patch, 6), dtype=float)
    con_ws: dict[str, Any] = {"FF": {"Normal_Force": True}}
    track_profiles: dict[str, TrackProfileSet] = {}
    result_d0_by_wheelset: dict[str, float] = {}
    result_relvel_max_by_wheelset: dict[str, dict[str, float]] = {}
    damping_clip_diagnostics: list[dict[str, Any]] = []
    wheel_radius_profiles = {"L": wheel_profiles.radius_left, "R": wheel_profiles.radius_right}
    wheel_shape_profiles = {"L": wheel_profiles.left, "R": wheel_profiles.right}
    wheel_angle_profiles = {
        "L": wheel_profiles.contact_angle_left,
        "R": wheel_profiles.contact_angle_right,
    }

    for wheel_index, wheelset in enumerate(exp_ws):
        mileage = float(front_mileage) - wheel_distances[wheelset]
        irregularity_sample = track_irregularity.sample(mileage) if track_irregularity is not None else None
        track_profile = _build_track_profile_for_wheelset(
            inp_par,
            rail_profile_selection,
            rail_response.dis_rail,
            wheel_index,
            wheelset,
            exp_dummy_rail,
            exp_dummy_rail_side,
            left_dummy_rails,
            right_dummy_rails,
            track_parameters,
            irregularity_sample,
        )
        track_profiles[wheelset] = track_profile

        pose = _wheel_pose(displacement, inp_par, vehicle_parameters, wheel_index)
        wheel_rate = _wheel_rate(velocity, inp_par, vehicle_parameters, wheel_index)
        if fixed_d0_by_wheelset is not None and wheelset in fixed_d0_by_wheelset:
            d0 = float(fixed_d0_by_wheelset[wheelset])
        else:
            d0 = _estimate_contact_offset(
                wheel_shape_profiles,
                wheel_angle_profiles,
                track_profile,
                pose,
                dlb=float(vehicle_parameters["Dlb"]),
                use_cal_d0_trace=use_cal_d0_trace,
            )
        result_d0_by_wheelset[wheelset] = d0
        result_relvel_max_by_wheelset[wheelset] = {}

        con_ws[wheelset] = {
            "Mileage": mileage,
            "Track_Irregularity": _track_irregularity_payload(irregularity_sample, float(inp_par["Vlc"])),
            "profile_r": {
                "L": np.asarray(track_profile.profile["L"], dtype=float),
                "R": np.asarray(track_profile.profile["R"], dtype=float),
            },
            "Normal_Force": {"L": np.zeros((0, 4), dtype=float), "R": np.zeros((0, 4), dtype=float)},
            "Con_wheel_2": {"L": np.zeros((0, 3), dtype=float), "R": np.zeros((0, 3), dtype=float)},
            "Con_wheel_2_full": {"L": np.zeros((0, 6), dtype=float), "R": np.zeros((0, 6), dtype=float)},
            "Con_rail_1": {"L": np.zeros((0, 2), dtype=float), "R": np.zeros((0, 2), dtype=float)},
            "Con_RelVel": {"L": np.zeros((0, 2), dtype=float), "R": np.zeros((0, 2), dtype=float)},
            "Con_RelVel_max": {"L": np.zeros((0, 1), dtype=float), "R": np.zeros((0, 1), dtype=float)},
            "Prhx_T": {"L": np.zeros((0, 3), dtype=float), "R": np.zeros((0, 3), dtype=float)},
            "Prhxf_T": {"L": np.zeros((0, 6), dtype=float), "R": np.zeros((0, 6), dtype=float)},
            "Prh": {"L": np.zeros((0, 3), dtype=float), "R": np.zeros((0, 3), dtype=float)},
            "RHXS": {"L": np.zeros((0, 4), dtype=float), "R": np.zeros((0, 4), dtype=float)},
            "RHLv": {"L": np.zeros((0, 3), dtype=float), "R": np.zeros((0, 3), dtype=float)},
            "a2": {"L": np.zeros((0,), dtype=float), "R": np.zeros((0,), dtype=float)},
            "b2": {"L": np.zeros((0,), dtype=float), "R": np.zeros((0,), dtype=float)},
            "Vjd": {"L": np.zeros((0, 3), dtype=float), "R": np.zeros((0, 3), dtype=float)},
            "Vjd_r": {"L": np.zeros((0, 3), dtype=float), "R": np.zeros((0, 3), dtype=float)},
            "Vsdc": {"L": np.zeros((0, 3), dtype=float), "R": np.zeros((0, 3), dtype=float)},
            "Vjsdc": {"L": np.zeros((0, 3), dtype=float), "R": np.zeros((0, 3), dtype=float)},
            "Vgd": {"L": np.zeros((0, 1), dtype=float), "R": np.zeros((0, 1), dtype=float)},
        }

        for side in ("L", "R"):
            side_result = _solve_wheel_side_contact(
                inp_par,
                vehicle_parameters,
                track_parameters,
                wheel_shape_profiles[side],
                wheel_angle_profiles[side],
                wheel_radius_profiles[side],
                rail_response,
                track_profile,
                pose,
                wheel_rate,
                wheel_index,
                side,
                mileage=mileage,
                d0=d0,
                exp_dummy_rail=exp_dummy_rail,
                previous_relvel_max_by_dummy_rail=(
                    previous_relvel_max_by_wheelset.get(wheelset)
                    if previous_relvel_max_by_wheelset is not None
                    else None
                ),
                track_irregularity_sample=irregularity_sample,
            )
            con_ws[wheelset]["Normal_Force"][side] = side_result["normal_force"]
            con_ws[wheelset]["Con_wheel_2"][side] = side_result["con_wheel_2"]
            con_ws[wheelset]["Con_wheel_2_full"][side] = side_result["con_wheel_2_full"]
            con_ws[wheelset]["Con_rail_1"][side] = side_result["con_rail_1"]
            con_ws[wheelset]["Con_RelVel"][side] = side_result["con_rel_vel"]
            con_ws[wheelset]["Con_RelVel_max"][side] = side_result["con_rel_vel_max"][:, np.newaxis]
            con_ws[wheelset]["Prhx_T"][side] = side_result["prhx_t"]
            con_ws[wheelset]["Prhxf_T"][side] = side_result["prhxf_t"]
            con_ws[wheelset]["Prh"][side] = side_result["prh"]
            con_ws[wheelset]["RHXS"][side] = side_result["rhxs"]
            con_ws[wheelset]["RHLv"][side] = side_result["rhlv"]
            con_ws[wheelset]["a2"][side] = side_result["a2"]
            con_ws[wheelset]["b2"][side] = side_result["b2"]
            con_ws[wheelset]["Vjd"][side] = side_result["vjd"]
            con_ws[wheelset]["Vjd_r"][side] = side_result["vjd_r"]
            con_ws[wheelset]["Vsdc"][side] = side_result["vsdc"]
            con_ws[wheelset]["Vjsdc"][side] = side_result["vjsdc"]
            con_ws[wheelset]["Vgd"][side] = side_result["vgd"][:, np.newaxis]
            con_ws[wheelset]["Normal_Damping_Clips"] = con_ws[wheelset].get(
                "Normal_Damping_Clips", {"L": (), "R": ()}
            )
            con_ws[wheelset]["Elastic_Normal_Force"] = con_ws[wheelset].get(
                "Elastic_Normal_Force", {"L": np.zeros((0, 6), dtype=float), "R": np.zeros((0, 6), dtype=float)}
            )
            con_ws[wheelset]["Con_wheel_2_a"] = con_ws[wheelset].get(
                "Con_wheel_2_a", {"L": np.zeros((0, 6), dtype=float), "R": np.zeros((0, 6), dtype=float)}
            )
            con_ws[wheelset]["Con_rail_1_a"] = con_ws[wheelset].get(
                "Con_rail_1_a", {"L": np.zeros((0, 2), dtype=float), "R": np.zeros((0, 2), dtype=float)}
            )
            con_ws[wheelset]["Ver_Pen_a"] = con_ws[wheelset].get(
                "Ver_Pen_a", {"L": np.zeros((0, 4), dtype=float), "R": np.zeros((0, 4), dtype=float)}
            )
            con_ws[wheelset]["Area_STRIPES"] = con_ws[wheelset].get(
                "Area_STRIPES", {"L": np.zeros((0,), dtype=float), "R": np.zeros((0,), dtype=float)}
            )
            con_ws[wheelset]["Epsilon"] = con_ws[wheelset].get(
                "Epsilon", {"L": np.zeros((0,), dtype=float), "R": np.zeros((0,), dtype=float)}
            )
            con_ws[wheelset]["Con_STRIPES"] = con_ws[wheelset].get(
                "Con_STRIPES", {"L": (), "R": ()}
            )
            con_ws[wheelset]["Elastic_Normal_Force"][side] = side_result["elastic_normal_force"]
            con_ws[wheelset]["Con_wheel_2_a"][side] = side_result["con_wheel_2_peak"]
            con_ws[wheelset]["Con_rail_1_a"][side] = side_result["con_rail_1_peak"]
            con_ws[wheelset]["Ver_Pen_a"][side] = side_result["penetration_peaks"]
            con_ws[wheelset]["Area_STRIPES"][side] = side_result["area_stripes"]
            con_ws[wheelset]["Epsilon"][side] = side_result["epsilon"]
            con_ws[wheelset]["Con_STRIPES"][side] = side_result["con_stripes"]
            clip_records = _side_damping_clip_diagnostics(
                tuple(side_result["normal_damping_clips"]),
                mileage=mileage,
                wheelset=wheelset,
                side=side,
                patch_ids=np.asarray(side_result["patch_ids"], dtype=int),
                exp_dummy_rail=exp_dummy_rail,
            )
            con_ws[wheelset]["Normal_Damping_Clips"][side] = clip_records
            damping_clip_diagnostics.extend(clip_records)
            for patch_id, relvel_max in zip(
                np.asarray(side_result["patch_ids"], dtype=int),
                np.asarray(side_result["con_rel_vel_max"], dtype=float),
                strict=True,
            ):
                dummy_rail = exp_dummy_rail[int(patch_id) - 1]
                existing = result_relvel_max_by_wheelset[wheelset].get(dummy_rail, float("-inf"))
                result_relvel_max_by_wheelset[wheelset][dummy_rail] = max(existing, float(relvel_max))

            for k in range(side_result["normal_force"].shape[0]):
                patch_id = int(side_result["normal_force"][k, 3])
                contact_index = n_contact_patch * wheel_index + patch_id - 1
                pjc[contact_index, 1] += side_result["normal_force"][k, 0]
                pjch[contact_index, 0] += side_result["normal_force"][k, 1]
                pjcc[contact_index, 0] += side_result["normal_force"][k, 2]
                prhx[contact_index, :] += side_result["prhx_t"][k, :]
                prhxf[contact_index, :] += side_result["prhxf_t"][k, :]

    return FullCaseWheelRailContactResult(
        xlcs=int(xlcs),
        con_ws=con_ws,
        pjc=pjc,
        pjch=pjch,
        pjcc=pjcc,
        prhx=prhx,
        prhxf=prhxf,
        track_profiles=track_profiles,
        d0_by_wheelset=result_d0_by_wheelset,
        relvel_max_by_wheelset=result_relvel_max_by_wheelset,
        damping_clip_diagnostics=tuple(damping_clip_diagnostics),
    )


def _build_track_profile_for_wheelset(
    inp_par: Mapping[str, Any],
    rail_profile_selection: Mapping[str, Any],
    dis_rail: np.ndarray,
    wheel_index: int,
    wheelset: str,
    exp_dummy_rail: list[str],
    exp_dummy_rail_side: list[str],
    left_dummy_rails: tuple[str, ...],
    right_dummy_rails: tuple[str, ...],
    track_parameters: DefaultTrackContactParameters,
    track_irregularity_sample: TrackIrregularitySample | None,
) -> TrackProfileSet:
    n_contact_patch = int(inp_par["N_ConPatch"])
    offset_profiles: dict[str, Any] = {}
    for patch_index, (dummy_rail, wheel_side) in enumerate(zip(exp_dummy_rail, exp_dummy_rail_side, strict=True)):
        record = rail_profile_selection[dummy_rail].by_station[wheelset]
        contact_index = n_contact_patch * wheel_index + patch_index
        irregularity_y = irregularity_z = 0.0
        if track_irregularity_sample is not None:
            irregularity_y, irregularity_z = track_irregularity_sample.rail_displacement_m[wheel_side]
        offset_profiles[dummy_rail] = offset_profile_to_track(
            record,
            wheel_side=wheel_side,
            ori_prr=track_parameters.ori_prr,
            dis_rail_y=float(dis_rail[contact_index, 1]),
            dis_rail_z=float(dis_rail[contact_index, 2]),
            irregularity_y=float(irregularity_y),
            irregularity_z=float(irregularity_z),
            gauge=track_parameters.gauge,
            vertical_offset=track_parameters.vertical_offset,
        )
    return build_track_profiles(
        offset_profiles,
        left_dummy_rails=left_dummy_rails,
        right_dummy_rails=right_dummy_rails,
    )


def _solve_wheel_side_contact(
    inp_par: Mapping[str, Any],
    vehicle_parameters: Mapping[str, Any],
    track_parameters: DefaultTrackContactParameters,
    wheel_profile: np.ndarray,
    wheel_angles: np.ndarray,
    wheel_radius_profile: np.ndarray,
    rail_response: Any,
    track_profile: TrackProfileSet,
    pose: WheelPose2D,
    wheel_rate: dict[str, Any],
    wheel_index: int,
    side: str,
    *,
    mileage: float,
    d0: float,
    exp_dummy_rail: list[str],
    previous_relvel_max_by_dummy_rail: Mapping[str, float] | None,
    track_irregularity_sample: TrackIrregularitySample | None,
) -> dict[str, np.ndarray]:
    rail_profile = np.asarray(track_profile.profile[side], dtype=float)
    if rail_profile.size == 0:
        return _empty_side_result()

    geometry = multi_point_contact_geometry(
        wheel_profile,
        wheel_angles,
        rail_profile,
        pose=pose,
        penetration_offset=d0,
        min_overlap_margin=1.0e-4,
        dlb=float(vehicle_parameters["Dlb"]),
    )
    if not geometry.patches:
        return _empty_side_result()

    patch_count = len(geometry.patches)
    transforms = np.zeros((patch_count, 3, 3), dtype=float)
    transforms_peak = np.zeros((patch_count, 3, 3), dtype=float)
    con_wheel_1 = np.zeros((patch_count, 3), dtype=float)
    con_wheel_2 = np.zeros((patch_count, 6), dtype=float)
    con_rail_1 = np.zeros((patch_count, 2), dtype=float)
    con_wheel_1_peak = np.zeros((patch_count, 3), dtype=float)
    con_wheel_2_peak = np.zeros((patch_count, 6), dtype=float)
    con_rail_1_peak = np.zeros((patch_count, 2), dtype=float)
    penetration_peaks = np.zeros((patch_count, 4), dtype=float)
    patch_ids = np.zeros((patch_count,), dtype=int)
    wheel_to_track = np.array(
        [
            [np.cos(pose.yaw), np.sin(pose.yaw), 0.0],
            [-np.cos(pose.roll) * np.sin(pose.yaw), np.cos(pose.roll) * np.cos(pose.yaw), np.sin(pose.roll)],
            [np.sin(pose.roll) * np.sin(pose.yaw), -np.sin(pose.roll) * np.cos(pose.yaw), np.cos(pose.roll)],
        ],
        dtype=float,
    )

    for i, patch in enumerate(geometry.patches):
        transforms[i, :, :] = contact_to_track_matrix(pose.yaw, pose.roll, patch.contact_angle)
        transforms_peak[i, :, :] = contact_to_track_matrix(pose.yaw, pose.roll, patch.peak_contact_angle)
        con_wheel_1[i, :] = np.array([0.0, patch.corrected_wheel_point[0], patch.corrected_wheel_point[1]], dtype=float)
        corrected_local = _right_matrix_divide(
            patch.corrected_wheel_point[np.newaxis, :] - np.array([0.0, pose.lateral, 0.0], dtype=float),
            wheel_to_track,
        )[0]
        con_wheel_2[i, :] = np.array(
            [
                corrected_local[0],
                corrected_local[1],
                corrected_local[2],
                patch.corrected_vertical_penetration,
                patch.corrected_normal_penetration,
                patch.contact_angle,
            ],
            dtype=float,
        )
        con_rail_1[i, :] = patch.corrected_rail_point
        peak_local = _right_matrix_divide(
            patch.peak_wheel_point[np.newaxis, :] - np.array([0.0, pose.lateral, 0.0], dtype=float),
            wheel_to_track,
        )[0]
        con_wheel_1_peak[i, :] = patch.peak_wheel_point
        con_wheel_2_peak[i, :] = np.array(
            [
                peak_local[0],
                peak_local[1],
                peak_local[2],
                patch.peak_vertical_penetration,
                patch.peak_normal_penetration,
                patch.peak_contact_angle,
            ],
            dtype=float,
        )
        con_rail_1_peak[i, :] = patch.peak_rail_point
        penetration_peaks[i, :] = np.array(
            [float(patch.peak_index), float(patch.peak_wheel_point[1]), patch.peak_vertical_penetration, 0.0],
            dtype=float,
        )
        patch_ids[i] = _judge_contact_patch_id(side, patch.corrected_rail_point[0], track_profile)

    r_yy_w, r_xx_w, r_xx_r, rou = _contact_radii(
        wheel_radius_profile,
        np.asarray(track_profile.radius[side], dtype=float),
        con_wheel_2,
        con_rail_1,
    )
    m, n, elastic_permeability, con_a, con_b, con_r = _hertz_parameters(
        r_yy_w,
        r_xx_w,
        r_xx_r,
        rou,
        elastic_modulus=track_parameters.er,
        poisson_ratio=track_parameters.vr,
    )
    if str(inp_par["Type_Normal"]) == "STRIPES&ConDamp":
        r_yy_w_peak, r_xx_w_peak, r_xx_r_peak, rou_peak = _contact_radii(
            wheel_radius_profile,
            np.asarray(track_profile.radius[side], dtype=float),
            con_wheel_2_peak,
            con_rail_1_peak,
        )
        m_peak, n_peak, _, con_a_peak, con_b_peak, con_r_peak = _hertz_parameters(
            r_yy_w_peak,
            r_xx_w_peak,
            r_xx_r_peak,
            rou_peak,
            elastic_modulus=track_parameters.er,
            poisson_ratio=track_parameters.vr,
        )
    else:
        m_peak = n_peak = con_a_peak = con_b_peak = con_r_peak = None
    vjd, vjd_r, vsdc, vjsdc = _creepage_inputs(
        rail_response,
        inp_par,
        vehicle_parameters,
        wheel_rate,
        pose,
        con_wheel_2,
        transforms,
        patch_ids,
        wheel_index,
        track_irregularity_sample,
    )
    rel_ratio, relvel_max = _relative_velocity_ratio(
        vsdc[:, 2],
        patch_ids,
        exp_dummy_rail,
        previous_relvel_max_by_dummy_rail=previous_relvel_max_by_dummy_rail,
    )

    if str(inp_par["Type_Normal"]) == "STRIPES&ConDamp":
        elastic = stripes_normal_force(
            wheel_radius_profile,
            np.asarray(track_profile.radius[side], dtype=float),
            con_wheel_1_peak,
            con_wheel_2_peak,
            con_rail_1_peak,
            geometry.wheel_interp,
            geometry.rail_interp,
            wheel_lateral=float(pose.lateral),
            wheel_to_track=wheel_to_track,
            contact_to_track=transforms_peak,
            penetration_peaks=penetration_peaks,
            m=m_peak,
            n=n_peak,
            con_a=con_a_peak,
            con_b=con_b_peak,
            con_r=con_r_peak,
            elastic_modulus=track_parameters.er,
            poisson_ratio=track_parameters.vr,
            stripe_count=51,
            correction="AB",
        )
        normal_force_result = add_hu_guo_stripes_damping(
            elastic,
            relative_velocity_ratio=rel_ratio,
            restitution_coefficient=float(inp_par["ConDamp_Coff"]),
            window=normal_damping_window(mileage, str(inp_par["VehicleDir"])),
        )
        normal_total = normal_force_result.normal_force[:, 0]
    else:
        normal_total = hertz_normal_force(con_wheel_2[:, 4], elastic_permeability, minimum_force=1.0e-3)
        normal_force_result = None
    if patch_count and np.all(np.abs(normal_total) <= 1.0e-9):
        normal_total = np.full((patch_count,), _nominal_contact_force(vehicle_parameters) / patch_count, dtype=float)
    vgd = np.abs(float(inp_par["Vlc"]) / 2.0 * (1.0 + con_wheel_2[:, 2] / float(vehicle_parameters["R0"]) * np.cos(pose.yaw)))
    creepage_inputs = np.column_stack((vsdc[:, 0], vsdc[:, 1], vjsdc[:, 2]))
    creepage = np.column_stack(
        (
            np.divide(creepage_inputs[:, 0], np.maximum(vgd, 1.0e-9)),
            np.divide(creepage_inputs[:, 1], np.maximum(vgd, 1.0e-9)),
            np.divide(creepage_inputs[:, 2], np.maximum(vgd, 1.0e-9)),
        )
    )
    tangential = kalker_linear_saturated_creep_force(
        normal_total,
        creepage,
        rolling_radius_sum=rou,
        wheel_rolling_radius=r_yy_w,
        m=m,
        n=n,
        elastic_modulus=track_parameters.er,
        poisson_ratio=track_parameters.vr,
        friction_coefficient=track_parameters.fr,
        vehicle_speed=float(inp_par["Vlc"]),
        contact_to_track=transforms,
    )
    prhx_t = tangential.saturated_force
    prhxf_t = tangential.force_track if tangential.force_track is not None else np.zeros((patch_count, 6), dtype=float)

    normal_force = np.zeros((patch_count, 4), dtype=float)
    for i in range(patch_count):
        normal_force[i, 0] = float(normal_total[i])
        normal_track = _normal_force_track_component(normal_total[i], con_wheel_2[i, 5] + pose.roll)
        normal_force[i, 1:3] = normal_track
        normal_force[i, 3] = float(patch_ids[i])

    return {
        "normal_force": normal_force,
        "con_wheel_2": con_wheel_2[:, :3],
        "con_wheel_2_full": con_wheel_2,
        "con_rail_1": con_rail_1,
        "con_wheel_2_peak": con_wheel_2_peak,
        "con_rail_1_peak": con_rail_1_peak,
        "penetration_peaks": penetration_peaks,
        "con_rel_vel": np.column_stack((vsdc[:, 2], rel_ratio)),
        "con_rel_vel_max": relvel_max,
        "prh": tangential.linear_force,
        "prhx_t": prhx_t,
        "prhxf_t": prhxf_t,
        "patch_ids": patch_ids,
        "rhxs": tangential.creep_stiffness,
        "rhlv": creepage,
        "a2": tangential.semi_axis_a,
        "b2": tangential.semi_axis_b,
        "vjd": vjd,
        "vjd_r": vjd_r,
        "vsdc": vsdc,
        "vjsdc": vjsdc,
        "vgd": vgd,
        "elastic_normal_force": (
            normal_force_result.normal_force if normal_force_result is not None else np.zeros((patch_count, 6), dtype=float)
        ),
        "area_stripes": normal_force_result.area if normal_force_result is not None else np.zeros((patch_count,), dtype=float),
        "epsilon": normal_force_result.epsilon if normal_force_result is not None else np.zeros((patch_count,), dtype=float),
        "con_stripes": normal_force_result.patches if normal_force_result is not None else (),
        "normal_damping_clips": (
            normal_force_result.damping_clip_diagnostics if normal_force_result is not None else ()
        ),
    }


def _side_damping_clip_diagnostics(
    diagnostics: tuple[NormalDampingClipDiagnostic, ...],
    *,
    mileage: float,
    wheelset: str,
    side: str,
    patch_ids: np.ndarray,
    exp_dummy_rail: list[str],
) -> tuple[dict[str, Any], ...]:
    records: list[dict[str, Any]] = []
    for diagnostic in diagnostics:
        patch_index = int(diagnostic.patch_index)
        patch_id = int(patch_ids[patch_index]) if 0 <= patch_index < patch_ids.size else 0
        dummy_rail = exp_dummy_rail[patch_id - 1] if 1 <= patch_id <= len(exp_dummy_rail) else ""
        records.append(
            {
                "mileage": float(mileage),
                "wheelset": str(wheelset),
                "side": str(side),
                "patch_index": patch_index,
                "patch_id": patch_id,
                "dummy_rail": dummy_rail,
                "elastic_force": float(diagnostic.elastic_force),
                "raw_damping_force": float(diagnostic.raw_damping_force),
                "clipped_damping_force": float(diagnostic.clipped_damping_force),
                "relative_velocity_ratio": float(diagnostic.relative_velocity_ratio),
            }
        )
    return tuple(records)


def _estimate_contact_offset(
    wheel_shape_profiles: Mapping[str, np.ndarray],
    wheel_angle_profiles: Mapping[str, np.ndarray],
    track_profile: TrackProfileSet,
    pose: WheelPose2D,
    *,
    dlb: float | None = None,
    use_cal_d0_trace: bool = True,
) -> float:
    gaps: list[float] = []
    min_overlap_margin = 0.15e-3 if use_cal_d0_trace else 1.0e-4
    discrete_len_tread = 2.0e-5 if use_cal_d0_trace else 2.5e-5
    for side in ("L", "R"):
        rail_profile = np.asarray(track_profile.profile[side], dtype=float)
        if rail_profile.size == 0:
            continue
        contact = single_point_contact_geometry(
            wheel_shape_profiles[side],
            wheel_angle_profiles[side],
            rail_profile,
            pose=pose,
            min_overlap_margin=min_overlap_margin,
            dlb=dlb,
            discrete_len_flange=0.5e-5,
            discrete_len_tread=discrete_len_tread,
        )
        if np.isfinite(contact.vertical_gap):
            gaps.append(float(contact.vertical_gap))
    if not gaps:
        return 0.0
    return float(np.mean(gaps))


def _wheel_pose(
    displacement: np.ndarray,
    inp_par: Mapping[str, Any],
    vehicle_parameters: Mapping[str, Any],
    wheel_index: int,
) -> WheelPose2D:
    del vehicle_parameters
    base = _wheelset_state_base(inp_par, wheel_index)
    return WheelPose2D(
        lateral=_state_value(displacement, base + 1),
        vertical=_state_value(displacement, base + 0),
        roll=_state_value(displacement, base + 2),
        yaw=_state_value(displacement, base + 4),
    )


def _wheel_rate(
    velocity: np.ndarray,
    inp_par: Mapping[str, Any],
    vehicle_parameters: Mapping[str, Any],
    wheel_index: int,
) -> dict[str, Any]:
    base = _wheelset_state_base(inp_par, wheel_index)
    spin_rate = -float(inp_par["Vlc"]) / float(vehicle_parameters["R0"]) + _state_value(velocity, base + 3)
    return {
        "base": base,
        "trans": np.array(
            [
                float(inp_par["Vlc"]),
                _state_value(velocity, base + 1),
                _state_value(velocity, base + 0),
            ],
            dtype=float,
        ),
        "roll_rate": _state_value(velocity, base + 2),
        "spin_rate": spin_rate,
        "spin_rate_relative": _state_value(velocity, base + 3),
        "yaw_rate": _state_value(velocity, base + 4),
    }


def _creepage_inputs(
    rail_response: Any,
    inp_par: Mapping[str, Any],
    vehicle_parameters: Mapping[str, Any],
    wheel_rate: Mapping[str, Any],
    pose: WheelPose2D,
    con_wheel_2: np.ndarray,
    transforms: np.ndarray,
    patch_ids: np.ndarray,
    wheel_index: int,
    track_irregularity_sample: TrackIrregularitySample | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    vjd = np.zeros((con_wheel_2.shape[0], 3), dtype=float)
    vjd_r = np.zeros((con_wheel_2.shape[0], 3), dtype=float)
    vsdc = np.zeros((con_wheel_2.shape[0], 3), dtype=float)
    vjsdc = np.zeros((con_wheel_2.shape[0], 3), dtype=float)
    if con_wheel_2.shape[0] == 0:
        return vjd, vjd_r, vsdc, vjsdc

    del vehicle_parameters
    a_ws = _wheelset_orientation(pose.roll, pose.yaw)
    a_ws_d1 = _wheelset_orientation_rate(
        pose.roll,
        pose.yaw,
        float(wheel_rate["roll_rate"]),
        float(wheel_rate["yaw_rate"]),
    )
    g_cardan_ws_body = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, np.sin(pose.roll)],
            [0.0, 0.0, np.cos(pose.roll)],
        ],
        dtype=float,
    )
    angvel_ws_body = g_cardan_ws_body @ np.array(
        [
            float(wheel_rate["roll_rate"]),
            float(wheel_rate["spin_rate"]),
            float(wheel_rate["yaw_rate"]),
        ],
        dtype=float,
    )
    angvel_track = angvel_ws_body @ a_ws

    patch_count = con_wheel_2.shape[0]
    vjd[:, :] = np.tile(np.asarray(wheel_rate["trans"], dtype=float), (patch_count, 1))
    vjd += con_wheel_2[:, :3] @ a_ws_d1
    spin_velocity = np.zeros((patch_count, 3), dtype=float)
    spin_velocity[:, 0] = float(wheel_rate["spin_rate"]) * con_wheel_2[:, 2]
    vjd += spin_velocity @ a_ws

    n_contact_patch = int(inp_par["N_ConPatch"])
    exp_dummy_rail = list(inp_par["Exp_DummyRail"])
    rail_beam_vel_z = rail_response.rail_beam_motion.get("Vel_Z", {})
    for patch_index, patch_id in enumerate(np.asarray(patch_ids, dtype=int).reshape(-1)):
        contact_index = n_contact_patch * wheel_index + patch_id - 1
        vjd_r[patch_index, :] = np.asarray(rail_response.vel_rail[contact_index, :3], dtype=float)
        dummy_rail = exp_dummy_rail[patch_id - 1]
        if track_irregularity_sample is not None:
            side = "L" if dummy_rail in tuple(inp_par["Exp_DummyRail_L"]) else "R"
            irregularity_velocity_y, irregularity_velocity_z = track_irregularity_sample.rail_velocity(
                side,
                float(inp_par["Vlc"]),
            )
            vjd_r[patch_index, 1] += irregularity_velocity_y
            vjd_r[patch_index, 2] += irregularity_velocity_z
        if dummy_rail == "R2" and dummy_rail in rail_beam_vel_z:
            vjd_r[patch_index, 2] += float(rail_beam_vel_z[dummy_rail][wheel_index, 0])
        vsdc[patch_index, :] = _right_matrix_divide(vjd[patch_index, :] - vjd_r[patch_index, :], transforms[patch_index])
        vjsdc[patch_index, :] = _right_matrix_divide(angvel_track, transforms[patch_index])

    return vjd, vjd_r, vsdc, vjsdc


def _track_irregularity_payload(
    sample: TrackIrregularitySample | None,
    speed: float,
) -> dict[str, Any]:
    if sample is None:
        return {}
    return {
        "mileage": sample.mileage,
        "components_m": dict(sample.components_m),
        "component_slopes": dict(sample.component_slopes),
        "rail_displacement_m": {side: tuple(value) for side, value in sample.rail_displacement_m.items()},
        "rail_velocity_m_per_s": {side: sample.rail_velocity(side, speed) for side in ("L", "R")},
    }


def _wheelset_orientation(roll: float, yaw: float) -> np.ndarray:
    return np.array(
        [
            [np.cos(yaw), np.sin(yaw), 0.0],
            [-np.cos(roll) * np.sin(yaw), np.cos(roll) * np.cos(yaw), np.sin(roll)],
            [np.sin(roll) * np.sin(yaw), -np.sin(roll) * np.cos(yaw), np.cos(roll)],
        ],
        dtype=float,
    )


def _wheelset_orientation_rate(roll: float, yaw: float, roll_rate: float, yaw_rate: float) -> np.ndarray:
    return np.array(
        [
            [-np.sin(yaw) * yaw_rate, np.cos(yaw) * yaw_rate, 0.0],
            [
                np.sin(roll) * np.sin(yaw) * roll_rate - np.cos(roll) * np.cos(yaw) * yaw_rate,
                -np.sin(roll) * np.cos(yaw) * roll_rate - np.cos(roll) * np.sin(yaw) * yaw_rate,
                np.cos(roll) * roll_rate,
            ],
            [
                np.cos(roll) * np.sin(yaw) * roll_rate + np.sin(roll) * np.cos(yaw) * yaw_rate,
                -np.cos(roll) * np.cos(yaw) * roll_rate + np.sin(roll) * np.sin(yaw) * yaw_rate,
                -np.sin(roll) * roll_rate,
            ],
        ],
        dtype=float,
    )


def _relative_velocity_ratio(
    relative_vertical_velocity: np.ndarray,
    patch_ids: np.ndarray,
    exp_dummy_rail: list[str],
    *,
    previous_relvel_max_by_dummy_rail: Mapping[str, float] | None,
) -> tuple[np.ndarray, np.ndarray]:
    relative_vertical_velocity = np.asarray(relative_vertical_velocity, dtype=float)
    patch_ids = np.asarray(patch_ids, dtype=int).reshape(-1)
    rel_ratio = np.zeros_like(relative_vertical_velocity)
    relvel_max = np.zeros_like(relative_vertical_velocity)
    previous_lookup = previous_relvel_max_by_dummy_rail or {}
    for index, current_velocity in enumerate(relative_vertical_velocity):
        dummy_rail = exp_dummy_rail[int(patch_ids[index]) - 1]
        previous_max = float(previous_lookup.get(dummy_rail, 0.0))
        if previous_max <= 0.0 and current_velocity <= 0.0:
            relvel_max[index] = current_velocity
            rel_ratio[index] = 0.0
        elif current_velocity >= previous_max:
            relvel_max[index] = current_velocity
            rel_ratio[index] = 1.0
        else:
            relvel_max[index] = previous_max
            rel_ratio[index] = current_velocity / previous_max
    return rel_ratio, relvel_max


def _judge_contact_patch_id(side: str, y_contact: float, track_profile: TrackProfileSet) -> int:
    if side == "L":
        return 1
    r1 = np.asarray(track_profile.profile.get("R1", np.empty((0, 2))), dtype=float)
    r2 = np.asarray(track_profile.profile.get("R2", np.empty((0, 2))), dtype=float)
    r3 = np.asarray(track_profile.profile.get("R3", np.empty((0, 2))), dtype=float)
    if r2.size == 0 and r3.size == 0:
        return 2
    if r1.size == 0 and r3.size == 0:
        return 3
    if r1.size == 0 and r2.size == 0:
        return 4
    if r1.size and r2.size:
        return 2 if y_contact > float(np.min(r1[:, 0])) else 3
    if r2.size and r3.size:
        return 3 if y_contact > float(np.min(r2[:, 0])) else 4
    return 2 if r1.size else 3


def _contact_radii(
    wheel_radius_profile: np.ndarray,
    rail_radius_profile: np.ndarray,
    con_wheel_2: np.ndarray,
    con_rail_1: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    r_yy_w = np.where(np.isfinite(con_wheel_2[:, 2]), con_wheel_2[:, 2], 0.43)
    r_xx_w = np.interp(con_wheel_2[:, 1], wheel_radius_profile[:, 0], wheel_radius_profile[:, 1])
    r_xx_r = np.interp(con_rail_1[:, 0], rail_radius_profile[:, 0], rail_radius_profile[:, 1])
    adjust = (r_xx_w < 0.0) & (np.abs(r_xx_w) * 0.9 <= np.abs(r_xx_r))
    r_xx_r = np.where(adjust, np.abs(r_xx_w) * 0.9, r_xx_r)
    r_xx_r = np.where(np.abs(r_xx_r) > 1.0, 1.0, r_xx_r)
    rou = 4.0 / (1.0 / r_yy_w + 1.0 / r_xx_w + 1.0 / r_xx_r)
    return r_yy_w, r_xx_w, r_xx_r, rou


def _hertz_parameters(
    r_yy_w: np.ndarray,
    r_xx_w: np.ndarray,
    r_xx_r: np.ndarray,
    rou: np.ndarray,
    *,
    elastic_modulus: float,
    poisson_ratio: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    tables = contact_tables()
    beta = np.arccos(np.clip(rou / 4.0 * np.abs(1.0 / r_yy_w - 1.0 / r_xx_w - 1.0 / r_xx_r), -1.0, 1.0))
    m = np.interp(beta, tables.bgmn[:, 0], tables.bgmn[:, 1])
    n = np.interp(beta, tables.bgmn[:, 0], tables.bgmn[:, 2])
    a1 = np.zeros_like(rou)
    b1 = np.zeros_like(rou)
    swap = rou / r_yy_w > 2.0
    a1[~swap] = 0.1506e-3 * m[~swap] * np.cbrt(rou[~swap])
    b1[~swap] = 0.1506e-3 * n[~swap] * np.cbrt(rou[~swap])
    a1[swap] = 0.1506e-3 * n[swap] * np.cbrt(rou[swap])
    b1[swap] = 0.1506e-3 * m[swap] * np.cbrt(rou[swap])
    con_a = 0.5 / r_yy_w
    con_b = 0.5 * (1.0 / r_xx_w + 1.0 / r_xx_r)

    bgmnr = np.array(
        [
            [0.0, 1.0, 1.0, 1.0],
            [0.1711, 1.1257, 0.8942, 0.9934],
            [0.3329, 1.2754, 0.8047, 0.9741],
            [0.4781, 1.4536, 0.7285, 0.9436],
            [0.6022, 1.6652, 0.6629, 0.9036],
            [0.7036, 1.9160, 0.6059, 0.8566],
            [0.7836, 2.2121, 0.5557, 0.8048],
            [0.8446, 2.5609, 0.5110, 0.7503],
            [0.8900, 2.9708, 0.4708, 0.6949],
            [0.9231, 3.4514, 0.4345, 0.6398],
            [0.9467, 4.0141, 0.4014, 0.5861],
            [0.9634, 4.6721, 0.3711, 0.5346],
            [0.9750, 5.4410, 0.3433, 0.4859],
            [0.9831, 6.3387, 0.3177, 0.4401],
            [0.9886, 7.3864, 0.2941, 0.3974],
            [0.9923, 8.6088, 0.2722, 0.3580],
            [0.9949, 10.0346, 0.2521, 0.3217],
            [0.9966, 11.6976, 0.2334, 0.2885],
            [0.9977, 13.6370, 0.2161, 0.2582],
            [0.9985, 15.8984, 0.2001, 0.2307],
            [0.9990, 18.5353, 0.1854, 0.2058],
        ],
        dtype=float,
    )
    cos_beta = np.abs(1.0 / r_yy_w - 1.0 / r_xx_w - 1.0 / r_xx_r) / (1.0 / r_yy_w + 1.0 / r_xx_w + 1.0 / r_xx_r)
    con_r = np.interp(cos_beta, bgmnr[:, 0], bgmnr[:, 3])

    elastic_permeability = np.zeros_like(rou)
    for i in range(rou.size):
        major = max(a1[i], b1[i])
        minor = min(a1[i], b1[i])
        psi = max(minor / max(major, np.finfo(float).eps), 1.0e-6)
        temp = np.trapezoid(
            1.0 / np.sqrt(1.0 - (1.0 - psi**2) * _ELLIPTIC_INTEGRAL_SIN2),
            _ELLIPTIC_INTEGRAL_PHI,
        )
        elastic_permeability[i] = 3.0 * (1.0 - poisson_ratio**2) / (pi * elastic_modulus * max(major, np.finfo(float).eps)) * temp
    return m, n, elastic_permeability, con_a, con_b, con_r


def _normal_force_track_component(normal_force: float, angle: float) -> np.ndarray:
    transform = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(angle), np.sin(angle)],
            [0.0, -np.sin(angle), np.cos(angle)],
        ],
        dtype=float,
    )
    temp = np.array([0.0, 0.0, -float(normal_force)], dtype=float) @ transform
    return temp[1:3]


def _wheelset_distances(vehicle_parameters: Mapping[str, Any], exp_ws: list[str]) -> dict[str, float]:
    ll1 = float(vehicle_parameters["Ll1"])
    ll2 = float(vehicle_parameters["Ll2"])
    lookup = {
        "FF": 0.0,
        "FR": 2.0 * ll1,
        "RF": 2.0 * ll2,
        "RR": 2.0 * (ll1 + ll2),
    }
    return {wheelset: lookup[wheelset] for wheelset in exp_ws}


def _nominal_contact_force(vehicle_parameters: Mapping[str, Any]) -> float:
    mw = float(vehicle_parameters["Mw"])
    mb = float(vehicle_parameters["Mb"])
    mc = float(vehicle_parameters["Mc"])
    return 9.81 * (mw + 0.5 * mb + 0.25 * mc) / 2.0


def _wheelset_state_base(inp_par: Mapping[str, Any], wheel_index: int) -> int:
    return int(inp_par["N_track"]) + int(inp_par["NM_FW"]) * int(inp_par["Nw"]) + 5 * wheel_index


def _state_value(state: np.ndarray, index: int) -> float:
    array = np.asarray(state, dtype=float)
    if array.ndim == 1:
        return float(array[index])
    if array.ndim == 2 and array.shape[1] > 3:
        return float(array[index, 3])
    if array.ndim == 2 and array.shape[1] == 1:
        return float(array[index, 0])
    raise ValueError("state arrays must be vectors, single-column arrays, or MATLAB-style arrays with column 4")


def _right_matrix_divide(values: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return np.linalg.solve(np.asarray(matrix, dtype=float).T, np.asarray(values, dtype=float).T).T


def _empty_side_result() -> dict[str, np.ndarray]:
    return {
        "normal_force": np.zeros((0, 4), dtype=float),
        "con_wheel_2": np.zeros((0, 3), dtype=float),
        "con_wheel_2_full": np.zeros((0, 6), dtype=float),
        "con_rail_1": np.zeros((0, 2), dtype=float),
        "con_wheel_2_peak": np.zeros((0, 6), dtype=float),
        "con_rail_1_peak": np.zeros((0, 2), dtype=float),
        "penetration_peaks": np.zeros((0, 4), dtype=float),
        "con_rel_vel": np.zeros((0, 2), dtype=float),
        "con_rel_vel_max": np.zeros((0,), dtype=float),
        "prh": np.zeros((0, 3), dtype=float),
        "prhx_t": np.zeros((0, 3), dtype=float),
        "prhxf_t": np.zeros((0, 6), dtype=float),
        "patch_ids": np.zeros((0,), dtype=int),
        "rhxs": np.zeros((0, 4), dtype=float),
        "rhlv": np.zeros((0, 3), dtype=float),
        "a2": np.zeros((0,), dtype=float),
        "b2": np.zeros((0,), dtype=float),
        "vjd": np.zeros((0, 3), dtype=float),
        "vjd_r": np.zeros((0, 3), dtype=float),
        "vsdc": np.zeros((0, 3), dtype=float),
        "vjsdc": np.zeros((0, 3), dtype=float),
        "vgd": np.zeros((0,), dtype=float),
        "elastic_normal_force": np.zeros((0, 6), dtype=float),
        "area_stripes": np.zeros((0,), dtype=float),
        "epsilon": np.zeros((0,), dtype=float),
        "con_stripes": (),
        "normal_damping_clips": (),
    }
