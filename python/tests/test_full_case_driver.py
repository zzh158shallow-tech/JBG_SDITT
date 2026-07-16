from __future__ import annotations

import json
import os
import subprocess
import tempfile
import uuid
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from sditt.config import DefaultOperatingCase
from sditt.io.matlab import load_mat_file
from sditt.simulation import (
    FullCaseFrozenContactInput,
    FullDefaultCaseSettings,
    MissingFullCasePhysicsError,
    find_default_full_case_checkpoint,
    list_default_full_case_checkpoints,
    prepare_default_full_case,
    run_default_full_case_driver,
)
from sditt.simulation.full_case import (
    _matlab_mileage_step_dt,
    _preload_cache_key,
    _run_checkpoint_key,
    _stage_step_count,
)
from sditt.track import TrackIrregularitySettings, rail_dyn_modal_ft, wr_force_modal_ft
from sditt.vehicle import build_nonlinear_damper_response, wr_force_vehicle_sys_rotation_iii


MATLAB_CLI = Path("/Applications/MATLAB_R2026a.app/bin/matlab")
MATLAB_VALIDATION_ENV = "SDITT_RUN_MATLAB_BASELINES"


def _matlab_validation_enabled() -> bool:
    return os.environ.get(MATLAB_VALIDATION_ENV) == "1"


def _matlab_quote(path: Path) -> str:
    return str(path).replace("'", "''")


def _iteration_snapshot(result: object) -> dict[str, object]:
    assert hasattr(result, "stages")
    return {
        "stages": [
            {
                "name": stage.stage,
                "iteration_records": [
                    {
                        "step_index": record.step_index,
                        "iteration": record.iteration,
                        "time": record.time,
                        "dt": record.dt,
                        "front_mileage": record.front_mileage,
                        "converged": record.converged,
                        "pjc": np.asarray(record.pjc, dtype=float).tolist(),
                        "pjch": np.asarray(record.pjch, dtype=float).tolist(),
                        "pjcc": np.asarray(record.pjcc, dtype=float).tolist(),
                        "prhxf": np.asarray(record.prhxf, dtype=float).tolist(),
                    }
                    for record in stage.iteration_records
                ],
            }
            for stage in result.stages
        ]
    }


def _run_matlab_iteration_output_baseline(snapshot_path: Path, output_path: Path) -> None:
    command = (
        f"cd('{_matlab_quote(Path(__file__).resolve().parents[1] / 'matlab')}'); "
        f"export_iteration_output_baseline('{_matlab_quote(snapshot_path)}', '{_matlab_quote(output_path)}');"
    )
    try:
        completed = subprocess.run(
            [str(MATLAB_CLI), "-batch", command],
            check=True,
            cwd=Path(__file__).resolve().parents[2],
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        message = (exc.stderr or exc.stdout or str(exc)).strip()
        pytest.skip(f"MATLAB CLI is not runnable in the current sandbox: {message}")
    if completed.stderr:
        stderr = completed.stderr.strip()
        if stderr:
            pytest.skip(f"MATLAB CLI reported a sandbox/environment issue: {stderr}")


def test_prepare_default_full_case_builds_low_cutoff_system_and_reports_gaps() -> None:
    preparation = prepare_default_full_case(settings=FullDefaultCaseSettings(cut_freq=50.0))

    assert preparation.total_dof == 56
    assert preparation.system.layout.n_track == 5
    assert preparation.stage_inp_par["Preload"]["Type_simulation"] == "Preload"
    assert preparation.stage_inp_par["Cal"]["Type_simulation"] == "Cal"
    assert preparation.stage_inp_par["Cal"]["Type_Layout"] == "Straight"
    assert preparation.is_physical_complete
    assert preparation.gravity_preload.pxt_gravity.shape == (56,)
    assert preparation.gravity_preload.pxt_gravity[5] == pytest.approx(9.81 * 1901.8)
    assert preparation.gravity_preload.pxt_gravity[25] == pytest.approx(9.81 * 2280.0)
    assert preparation.gravity_preload.pxt_gravity[35] == pytest.approx(9.81 * 40840.0)
    assert preparation.gravity_preload.pxt_gravity[:5].any()
    assert preparation.operating_case.rail_layout == "interval"
    assert preparation.operating_case.n_contact_patch == 2
    assert preparation.stage_inp_par["Cal"]["Exp_DummyRail"] == ("L1", "R1")
    assert preparation.stage_inp_par["Cal"]["Type_Rail"] == ("zjbg", "qjbg")
    assert preparation.profile_selector.select(54.0)["R1"].by_station["FF"].profile_num == 1
    assert preparation.shape_function_context.n_track == 5
    assert preparation.stage_inp_par["Cal"]["N_track"] == 5
    assert preparation.stage_inp_par["Cal"]["ModeShape"]["qjbg"].shape == (4084, 5)
    assert "wheel_rail_contact_main" not in {stage.name for stage in preparation.missing_stages}
    assert "gravity_load_modal_ft" not in {stage.name for stage in preparation.missing_stages}
    assert "main_profile_selection" not in {stage.name for stage in preparation.missing_stages}
    assert "beam_shape_function" not in {stage.name for stage in preparation.missing_stages}
    assert "nonlinear_vehicle_dampers" not in {stage.name for stage in preparation.missing_stages}
    assert "curve_external_force" not in {stage.name for stage in preparation.missing_stages}


def test_prepare_turnout_full_case_preserves_original_four_rail_contact_layout() -> None:
    preparation = prepare_default_full_case(
        settings=FullDefaultCaseSettings(cut_freq=50.0),
        operating_case=DefaultOperatingCase(rail_layout="turnout"),
    )

    assert preparation.operating_case.n_contact_patch == 4
    assert preparation.stage_inp_par["Cal"]["Exp_DummyRail"] == ("L1", "R1", "R2", "R3")
    assert preparation.profile_selector.select(54.0)["R1"].by_station["FF"].profile_num == 22
    assert "R2_zjg" in preparation.shape_function_context.rail_beam.pos_z


def test_interval_and_turnout_use_distinct_cache_and_checkpoint_fingerprints() -> None:
    settings = FullDefaultCaseSettings(cut_freq=50.0, preload_cache_dir="cache", checkpoint_dir="checkpoints")
    interval = prepare_default_full_case(settings=settings)
    turnout = prepare_default_full_case(
        settings=settings,
        operating_case=DefaultOperatingCase(rail_layout="turnout"),
    )

    assert _preload_cache_key(interval) != _preload_cache_key(turnout)
    assert _run_checkpoint_key(interval) != _run_checkpoint_key(turnout)


def test_irregularity_seed_changes_cache_and_checkpoint_fingerprints() -> None:
    first_settings = FullDefaultCaseSettings(
        cut_freq=50.0,
        track_irregularity=TrackIrregularitySettings(model="china-ballastless", seed=1),
    )
    second_settings = replace(
        first_settings,
        track_irregularity=TrackIrregularitySettings(model="china-ballastless", seed=2),
    )
    first = prepare_default_full_case(settings=first_settings)
    second = prepare_default_full_case(settings=second_settings)

    assert _preload_cache_key(first) != _preload_cache_key(second)
    assert _run_checkpoint_key(first) != _run_checkpoint_key(second)


@pytest.mark.parametrize("rail_layout", ["interval", "turnout"])
def test_full_case_applies_same_side_irregularity_to_all_dummy_rails(rail_layout: str) -> None:
    settings = FullDefaultCaseSettings(
        cut_freq=50.0,
        n_steps_per_stage=1,
        track_irregularity=TrackIrregularitySettings(model="china-ballastless", seed=123),
    )
    result = run_default_full_case_driver(
        settings=settings,
        operating_case=DefaultOperatingCase(rail_layout=rail_layout),
    )
    cal = result.stages[-1]
    contact = cal.history.contact_geometry[-1].wheel_rail_contact
    assert contact is not None
    assert result.preparation.track_irregularity_profile is not None

    for wheel_index, wheelset in enumerate(result.preparation.operating_case.wheelsets):
        metadata = contact.con_ws[wheelset]["Track_Irregularity"]
        assert metadata
        track_profile = contact.track_profiles[wheelset]
        for patch_index, (dummy_rail, side) in enumerate(
            zip(
                result.preparation.operating_case.dummy_rails,
                result.preparation.operating_case.dummy_rail_wheel_side,
                strict=True,
            )
        ):
            contact_index = result.preparation.operating_case.n_contact_patch * wheel_index + patch_index
            irregularity_y, irregularity_z = metadata["rail_displacement_m"][side]
            assert track_profile.offsets[dummy_rail][0] == pytest.approx(
                cal.history.rail_response[-1].dis_rail[contact_index, 1] + irregularity_y
            )
            assert track_profile.offsets[dummy_rail][1] == pytest.approx(
                0.6 + cal.history.rail_response[-1].dis_rail[contact_index, 2] + irregularity_z
            )
        for side in ("L", "R"):
            assert np.all(np.isfinite(metadata["rail_velocity_m_per_s"][side]))


def test_run_default_full_case_driver_converges_with_default_full_size_settings() -> None:
    result = run_default_full_case_driver(settings=FullDefaultCaseSettings())

    assert [stage.stage for stage in result.stages] == ["Preload", "Cal"]
    assert result.is_physical_complete
    assert result.preparation.total_dof == 6248
    assert result.preparation.missing_stages == ()
    assert result.stages[0].history.iterations[0] == 0
    assert result.stages[1].history.iterations[0] == 0
    assert np.all(result.stages[0].history.iterations[1:] >= 1)
    assert np.all(result.stages[1].history.iterations[1:] >= 1)
    assert result.stages[0].history.contact_geometry[-1].wheel_rail_contact is not None
    assert result.stages[1].history.contact_geometry[-1].wheel_rail_contact is not None
    assert result.stages[0].output_rows[-1].contact_state is not None
    assert result.stages[1].output_rows[-1].contact_state is not None


def test_full_size_default_reuses_same_d0_within_each_nonlinear_step() -> None:
    result = run_default_full_case_driver(settings=FullDefaultCaseSettings())

    preload = result.stages[0]
    step_one_records = [record for record in preload.iteration_records if record.step_index == 1]

    assert len(step_one_records) > 1
    assert step_one_records[0].contact_state is not None
    expected = step_one_records[0].contact_state.d0_by_wheelset
    for record in step_one_records[1:]:
        assert record.contact_state is not None
        assert record.contact_state.d0_by_wheelset == expected


def test_full_size_default_records_step_contact_state_on_outputs() -> None:
    result = run_default_full_case_driver(settings=FullDefaultCaseSettings())

    preload, cal = result.stages

    assert preload.output_rows[-1].contact_state is not None
    assert cal.output_rows[-1].contact_state is not None
    assert set(preload.output_rows[-1].contact_state.d0_by_wheelset) == set(result.preparation.operating_case.wheelsets)
    assert set(cal.output_rows[-1].contact_state.d0_by_wheelset) == set(result.preparation.operating_case.wheelsets)


def test_full_case_driver_carries_front_mileage_across_preload_and_cal() -> None:
    result = run_default_full_case_driver(settings=FullDefaultCaseSettings())

    preload, cal = result.stages

    assert preload.output_rows[-1].front_mileage > _initial_stage_front_mileage_for_test(result)
    assert cal.output_rows[0].front_mileage > preload.output_rows[-1].front_mileage


def test_full_case_driver_carries_contact_state_from_preload_to_cal() -> None:
    result = run_default_full_case_driver(settings=FullDefaultCaseSettings())

    preload, cal = result.stages
    preload_state = preload.output_rows[-1].contact_state
    cal_step_one_records = [record for record in cal.iteration_records if record.step_index == 1]

    assert preload_state is not None
    assert cal_step_one_records
    assert cal_step_one_records[0].contact_state is not None
    assert cal_step_one_records[0].contact_state.d0_by_wheelset == preload_state.d0_by_wheelset
    assert preload_state.pjc is not None
    assert preload_state.pjch is not None
    assert preload_state.pjcc is not None
    assert preload_state.prhx is not None
    assert preload_state.prhxf is not None
    assert preload_state.con_ws is not None
    assert preload_state.pjc.shape == (8, 2)
    assert preload_state.pjch.shape == (8, 1)
    assert preload_state.pjcc.shape == (8, 1)
    assert preload_state.prhx.shape == (8, 3)
    assert preload_state.prhxf.shape == (8, 6)
    assert set(preload_state.con_ws) >= set(result.preparation.operating_case.wheelsets)
    assert set(preload_state.relvel_max_by_wheelset) == set(result.preparation.operating_case.wheelsets)
    assert set(cal_step_one_records[0].contact_state.relvel_max_by_wheelset) == set(
        result.preparation.operating_case.wheelsets
    )
    assert cal_step_one_records[0].normal_error > 0.0


def test_run_default_full_case_driver_executes_preload_then_cal_in_diagnostic_mode() -> None:
    progress_events = []
    result = run_default_full_case_driver(
        settings=FullDefaultCaseSettings(
            cut_freq=50.0,
            dt=1.0e-4,
            n_steps_per_stage=2,
            progress_callback=progress_events.append,
        )
    )

    assert [stage.stage for stage in result.stages] == ["Preload", "Cal"]
    assert result.is_physical_complete
    assert result.preparation.total_dof == 56

    preload, cal = result.stages
    assert preload.history.displacement.shape == (3, 56)
    assert cal.history.displacement.shape == (3, 56)
    assert len(preload.iteration_records) >= 2
    assert len(cal.iteration_records) >= 2
    assert len(preload.output_rows) == 2
    assert len(cal.output_rows) == 2
    assert len(result.iteration_records) == len(preload.iteration_records) + len(cal.iteration_records)
    assert len(result.output_rows) == 4
    assert preload.history.iterations[0] == 0
    assert cal.history.iterations[0] == 0
    assert preload.history.iterations[1:].tolist() == [row.iterations for row in preload.output_rows]
    assert cal.history.iterations[1:].tolist() == [row.iterations for row in cal.output_rows]
    assert preload.convergence_history.shape == (len(preload.iteration_records), 7)
    assert cal.convergence_history.shape == (len(cal.iteration_records), 7)
    assert preload.output_table.shape == (2, 7)
    assert cal.output_table.shape == (2, 7)
    assert sum(record.converged for record in preload.iteration_records) == 2
    assert sum(record.converged for record in cal.iteration_records) == 2
    assert preload.iteration_records[-1].converged
    assert cal.iteration_records[-1].converged
    assert cal.history.rail_response[-1].stage == "Cal"
    assert cal.history.rail_response[-1].physical_rail_recovered
    assert cal.history.rail_response[-1].shape_function["FF_qjbg_Y"].size == 4
    assert cal.history.rail_response[-1].dis_rail.shape == (8, 6)
    assert cal.history.rail_response[-1].vel_rail.shape == (8, 6)
    assert cal.history.rail_response[-1].acc_rail.shape == (8, 6)
    assert "qjbg_Dis" in cal.history.rail_response[-1].dyn_status_rail
    assert cal.history.rail_response[-1].missing_reason is None
    assert cal.history.contact_geometry[-1].contact_force_enabled
    assert cal.history.contact_geometry[-1].wheel_rail_contact is not None
    assert cal.history.contact_geometry[-1].rail_profile_numbers["L1"]["FF"] == 1
    assert cal.history.contact_geometry[-1].front_mileage == pytest.approx(cal.history.rail_response[-1].front_mileage)
    assert np.linalg.norm(cal.history.contact_force[-1]) > 0.0
    assert np.linalg.norm(cal.history.contact_geometry[-1].wheel_rail_contact.pjcc) > 0.0
    assert cal.output_rows[-1].iterations == cal.history.iterations[-1]
    assert cal.output_rows[0].front_mileage > preload.output_rows[-1].front_mileage
    assert cal.output_rows[-1].front_mileage == pytest.approx(cal.history.rail_response[-1].front_mileage)
    assert cal.output_rows[-1].vehicle_displacement.shape == (51,)
    assert cal.output_rows[-1].contact_state is not None
    assert cal.output_rows[-1].pjcc.shape == (8, 1)
    assert cal.output_rows[-1].prhxf.shape == (8, 6)
    assert cal.output_rows[-1].patch_force_z.shape == (8,)
    assert cal.output_rows[-1].wheelset_vertical_force.shape == (4, 2)
    assert cal.output_rows[-1].wheelset_lateral_force.shape == (4, 2)
    assert progress_events[-1].patch_force_labels == (
        "FF-L1",
        "FF-R1",
        "FR-L1",
        "FR-R1",
        "RF-L1",
        "RF-R1",
        "RR-L1",
        "RR-R1",
    )

    expected_shape_function, _ = result.preparation.shape_function_context.evaluate(cal.history.rail_response[-1].front_mileage)
    expected_dis_rail, expected_vel_rail, expected_acc_rail, expected_dyn_status = rail_dyn_modal_ft(
        result.preparation.stage_inp_par["Cal"],
        cal.history.displacement[-1, result.preparation.system.layout.track],
        cal.history.velocity[-1, result.preparation.system.layout.track],
        cal.history.acceleration[-1, result.preparation.system.layout.track],
        expected_shape_function,
    )
    assert set(cal.history.rail_response[-1].shape_function) == set(expected_shape_function)
    assert np.allclose(cal.history.rail_response[-1].dis_rail, expected_dis_rail)
    assert np.allclose(cal.history.rail_response[-1].vel_rail, expected_vel_rail)
    assert np.allclose(cal.history.rail_response[-1].acc_rail, expected_acc_rail)
    for key, value in expected_dyn_status.items():
        assert np.allclose(cal.history.rail_response[-1].dyn_status_rail[key], value)

    wheel_rail_contact = cal.history.contact_geometry[-1].wheel_rail_contact
    assert wheel_rail_contact is not None
    ff_track_profile = wheel_rail_contact.track_profiles["FF"]
    assert ff_track_profile.offsets["L1"][0] == pytest.approx(cal.history.rail_response[-1].dis_rail[0, 1])
    assert ff_track_profile.offsets["L1"][1] == pytest.approx(0.6 + cal.history.rail_response[-1].dis_rail[0, 2])
    assert ff_track_profile.offsets["R1"][0] == pytest.approx(cal.history.rail_response[-1].dis_rail[1, 1])
    assert ff_track_profile.offsets["R1"][1] == pytest.approx(0.6 + cal.history.rail_response[-1].dis_rail[1, 2])
    assert "R2" not in ff_track_profile.offsets
    assert "R3" not in ff_track_profile.offsets
    assert cal.history.rail_response[-1].rail_beam_motion["Pos_Z"] == {}
    assert cal.history.rail_response[-1].rail_beam_motion["Vel_Z"] == {}
    for wheelset in result.preparation.operating_case.wheelsets:
        right_normal_force = np.asarray(wheel_rail_contact.con_ws[wheelset]["Normal_Force"]["R"], dtype=float)
        if right_normal_force.size:
            assert np.all(right_normal_force[:, 3] == 2)
    vjd_blocks = [
        np.asarray(wheel_rail_contact.con_ws[wheelset]["Vjd"][side], dtype=float)
        for wheelset in result.preparation.operating_case.wheelsets
        for side in ("L", "R")
        if np.asarray(wheel_rail_contact.con_ws[wheelset]["Vjd"][side]).size
    ]
    vjd_r_blocks = [
        np.asarray(wheel_rail_contact.con_ws[wheelset]["Vjd_r"][side], dtype=float)
        for wheelset in result.preparation.operating_case.wheelsets
        for side in ("L", "R")
        if np.asarray(wheel_rail_contact.con_ws[wheelset]["Vjd_r"][side]).size
    ]
    creepage_blocks = [
        np.asarray(wheel_rail_contact.con_ws[wheelset]["RHLv"][side], dtype=float)
        for wheelset in result.preparation.operating_case.wheelsets
        for side in ("L", "R")
        if np.asarray(wheel_rail_contact.con_ws[wheelset]["RHLv"][side]).size
    ]
    assert any(np.linalg.norm(block) > 0.0 for block in vjd_blocks)
    assert any(np.linalg.norm(block) > 0.0 for block in vjd_r_blocks)
    assert any(np.linalg.norm(block) > 0.0 for block in creepage_blocks)
    nonlinear = build_nonlinear_damper_response(
        result.preparation.stage_inp_par["Cal"],
        result.preparation.vehicle_parameters.values,
        result.preparation.vehicle,
        cal.history.displacement[-1],
        cal.history.velocity[-1],
    )
    expected_contact_force = nonlinear.equivalent_force.reshape(-1, 1)
    expected_contact_force, _ = wr_force_modal_ft(
        result.preparation.stage_inp_par["Cal"],
        wheel_rail_contact.xlcs,
        expected_contact_force,
        wheel_rail_contact.pjcc,
        wheel_rail_contact.pjch,
        wheel_rail_contact.prhxf,
        wheel_rail_contact.con_ws,
        cal.history.rail_response[-1].shape_function,
    )
    expected_contact_force, _ = wr_force_vehicle_sys_rotation_iii(
        result.preparation.stage_inp_par["Cal"],
        result.preparation.vehicle_parameters.values,
        {"Br": 0.7175},
        {},
        expected_contact_force,
        np.zeros_like(cal.history.displacement[-1]),
        wheel_rail_contact.pjcc,
        wheel_rail_contact.pjch,
        wheel_rail_contact.prhxf,
        wheel_rail_contact.con_ws,
    )
    assert np.allclose(cal.history.contact_force[-1], expected_contact_force[:, 0])
    assert np.allclose(
        cal.history.total_force[-1],
        result.preparation.gravity_preload.pxt_gravity + expected_contact_force[:, 0],
    )


def test_preload_cache_reuses_completed_preload_state() -> None:
    cache_dir = Path("outputs") / "test_preload_cache" / uuid.uuid4().hex
    first_events = []
    second_events = []
    first_settings = FullDefaultCaseSettings(
        cut_freq=50.0,
        dt=1.0e-4,
        n_steps_per_stage=1,
        preload_cache_dir=cache_dir,
        progress_callback=first_events.append,
    )
    second_settings = FullDefaultCaseSettings(
        cut_freq=50.0,
        dt=1.0e-4,
        n_steps_per_stage=1,
        preload_cache_dir=cache_dir,
        progress_callback=second_events.append,
    )
    first = run_default_full_case_driver(settings=first_settings)
    second = run_default_full_case_driver(settings=second_settings)

    assert first.preload_cache_status == "saved"
    assert first.preload_cache_path is not None
    assert first.preload_cache_path.exists()
    assert [stage.stage for stage in first.stages] == ["Preload", "Cal"]
    assert second.preload_cache_status == "hit"
    assert [stage.stage for stage in second.stages] == ["Cal"]
    assert second.stages[0].output_rows[0].front_mileage > first.stages[0].output_rows[-1].front_mileage
    assert second.stages[0].history.timing
    assert [event.stage for event in first_events] == ["Preload", "Cal"]
    assert [event.stage for event in second_events] == ["Preload", "Cal"]
    assert second_events[0].front_mileage == pytest.approx(first_events[0].front_mileage)
    assert all(event.step_wall_time >= 0.0 for event in second_events)


def test_run_checkpoint_saves_non_overwriting_archives_and_resumes_latest_matching_state() -> None:
    checkpoint_dir = Path("outputs") / "test_run_checkpoints" / uuid.uuid4().hex
    first_events = []
    resumed_events = []
    settings = FullDefaultCaseSettings(
        cut_freq=50.0,
        dt=1.0e-4,
        stage_end_mileage={"Preload": 32.03, "Cal": 32.12},
        checkpoint_dir=checkpoint_dir,
        save_checkpoints=True,
        checkpoint_interval_m=0.03,
        history_retention_steps=None,
    )

    first = run_default_full_case_driver(settings=replace(settings, progress_callback=first_events.append))
    summaries = list_default_full_case_checkpoints(settings=settings)
    summary = find_default_full_case_checkpoint(settings=settings)
    resumed = run_default_full_case_driver(
        settings=replace(settings, save_checkpoints=False, resume_checkpoint=True, progress_callback=resumed_events.append)
    )

    assert first.checkpoint_status == "saved"
    assert first.checkpoint_path is not None
    assert first.checkpoint_path.exists()
    assert len(summaries) >= 2
    assert len({Path(item["path"]).name for item in summaries}) == len(summaries)
    assert summary is not None
    assert summary["path"] == summaries[0]["path"]
    assert summary["stage"] in {"Preload", "Cal"}
    assert resumed.resumed_from_checkpoint
    assert resumed.resumed_checkpoint_mileage == pytest.approx(float(summary["front_mileage"]))
    assert resumed.output_rows[0].front_mileage > float(summary["front_mileage"])
    replayed = first_events[: len(resumed_events) - len(resumed.output_rows)]
    assert replayed
    assert [event.front_mileage for event in resumed_events[: len(replayed)]] == pytest.approx(
        [event.front_mileage for event in replayed]
    )


def test_run_checkpoint_resumes_from_selected_archive_instead_of_latest() -> None:
    checkpoint_dir = Path("outputs") / "test_run_checkpoints" / uuid.uuid4().hex
    settings = FullDefaultCaseSettings(
        cut_freq=50.0,
        dt=1.0e-4,
        stage_end_mileage={"Preload": 32.03, "Cal": 32.12},
        checkpoint_dir=checkpoint_dir,
        save_checkpoints=True,
        checkpoint_interval_m=0.03,
        history_retention_steps=None,
    )

    run_default_full_case_driver(settings=settings)
    summaries = list_default_full_case_checkpoints(settings=settings)
    assert len(summaries) >= 2
    selected = summaries[-1]
    latest = summaries[0]
    resumed = run_default_full_case_driver(
        settings=replace(
            settings,
            save_checkpoints=False,
            resume_checkpoint=True,
            resume_checkpoint_path=selected["path"],
        )
    )

    assert float(selected["front_mileage"]) < float(latest["front_mileage"])
    assert resumed.resumed_from_checkpoint
    assert resumed.checkpoint_path == Path(selected["path"])
    assert resumed.resumed_checkpoint_mileage == pytest.approx(float(selected["front_mileage"]))


def test_run_checkpoint_ignores_incompatible_archives() -> None:
    checkpoint_dir = Path("outputs") / "test_run_checkpoints" / uuid.uuid4().hex
    settings = FullDefaultCaseSettings(
        cut_freq=50.0,
        dt=1.0e-4,
        stage_end_mileage={"Preload": 32.03, "Cal": 32.12},
        checkpoint_dir=checkpoint_dir,
        save_checkpoints=True,
        checkpoint_interval_m=0.05,
        history_retention_steps=None,
    )

    run_default_full_case_driver(settings=settings)
    compatible = list_default_full_case_checkpoints(settings=settings)
    incompatible = list_default_full_case_checkpoints(settings=replace(settings, dt=2.0e-4))
    resumed = run_default_full_case_driver(
        settings=replace(
            settings,
            dt=2.0e-4,
            save_checkpoints=False,
            resume_checkpoint=True,
            resume_checkpoint_path=compatible[0]["path"],
        )
    )

    assert compatible
    assert incompatible == ()
    assert not resumed.resumed_from_checkpoint
    assert resumed.checkpoint_status == "miss"


def _initial_stage_front_mileage_for_test(result) -> float:
    preparation = result.preparation
    if preparation.operating_case.vehicle_direction == "Face":
        return 32.0 - 2.0 * (preparation.operating_case.vlc * 3.6 - 350.0) / 50.0
    return 32.0


def test_run_default_full_case_driver_maps_frozen_contact_force_into_track_and_vehicle() -> None:
    pjcc = np.zeros((16, 1), dtype=float)
    pjch = np.zeros((16, 1), dtype=float)
    prhxf = np.zeros((16, 6), dtype=float)
    for index, scale in zip((0, 1, 4, 5, 8, 9, 12, 13), (1.0, 1.5, 0.8, 1.2, 0.6, 1.1, 0.7, 0.9), strict=True):
        pjcc[index, 0] = 100.0 * scale
        pjch[index, 0] = 0.0
        prhxf[index, :] = np.array([2.0, 0.0, 4.0, 0.0, 5.0, 6.0]) * scale
    for index, scale in zip((3, 7, 11, 15), (0.5, 0.4, 0.3, 0.2), strict=True):
        pjcc[index, 0] = 50.0 * scale
        pjch[index, 0] = 0.0
        prhxf[index, :] = np.array([1.0, 0.0, 2.5, 0.0, 3.0, 4.0]) * scale

    frozen_input = FullCaseFrozenContactInput(
        xlcs=1,
        pjcc=pjcc,
        pjch=pjch,
        prhxf=prhxf,
        con_ws={"FF": {}},
    )
    result = run_default_full_case_driver(
        settings=FullDefaultCaseSettings(
            cut_freq=50.0,
            dt=1.0e-4,
            n_steps_per_stage=1,
            frozen_contact_input=frozen_input,
        ),
        operating_case=DefaultOperatingCase(rail_layout="turnout"),
    )

    cal = result.stages[1]
    rail = cal.history.rail_response[-1]
    assert cal.history.contact_geometry[-1].contact_force_enabled

    nonlinear = build_nonlinear_damper_response(
        result.preparation.stage_inp_par["Cal"],
        result.preparation.vehicle_parameters.values,
        result.preparation.vehicle,
        cal.history.displacement[-1],
        cal.history.velocity[-1],
    )
    expected_contact_force = nonlinear.equivalent_force.reshape(-1, 1)
    expected_contact_force, expected_track_force = wr_force_modal_ft(
        result.preparation.stage_inp_par["Cal"],
        frozen_input.xlcs,
        expected_contact_force,
        frozen_input.pjcc,
        frozen_input.pjch,
        frozen_input.prhxf,
        frozen_input.con_ws,
        rail.shape_function,
    )
    expected_contact_force, _ = wr_force_vehicle_sys_rotation_iii(
        result.preparation.stage_inp_par["Cal"],
        result.preparation.vehicle_parameters.values,
        {"Br": 0.7175},
        {},
        expected_contact_force,
        cal.history.displacement[-1],
        frozen_input.pjcc,
        frozen_input.pjch,
        frozen_input.prhxf,
        frozen_input.con_ws,
    )

    assert np.allclose(cal.history.contact_force[-1], expected_contact_force[:, 0])
    assert np.allclose(
        cal.history.total_force[-1],
        result.preparation.gravity_preload.pxt_gravity + expected_contact_force[:, 0],
    )
    assert np.linalg.norm(cal.history.contact_force[-1, : result.preparation.system.layout.n_track]) > 0.0
    assert np.linalg.norm(cal.history.contact_force[-1, result.preparation.system.layout.rigid_vehicle]) > 0.0
    assert np.linalg.norm(expected_track_force["zjbg"]) > 0.0
    assert np.linalg.norm(expected_track_force["qjbg"]) > 0.0


def test_run_default_full_case_driver_strict_mode_accepts_default_complete_route() -> None:
    result = run_default_full_case_driver(
        settings=FullDefaultCaseSettings(
            cut_freq=50.0,
            fail_on_missing_physics=True,
            n_steps_per_stage=1,
        )
    )

    assert result.is_physical_complete
    assert not result.preparation.missing_stages


def test_run_default_full_case_driver_strict_mode_still_rejects_curve_route() -> None:
    with pytest.raises(MissingFullCasePhysicsError, match="curve_external_force"):
        run_default_full_case_driver(
            settings=FullDefaultCaseSettings(
                cut_freq=50.0,
                fail_on_missing_physics=True,
            ),
            operating_case=DefaultOperatingCase(layout_type="Curve"),
        )


def test_matlab_mileage_step_dt_matches_crossing_small_step_windows() -> None:
    preparation = prepare_default_full_case(
        settings=FullDefaultCaseSettings(cut_freq=50.0),
        operating_case=DefaultOperatingCase(rail_layout="turnout"),
    )

    assert _matlab_mileage_step_dt(preparation, 59.9, 1.0e-4) == pytest.approx(1.0e-4)
    assert _matlab_mileage_step_dt(preparation, 60.0, 1.0e-4) == pytest.approx(5.0e-5)
    assert _matlab_mileage_step_dt(preparation, 104.0, 1.0e-4) == pytest.approx(2.5e-5)
    assert _matlab_mileage_step_dt(preparation, 105.0, 1.0e-4) == pytest.approx(5.0e-5)


def test_stage_step_count_accounts_for_matlab_mileage_small_steps() -> None:
    preparation = prepare_default_full_case(
        settings=FullDefaultCaseSettings(
            cut_freq=50.0,
            dt=1.0e-4,
            stage_end_mileage={"Cal": 104.01},
        ),
        operating_case=DefaultOperatingCase(rail_layout="turnout"),
    )

    steps = _stage_step_count(preparation, "Cal", 103.99)

    assert steps > int(np.ceil((104.01 - 103.99) / (preparation.operating_case.vlc * preparation.settings.dt)))


@pytest.mark.skipif(
    not MATLAB_CLI.exists() or not _matlab_validation_enabled(),
    reason=f"set {MATLAB_VALIDATION_ENV}=1 and install local MATLAB CLI to run MATLAB baseline validation",
)
def test_iteration_output_summary_matches_matlab_baseline() -> None:
    result = run_default_full_case_driver(
        settings=FullDefaultCaseSettings(
            cut_freq=50.0,
            dt=1.0e-4,
            n_steps_per_stage=2,
        ),
        operating_case=DefaultOperatingCase(rail_layout="turnout"),
    )

    with tempfile.TemporaryDirectory() as temp_dir:
        temp_root = Path(temp_dir)
        snapshot_path = temp_root / "sditt_iteration_snapshot.json"
        output_path = temp_root / "sditt_iteration_output_baseline.mat"
        snapshot_path.write_text(json.dumps(_iteration_snapshot(result)), encoding="utf-8")

        _run_matlab_iteration_output_baseline(snapshot_path, output_path)
        baseline = load_mat_file(output_path).variables

    preload, cal = result.stages
    assert np.allclose(np.asarray(baseline["preload_convergence_history"], dtype=float), preload.convergence_history)
    assert np.allclose(np.asarray(baseline["preload_output_table"], dtype=float), preload.output_table)
    assert np.allclose(np.asarray(baseline["cal_convergence_history"], dtype=float), cal.convergence_history)
    assert np.allclose(np.asarray(baseline["cal_output_table"], dtype=float), cal.output_table)
