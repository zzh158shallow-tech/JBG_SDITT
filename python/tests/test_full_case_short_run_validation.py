from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import sditt.validation.full_case_short_run as full_case_short_run_module
from sditt.simulation import FullCaseProgressEvent
from sditt.validation.full_case_short_run import (
    _ContactKeyDataRecorder,
    _ProgressRecorder,
    _format_elapsed_time,
    _format_timing_top,
    _parse_args,
    _sample_progress_events,
    _step_wall_times,
    build_full_case_short_run_report,
    build_python_short_run_snapshot,
    compare_short_run_snapshots,
    run_full_case_contact_key_data,
    write_full_case_short_run_report,
)


def test_contact_key_data_recorder_writes_only_accepted_force_and_location_fields(tmp_path) -> None:
    recorder = _ContactKeyDataRecorder(tmp_path / "wheel_rail_contact_key_data.csv")
    wheelset_contact = {
        "Mileage": 42.5,
        "Normal_Force": {
            "L": np.asarray([[100.0, 0.0, -100.0, 1.0]]),
            "R": np.zeros((0, 4)),
        },
        "Prhxf_T": {
            "L": np.asarray([[10.0, 20.0, 30.0, 0.0, 0.0, 0.0]]),
            "R": np.zeros((0, 6)),
        },
        "Con_wheel_2_full": {
            "L": np.asarray([[0.0, 0.01, 0.02, 0.003, 0.004, 0.5]]),
            "R": np.zeros((0, 6)),
        },
        "Con_rail_1": {
            "L": np.asarray([[0.011, 0.0]]),
            "R": np.zeros((0, 2)),
        },
    }
    recorder(
        SimpleNamespace(
            stage="Cal",
            step_index=7,
            iterations=2,
            time=0.1,
            dt=1.0e-4,
            front_mileage=50.0,
            dummy_rail_labels=("L1", "R1"),
            wheel_pose_by_wheelset={
                "FF": SimpleNamespace(lateral=0.0, vertical=0.0, roll=0.0, yaw=0.0)
            },
            wheel_rail_contact=SimpleNamespace(
                con_ws={"FF": wheelset_contact},
                d0_by_wheelset={"FF": 0.001},
            ),
        )
    )
    recorder.close()

    lines = (tmp_path / "wheel_rail_contact_key_data.csv").read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert "force_on_wheel_x_N" in lines[0]
    assert "wheel_contact_y_m" in lines[0]
    assert "network_b_fallback" in lines[0]
    values = lines[1].split(",")
    assert values[0:2] == ["Cal", "7"]
    assert values[6:12] == ["FF", "42.5", "L", "1", "1", "L1"]
    assert np.asarray(values[12:15], dtype=float) == pytest.approx([-10.0, -20.0, 70.0])
    assert np.asarray(values[17:22], dtype=float) == pytest.approx([42.5, 0.01, 0.021, 42.5, 0.011])
    assert values[-1] == "false"


def test_full_case_short_run_validation_writes_python_snapshot_and_report(tmp_path) -> None:
    snapshot_path, report_path = write_full_case_short_run_report(
        tmp_path,
        cut_freq=50.0,
        dt=1.0e-4,
        n_steps_per_stage=1,
    )

    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    report = report_path.read_text(encoding="utf-8")

    assert snapshot["schema"] == "sditt-full-case-short-run-v1"
    assert snapshot["source"] == "python"
    assert snapshot["settings"]["rail_layout"] == "interval"
    assert snapshot["settings"]["network_b_enabled"] is False
    assert snapshot["settings"]["network_b_fallback_active"] is False
    assert snapshot["preparation"]["rail_profile_layout"] == "interval"
    assert snapshot["preparation"]["dynamic_track_model"] == "turnout_ft_modal_surrogate"
    assert [stage["name"] for stage in snapshot["stages"]] == ["Preload", "Cal"]
    assert snapshot["stages"][0]["output_rows"][0]["pjcc"]
    assert "SDITT Full-Case Short-Run Validation" in report
    assert "not directly comparable" in report


def test_full_case_short_run_compact_snapshot_keeps_counts_and_final_rows(tmp_path) -> None:
    snapshot_path, report_path = write_full_case_short_run_report(
        tmp_path,
        cut_freq=50.0,
        dt=1.0e-4,
        n_steps_per_stage=2,
        snapshot_history_limit=1,
    )

    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    report = report_path.read_text(encoding="utf-8")

    for stage in snapshot["stages"]:
        assert stage["accepted_step_count"] == 2
        assert stage["iteration_record_count"] >= 2
        assert stage["snapshot_history_limit"] == 1
        assert len(stage["output_rows"]) == 1
        assert len(stage["convergence_history"]) == 1
        assert len(stage["output_table"]) == 1
    assert "| Preload | 2 |" in report
    assert "| Cal | 2 |" in report


def test_full_case_short_run_validation_compares_matching_snapshots() -> None:
    snapshot = build_python_short_run_snapshot(
        cut_freq=50.0,
        dt=1.0e-4,
        n_steps_per_stage=1,
        rail_layout="turnout",
    )

    metrics = compare_short_run_snapshots(snapshot, snapshot)

    assert metrics
    assert {metric.status for metric in metrics} == {"ok"}


def test_full_case_short_run_validation_accepts_single_matlab_stage_object() -> None:
    snapshot = build_python_short_run_snapshot(
        cut_freq=50.0,
        dt=1.0e-4,
        n_steps_per_stage=1,
        rail_layout="turnout",
    )
    single_stage = {
        **snapshot["stages"][0],
        "output_rows": snapshot["stages"][0]["output_rows"][0],
    }
    matlab_snapshot = {
        **snapshot,
        "stages": single_stage,
    }

    metrics = compare_short_run_snapshots(snapshot, matlab_snapshot)
    report = build_full_case_short_run_report(snapshot, matlab_snapshot=matlab_snapshot)

    assert metrics
    assert {metric.stage for metric in metrics} == {"Preload"}
    assert "baseline stages: `Preload`" in report
    assert "missing baseline stages: `Cal`" in report


def test_full_case_short_run_validation_allows_small_norm_relative_vector_error() -> None:
    python_snapshot = {
        "preparation": {"total_dof": 3, "n_track": 2},
        "settings": {"cut_freq": 50.0, "dt": 1.0e-4, "n_steps_per_stage": 1},
        "stages": [{"name": "Cal", "output_rows": [{"contact_force": [1000.0, 1.0e-3, 2000.0]}]}],
    }
    matlab_snapshot = {
        **python_snapshot,
        "source": "matlab",
        "stages": [{"name": "Cal", "output_rows": [{"contact_force": [1000.0, 1.0e-6, 2000.0]}]}],
    }

    metrics = compare_short_run_snapshots(python_snapshot, matlab_snapshot, rel_tolerance=1.0e-4)
    report = build_full_case_short_run_report(python_snapshot, matlab_snapshot=matlab_snapshot)
    contact_metric = next(metric for metric in metrics if metric.quantity == "final.contact_force")

    assert contact_metric.max_rel_error > 1.0
    assert contact_metric.norm_rel_error < 1.0e-4
    assert contact_metric.status == "ok"
    assert "norm rel error" in report


def test_progress_recorder_writes_only_final_outputs(tmp_path) -> None:
    recorder = _ProgressRecorder()
    recorder(
        FullCaseProgressEvent(
            stage="Preload",
            step_index=1,
            n_steps=2,
            time=1.0e-4,
            dt=1.0e-4,
            front_mileage=32.01,
            iterations=2,
            normal_error=0.1,
            normal_tangential_error=0.2,
            contact_force_norm=1000.0,
            total_force_norm=2000.0,
            max_patch_force_z=300.0,
            patch_force_labels=("FF-L1", "FF-R1", "FR-L1", "FR-R1"),
            patch_force_magnitude=np.asarray([130.0, 250.0, 410.0, 510.0], dtype=float),
            patch_lateral_force_y=np.asarray([50.0, -70.0, 90.0, -100.0], dtype=float),
            patch_vertical_force_z=np.asarray([120.0, -240.0, 400.0, 500.0], dtype=float),
            wheelset_force_labels=("FF", "FR"),
            wheelset_mileage=np.asarray([32.01, 29.51], dtype=float),
            wheel_lateral_force_y=np.asarray([[50.0, -70.0], [90.0, -100.0]], dtype=float),
            wheel_vertical_force_z=np.asarray([[120.0, -240.0], [400.0, 500.0]], dtype=float),
        )
    )
    recorder(
        FullCaseProgressEvent(
            stage="Cal",
            step_index=2,
            n_steps=2,
            time=2.0e-4,
            dt=1.0e-4,
            front_mileage=32.02,
            iterations=3,
            normal_error=0.01,
            normal_tangential_error=0.02,
            contact_force_norm=1100.0,
            total_force_norm=2100.0,
            max_patch_force_z=330.0,
            patch_force_labels=("FF-L1", "FF-R1", "FR-L1", "FR-R1"),
            patch_force_magnitude=np.asarray([160.0, 340.0, 420.0, 520.0], dtype=float),
            patch_lateral_force_y=np.asarray([55.0, -75.0, 95.0, -105.0], dtype=float),
            patch_vertical_force_z=np.asarray([150.0, -330.0, 410.0, 510.0], dtype=float),
            wheelset_force_labels=("FF", "FR"),
            wheelset_mileage=np.asarray([32.02, 29.52], dtype=float),
            wheel_lateral_force_y=np.asarray([[55.0, -75.0], [95.0, -105.0]], dtype=float),
            wheel_vertical_force_z=np.asarray([[150.0, -330.0], [410.0, 510.0]], dtype=float),
        )
    )

    csv_path, svg_path = recorder.write_outputs(tmp_path)

    assert csv_path.name == "progress.csv"
    assert svg_path.name == "progress_final.svg"
    assert csv_path.exists()
    assert svg_path.exists()
    csv = csv_path.read_text(encoding="utf-8")
    assert (
        "patch_FF_L1_wheel_rail_force_magnitude_N,patch_FF_R1_wheel_rail_force_magnitude_N,"
        "patch_FR_L1_wheel_rail_force_magnitude_N,patch_FR_R1_wheel_rail_force_magnitude_N"
    ) in csv
    assert "damping_clip_count,damping_clip_max_delta_N" in csv
    assert "Preload,1,2" in csv
    assert ",0,0,130,250,410,510" in csv
    svg = svg_path.read_text(encoding="utf-8")
    assert "<polyline" in svg
    assert 'font-family="Times New Roman"' in svg
    assert 'shape-rendering="geometricPrecision"' in svg
    assert '<line x1="70" y1=' in svg and 'x2="76"' in svg
    assert '>0.0</text>' in svg
    assert "Mileage (m)" in svg
    assert "Patch force magnitude (kN)" in svg
    assert "FF-L1" in svg
    assert "FF-R1" in svg
    assert "FR-L1" not in svg
    assert "FR-R1" not in svg
    wheel_force_csv = (tmp_path / "wheel_rail_forces.csv").read_text(encoding="utf-8")
    assert "wheelset,side,lateral_force_y_N,vertical_force_z_N,resultant_yz_force_N" in wheel_force_csv
    assert ",FF,L,50,120,130\n" in wheel_force_csv
    assert ",FR,R,-100,500," in wheel_force_csv
    patch_force_csv = (tmp_path / "contact_patch_forces.csv").read_text(encoding="utf-8")
    assert "wheelset,side,dummy_rail,lateral_force_y_N" in patch_force_csv
    assert ",FF,R,R1,-70,-240,250\n" in patch_force_csv
    assert not (tmp_path / "progress_latest.svg").exists()
    assert not list(tmp_path.glob("preload_*.svg"))
    assert not list(tmp_path.glob("cal_*.svg"))


def test_full_case_short_run_validation_parses_live_window_and_save_progress_flags() -> None:
    args = _parse_args(
        [
            "--live-window",
            "--save-progress",
            "--plot-every",
            "25",
            "--preload-cache-dir",
            "cache",
            "--checkpoint-dir",
            "checkpoints",
            "--save-checkpoints",
            "--checkpoint-path",
            "checkpoints/manual.pkl",
            "--resume-checkpoint",
            "--snapshot-history-limit",
            "1",
        ]
    )

    assert args.live_window
    assert args.save_progress
    assert args.plot_every == 25
    assert str(args.preload_cache_dir) == "cache"
    assert str(args.checkpoint_dir) == "checkpoints"
    assert args.save_checkpoints
    assert str(args.checkpoint_path) == "checkpoints/manual.pkl"
    assert args.resume_checkpoint is True
    assert args.snapshot_history_limit == 1


def test_full_case_short_run_validation_defaults_to_not_saving_checkpoints() -> None:
    args = _parse_args([])

    assert not args.save_checkpoints
    assert args.resume_checkpoint is None
    assert args.checkpoint_path is None
    assert args.rail_layout == "interval"
    assert args.track_irregularity == "none"
    assert args.irregularity_seed == 20260716
    assert args.contact_geometry_mode == "traditional"
    assert args.network_a_trace_dir is None
    assert args.network_a_trace_mode == "selective"
    assert args.network_a_trace_low_confidence == pytest.approx(0.95)
    assert args.network_a_trace_sample_interval_m == pytest.approx(1.0)
    assert args.network_a_force_mode == "traditional"
    assert not args.disable_network_b
    assert not args.save_contact_key_data
    assert not args.contact_key_data_only
    assert args.snapshot_history_limit is None


def test_full_case_short_run_parses_explicit_network_b_disable_and_key_data_output() -> None:
    args = _parse_args(
        [
            "--disable-network-b",
            "--save-contact-key-data",
            "--contact-key-data-only",
            "--live-window",
        ]
    )

    assert args.disable_network_b
    assert args.network_a_force_mode == "traditional"
    assert args.save_contact_key_data
    assert args.contact_key_data_only
    assert args.live_window


def test_main_routes_live_key_data_only_mode_to_realtime_window(monkeypatch) -> None:
    called: dict[str, object] = {}

    def fake_live_window(args: object, *, cut_freq: float | None) -> int:
        called["args"] = args
        called["cut_freq"] = cut_freq
        return 0

    monkeypatch.setattr(full_case_short_run_module, "_run_with_live_window", fake_live_window)

    result = full_case_short_run_module.main(
        ["--disable-network-b", "--contact-key-data-only", "--live-window"]
    )

    assert result == 0
    assert getattr(called["args"], "contact_key_data_only")
    assert getattr(called["args"], "disable_network_b")
    assert called["cut_freq"] == pytest.approx(50.0)


def test_contact_key_data_only_mode_rejects_network_b_before_running(tmp_path) -> None:
    with pytest.raises(ValueError, match="does not permit Network B"):
        run_full_case_contact_key_data(
            tmp_path,
            contact_geometry_mode="network-a-after-preload",
            network_a_force_mode="network-b",
        )


def test_full_case_short_run_parses_network_a_after_preload() -> None:
    args = _parse_args(
        [
            "--contact-geometry-mode",
            "network-a-after-preload",
            "--network-a-model",
            "outputs/custom-a2g/model.npz",
            "--network-a-force-mode",
            "hertz",
            "--network-a-trace-dir",
            "outputs/custom-a2g/trace",
            "--network-a-trace-mode",
            "full",
            "--network-a-trace-low-confidence",
            "0.9",
            "--network-a-trace-sample-interval-m",
            "2.5",
        ]
    )

    assert args.contact_geometry_mode == "network-a-after-preload"
    assert args.network_a_model == Path("outputs/custom-a2g/model.npz")
    assert args.network_a_force_mode == "hertz"
    assert args.network_a_trace_dir == Path("outputs/custom-a2g/trace")
    assert args.network_a_trace_mode == "full"
    assert args.network_a_trace_low_confidence == pytest.approx(0.9)
    assert args.network_a_trace_sample_interval_m == pytest.approx(2.5)


def test_full_case_short_run_validation_parses_turnout_layout() -> None:
    args = _parse_args(
        [
            "--rail-layout",
            "turnout",
            "--track-irregularity",
            "china-ballastless",
            "--irregularity-seed",
            "99",
        ]
    )

    assert args.rail_layout == "turnout"
    assert args.track_irregularity == "china-ballastless"
    assert args.irregularity_seed == 99


def test_full_case_short_run_writes_track_irregularity_outputs(tmp_path) -> None:
    snapshot_path, report_path = write_full_case_short_run_report(
        tmp_path,
        cut_freq=50.0,
        n_steps_per_stage=1,
        save_progress=True,
        track_irregularity="china-ballastless",
        irregularity_seed=44,
    )

    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    assert snapshot["settings"]["track_irregularity"]["model"] == "china-ballastless"
    assert snapshot["settings"]["track_irregularity"]["seed"] == 44
    assert snapshot["settings"]["track_irregularity"]["theoretical_rms_mm"]["vertical"] == pytest.approx(1.2203026)
    assert (tmp_path / "progress" / "track_irregularity.csv").exists()
    assert (tmp_path / "progress" / "track_irregularity_spectrum.csv").exists()
    assert (tmp_path / "progress" / "track_irregularity.svg").exists()
    wheel_force_lines = (tmp_path / "progress" / "wheel_rail_forces.csv").read_text(
        encoding="utf-8"
    ).splitlines()
    patch_force_lines = (tmp_path / "progress" / "contact_patch_forces.csv").read_text(
        encoding="utf-8"
    ).splitlines()
    assert len(wheel_force_lines) == 1 + 2 * 4 * 2
    assert len(patch_force_lines) == 1 + 2 * 4 * 2
    assert {line.split(",")[9] for line in wheel_force_lines[1:]} == {"FF", "FR", "RF", "RR"}
    assert {line.split(",")[11] for line in patch_force_lines[1:]} == {"L1", "R1"}
    assert "not directly comparable" in report_path.read_text(encoding="utf-8")


def test_format_elapsed_time_uses_stable_clock_format() -> None:
    assert _format_elapsed_time(-1.0) == "00:00:00"
    assert _format_elapsed_time(9.9) == "00:00:09"
    assert _format_elapsed_time(65.0) == "00:01:05"
    assert _format_elapsed_time(3661.0) == "01:01:01"


def test_format_timing_top_ranks_slowest_items() -> None:
    assert _format_timing_top({"contact_force": 1.25, "integrate": 3.5, "geometry": 2.0}, limit=2) == (
        "integrate 3.5s | geometry 2.0s"
    )


def test_step_wall_times_uses_progress_event_values() -> None:
    events = (
        FullCaseProgressEvent(
            stage="Cal",
            step_index=1,
            n_steps=2,
            time=1.0e-4,
            dt=1.0e-4,
            front_mileage=48.0,
            iterations=2,
            normal_error=0.0,
            normal_tangential_error=0.0,
            contact_force_norm=1.0,
            total_force_norm=2.0,
            max_patch_force_z=3.0,
            step_wall_time=1.25,
        ),
        FullCaseProgressEvent(
            stage="Cal",
            step_index=2,
            n_steps=2,
            time=2.0e-4,
            dt=1.0e-4,
            front_mileage=48.1,
            iterations=3,
            normal_error=0.0,
            normal_tangential_error=0.0,
            contact_force_norm=1.0,
            total_force_norm=2.0,
            max_patch_force_z=3.0,
            step_wall_time=0.5,
        ),
    )

    assert np.allclose(_step_wall_times(events), [1.25, 0.5])


def test_sample_progress_events_keeps_latest_event() -> None:
    events = tuple(
        FullCaseProgressEvent(
            stage="Cal",
            step_index=index,
            n_steps=2505,
            time=float(index),
            dt=1.0e-4,
            front_mileage=float(index),
            iterations=1,
            normal_error=0.0,
            normal_tangential_error=0.0,
            contact_force_norm=1.0,
            total_force_norm=2.0,
            max_patch_force_z=3.0,
        )
        for index in range(2505)
    )

    sampled = _sample_progress_events(events, max_points=2000)

    assert len(sampled) <= 2000
    assert sampled[0] is events[0]
    assert sampled[-1] is events[-1]
    assert [event.step_index for event in sampled] == sorted(event.step_index for event in sampled)
