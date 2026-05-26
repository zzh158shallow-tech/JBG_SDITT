from __future__ import annotations

import json

import numpy as np

from sditt.simulation import FullCaseProgressEvent
from sditt.validation.full_case_short_run import (
    _ProgressRecorder,
    _format_elapsed_time,
    _format_timing_top,
    _parse_args,
    _step_wall_times,
    build_full_case_short_run_report,
    build_python_short_run_snapshot,
    compare_short_run_snapshots,
    write_full_case_short_run_report,
)


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
    assert [stage["name"] for stage in snapshot["stages"]] == ["Preload", "Cal"]
    assert snapshot["stages"][0]["output_rows"][0]["pjcc"]
    assert "SDITT Full-Case Short-Run Validation" in report
    assert "No MATLAB baseline was supplied" in report


def test_full_case_short_run_validation_compares_matching_snapshots() -> None:
    snapshot = build_python_short_run_snapshot(cut_freq=50.0, dt=1.0e-4, n_steps_per_stage=1)

    metrics = compare_short_run_snapshots(snapshot, snapshot)

    assert metrics
    assert {metric.status for metric in metrics} == {"ok"}


def test_full_case_short_run_validation_accepts_single_matlab_stage_object() -> None:
    snapshot = build_python_short_run_snapshot(cut_freq=50.0, dt=1.0e-4, n_steps_per_stage=1)
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
            patch_vertical_force_z=np.asarray([120.0, -240.0, 400.0, 500.0], dtype=float),
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
            patch_vertical_force_z=np.asarray([150.0, -330.0, 410.0, 510.0], dtype=float),
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
    assert "Preload,1,2" in csv
    assert ",130,250,410,510" in csv
    svg = svg_path.read_text(encoding="utf-8")
    assert "<polyline" in svg
    assert "Mileage (m)" in svg
    assert "Patch force magnitude (kN)" in svg
    assert "FF-L1" in svg
    assert "FF-R1" in svg
    assert "FR-L1" not in svg
    assert "FR-R1" not in svg
    assert not (tmp_path / "progress_latest.svg").exists()
    assert not list(tmp_path.glob("preload_*.svg"))
    assert not list(tmp_path.glob("cal_*.svg"))


def test_full_case_short_run_validation_parses_live_window_and_save_progress_flags() -> None:
    args = _parse_args(["--live-window", "--save-progress", "--plot-every", "25", "--preload-cache-dir", "cache"])

    assert args.live_window
    assert args.save_progress
    assert args.plot_every == 25
    assert str(args.preload_cache_dir) == "cache"


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
