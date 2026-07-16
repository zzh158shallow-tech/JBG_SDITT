from __future__ import annotations

import argparse
import json
import queue
import threading
import time
import traceback
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from sditt.config import DefaultOperatingCase, RailLayout
from sditt.simulation import (
    FullCaseProgressEvent,
    FullCaseProfileSnapshot,
    FullCaseSideProfileSnapshot,
    FullDefaultCaseRunResult,
    FullDefaultCaseSettings,
    list_default_full_case_checkpoints,
    load_default_full_case_checkpoint_summary,
    run_default_full_case_driver,
)


DEFAULT_OUTPUT_DIR = Path("python/outputs/full_case_short_run")
DEFAULT_ABS_TOLERANCE = 1.0e-6
DEFAULT_REL_TOLERANCE = 1.0e-4
_PATCH_FORCE_COLORS = (
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
    "#4c78a8",
    "#f58518",
    "#54a24b",
    "#e45756",
    "#72b7b2",
    "#b279a2",
)
_PLOT_FONT_FAMILY = "Times New Roman"
_LIVE_REFRESH_MS = 1000


@dataclass(frozen=True)
class ComparisonMetric:
    stage: str
    quantity: str
    shape: tuple[int, ...]
    max_abs_error: float
    max_rel_error: float
    norm_rel_error: float
    python_norm: float
    baseline_norm: float
    status: str


def build_python_short_run_snapshot(
    *,
    cut_freq: float | None = 50.0,
    dt: float = 1.0e-4,
    n_steps_per_stage: int = 1,
    use_sparse: bool = True,
    use_matlab_mileage_endpoints: bool = False,
    preload_cache_dir: str | Path | None = None,
    history_retention_steps: int | None = 256,
    checkpoint_dir: str | Path | None = None,
    save_checkpoints: bool = False,
    resume_checkpoint: bool = False,
    resume_checkpoint_path: str | Path | None = None,
    progress_callback: Any = None,
    rail_layout: RailLayout = "interval",
) -> dict[str, Any]:
    """Run the Python full-case driver and serialize key validation quantities."""

    settings = FullDefaultCaseSettings(
        cut_freq=cut_freq,
        dt=dt,
        n_steps_per_stage=n_steps_per_stage,
        use_matlab_mileage_endpoints=use_matlab_mileage_endpoints,
        use_sparse=use_sparse,
        preload_cache_dir=preload_cache_dir,
        history_retention_steps=history_retention_steps,
        checkpoint_dir=checkpoint_dir,
        save_checkpoints=save_checkpoints,
        resume_checkpoint=resume_checkpoint,
        resume_checkpoint_path=resume_checkpoint_path,
        progress_callback=progress_callback,
    )
    result = run_default_full_case_driver(
        settings=settings,
        operating_case=DefaultOperatingCase(rail_layout=rail_layout),
    )
    return snapshot_from_run_result(result)


def snapshot_from_run_result(result: FullDefaultCaseRunResult) -> dict[str, Any]:
    preparation = result.preparation
    return {
        "schema": "sditt-full-case-short-run-v1",
        "source": "python",
        "settings": {
            "rail_layout": preparation.operating_case.rail_layout,
            "cut_freq": preparation.settings.cut_freq,
            "dt": preparation.settings.dt,
            "n_steps_per_stage": preparation.settings.n_steps_per_stage,
            "use_matlab_mileage_endpoints": preparation.settings.use_matlab_mileage_endpoints,
            "use_sparse": preparation.settings.use_sparse,
            "history_retention_steps": preparation.settings.history_retention_steps,
            "checkpoint_dir": None
            if preparation.settings.checkpoint_dir is None
            else str(preparation.settings.checkpoint_dir),
            "save_checkpoints": preparation.settings.save_checkpoints,
            "resume_checkpoint": preparation.settings.resume_checkpoint,
            "resume_checkpoint_path": None
            if preparation.settings.resume_checkpoint_path is None
            else str(preparation.settings.resume_checkpoint_path),
            "preload_cache_dir": None
            if preparation.settings.preload_cache_dir is None
            else str(preparation.settings.preload_cache_dir),
        },
        "preload_cache": {
            "status": result.preload_cache_status,
            "path": None if result.preload_cache_path is None else str(result.preload_cache_path),
        },
        "checkpoint": {
            "status": result.checkpoint_status,
            "path": None if result.checkpoint_path is None else str(result.checkpoint_path),
            "resumed": result.resumed_from_checkpoint,
            "mileage": result.resumed_checkpoint_mileage,
        },
        "preparation": {
            "rail_profile_layout": preparation.operating_case.rail_layout,
            "dynamic_track_model": (
                "turnout_ft_modal_surrogate"
                if preparation.operating_case.rail_layout == "interval"
                else "turnout_ft_modal"
            ),
            "total_dof": preparation.total_dof,
            "n_track": preparation.system.layout.n_track,
            "wheelsets": list(preparation.operating_case.wheelsets),
            "missing_stages": [stage.name for stage in preparation.missing_stages],
        },
        "stages": [_stage_snapshot(stage) for stage in result.stages],
        "timing": _timing_snapshot(result),
    }


def write_full_case_short_run_report(
    output_dir: str | Path = DEFAULT_OUTPUT_DIR,
    *,
    cut_freq: float | None = 50.0,
    dt: float = 1.0e-4,
    n_steps_per_stage: int = 1,
    use_sparse: bool = True,
    use_matlab_mileage_endpoints: bool = False,
    plot_progress: bool = False,
    plot_every: int = 1,
    save_progress: bool = False,
    preload_cache_dir: str | Path | None = None,
    history_retention_steps: int | None = 256,
    checkpoint_dir: str | Path | None = None,
    save_checkpoints: bool = False,
    resume_checkpoint: bool = False,
    resume_checkpoint_path: str | Path | None = None,
    progress_recorder: "_ProgressRecorder | None" = None,
    matlab_baseline_path: str | Path | None = None,
    abs_tolerance: float = DEFAULT_ABS_TOLERANCE,
    rel_tolerance: float = DEFAULT_REL_TOLERANCE,
    rail_layout: RailLayout = "interval",
) -> tuple[Path, Path]:
    """Write a Python snapshot and Markdown error report for a short full-case run."""

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    progress_writer = progress_recorder or (
        _ProgressRecorder(every=plot_every) if plot_progress or save_progress else None
    )
    python_snapshot = build_python_short_run_snapshot(
        cut_freq=cut_freq,
        dt=dt,
        n_steps_per_stage=n_steps_per_stage,
        use_sparse=use_sparse,
        use_matlab_mileage_endpoints=use_matlab_mileage_endpoints,
        preload_cache_dir=preload_cache_dir,
        history_retention_steps=history_retention_steps,
        checkpoint_dir=checkpoint_dir,
        save_checkpoints=save_checkpoints,
        resume_checkpoint=resume_checkpoint,
        resume_checkpoint_path=resume_checkpoint_path,
        progress_callback=progress_writer,
        rail_layout=rail_layout,
    )
    snapshot_path = output_path / "python_snapshot.json"
    snapshot_path.write_text(_json_dumps(python_snapshot), encoding="utf-8")

    matlab_snapshot = None
    if matlab_baseline_path is not None:
        matlab_snapshot = _read_json(matlab_baseline_path)

    markdown = build_full_case_short_run_report(
        python_snapshot,
        matlab_snapshot=matlab_snapshot,
        abs_tolerance=abs_tolerance,
        rel_tolerance=rel_tolerance,
    )
    report_path = output_path / "report.md"
    report_path.write_text(markdown, encoding="utf-8")
    if progress_writer is not None and (plot_progress or save_progress):
        progress_writer.write_outputs(output_path / "progress")
    write_timing_summary(python_snapshot, output_path / "timing")
    return snapshot_path, report_path


def build_full_case_short_run_report(
    python_snapshot: dict[str, Any],
    *,
    matlab_snapshot: dict[str, Any] | None = None,
    abs_tolerance: float = DEFAULT_ABS_TOLERANCE,
    rel_tolerance: float = DEFAULT_REL_TOLERANCE,
) -> str:
    lines = [
        "# SDITT Full-Case Short-Run Validation",
        "",
        "## Python Run",
        "",
        f"- total_dof: `{python_snapshot['preparation']['total_dof']}`",
        f"- n_track: `{python_snapshot['preparation']['n_track']}`",
        f"- rail_profile_layout: `{python_snapshot['preparation'].get('rail_profile_layout', 'turnout')}`",
        f"- dynamic_track_model: `{python_snapshot['preparation'].get('dynamic_track_model', 'turnout_ft_modal')}`",
        f"- cut_freq: `{python_snapshot['settings']['cut_freq']}`",
        f"- dt: `{python_snapshot['settings']['dt']}`",
        f"- n_steps_per_stage: `{python_snapshot['settings']['n_steps_per_stage']}`",
        f"- use_matlab_mileage_endpoints: `{python_snapshot['settings'].get('use_matlab_mileage_endpoints', False)}`",
        "",
        "## Stage Summary",
        "",
        "| stage | output rows | final mileage | final iterations | final normal error | final normal+tangent error |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for stage in python_snapshot["stages"]:
        rows = stage["output_rows"]
        final = rows[-1] if rows else {}
        lines.append(
            "| {stage} | {row_count} | {mileage:.12g} | {iterations} | {normal:.6g} | {normal_tan:.6g} |".format(
                stage=stage["name"],
                row_count=len(rows),
                mileage=float(final.get("front_mileage", 0.0)),
                iterations=int(final.get("iterations", 0)),
                normal=float(final.get("normal_error", 0.0)),
                normal_tan=float(final.get("normal_tangential_error", 0.0)),
            )
        )

    rail_layout = python_snapshot.get("preparation", {}).get("rail_profile_layout", "turnout")
    if rail_layout != "turnout":
        lines.extend(
            [
                "",
                "## MATLAB Comparison",
                "",
                "Skipped: interval basic-rail contact geometry uses the turnout FT-modal dynamic model as a surrogate, "
                "so it is not directly comparable with the existing MATLAB turnout baseline.",
            ]
        )
        return "\n".join(lines) + "\n"

    if matlab_snapshot is None:
        lines.extend(
            [
                "",
                "## MATLAB Comparison",
                "",
                "No MATLAB baseline was supplied. Generate one with `python/matlab/export_full_case_short_run_baseline.m` "
                "from a MATLAB output `.mat`, then rerun this report with `--matlab-baseline <baseline.json>`.",
            ]
        )
        return "\n".join(lines) + "\n"

    metrics = compare_short_run_snapshots(
        python_snapshot,
        matlab_snapshot,
        abs_tolerance=abs_tolerance,
        rel_tolerance=rel_tolerance,
    )
    python_stage_names = _stage_names(python_snapshot)
    matlab_stage_names = _stage_names(matlab_snapshot)
    missing_matlab_stages = tuple(name for name in python_stage_names if name not in matlab_stage_names)
    lines.extend(
        [
            "",
            "## MATLAB Comparison",
            "",
            f"- baseline source: `{matlab_snapshot.get('source', 'matlab')}`",
            f"- baseline stages: `{', '.join(matlab_stage_names) or 'none'}`",
            f"- missing baseline stages: `{', '.join(missing_matlab_stages) or 'none'}`",
            f"- abs_tolerance: `{abs_tolerance}`",
            f"- rel_tolerance: `{rel_tolerance}`",
            "",
            "| stage | quantity | shape | max abs error | max rel error | norm rel error | status |",
            "| --- | --- | --- | ---: | ---: | ---: | --- |",
        ]
    )
    for metric in metrics:
        lines.append(
            "| {stage} | `{quantity}` | `{shape}` | {abs_err:.6g} | {rel_err:.6g} | {norm_rel:.6g} | {status} |".format(
                stage=metric.stage,
                quantity=metric.quantity,
                shape="x".join(str(part) for part in metric.shape) or "scalar",
                abs_err=metric.max_abs_error,
                rel_err=metric.max_rel_error,
                norm_rel=metric.norm_rel_error,
                status=metric.status,
            )
        )
    return "\n".join(lines) + "\n"


def compare_short_run_snapshots(
    python_snapshot: dict[str, Any],
    matlab_snapshot: dict[str, Any],
    *,
    abs_tolerance: float = DEFAULT_ABS_TOLERANCE,
    rel_tolerance: float = DEFAULT_REL_TOLERANCE,
) -> tuple[ComparisonMetric, ...]:
    matlab_by_stage = {stage["name"]: stage for stage in _snapshot_stages(matlab_snapshot)}
    metrics: list[ComparisonMetric] = []
    for python_stage in _snapshot_stages(python_snapshot):
        stage_name = python_stage["name"]
        matlab_stage = matlab_by_stage.get(stage_name)
        if matlab_stage is None:
            continue
        for quantity in _comparison_quantities():
            python_value = _stage_quantity(python_stage, quantity)
            matlab_value = _stage_quantity(matlab_stage, quantity)
            if python_value is None or matlab_value is None:
                continue
            metric = _compare_quantity(
                stage=stage_name,
                quantity=quantity,
                python_value=python_value,
                baseline_value=matlab_value,
                abs_tolerance=abs_tolerance,
                rel_tolerance=rel_tolerance,
            )
            metrics.append(metric)
    return tuple(metrics)


def _snapshot_stages(snapshot: dict[str, Any]) -> tuple[dict[str, Any], ...]:
    stages = snapshot.get("stages", [])
    if isinstance(stages, dict):
        return (stages,)
    return tuple(stage for stage in stages if isinstance(stage, dict))


def _stage_names(snapshot: dict[str, Any]) -> tuple[str, ...]:
    return tuple(str(stage.get("name", "")) for stage in _snapshot_stages(snapshot) if stage.get("name"))


def _stage_snapshot(stage: Any) -> dict[str, Any]:
    return {
        "name": stage.stage,
        "convergence_history": _array_payload(stage.convergence_history),
        "iteration_diagnostics": [_iteration_snapshot(record) for record in stage.iteration_records],
        "output_table": _array_payload(stage.output_table),
        "output_rows": [_output_row_snapshot(row) for row in stage.output_rows],
        "timing": {str(key): float(value) for key, value in getattr(stage.history, "timing", {}).items()},
    }


def _timing_snapshot(result: FullDefaultCaseRunResult) -> dict[str, Any]:
    stages = []
    totals: dict[str, float] = {}
    for stage in result.stages:
        timing = {str(key): float(value) for key, value in getattr(stage.history, "timing", {}).items()}
        for key, value in timing.items():
            totals[key] = totals.get(key, 0.0) + value
        stages.append({"name": stage.stage, "timing": timing})
    return {"stages": stages, "total": totals}


def write_timing_summary(snapshot: dict[str, Any], output_dir: str | Path) -> tuple[Path, Path]:
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    rows: list[tuple[str, str, float]] = []
    timing = snapshot.get("timing", {})
    for stage in timing.get("stages", []):
        stage_name = str(stage.get("name", ""))
        for name, seconds in stage.get("timing", {}).items():
            rows.append((stage_name, str(name), float(seconds)))
    total = {str(name): float(seconds) for name, seconds in timing.get("total", {}).items()}
    for name, seconds in total.items():
        rows.append(("TOTAL", name, seconds))

    csv_path = output_path / "timing_summary.csv"
    with csv_path.open("w", encoding="utf-8") as handle:
        handle.write("stage,item,seconds\n")
        for stage, name, seconds in rows:
            handle.write(f"{stage},{name},{seconds:.9f}\n")

    md_path = output_path / "timing_summary.md"
    ranked = sorted(total.items(), key=lambda item: item[1], reverse=True)
    lines = [
        "# SDITT Timing Summary",
        "",
        f"- preload cache: `{snapshot.get('preload_cache', {}).get('status', 'unknown')}`",
        f"- checkpoint: `{snapshot.get('checkpoint', {}).get('status', 'unknown')}`",
        "",
        "| rank | item | seconds |",
        "| ---: | --- | ---: |",
    ]
    for index, (name, seconds) in enumerate(ranked, start=1):
        lines.append(f"| {index} | `{name}` | {seconds:.3f} |")
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return csv_path, md_path


def _iteration_snapshot(record: Any) -> dict[str, Any]:
    return {
        "step_index": int(record.step_index),
        "iteration": int(record.iteration),
        "time": float(record.time),
        "dt": float(record.dt),
        "front_mileage": float(record.front_mileage),
        "normal_error": float(record.normal_error),
        "normal_tangential_error": float(record.normal_tangential_error),
        "force_guess": _array_payload(record.force_guess),
        "contact_force": _array_payload(record.contact_force),
        "gravity_plus_damper_force": _array_payload(
            record.total_force - record.contact_force + record.damper_equivalent_force
        ),
        "damper_equivalent_force": _array_payload(record.damper_equivalent_force),
        "total_force": _array_payload(record.total_force),
        "modal_displacement": _array_payload(record.rail_response.modal_displacement),
        "modal_velocity": _array_payload(record.rail_response.modal_velocity),
        "modal_acceleration": _array_payload(record.rail_response.modal_acceleration),
    }


def _output_row_snapshot(row: Any) -> dict[str, Any]:
    contact_state = row.contact_state
    rail = row.rail_response
    return {
        "step_index": int(row.step_index),
        "iterations": int(row.iterations),
        "time": float(row.time),
        "dt": float(row.dt),
        "front_mileage": float(row.front_mileage),
        "normal_error": float(row.normal_error),
        "normal_tangential_error": float(row.normal_tangential_error),
        "d0_by_wheelset": dict(contact_state.d0_by_wheelset) if contact_state is not None else {},
        "relvel_max_by_wheelset": contact_state.relvel_max_by_wheelset if contact_state is not None else {},
        "contact_diagnostics": _contact_diagnostics(contact_state.con_ws if contact_state is not None else None),
        "pjc": _array_payload(row.pjc),
        "pjch": _array_payload(row.pjch),
        "pjcc": _array_payload(row.pjcc),
        "prhx": _array_payload(row.prhx),
        "prhxf": _array_payload(row.prhxf),
        "patch_force_y": _array_payload(row.patch_force_y),
        "patch_force_z": _array_payload(row.patch_force_z),
        "wheelset_lateral_force": _array_payload(row.wheelset_lateral_force),
        "wheelset_vertical_force": _array_payload(row.wheelset_vertical_force),
        "dis_rail": _array_payload(rail.dis_rail),
        "vel_rail": _array_payload(rail.vel_rail),
        "acc_rail": _array_payload(rail.acc_rail),
        "rail_diagnostics": _rail_diagnostics(rail),
        "vehicle_displacement": _array_payload(row.vehicle_displacement),
        "vehicle_velocity": _array_payload(row.vehicle_velocity),
        "vehicle_acceleration": _array_payload(row.vehicle_acceleration),
        "gravity_force": _array_payload(row.total_force - row.contact_force),
        "contact_force": _array_payload(row.contact_force),
        "total_force": _array_payload(row.total_force),
        "norms": {
            "gravity_force": _norm(row.total_force - row.contact_force),
            "contact_force": _norm(row.contact_force),
            "total_force": _norm(row.total_force),
            "vehicle_displacement": _norm(row.vehicle_displacement),
            "rail_displacement": _norm(rail.dis_rail),
            "pjcc": _norm(row.pjcc),
            "prhxf": _norm(row.prhxf),
        },
    }


def _contact_diagnostics(con_ws: Any) -> dict[str, Any]:
    if not isinstance(con_ws, dict):
        return {}
    diagnostics: dict[str, Any] = {}
    for wheelset, value in con_ws.items():
        if not isinstance(value, dict):
            continue
        if not isinstance(value.get("Normal_Force"), dict):
            continue
        diagnostics[wheelset] = {}
        for side in ("L", "R"):
            diagnostics[wheelset][side] = {
                "normal_force": _array_payload(value.get("Normal_Force", {}).get(side, [])),
                "profile_r": _array_payload(value.get("profile_r", {}).get(side, [])),
                "con_wheel_2": _array_payload(value.get("Con_wheel_2", {}).get(side, [])),
                "con_wheel_2_full": _array_payload(value.get("Con_wheel_2_full", {}).get(side, [])),
                "con_rail_1": _array_payload(value.get("Con_rail_1", {}).get(side, [])),
                "con_wheel_2_a": _array_payload(value.get("Con_wheel_2_a", {}).get(side, [])),
                "con_rail_1_a": _array_payload(value.get("Con_rail_1_a", {}).get(side, [])),
                "ver_pen_a": _array_payload(value.get("Ver_Pen_a", {}).get(side, [])),
                "elastic_normal_force": _array_payload(value.get("Elastic_Normal_Force", {}).get(side, [])),
                "area_stripes": _array_payload(value.get("Area_STRIPES", {}).get(side, [])),
                "epsilon": _array_payload(value.get("Epsilon", {}).get(side, [])),
                "prh": _array_payload(value.get("Prh", {}).get(side, [])),
                "prhx_t": _array_payload(value.get("Prhx_T", {}).get(side, [])),
                "prhxf_t": _array_payload(value.get("Prhxf_T", {}).get(side, [])),
                "rhxs": _array_payload(value.get("RHXS", {}).get(side, [])),
                "rhlv": _array_payload(value.get("RHLv", {}).get(side, [])),
                "a2": _array_payload(value.get("a2", {}).get(side, [])),
                "b2": _array_payload(value.get("b2", {}).get(side, [])),
                "vjd": _array_payload(value.get("Vjd", {}).get(side, [])),
                "vjd_r": _array_payload(value.get("Vjd_r", {}).get(side, [])),
                "vsdc": _array_payload(value.get("Vsdc", {}).get(side, [])),
                "vjsdc": _array_payload(value.get("Vjsdc", {}).get(side, [])),
                "vgd": _array_payload(value.get("Vgd", {}).get(side, [])),
                "stripes": _stripe_patch_payload(value.get("Con_STRIPES", {}).get(side, ())),
            }
    return diagnostics


def _rail_diagnostics(rail: Any) -> dict[str, Any]:
    if rail is None:
        return {}
    shape_function = getattr(rail, "shape_function", {})
    dyn_status_rail = getattr(rail, "dyn_status_rail", {})
    rail_beam_motion = getattr(rail, "rail_beam_motion", {})
    if not isinstance(shape_function, dict):
        return {}

    diagnostics: dict[str, Any] = {
        "modal_displacement": _array_payload(getattr(rail, "modal_displacement", [])),
        "modal_velocity": _array_payload(getattr(rail, "modal_velocity", [])),
        "modal_acceleration": _array_payload(getattr(rail, "modal_acceleration", [])),
        "rail_beam_motion": _nested_array_payload(rail_beam_motion),
        "shape_entries": {},
    }
    prefixes = sorted(
        key.removesuffix("_Y")
        for key in shape_function
        if isinstance(key, str) and key.endswith("_Y") and "_Mapping_DynStatus_" not in key
    )
    for prefix in prefixes:
        rail_name = prefix.split("_", 1)[1] if "_" in prefix else prefix
        dyn_vel = _dyn_track_vector(dyn_status_rail.get(f"{rail_name}_Vel"))
        entry: dict[str, Any] = {}
        for component in ("Y", "Z", "ROTY", "ROTZ"):
            shape = np.asarray(shape_function.get(f"{prefix}_{component}", []), dtype=float).reshape(-1)
            mapping_matlab = np.asarray(
                shape_function.get(f"{prefix}_Mapping_DynStatus_{component}", []),
                dtype=int,
            ).reshape(-1)
            mapping_python = mapping_matlab - 1 if mapping_matlab.size and np.min(mapping_matlab) >= 1 else mapping_matlab
            values = (
                dyn_vel[mapping_python]
                if dyn_vel.size and mapping_python.size and np.max(mapping_python, initial=-1) < dyn_vel.size
                else np.zeros((0,), dtype=float)
            )
            entry[component.lower()] = {
                "shape": _array_payload(shape),
                "mapping": _array_payload(mapping_matlab),
                "dyn_velocity": _array_payload(values),
                "velocity": float(shape @ values) if shape.size and values.size == shape.size else None,
            }
        diagnostics["shape_entries"][prefix] = entry
    return diagnostics


def _dyn_track_vector(status: Any) -> np.ndarray:
    array = np.asarray(status, dtype=float)
    if array.size == 0:
        return np.zeros((0,), dtype=float)
    array = array.reshape((-1, 6))
    return array[:, [1, 2, 4, 5]].reshape(-1)


def _stripe_patch_payload(patches: Any) -> list[dict[str, Any]]:
    payload: list[dict[str, Any]] = []
    for patch in tuple(patches or ()):
        stripes = getattr(patch, "stripes", np.zeros((0, 3), dtype=float))
        curvature = getattr(patch, "curvature", np.zeros((0, 10), dtype=float))
        payload.append(
            {
                "normal_force": float(getattr(patch, "normal_force", 0.0)),
                "area": float(getattr(patch, "area", 0.0)),
                "wheel_points": _array_payload(getattr(patch, "wheel_points", np.zeros((0, 3), dtype=float))),
                "rail_points": _array_payload(getattr(patch, "rail_points", np.zeros((0, 2), dtype=float))),
                "nor_gap_6": _array_payload(getattr(patch, "penetration_raw", np.zeros((0, 2), dtype=float))),
                "nor_gap_8": _array_payload(getattr(patch, "penetration_overlap", np.zeros((0, 2), dtype=float))),
                "nor_gap_11": _array_payload(getattr(patch, "penetration_window", np.zeros((0, 2), dtype=float))),
                "stripe_y": _array_payload(getattr(patch, "stripe_y", np.zeros((0,), dtype=float))),
                "stripe_penetration": _array_payload(
                    getattr(patch, "stripe_penetration", np.zeros((0,), dtype=float))
                ),
                "stripe_dy": _array_payload(getattr(patch, "stripe_dy", np.zeros((0,), dtype=float))),
                "stripes": _array_payload(stripes),
                "curvature": _array_payload(curvature),
            }
        )
    return payload


def _comparison_quantities() -> tuple[str, ...]:
    return (
        "convergence_history",
        "output_table",
        "final.pjc",
        "final.pjch",
        "final.pjcc",
        "final.prhxf",
        "final.dis_rail",
        "final.vel_rail",
        "final.acc_rail",
        "final.vehicle_displacement",
        "final.vehicle_velocity",
        "final.vehicle_acceleration",
        "final.gravity_force",
        "final.contact_force",
        "final.total_force",
        "final.patch_force_y",
        "final.patch_force_z",
        "final.d0",
    )


def _stage_quantity(stage: dict[str, Any], quantity: str) -> np.ndarray | None:
    if quantity in {"convergence_history", "output_table"}:
        return _as_array(stage.get(quantity))
    if not quantity.startswith("final."):
        return None
    rows = _stage_output_rows(stage)
    if not rows:
        return None
    final = rows[-1]
    key = quantity.removeprefix("final.")
    if key == "d0":
        values = final.get("d0_by_wheelset", {})
        return np.asarray([values[name] for name in sorted(values)], dtype=float)
    return _as_array(final.get(key))


def _stage_output_rows(stage: dict[str, Any]) -> tuple[dict[str, Any], ...]:
    rows = stage.get("output_rows", [])
    if isinstance(rows, dict):
        return (rows,)
    return tuple(row for row in rows if isinstance(row, dict))


def _compare_quantity(
    *,
    stage: str,
    quantity: str,
    python_value: Any,
    baseline_value: Any,
    abs_tolerance: float,
    rel_tolerance: float,
) -> ComparisonMetric:
    python_array = np.asarray(python_value, dtype=float)
    baseline_array = np.asarray(baseline_value, dtype=float)
    python_array, baseline_array = _align_vector_shape(python_array, baseline_array)
    if python_array.shape != baseline_array.shape:
        return ComparisonMetric(
            stage=stage,
            quantity=quantity,
            shape=tuple(int(part) for part in python_array.shape),
            max_abs_error=float("inf"),
            max_rel_error=float("inf"),
            norm_rel_error=float("inf"),
            python_norm=_norm(python_array),
            baseline_norm=_norm(baseline_array),
            status=f"shape mismatch: python {python_array.shape}, baseline {baseline_array.shape}",
        )
    diff = python_array - baseline_array
    max_abs = float(np.max(np.abs(diff), initial=0.0))
    denominator = np.maximum(np.abs(baseline_array), np.finfo(float).eps)
    rel_error = np.abs(diff) / denominator
    max_rel = float(np.max(rel_error, initial=0.0))
    python_norm = _norm(python_array)
    baseline_norm = _norm(baseline_array)
    norm_rel = _norm(diff) / max(baseline_norm, np.finfo(float).eps)
    elementwise_ok = (np.abs(diff) <= abs_tolerance) | (rel_error <= rel_tolerance)
    status = "ok" if bool(np.all(elementwise_ok)) or norm_rel <= rel_tolerance else "diff"
    return ComparisonMetric(
        stage=stage,
        quantity=quantity,
        shape=tuple(int(part) for part in python_array.shape),
        max_abs_error=max_abs,
        max_rel_error=max_rel,
        norm_rel_error=norm_rel,
        python_norm=python_norm,
        baseline_norm=baseline_norm,
        status=status,
    )


def _align_vector_shape(python_array: np.ndarray, baseline_array: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if python_array.shape == baseline_array.shape:
        return python_array, baseline_array
    if python_array.size != baseline_array.size:
        return python_array, baseline_array
    if python_array.ndim == 1:
        return python_array.reshape(baseline_array.shape), baseline_array
    if baseline_array.ndim == 1:
        return python_array, baseline_array.reshape(python_array.shape)
    return python_array, baseline_array


def _array_payload(value: Any) -> list[Any]:
    array = np.asarray(value, dtype=float)
    if np.issubdtype(array.dtype, np.floating) and np.any(~np.isfinite(array)):
        object_array = array.astype(object)
        object_array[~np.isfinite(array)] = None
        return object_array.tolist()
    return array.tolist()


def _nested_array_payload(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _nested_array_payload(item) for key, item in value.items()}
    return _array_payload(value)


def _as_array(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    return np.asarray(value, dtype=float)


def _norm(value: Any) -> float:
    array = np.asarray(value, dtype=float)
    return float(np.linalg.norm(array.reshape(-1)))


class _ProgressRecorder:
    def __init__(self, *, every: int = 1) -> None:
        self.every = max(1, int(every))
        self.events: list[FullCaseProgressEvent] = []
        self._lock = threading.Lock()
        self._queue: queue.Queue[FullCaseProgressEvent] = queue.Queue()

    def __call__(self, event: FullCaseProgressEvent) -> None:
        with self._lock:
            if self.events and self.events[-1].profile_snapshot is not None:
                self.events[-1] = replace(self.events[-1], profile_snapshot=None)
            self.events.append(event)
        self._queue.put(event)

    def snapshot(self) -> tuple[FullCaseProgressEvent, ...]:
        with self._lock:
            return tuple(self.events)

    def sampled_snapshot(self, max_points: int) -> tuple[FullCaseProgressEvent, ...]:
        with self._lock:
            return _sample_progress_events(self.events, max_points=max_points)

    def drain(self) -> tuple[FullCaseProgressEvent, ...]:
        drained: list[FullCaseProgressEvent] = []
        while True:
            try:
                drained.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return tuple(drained)

    def write_outputs(self, output_dir: Path) -> tuple[Path, Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / "progress.csv"
        svg_path = output_dir / "progress_final.svg"
        events = self.snapshot()
        patch_labels = _patch_force_labels(events)
        patch_columns = ",".join(_patch_force_column(label) for label in patch_labels)
        patch_suffix = f",{patch_columns}" if patch_columns else ""
        with csv_path.open("w", encoding="utf-8") as handle:
            handle.write(
                "stage,step,n_steps,time,dt,front_mileage,iterations,retry_count,step_wall_time,"
                f"contact_force_norm,total_force_norm,max_patch_force_z,"
                f"damping_clip_count,damping_clip_max_delta_N{patch_suffix}\n"
            )
            for event in events:
                patch_values = _patch_force_values(event, len(patch_labels), fallback_max=len(patch_labels) == 1)
                patch_text = "".join(f",{value:.17g}" for value in patch_values)
                handle.write(
                    f"{event.stage},{event.step_index},{event.n_steps},{event.time:.17g},{event.dt:.17g},"
                    f"{event.front_mileage:.17g},{event.iterations},{event.retry_count},{event.step_wall_time:.17g},"
                    f"{event.contact_force_norm:.17g},"
                    f"{event.total_force_norm:.17g},{event.max_patch_force_z:.17g},"
                    f"{event.damping_clip_count},{event.damping_clip_max_delta:.17g}{patch_text}\n"
                )
        svg_path.write_text(self.svg(events), encoding="utf-8")
        self._write_damping_clip_diagnostics(output_dir, events)
        return csv_path, svg_path

    def _write_damping_clip_diagnostics(self, output_dir: Path, events: tuple[FullCaseProgressEvent, ...]) -> None:
        rows: list[tuple[FullCaseProgressEvent, dict[str, Any]]] = []
        for event in events:
            for diagnostic in event.damping_clip_diagnostics:
                rows.append((event, diagnostic))
        if not rows:
            return
        path = output_dir / "damping_clips.csv"
        with path.open("w", encoding="utf-8") as handle:
            handle.write(
                "stage,step,time,front_mileage,wheelset,side,patch_id,dummy_rail,"
                "elastic_force_N,raw_damping_force_N,clipped_damping_force_N,relative_velocity_ratio\n"
            )
            for event, diagnostic in rows:
                handle.write(
                    f"{event.stage},{event.step_index},{event.time:.17g},{event.front_mileage:.17g},"
                    f"{diagnostic.get('wheelset', '')},{diagnostic.get('side', '')},"
                    f"{diagnostic.get('patch_id', '')},{diagnostic.get('dummy_rail', '')},"
                    f"{float(diagnostic.get('elastic_force', 0.0)):.17g},"
                    f"{float(diagnostic.get('raw_damping_force', 0.0)):.17g},"
                    f"{float(diagnostic.get('clipped_damping_force', 0.0)):.17g},"
                    f"{float(diagnostic.get('relative_velocity_ratio', 0.0)):.17g}\n"
                )

    def svg(self, events: Iterable[FullCaseProgressEvent] | None = None) -> str:
        event_list = list(self.snapshot() if events is None else events)
        return self._svg(event_list)

    def csv_text(self) -> str:
        events = self.snapshot()
        patch_labels = _patch_force_labels(events)
        patch_columns = ",".join(_patch_force_column(label) for label in patch_labels)
        patch_suffix = f",{patch_columns}" if patch_columns else ""
        lines = [
            "stage,step,n_steps,time,dt,front_mileage,iterations,retry_count,step_wall_time,"
            f"contact_force_norm,total_force_norm,max_patch_force_z,damping_clip_count,damping_clip_max_delta_N{patch_suffix}"
        ]
        for event in events:
            patch_values = _patch_force_values(event, len(patch_labels), fallback_max=len(patch_labels) == 1)
            patch_text = "".join(f",{value:.17g}" for value in patch_values)
            lines.append(
                f"{event.stage},{event.step_index},{event.n_steps},{event.time:.17g},{event.dt:.17g},"
                f"{event.front_mileage:.17g},{event.iterations},{event.retry_count},{event.step_wall_time:.17g},"
                f"{event.contact_force_norm:.17g},"
                f"{event.total_force_norm:.17g},{event.max_patch_force_z:.17g},"
                f"{event.damping_clip_count},{event.damping_clip_max_delta:.17g}{patch_text}"
            )
        return "\n".join(lines) + "\n"

    def _svg(self, events: list[FullCaseProgressEvent]) -> str:
        width = 1120
        height = 520
        margin_left = 70
        margin_right = 205
        plot_top = 50
        plot_height = 360
        plot_width = width - margin_left - margin_right
        if not events:
            return (
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
                f'viewBox="0 0 {width} {height}" shape-rendering="geometricPrecision" text-rendering="geometricPrecision">'
                '<rect width="100%" height="100%" fill="#ffffff"/>'
                f'<text x="24" y="28" font-family="{_PLOT_FONT_FAMILY}" font-size="18" fill="#202020">'
                "SDITT full-case progress</text></svg>"
            )
        xs = np.asarray([event.front_mileage for event in events], dtype=float)
        patch_labels, patch_indices = _display_patch_force_selection(events)
        patch_forces = _patch_force_matrix(events, patch_labels, patch_indices)
        stages = [event.stage for event in events]
        parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}" shape-rendering="geometricPrecision" text-rendering="geometricPrecision">',
            '<rect width="100%" height="100%" fill="#ffffff"/>',
            f'<text x="24" y="28" font-family="{_PLOT_FONT_FAMILY}" font-size="18" fill="#202020">SDITT full-case progress</text>',
        ]
        parts.extend(
            self._plot_multi(
                xs,
                patch_forces,
                patch_labels,
                stages,
                x=margin_left,
                y=plot_top,
                width=plot_width,
            height=plot_height,
            title="Patch wheel-rail force magnitude",
            y_label="Patch force magnitude (kN)",
            y_tick_digits=1,
        )
        )
        last = events[-1]
        parts.append(
            f'<text x="24" y="{height - 24}" font-family="{_PLOT_FONT_FAMILY}" font-size="13" fill="#404040">'
            f"latest: {last.stage} step {last.step_index}/{last.n_steps}, mileage {last.front_mileage:.6f} m, "
            f"iterations {last.iterations}, retries {last.retry_count}</text>"
        )
        parts.append("</svg>")
        return "\n".join(parts)

    def _plot_multi(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        labels: tuple[str, ...],
        stages: list[str],
        *,
        x: int,
        y: int,
        width: int,
        height: int,
        title: str,
        y_label: str,
        y_tick_digits: int,
    ) -> list[str]:
        x0, x1 = _plot_range(xs)
        y0, y1 = _plot_range(ys, min_value=0.0)
        x_ticks = _plot_ticks(x0, x1)
        y_ticks = _plot_ticks(y0, y1)
        stage_lines = []
        seen_stage_changes: set[tuple[str, int]] = set()
        previous_stage = stages[0]
        for index, stage in enumerate(stages):
            if index == 0 or stage == previous_stage:
                previous_stage = stage
                continue
            key = (stage, index)
            if key in seen_stage_changes:
                continue
            seen_stage_changes.add(key)
            px = x + (float(xs[index]) - x0) / (x1 - x0) * width
            stage_lines.append(f'<line x1="{px:.3f}" y1="{y}" x2="{px:.3f}" y2="{y + height}" stroke="#8a8a8a" stroke-dasharray="4 4"/>')
            stage_lines.append(f'<text x="{px + 6:.3f}" y="{y + 16}" font-family="{_PLOT_FONT_FAMILY}" font-size="12" fill="#555">{stage}</text>')
            previous_stage = stage
        tick_lines: list[str] = []
        for tick in x_ticks:
            px = x + (tick - x0) / (x1 - x0) * width
            tick_lines.append(f'<line x1="{px:.3f}" y1="{y + height}" x2="{px:.3f}" y2="{y + height - 6}" stroke="#444"/>')
            tick_lines.append(f'<text x="{px:.3f}" y="{y + height + 20}" font-family="{_PLOT_FONT_FAMILY}" font-size="11" fill="#444" text-anchor="middle">{tick:.3g}</text>')
        for tick in y_ticks:
            py = y + height - (tick - y0) / (y1 - y0) * height
            tick_lines.append(f'<line x1="{x}" y1="{py:.3f}" x2="{x + 6}" y2="{py:.3f}" stroke="#444"/>')
            tick_lines.append(f'<text x="{x - 8}" y="{py + 4:.3f}" font-family="{_PLOT_FONT_FAMILY}" font-size="11" fill="#444" text-anchor="end">{tick:.{y_tick_digits}f}</text>')
        series_parts: list[str] = []
        for series_index, label in enumerate(labels):
            color = _PATCH_FORCE_COLORS[series_index % len(_PATCH_FORCE_COLORS)]
            for segment in _plot_series_segments(xs, ys[:, series_index], x0=x0, x1=x1, y0=y0, y1=y1, x=x, y=y, width=width, height=height):
                if len(segment) == 1:
                    px, py = segment[0]
                    series_parts.append(f'<circle cx="{px:.3f}" cy="{py:.3f}" r="3" fill="{color}"/>')
                else:
                    points = " ".join(f"{px:.3f},{py:.3f}" for px, py in segment)
                    series_parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.7" stroke-linejoin="round" stroke-linecap="round"/>')
            legend_y = y + 12 + series_index * 18
            legend_x = x + width - 92
            series_parts.append(f'<line x1="{legend_x}" y1="{legend_y}" x2="{legend_x + 18}" y2="{legend_y}" stroke="{color}" stroke-width="2"/>')
            series_parts.append(f'<text x="{legend_x + 24}" y="{legend_y + 4}" font-family="{_PLOT_FONT_FAMILY}" font-size="11" fill="#333">{label}</text>')
        return [
            f'<text x="{x}" y="{y - 14}" font-family="{_PLOT_FONT_FAMILY}" font-size="15" fill="#202020">{title}</text>',
            f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="#f8f8f8" stroke="#c8c8c8"/>',
            f'<line x1="{x}" y1="{y + height}" x2="{x + width}" y2="{y + height}" stroke="#444"/>',
            f'<line x1="{x}" y1="{y}" x2="{x}" y2="{y + height}" stroke="#444"/>',
            *tick_lines,
            *series_parts,
            *stage_lines,
            f'<text x="{x + width / 2:.3f}" y="{y + height + 42}" font-family="{_PLOT_FONT_FAMILY}" font-size="12" fill="#202020" text-anchor="middle">Mileage (m)</text>',
            f'<text x="{x - 50}" y="{y + height / 2:.3f}" font-family="{_PLOT_FONT_FAMILY}" font-size="12" fill="#202020" text-anchor="middle" transform="rotate(-90 {x - 50} {y + height / 2:.3f})">{y_label}</text>',
        ]

    def _plot(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        stages: list[str],
        *,
        x: int,
        y: int,
        width: int,
        height: int,
        title: str,
        y_label: str,
        y_tick_digits: int,
    ) -> list[str]:
        x0, x1 = _plot_range(xs)
        y0, y1 = _plot_range(ys, min_value=0.0)
        x_ticks = _plot_ticks(x0, x1)
        y_ticks = _plot_ticks(y0, y1)
        points = []
        for x_value, y_value in zip(xs, ys, strict=True):
            px = x + (float(x_value) - x0) / (x1 - x0) * width
            py = y + height - (float(y_value) - y0) / (y1 - y0) * height
            points.append(f"{px:.3f},{py:.3f}")
        stage_lines = []
        seen_stage_changes: set[tuple[str, int]] = set()
        previous_stage = stages[0]
        for index, stage in enumerate(stages):
            if index == 0 or stage == previous_stage:
                previous_stage = stage
                continue
            key = (stage, index)
            if key in seen_stage_changes:
                continue
            seen_stage_changes.add(key)
            px = x + (float(xs[index]) - x0) / (x1 - x0) * width
            stage_lines.append(f'<line x1="{px:.3f}" y1="{y}" x2="{px:.3f}" y2="{y + height}" stroke="#8a8a8a" stroke-dasharray="4 4"/>')
            stage_lines.append(f'<text x="{px + 6:.3f}" y="{y + 16}" font-family="{_PLOT_FONT_FAMILY}" font-size="12" fill="#555">{stage}</text>')
            previous_stage = stage
        tick_lines: list[str] = []
        for tick in x_ticks:
            px = x + (tick - x0) / (x1 - x0) * width
            tick_lines.append(f'<line x1="{px:.3f}" y1="{y + height}" x2="{px:.3f}" y2="{y + height - 6}" stroke="#444"/>')
            tick_lines.append(f'<text x="{px:.3f}" y="{y + height + 20}" font-family="{_PLOT_FONT_FAMILY}" font-size="11" fill="#444" text-anchor="middle">{tick:.3g}</text>')
        for tick in y_ticks:
            py = y + height - (tick - y0) / (y1 - y0) * height
            tick_lines.append(f'<line x1="{x}" y1="{py:.3f}" x2="{x + 6}" y2="{py:.3f}" stroke="#444"/>')
            tick_lines.append(f'<text x="{x - 8}" y="{py + 4:.3f}" font-family="{_PLOT_FONT_FAMILY}" font-size="11" fill="#444" text-anchor="end">{tick:.{y_tick_digits}f}</text>')
        return [
            f'<text x="{x}" y="{y - 14}" font-family="{_PLOT_FONT_FAMILY}" font-size="15" fill="#202020">{title}</text>',
            f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="#f8f8f8" stroke="#c8c8c8"/>',
            f'<line x1="{x}" y1="{y + height}" x2="{x + width}" y2="{y + height}" stroke="#444"/>',
            f'<line x1="{x}" y1="{y}" x2="{x}" y2="{y + height}" stroke="#444"/>',
            *tick_lines,
            f'<polyline points="{" ".join(points)}" fill="none" stroke="#1f77b4" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>',
            *stage_lines,
            f'<text x="{x + width / 2:.3f}" y="{y + height + 42}" font-family="{_PLOT_FONT_FAMILY}" font-size="12" fill="#202020" text-anchor="middle">Mileage (m)</text>',
            f'<text x="{x - 50}" y="{y + height / 2:.3f}" font-family="{_PLOT_FONT_FAMILY}" font-size="12" fill="#202020" text-anchor="middle" transform="rotate(-90 {x - 50} {y + height / 2:.3f})">{y_label}</text>',
        ]


class _PausableProgressRecorder:
    def __init__(
        self,
        recorder: _ProgressRecorder,
        *,
        pause_event: threading.Event,
        state: dict[str, Any],
    ) -> None:
        self.recorder = recorder
        self.pause_event = pause_event
        self.state = state

    def __call__(self, event: FullCaseProgressEvent) -> None:
        self.recorder(event)
        while not self.pause_event.wait(0.2):
            if self.state.get("done") or self.state.get("error"):
                return

    def write_outputs(self, output_dir: Path) -> tuple[Path, Path]:
        return self.recorder.write_outputs(output_dir)


def _patch_force_labels(events: Iterable[FullCaseProgressEvent]) -> tuple[str, ...]:
    event_list = tuple(events)
    for event in event_list:
        if event.patch_force_labels:
            return tuple(str(label) for label in event.patch_force_labels)
    patch_count = max(
        (
            max(
                np.asarray(event.patch_force_magnitude, dtype=float).size,
                np.asarray(event.patch_vertical_force_z, dtype=float).size,
            )
            for event in event_list
        ),
        default=0,
    )
    if patch_count > 0:
        return tuple(f"patch-{index + 1}" for index in range(patch_count))
    return ("max-patch",) if event_list else ()


def _patch_force_column(label: str) -> str:
    safe = "".join(character if character.isalnum() else "_" for character in label).strip("_")
    return f"patch_{safe}_wheel_rail_force_magnitude_N"


def _patch_force_values(event: FullCaseProgressEvent, count: int, *, fallback_max: bool = False) -> np.ndarray:
    values = np.full((count,), np.nan, dtype=float)
    patch_force = np.asarray(event.patch_force_magnitude, dtype=float).reshape(-1)
    if patch_force.size == 0:
        patch_force = np.abs(np.asarray(event.patch_vertical_force_z, dtype=float).reshape(-1))
    if patch_force.size == 0 and fallback_max and count:
        patch_force = np.asarray([event.max_patch_force_z], dtype=float)
    copied = min(count, patch_force.size)
    if copied:
        values[:copied] = patch_force[:copied]
    return values


def _display_patch_force_selection(events: Iterable[FullCaseProgressEvent]) -> tuple[tuple[str, ...], tuple[int, ...]]:
    labels = _patch_force_labels(events)
    if not labels:
        return (), ()
    first_label = labels[0]
    if "-" not in first_label:
        return labels, tuple(range(len(labels)))
    first_wheelset = first_label.split("-", 1)[0]
    indexes = tuple(index for index, label in enumerate(labels) if label.split("-", 1)[0] == first_wheelset)
    if not indexes:
        return labels, tuple(range(len(labels)))
    return tuple(labels[index] for index in indexes), indexes


def _patch_force_matrix(
    events: Iterable[FullCaseProgressEvent],
    labels: tuple[str, ...],
    indices: tuple[int, ...] | None = None,
) -> np.ndarray:
    event_list = tuple(events)
    matrix = np.full((len(event_list), len(labels)), np.nan, dtype=float)
    fallback_max = len(labels) == 1 and labels[0] == "max-patch"
    source_count = max(indices, default=len(labels) - 1) + 1 if indices is not None else len(labels)
    for row, event in enumerate(event_list):
        values = _patch_force_values(event, source_count, fallback_max=fallback_max)
        if indices is None:
            matrix[row, :] = values[: len(labels)] / 1000.0
        else:
            matrix[row, :] = values[np.asarray(indices, dtype=int)] / 1000.0
    return matrix


def _plot_series_segments(
    xs: np.ndarray,
    ys: np.ndarray,
    *,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    x: int,
    y: int,
    width: int,
    height: int,
) -> list[list[tuple[float, float]]]:
    segments: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for x_value, y_value in zip(xs, ys, strict=True):
        if not np.isfinite(x_value) or not np.isfinite(y_value):
            if current:
                segments.append(current)
                current = []
            continue
        px = x + (float(x_value) - x0) / (x1 - x0) * width
        py = y + height - (float(y_value) - y0) / (y1 - y0) * height
        current.append((px, py))
    if current:
        segments.append(current)
    return segments


def _plot_range(values: np.ndarray, *, min_value: float | None = None) -> tuple[float, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        low = 0.0 if min_value is None else float(min_value)
        return low, low + 1.0
    low = float(np.min(finite))
    high = float(np.max(finite))
    if min_value is not None:
        low = float(min_value)
        high = max(high, low)
    if low == high:
        pad = max(abs(low) * 0.05, 1.0)
        if min_value is not None:
            return float(min_value), max(float(min_value) + pad, high + pad)
        return low - pad, high + pad
    pad = (high - low) * 0.05
    if min_value is not None:
        return float(min_value), high + pad
    return low - pad, high + pad


def _plot_ticks(low: float, high: float, count: int = 5) -> np.ndarray:
    return np.linspace(low, high, count)


def _profile_points(value: Any) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.size == 0:
        return np.zeros((0, 2), dtype=float)
    return array.reshape((-1, array.shape[-1]))[:, :2]


def _format_elapsed_time(seconds: float) -> str:
    total_seconds = max(0, int(seconds))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def _format_timing_top(timing: dict[str, float] | Any, *, limit: int = 3) -> str:
    if not isinstance(timing, dict):
        return ""
    ranked = sorted(
        ((str(name), float(seconds)) for name, seconds in timing.items() if float(seconds) > 0.0),
        key=lambda item: item[1],
        reverse=True,
    )
    if not ranked:
        return ""
    return " | ".join(f"{name} {seconds:.1f}s" for name, seconds in ranked[:limit])


def _step_wall_times(events: Iterable[FullCaseProgressEvent]) -> np.ndarray:
    return np.asarray([max(0.0, float(getattr(event, "step_wall_time", 0.0) or 0.0)) for event in events], dtype=float)


def _sample_progress_events(events: Sequence[FullCaseProgressEvent], *, max_points: int) -> tuple[FullCaseProgressEvent, ...]:
    limit = max(1, int(max_points))
    event_count = len(events)
    if event_count <= limit:
        return tuple(events)
    indexes = np.linspace(0, event_count - 1, limit, dtype=int)
    indexes[-1] = event_count - 1
    return tuple(events[int(index)] for index in np.unique(indexes))


_LIVE_DISPLAY_MAX_POINTS = 2000


class _LiveProgressWindow:
    def __init__(self, recorder: _ProgressRecorder, state: dict[str, Any], *, start_callback: Any) -> None:
        import tkinter as tk
        from tkinter import filedialog
        from tkinter import ttk

        self.tk = tk
        self.filedialog = filedialog
        self.recorder = recorder
        self.state = state
        self.start_callback = start_callback
        self.root = tk.Tk()
        self.root.title("SDITT Full-Case Progress")
        self.root.geometry("1040x720")
        self.closed = False
        self.started_at = time.monotonic()
        self.completed_elapsed: float | None = None

        controls = ttk.Frame(self.root)
        controls.pack(fill="x", padx=12, pady=(10, 4))
        self.save_checkpoints_var = tk.BooleanVar(value=bool(state.get("save_checkpoints")))
        self.save_check = ttk.Checkbutton(
            controls,
            text="本次运行生成存档",
            variable=self.save_checkpoints_var,
        )
        self.save_check.pack(side="left", padx=(0, 12))
        self.resume_var = tk.BooleanVar(value=bool(state.get("resume_checkpoint")))
        self.resume_check = ttk.Checkbutton(
            controls,
            text="采用之前的存档",
            variable=self.resume_var,
            state="normal" if state.get("checkpoint_summaries") else "disabled",
        )
        self.resume_check.pack(side="left")
        self.choose_button = ttk.Button(controls, text="选择存档文件", command=self._choose_checkpoint_file)
        self.choose_button.pack(side="left", padx=(12, 0))
        self.start_button = ttk.Button(controls, text="开始计算", command=self._start_run)
        self.start_button.pack(side="right")
        self.pause_button = ttk.Button(controls, text="暂停计算", command=self._toggle_pause, state="disabled")
        self.pause_button.pack(side="right", padx=(0, 8))
        chooser = ttk.Frame(self.root)
        chooser.pack(fill="x", padx=12, pady=(0, 4))
        self.checkpoint_choice_var = tk.StringVar(value=self._selected_checkpoint_label())
        self.checkpoint_combo = ttk.Combobox(
            chooser,
            textvariable=self.checkpoint_choice_var,
            values=self._checkpoint_choice_labels(),
            state="readonly" if state.get("checkpoint_summaries") else "disabled",
        )
        self.checkpoint_combo.pack(fill="x")
        self.summary = ttk.Label(self.root, text="Starting...", anchor="w")
        self.summary.pack(fill="x", padx=12, pady=(0, 4))
        self.progress = ttk.Progressbar(self.root, orient="horizontal", mode="determinate", maximum=100.0)
        self.progress.pack(fill="x", padx=12, pady=(0, 10))
        self.canvas = tk.Canvas(self.root, width=1000, height=580, bg="white", highlightthickness=1, highlightbackground="#c8c8c8")
        self.canvas.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        self.status = ttk.Label(self.root, text="Close hides the window; the simulation keeps running.", anchor="w")
        self.status.pack(fill="x", padx=12, pady=(0, 10))
        self._latest_events: tuple[FullCaseProgressEvent, ...] = ()
        self._refresh_after_id: str | None = None
        self._resize_after_id: str | None = None
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self.root.protocol("WM_DELETE_WINDOW", self._hide_window)

    def _checkpoint_choice_labels(self) -> tuple[str, ...]:
        return tuple(self._checkpoint_label(summary) for summary in self.state.get("checkpoint_summaries", ()))

    def _selected_checkpoint_label(self) -> str:
        summaries = tuple(self.state.get("checkpoint_summaries", ()))
        if not summaries:
            return "无可用历史存档"
        selected = self.state.get("checkpoint_summary") or summaries[0]
        return self._checkpoint_label(selected)

    def _checkpoint_label(self, summary: dict[str, Any]) -> str:
        path = Path(summary.get("path", ""))
        return (
            f"{summary.get('stage')} | {float(summary.get('front_mileage', 0.0)):.3f} m | "
            f"step {int(summary.get('step_index') or 0)} | {path.name}"
        )

    def _selected_checkpoint_summary(self) -> dict[str, Any] | None:
        label = self.checkpoint_choice_var.get()
        for summary in self.state.get("checkpoint_summaries", ()):
            if self._checkpoint_label(summary) == label:
                return summary
        return None

    def _choose_checkpoint_file(self) -> None:
        path = self.filedialog.askopenfilename(
            title="选择 SDITT 存档文件",
            filetypes=(("Checkpoint files", "*.pkl"), ("All files", "*")),
        )
        if not path:
            return
        validator = self.state.get("validate_checkpoint_path")
        summary = None if validator is None else validator(Path(path))
        if summary is None:
            self.status.configure(text="选择的存档与当前运行参数或数据文件不兼容。")
            return
        summaries = [summary]
        summaries.extend(
            existing
            for existing in self.state.get("checkpoint_summaries", ())
            if Path(existing.get("path", "")) != Path(summary.get("path", ""))
        )
        self.state["checkpoint_summaries"] = tuple(summaries)
        self.state["checkpoint_summary"] = summary
        labels = self._checkpoint_choice_labels()
        self.checkpoint_combo.configure(values=labels, state="readonly")
        self.resume_check.configure(state="normal")
        self.checkpoint_choice_var.set(self._checkpoint_label(summary))
        self.resume_var.set(True)

    def _start_run(self) -> None:
        if self.state.get("worker_started"):
            return
        selected_summary = self._selected_checkpoint_summary()
        self.state["save_checkpoints"] = bool(self.save_checkpoints_var.get())
        self.state["resume_checkpoint"] = bool(self.resume_var.get() and selected_summary is not None)
        self.state["checkpoint_summary"] = selected_summary
        self.state["resume_checkpoint_path"] = None if selected_summary is None else Path(selected_summary["path"])
        self.state["worker_started"] = True
        self.started_at = time.monotonic()
        self.completed_elapsed = None
        self.start_button.configure(state="disabled")
        self.save_check.configure(state="disabled")
        self.resume_check.configure(state="disabled")
        self.choose_button.configure(state="disabled")
        self.checkpoint_combo.configure(state="disabled")
        self.pause_button.configure(state="normal")
        self.start_callback()

    def _toggle_pause(self) -> None:
        pause_event = self.state.get("pause_event")
        if pause_event is None or not self.state.get("worker_started") or self.state.get("done"):
            return
        if self.state.get("paused"):
            pause_event.set()
            self.state["paused"] = False
            self.pause_button.configure(text="暂停计算")
            self._schedule_refresh(0)
        else:
            pause_event.clear()
            self.state["paused"] = True
            self.pause_button.configure(text="继续计算")
            self.status.configure(text="Paused. 点击继续计算恢复。")
            if self._refresh_after_id is not None:
                self.root.after_cancel(self._refresh_after_id)
                self._refresh_after_id = None

    def run(self) -> None:
        self._schedule_refresh(_LIVE_REFRESH_MS)
        self.root.mainloop()

    def _schedule_refresh(self, delay_ms: int = _LIVE_REFRESH_MS) -> None:
        if self._refresh_after_id is None:
            self._refresh_after_id = self.root.after(delay_ms, self._refresh)

    def _hide_window(self) -> None:
        self.closed = True
        self.root.withdraw()

    def _refresh(self) -> None:
        self._refresh_after_id = None
        events = self.recorder.sampled_snapshot(_LIVE_DISPLAY_MAX_POINTS)
        if not self.state.get("worker_started"):
            self.summary.configure(text="Ready to run.")
            self.status.configure(text="选择是否生成存档、是否使用历史存档，然后点击开始计算。")
            self._schedule_refresh()
            return
        elapsed_seconds = self._elapsed_seconds()
        elapsed_text = _format_elapsed_time(elapsed_seconds)
        timing_text = ""
        if events:
            latest = events[-1]
            timing_text = _format_timing_top(dict(latest.timing))
            percent = 0.0 if latest.n_steps <= 0 else min(100.0, latest.step_index / latest.n_steps * 100.0)
            self.progress.configure(value=percent)
            self.summary.configure(
                text=(
                    f"{latest.stage} step {latest.step_index}/{latest.n_steps} | "
                    f"elapsed {elapsed_text} | "
                    f"mileage {latest.front_mileage:.6f} m | time {latest.time:.6g} s | "
                    f"iterations {latest.iterations} | retries {latest.retry_count}"
                )
            )
            self._draw(events)
        if self.state.get("error") is not None:
            self.pause_button.configure(state="disabled")
            self.status.configure(text=f"Error: {self.state['error']}")
            if self.closed:
                self.root.destroy()
                return
        elif self.state.get("done"):
            self.pause_button.configure(state="disabled")
            suffix = f" Timing: {timing_text}." if timing_text else ""
            self.status.configure(text=f"Completed in {elapsed_text}.{suffix} Final progress files have been written; close the window to exit.")
            if self.closed:
                self.root.destroy()
                return
        elif self.state.get("paused"):
            suffix = f" Timing: {timing_text}." if timing_text else ""
            self.status.configure(text=f"Paused at {elapsed_text}.{suffix} 点击继续计算恢复。")
            return
        else:
            suffix = f" Timing: {timing_text}." if timing_text else ""
            resume = self.state.get("resumed_text", "")
            prefix = f"{resume} " if resume else ""
            self.status.configure(text=f"{prefix}Running for {elapsed_text}.{suffix} Close hides the window; the simulation keeps running.")
        self._schedule_refresh()

    def _elapsed_seconds(self) -> float:
        if self.state.get("done") and self.completed_elapsed is None:
            self.completed_elapsed = time.monotonic() - self.started_at
        if self.completed_elapsed is not None:
            return self.completed_elapsed
        return time.monotonic() - self.started_at

    def _draw(self, events: tuple[FullCaseProgressEvent, ...]) -> None:
        self._latest_events = events
        self.canvas.delete("all")
        width = max(int(self.canvas.winfo_width()), 420)
        height = max(int(self.canvas.winfo_height()), 360)
        margin_left = 70
        content_width = max(260, width - margin_left - 30)
        wide_layout = width >= 860
        profile_column_gap = 62 if wide_layout else 0
        profile_width = max(270, min(360, int(content_width * 0.35))) if wide_layout else 0
        plot_width = max(260, content_width - profile_column_gap - profile_width)
        profile_x = margin_left + plot_width + profile_column_gap
        plot_top = 45
        compact = height < 560
        available_height = max(220, height - plot_top - 62)
        if wide_layout:
            chart_gap = 54 if compact else 66
            plot_height = max(115, min(250, int((available_height - chart_gap) * 0.52)))
            step_plot_height = max(105, available_height - plot_height - chart_gap)
            step_plot_y = plot_top + plot_height + chart_gap
            profile_height = available_height
        else:
            profile_gap = 42 if compact else 60
            plot_height = max(115, min(250, int(available_height * 0.42)))
            step_plot_height = plot_height
            step_plot_y = plot_top
            profile_height = max(60, available_height - plot_height - profile_gap)
        xs = np.asarray([event.front_mileage for event in events], dtype=float)
        patch_labels, patch_indices = _display_patch_force_selection(events)
        patch_forces = _patch_force_matrix(events, patch_labels, patch_indices)
        stages = [event.stage for event in events]
        self._draw_multi_plot(
            xs,
            patch_forces,
            patch_labels,
            stages,
            x=margin_left,
            y=plot_top,
            width=plot_width,
            height=plot_height,
            title="Patch wheel-rail force magnitude",
            y_label="Patch force magnitude (kN)",
            show_legend=True,
            y_tick_digits=1,
        )
        if wide_layout:
            self._draw_plot(
                xs,
                _step_wall_times(events),
                stages,
                x=margin_left,
                y=step_plot_y,
                width=plot_width,
                height=step_plot_height,
                title="Step wall time",
                y_label="Seconds/step",
                latest_marker="※",
                y_tick_digits=2,
            )
            self._draw_profile_panel(
                events[-1].profile_snapshot,
                x=profile_x,
                y=plot_top,
                width=profile_width,
                height=profile_height,
                orientation="vertical",
            )
        else:
            self._draw_profile_panel(
                events[-1].profile_snapshot,
                x=margin_left,
                y=plot_top + plot_height + profile_gap,
                width=plot_width,
                height=profile_height,
            )

    def _on_canvas_resize(self, _event: Any) -> None:
        if not self._latest_events:
            return
        if self._resize_after_id is not None:
            self.root.after_cancel(self._resize_after_id)
        self._resize_after_id = self.root.after(60, self._redraw_after_resize)

    def _redraw_after_resize(self) -> None:
        self._resize_after_id = None
        if self._latest_events:
            self._draw(self._latest_events)

    def _draw_multi_plot(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        labels: tuple[str, ...],
        stages: list[str],
        *,
        x: int,
        y: int,
        width: int,
        height: int,
        title: str,
        y_label: str,
        show_legend: bool,
        y_tick_digits: int,
    ) -> None:
        x0, x1 = _plot_range(xs)
        y0, y1 = _plot_range(ys, min_value=0.0)
        x_ticks = _plot_ticks(x0, x1)
        y_ticks = _plot_ticks(y0, y1)
        self.canvas.create_text(x, y - 18, text=title, anchor="w", font=(_PLOT_FONT_FAMILY, 14), fill="#202020")
        self.canvas.create_rectangle(x, y, x + width, y + height, fill="#f8f8f8", outline="#c8c8c8")
        self.canvas.create_line(x, y + height, x + width, y + height, fill="#444444")
        self.canvas.create_line(x, y, x, y + height, fill="#444444")
        for tick in x_ticks:
            px = x + (float(tick) - x0) / (x1 - x0) * width
            self.canvas.create_line(px, y + height, px, y + height - 6, fill="#444444")
            self.canvas.create_text(px, y + height + 18, text=f"{tick:.3g}", anchor="n", font=(_PLOT_FONT_FAMILY, 10), fill="#444444")
        for tick in y_ticks:
            py = y + height - (float(tick) - y0) / (y1 - y0) * height
            self.canvas.create_line(x, py, x + 6, py, fill="#444444")
            self.canvas.create_text(x - 8, py, text=f"{tick:.{y_tick_digits}f}", anchor="e", font=(_PLOT_FONT_FAMILY, 10), fill="#444444")
        self.canvas.create_text(x + width / 2, y + height + 38, text="Mileage (m)", anchor="n", font=(_PLOT_FONT_FAMILY, 11), fill="#202020")
        self.canvas.create_text(x - 52, y + height / 2, text=y_label, anchor="center", angle=90, font=(_PLOT_FONT_FAMILY, 11), fill="#202020")
        for series_index, label in enumerate(labels):
            color = _PATCH_FORCE_COLORS[series_index % len(_PATCH_FORCE_COLORS)]
            for segment in _plot_series_segments(xs, ys[:, series_index], x0=x0, x1=x1, y0=y0, y1=y1, x=x, y=y, width=width, height=height):
                points: list[float] = []
                for px, py in segment:
                    points.extend([px, py])
                if len(points) >= 4:
                    self.canvas.create_line(*points, fill=color, width=2, smooth=True, splinesteps=24)
                elif len(points) == 2:
                    self.canvas.create_oval(points[0] - 3, points[1] - 3, points[0] + 3, points[1] + 3, fill=color, outline="")
            if show_legend:
                row_spacing = 14
                rows = max(1, int((height - 16) / row_spacing))
                row = series_index % rows
                column = series_index // rows
                legend_x = x + width - 88 - column * 76
                legend_y = y + 12 + row * row_spacing
                self.canvas.create_line(legend_x, legend_y, legend_x + 14, legend_y, fill=color, width=2)
                self.canvas.create_text(legend_x + 19, legend_y, text=label, anchor="w", font=(_PLOT_FONT_FAMILY, 9), fill="#333333")
        previous_stage = stages[0] if stages else ""
        for index, stage in enumerate(stages):
            if index == 0 or stage == previous_stage:
                previous_stage = stage
                continue
            px = x + (float(xs[index]) - x0) / (x1 - x0) * width
            self.canvas.create_line(px, y, px, y + height, fill="#8a8a8a", dash=(4, 4))
            self.canvas.create_text(px + 6, y + 14, text=stage, anchor="w", font=(_PLOT_FONT_FAMILY, 11), fill="#555555")
            previous_stage = stage

    def _draw_plot(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        stages: list[str],
        *,
        x: int,
        y: int,
        width: int,
        height: int,
        title: str,
        y_label: str,
        latest_marker: str | None = None,
        y_tick_digits: int = 2,
    ) -> None:
        x0, x1 = _plot_range(xs)
        y0, y1 = _plot_range(ys, min_value=0.0)
        x_ticks = _plot_ticks(x0, x1)
        y_ticks = _plot_ticks(y0, y1)
        self.canvas.create_text(x, y - 18, text=title, anchor="w", font=(_PLOT_FONT_FAMILY, 14), fill="#202020")
        self.canvas.create_rectangle(x, y, x + width, y + height, fill="#f8f8f8", outline="#c8c8c8")
        self.canvas.create_line(x, y + height, x + width, y + height, fill="#444444")
        self.canvas.create_line(x, y, x, y + height, fill="#444444")
        for tick in x_ticks:
            px = x + (float(tick) - x0) / (x1 - x0) * width
            self.canvas.create_line(px, y + height, px, y + height - 6, fill="#444444")
            self.canvas.create_text(px, y + height + 18, text=f"{tick:.3g}", anchor="n", font=(_PLOT_FONT_FAMILY, 10), fill="#444444")
        for tick in y_ticks:
            py = y + height - (float(tick) - y0) / (y1 - y0) * height
            self.canvas.create_line(x, py, x + 6, py, fill="#444444")
            self.canvas.create_text(x - 8, py, text=f"{tick:.{y_tick_digits}f}", anchor="e", font=(_PLOT_FONT_FAMILY, 10), fill="#444444")
        self.canvas.create_text(x + width / 2, y + height + 38, text="Mileage (m)", anchor="n", font=(_PLOT_FONT_FAMILY, 11), fill="#202020")
        self.canvas.create_text(x - 52, y + height / 2, text=y_label, anchor="center", angle=90, font=(_PLOT_FONT_FAMILY, 11), fill="#202020")
        points: list[float] = []
        for x_value, y_value in zip(xs, ys, strict=True):
            px = x + (float(x_value) - x0) / (x1 - x0) * width
            py = y + height - (float(y_value) - y0) / (y1 - y0) * height
            points.extend([px, py])
        if len(points) >= 4:
            self.canvas.create_line(*points, fill="#1f77b4", width=2)
        elif len(points) == 2:
            self.canvas.create_oval(points[0] - 3, points[1] - 3, points[0] + 3, points[1] + 3, fill="#1f77b4", outline="")
        latest_marker_point: tuple[float, float] | None = None
        if latest_marker:
            finite = np.flatnonzero(np.isfinite(xs) & np.isfinite(ys))
            if finite.size:
                latest_index = int(finite[-1])
                px = x + (float(xs[latest_index]) - x0) / (x1 - x0) * width
                py = y + height - (float(ys[latest_index]) - y0) / (y1 - y0) * height
                latest_marker_point = (px, py)
        previous_stage = stages[0] if stages else ""
        for index, stage in enumerate(stages):
            if index == 0 or stage == previous_stage:
                previous_stage = stage
                continue
            px = x + (float(xs[index]) - x0) / (x1 - x0) * width
            self.canvas.create_line(px, y, px, y + height, fill="#8a8a8a", dash=(4, 4))
            self.canvas.create_text(px + 6, y + 14, text=stage, anchor="w", font=(_PLOT_FONT_FAMILY, 11), fill="#555555")
            previous_stage = stage
        if latest_marker and latest_marker_point is not None:
            self.canvas.create_text(
                latest_marker_point[0],
                latest_marker_point[1],
                text=latest_marker,
                anchor="center",
                font=(_PLOT_FONT_FAMILY, 16, "bold"),
                fill="#d62728",
            )

    def _draw_profile_panel(
        self,
        snapshot: FullCaseProfileSnapshot | None,
        *,
        x: int,
        y: int,
        width: int,
        height: int,
        orientation: str = "horizontal",
    ) -> None:
        self.canvas.create_text(
            x,
            y - 18,
            text="Wheel/Rail profile contact",
            anchor="w",
            font=(_PLOT_FONT_FAMILY, 14),
            fill="#202020",
        )
        self.canvas.create_rectangle(x, y, x + width, y + height, fill="#fbfbfb", outline="#c8c8c8")
        if snapshot is None:
            self.canvas.create_text(
                x + width / 2,
                y + height / 2,
                text="Waiting for contact profile data...",
                anchor="center",
                font=(_PLOT_FONT_FAMILY, 12),
                fill="#666666",
            )
            return

        if orientation == "vertical":
            row_gap = max(16, int(height * 0.035))
            row_height = max(120, int((height - row_gap) / 2))
            self._draw_side_profile_panel(
                snapshot.sides.get("L"),
                label=f"Left {snapshot.wheelset}",
                x=x,
                y=y,
                width=width,
                height=row_height,
            )
            self._draw_side_profile_panel(
                snapshot.sides.get("R"),
                label=f"Right {snapshot.wheelset}",
                x=x,
                y=y + row_height + row_gap,
                width=width,
                height=max(120, height - row_height - row_gap),
            )
        else:
            column_gap = max(18, int(width * 0.035))
            column_width = max(120, int((width - column_gap) / 2))
            self._draw_side_profile_panel(
                snapshot.sides.get("L"),
                label=f"Left {snapshot.wheelset}",
                x=x,
                y=y,
                width=column_width,
                height=height,
            )
            self._draw_side_profile_panel(
                snapshot.sides.get("R"),
                label=f"Right {snapshot.wheelset}",
                x=x + column_width + column_gap,
                y=y,
                width=max(120, width - column_width - column_gap),
                height=height,
            )

    def _draw_side_profile_panel(
        self,
        side_snapshot: FullCaseSideProfileSnapshot | None,
        *,
        label: str,
        x: int,
        y: int,
        width: int,
        height: int,
    ) -> None:
        self.canvas.create_rectangle(x, y, x + width, y + height, fill="#fbfbfb", outline="#dddddd")
        self.canvas.create_text(x + 8, y + 8, text=label, anchor="nw", font=(_PLOT_FONT_FAMILY, 11), fill="#404040")
        if side_snapshot is None:
            self.canvas.create_text(
                x + width / 2,
                y + height / 2,
                text="No profile",
                anchor="center",
                font=(_PLOT_FONT_FAMILY, 11),
                fill="#777777",
            )
            return

        wheel = _profile_points(side_snapshot.wheel_profile)
        rail = _profile_points(side_snapshot.rail_profile)
        wheel_contacts = _profile_points(side_snapshot.wheel_contact_points)
        rail_contacts = _profile_points(side_snapshot.rail_contact_points)
        if wheel.size == 0 or rail.size == 0:
            return

        padding_x = 18
        top_padding = max(24, min(36, int(height * 0.16)))
        bottom_padding = max(8, min(20, int(height * 0.10)))
        band_gap = max(10, min(22, int(height * 0.11)))
        band_height = max(20, int((height - top_padding - bottom_padding - band_gap) / 2))
        wheel_y = y + top_padding
        rail_y = wheel_y + band_height + band_gap

        x_values = np.concatenate(
            [
                wheel[:, 0],
                rail[:, 0],
                wheel_contacts[:, 0] if wheel_contacts.size else np.zeros((0,), dtype=float),
                rail_contacts[:, 0] if rail_contacts.size else np.zeros((0,), dtype=float),
            ]
        )
        x0, x1 = _plot_range(x_values)
        wheel_z0, wheel_z1 = _plot_range(wheel[:, 1])
        rail_z0, rail_z1 = _plot_range(rail[:, 1])

        def px(value: float) -> float:
            return x + padding_x + (float(value) - x0) / (x1 - x0) * max(1, width - 2 * padding_x)

        def py(value: float, z0: float, z1: float, band_y: int) -> float:
            return band_y + (float(value) - z0) / (z1 - z0) * band_height

        self.canvas.create_text(x + 8, wheel_y, text="Wheel", anchor="sw", font=(_PLOT_FONT_FAMILY, 10), fill="#555555")
        self.canvas.create_text(x + 8, rail_y, text="Rail", anchor="sw", font=(_PLOT_FONT_FAMILY, 10), fill="#555555")

        self._draw_profile_polyline(
            wheel,
            px=px,
            py=lambda value: py(value, wheel_z0, wheel_z1, wheel_y),
            fill="#d95f02",
            width=2,
        )
        self._draw_profile_polyline(
            rail,
            px=px,
            py=lambda value: py(value, rail_z0, rail_z1, rail_y),
            fill="#1b9e77",
            width=2,
        )

        contact_count = min(wheel_contacts.shape[0], rail_contacts.shape[0])
        for index in range(contact_count):
            wx = px(wheel_contacts[index, 0])
            wy = py(wheel_contacts[index, 1], wheel_z0, wheel_z1, wheel_y)
            rx = px(rail_contacts[index, 0])
            ry = py(rail_contacts[index, 1], rail_z0, rail_z1, rail_y)
            self.canvas.create_line(wx, wy, rx, ry, fill="#4c78a8", width=1, dash=(3, 3))
            self.canvas.create_oval(wx - 3, wy - 3, wx + 3, wy + 3, fill="#d95f02", outline="")
            self.canvas.create_oval(rx - 3, ry - 3, rx + 3, ry + 3, fill="#1b9e77", outline="")

    def _draw_profile_polyline(
        self,
        points: np.ndarray,
        *,
        px: Any,
        py: Any,
        fill: str,
        width: int,
    ) -> None:
        line_points: list[float] = []
        finite = points[np.all(np.isfinite(points), axis=1)]
        for lateral, vertical in finite:
            line_points.extend([px(float(lateral)), py(float(vertical))])
        if len(line_points) >= 4:
            self.canvas.create_line(*line_points, fill=fill, width=width, smooth=True)


def _run_with_live_window(args: argparse.Namespace, *, cut_freq: float | None) -> int:
    recorder = _ProgressRecorder(every=args.plot_every)
    pause_event = threading.Event()
    pause_event.set()
    checkpoint_summaries = _checkpoint_summaries_for_args(args, cut_freq=cut_freq)
    checkpoint_summary = checkpoint_summaries[0] if checkpoint_summaries else None
    if args.checkpoint_path is not None:
        explicit_summary = _checkpoint_summary_for_path(args, args.checkpoint_path, cut_freq=cut_freq)
        if explicit_summary is not None:
            checkpoint_summaries = (explicit_summary,) + tuple(
                summary
                for summary in checkpoint_summaries
                if Path(summary.get("path", "")) != Path(explicit_summary.get("path", ""))
            )
            checkpoint_summary = explicit_summary
    state: dict[str, Any] = {
        "done": False,
        "error": None,
        "traceback": None,
        "paths": None,
        "worker_started": False,
        "checkpoint_summary": checkpoint_summary,
        "checkpoint_summaries": checkpoint_summaries,
        "save_checkpoints": bool(args.save_checkpoints),
        "resume_checkpoint": False if args.resume_checkpoint is None else bool(args.resume_checkpoint),
        "resume_checkpoint_path": args.checkpoint_path,
        "validate_checkpoint_path": lambda path: _checkpoint_summary_for_path(args, path, cut_freq=cut_freq),
        "pause_event": pause_event,
        "paused": False,
        "resumed_text": "",
    }
    progress_recorder = _PausableProgressRecorder(recorder, pause_event=pause_event, state=state)

    def worker() -> None:
        try:
            if state.get("resume_checkpoint") and state.get("checkpoint_summary"):
                summary = state["checkpoint_summary"]
                state["resumed_text"] = f"Resumed from checkpoint at {float(summary.get('front_mileage', 0.0)):.3f} m."
            state["paths"] = write_full_case_short_run_report(
                args.output_dir,
                cut_freq=cut_freq,
                dt=args.dt,
                n_steps_per_stage=args.steps,
                use_sparse=not args.dense,
                use_matlab_mileage_endpoints=args.matlab_mileage_endpoints,
                plot_progress=args.plot_progress,
                plot_every=args.plot_every,
                save_progress=True,
                preload_cache_dir=args.preload_cache_dir,
                history_retention_steps=args.history_retention_steps,
                checkpoint_dir=args.checkpoint_dir,
                save_checkpoints=bool(state.get("save_checkpoints")),
                resume_checkpoint=bool(state.get("resume_checkpoint")),
                resume_checkpoint_path=state.get("resume_checkpoint_path"),
                progress_recorder=progress_recorder,
                matlab_baseline_path=args.matlab_baseline,
                abs_tolerance=args.abs_tol,
                rel_tolerance=args.rel_tol,
                rail_layout=args.rail_layout,
            )
        except Exception as exc:  # pragma: no cover - exercised manually with GUI failures.
            state["error"] = f"{type(exc).__name__}: {exc}"
            state["traceback"] = traceback.format_exc()
            if recorder.snapshot():
                try:
                    recorder.write_outputs(args.output_dir)
                except Exception:
                    pass
        finally:
            pause_event.set()
            state["paused"] = False
            state["done"] = True

    thread: threading.Thread | None = None

    def start_worker() -> None:
        nonlocal thread
        thread = threading.Thread(target=worker, name="sditt-full-case-runner")
        thread.start()

    window = _LiveProgressWindow(recorder, state, start_callback=start_worker)
    window.run()
    if thread is not None:
        thread.join()
    if state.get("traceback"):
        print(state["traceback"])
    paths = state.get("paths")
    if paths:
        snapshot_path, report_path = paths
        print(f"wrote {snapshot_path}")
        print(f"wrote {report_path}")
    return 1 if state.get("error") else 0


def _checkpoint_summary_for_args(args: argparse.Namespace, *, cut_freq: float | None) -> dict[str, Any] | None:
    summaries = _checkpoint_summaries_for_args(args, cut_freq=cut_freq)
    return summaries[0] if summaries else None


def _checkpoint_summaries_for_args(args: argparse.Namespace, *, cut_freq: float | None) -> tuple[dict[str, Any], ...]:
    if args.checkpoint_dir is None:
        return ()
    settings = _checkpoint_settings_for_args(args, cut_freq=cut_freq)
    return list_default_full_case_checkpoints(
        settings=settings,
        operating_case=DefaultOperatingCase(rail_layout=args.rail_layout),
    )


def _checkpoint_summary_for_path(
    args: argparse.Namespace,
    checkpoint_path: str | Path,
    *,
    cut_freq: float | None,
) -> dict[str, Any] | None:
    settings = _checkpoint_settings_for_args(args, cut_freq=cut_freq)
    return load_default_full_case_checkpoint_summary(
        checkpoint_path,
        settings=settings,
        operating_case=DefaultOperatingCase(rail_layout=args.rail_layout),
    )


def _checkpoint_settings_for_args(args: argparse.Namespace, *, cut_freq: float | None) -> FullDefaultCaseSettings:
    return FullDefaultCaseSettings(
        cut_freq=cut_freq,
        dt=args.dt,
        n_steps_per_stage=args.steps,
        use_matlab_mileage_endpoints=args.matlab_mileage_endpoints,
        use_sparse=not args.dense,
        preload_cache_dir=args.preload_cache_dir,
        history_retention_steps=args.history_retention_steps,
        checkpoint_dir=args.checkpoint_dir,
        resume_checkpoint=False,
    )


def _json_dumps(value: Any) -> str:
    return json.dumps(value, indent=2, sort_keys=True, allow_nan=False)


def _read_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _history_retention_arg(value: str) -> int | None:
    parsed = int(value)
    return None if parsed <= 0 else parsed


def _parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run and compare a short SDITT full-case validation.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--matlab-baseline", type=Path, default=None)
    parser.add_argument(
        "--rail-layout",
        choices=("interval", "turnout"),
        default="interval",
        help="Use two constant basic rails (interval, default) or the original R1+R2+R3 turnout contact layout.",
    )
    parser.add_argument("--dt", type=float, default=1.0e-4)
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument(
        "--matlab-mileage-endpoints",
        action="store_true",
        help="Run each stage to the default MATLAB Face/Trail mileage endpoint instead of using --steps.",
    )
    parser.add_argument("--cut-freq", type=float, default=50.0)
    parser.add_argument("--full-size", action="store_true", help="Use the default full-size modal set instead of cut_freq.")
    parser.add_argument("--dense", action="store_true", help="Use dense system matrices.")
    parser.add_argument("--plot-progress", action="store_true", help="Compatibility alias for final progress output.")
    parser.add_argument("--save-progress", action="store_true", help="Write final progress CSV and SVG after the run completes.")
    parser.add_argument("--live-window", action="store_true", help="Show a Tk realtime progress window while the simulation runs.")
    parser.add_argument(
        "--preload-cache-dir",
        type=Path,
        default=None,
        help="Cache completed Preload state here and reuse it for identical input settings.",
    )
    parser.add_argument(
        "--checkpoint-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR / "checkpoints",
        help="Discover matching run checkpoints here, and save new checkpoints here when --save-checkpoints is used.",
    )
    parser.add_argument(
        "--save-checkpoints",
        action="store_true",
        help="Save non-overwriting run checkpoints every checkpoint interval.",
    )
    parser.add_argument(
        "--checkpoint-path",
        type=Path,
        default=None,
        help="Resume from this specific checkpoint when --resume-checkpoint is used.",
    )
    resume_group = parser.add_mutually_exclusive_group()
    resume_group.add_argument(
        "--resume-checkpoint",
        dest="resume_checkpoint",
        action="store_true",
        default=None,
        help="Resume from the latest matching run checkpoint when available.",
    )
    resume_group.add_argument(
        "--no-resume-checkpoint",
        dest="resume_checkpoint",
        action="store_false",
        help="Ignore matching run checkpoints and start from the beginning.",
    )
    parser.add_argument(
        "--history-retention-steps",
        type=_history_retention_arg,
        default=256,
        help="Retain only this many recent full-state diagnostic steps in memory; use 0 to keep all.",
    )
    parser.add_argument("--plot-every", type=int, default=1, help="Reserved progress refresh interval; final files are written once.")
    parser.add_argument("--abs-tol", type=float, default=DEFAULT_ABS_TOLERANCE)
    parser.add_argument("--rel-tol", type=float, default=DEFAULT_REL_TOLERANCE)
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    cut_freq = None if args.full_size else args.cut_freq
    if args.live_window:
        return _run_with_live_window(args, cut_freq=cut_freq)
    snapshot_path, report_path = write_full_case_short_run_report(
        args.output_dir,
        cut_freq=cut_freq,
        dt=args.dt,
        n_steps_per_stage=args.steps,
        use_sparse=not args.dense,
        use_matlab_mileage_endpoints=args.matlab_mileage_endpoints,
        plot_progress=args.plot_progress,
        plot_every=args.plot_every,
        save_progress=args.save_progress,
        preload_cache_dir=args.preload_cache_dir,
        history_retention_steps=args.history_retention_steps,
        checkpoint_dir=args.checkpoint_dir,
        save_checkpoints=args.save_checkpoints,
        resume_checkpoint=bool(args.resume_checkpoint),
        resume_checkpoint_path=args.checkpoint_path,
        matlab_baseline_path=args.matlab_baseline,
        abs_tolerance=args.abs_tol,
        rel_tolerance=args.rel_tol,
        rail_layout=args.rail_layout,
    )
    print(f"wrote {snapshot_path}")
    print(f"wrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
