from __future__ import annotations

import argparse
import json
import queue
import threading
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from sditt.simulation import (
    FullCaseProgressEvent,
    FullDefaultCaseRunResult,
    FullDefaultCaseSettings,
    run_default_full_case_driver,
)


DEFAULT_OUTPUT_DIR = Path("python/outputs/full_case_short_run")
DEFAULT_ABS_TOLERANCE = 1.0e-6
DEFAULT_REL_TOLERANCE = 1.0e-4


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
    progress_callback: Any = None,
) -> dict[str, Any]:
    """Run the Python full-case driver and serialize key validation quantities."""

    settings = FullDefaultCaseSettings(
        cut_freq=cut_freq,
        dt=dt,
        n_steps_per_stage=n_steps_per_stage,
        use_matlab_mileage_endpoints=use_matlab_mileage_endpoints,
        use_sparse=use_sparse,
        progress_callback=progress_callback,
    )
    result = run_default_full_case_driver(settings=settings)
    return snapshot_from_run_result(result)


def snapshot_from_run_result(result: FullDefaultCaseRunResult) -> dict[str, Any]:
    preparation = result.preparation
    return {
        "schema": "sditt-full-case-short-run-v1",
        "source": "python",
        "settings": {
            "cut_freq": preparation.settings.cut_freq,
            "dt": preparation.settings.dt,
            "n_steps_per_stage": preparation.settings.n_steps_per_stage,
            "use_matlab_mileage_endpoints": preparation.settings.use_matlab_mileage_endpoints,
            "use_sparse": preparation.settings.use_sparse,
        },
        "preparation": {
            "total_dof": preparation.total_dof,
            "n_track": preparation.system.layout.n_track,
            "wheelsets": list(preparation.operating_case.wheelsets),
            "missing_stages": [stage.name for stage in preparation.missing_stages],
        },
        "stages": [_stage_snapshot(stage) for stage in result.stages],
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
    progress_recorder: "_ProgressRecorder | None" = None,
    matlab_baseline_path: str | Path | None = None,
    abs_tolerance: float = DEFAULT_ABS_TOLERANCE,
    rel_tolerance: float = DEFAULT_REL_TOLERANCE,
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
        progress_callback=progress_writer,
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
    }


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
            self.events.append(event)
        self._queue.put(event)

    def snapshot(self) -> tuple[FullCaseProgressEvent, ...]:
        with self._lock:
            return tuple(self.events)

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
        with csv_path.open("w", encoding="utf-8") as handle:
            handle.write(
                "stage,step,n_steps,time,dt,front_mileage,iterations,"
                "contact_force_norm,total_force_norm,max_patch_force_z\n"
            )
            for event in events:
                handle.write(
                    f"{event.stage},{event.step_index},{event.n_steps},{event.time:.17g},{event.dt:.17g},"
                    f"{event.front_mileage:.17g},{event.iterations},{event.contact_force_norm:.17g},"
                    f"{event.total_force_norm:.17g},{event.max_patch_force_z:.17g}\n"
                )
        svg_path.write_text(self.svg(events), encoding="utf-8")
        return csv_path, svg_path

    def svg(self, events: Iterable[FullCaseProgressEvent] | None = None) -> str:
        event_list = list(self.snapshot() if events is None else events)
        return self._svg(event_list)

    def csv_text(self) -> str:
        lines = [
            "stage,step,n_steps,time,dt,front_mileage,iterations,contact_force_norm,total_force_norm,max_patch_force_z"
        ]
        for event in self.snapshot():
            lines.append(
                f"{event.stage},{event.step_index},{event.n_steps},{event.time:.17g},{event.dt:.17g},"
                f"{event.front_mileage:.17g},{event.iterations},{event.contact_force_norm:.17g},"
                f"{event.total_force_norm:.17g},{event.max_patch_force_z:.17g}"
            )
        return "\n".join(lines) + "\n"

    def _svg(self, events: list[FullCaseProgressEvent]) -> str:
        width = 980
        height = 520
        margin_left = 70
        margin_right = 25
        plot_top = 50
        plot_gap = 60
        plot_height = 170
        plot_width = width - margin_left - margin_right
        if not events:
            return (
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
                f'viewBox="0 0 {width} {height}"><rect width="100%" height="100%" fill="#ffffff"/>'
                '<text x="24" y="28" font-family="Arial" font-size="18" fill="#202020">'
                "SDITT full-case progress</text></svg>"
            )
        xs = np.asarray([event.front_mileage for event in events], dtype=float)
        force = np.asarray([event.contact_force_norm for event in events], dtype=float) / 1000.0
        patch = np.asarray([event.max_patch_force_z for event in events], dtype=float) / 1000.0
        stages = [event.stage for event in events]
        parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="#ffffff"/>',
            '<text x="24" y="28" font-family="Arial" font-size="18" fill="#202020">SDITT full-case progress</text>',
        ]
        parts.extend(
            self._plot(
                xs,
                force,
                stages,
                x=margin_left,
                y=plot_top,
                width=plot_width,
                height=plot_height,
                title="Contact force norm",
                y_label="Contact force norm (kN)",
            )
        )
        parts.extend(
            self._plot(
                xs,
                patch,
                stages,
                x=margin_left,
                y=plot_top + plot_height + plot_gap,
                width=plot_width,
                height=plot_height,
                title="Max patch vertical force",
                y_label="Max patch vertical force (kN)",
            )
        )
        last = events[-1]
        parts.append(
            f'<text x="24" y="{height - 24}" font-family="Arial" font-size="13" fill="#404040">'
            f"latest: {last.stage} step {last.step_index}/{last.n_steps}, mileage {last.front_mileage:.6f} m, "
            f"iterations {last.iterations}</text>"
        )
        parts.append("</svg>")
        return "\n".join(parts)

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
    ) -> list[str]:
        x0, x1 = _plot_range(xs)
        y0, y1 = _plot_range(ys)
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
            stage_lines.append(f'<text x="{px + 6:.3f}" y="{y + 16}" font-family="Arial" font-size="12" fill="#555">{stage}</text>')
            previous_stage = stage
        tick_lines: list[str] = []
        for tick in x_ticks:
            px = x + (tick - x0) / (x1 - x0) * width
            tick_lines.append(f'<line x1="{px:.3f}" y1="{y + height}" x2="{px:.3f}" y2="{y + height + 5}" stroke="#444"/>')
            tick_lines.append(f'<text x="{px:.3f}" y="{y + height + 20}" font-family="Arial" font-size="11" fill="#444" text-anchor="middle">{tick:.3g}</text>')
        for tick in y_ticks:
            py = y + height - (tick - y0) / (y1 - y0) * height
            tick_lines.append(f'<line x1="{x - 5}" y1="{py:.3f}" x2="{x}" y2="{py:.3f}" stroke="#444"/>')
            tick_lines.append(f'<text x="{x - 8}" y="{py + 4:.3f}" font-family="Arial" font-size="11" fill="#444" text-anchor="end">{tick:.3g}</text>')
        return [
            f'<text x="{x}" y="{y - 14}" font-family="Arial" font-size="15" fill="#202020">{title}</text>',
            f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="#f8f8f8" stroke="#c8c8c8"/>',
            f'<line x1="{x}" y1="{y + height}" x2="{x + width}" y2="{y + height}" stroke="#444"/>',
            f'<line x1="{x}" y1="{y}" x2="{x}" y2="{y + height}" stroke="#444"/>',
            *tick_lines,
            f'<polyline points="{" ".join(points)}" fill="none" stroke="#1f77b4" stroke-width="2"/>',
            *stage_lines,
            f'<text x="{x + width / 2:.3f}" y="{y + height + 42}" font-family="Arial" font-size="12" fill="#202020" text-anchor="middle">Mileage (m)</text>',
            f'<text x="{x - 50}" y="{y + height / 2:.3f}" font-family="Arial" font-size="12" fill="#202020" text-anchor="middle" transform="rotate(-90 {x - 50} {y + height / 2:.3f})">{y_label}</text>',
        ]


def _plot_range(values: np.ndarray) -> tuple[float, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return 0.0, 1.0
    low = float(np.min(finite))
    high = float(np.max(finite))
    if low == high:
        pad = max(abs(low) * 0.05, 1.0)
        return low - pad, high + pad
    pad = (high - low) * 0.05
    return low - pad, high + pad


def _plot_ticks(low: float, high: float, count: int = 5) -> np.ndarray:
    return np.linspace(low, high, count)


class _LiveProgressWindow:
    def __init__(self, recorder: _ProgressRecorder, state: dict[str, Any]) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.recorder = recorder
        self.state = state
        self.root = tk.Tk()
        self.root.title("SDITT Full-Case Progress")
        self.root.geometry("1040x720")
        self.closed = False

        self.summary = ttk.Label(self.root, text="Starting...", anchor="w")
        self.summary.pack(fill="x", padx=12, pady=(10, 4))
        self.progress = ttk.Progressbar(self.root, orient="horizontal", mode="determinate", maximum=100.0)
        self.progress.pack(fill="x", padx=12, pady=(0, 10))
        self.canvas = tk.Canvas(self.root, width=1000, height=580, bg="white", highlightthickness=1, highlightbackground="#c8c8c8")
        self.canvas.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        self.status = ttk.Label(self.root, text="Close hides the window; the simulation keeps running.", anchor="w")
        self.status.pack(fill="x", padx=12, pady=(0, 10))
        self.root.protocol("WM_DELETE_WINDOW", self._hide_window)

    def run(self) -> None:
        self.root.after(250, self._refresh)
        self.root.mainloop()

    def _hide_window(self) -> None:
        self.closed = True
        self.root.withdraw()

    def _refresh(self) -> None:
        events = self.recorder.snapshot()
        if events:
            latest = events[-1]
            percent = 0.0 if latest.n_steps <= 0 else min(100.0, latest.step_index / latest.n_steps * 100.0)
            self.progress.configure(value=percent)
            self.summary.configure(
                text=(
                    f"{latest.stage} step {latest.step_index}/{latest.n_steps} | "
                    f"mileage {latest.front_mileage:.6f} m | time {latest.time:.6g} s | "
                    f"iterations {latest.iterations}"
                )
            )
            self._draw(events)
        if self.state.get("error") is not None:
            self.status.configure(text=f"Error: {self.state['error']}")
            if self.closed:
                self.root.destroy()
                return
        elif self.state.get("done"):
            self.status.configure(text="Completed. Final progress files have been written; close the window to exit.")
            if self.closed:
                self.root.destroy()
                return
        else:
            self.status.configure(text="Running. Close hides the window; the simulation keeps running.")
        self.root.after(500, self._refresh)

    def _draw(self, events: tuple[FullCaseProgressEvent, ...]) -> None:
        self.canvas.delete("all")
        width = max(int(self.canvas.winfo_width()), 900)
        height = max(int(self.canvas.winfo_height()), 520)
        margin_left = 70
        margin_right = 30
        plot_top = 45
        plot_gap = 65
        plot_height = max(150, (height - 130) // 2)
        plot_width = width - margin_left - margin_right
        xs = np.asarray([event.front_mileage for event in events], dtype=float)
        force = np.asarray([event.contact_force_norm for event in events], dtype=float) / 1000.0
        patch = np.asarray([event.max_patch_force_z for event in events], dtype=float) / 1000.0
        stages = [event.stage for event in events]
        self._draw_plot(
            xs,
            force,
            stages,
            x=margin_left,
            y=plot_top,
            width=plot_width,
            height=plot_height,
            title="Contact force norm",
            y_label="Contact force norm (kN)",
        )
        self._draw_plot(
            xs,
            patch,
            stages,
            x=margin_left,
            y=plot_top + plot_height + plot_gap,
            width=plot_width,
            height=plot_height,
            title="Max patch vertical force",
            y_label="Max patch vertical force (kN)",
        )

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
    ) -> None:
        x0, x1 = _plot_range(xs)
        y0, y1 = _plot_range(ys)
        x_ticks = _plot_ticks(x0, x1)
        y_ticks = _plot_ticks(y0, y1)
        self.canvas.create_text(x, y - 18, text=title, anchor="w", font=("Arial", 14), fill="#202020")
        self.canvas.create_rectangle(x, y, x + width, y + height, fill="#f8f8f8", outline="#c8c8c8")
        self.canvas.create_line(x, y + height, x + width, y + height, fill="#444444")
        self.canvas.create_line(x, y, x, y + height, fill="#444444")
        for tick in x_ticks:
            px = x + (float(tick) - x0) / (x1 - x0) * width
            self.canvas.create_line(px, y + height, px, y + height + 5, fill="#444444")
            self.canvas.create_text(px, y + height + 18, text=f"{tick:.3g}", anchor="n", font=("Arial", 10), fill="#444444")
        for tick in y_ticks:
            py = y + height - (float(tick) - y0) / (y1 - y0) * height
            self.canvas.create_line(x - 5, py, x, py, fill="#444444")
            self.canvas.create_text(x - 8, py, text=f"{tick:.3g}", anchor="e", font=("Arial", 10), fill="#444444")
        self.canvas.create_text(x + width / 2, y + height + 38, text="Mileage (m)", anchor="n", font=("Arial", 11), fill="#202020")
        self.canvas.create_text(x - 52, y + height / 2, text=y_label, anchor="center", angle=90, font=("Arial", 11), fill="#202020")
        points: list[float] = []
        for x_value, y_value in zip(xs, ys, strict=True):
            px = x + (float(x_value) - x0) / (x1 - x0) * width
            py = y + height - (float(y_value) - y0) / (y1 - y0) * height
            points.extend([px, py])
        if len(points) >= 4:
            self.canvas.create_line(*points, fill="#1f77b4", width=2)
        elif len(points) == 2:
            self.canvas.create_oval(points[0] - 3, points[1] - 3, points[0] + 3, points[1] + 3, fill="#1f77b4", outline="")
        previous_stage = stages[0] if stages else ""
        for index, stage in enumerate(stages):
            if index == 0 or stage == previous_stage:
                previous_stage = stage
                continue
            px = x + (float(xs[index]) - x0) / (x1 - x0) * width
            self.canvas.create_line(px, y, px, y + height, fill="#8a8a8a", dash=(4, 4))
            self.canvas.create_text(px + 6, y + 14, text=stage, anchor="w", font=("Arial", 11), fill="#555555")
            previous_stage = stage


def _run_with_live_window(args: argparse.Namespace, *, cut_freq: float | None) -> int:
    recorder = _ProgressRecorder(every=args.plot_every)
    state: dict[str, Any] = {"done": False, "error": None, "traceback": None, "paths": None}

    def worker() -> None:
        try:
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
                progress_recorder=recorder,
                matlab_baseline_path=args.matlab_baseline,
                abs_tolerance=args.abs_tol,
                rel_tolerance=args.rel_tol,
            )
        except Exception as exc:  # pragma: no cover - exercised manually with GUI failures.
            state["error"] = f"{type(exc).__name__}: {exc}"
            state["traceback"] = traceback.format_exc()
        finally:
            state["done"] = True

    thread = threading.Thread(target=worker, name="sditt-full-case-runner")
    thread.start()
    window = _LiveProgressWindow(recorder, state)
    window.run()
    thread.join()
    if state.get("traceback"):
        print(state["traceback"])
    paths = state.get("paths")
    if paths:
        snapshot_path, report_path = paths
        print(f"wrote {snapshot_path}")
        print(f"wrote {report_path}")
    return 1 if state.get("error") else 0


def _json_dumps(value: Any) -> str:
    return json.dumps(value, indent=2, sort_keys=True, allow_nan=False)


def _read_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run and compare a short SDITT full-case validation.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--matlab-baseline", type=Path, default=None)
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
        matlab_baseline_path=args.matlab_baseline,
        abs_tolerance=args.abs_tol,
        rel_tolerance=args.rel_tol,
    )
    print(f"wrote {snapshot_path}")
    print(f"wrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
