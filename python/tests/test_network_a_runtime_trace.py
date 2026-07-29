from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from sditt.contact.geometry import BoundaryExtrema, MultiPointContactGeometry
from sditt.models.network_a_runtime_trace import NetworkARuntimeTraceWriter


def _empty_geometry() -> MultiPointContactGeometry:
    empty2 = np.empty((0, 2), dtype=float)
    empty3 = np.empty((0, 3), dtype=float)
    empty4 = np.empty((0, 4), dtype=float)
    return MultiPointContactGeometry(
        has_contact=False,
        elastic_penetration=empty2,
        wheel_interp=empty3,
        rail_interp=empty2,
        contact_angles=np.empty((0,), dtype=float),
        wheel_profile_lateral=np.empty((0,), dtype=float),
        boundaries=BoundaryExtrema(empty4, empty4, empty4, empty4),
        patches=(),
    )


def _record(
    writer: NetworkARuntimeTraceWriter,
    *,
    step: int,
    iteration: int,
    time_s: float,
    dt_s: float,
    mileage: float,
    probability: tuple[float, float, float] = (0.01, 0.98, 0.01),
    predicted_patch_count: int = 0,
) -> None:
    writer.record(
        context={
            "stage": "Cal",
            "step_index": step,
            "iteration": iteration,
            "time_s": time_s,
            "dt_s": dt_s,
            "front_mileage_m": mileage,
            "actual_mileage_m": mileage,
            "wheelset": "FF",
            "side": "L",
        },
        features=np.zeros((6,), dtype=np.float32),
        class_probability=np.asarray(probability, dtype=np.float32),
        predicted_patch_count=predicted_patch_count,
        geometry=_empty_geometry(),
        gap_field_m=np.array([-1.0e-3, 1.0e-4], dtype=np.float32),
    )


def _writer(tmp_path: Path, **kwargs: object) -> NetworkARuntimeTraceWriter:
    model_path = tmp_path / "model.npz"
    model_path.write_bytes(b"model")
    return NetworkARuntimeTraceWriter(
        tmp_path / "trace",
        model_path=model_path,
        shard_size=64,
        **kwargs,
    )


def _load_trace(trace_dir: Path) -> dict[str, np.ndarray]:
    loaded: dict[str, list[np.ndarray]] = {}
    for path in sorted(trace_dir.glob("trace_*.npz")):
        with np.load(path, allow_pickle=False) as archive:
            for name in archive.files:
                loaded.setdefault(name, []).append(archive[name].copy())
    return {name: np.concatenate(values, axis=0) for name, values in loaded.items()}


def test_selective_trace_keeps_only_accepted_final_iteration_by_default(tmp_path: Path) -> None:
    writer = _writer(tmp_path, mode="selective", sample_interval_m=None)
    _record(writer, step=1, iteration=1, time_s=1.0e-4, dt_s=1.0e-4, mileage=48.0)
    _record(writer, step=1, iteration=2, time_s=1.0e-4, dt_s=1.0e-4, mileage=48.0)
    writer.mark_accepted(
        stage="Cal",
        step_index=1,
        iteration=2,
        time_s=1.0e-4,
        dt_s=1.0e-4,
        front_mileage_m=48.0,
    )
    writer.close()

    trace = _load_trace(tmp_path / "trace")
    assert trace["iteration"].tolist() == [2]
    assert trace["selection_reason"].tolist() == ["accepted_final"]
    manifest = json.loads((tmp_path / "trace" / "manifest.json").read_text())
    assert manifest["selection"]["mode"] == "selective"
    assert manifest["observed_samples"] == 2
    assert manifest["completed_samples"] == 1


def test_selective_trace_keeps_all_candidates_for_retried_step(tmp_path: Path) -> None:
    writer = _writer(tmp_path, mode="selective", sample_interval_m=None)
    _record(writer, step=1, iteration=11, time_s=1.0e-4, dt_s=1.0e-4, mileage=48.0)
    _record(writer, step=1, iteration=2, time_s=5.0e-5, dt_s=5.0e-5, mileage=47.995)
    writer.mark_accepted(
        stage="Cal",
        step_index=1,
        iteration=2,
        time_s=5.0e-5,
        dt_s=5.0e-5,
        retry_count=1,
        front_mileage_m=47.995,
    )
    writer.close()

    trace = _load_trace(tmp_path / "trace")
    assert trace["iteration"].tolist() == [11, 2]
    assert trace["selection_reason"].tolist() == ["retry", "accepted_final+retry"]
    accepted = json.loads((tmp_path / "trace" / "accepted_steps.jsonl").read_text())
    assert accepted["retry_count"] == 1
    assert accepted["retained_reasons"] == ["retry"]


def test_selective_trace_keeps_low_confidence_multi_patch_and_mileage_samples(
    tmp_path: Path,
) -> None:
    writer = _writer(
        tmp_path,
        mode="selective",
        low_confidence_threshold=0.95,
        sample_interval_m=1.0,
    )
    _record(
        writer,
        step=1,
        iteration=1,
        time_s=1.0e-4,
        dt_s=1.0e-4,
        mileage=48.2,
        probability=(0.2, 0.6, 0.2),
    )
    _record(writer, step=1, iteration=2, time_s=1.0e-4, dt_s=1.0e-4, mileage=48.2)
    writer.mark_accepted(
        stage="Cal",
        step_index=1,
        iteration=2,
        time_s=1.0e-4,
        dt_s=1.0e-4,
        front_mileage_m=48.2,
    )
    _record(
        writer,
        step=2,
        iteration=1,
        time_s=2.0e-4,
        dt_s=1.0e-4,
        mileage=48.3,
        predicted_patch_count=2,
    )
    _record(writer, step=2, iteration=2, time_s=2.0e-4, dt_s=1.0e-4, mileage=48.3)
    writer.mark_accepted(
        stage="Cal",
        step_index=2,
        iteration=2,
        time_s=2.0e-4,
        dt_s=1.0e-4,
        front_mileage_m=48.3,
    )
    _record(writer, step=3, iteration=1, time_s=3.0e-4, dt_s=1.0e-4, mileage=49.0)
    _record(writer, step=3, iteration=2, time_s=3.0e-4, dt_s=1.0e-4, mileage=49.0)
    writer.mark_accepted(
        stage="Cal",
        step_index=3,
        iteration=2,
        time_s=3.0e-4,
        dt_s=1.0e-4,
        front_mileage_m=49.0,
    )
    writer.close()

    trace = _load_trace(tmp_path / "trace")
    assert len(trace["iteration"]) == 6
    reasons = trace["selection_reason"].tolist()
    assert "low_confidence+mileage_sample" in reasons
    assert "multi_patch" in reasons
    assert "mileage_sample" in reasons


def test_full_trace_keeps_every_candidate_iteration(tmp_path: Path) -> None:
    writer = _writer(tmp_path, mode="full")
    _record(writer, step=1, iteration=1, time_s=1.0e-4, dt_s=1.0e-4, mileage=48.0)
    _record(writer, step=1, iteration=2, time_s=1.0e-4, dt_s=1.0e-4, mileage=48.0)
    writer.mark_accepted(
        stage="Cal",
        step_index=1,
        iteration=2,
        time_s=1.0e-4,
        dt_s=1.0e-4,
        front_mileage_m=48.0,
    )
    writer.close()

    trace = _load_trace(tmp_path / "trace")
    assert trace["iteration"].tolist() == [1, 2]
    assert trace["selection_reason"].tolist() == ["full_iteration", "full_iteration"]
