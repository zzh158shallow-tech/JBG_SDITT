from __future__ import annotations

import numpy as np
import pytest

from sditt.training_data.network_a import LABEL_NAMES, METADATA_FIELDS, NetworkADataset
from sditt.training_data.network_a_direct_full_cal import (
    final_preload_anchor_indexes,
    select_full_cal_indexes,
    validate_full_cal_source,
)
from sditt.training_data.network_a_direct_runtime_dagger import (
    _load_accepted_topology_reference,
    _match_accepted_topology,
)
from sditt.training_data.network_a_direct_multiseed_full_cal import (
    _validate_stage_endpoints,
)


def _synthetic_source(cal_steps: int = 20) -> NetworkADataset:
    rows: list[tuple[str, int, str, str]] = []
    for stage, steps in (("Preload", 2), ("Cal", cal_steps)):
        for step in range(1, steps + 1):
            for wheelset in ("FF", "FR", "RF", "RR"):
                for side in ("L", "R"):
                    rows.append((stage, step, wheelset, side))
    n = len(rows)
    features = np.zeros((n, 6), dtype=float)
    patch_count = np.ones((n,), dtype=np.int8)
    patch_mask = np.zeros((n, 2), dtype=bool)
    patch_mask[:, 0] = True
    labels = np.zeros((n, 2, len(LABEL_NAMES)), dtype=float)
    metadata = {name: np.full((n,), "", dtype="<U96") for name in METADATA_FIELDS}
    numeric = {
        "front_mileage_m": float,
        "actual_mileage_m": float,
        "step_index": np.int64,
        "iteration": np.int64,
        "time_s": float,
        "dt_s": float,
        "converged": bool,
        "irregularity_seed": np.int64,
    }
    for name, dtype in numeric.items():
        metadata[name] = np.zeros((n,), dtype=dtype)
    for index, (stage, step, wheelset, side) in enumerate(rows):
        features[index, 0] = 0.0 if side == "L" else 1.0
        metadata["sample_id"][index] = f"sample-{index}"
        metadata["source"][index] = "full_cal_accepted"
        metadata["sample_class"][index] = "normal"
        metadata["group_id"][index] = "full-cal-v11-seed-20260716"
        metadata["front_mileage_m"][index] = (0.0 if stage == "Preload" else 10.0) + step * 0.01
        metadata["actual_mileage_m"][index] = metadata["front_mileage_m"][index]
        metadata["wheelset"][index] = wheelset
        metadata["side"][index] = side
        metadata["stage"][index] = stage
        metadata["step_index"][index] = step
        metadata["iteration"][index] = 2
        metadata["time_s"][index] = step * 1.0e-4
        metadata["dt_s"][index] = 1.0e-4
        metadata["converged"][index] = True
        metadata["irregularity_model"][index] = "china-ballastless"
        metadata["irregularity_seed"][index] = 20260716
    transition = np.flatnonzero(
        (metadata["stage"] == "Cal")
        & (metadata["wheelset"] == "FF")
        & (metadata["side"] == "L")
        & (metadata["step_index"] >= 12)
    )
    patch_count[transition] = 0
    patch_mask[transition, 0] = False
    return NetworkADataset(features, patch_count, patch_mask, labels, metadata)


def test_full_cal_quality_requires_all_eight_lanes_and_monotonic_mileage() -> None:
    quality = validate_full_cal_source(_synthetic_source())
    assert quality.accepted_steps == 20
    assert quality.samples == 160
    assert set(quality.wheel_side_counts.values()) == {20}
    assert quality.front_mileage_end_m > quality.front_mileage_start_m


def test_full_cal_selection_keeps_rollout_and_topology_windows() -> None:
    source = _synthetic_source()
    indexes, steps = select_full_cal_indexes(
        source,
        sampling_stride_steps=10,
        rollout_window_steps=4,
        topology_context_steps=2,
    )
    selected = set(int(value) for value in steps)
    assert {1, 2, 3, 4}.issubset(selected)
    assert {7, 8, 9, 10}.issubset(selected)
    assert {17, 18, 19, 20}.issubset(selected)
    assert {10, 11, 12, 13, 14}.issubset(selected)
    assert np.all(np.asarray(source.metadata["stage"])[indexes] == "Cal")


def test_full_cal_selection_keeps_dedicated_transition_window() -> None:
    source = _synthetic_source()
    _, steps = select_full_cal_indexes(
        source,
        sampling_stride_steps=10,
        rollout_window_steps=4,
        transition_window_steps=7,
        topology_context_steps=2,
    )
    assert set(range(1, 8)).issubset(set(int(value) for value in steps))


def test_full_cal_keeps_final_preload_anchors_for_all_wheel_sides() -> None:
    source = _synthetic_source()
    indexes = final_preload_anchor_indexes(source)
    assert indexes.size == 8
    assert set(np.asarray(source.metadata["step_index"])[indexes]) == {2}
    assert set(zip(
        np.asarray(source.metadata["wheelset"])[indexes],
        np.asarray(source.metadata["side"])[indexes],
        strict=True,
    )) == {
        (wheelset, side)
        for wheelset in ("FF", "FR", "RF", "RR")
        for side in ("L", "R")
    }


def test_multiseed_source_must_reach_every_required_stage_endpoint() -> None:
    source = _synthetic_source()
    _validate_stage_endpoints(source, {"Preload": 0.02, "Cal": 10.2})
    with pytest.raises(ValueError, match="required full-stage endpoint"):
        _validate_stage_endpoints(source, {"Preload": 0.02, "Cal": 10.21})


def test_runtime_dagger_uses_full_accepted_topology_reference(tmp_path) -> None:
    path = tmp_path / "accepted_topology_reference.npz"
    np.savez_compressed(
        path,
        schema=np.array("network-a1-accepted-topology-reference-v1"),
        irregularity_seed=np.array([20260716, 20260716]),
        wheelset=np.array(["FF", "FF"]),
        side=np.array(["L", "L"]),
        actual_mileage_m=np.array([48.0, 48.01]),
        patch_count=np.array([1, 2], dtype=np.int8),
    )
    reference = _load_accepted_topology_reference(path)
    available, matches, count, distance = _match_accepted_topology(
        reference,
        seed=20260716,
        wheelset="FF",
        side="L",
        actual_mileage_m=48.011,
        teacher_patch_count=2,
        tolerance_m=0.02,
    )
    assert available and matches and count == 2
    assert distance is not None and distance < 0.002

    unavailable = _match_accepted_topology(
        reference,
        seed=20260716,
        wheelset="FF",
        side="L",
        actual_mileage_m=48.04,
        teacher_patch_count=2,
        tolerance_m=0.02,
    )
    assert unavailable[:3] == (False, False, None)
