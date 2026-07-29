from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from sditt.config import DEFAULT_OPERATING_CASE
from sditt.simulation import FullCaseProgressEvent, FullDefaultCaseSettings, run_default_full_case_driver
from sditt.simulation.full_case import _stage_end_mileage
from sditt.track import TrackIrregularitySettings
from sditt.training_data.network_a import (
    NetworkACoupledCollector,
    NetworkADataset,
    load_network_a_dataset,
    save_network_a_archive,
)
from sditt.training_data.network_a_direct import (
    NetworkA1DirectDataset,
    build_direct_dataset,
    load_direct_dataset,
    save_direct_dataset,
    validate_direct_dataset,
)
from sditt.training_data.network_a_direct_fresh import concatenate_direct_datasets


FULL_CAL_SCHEMA = "network-a1-direct-full-cal-training-v1"
EXPECTED_WHEEL_SIDES = tuple(
    (wheelset, side)
    for wheelset in ("FF", "FR", "RF", "RR")
    for side in ("L", "R")
)


@dataclass(frozen=True)
class FullCalQuality:
    accepted_steps: int
    samples: int
    front_mileage_start_m: float
    front_mileage_end_m: float
    topology_counts: dict[str, int]
    wheel_side_counts: dict[str, int]


def prepare_full_cal_training_dataset(
    *,
    base_dataset_dir: str | Path,
    output_dir: str | Path,
    repo_root: str | Path | None,
    irregularity_seed: int = 20260716,
    model_path: str | Path | None = None,
    sampling_stride_steps: int = 100,
    rollout_window_steps: int = 8,
    transition_window_steps: int = 0,
    topology_context_steps: int = 4,
    cut_freq: float | None = None,
    source_archive: str | Path | None = None,
    stage_end_mileage: dict[str, float] | None = None,
) -> Path:
    """Prepare accepted-step training data spanning the complete Cal route.

    The raw archive contains every traditional accepted Preload and Cal state.
    The training increment keeps short consecutive windows throughout Cal so a
    single long trajectory does not dominate the established training mixture.
    """

    if sampling_stride_steps <= 0:
        raise ValueError("sampling_stride_steps must be positive")
    if rollout_window_steps <= 0:
        raise ValueError("rollout_window_steps must be positive")
    if transition_window_steps < 0:
        raise ValueError("transition_window_steps cannot be negative")
    if topology_context_steps < 0:
        raise ValueError("topology_context_steps cannot be negative")

    base_dir = Path(base_dataset_dir).resolve()
    destination = Path(output_dir).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    raw_path = destination / "traditional_accepted_steps.npz"
    selected_path = destination / "full_cal_selected.npz"

    if source_archive is None:
        source = _collect_full_cal_source(
            repo_root=repo_root,
            irregularity_seed=irregularity_seed,
            cut_freq=cut_freq,
            stage_end_mileage=stage_end_mileage,
        )
        save_network_a_archive(raw_path, source)
    else:
        source_value = Path(source_archive).resolve()
        source = load_network_a_dataset(source_value, split="all", dtype=np.float64)
        if source_value != raw_path:
            save_network_a_archive(raw_path, source)

    quality = validate_full_cal_source(source)
    selected_indexes, selected_steps = select_full_cal_indexes(
        source,
        sampling_stride_steps=sampling_stride_steps,
        rollout_window_steps=rollout_window_steps,
        transition_window_steps=transition_window_steps,
        topology_context_steps=topology_context_steps,
    )
    preload_anchor_indexes = final_preload_anchor_indexes(source)
    training_indexes = np.concatenate((preload_anchor_indexes, selected_indexes))
    direct = build_direct_dataset(source, repo_root=repo_root, indexes=training_indexes)
    _validate_replay_topology(source, training_indexes, direct)
    save_direct_dataset(selected_path, direct)

    train = load_direct_dataset(base_dir / "train.npz")
    validation = load_direct_dataset(base_dir / "validation.npz")
    test = load_direct_dataset(base_dir / "test.npz")
    augmented = concatenate_direct_datasets((train, direct))
    validate_direct_dataset(augmented)
    split_isolation = _split_isolation(augmented, validation, test)
    if any(split_isolation.values()):
        raise ValueError(f"full-Cal group leakage detected: {split_isolation}")

    save_direct_dataset(destination / "train.npz", augmented)
    save_direct_dataset(destination / "validation.npz", validation)
    save_direct_dataset(destination / "test.npz", test)

    model = None if model_path is None else Path(model_path).resolve()
    manifest = {
        "schema": FULL_CAL_SCHEMA,
        "decision": (
            "traditional accepted-step full-Cal supervision; V11 is provenance/audit only; "
            "nonlinear retries and failed V11 iterations are excluded"
        ),
        "base_dataset_dir": str(base_dir),
        "repo_root": str(Path(repo_root).resolve()) if repo_root is not None else None,
        "irregularity": {"model": "china-ballastless", "seed": int(irregularity_seed)},
        "mileage_mode": "MATLAB endpoints" if stage_end_mileage is None else "explicit test endpoints",
        "full_modal_system": cut_freq is None,
        "cut_freq_hz": cut_freq,
        "v11_model": None if model is None else {
            "path": str(model),
            "sha256": _sha256(model),
        },
        "raw_accepted_source": {
            "path": raw_path.name,
            "sha256": _sha256(raw_path),
            "preload_and_cal_samples": len(source),
            "cal_accepted_steps": quality.accepted_steps,
            "cal_samples": quality.samples,
            "front_mileage_start_m": quality.front_mileage_start_m,
            "front_mileage_end_m": quality.front_mileage_end_m,
            "topology_counts": quality.topology_counts,
            "wheel_side_counts": quality.wheel_side_counts,
        },
        "selection": {
            "policy": (
                "first/last rollout windows, every stride endpoint with its preceding rollout "
                "window, and topology-change context windows"
            ),
            "sampling_stride_steps": int(sampling_stride_steps),
            "rollout_window_steps": int(rollout_window_steps),
            "topology_context_steps": int(topology_context_steps),
            "selected_accepted_steps": int(selected_steps.size),
            "preload_anchor_samples": int(preload_anchor_indexes.size),
            "selected_cal_samples": int(selected_indexes.size),
            "selected_samples": len(direct),
            "selected_topology_counts": {
                str(value): int(np.sum(direct.patch_count == value)) for value in range(3)
            },
            "accepted_step_label": True,
            "history_rule": (
                "the final traditional Preload accepted step is retained as eight explicit anchor "
                "rows; Cal-1 links to those anchors; later rows use the immediately preceding "
                "accepted geometry; previous_row links are retained inside each selected "
                "consecutive window"
            ),
        },
        "training_dataset": {
            "base_train_samples": len(train),
            "full_cal_increment_samples": len(direct),
            "augmented_train_samples": len(augmented),
            "validation_samples": len(validation),
            "test_samples": len(test),
            "split_group_overlap": split_isolation,
        },
        "quality": {
            "passed": True,
            "finite_features_and_labels": True,
            "strictly_increasing_accepted_step_mileage": True,
            "complete_wheel_side_coverage": True,
            "traditional_replay_topology_match": True,
        },
        "files": {
            "raw_accepted_source": raw_path.name,
            "selected_increment": selected_path.name,
            "train": "train.npz",
            "validation": "validation.npz",
            "test": "test.npz",
        },
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def select_full_cal_indexes(
    source: NetworkADataset,
    *,
    sampling_stride_steps: int,
    rollout_window_steps: int,
    transition_window_steps: int = 0,
    topology_context_steps: int,
) -> tuple[np.ndarray, np.ndarray]:
    stage = np.asarray(source.metadata["stage"], dtype=str)
    cal_indexes = np.flatnonzero(stage == "Cal")
    if not cal_indexes.size:
        raise ValueError("source dataset contains no Cal rows")
    step = np.asarray(source.metadata["step_index"], dtype=int)
    cal_steps = np.unique(step[cal_indexes])
    if cal_steps[0] != 1 or not np.array_equal(cal_steps, np.arange(1, cal_steps[-1] + 1)):
        raise ValueError("Cal accepted step indexes are not contiguous from 1")

    selected_steps: set[int] = set()

    def add_window(end_step: int, *, before: int, after: int = 0) -> None:
        start = max(1, int(end_step) - int(before) + 1)
        stop = min(int(cal_steps[-1]), int(end_step) + int(after))
        selected_steps.update(range(start, stop + 1))

    initial_window_steps = max(int(rollout_window_steps), int(transition_window_steps))
    add_window(initial_window_steps, before=initial_window_steps)
    add_window(int(cal_steps[-1]), before=rollout_window_steps)
    for anchor in range(sampling_stride_steps, int(cal_steps[-1]) + 1, sampling_stride_steps):
        add_window(anchor, before=rollout_window_steps)

    wheelset = np.asarray(source.metadata["wheelset"], dtype=str)
    side = np.asarray(source.metadata["side"], dtype=str)
    for wheel, rail_side in EXPECTED_WHEEL_SIDES:
        lane = cal_indexes[(wheelset[cal_indexes] == wheel) & (side[cal_indexes] == rail_side)]
        order = lane[np.argsort(step[lane], kind="stable")]
        changes = np.flatnonzero(np.diff(source.patch_count[order]) != 0) + 1
        for position in changes:
            transition_step = int(step[order[position]])
            add_window(
                transition_step + topology_context_steps,
                before=rollout_window_steps + topology_context_steps,
                after=topology_context_steps,
            )

    chosen_steps = np.asarray(sorted(selected_steps), dtype=int)
    indexes = cal_indexes[np.isin(step[cal_indexes], chosen_steps)]
    return indexes, chosen_steps


def validate_full_cal_source(source: NetworkADataset) -> FullCalQuality:
    stage = np.asarray(source.metadata["stage"], dtype=str)
    if not {"Preload", "Cal"}.issubset(set(stage)):
        raise ValueError("full-Cal source must contain both Preload and Cal accepted states")
    if not np.isfinite(source.features).all():
        raise ValueError("full-Cal features contain NaN or Inf")
    if not np.isfinite(source.labels[source.patch_mask]).all():
        raise ValueError("full-Cal active labels contain NaN or Inf")

    cal = np.flatnonzero(stage == "Cal")
    step = np.asarray(source.metadata["step_index"], dtype=int)[cal]
    mileage = np.asarray(source.metadata["front_mileage_m"], dtype=float)[cal]
    wheelset = np.asarray(source.metadata["wheelset"], dtype=str)[cal]
    side = np.asarray(source.metadata["side"], dtype=str)[cal]
    unique_steps, first = np.unique(step, return_index=True)
    if unique_steps[0] != 1 or not np.array_equal(unique_steps, np.arange(1, unique_steps[-1] + 1)):
        raise ValueError("Cal accepted step indexes are incomplete")
    step_mileage = mileage[first]
    if not np.isfinite(step_mileage).all() or np.any(np.diff(step_mileage) <= 0.0):
        raise ValueError("Cal accepted-step mileage is not finite and strictly increasing")

    lane_counts = {
        f"{wheel}-{rail_side}": int(np.sum((wheelset == wheel) & (side == rail_side)))
        for wheel, rail_side in EXPECTED_WHEEL_SIDES
    }
    if set(lane_counts.values()) != {unique_steps.size}:
        raise ValueError(f"incomplete wheel/side coverage in full-Cal source: {lane_counts}")
    if len(cal) != unique_steps.size * len(EXPECTED_WHEEL_SIDES):
        raise ValueError("full-Cal source does not have exactly eight wheel/side rows per accepted step")

    return FullCalQuality(
        accepted_steps=int(unique_steps.size),
        samples=int(len(cal)),
        front_mileage_start_m=float(step_mileage[0]),
        front_mileage_end_m=float(step_mileage[-1]),
        topology_counts={
            str(value): int(np.sum(source.patch_count[cal] == value)) for value in range(3)
        },
        wheel_side_counts=lane_counts,
    )


def final_preload_anchor_indexes(source: NetworkADataset) -> np.ndarray:
    stage = np.asarray(source.metadata["stage"], dtype=str)
    step = np.asarray(source.metadata["step_index"], dtype=int)
    wheelset = np.asarray(source.metadata["wheelset"], dtype=str)
    side = np.asarray(source.metadata["side"], dtype=str)
    preload = np.flatnonzero(stage == "Preload")
    if not preload.size:
        raise ValueError("source dataset contains no Preload rows")
    final_step = int(np.max(step[preload]))
    anchors = preload[step[preload] == final_step]
    combinations = {(str(wheelset[row]), str(side[row])) for row in anchors}
    if combinations != set(EXPECTED_WHEEL_SIDES) or anchors.size != len(EXPECTED_WHEEL_SIDES):
        raise ValueError("final Preload accepted step does not contain exactly eight wheel/side anchors")
    return anchors


def _collect_full_cal_source(
    *,
    repo_root: str | Path | None,
    irregularity_seed: int,
    cut_freq: float | None,
    stage_end_mileage: dict[str, float] | None,
) -> NetworkADataset:
    collector = NetworkACoupledCollector(
        group_id=f"full-cal-v11-seed-{int(irregularity_seed)}",
        source="full_cal_accepted",
        irregularity_model="china-ballastless",
        irregularity_seed=int(irregularity_seed),
    )
    last_reported: dict[str, int] = {}

    def progress(event: FullCaseProgressEvent) -> None:
        previous = last_reported.get(event.stage, 0)
        if event.step_index == 1 or event.step_index - previous >= 250:
            last_reported[event.stage] = int(event.step_index)
            print(
                f"{event.stage}: accepted {event.step_index}/{event.n_steps}, "
                f"front_mileage={event.front_mileage:.6f} m",
                flush=True,
            )

    settings = FullDefaultCaseSettings(
        cut_freq=cut_freq,
        n_steps_per_stage=1,
        stage_end_mileage=stage_end_mileage,
        use_matlab_mileage_endpoints=stage_end_mileage is None,
        use_sparse=True,
        history_retention_steps=2,
        progress_callback=progress,
        accepted_contact_callback=collector,
        track_irregularity=TrackIrregularitySettings(
            model="china-ballastless",
            seed=int(irregularity_seed),
        ),
        contact_geometry_mode="traditional",
        network_a_force_mode="traditional",
    )
    run = run_default_full_case_driver(
        repo_root=repo_root,
        settings=settings,
        operating_case=DEFAULT_OPERATING_CASE,
    )
    for stage_result in run.stages:
        expected_end = _stage_end_mileage(run.preparation, stage_result.stage)
        if expected_end is None:
            continue
        actual_end = float(stage_result.history.rail_response[-1].front_mileage)
        if not np.isclose(actual_end, expected_end, rtol=0.0, atol=1.0e-9):
            raise RuntimeError(
                f"{stage_result.stage} accepted trajectory stopped at {actual_end:.12f} m "
                f"instead of endpoint {expected_end:.12f} m"
            )
    if collector.accumulator.rejected_overflow:
        raise RuntimeError(
            "traditional full-Cal collection produced contact states with more than two patches"
        )
    return collector.accumulator.dataset()


def _validate_replay_topology(
    source: NetworkADataset,
    selected_indexes: np.ndarray,
    direct: NetworkA1DirectDataset,
) -> None:
    expected = np.asarray(source.patch_count[selected_indexes], dtype=np.int8)
    if not np.array_equal(expected, direct.patch_count):
        mismatch = int(np.sum(expected != direct.patch_count))
        raise ValueError(f"traditional direct replay changed topology for {mismatch} selected rows")


def _split_isolation(
    train: NetworkA1DirectDataset,
    validation: NetworkA1DirectDataset,
    test: NetworkA1DirectDataset,
) -> dict[str, list[str]]:
    groups = {
        name: set(np.asarray(dataset.metadata["group_id"], dtype=str))
        for name, dataset in {"train": train, "validation": validation, "test": test}.items()
    }
    return {
        "train_validation": sorted(groups["train"] & groups["validation"]),
        "train_test": sorted(groups["train"] & groups["test"]),
        "validation_test": sorted(groups["validation"] & groups["test"]),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prepare full-Cal accepted-step training data for WRCP-Net A1 Direct."
    )
    parser.add_argument(
        "--base-dataset-dir",
        type=Path,
        default=Path("outputs/network_a_direct_set_v10_v12_runtime_dagger"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("outputs/network_a_direct_set_v11_full_cal_seed20260716"),
    )
    parser.add_argument("--repo-root", type=Path, default=Path(".."))
    parser.add_argument("--irregularity-seed", type=int, default=20260716)
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("outputs/wrcp_net_a1_direct_stageaware_v11_preload200_projection/model.npz"),
    )
    parser.add_argument("--sampling-stride-steps", type=int, default=100)
    parser.add_argument("--rollout-window-steps", type=int, default=8)
    parser.add_argument(
        "--transition-window-steps",
        type=int,
        default=0,
        help="Keep the first N accepted Cal steps in addition to sparse rollout windows.",
    )
    parser.add_argument("--topology-context-steps", type=int, default=4)
    parser.add_argument("--cut-freq", type=float, default=None)
    parser.add_argument(
        "--source-archive",
        type=Path,
        default=None,
        help="Reuse an existing raw accepted-step archive instead of rerunning the full case.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    manifest = prepare_full_cal_training_dataset(
        base_dataset_dir=args.base_dataset_dir,
        output_dir=args.output_dir,
        repo_root=args.repo_root,
        irregularity_seed=args.irregularity_seed,
        model_path=args.model,
        sampling_stride_steps=args.sampling_stride_steps,
        rollout_window_steps=args.rollout_window_steps,
        transition_window_steps=args.transition_window_steps,
        topology_context_steps=args.topology_context_steps,
        cut_freq=args.cut_freq,
        source_archive=args.source_archive,
    )
    print(f"wrote {manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
