from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sditt.contact.geometry import DirectContactGeometry, DirectContactPatch, WheelPose2D
from sditt.models.network_b_features import (
    DIRECT_FEATURE_NAMES,
    FEATURE_NAMES,
    direct_network_b_patch_shape_features,
    network_b_patch_shape_features,
)
from sditt.models.wrcp_net_a1 import WRCPNetA1
from sditt.models.wrcp_net_b import (
    DIRECT_HERTZ_FIXED_MODEL_SCHEMA,
    DIRECT_MODEL_SCHEMA,
    DIRECT_RESIDUAL_MODEL_SCHEMA,
    WRCPNetB,
    load_wrcp_net_b,
    save_wrcp_net_b,
    train_wrcp_net_b,
)
from sditt.simulation import FullCaseAcceptedContactSnapshot
from sditt.training_data.network_b import (
    NetworkBAcceptedStepCollector,
    NetworkBDataset,
    TARGET_NAMES,
    load_network_b_dataset,
    save_network_b_dataset,
    validate_network_b_dataset,
)


def _model() -> WRCPNetB:
    network = WRCPNetA1(input_size=len(FEATURE_NAMES), hidden_sizes=(8,), output_size=len(TARGET_NAMES), seed=3)
    for weight in network.weights:
        weight.fill(0.0)
    for bias in network.biases:
        bias.fill(0.0)
    return WRCPNetB(
        network=network,
        feature_mean=np.zeros(len(FEATURE_NAMES)),
        feature_scale=np.ones(len(FEATURE_NAMES)),
        target_mean=np.array([1000.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0]),
        target_scale=np.ones(len(TARGET_NAMES)),
        moment_scale_m=0.46,
        friction_limit=0.55,
        feature_clip=6.0,
    )


def _dataset(seed: int) -> NetworkBDataset:
    features = np.zeros((4, len(FEATURE_NAMES)), dtype=float)
    targets = np.tile(np.array([1.0e5, 100.0, 200.0, 300.0, 0.1, 0.2, 0.3]), (4, 1))
    return NetworkBDataset(
        features=features,
        targets=targets,
        irregularity_seed=np.full(4, seed, dtype=np.int64),
        front_mileage_m=np.arange(4, dtype=float),
        actual_mileage_m=np.arange(4, dtype=float),
        step_index=np.arange(4, dtype=np.int64),
        time_s=np.arange(4, dtype=float) * 1.0e-4,
        dt_s=np.full(4, 1.0e-4),
        iterations=np.full(4, 2, dtype=np.int64),
        retry_count=np.zeros(4, dtype=np.int64),
        wheelset=np.full(4, "FF"),
        side=np.full(4, "L"),
        patch_index=np.zeros(4, dtype=np.int64),
        patch_count=np.ones(4, dtype=np.int64),
        accepted_step_label=np.ones(4, dtype=bool),
        teacher_match_distance_m=np.zeros(4),
    )


def _full_coverage_dataset(seed: int) -> NetworkBDataset:
    wheelset = np.repeat(np.array(["FF", "FR", "RF", "RR"]), 2)
    side = np.tile(np.array(["L", "R"]), 4)
    count = wheelset.size
    return NetworkBDataset(
        features=np.zeros((count, len(FEATURE_NAMES)), dtype=float),
        targets=np.tile(
            np.array([1.0e5, 100.0, 200.0, 300.0, 0.1, 0.2, 0.3]),
            (count, 1),
        ),
        irregularity_seed=np.full(count, seed, dtype=np.int64),
        front_mileage_m=np.full(count, 50.0),
        actual_mileage_m=np.full(count, 50.0),
        step_index=np.ones(count, dtype=np.int64),
        time_s=np.full(count, 1.0e-4),
        dt_s=np.full(count, 1.0e-4),
        iterations=np.full(count, 2, dtype=np.int64),
        retry_count=np.zeros(count, dtype=np.int64),
        wheelset=wheelset,
        side=side,
        patch_index=np.zeros(count, dtype=np.int64),
        patch_count=np.ones(count, dtype=np.int64),
        accepted_step_label=np.ones(count, dtype=bool),
        teacher_match_distance_m=np.zeros(count),
    )


def test_network_b_round_trip_enforces_force_bound_and_distribution_guard(tmp_path) -> None:
    model = _model()
    features = np.zeros((2, len(FEATURE_NAMES)), dtype=float)
    prediction = model.predict(features)
    assert prediction.shape == (2, len(TARGET_NAMES))
    assert np.all(prediction[:, 0] >= 0.0)
    assert np.all(np.linalg.norm(prediction[:, 1:4], axis=1) <= 0.55 * prediction[:, 0] + 1.0e-9)
    assert model.in_distribution(features, threshold=4.0).all()
    features[1, 0] = 5.0
    assert model.in_distribution(features, threshold=4.0).tolist() == [True, False]

    path = tmp_path / "model.npz"
    save_wrcp_net_b(path, model)
    loaded = load_wrcp_net_b(path)
    np.testing.assert_allclose(loaded.predict(features), model.predict(features))


def test_direct_network_b_v2_artifact_round_trip(tmp_path) -> None:
    network = WRCPNetA1(
        input_size=len(DIRECT_FEATURE_NAMES),
        hidden_sizes=(8,),
        output_size=len(TARGET_NAMES),
        seed=7,
    )
    model = WRCPNetB(
        network=network,
        feature_mean=np.zeros(len(DIRECT_FEATURE_NAMES)),
        feature_scale=np.ones(len(DIRECT_FEATURE_NAMES)),
        target_mean=np.zeros(len(TARGET_NAMES)),
        target_scale=np.ones(len(TARGET_NAMES)),
        moment_scale_m=0.46,
        friction_limit=0.55,
        feature_clip=6.0,
        feature_names=DIRECT_FEATURE_NAMES,
        model_schema=DIRECT_MODEL_SCHEMA,
    )
    path = tmp_path / "direct_model.npz"
    save_wrcp_net_b(path, model)
    loaded = load_wrcp_net_b(path)
    assert loaded.model_schema == DIRECT_MODEL_SCHEMA
    assert loaded.feature_names == DIRECT_FEATURE_NAMES
    assert loaded.predict(np.zeros((2, len(DIRECT_FEATURE_NAMES)))).shape == (2, 7)


def test_direct_hertz_residual_model_reproduces_physical_baseline(tmp_path) -> None:
    network = WRCPNetA1(
        input_size=len(DIRECT_FEATURE_NAMES),
        hidden_sizes=(8,),
        output_size=len(TARGET_NAMES),
        seed=8,
    )
    for weight in network.weights:
        weight.fill(0.0)
    for bias in network.biases:
        bias.fill(0.0)
    model = WRCPNetB(
        network=network,
        feature_mean=np.zeros(len(DIRECT_FEATURE_NAMES)),
        feature_scale=np.ones(len(DIRECT_FEATURE_NAMES)),
        target_mean=np.zeros(len(TARGET_NAMES)),
        target_scale=np.ones(len(TARGET_NAMES)),
        moment_scale_m=0.46,
        friction_limit=0.40,
        feature_clip=6.0,
        feature_names=DIRECT_FEATURE_NAMES,
        model_schema=DIRECT_RESIDUAL_MODEL_SCHEMA,
        normal_force_mode="hertz_residual",
        ood_feature_scale=np.full(len(DIRECT_FEATURE_NAMES), 2.0),
    )
    features = np.zeros((2, len(DIRECT_FEATURE_NAMES)))
    expected_normal_force = 1.0e5
    penetration = 1.0e-4
    permeability = penetration / expected_normal_force ** (2.0 / 3.0)
    features[
        :,
        DIRECT_FEATURE_NAMES.index("corrected_normal_penetration_m"),
    ] = penetration
    features[
        :,
        DIRECT_FEATURE_NAMES.index("elastic_permeability_m_per_N_2over3"),
    ] = permeability
    prediction = model.predict(features)
    np.testing.assert_allclose(prediction[:, 0], expected_normal_force)

    path = tmp_path / "direct_residual_model.npz"
    save_wrcp_net_b(path, model)
    loaded = load_wrcp_net_b(path)
    assert loaded.model_schema == DIRECT_RESIDUAL_MODEL_SCHEMA
    assert loaded.normal_force_mode == "hertz_residual"
    np.testing.assert_allclose(loaded.ood_feature_scale, 2.0)
    np.testing.assert_allclose(loaded.predict(features), prediction)

    fixed_model = WRCPNetB(
        network=network,
        feature_mean=model.feature_mean,
        feature_scale=model.feature_scale,
        target_mean=model.target_mean,
        target_scale=model.target_scale,
        moment_scale_m=model.moment_scale_m,
        friction_limit=model.friction_limit,
        feature_clip=model.feature_clip,
        feature_names=DIRECT_FEATURE_NAMES,
        model_schema=DIRECT_HERTZ_FIXED_MODEL_SCHEMA,
        normal_force_mode="hertz_fixed",
        ood_feature_scale=model.ood_feature_scale,
    )
    fixed_path = tmp_path / "direct_hertz_fixed_model.npz"
    save_wrcp_net_b(fixed_path, fixed_model)
    fixed_loaded = load_wrcp_net_b(fixed_path)
    assert fixed_loaded.model_schema == DIRECT_HERTZ_FIXED_MODEL_SCHEMA
    assert fixed_loaded.normal_force_mode == "hertz_fixed"
    np.testing.assert_allclose(
        fixed_loaded.predict(features)[:, 0],
        expected_normal_force,
    )


def test_network_b_training_rejects_seed_leakage(tmp_path) -> None:
    with pytest.raises(ValueError, match="disjoint irregularity seeds"):
        train_wrcp_net_b(
            _dataset(1),
            _dataset(1),
            _dataset(2),
            output_dir=tmp_path,
            epochs=1,
            patience=1,
        )


def test_network_b_training_without_test_keeps_blind_split_sealed(tmp_path) -> None:
    result = train_wrcp_net_b(
        _dataset(1),
        _dataset(2),
        output_dir=tmp_path,
        hidden_sizes=(4,),
        epochs=1,
        patience=1,
    )
    assert "test" not in result.metrics
    assert "test_seeds" not in result.metrics["split"]
    assert result.metrics["split"]["train_seeds"] == [1]
    assert result.metrics["split"]["validation_seeds"] == [2]
    stored = json.loads(result.metrics_path.read_text(encoding="utf-8"))
    assert "test" not in stored


def test_network_b_training_cli_uses_separate_directories(tmp_path) -> None:
    train_dir = tmp_path / "train"
    validation_dir = tmp_path / "validation"
    output_dir = tmp_path / "model"
    train_dir.mkdir()
    validation_dir.mkdir()
    save_network_b_dataset(train_dir / "seed_1.npz", _dataset(1))
    save_network_b_dataset(validation_dir / "seed_2.npz", _dataset(2))
    script = Path(__file__).resolve().parents[1] / "scripts" / "train_network_b.py"
    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--train-dataset-dir",
            str(train_dir),
            "--validation-dataset-dir",
            str(validation_dir),
            "--train-seeds",
            "1",
            "--validation-seeds",
            "2",
            "--epochs",
            "1",
            "--hidden-sizes",
            "4",
            "--max-train-teacher-match-um",
            "50",
            "--output-dir",
            str(output_dir),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    metrics = json.loads((output_dir / "metrics.json").read_text(encoding="utf-8"))
    assert "test" not in metrics
    assert metrics["split"]["train_seeds"] == [1]
    assert metrics["split"]["validation_seeds"] == [2]
    assert metrics["training_metadata"]["train_dataset_dir"] == str(train_dir)
    assert metrics["training_metadata"]["validation_dataset_dir"] == str(validation_dir)


def test_network_b_blind_evaluator_checks_split_and_refuses_overwrite(tmp_path) -> None:
    model_path = tmp_path / "model.npz"
    save_wrcp_net_b(model_path, _model())
    model_sha256 = hashlib.sha256(model_path.read_bytes()).hexdigest()
    metrics_path = tmp_path / "metrics.json"
    metrics_path.write_text(
        json.dumps(
            {
                "model_sha256": model_sha256,
                "split": {
                    "train_seeds": [1],
                    "validation_seeds": [2],
                },
            }
        ),
        encoding="utf-8",
    )
    dataset_dir = tmp_path / "test_data"
    dataset_dir.mkdir()
    save_network_b_dataset(
        dataset_dir / "seed_3.npz",
        _full_coverage_dataset(3),
    )
    output_path = tmp_path / "test_metrics.json"
    script = Path(__file__).resolve().parents[1] / "scripts" / "evaluate_network_b.py"
    command = [
        sys.executable,
        str(script),
        "--model",
        str(model_path),
        "--training-metrics",
        str(metrics_path),
        "--dataset-dir",
        str(dataset_dir),
        "--test-seeds",
        "3",
        "--output",
        str(output_path),
    ]
    first = subprocess.run(command, check=False, capture_output=True, text=True)
    assert first.returncode == 0, first.stderr
    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert report["test"]["seeds"] == [3]
    assert report["test"]["rows"] == 8
    assert report["physical_constraints"]["finite"]

    second = subprocess.run(command, check=False, capture_output=True, text=True)
    assert second.returncode != 0
    assert "test report already exists" in second.stderr

    overlap = subprocess.run(
        [*command[:-3], "1", *command[-2:]],
        check=False,
        capture_output=True,
        text=True,
    )
    assert overlap.returncode != 0
    assert "overlap training or validation seeds" in overlap.stderr


def test_network_b_dataset_v4_round_trip_preserves_accepted_step_metadata(tmp_path) -> None:
    dataset = _dataset(17)
    path = tmp_path / "seed_17.npz"
    save_network_b_dataset(path, dataset)
    loaded = load_network_b_dataset(path)
    np.testing.assert_allclose(loaded.time_s, dataset.time_s)
    np.testing.assert_allclose(loaded.dt_s, dataset.dt_s)
    np.testing.assert_array_equal(loaded.iterations, dataset.iterations)
    np.testing.assert_array_equal(loaded.retry_count, dataset.retry_count)
    np.testing.assert_array_equal(loaded.accepted_step_label, dataset.accepted_step_label)
    report = validate_network_b_dataset(loaded, expected_seed=17)
    assert report["valid"]
    assert report["unique_patch_keys"] == len(dataset)


def test_direct_collector_flushes_resumable_shards_and_deduplicates_steps(tmp_path) -> None:
    patch = DirectContactPatch(
        start_y=-2.0e-3,
        end_y=3.0e-3,
        peak_wheel_point=np.array([0.0, 0.0, 0.0]),
        peak_rail_point=np.array([0.0, 0.0]),
        corrected_wheel_point=np.array([0.0, 0.0, 0.0]),
        corrected_rail_point=np.array([0.0, 0.0]),
        wheel_profile_lateral=0.0,
        peak_vertical_penetration=2.0e-4,
        peak_normal_penetration=2.0e-4,
        peak_contact_angle=0.0,
        corrected_vertical_penetration=1.0e-4,
        corrected_normal_penetration=1.0e-4,
        contact_angle=0.0,
        shape_moments=np.array([0.5, 0.4, 0.3]),
    )
    geometry = DirectContactGeometry(
        has_contact=True,
        patches=(patch,),
        topology_probability=np.array([0.0, 1.0, 0.0]),
    )

    def snapshot(step: int, mileage: float) -> FullCaseAcceptedContactSnapshot:
        empty_normal = np.zeros((0, 4), dtype=float)
        values = {
            "Mileage": mileage,
            "Normal_Force": {
                "L": np.array([[1.0e5, 0.0, 0.0, 1.0]]),
                "R": empty_normal,
            },
            "Prhxf_T": {
                "L": np.array([[100.0, 200.0, 300.0, 0.1, 0.2, 0.3]]),
                "R": np.zeros((0, 6)),
            },
            "Con_RelVel": {"L": np.array([[0.0, 0.0]])},
            "Con_wheel_2_full": {"L": np.array([[0.0, 0.0, 0.0, 1e-4, 1e-4, 0.0]])},
            "Con_wheel_2_a": {"L": np.array([[0.0, 0.0, 0.0, 2e-4, 2e-4, 0.0]])},
            "Con_rail_1": {"L": np.array([[0.0, 0.0]])},
            "Con_rail_1_a": {"L": np.array([[0.0, 0.0]])},
            "Vsdc": {"L": np.zeros((1, 3))},
            "Vjsdc": {"L": np.zeros((1, 3))},
            "Vgd": {"L": np.ones((1, 1))},
            "Network_B_Curvature_Inputs": {"L": np.ones((1, 10))},
            "Network_B_Teacher_Targets": {
                "L": np.array([[1.0e5, 100.0, 200.0, 300.0, 0.1, 0.2, 0.3]])
            },
            "Network_B_Teacher_Topology_Match": {"L": True},
            "Network_B_Teacher_Match_Distance_m": {"L": np.array([1.0e-6])},
        }
        contact = SimpleNamespace(
            con_ws={"FF": values},
            d0_by_wheelset={"FF": 0.0},
            geometry_by_wheelset_side={"FF": {"L": geometry, "R": None}},
        )
        return FullCaseAcceptedContactSnapshot(
            stage="Cal",
            step_index=step,
            iterations=2,
            time=step * 1.0e-4,
            dt=1.0e-4,
            retry_count=0,
            front_mileage=mileage,
            wheel_pose_by_wheelset={"FF": WheelPose2D()},
            effective_rail_displacement_by_wheelset_side={"FF": {}},
            wheel_rail_contact=contact,
        )

    shard_dir = tmp_path / "shards"
    collector = NetworkBAcceptedStepCollector(
        irregularity_seed=17,
        require_shadow_teacher=True,
        shard_dir=shard_dir,
        shard_interval_m=10.0,
    )
    collector(snapshot(1, 49.9))
    collector(snapshot(2, 50.1))
    first = collector.finalize()
    assert len(first) == 2
    assert collector.summary()["accepted_steps"] == 2
    assert list(shard_dir.glob("shard_*.npz"))

    resumed = NetworkBAcceptedStepCollector(
        irregularity_seed=17,
        require_shadow_teacher=True,
        shard_dir=shard_dir,
        shard_interval_m=10.0,
        resume=True,
    )
    resumed(snapshot(2, 50.1))
    resumed(snapshot(3, 50.2))
    combined = resumed.finalize()
    assert len(combined) == 3
    assert resumed.summary()["accepted_steps"] == 3
    report = validate_network_b_dataset(
        combined,
        expected_seed=17,
        require_direct_schema=True,
    )
    assert report["unique_patch_keys"] == 3


@dataclass(frozen=True)
class _Patch:
    start_index: int
    end_index: int


@dataclass(frozen=True)
class _Geometry:
    elastic_penetration: np.ndarray
    patches: tuple[_Patch, ...]


def test_patch_shape_features_are_finite_and_nonnegative() -> None:
    geometry = _Geometry(
        elastic_penetration=np.array([[0.0, 0.0], [0.1, 1.0e-4], [0.2, 0.0]]),
        patches=(_Patch(0, 2),),
    )
    features = network_b_patch_shape_features(geometry)
    assert features.shape == (1, 5)
    assert np.isfinite(features).all()
    assert np.all(features >= 0.0)


def test_direct_patch_shape_features_are_grid_independent() -> None:
    patch = DirectContactPatch(
        start_y=-2.0e-3,
        end_y=3.0e-3,
        peak_wheel_point=np.array([0.0, 0.0, 0.0]),
        peak_rail_point=np.array([0.0, 0.0]),
        corrected_wheel_point=np.array([0.0, 0.0, 0.0]),
        corrected_rail_point=np.array([0.0, 0.0]),
        wheel_profile_lateral=0.0,
        peak_vertical_penetration=2.0e-4,
        peak_normal_penetration=2.0e-4,
        peak_contact_angle=0.0,
        corrected_vertical_penetration=1.0e-4,
        corrected_normal_penetration=1.0e-4,
        contact_angle=0.0,
        shape_moments=np.array([0.5, 0.4, 0.3]),
    )
    geometry = DirectContactGeometry(
        has_contact=True,
        patches=(patch,),
        topology_probability=np.array([0.0, 1.0, 0.0]),
    )
    features = direct_network_b_patch_shape_features(geometry)
    assert features.shape == (1, 4)
    np.testing.assert_allclose(features[0, 0], 5.0e-3 * 2.0e-4 * 0.5)
    assert np.isfinite(features).all()
    assert np.all(features >= 0.0)
