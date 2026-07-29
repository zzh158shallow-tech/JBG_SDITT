from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from sditt.training_data.network_a import (
    FLANGE_CONTACT_ANGLE_RAD,
    LABEL_NAMES,
    MAX_PATCHES,
    MULTI_PATCH_MIN_SEPARATION_M,
    NetworkADataset,
    assign_group_splits,
    build_network_a_teacher_context,
    collect_coupled_samples,
    derive_parameter_bounds,
    generate_parameter_samples,
    geometry_to_network_a_labels,
    iter_network_a_batches,
    load_network_a_dataset,
    save_network_a_archive,
    teacher_geometry_from_features,
)


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_teacher_geometry_labels_are_fixed_shape_and_replayable() -> None:
    context = build_network_a_teacher_context(REPO_ROOT)
    features = np.array([0.0, 0.0, 0.001, 0.0, 0.0, 0.1702], dtype=float)
    geometry = teacher_geometry_from_features(context, features)
    patch_count, patch_mask, labels = geometry_to_network_a_labels(geometry)

    replay = teacher_geometry_from_features(context, features)
    replay_count, replay_mask, replay_labels = geometry_to_network_a_labels(replay)
    assert patch_count <= MAX_PATCHES
    assert patch_mask.shape == (MAX_PATCHES,)
    assert labels.shape == (MAX_PATCHES, len(LABEL_NAMES))
    assert replay_count == patch_count
    assert np.array_equal(replay_mask, patch_mask)
    assert np.allclose(replay_labels, labels, rtol=1.0e-10, atol=1.0e-12)


def test_parameter_generator_meets_small_class_quotas() -> None:
    context = build_network_a_teacher_context(REPO_ROOT)
    coupled_like = np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.1702],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.1703],
        ],
        dtype=float,
    )
    bounds = derive_parameter_bounds(coupled_like)
    dataset, rejected = generate_parameter_samples(
        {"normal": 5, "boundary": 5, "no_contact": 5},
        context=context,
        bounds=bounds,
        seed=11,
    )
    values, counts = np.unique(dataset.metadata["sample_class"], return_counts=True)
    assert dict(zip(values, counts, strict=True)) == {"boundary": 5, "no_contact": 5, "normal": 5}
    assert not rejected
    assert np.all(dataset.patch_count[dataset.metadata["sample_class"] == "no_contact"] == 0)


def test_parameter_generator_targets_balanced_flange_multi_contact() -> None:
    context = build_network_a_teacher_context(REPO_ROOT)
    bounds = derive_parameter_bounds(
        np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.1702],
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.1703],
            ],
            dtype=float,
        )
    )
    dataset, rejected = generate_parameter_samples(
        {"flange_multi": 12},
        context=context,
        bounds=bounds,
        seed=13,
    )

    assert not rejected
    assert len(dataset) == 12
    assert set(dataset.metadata["sample_class"]) == {"flange_multi"}
    assert dict(zip(*np.unique(dataset.metadata["side"], return_counts=True), strict=True)) == {
        "L": 6,
        "R": 6,
    }
    assert np.all(dataset.patch_count == 2)
    angle_index = LABEL_NAMES.index("corrected_contact_angle_rad")
    rail_y_index = LABEL_NAMES.index("corrected_rail_y_m")
    angles = np.abs(dataset.labels[:, :, angle_index])
    separation = np.abs(dataset.labels[:, 1, rail_y_index] - dataset.labels[:, 0, rail_y_index])
    assert np.all(np.max(angles, axis=1) >= FLANGE_CONTACT_ANGLE_RAD)
    assert np.all(separation >= MULTI_PATCH_MIN_SEPARATION_M)


def test_accepted_step_collector_replays_coupled_geometry() -> None:
    context = build_network_a_teacher_context(REPO_ROOT)
    dataset, rejected = collect_coupled_samples(
        8,
        repo_root=REPO_ROOT,
        group_id="test-smooth",
        irregularity_model="none",
        irregularity_seed=-1,
        cut_freq=50.0,
    )
    assert len(dataset) == 8
    assert not rejected
    assert set(dataset.metadata["source"]) == {"coupled_smooth"}
    for index in range(len(dataset)):
        geometry = teacher_geometry_from_features(context, dataset.features[index])
        patch_count, patch_mask, labels = geometry_to_network_a_labels(geometry)
        assert patch_count == dataset.patch_count[index]
        assert np.array_equal(patch_mask, dataset.patch_mask[index])
        assert np.allclose(labels, dataset.labels[index], rtol=1.0e-10, atol=1.0e-12)


def test_group_split_archive_loader_and_batch_iterator(tmp_path) -> None:
    context = build_network_a_teacher_context(REPO_ROOT)
    bounds = derive_parameter_bounds(
        np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.1702],
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.1703],
            ],
            dtype=float,
        )
    )
    dataset, _ = generate_parameter_samples(
        {"normal": 10, "boundary": 10, "no_contact": 10},
        context=context,
        bounds=bounds,
        seed=12,
    )
    splits = assign_group_splits(dataset, train_count=20, validation_count=5, test_count=5)
    split_groups = {
        name: set(dataset.metadata["group_id"][indexes]) for name, indexes in splits.items()
    }
    assert not (split_groups["train"] & split_groups["validation"])
    assert not (split_groups["train"] & split_groups["test"])
    assert not (split_groups["validation"] & split_groups["test"])

    path = save_network_a_archive(tmp_path / "all_samples.npz", dataset)
    loaded = load_network_a_dataset(path, split="all", dtype=np.float64)
    assert isinstance(loaded, NetworkADataset)
    assert len(loaded) == 30
    assert np.array_equal(loaded.patch_mask, dataset.patch_mask)
    assert sum(len(batch) for batch in iter_network_a_batches(loaded, batch_size=7)) == 30


def test_manifest_json_shapes_are_serializable() -> None:
    payload = {
        "feature_names": ["side_id", "delta_y_m"],
        "parameter_bounds": np.zeros((5, 2), dtype=float).tolist(),
    }
    assert json.loads(json.dumps(payload)) == payload
