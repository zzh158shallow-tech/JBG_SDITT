from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from sditt.contact.geometry import BoundaryExtrema, MultiPointContactGeometry, WheelPose2D
from sditt.models import network_a_full_case
from sditt.models.network_a_full_case import NetworkAContactGeometryAdapter
from sditt.models.wrcp_net_a2g import (
    GAP_GRID_SIZE,
    WRCPNetA2G,
    WRCPNetA2GPrediction,
    _canonical_grid,
    _field_loss_and_gradient,
    _geometry_from_predicted_field,
    _topology_matching_threshold,
    _teacher_gap_on_grid,
)
from sditt.training_data.network_a import (
    build_network_a_teacher_context,
    geometry_to_network_a_labels,
    teacher_geometry_from_features,
)


def test_wrcp_net_a2g_loss_and_gradient_shapes() -> None:
    model = WRCPNetA2G(hidden_sizes=(16, 12), grid_size=33, seed=3)
    features = np.zeros((8, 6), dtype=np.float32)
    output, cache = model.forward(features, return_cache=True)
    target = np.zeros((8, 33), dtype=np.float32)
    physical = np.linspace(-0.01, 0.001, 33, dtype=np.float32)[None, :].repeat(8, axis=0)
    patch_count = np.array([0, 1, 2, 1, 0, 2, 1, 1], dtype=np.int8)
    losses = _field_loss_and_gradient(
        output,
        patch_count=patch_count,
        target_normalized=target,
        target_physical=physical,
        class_weights=np.ones((3,), dtype=np.float32),
        near_surface_scale_m=5.0e-4,
        positive_weight=12.0,
        double_field_weight=1.0,
        field_weight=2.0,
        derivative_weight=0.2,
        huber_delta=1.0,
    )
    weight_gradients, bias_gradients = model.backward(cache, losses[4], weight_decay=1.0e-5)

    assert output.shape == (8, 36)
    assert losses[4].shape == output.shape
    assert np.isfinite(losses[:4]).all()
    assert [value.shape for value in weight_gradients] == [value.shape for value in model.weights]
    assert [value.shape for value in bias_gradients] == [value.shape for value in model.biases]


def test_fast_topology_threshold_matches_original_brute_force_search() -> None:
    rng = np.random.default_rng(20260720)
    fields = [
        rng.normal(size=127),
        np.round(rng.normal(size=127), decimals=1),
        np.array([-1.0, 1.0, -1.0, 1.0, -1.0]),
        np.ones((17,), dtype=float),
    ]
    for field in fields:
        for desired_count in (1, 2):
            sorted_values = np.unique(np.sort(field))
            thresholds = np.concatenate(
                (
                    np.array([0.0]),
                    sorted_values[:1] - 1.0e-12,
                    0.5 * (sorted_values[:-1] + sorted_values[1:]),
                    sorted_values[-1:] + 1.0e-12,
                )
            )
            expected = None
            for threshold in thresholds:
                positive = field > float(threshold)
                count = int(positive[0]) + int(
                    np.count_nonzero(positive[1:] & ~positive[:-1])
                )
                if count == desired_count and (
                    expected is None or abs(float(threshold)) < abs(expected)
                ):
                    expected = float(threshold)
            actual = _topology_matching_threshold(field, desired_count)
            if expected is None:
                assert actual is None
            else:
                assert actual == expected


def test_true_canonical_gap_field_reconstructs_teacher_geometry() -> None:
    context = build_network_a_teacher_context()
    grid = _canonical_grid(context, grid_size=GAP_GRID_SIZE)
    features = np.array([1.0, 0.001, 0.001, 0.0, 0.0, 0.1702], dtype=float)
    teacher = teacher_geometry_from_features(context, features)
    field = _teacher_gap_on_grid(context, features, grid)
    reconstructed = _geometry_from_predicted_field(
        context,
        features,
        grid,
        field,
        len(teacher.patches),
    )
    expected_count, expected_mask, expected_labels = geometry_to_network_a_labels(teacher)
    actual_count, actual_mask, actual_labels = geometry_to_network_a_labels(reconstructed)

    assert actual_count == expected_count
    assert np.array_equal(actual_mask, expected_mask)
    assert np.allclose(actual_labels, expected_labels, rtol=0.0, atol=1.0e-4)


def test_profile_consistency_refinement_recovers_native_geometry() -> None:
    context = build_network_a_teacher_context()
    grid = _canonical_grid(context, grid_size=GAP_GRID_SIZE)
    features = np.array([1.0, 0.001, 0.001, 0.0, 0.0, 0.1702], dtype=float)
    teacher = teacher_geometry_from_features(context, features)
    deliberately_coarse_field = np.zeros_like(grid)
    reconstructed = _geometry_from_predicted_field(
        context,
        features,
        grid,
        deliberately_coarse_field,
        len(teacher.patches),
    )
    expected_count, expected_mask, expected_labels = geometry_to_network_a_labels(teacher)
    actual_count, actual_mask, actual_labels = geometry_to_network_a_labels(reconstructed)

    assert actual_count == expected_count
    assert np.array_equal(actual_mask, expected_mask)
    assert np.allclose(actual_labels, expected_labels, rtol=1.0e-10, atol=1.0e-12)


def test_candidate_region_refinement_recovers_native_geometry_without_full_search() -> None:
    context = build_network_a_teacher_context()
    grid = _canonical_grid(context, grid_size=GAP_GRID_SIZE)
    features = np.array([1.0, 0.001, 0.001, 0.0, 0.0, 0.1702], dtype=float)
    teacher = teacher_geometry_from_features(context, features)
    reconstructed = _geometry_from_predicted_field(
        context,
        features,
        grid,
        np.zeros_like(grid),
        len(teacher.patches),
        profile_refinement=False,
        candidate_profile_refinement=True,
    )
    expected_count, expected_mask, expected_labels = geometry_to_network_a_labels(teacher)
    actual_count, actual_mask, actual_labels = geometry_to_network_a_labels(reconstructed)

    assert actual_count == expected_count
    assert np.array_equal(actual_mask, expected_mask)
    assert np.allclose(actual_labels, expected_labels, rtol=1.0e-10, atol=1.0e-12)


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


def test_full_case_adapter_uses_strict_unrefined_network_geometry(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: dict[str, object] = {}

    def fake_predict(*_args: object, **_kwargs: object) -> WRCPNetA2GPrediction:
        return WRCPNetA2GPrediction(
            patch_count=np.array([0], dtype=np.int8),
            class_probability=np.array([[1.0, 0.0, 0.0]], dtype=np.float32),
            gap_field_m=np.zeros((1, 5), dtype=np.float32),
        )

    def fake_reconstruct(*_args: object, **kwargs: object) -> MultiPointContactGeometry:
        captured["profile_refinement"] = kwargs["profile_refinement"]
        return _empty_geometry()

    monkeypatch.setattr(network_a_full_case, "predict_wrcp_net_a2g", fake_predict)
    monkeypatch.setattr(network_a_full_case, "_geometry_from_predicted_field", fake_reconstruct)
    adapter = NetworkAContactGeometryAdapter(
        model=object(),  # type: ignore[arg-type]
        normalization={"canonical_y_m": np.linspace(0.0, 1.0, 5)},
        context=object(),  # type: ignore[arg-type]
        model_path=Path("model.npz"),
    )
    result = adapter.solve(
        side="L",
        pose=WheelPose2D(),
        d0=0.1705,
        rail_shift_yz=np.zeros((2,), dtype=float),
    )

    assert captured["profile_refinement"] is False
    assert not result.has_contact


def test_full_case_adapter_raises_on_topology_mismatch_without_fallback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        network_a_full_case,
        "predict_wrcp_net_a2g",
        lambda *_args, **_kwargs: WRCPNetA2GPrediction(
            patch_count=np.array([1], dtype=np.int8),
            class_probability=np.array([[0.0, 1.0, 0.0]], dtype=np.float32),
            gap_field_m=np.zeros((1, 5), dtype=np.float32),
        ),
    )
    monkeypatch.setattr(
        network_a_full_case,
        "_geometry_from_predicted_field",
        lambda *_args, **_kwargs: _empty_geometry(),
    )
    adapter = NetworkAContactGeometryAdapter(
        model=object(),  # type: ignore[arg-type]
        normalization={"canonical_y_m": np.linspace(0.0, 1.0, 5)},
        context=object(),  # type: ignore[arg-type]
        model_path=Path("model.npz"),
    )

    with pytest.raises(RuntimeError, match="without fallback"):
        adapter.solve(
            side="R",
            pose=WheelPose2D(),
            d0=0.1705,
            rail_shift_yz=np.zeros((2,), dtype=float),
        )
