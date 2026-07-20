from __future__ import annotations

import numpy as np

from sditt.models.wrcp_net_a1 import WRCPNetA1
from sditt.models.wrcp_net_a2r import (
    PenetrationResidualHeads,
    _save_augmented_model,
    apply_penetration_residuals,
    load_penetration_residual_heads,
)
from sditt.training_data.network_a import (
    build_network_a_teacher_context,
    teacher_geometry_from_features,
)


def _constant_model(values: np.ndarray) -> WRCPNetA1:
    model = WRCPNetA1(input_size=9, hidden_sizes=(), output_size=4, seed=7)
    model.weights[0][...] = 0.0
    model.biases[0][...] = np.asarray(values, dtype=np.float32)
    return model


def _heads(*, left_gain: float = 0.6, right_gain: float = 0.25) -> PenetrationResidualHeads:
    return PenetrationResidualHeads(
        left=_constant_model(np.ones((4,), dtype=np.float32)),
        right=_constant_model(-np.ones((4,), dtype=np.float32)),
        feature_mean=np.zeros((9,), dtype=np.float32),
        feature_std=np.ones((9,), dtype=np.float32),
        target_scale=np.full((4,), 1.0e-5, dtype=np.float32),
        residual_gain=np.array([left_gain, right_gain], dtype=np.float32),
    )


def test_residual_heads_apply_independent_left_right_closed_loop_gain() -> None:
    heads = _heads()
    base = np.full((2, 4), 2.0e-5, dtype=np.float32)
    features = np.array(
        [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.17],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.17],
        ],
        dtype=np.float32,
    )

    correction = heads.predict(features, base)

    bounded = 5.0e-5 * np.tanh(1.0e-5 / 5.0e-5)
    assert np.allclose(correction[0], 0.6 * bounded)
    assert np.allclose(correction[1], -0.25 * bounded)


def test_apply_penetration_residuals_updates_vertical_and_normal_penetration() -> None:
    context = build_network_a_teacher_context()
    features = np.array([0.0, 0.0, 0.001, 0.0, 0.0, 0.1702], dtype=float)
    geometry = teacher_geometry_from_features(context, features)
    assert geometry.patches
    before = geometry.patches[0]

    corrected = apply_penetration_residuals(_heads(), features, geometry)
    after = corrected.patches[0]

    expected = 0.6 * 5.0e-5 * np.tanh(1.0e-5 / 5.0e-5)
    assert np.isclose(
        after.corrected_vertical_penetration,
        before.corrected_vertical_penetration + expected,
    )
    assert np.isclose(
        after.peak_vertical_penetration,
        before.peak_vertical_penetration + expected,
    )
    assert np.isclose(
        after.corrected_normal_penetration,
        after.corrected_vertical_penetration / np.cos(after.contact_angle),
    )


def test_augmented_model_round_trip_preserves_residual_gain(tmp_path) -> None:
    base = tmp_path / "base.npz"
    output = tmp_path / "augmented.npz"
    np.savez_compressed(base, marker=np.array([1], dtype=np.int8))

    _save_augmented_model(base, output, _heads())
    loaded = load_penetration_residual_heads(output)

    assert loaded is not None
    assert np.allclose(loaded.residual_gain, [0.6, 0.25])
    with np.load(output, allow_pickle=False) as archive:
        assert archive["marker"][0] == 1
