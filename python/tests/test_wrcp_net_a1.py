from __future__ import annotations

import numpy as np

from sditt.models.wrcp_net_a1 import (
    OUTPUT_SIZE,
    WRCPNetA1,
    _loss_and_output_gradient,
    _save_model,
    load_wrcp_net_a1,
    predict_wrcp_net_a1,
)
from sditt.training_data.network_a import FEATURE_NAMES, LABEL_NAMES, MAX_PATCHES


def test_wrcp_net_a1_forward_backward_shapes() -> None:
    model = WRCPNetA1(hidden_sizes=(16, 12), seed=1)
    features = np.zeros((8, len(FEATURE_NAMES)), dtype=np.float32)
    output, cache = model.forward(features, return_cache=True)
    labels = np.zeros((8, MAX_PATCHES, len(LABEL_NAMES)), dtype=np.float32)
    patch_count = np.array([0, 1, 2, 1, 0, 2, 1, 1], dtype=np.int8)
    patch_mask = np.arange(MAX_PATCHES)[None, :] < patch_count[:, None]
    total, classification, regression, gradient = _loss_and_output_gradient(
        output,
        patch_count=patch_count,
        patch_mask=patch_mask,
        normalized_labels=labels,
        class_weights=np.ones((3,), dtype=np.float32),
        regression_weight=1.0,
        slot_weights=np.array([1.0, 4.0], dtype=np.float32),
        huber_delta=1.0,
    )
    weight_gradients, bias_gradients = model.backward(cache, gradient, weight_decay=1.0e-5)

    assert output.shape == (8, OUTPUT_SIZE)
    assert gradient.shape == output.shape
    assert np.isfinite([total, classification, regression]).all()
    assert [value.shape for value in weight_gradients] == [value.shape for value in model.weights]
    assert [value.shape for value in bias_gradients] == [value.shape for value in model.biases]


def test_wrcp_net_a1_artifact_roundtrip(tmp_path) -> None:
    model = WRCPNetA1(hidden_sizes=(8, 8), seed=2)
    feature_mean = np.zeros((len(FEATURE_NAMES),), dtype=np.float32)
    feature_std = np.ones((len(FEATURE_NAMES),), dtype=np.float32)
    label_mean = np.zeros((len(LABEL_NAMES),), dtype=np.float32)
    label_std = np.ones((len(LABEL_NAMES),), dtype=np.float32)
    path = tmp_path / "model.npz"
    _save_model(
        path,
        model,
        feature_mean=feature_mean,
        feature_std=feature_std,
        label_mean=label_mean,
        label_std=label_std,
    )
    loaded, normalization = load_wrcp_net_a1(path)
    features = np.zeros((3, len(FEATURE_NAMES)), dtype=np.float32)

    assert np.allclose(model.forward(features), loaded.forward(features))
    prediction = predict_wrcp_net_a1(loaded, features, normalization)
    assert prediction.patch_count.shape == (3,)
    assert prediction.patch_mask.shape == (3, MAX_PATCHES)
    assert prediction.labels.shape == (3, MAX_PATCHES, len(LABEL_NAMES))
    assert prediction.class_probability.shape == (3, 3)
    assert np.allclose(np.sum(prediction.class_probability, axis=1), 1.0)
