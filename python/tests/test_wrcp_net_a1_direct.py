from __future__ import annotations

import json
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from sditt.contact.geometry import (
    BoundaryExtrema,
    ContactPatch,
    DirectContactGeometry,
    MultiPointContactGeometry,
    WheelPose2D,
)
from sditt.models.train_wrcp_net_a1_direct import (
    RUNTIME_DAGGER_SAMPLE_WEIGHT,
    _direct_sample_weights,
    _direct_topology_weights,
    _predict_history_batch,
    _predicted_previous_histories,
    _validate_supervision_policy,
)
from sditt.models.network_a1_direct_full_case import (
    NetworkA1DirectContactGeometryAdapter,
    direct_geometry_from_traditional,
)
from sditt.models.wrcp_net_a1_direct import (
    _canonical_side_target_transform,
    _fixed_wheel_trace_point,
    _invert_fixed_wheel_lateral,
    geometry_to_history,
    initialize_wrcp_net_a1_direct,
    load_wrcp_net_a1_direct,
    save_wrcp_net_a1_direct,
)
from sditt.training_data.network_a_direct import (
    NetworkA1DirectDataset,
    _previous_accepted_indexes,
    _softplus_inverse,
    ensure_supervision_metadata,
    load_direct_dataset,
    save_direct_dataset,
)
from sditt.training_data.network_a_direct_dagger import _geometry_errors
from sditt.training_data.network_a_direct_fresh import concatenate_direct_datasets
from sditt.training_data.network_a_direct_rebalance import build_rebalanced_dataset


def _deterministic_single_patch_model():
    target_mean = np.array(
        [
            0.0,
            _softplus_inverse(0.01),
            _softplus_inverse(0.02),
            0.0,
            0.60,
            0.0,
            _softplus_inverse(1.0e-4),
            0.05,
            0.0,
            0.0,
            0.60,
            _softplus_inverse(5.0e-5),
            0.06,
            0.0,
            0.0,
            0.0,
        ],
        dtype=float,
    )
    model = initialize_wrcp_net_a1_direct(target_mean=target_mean)
    for values in (
        model.state_weights,
        model.state_biases,
        model.history_weights,
        model.history_biases,
        model.decoder_weights,
        model.decoder_biases,
    ):
        for value in values:
            value.fill(0.0)
    model.fusion_weight.fill(0.0)
    model.fusion_bias.fill(0.0)
    for weights in model.residual_weights:
        for value in weights:
            value.fill(0.0)
    for biases in model.residual_biases:
        for value in biases:
            value.fill(0.0)
    model.topology_weight.fill(0.0)
    model.topology_bias[:] = np.array([0.0, 12.0, 0.0])
    model.query_embeddings.fill(0.0)
    return replace(model, topology_confidence_min=0.95, ood_threshold=10.0)


def test_direct_model_decodes_final_geometry_without_profile_arrays(tmp_path) -> None:
    model = _deterministic_single_patch_model()
    result = model.predict(features=np.zeros((6,)))
    geometry = result.geometry
    assert geometry.has_contact
    assert len(geometry.patches) == 1
    assert not hasattr(geometry, "elastic_penetration")
    patch = geometry.patches[0]
    assert patch.start_y == pytest.approx(-0.01)
    assert patch.end_y == pytest.approx(0.02)
    assert patch.corrected_rail_point[0] == pytest.approx(patch.corrected_wheel_point[1])
    assert patch.peak_rail_point[0] == pytest.approx(patch.peak_wheel_point[1])
    assert patch.peak_vertical_penetration > patch.corrected_vertical_penetration > 0.0
    assert patch.corrected_normal_penetration == pytest.approx(
        patch.corrected_vertical_penetration / np.cos(patch.contact_angle)
    )

    path = tmp_path / "model.npz"
    save_wrcp_net_a1_direct(path, model)
    loaded = load_wrcp_net_a1_direct(path)
    loaded_patch = loaded.predict(features=np.zeros((6,))).geometry.patches[0]
    np.testing.assert_allclose(loaded_patch.corrected_wheel_point, patch.corrected_wheel_point)


def test_history_encoder_is_permutation_invariant() -> None:
    model = replace(_deterministic_single_patch_model(), topology_confidence_min=0.0)
    history = np.arange(40, dtype=float).reshape(2, 20) * 1.0e-4
    mask = np.array([True, True])
    first = model.forward_raw(np.zeros((6,)), history, mask)
    second = model.forward_raw(np.zeros((6,)), history[::-1], mask[::-1])
    np.testing.assert_allclose(first[0], second[0])
    np.testing.assert_allclose(first[1], second[1])


def test_forward_raw_batch_matches_scalar_forward() -> None:
    model = initialize_wrcp_net_a1_direct(seed=20260726)
    rng = np.random.default_rng(20260726)
    features = rng.normal(0.0, 0.1, size=(8, 6))
    history = rng.normal(0.0, 0.01, size=(8, 2, 20))
    history_mask = rng.random((8, 2)) > 0.4

    actual_topology, actual_queries = model.forward_raw_batch(
        features,
        history,
        history_mask,
    )
    expected = tuple(
        model.forward_raw(features[index], history[index], history_mask[index])
        for index in range(features.shape[0])
    )
    np.testing.assert_allclose(
        actual_topology,
        np.stack([value[0] for value in expected]),
        rtol=2.0e-13,
        atol=2.0e-13,
    )
    np.testing.assert_allclose(
        actual_queries,
        np.stack([value[1] for value in expected]),
        rtol=2.0e-13,
        atol=2.0e-13,
    )


def test_predict_batch_matches_scalar_geometry() -> None:
    wheel = np.array(
        [
            [-1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ]
    )
    rail = np.array([[-1.0, 0.4], [0.0, 0.5], [1.0, 0.6]])
    model = replace(
        _deterministic_single_patch_model(),
        topology_confidence_min=0.0,
        ood_threshold=1.0e12,
        wheel_trace_profile_left=wheel,
        wheel_trace_profile_right=wheel,
        rail_profile_left=rail,
        rail_profile_right=rail,
    )
    features = np.array(
        [
            [float(index % 2), 1.0e-3 * index, -2.0e-4 * index, 1.0e-4, 0.0, 0.0]
            for index in range(8)
        ],
        dtype=float,
    )
    history = np.zeros((8, 2, 20), dtype=float)
    history_mask = np.zeros((8, 2), dtype=bool)

    actual = model.predict_batch(
        features=features,
        history=history,
        history_mask=history_mask,
    )
    expected = tuple(
        model.predict(
            features=features[index],
            history=history[index],
            history_mask=history_mask[index],
        )
        for index in range(features.shape[0])
    )
    for actual_prediction, expected_prediction in zip(actual, expected, strict=True):
        actual_history, actual_mask = geometry_to_history(actual_prediction.geometry)
        expected_history, expected_mask = geometry_to_history(expected_prediction.geometry)
        np.testing.assert_allclose(
            actual_history,
            expected_history,
            rtol=2.0e-12,
            atol=2.0e-10,
        )
        np.testing.assert_array_equal(actual_mask, expected_mask)
        np.testing.assert_allclose(
            actual_prediction.topology_probability,
            expected_prediction.topology_probability,
            rtol=2.0e-13,
            atol=2.0e-13,
        )


def test_batched_rollout_history_matches_scalar_runtime_projection() -> None:
    wheel = np.array(
        [
            [-1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ]
    )
    rail = np.array([[-1.0, 0.4], [0.0, 0.5], [1.0, 0.6]])
    model = replace(
        _deterministic_single_patch_model(),
        topology_confidence_min=0.0,
        ood_threshold=1.0e12,
        wheel_trace_profile_left=wheel,
        wheel_trace_profile_right=wheel,
        rail_profile_left=rail,
        rail_profile_right=rail,
    )
    features = np.array(
        [
            [0.0, -0.01, 0.001, 0.002, 0.001, 0.0001],
            [1.0, 0.01, -0.001, -0.002, -0.001, 0.0002],
        ]
    )
    history = np.zeros((2, 2, 20), dtype=float)
    history_mask = np.zeros((2, 2), dtype=bool)
    actual, actual_mask, valid = _predict_history_batch(
        model,
        features,
        history,
        history_mask,
    )
    assert valid.all()
    for row in range(features.shape[0]):
        expected, expected_mask = geometry_to_history(
            model.predict(
                features=features[row],
                history=history[row],
                history_mask=history_mask[row],
            ).geometry
        )
        np.testing.assert_allclose(actual[row], expected, atol=2.0e-10)
        np.testing.assert_array_equal(actual_mask[row], expected_mask)


def test_fixed_profile_projection_overwrites_dependent_point_fields(tmp_path) -> None:
    wheel = np.array(
        [
            [-1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ]
    )
    rail = np.array([[-1.0, 0.4], [0.0, 0.5], [1.0, 0.6]])
    model = replace(
        _deterministic_single_patch_model(),
        wheel_trace_profile_left=wheel,
        wheel_trace_profile_right=wheel,
        rail_profile_left=rail,
        rail_profile_right=rail,
    )
    prediction = model.predict(features=np.zeros((6,)))
    patch = prediction.geometry.patches[0]
    assert model.has_fixed_profile_projection
    assert patch.corrected_wheel_point[0] == pytest.approx(0.0)
    assert patch.corrected_rail_point[1] == pytest.approx(0.5)
    assert patch.wheel_profile_lateral == pytest.approx(0.0, abs=2.0e-9)
    assert patch.contact_angle == pytest.approx(0.0)
    assert patch.peak_wheel_point[0] == pytest.approx(0.0)
    # The learned peak candidate lies on the downhill side of this synthetic
    # linear gap, so the physically higher corrected centre is retained.
    assert patch.peak_rail_point[0] == pytest.approx(patch.corrected_rail_point[0])
    assert patch.peak_rail_point[1] == pytest.approx(0.5)
    assert patch.corrected_vertical_penetration == pytest.approx(0.5)
    assert patch.peak_vertical_penetration == pytest.approx(0.5, abs=5.0e-12)
    assert patch.peak_contact_angle == pytest.approx(0.0)

    path = tmp_path / "projected-model.npz"
    save_wrcp_net_a1_direct(path, model)
    loaded = load_wrcp_net_a1_direct(path)
    assert loaded.has_fixed_profile_projection
    np.testing.assert_allclose(loaded.wheel_trace_profile_left, wheel)


def test_fixed_wheel_inverse_recovers_production_scale_points() -> None:
    wheel_lateral = np.linspace(-0.07, 0.07, 11_644)
    wheel = np.column_stack(
        (
            wheel_lateral,
            0.43 - 0.012 * wheel_lateral**2,
            0.025 * np.tanh(-18.0 * wheel_lateral),
        )
    )
    poses = (
        (-7.4e-4, -2.5e-4, 5.1e-5),
        (-3.5e-4, 1.6e-4, -2.8e-6),
        (2.0e-4, 3.9e-4, -3.9e-5),
    )
    samples = np.linspace(-0.055, 0.055, 17)
    for lateral, roll, yaw in poses:
        for expected in samples:
            target_y = _fixed_wheel_trace_point(
                float(expected),
                wheel,
                lateral=lateral,
                roll=roll,
                yaw=yaw,
            )[1]
            actual = _invert_fixed_wheel_lateral(
                target_y,
                wheel,
                lateral=lateral,
                roll=roll,
                yaw=yaw,
            )
            assert actual == pytest.approx(expected, abs=2.0e-11)


def test_accepted_step_increment_limit_bounds_existing_patch_and_persists(tmp_path) -> None:
    wheel = np.array(
        [
            [-1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ]
    )
    rail = np.array([[-1.0, 0.4], [0.0, 0.5], [1.0, 0.6]])
    anchor_model = replace(
        _deterministic_single_patch_model(),
        wheel_trace_profile_left=wheel,
        wheel_trace_profile_right=wheel,
        rail_profile_left=rail,
        rail_profile_right=rail,
    )
    anchor = anchor_model.predict(features=np.zeros((6,))).geometry
    history, history_mask = geometry_to_history(anchor)

    shifted_mean = anchor_model.target_mean.copy()
    shifted_mean[0] = 5.0e-3
    limited_model = replace(
        anchor_model,
        target_mean=shifted_mean,
        accepted_step_increment_limits=np.array(
            [1.0e-3, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        ),
        topology_hysteresis_min=0.9,
        shape_moment_min=np.array([0.60, 0.52, 0.45]),
        shape_moment_max=np.array([0.70, 0.62, 0.56]),
        shape_moment_history_blend=0.25,
    )
    limited = limited_model.predict(
        features=np.zeros((6,)),
        history=history,
        history_mask=history_mask,
    )
    assert limited.geometry.patches[0].corrected_rail_point[0] == pytest.approx(1.0e-3)

    path = tmp_path / "limited-model.npz"
    save_wrcp_net_a1_direct(path, limited_model)
    loaded = load_wrcp_net_a1_direct(path)
    np.testing.assert_allclose(
        loaded.accepted_step_increment_limits,
        limited_model.accepted_step_increment_limits,
    )
    assert loaded.topology_hysteresis_min == pytest.approx(0.9)
    np.testing.assert_allclose(loaded.shape_moment_min, limited_model.shape_moment_min)
    np.testing.assert_allclose(loaded.shape_moment_max, limited_model.shape_moment_max)
    assert loaded.shape_moment_history_blend == pytest.approx(0.25)


def test_topology_hysteresis_requires_strong_evidence_to_drop_existing_patch() -> None:
    model = _deterministic_single_patch_model()
    anchor = model.predict(features=np.zeros((6,))).geometry
    history, history_mask = geometry_to_history(anchor)
    model.topology_bias[:] = np.array([2.0, 0.0, 0.0])
    prediction = replace(
        model,
        topology_confidence_min=0.95,
        topology_hysteresis_min=0.9,
    ).predict(
        features=np.zeros((6,)),
        history=history,
        history_mask=history_mask,
    )
    assert int(np.argmax(prediction.topology_probability)) == 0
    assert len(prediction.geometry.patches) == 1


def test_topology_hysteresis_retains_existing_argmax_below_static_confidence() -> None:
    model = _deterministic_single_patch_model()
    anchor = model.predict(features=np.zeros((6,))).geometry
    history, history_mask = geometry_to_history(anchor)
    model.topology_bias[:] = np.array([0.0, 0.2, -10.0])
    prediction = replace(
        model,
        topology_confidence_min=0.95,
        topology_hysteresis_min=0.9,
    ).predict(
        features=np.zeros((6,)),
        history=history,
        history_mask=history_mask,
    )
    assert prediction.topology_probability[1] < 0.95
    assert len(prediction.geometry.patches) == 1


def test_topology_hysteresis_rejects_transition_without_query_evidence() -> None:
    model = _deterministic_single_patch_model()
    anchor = model.predict(features=np.zeros((6,))).geometry
    history, history_mask = geometry_to_history(anchor)
    model.topology_bias[:] = np.array([-10.0, 0.0, 12.0])
    model.decoder_biases[-1][0] = -20.0
    prediction = replace(
        model,
        topology_confidence_min=0.95,
        topology_hysteresis_min=0.9,
    ).predict(
        features=np.zeros((6,)),
        history=history,
        history_mask=history_mask,
    )
    assert int(np.argmax(prediction.topology_probability)) == 2
    assert np.all(prediction.query_probability < 0.5)
    assert len(prediction.geometry.patches) == 1
    np.testing.assert_allclose(
        prediction.geometry.patches[0].corrected_rail_point,
        anchor.patches[0].corrected_rail_point,
    )
    np.testing.assert_allclose(
        prediction.geometry.patches[0].shape_moments,
        anchor.patches[0].shape_moments,
    )


def test_topology_hysteresis_treats_accepted_no_contact_as_prior() -> None:
    model = _deterministic_single_patch_model()
    model.topology_bias[:] = np.array([0.0, 1.4, -10.0])
    prediction = replace(
        model,
        topology_confidence_min=0.95,
        topology_hysteresis_min=0.9,
    ).predict(
        features=np.zeros((6,)),
        history=np.zeros((2, 20)),
        history_mask=np.zeros((2,), dtype=bool),
        topology_prior_available=True,
        previous_patch_count=0,
    )
    assert prediction.topology_probability[1] < 0.9
    assert not prediction.geometry.patches


def test_runtime_projects_shape_moments_to_physical_ordering() -> None:
    model = _deterministic_single_patch_model()
    target_mean = model.target_mean.copy()
    values = np.array([0.6, 0.8, 0.7])
    target_mean[13:16] = np.log(values) - np.log1p(-values)
    patch = replace(model, target_mean=target_mean).predict(
        features=np.zeros((6,))
    ).geometry.patches[0]
    moment_1, moment_1p5, moment_2 = patch.shape_moments
    assert moment_1**1.5 <= moment_1p5 <= moment_1
    assert moment_1**2 <= moment_2 <= moment_1p5


def test_direct_model_fails_fast_on_out_of_distribution_input() -> None:
    model = replace(_deterministic_single_patch_model(), ood_threshold=1.0)
    with pytest.raises(RuntimeError, match="outside its calibrated distribution"):
        model.predict(features=np.array([0.0, 2.0, 0.0, 0.0, 0.0, 0.0]))


def test_direct_adapter_persists_fail_fast_diagnostic(tmp_path) -> None:
    model_path = tmp_path / "model.npz"
    save_wrcp_net_a1_direct(
        model_path,
        replace(_deterministic_single_patch_model(), ood_threshold=1.0),
    )
    trace_dir = tmp_path / "trace"
    adapter = NetworkA1DirectContactGeometryAdapter.load(
        model_path,
        trace_output_dir=trace_dir,
    )
    with pytest.raises(RuntimeError, match="outside its calibrated distribution"):
        adapter.solve(
            side="L",
            pose=WheelPose2D(lateral=2.0),
            d0=0.0,
            rail_shift_yz=np.zeros((2,)),
            diagnostic_context={
                "stage": "Cal",
                "wheelset": "FF",
                "step_index": 0,
                "iteration": 0,
                "time_s": 0.0,
                "dt_s": 1.0e-4,
            },
        )
    record = json.loads((trace_dir / "iterations.jsonl").read_text(encoding="utf-8"))
    assert record["hard_constraints_passed"] is False
    assert record["in_distribution"] is False
    assert "outside its calibrated distribution" in record["failure"]["message"]
    manifest = json.loads((trace_dir / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["iteration_records"] == 1


def test_direct_adapter_reports_nested_prediction_timing(tmp_path) -> None:
    model_path = tmp_path / "model.npz"
    save_wrcp_net_a1_direct(model_path, _deterministic_single_patch_model())
    adapter = NetworkA1DirectContactGeometryAdapter.load(model_path)
    timing: dict[str, float] = {}

    adapter.solve(
        side="L",
        pose=WheelPose2D(),
        d0=0.0,
        rail_shift_yz=np.zeros((2,)),
        diagnostic_context={
            "stage": "Cal",
            "wheelset": "FF",
            "step_index": 1,
            "iteration": 1,
            "time_s": 1.0e-4,
            "dt_s": 1.0e-4,
        },
        timing=timing,
    )

    for key in (
        "network_a1.adapter_input",
        "network_a1.input_ood",
        "network_a1.forward_raw",
        "network_a1.topology_history",
        "network_a1.fixed_profile_projection",
        "network_a1.decode_validate",
        "network_a1.predict_total",
        "network_a1.adapter_finalize",
        "network_a1.adapter_total",
    ):
        assert timing[key] >= 0.0
    assert timing["network_a1.adapter_total"] >= timing["network_a1.predict_total"]


def test_direct_adapter_batch_matches_eight_scalar_solves(tmp_path) -> None:
    model_path = tmp_path / "model.npz"
    save_wrcp_net_a1_direct(model_path, _deterministic_single_patch_model())
    batched_adapter = NetworkA1DirectContactGeometryAdapter.load(model_path)
    scalar_adapter = NetworkA1DirectContactGeometryAdapter.load(model_path)
    requests = tuple(
        {
            "side": side,
            "pose": WheelPose2D(
                lateral=1.0e-4 * index,
                vertical=-2.0e-5 * index,
                roll=1.0e-5,
            ),
            "d0": 0.0,
            "rail_shift_yz": np.array([2.0e-5 * index, -1.0e-5 * index]),
            "diagnostic_context": {
                "stage": "Cal",
                "wheelset": wheelset,
                "step_index": 1,
                "iteration": 1,
                "time_s": 1.0e-4,
                "dt_s": 1.0e-4,
            },
        }
        for index, (wheelset, side) in enumerate(
            (wheelset, side)
            for wheelset in ("FF", "FR", "RF", "RR")
            for side in ("L", "R")
        )
    )
    timing: dict[str, float] = {}

    actual = batched_adapter.solve_many(requests, timing=timing)
    expected = tuple(
        scalar_adapter.solve(**request)
        for request in requests
    )

    assert len(actual) == 8
    for actual_geometry, expected_geometry in zip(actual, expected, strict=True):
        actual_history, actual_mask = geometry_to_history(actual_geometry)
        expected_history, expected_mask = geometry_to_history(expected_geometry)
        np.testing.assert_allclose(
            actual_history,
            expected_history,
            rtol=2.0e-12,
            atol=2.0e-10,
        )
        np.testing.assert_array_equal(actual_mask, expected_mask)
    assert timing["network_a1.forward_raw"] >= 0.0
    assert timing["network_a1.adapter_total"] >= timing["network_a1.predict_total"]


def test_accepted_history_orders_all_preload_steps_before_cal() -> None:
    class Dataset:
        metadata = {
            "source": np.array(["coupled_smooth"] * 4),
            "group_id": np.array(["trajectory-1"] * 4),
            "wheelset": np.array(["FF"] * 4),
            "side": np.array(["R"] * 4),
            "stage": np.array(["Preload", "Preload", "Cal", "Cal"]),
            "step_index": np.array([1, 2, 1, 2]),
            "time_s": np.array([1.0e-4, 2.0e-4, 1.0e-4, 2.0e-4]),
        }

        def __len__(self) -> int:
            return 4

    previous = _previous_accepted_indexes(Dataset())
    assert previous.tolist() == [-1, 0, 1, 2]


def test_left_side_canonical_target_transform_is_involutive() -> None:
    target = np.arange(32, dtype=float).reshape(2, 16) / 100.0
    restored = _canonical_side_target_transform(
        _canonical_side_target_transform(target, 0.0),
        0.0,
    )
    np.testing.assert_allclose(restored, target)


def test_closed_loop_keeps_teacher_anchor_at_preload_to_cal_boundary() -> None:
    class Dataset:
        previous_row = np.array([-1, 0, 1])
        features = np.zeros((3, 6), dtype=float)
        history = np.ones((3, 2, 20), dtype=float)
        history_mask = np.ones((3, 2), dtype=bool)
        metadata = {"stage": np.array(["Preload", "Cal", "Cal"])}

        def __len__(self) -> int:
            return 3

    class Model:
        def predict(self, **_: object) -> SimpleNamespace:
            return SimpleNamespace(
                geometry=DirectContactGeometry(
                    has_contact=False,
                    patches=(),
                    topology_probability=np.array([1.0, 0.0, 0.0]),
                    in_distribution=True,
                )
            )

    history, mask = _predicted_previous_histories(Dataset(), Model())
    np.testing.assert_allclose(history[1], 1.0)
    assert mask[1].all()
    np.testing.assert_allclose(history[2], 0.0)
    assert not mask[2].any()


def test_dagger_geometry_error_detects_producer_drift() -> None:
    prediction = _deterministic_single_patch_model().predict(features=np.zeros((6,))).geometry
    patch = prediction.patches[0]
    shifted = replace(
        patch,
        corrected_rail_point=patch.corrected_rail_point + np.array([1.0e-3, 0.0]),
        corrected_vertical_penetration=patch.corrected_vertical_penetration + 2.0e-5,
        contact_angle=patch.contact_angle + 1.0e-2,
        shape_moments=patch.shape_moments + np.array([0.03, 0.0, 0.0]),
    )
    teacher = replace(prediction, patches=(shifted,))
    errors = _geometry_errors(prediction, teacher)
    assert errors["center_error_m"] == pytest.approx(1.0e-3)
    assert errors["penetration_error_m"] == pytest.approx(2.0e-5)
    assert errors["angle_error_rad"] == pytest.approx(1.0e-2)
    assert errors["shape_error"] == pytest.approx(0.03)


def test_runtime_dagger_rows_only_receive_matched_local_geometry_weight() -> None:
    class Dataset:
        patch_count = np.array([1, 1, 1])
        targets = np.zeros((3, 2, 16), dtype=float)
        metadata = {
            "sample_class": np.array(["normal", "normal", "normal"]),
            "source": np.array([
                "coupled_irregular",
                "dagger_runtime_iteration",
                "dagger_runtime_iteration",
            ]),
            "teacher_relabel": np.array([False, True, True]),
            "topology_reference_available": np.array([True, True, True]),
            "topology_matches_accepted_trajectory": np.array([True, True, False]),
        }

        def __len__(self) -> int:
            return 3

    weights = _direct_sample_weights(Dataset(), np.array([0, 1, 2]))
    np.testing.assert_allclose(weights, [1.0, RUNTIME_DAGGER_SAMPLE_WEIGHT, 0.0])
    assert RUNTIME_DAGGER_SAMPLE_WEIGHT <= 1.0


def test_runtime_dagger_rows_never_receive_formal_topology_weight() -> None:
    class Dataset:
        metadata = {
            "source": np.array([
                "full_cal_accepted",
                "dagger_runtime_iteration",
                "sobol",
            ]),
            "stage": np.array(["Cal", "Cal", ""]),
            "accepted_step_label": np.array([True, False, False]),
        }

        def __len__(self) -> int:
            return 3

    weights = _direct_topology_weights(Dataset(), np.arange(3))
    assert weights[0] > weights[2] > weights[1]
    assert weights[1] == 0.0


def test_legacy_dagger_rows_default_to_nonaccepted_audit_labels() -> None:
    dataset = NetworkA1DirectDataset(
        features=np.zeros((1, 6)),
        history=np.zeros((1, 2, 20)),
        history_mask=np.zeros((1, 2), dtype=bool),
        patch_count=np.ones((1,), dtype=np.int8),
        patch_mask=np.array([[True, False]]),
        targets=np.zeros((1, 2, 16)),
        previous_row=np.array([-1]),
        metadata={
            "source": np.array(["dagger"]),
            "converged": np.array([True]),
        },
    )
    normalized = ensure_supervision_metadata(dataset)
    assert not normalized.metadata["accepted_step_label"][0]
    assert not normalized.metadata["teacher_relabel"][0]
    assert not normalized.metadata["topology_reference_available"][0]
    assert not normalized.metadata["topology_matches_accepted_trajectory"][0]


def test_supervision_policy_locks_multiseed_train_and_evaluation_seeds() -> None:
    def dataset(seeds: list[int], split: str) -> NetworkA1DirectDataset:
        n = len(seeds)
        return NetworkA1DirectDataset(
            features=np.zeros((n, 6)),
            history=np.zeros((n, 2, 20)),
            history_mask=np.zeros((n, 2), dtype=bool),
            patch_count=np.ones((n,), dtype=np.int8),
            patch_mask=np.tile(np.array([[True, False]]), (n, 1)),
            targets=np.zeros((n, 2, 16)),
            previous_row=np.full((n,), -1, dtype=np.int64),
            metadata={
                "source": np.array(["full_cal_accepted"] * n),
                "stage": np.array(["Cal"] * n),
                "group_id": np.array([f"{split}-{seed}" for seed in seeds]),
                "irregularity_seed": np.asarray(seeds),
                "accepted_step_label": np.ones((n,), dtype=bool),
                "teacher_relabel": np.zeros((n,), dtype=bool),
                "topology_reference_available": np.ones((n,), dtype=bool),
                "topology_matches_accepted_trajectory": np.ones((n,), dtype=bool),
            },
        )

    train = dataset([20260716, 20260717], "train")
    validation = dataset([20260721], "validation")
    test = dataset([20260722], "test")
    summary = _validate_supervision_policy(train, validation, test)
    assert summary["full_cal_training_seeds"] == [20260716, 20260717]
    assert summary["evaluation_seeds"] == {
        "validation": 20260721,
        "test": 20260722,
    }
    with pytest.raises(ValueError, match="validation split must contain only"):
        _validate_supervision_policy(train, dataset([20260720], "bad-validation"), test)


def test_direct_dataset_concatenation_offsets_only_internal_history_links() -> None:
    def dataset(group: str) -> NetworkA1DirectDataset:
        return NetworkA1DirectDataset(
            features=np.zeros((2, 6)),
            history=np.zeros((2, 2, 20)),
            history_mask=np.zeros((2, 2), dtype=bool),
            patch_count=np.zeros((2,), dtype=np.int8),
            patch_mask=np.zeros((2, 2), dtype=bool),
            targets=np.zeros((2, 2, 16)),
            previous_row=np.array([-1, 0]),
            metadata={
                "source": np.array(["coupled_irregular"] * 2),
                "group_id": np.array([group] * 2),
            },
        )

    combined = concatenate_direct_datasets((dataset("first"), dataset("second")))
    assert combined.previous_row.tolist() == [-1, 0, -1, 2]


def test_rebalanced_dataset_keeps_only_requested_long_stage_steps(tmp_path) -> None:
    def dataset(groups: list[str], steps: list[int]) -> NetworkA1DirectDataset:
        n = len(groups)
        return NetworkA1DirectDataset(
            features=np.zeros((n, 6)),
            history=np.zeros((n, 2, 20)),
            history_mask=np.zeros((n, 2), dtype=bool),
            patch_count=np.zeros((n,), dtype=np.int8),
            patch_mask=np.zeros((n, 2), dtype=bool),
            targets=np.zeros((n, 2, 16)),
            previous_row=np.full((n,), -1, dtype=np.int64),
            metadata={
                "source": np.array(["coupled_irregular"] * n),
                "group_id": np.asarray(groups),
                "step_index": np.asarray(steps),
            },
        )

    base_dir = tmp_path / "base"
    candidate_dir = tmp_path / "candidate"
    for directory in (base_dir, candidate_dir):
        directory.mkdir()
    save_direct_dataset(base_dir / "train.npz", dataset(["base"], [1]))
    save_direct_dataset(base_dir / "validation.npz", dataset(["validation"], [1]))
    save_direct_dataset(base_dir / "test.npz", dataset(["test"], [1]))
    save_direct_dataset(
        candidate_dir / "train.npz",
        dataset(["stage-steps-200"] * 4, [1, 2, 3, 4]),
    )

    output = tmp_path / "rebalanced"
    build_rebalanced_dataset(
        base_dataset_dir=base_dir,
        candidate_dataset_dir=candidate_dir,
        output_dir=output,
        group_substring="stage-steps-200",
        retained_step_indexes=(1, 4),
    )
    result = load_direct_dataset(output / "train.npz")
    assert len(result) == 3
    assert result.metadata["step_index"].tolist() == [1, 1, 4]


def test_traditional_geometry_conversion_adds_grid_independent_shape_moments() -> None:
    elastic = np.array([[-0.01, 0.0], [0.0, 1.0e-4], [0.01, 0.0]], dtype=float)
    patch = ContactPatch(
        peak_index=1,
        start_index=0,
        end_index=2,
        peak_wheel_point=np.array([0.0, 0.0, 0.6001]),
        peak_rail_point=np.array([0.0, 0.6]),
        corrected_wheel_point=np.array([0.0, 0.0, 0.60008]),
        corrected_rail_point=np.array([0.0, 0.6]),
        wheel_profile_lateral=0.0,
        peak_vertical_penetration=1.0e-4,
        peak_normal_penetration=1.0e-4,
        peak_contact_angle=0.0,
        corrected_vertical_penetration=8.0e-5,
        corrected_normal_penetration=8.0e-5,
        contact_angle=0.0,
    )
    empty = np.zeros((0, 4), dtype=float)
    geometry = MultiPointContactGeometry(
        has_contact=True,
        elastic_penetration=elastic,
        wheel_interp=np.zeros((3, 3)),
        rail_interp=np.zeros((3, 2)),
        contact_angles=np.zeros((3,)),
        wheel_profile_lateral=np.zeros((3,)),
        boundaries=BoundaryExtrema(empty, empty, empty, empty),
        patches=(patch,),
    )
    direct = direct_geometry_from_traditional(geometry)
    assert direct.patches[0].shape_moments.shape == (3,)
    assert np.all((direct.patches[0].shape_moments > 0.0) & (direct.patches[0].shape_moments < 1.0))
    history, mask = geometry_to_history(direct)
    assert history.shape == (2, 20)
    assert mask.tolist() == [True, False]
