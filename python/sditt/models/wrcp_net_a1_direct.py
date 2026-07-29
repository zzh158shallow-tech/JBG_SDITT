from __future__ import annotations

import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Mapping

import numpy as np
from scipy.optimize import brentq

from sditt.contact.geometry import DirectContactGeometry, DirectContactPatch, WheelPose2D


MODEL_NAME = "WRCP-Net A1 Direct Set"
MODEL_SCHEMA = "wrcp-net-a1-direct-set-v1"
MAX_PATCHES = 2
FEATURE_NAMES = (
    "side_id",
    "delta_y_m",
    "delta_z_m",
    "roll_rad",
    "yaw_rad",
    "d0_m",
)
HISTORY_NAMES = (
    "patch_start_y_m",
    "patch_end_y_m",
    "corrected_wheel_x_m",
    "corrected_wheel_y_m",
    "corrected_wheel_z_m",
    "corrected_rail_y_m",
    "corrected_rail_z_m",
    "wheel_profile_lateral_m",
    "corrected_vertical_penetration_m",
    "corrected_contact_angle_rad",
    "peak_wheel_x_m",
    "peak_wheel_y_m",
    "peak_wheel_z_m",
    "peak_rail_y_m",
    "peak_rail_z_m",
    "peak_vertical_penetration_m",
    "peak_contact_angle_rad",
    "shape_moment_1",
    "shape_moment_1p5",
    "shape_moment_2",
)
TARGET_NAMES = (
    "center_y_m",
    "left_half_width_softplus_inverse",
    "right_half_width_softplus_inverse",
    "corrected_wheel_x_m",
    "corrected_rail_z_m",
    "wheel_profile_lateral_m",
    "corrected_vertical_penetration_softplus_inverse",
    "corrected_contact_angle_rad",
    "peak_fraction_logit",
    "peak_wheel_x_m",
    "peak_rail_z_m",
    "peak_penetration_increment_softplus_inverse",
    "peak_contact_angle_rad",
    "shape_moment_1_logit",
    "shape_moment_1p5_logit",
    "shape_moment_2_logit",
)
QUERY_OUTPUT_SIZE = 1 + len(TARGET_NAMES)
ACCEPTED_INCREMENT_NAMES = (
    "center_y_m",
    "left_half_width_m",
    "right_half_width_m",
    "corrected_vertical_penetration_m",
    "peak_rail_y_m",
    "peak_penetration_increment_m",
    "shape_moment_1",
    "shape_moment_1p5",
    "shape_moment_2",
)


@dataclass(frozen=True)
class WRCPNetA1DirectPrediction:
    geometry: DirectContactGeometry
    features: np.ndarray
    normalized_feature_distance: float
    normalized_history_distance: float
    topology_probability: np.ndarray
    query_probability: np.ndarray
    decoded_targets: np.ndarray


@dataclass(frozen=True)
class WRCPNetA1DirectSet:
    state_weights: tuple[np.ndarray, ...]
    state_biases: tuple[np.ndarray, ...]
    history_weights: tuple[np.ndarray, ...]
    history_biases: tuple[np.ndarray, ...]
    fusion_weight: np.ndarray
    fusion_bias: np.ndarray
    residual_weights: tuple[tuple[np.ndarray, np.ndarray], ...]
    residual_biases: tuple[tuple[np.ndarray, np.ndarray], ...]
    topology_weight: np.ndarray
    topology_bias: np.ndarray
    query_embeddings: np.ndarray
    decoder_weights: tuple[np.ndarray, ...]
    decoder_biases: tuple[np.ndarray, ...]
    feature_mean: np.ndarray
    feature_scale: np.ndarray
    history_mean: np.ndarray
    history_scale: np.ndarray
    target_mean: np.ndarray
    target_scale: np.ndarray
    feature_ood_scale: np.ndarray | None = None
    history_ood_scale: np.ndarray | None = None
    accepted_step_increment_limits: np.ndarray | None = None
    shape_moment_min: np.ndarray | None = None
    shape_moment_max: np.ndarray | None = None
    shape_moment_history_blend: float = 1.0
    topology_hysteresis_min: float = 0.0
    canonical_side_targets: bool = False
    topology_confidence_min: float = 0.95
    ood_threshold: float = 4.0
    angle_min_rad: float = -1.45
    angle_max_rad: float = 1.45
    minimum_patch_separation_m: float = 1.0e-6
    wheel_trace_profile_left: np.ndarray | None = None
    wheel_trace_profile_right: np.ndarray | None = None
    rail_profile_left: np.ndarray | None = None
    rail_profile_right: np.ndarray | None = None

    def forward_raw(
        self,
        features: np.ndarray,
        history: np.ndarray,
        history_mask: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        x = np.asarray(features, dtype=float).reshape(len(FEATURE_NAMES))
        h = np.asarray(history, dtype=float).reshape(MAX_PATCHES, len(HISTORY_NAMES))
        mask = np.asarray(history_mask, dtype=bool).reshape(MAX_PATCHES)
        x_normalized = (x - self.feature_mean) / self.feature_scale
        h_normalized = (h - self.history_mean) / self.history_scale
        h_normalized = np.where(mask[:, None], h_normalized, 0.0)

        state = x_normalized
        for weight, bias in zip(self.state_weights, self.state_biases, strict=True):
            state = _silu(state @ weight + bias)
        encoded_history = h_normalized
        for weight, bias in zip(self.history_weights, self.history_biases, strict=True):
            encoded_history = _silu(encoded_history @ weight + bias)
        pooled_history = np.sum(encoded_history * mask[:, None], axis=0)
        fused = np.concatenate((state, pooled_history, [float(np.sum(mask)) / MAX_PATCHES]))
        latent = _silu(fused @ self.fusion_weight + self.fusion_bias)
        for (weight_1, weight_2), (bias_1, bias_2) in zip(
            self.residual_weights,
            self.residual_biases,
            strict=True,
        ):
            residual = _silu(latent @ weight_1 + bias_1)
            latent = _silu(latent + residual @ weight_2 + bias_2)

        topology_logits = latent @ self.topology_weight + self.topology_bias
        query_input = np.concatenate(
            (
                np.repeat(latent[None, :], MAX_PATCHES, axis=0),
                self.query_embeddings,
            ),
            axis=1,
        )
        decoded = query_input
        for index, (weight, bias) in enumerate(
            zip(self.decoder_weights, self.decoder_biases, strict=True)
        ):
            decoded = decoded @ weight + bias
            if index + 1 < len(self.decoder_weights):
                decoded = _silu(decoded)
        return np.asarray(topology_logits, dtype=float), np.asarray(decoded, dtype=float)

    def forward_raw_batch(
        self,
        features: np.ndarray,
        history: np.ndarray,
        history_mask: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        x = np.asarray(features, dtype=float).reshape(-1, len(FEATURE_NAMES))
        h = np.asarray(history, dtype=float).reshape(
            x.shape[0],
            MAX_PATCHES,
            len(HISTORY_NAMES),
        )
        mask = np.asarray(history_mask, dtype=bool).reshape(
            x.shape[0],
            MAX_PATCHES,
        )
        x_normalized = (x - self.feature_mean) / self.feature_scale
        h_normalized = (h - self.history_mean) / self.history_scale
        h_normalized = np.where(mask[:, :, None], h_normalized, 0.0)

        state = x_normalized
        for weight, bias in zip(self.state_weights, self.state_biases, strict=True):
            state = _silu(state @ weight + bias)
        encoded_history = h_normalized
        for weight, bias in zip(self.history_weights, self.history_biases, strict=True):
            encoded_history = _silu(encoded_history @ weight + bias)
        pooled_history = np.sum(
            encoded_history * mask[:, :, None],
            axis=1,
        )
        fused = np.concatenate(
            (
                state,
                pooled_history,
                np.sum(mask, axis=1, keepdims=True) / MAX_PATCHES,
            ),
            axis=1,
        )
        latent = _silu(fused @ self.fusion_weight + self.fusion_bias)
        for (weight_1, weight_2), (bias_1, bias_2) in zip(
            self.residual_weights,
            self.residual_biases,
            strict=True,
        ):
            residual = _silu(latent @ weight_1 + bias_1)
            latent = _silu(latent + residual @ weight_2 + bias_2)

        topology_logits = latent @ self.topology_weight + self.topology_bias
        query_input = np.concatenate(
            (
                np.repeat(latent[:, None, :], MAX_PATCHES, axis=1),
                np.broadcast_to(
                    self.query_embeddings[None, :, :],
                    (x.shape[0],) + self.query_embeddings.shape,
                ),
            ),
            axis=2,
        )
        decoded = query_input
        for index, (weight, bias) in enumerate(
            zip(self.decoder_weights, self.decoder_biases, strict=True)
        ):
            decoded = decoded @ weight + bias
            if index + 1 < len(self.decoder_weights):
                decoded = _silu(decoded)
        return (
            np.asarray(topology_logits, dtype=float),
            np.asarray(decoded, dtype=float),
        )

    def predict(
        self,
        *,
        features: np.ndarray,
        history: np.ndarray | None = None,
        history_mask: np.ndarray | None = None,
        topology_prior_available: bool | None = None,
        previous_patch_count: int | None = None,
        timing: dict[str, float] | None = None,
        _precomputed_raw_output: tuple[np.ndarray, np.ndarray] | None = None,
        _precomputed_distances: tuple[float, float] | None = None,
        _record_input_timing: bool = True,
        _record_total_timing: bool = True,
    ) -> WRCPNetA1DirectPrediction:
        total_started = time.perf_counter() if timing is not None else None
        phase_started = time.perf_counter() if timing is not None else None
        x = np.asarray(features, dtype=float).reshape(len(FEATURE_NAMES))
        h = (
            np.zeros((MAX_PATCHES, len(HISTORY_NAMES)), dtype=float)
            if history is None
            else np.asarray(history, dtype=float).reshape(MAX_PATCHES, len(HISTORY_NAMES))
        )
        mask = (
            np.zeros((MAX_PATCHES,), dtype=bool)
            if history_mask is None
            else np.asarray(history_mask, dtype=bool).reshape(MAX_PATCHES)
        )
        if not np.isfinite(x).all() or not np.isfinite(h[mask]).all():
            raise RuntimeError("WRCP-Net A1 Direct input contains NaN or Inf")
        if _precomputed_distances is None:
            feature_distance = float(
                np.max(
                    np.abs((x - self.feature_mean) / self.feature_scale)
                    / _runtime_ood_scale(self.feature_ood_scale, len(FEATURE_NAMES))
                )
            )
            history_distance = (
                0.0
                if not np.any(mask)
                else float(
                    np.max(
                        np.abs((h[mask] - self.history_mean) / self.history_scale)
                        / _runtime_ood_scale(
                            self.history_ood_scale,
                            len(HISTORY_NAMES),
                        )
                    )
                )
            )
        else:
            feature_distance, history_distance = (
                float(_precomputed_distances[0]),
                float(_precomputed_distances[1]),
            )
        if feature_distance > self.ood_threshold or history_distance > self.ood_threshold:
            raise RuntimeError(
                "WRCP-Net A1 Direct input is outside its calibrated distribution: "
                f"feature_distance={feature_distance:.6g}, history_distance={history_distance:.6g}, "
                f"threshold={self.ood_threshold:.6g}"
            )
        if _record_input_timing:
            _record_timing(timing, "network_a1.input_ood", phase_started)

        phase_started = time.perf_counter() if timing is not None else None
        if _precomputed_raw_output is None:
            topology_logits, query_raw = self.forward_raw(x, h, mask)
            _record_timing(timing, "network_a1.forward_raw", phase_started)
        else:
            topology_logits = np.asarray(
                _precomputed_raw_output[0],
                dtype=float,
            )
            query_raw = np.asarray(
                _precomputed_raw_output[1],
                dtype=float,
            )
        phase_started = time.perf_counter() if timing is not None else None
        topology_probability = _softmax(topology_logits)
        query_probability = _sigmoid(query_raw[:, 0])
        patch_count = int(np.argmax(topology_probability))
        query_patch_count = int(np.sum(query_probability >= 0.5))
        confidence = float(np.max(topology_probability))
        inferred_patch_count = int(np.sum(mask))
        previous_patch_count = (
            inferred_patch_count
            if previous_patch_count is None
            else int(previous_patch_count)
        )
        if previous_patch_count != inferred_patch_count:
            raise ValueError("previous patch count does not match accepted history mask")
        topology_prior_available = (
            bool(np.any(mask))
            if topology_prior_available is None
            else bool(topology_prior_available)
        )
        topology_branch_retained = bool(
            topology_prior_available and patch_count == previous_patch_count
        )
        topology_transition_suppressed = bool(
            topology_prior_available
            and
            self.topology_hysteresis_min > 0.0
            and patch_count != previous_patch_count
            and (
                topology_probability[patch_count] < self.topology_hysteresis_min
                or query_patch_count != patch_count
            )
        )
        if topology_transition_suppressed:
            patch_count = previous_patch_count
        if confidence < self.topology_confidence_min and not (
            topology_branch_retained or topology_transition_suppressed
        ):
            raise RuntimeError(
                "WRCP-Net A1 Direct topology confidence is below its calibrated minimum: "
                f"confidence={confidence:.6g}, minimum={self.topology_confidence_min:.6g}"
            )
        decoded_targets = query_raw[:, 1:] * self.target_scale + self.target_mean
        if self.canonical_side_targets:
            decoded_targets = _canonical_side_target_transform(decoded_targets, x[0])
        if topology_transition_suppressed:
            _record_timing(timing, "network_a1.topology_history", phase_started)
            phase_started = time.perf_counter() if timing is not None else None
            geometry = _geometry_from_accepted_history(
                history=h,
                history_mask=mask,
                topology_probability=topology_probability,
                roll=x[3],
            )
            geometry = _bound_direct_geometry(
                geometry,
                roll=x[3],
                angle_min_rad=self.angle_min_rad,
                angle_max_rad=self.angle_max_rad,
                minimum_patch_separation_m=self.minimum_patch_separation_m,
                shape_moment_min=self.shape_moment_min,
                shape_moment_max=self.shape_moment_max,
            )
            validate_direct_geometry(
                geometry,
                roll=x[3],
                angle_min_rad=self.angle_min_rad,
                angle_max_rad=self.angle_max_rad,
                minimum_patch_separation_m=self.minimum_patch_separation_m,
            )
            prediction = WRCPNetA1DirectPrediction(
                geometry=geometry,
                features=x.copy(),
                normalized_feature_distance=feature_distance,
                normalized_history_distance=history_distance,
                topology_probability=topology_probability,
                query_probability=query_probability,
                decoded_targets=decoded_targets,
            )
            _record_timing(timing, "network_a1.decode_validate", phase_started)
            if _record_total_timing:
                _record_timing(timing, "network_a1.predict_total", total_started)
            return prediction
        selected = np.argsort(-query_probability, kind="stable")[:patch_count]
        if self.accepted_step_increment_limits is not None and patch_count:
            if not self.has_fixed_profile_projection:
                raise RuntimeError(
                    "WRCP-Net A1 Direct accepted-step increment limiting requires fixed profiles"
                )
            decoded_targets = _limit_targets_to_accepted_history(
                decoded_targets,
                selected=selected,
                history=h,
                history_mask=mask,
                limits=self.accepted_step_increment_limits,
            )
        _record_timing(timing, "network_a1.topology_history", phase_started)
        phase_started = time.perf_counter() if timing is not None else None
        if self.has_fixed_profile_projection:
            decoded_targets = decoded_targets.copy()
            for index in selected:
                decoded_targets[index] = _project_target_to_fixed_profiles(
                    decoded_targets[index],
                    features=x,
                    wheel_trace_profile=(
                        self.wheel_trace_profile_left
                        if x[0] < 0.5
                        else self.wheel_trace_profile_right
                    ),
                    rail_profile=(
                        self.rail_profile_left
                        if x[0] < 0.5
                        else self.rail_profile_right
                    ),
                )
        _record_timing(timing, "network_a1.fixed_profile_projection", phase_started)
        phase_started = time.perf_counter() if timing is not None else None
        patches = tuple(
            _decode_patch(decoded_targets[index], delta_z=x[2], roll=x[3], d0=x[5])
            for index in selected
        )
        patches = tuple(sorted(patches, key=lambda patch: patch.corrected_rail_point[0]))
        geometry = DirectContactGeometry(
            has_contact=bool(patches),
            patches=patches,
            topology_probability=topology_probability,
            in_distribution=True,
        )
        geometry = _blend_shape_moments_with_accepted_history(
            geometry,
            history=h,
            history_mask=mask,
            new_prediction_weight=self.shape_moment_history_blend,
        )
        geometry = _bound_direct_geometry(
            geometry,
            roll=x[3],
            angle_min_rad=self.angle_min_rad,
            angle_max_rad=self.angle_max_rad,
            minimum_patch_separation_m=self.minimum_patch_separation_m,
            shape_moment_min=self.shape_moment_min,
            shape_moment_max=self.shape_moment_max,
        )
        validate_direct_geometry(
            geometry,
            roll=x[3],
            angle_min_rad=self.angle_min_rad,
            angle_max_rad=self.angle_max_rad,
            minimum_patch_separation_m=self.minimum_patch_separation_m,
        )
        prediction = WRCPNetA1DirectPrediction(
            geometry=geometry,
            features=x.copy(),
            normalized_feature_distance=feature_distance,
            normalized_history_distance=history_distance,
            topology_probability=topology_probability,
            query_probability=query_probability,
            decoded_targets=decoded_targets,
        )
        _record_timing(timing, "network_a1.decode_validate", phase_started)
        if _record_total_timing:
            _record_timing(timing, "network_a1.predict_total", total_started)
        return prediction

    def predict_batch(
        self,
        *,
        features: np.ndarray,
        history: np.ndarray,
        history_mask: np.ndarray,
        topology_prior_available: np.ndarray | None = None,
        previous_patch_count: np.ndarray | None = None,
        timing: dict[str, float] | None = None,
    ) -> tuple[WRCPNetA1DirectPrediction, ...]:
        total_started = time.perf_counter() if timing is not None else None
        phase_started = time.perf_counter() if timing is not None else None
        x = np.asarray(features, dtype=float).reshape(-1, len(FEATURE_NAMES))
        h = np.asarray(history, dtype=float).reshape(
            x.shape[0],
            MAX_PATCHES,
            len(HISTORY_NAMES),
        )
        mask = np.asarray(history_mask, dtype=bool).reshape(
            x.shape[0],
            MAX_PATCHES,
        )
        if not np.isfinite(x).all() or not np.isfinite(h[mask]).all():
            raise RuntimeError("WRCP-Net A1 Direct batch input contains NaN or Inf")
        feature_distance = np.max(
            np.abs((x - self.feature_mean) / self.feature_scale)
            / _runtime_ood_scale(self.feature_ood_scale, len(FEATURE_NAMES)),
            axis=1,
        )
        history_scaled = (
            np.abs((h - self.history_mean) / self.history_scale)
            / _runtime_ood_scale(self.history_ood_scale, len(HISTORY_NAMES))
        )
        history_scaled = np.where(mask[:, :, None], history_scaled, 0.0)
        history_distance = np.max(history_scaled, axis=(1, 2))
        outside = np.flatnonzero(
            (feature_distance > self.ood_threshold)
            | (history_distance > self.ood_threshold)
        )
        if outside.size:
            index = int(outside[0])
            raise RuntimeError(
                "WRCP-Net A1 Direct batch input is outside its calibrated distribution: "
                f"index={index}, feature_distance={feature_distance[index]:.6g}, "
                f"history_distance={history_distance[index]:.6g}, "
                f"threshold={self.ood_threshold:.6g}"
            )
        _record_timing(timing, "network_a1.input_ood", phase_started)

        phase_started = time.perf_counter() if timing is not None else None
        topology_logits, query_raw = self.forward_raw_batch(x, h, mask)
        _record_timing(timing, "network_a1.forward_raw", phase_started)
        prior = (
            np.any(mask, axis=1)
            if topology_prior_available is None
            else np.asarray(topology_prior_available, dtype=bool).reshape(x.shape[0])
        )
        prior_count = (
            np.sum(mask, axis=1)
            if previous_patch_count is None
            else np.asarray(previous_patch_count, dtype=int).reshape(x.shape[0])
        )
        predictions = tuple(
            self.predict(
                features=x[index],
                history=h[index],
                history_mask=mask[index],
                topology_prior_available=bool(prior[index]),
                previous_patch_count=int(prior_count[index]),
                timing=timing,
                _precomputed_raw_output=(
                    topology_logits[index],
                    query_raw[index],
                ),
                _precomputed_distances=(
                    float(feature_distance[index]),
                    float(history_distance[index]),
                ),
                _record_input_timing=False,
                _record_total_timing=False,
            )
            for index in range(x.shape[0])
        )
        _record_timing(timing, "network_a1.predict_total", total_started)
        return predictions

    @property
    def has_fixed_profile_projection(self) -> bool:
        """Whether the artifact can deterministically project final point fields.

        The compact tables are fixed-model data.  Runtime only solves two
        scalar inverse mappings per retained patch; it never reconstructs a
        gap field or performs a candidate contact search.
        """

        values = (
            self.wheel_trace_profile_left,
            self.wheel_trace_profile_right,
            self.rail_profile_left,
            self.rail_profile_right,
        )
        if all(value is None for value in values):
            return False
        if any(value is None for value in values):
            raise RuntimeError("WRCP-Net A1 Direct fixed-profile projection tables are incomplete")
        return True


def initialize_wrcp_net_a1_direct(
    *,
    seed: int = 20260721,
    feature_mean: np.ndarray | None = None,
    feature_scale: np.ndarray | None = None,
    history_mean: np.ndarray | None = None,
    history_scale: np.ndarray | None = None,
    target_mean: np.ndarray | None = None,
    target_scale: np.ndarray | None = None,
    accepted_step_increment_limits: np.ndarray | None = None,
    topology_hysteresis_min: float = 0.0,
) -> WRCPNetA1DirectSet:
    """Create the exact runtime architecture used by the optional trainer."""

    rng = np.random.default_rng(seed)

    def weight(fan_in: int, fan_out: int) -> np.ndarray:
        return rng.normal(0.0, np.sqrt(2.0 / fan_in), size=(fan_in, fan_out)).astype(np.float32)

    state_weights = (weight(len(FEATURE_NAMES), 128), weight(128, 128))
    history_weights = (weight(len(HISTORY_NAMES), 128), weight(128, 128))
    residual_weights = tuple((weight(256, 256), weight(256, 256)) for _ in range(3))
    decoder_weights = (weight(256 + 32, 128), weight(128, 128), weight(128, QUERY_OUTPUT_SIZE))
    return WRCPNetA1DirectSet(
        state_weights=state_weights,
        state_biases=tuple(np.zeros((value.shape[1],), dtype=np.float32) for value in state_weights),
        history_weights=history_weights,
        history_biases=tuple(np.zeros((value.shape[1],), dtype=np.float32) for value in history_weights),
        fusion_weight=weight(257, 256),
        fusion_bias=np.zeros((256,), dtype=np.float32),
        residual_weights=residual_weights,
        residual_biases=tuple(
            (np.zeros((256,), dtype=np.float32), np.zeros((256,), dtype=np.float32))
            for _ in residual_weights
        ),
        topology_weight=weight(256, 3),
        topology_bias=np.zeros((3,), dtype=np.float32),
        query_embeddings=rng.normal(0.0, 0.02, size=(MAX_PATCHES, 32)).astype(np.float32),
        decoder_weights=decoder_weights,
        decoder_biases=tuple(np.zeros((value.shape[1],), dtype=np.float32) for value in decoder_weights),
        feature_mean=_default_vector(feature_mean, len(FEATURE_NAMES), 0.0),
        feature_scale=_default_vector(feature_scale, len(FEATURE_NAMES), 1.0, positive=True),
        history_mean=_default_vector(history_mean, len(HISTORY_NAMES), 0.0),
        history_scale=_default_vector(history_scale, len(HISTORY_NAMES), 1.0, positive=True),
        target_mean=_default_vector(target_mean, len(TARGET_NAMES), 0.0),
        target_scale=_default_vector(target_scale, len(TARGET_NAMES), 1.0, positive=True),
        feature_ood_scale=np.ones((len(FEATURE_NAMES),), dtype=float),
        history_ood_scale=np.ones((len(HISTORY_NAMES),), dtype=float),
        accepted_step_increment_limits=(
            None
            if accepted_step_increment_limits is None
            else _positive_vector(
                accepted_step_increment_limits,
                len(ACCEPTED_INCREMENT_NAMES),
                "accepted-step increment limits",
            )
        ),
        topology_hysteresis_min=float(topology_hysteresis_min),
    )


def accepted_increment_state_from_history(history: np.ndarray) -> np.ndarray:
    """Return the minimal physical state used for accepted-step limiting."""

    values = np.asarray(history, dtype=float)
    if values.shape[-1] != len(HISTORY_NAMES):
        raise ValueError("accepted history has an incompatible final dimension")
    start = values[..., 0]
    end = values[..., 1]
    center = values[..., 5]
    return np.stack(
        (
            center,
            np.maximum(center - start, 1.0e-12),
            np.maximum(end - center, 1.0e-12),
            np.maximum(values[..., 8], 1.0e-12),
            values[..., 13],
            np.maximum(values[..., 15] - values[..., 8], 1.0e-12),
            *np.moveaxis(np.clip(values[..., 17:20], 1.0e-7, 1.0 - 1.0e-7), -1, 0),
        ),
        axis=-1,
    )


def accepted_increment_state_from_targets(targets: np.ndarray) -> np.ndarray:
    values = np.asarray(targets, dtype=float)
    if values.shape[-1] != len(TARGET_NAMES):
        raise ValueError("direct targets have an incompatible final dimension")
    center = values[..., 0]
    left_width = _softplus(values[..., 1]) + 1.0e-9
    right_width = _softplus(values[..., 2]) + 1.0e-9
    peak_y = center - left_width + _sigmoid(values[..., 8]) * (
        left_width + right_width
    )
    return np.stack(
        (
            center,
            left_width,
            right_width,
            _softplus(values[..., 6]) + 1.0e-12,
            peak_y,
            _softplus(values[..., 11]) + 1.0e-12,
            *np.moveaxis(_sigmoid(values[..., 13:16]), -1, 0),
        ),
        axis=-1,
    )


def _limit_targets_to_accepted_history(
    decoded_targets: np.ndarray,
    *,
    selected: np.ndarray,
    history: np.ndarray,
    history_mask: np.ndarray,
    limits: np.ndarray,
) -> np.ndarray:
    result = np.asarray(decoded_targets, dtype=float).copy()
    selected_indexes = np.asarray(selected, dtype=int)
    previous_indexes = np.flatnonzero(np.asarray(history_mask, dtype=bool))
    if selected_indexes.size == 0 or selected_indexes.size != previous_indexes.size:
        return result
    bounds = _positive_vector(
        limits,
        len(ACCEPTED_INCREMENT_NAMES),
        "accepted-step increment limits",
    )
    predicted_state = accepted_increment_state_from_targets(result[selected_indexes])
    previous_state = accepted_increment_state_from_history(
        np.asarray(history, dtype=float)[previous_indexes]
    )
    if selected_indexes.size == 2:
        direct = abs(predicted_state[0, 0] - previous_state[0, 0]) + abs(
            predicted_state[1, 0] - previous_state[1, 0]
        )
        swapped = abs(predicted_state[0, 0] - previous_state[1, 0]) + abs(
            predicted_state[1, 0] - previous_state[0, 0]
        )
        if swapped < direct:
            previous_state = previous_state[::-1]
    limited = previous_state + np.clip(predicted_state - previous_state, -bounds, bounds)
    for row, query_index in enumerate(selected_indexes):
        target = result[query_index]
        state = limited[row]
        target[0] = state[0]
        target[1] = _softplus_inverse(state[1])
        target[2] = _softplus_inverse(state[2])
        target[6] = _softplus_inverse(state[3])
        start = state[0] - state[1]
        width = max(state[1] + state[2], 2.0e-12)
        peak_fraction = np.clip((state[4] - start) / width, 1.0e-7, 1.0 - 1.0e-7)
        target[8] = _logit(peak_fraction)
        target[11] = _softplus_inverse(state[5])
        target[13:16] = _logit(state[6:9])
    return result


def patch_to_history_vector(patch: DirectContactPatch) -> np.ndarray:
    return np.array(
        [
            patch.start_y,
            patch.end_y,
            *np.asarray(patch.corrected_wheel_point, dtype=float),
            *np.asarray(patch.corrected_rail_point, dtype=float),
            patch.wheel_profile_lateral,
            patch.corrected_vertical_penetration,
            patch.contact_angle,
            *np.asarray(patch.peak_wheel_point, dtype=float),
            *np.asarray(patch.peak_rail_point, dtype=float),
            patch.peak_vertical_penetration,
            patch.peak_contact_angle,
            *np.asarray(patch.shape_moments, dtype=float),
        ],
        dtype=float,
    )


def geometry_to_history(
    geometry: DirectContactGeometry | None,
) -> tuple[np.ndarray, np.ndarray]:
    values = np.zeros((MAX_PATCHES, len(HISTORY_NAMES)), dtype=float)
    mask = np.zeros((MAX_PATCHES,), dtype=bool)
    if geometry is None:
        return values, mask
    for index, patch in enumerate(geometry.patches[:MAX_PATCHES]):
        values[index] = patch_to_history_vector(patch)
        mask[index] = True
    return values, mask


def _geometry_from_accepted_history(
    *,
    history: np.ndarray,
    history_mask: np.ndarray,
    topology_probability: np.ndarray,
    roll: float,
) -> DirectContactGeometry:
    values = np.asarray(history, dtype=float)
    indexes = np.flatnonzero(np.asarray(history_mask, dtype=bool))
    patches = []
    for index in indexes:
        row = values[index]
        corrected_angle = float(row[9])
        peak_angle = float(row[16])
        corrected_cosine = max(abs(float(np.cos(corrected_angle + roll))), 1.0e-3)
        peak_cosine = max(abs(float(np.cos(peak_angle + roll))), 1.0e-3)
        patches.append(
            DirectContactPatch(
                start_y=float(row[0]),
                end_y=float(row[1]),
                corrected_wheel_point=np.asarray(row[2:5], dtype=float).copy(),
                corrected_rail_point=np.asarray(row[5:7], dtype=float).copy(),
                wheel_profile_lateral=float(row[7]),
                corrected_vertical_penetration=float(row[8]),
                corrected_normal_penetration=float(row[8] / corrected_cosine),
                contact_angle=corrected_angle,
                peak_wheel_point=np.asarray(row[10:13], dtype=float).copy(),
                peak_rail_point=np.asarray(row[13:15], dtype=float).copy(),
                peak_vertical_penetration=float(row[15]),
                peak_normal_penetration=float(row[15] / peak_cosine),
                peak_contact_angle=peak_angle,
                shape_moments=np.asarray(row[17:20], dtype=float).copy(),
            )
        )
    patches.sort(key=lambda patch: patch.corrected_rail_point[0])
    return DirectContactGeometry(
        has_contact=bool(patches),
        patches=tuple(patches),
        topology_probability=np.asarray(topology_probability, dtype=float).copy(),
        in_distribution=True,
    )


def shift_direct_geometry(
    geometry: DirectContactGeometry,
    rail_shift_yz: np.ndarray,
) -> DirectContactGeometry:
    shift = np.asarray(rail_shift_yz, dtype=float).reshape(2)

    def shift_patch(patch: DirectContactPatch) -> DirectContactPatch:
        corrected_wheel = np.asarray(patch.corrected_wheel_point, dtype=float).copy()
        peak_wheel = np.asarray(patch.peak_wheel_point, dtype=float).copy()
        corrected_rail = np.asarray(patch.corrected_rail_point, dtype=float).copy()
        peak_rail = np.asarray(patch.peak_rail_point, dtype=float).copy()
        corrected_wheel[1] += shift[0]
        peak_wheel[1] += shift[0]
        corrected_rail += shift
        peak_rail += shift
        return DirectContactPatch(
            start_y=float(patch.start_y + shift[0]),
            end_y=float(patch.end_y + shift[0]),
            peak_wheel_point=peak_wheel,
            peak_rail_point=peak_rail,
            corrected_wheel_point=corrected_wheel,
            corrected_rail_point=corrected_rail,
            wheel_profile_lateral=float(patch.wheel_profile_lateral),
            peak_vertical_penetration=float(patch.peak_vertical_penetration),
            peak_normal_penetration=float(patch.peak_normal_penetration),
            peak_contact_angle=float(patch.peak_contact_angle),
            corrected_vertical_penetration=float(patch.corrected_vertical_penetration),
            corrected_normal_penetration=float(patch.corrected_normal_penetration),
            contact_angle=float(patch.contact_angle),
            shape_moments=np.asarray(patch.shape_moments, dtype=float).copy(),
        )

    return DirectContactGeometry(
        has_contact=geometry.has_contact,
        patches=tuple(shift_patch(patch) for patch in geometry.patches),
        topology_probability=np.asarray(geometry.topology_probability, dtype=float).copy(),
        in_distribution=geometry.in_distribution,
    )


def validate_direct_geometry(
    geometry: DirectContactGeometry,
    *,
    roll: float,
    angle_min_rad: float = -1.45,
    angle_max_rad: float = 1.45,
    minimum_patch_separation_m: float = 1.0e-6,
) -> None:
    if len(geometry.patches) > MAX_PATCHES or geometry.has_contact != bool(geometry.patches):
        raise RuntimeError("WRCP-Net A1 Direct produced an inconsistent topology")
    if np.asarray(geometry.topology_probability).shape != (3,):
        raise RuntimeError("WRCP-Net A1 Direct topology probability must have three entries")
    previous_end = -np.inf
    for patch in geometry.patches:
        arrays = (
            patch.peak_wheel_point,
            patch.peak_rail_point,
            patch.corrected_wheel_point,
            patch.corrected_rail_point,
            patch.shape_moments,
        )
        scalars = np.array(
            [
                patch.start_y,
                patch.end_y,
                patch.peak_vertical_penetration,
                patch.peak_normal_penetration,
                patch.peak_contact_angle,
                patch.corrected_vertical_penetration,
                patch.corrected_normal_penetration,
                patch.contact_angle,
            ]
        )
        if not np.isfinite(scalars).all() or any(
            not np.isfinite(np.asarray(value, dtype=float)).all() for value in arrays
        ):
            raise RuntimeError("WRCP-Net A1 Direct produced NaN or Inf")
        center_y = float(patch.corrected_rail_point[0])
        peak_y = float(patch.peak_rail_point[0])
        if not patch.start_y < center_y < patch.end_y:
            raise RuntimeError("WRCP-Net A1 Direct corrected point is outside its patch")
        if not patch.start_y <= peak_y <= patch.end_y:
            raise RuntimeError("WRCP-Net A1 Direct peak point is outside its patch")
        if patch.start_y < previous_end + minimum_patch_separation_m:
            raise RuntimeError("WRCP-Net A1 Direct patches overlap or are not separated")
        previous_end = patch.end_y
        if patch.corrected_vertical_penetration <= 0.0:
            raise RuntimeError("WRCP-Net A1 Direct corrected penetration must be positive")
        if patch.peak_vertical_penetration < patch.corrected_vertical_penetration:
            raise RuntimeError("WRCP-Net A1 Direct peak penetration is below corrected penetration")
        if not (
            angle_min_rad <= patch.contact_angle <= angle_max_rad
            and angle_min_rad <= patch.peak_contact_angle <= angle_max_rad
        ):
            raise RuntimeError("WRCP-Net A1 Direct contact angle is outside calibrated bounds")
        if min(abs(np.cos(patch.contact_angle + roll)), abs(np.cos(patch.peak_contact_angle + roll))) < 1.0e-3:
            raise RuntimeError("WRCP-Net A1 Direct contact angle makes normal penetration singular")
        moments = np.asarray(patch.shape_moments, dtype=float)
        if moments.shape != (3,) or np.any((moments <= 0.0) | (moments >= 1.0)):
            raise RuntimeError("WRCP-Net A1 Direct shape moments must be strictly between zero and one")
        if not (
            moments[0] ** 1.5 <= moments[1] <= moments[0]
            and moments[0] ** 2 <= moments[2] <= moments[1]
        ):
            raise RuntimeError("WRCP-Net A1 Direct shape moments violate physical ordering")


def save_wrcp_net_a1_direct(path: str | Path, model: WRCPNetA1DirectSet) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    values: dict[str, np.ndarray] = {
        "schema": np.array(MODEL_SCHEMA),
        "feature_names": np.asarray(FEATURE_NAMES),
        "history_names": np.asarray(HISTORY_NAMES),
        "target_names": np.asarray(TARGET_NAMES),
        "feature_mean": model.feature_mean,
        "feature_scale": model.feature_scale,
        "history_mean": model.history_mean,
        "history_scale": model.history_scale,
        "target_mean": model.target_mean,
        "target_scale": model.target_scale,
        "feature_ood_scale": _runtime_ood_scale(model.feature_ood_scale, len(FEATURE_NAMES)),
        "history_ood_scale": _runtime_ood_scale(model.history_ood_scale, len(HISTORY_NAMES)),
        "topology_hysteresis_min": np.array(model.topology_hysteresis_min),
        "canonical_side_targets": np.array(model.canonical_side_targets),
        "topology_confidence_min": np.array(model.topology_confidence_min),
        "ood_threshold": np.array(model.ood_threshold),
        "angle_min_rad": np.array(model.angle_min_rad),
        "angle_max_rad": np.array(model.angle_max_rad),
        "minimum_patch_separation_m": np.array(model.minimum_patch_separation_m),
        "shape_moment_history_blend": np.array(model.shape_moment_history_blend),
        "fusion_weight": model.fusion_weight,
        "fusion_bias": model.fusion_bias,
        "topology_weight": model.topology_weight,
        "topology_bias": model.topology_bias,
        "query_embeddings": model.query_embeddings,
    }
    if model.accepted_step_increment_limits is not None:
        values["accepted_increment_names"] = np.asarray(ACCEPTED_INCREMENT_NAMES)
        values["accepted_step_increment_limits"] = _positive_vector(
            model.accepted_step_increment_limits,
            len(ACCEPTED_INCREMENT_NAMES),
            "accepted-step increment limits",
        )
    if (model.shape_moment_min is None) != (model.shape_moment_max is None):
        raise ValueError("shape moment minimum and maximum must be configured together")
    if model.shape_moment_min is not None:
        shape_min, shape_max = _validate_shape_moment_bounds(
            model.shape_moment_min,
            model.shape_moment_max,
        )
        values["shape_moment_min"] = shape_min
        values["shape_moment_max"] = shape_max
    if model.has_fixed_profile_projection:
        values.update(
            {
                "wheel_trace_profile_left": _validate_projection_table(
                    model.wheel_trace_profile_left,
                    columns=3,
                    name="left wheel trace profile",
                ),
                "wheel_trace_profile_right": _validate_projection_table(
                    model.wheel_trace_profile_right,
                    columns=3,
                    name="right wheel trace profile",
                ),
                "rail_profile_left": _validate_projection_table(
                    model.rail_profile_left,
                    columns=2,
                    name="left rail profile",
                ),
                "rail_profile_right": _validate_projection_table(
                    model.rail_profile_right,
                    columns=2,
                    name="right rail profile",
                ),
            }
        )
    _store_layers(values, "state", model.state_weights, model.state_biases)
    _store_layers(values, "history", model.history_weights, model.history_biases)
    _store_layers(values, "decoder", model.decoder_weights, model.decoder_biases)
    values["residual_count"] = np.array(len(model.residual_weights), dtype=np.int64)
    for index, ((weight_1, weight_2), (bias_1, bias_2)) in enumerate(
        zip(model.residual_weights, model.residual_biases, strict=True)
    ):
        values[f"residual_{index}_weight_0"] = weight_1
        values[f"residual_{index}_weight_1"] = weight_2
        values[f"residual_{index}_bias_0"] = bias_1
        values[f"residual_{index}_bias_1"] = bias_2
    np.savez_compressed(destination, **values)


def load_wrcp_net_a1_direct(path: str | Path) -> WRCPNetA1DirectSet:
    with np.load(path, allow_pickle=False) as payload:
        if str(payload["schema"].item()) != MODEL_SCHEMA:
            raise ValueError("unsupported WRCP-Net A1 Direct model schema")
        _validate_names(payload, "feature_names", FEATURE_NAMES)
        _validate_names(payload, "history_names", HISTORY_NAMES)
        _validate_names(payload, "target_names", TARGET_NAMES)
        state_weights, state_biases = _load_layers(payload, "state")
        history_weights, history_biases = _load_layers(payload, "history")
        decoder_weights, decoder_biases = _load_layers(payload, "decoder")
        residual_weights: list[tuple[np.ndarray, np.ndarray]] = []
        residual_biases: list[tuple[np.ndarray, np.ndarray]] = []
        for index in range(int(payload["residual_count"].item())):
            residual_weights.append(
                (
                    np.asarray(payload[f"residual_{index}_weight_0"], dtype=np.float32),
                    np.asarray(payload[f"residual_{index}_weight_1"], dtype=np.float32),
                )
            )
            residual_biases.append(
                (
                    np.asarray(payload[f"residual_{index}_bias_0"], dtype=np.float32),
                    np.asarray(payload[f"residual_{index}_bias_1"], dtype=np.float32),
                )
            )
        return WRCPNetA1DirectSet(
            state_weights=state_weights,
            state_biases=state_biases,
            history_weights=history_weights,
            history_biases=history_biases,
            fusion_weight=np.asarray(payload["fusion_weight"], dtype=np.float32),
            fusion_bias=np.asarray(payload["fusion_bias"], dtype=np.float32),
            residual_weights=tuple(residual_weights),
            residual_biases=tuple(residual_biases),
            topology_weight=np.asarray(payload["topology_weight"], dtype=np.float32),
            topology_bias=np.asarray(payload["topology_bias"], dtype=np.float32),
            query_embeddings=np.asarray(payload["query_embeddings"], dtype=np.float32),
            decoder_weights=decoder_weights,
            decoder_biases=decoder_biases,
            feature_mean=np.asarray(payload["feature_mean"], dtype=float),
            feature_scale=np.asarray(payload["feature_scale"], dtype=float),
            history_mean=np.asarray(payload["history_mean"], dtype=float),
            history_scale=np.asarray(payload["history_scale"], dtype=float),
            target_mean=np.asarray(payload["target_mean"], dtype=float),
            target_scale=np.asarray(payload["target_scale"], dtype=float),
            feature_ood_scale=(
                np.asarray(payload["feature_ood_scale"], dtype=float)
                if "feature_ood_scale" in payload.files
                else np.ones((len(FEATURE_NAMES),), dtype=float)
            ),
            history_ood_scale=(
                np.asarray(payload["history_ood_scale"], dtype=float)
                if "history_ood_scale" in payload.files
                else np.ones((len(HISTORY_NAMES),), dtype=float)
            ),
            accepted_step_increment_limits=(
                _load_accepted_step_increment_limits(payload)
                if "accepted_step_increment_limits" in payload.files
                else None
            ),
            shape_moment_min=(
                np.asarray(payload["shape_moment_min"], dtype=float)
                if "shape_moment_min" in payload.files
                else None
            ),
            shape_moment_max=(
                np.asarray(payload["shape_moment_max"], dtype=float)
                if "shape_moment_max" in payload.files
                else None
            ),
            topology_hysteresis_min=(
                float(payload["topology_hysteresis_min"].item())
                if "topology_hysteresis_min" in payload.files
                else 0.0
            ),
            canonical_side_targets=(
                bool(payload["canonical_side_targets"].item())
                if "canonical_side_targets" in payload.files
                else False
            ),
            topology_confidence_min=float(payload["topology_confidence_min"].item()),
            ood_threshold=float(payload["ood_threshold"].item()),
            angle_min_rad=float(payload["angle_min_rad"].item()),
            angle_max_rad=float(payload["angle_max_rad"].item()),
            minimum_patch_separation_m=float(payload["minimum_patch_separation_m"].item()),
            shape_moment_history_blend=(
                float(payload["shape_moment_history_blend"].item())
                if "shape_moment_history_blend" in payload.files
                else 1.0
            ),
            wheel_trace_profile_left=(
                np.asarray(payload["wheel_trace_profile_left"], dtype=float)
                if "wheel_trace_profile_left" in payload.files
                else None
            ),
            wheel_trace_profile_right=(
                np.asarray(payload["wheel_trace_profile_right"], dtype=float)
                if "wheel_trace_profile_right" in payload.files
                else None
            ),
            rail_profile_left=(
                np.asarray(payload["rail_profile_left"], dtype=float)
                if "rail_profile_left" in payload.files
                else None
            ),
            rail_profile_right=(
                np.asarray(payload["rail_profile_right"], dtype=float)
                if "rail_profile_right" in payload.files
                else None
            ),
        )


def _project_target_to_fixed_profiles(
    values: np.ndarray,
    *,
    features: np.ndarray,
    wheel_trace_profile: np.ndarray | None,
    rail_profile: np.ndarray | None,
) -> np.ndarray:
    """Derive final point fields from fixed profiles using scalar lookups.

    The network still supplies the contact interval, corrected centre and peak
    fraction.  Given those two final lateral coordinates, the fixed wheel and
    rail profiles uniquely determine longitudinal wheel position, local wheel
    coordinate, rail height and contact angle.  The inverse wheel mapping uses
    scalar bisection and therefore does not recreate a 501/1001-point profile
    or a penetration field in the Cal path.
    """

    wheel = _validate_projection_table(
        wheel_trace_profile,
        columns=3,
        name="wheel trace profile",
    )
    rail = _validate_projection_table(
        rail_profile,
        columns=2,
        name="rail profile",
    )
    x = np.asarray(features, dtype=float).reshape(len(FEATURE_NAMES))
    result = np.asarray(values, dtype=float).reshape(len(TARGET_NAMES)).copy()
    center_y = float(result[0])
    left_width = float(_softplus(result[1]) + 1.0e-9)
    right_width = float(_softplus(result[2]) + 1.0e-9)
    start_y = center_y - left_width
    end_y = center_y + right_width
    peak_y = start_y + float(_sigmoid(result[8])) * (end_y - start_y)

    corrected_lateral = _invert_fixed_wheel_lateral(
        center_y,
        wheel,
        lateral=float(x[1]),
        roll=float(x[3]),
        yaw=float(x[4]),
    )
    corrected_x, _, corrected_z, corrected_angle = _fixed_wheel_trace_point(
        corrected_lateral,
        wheel,
        lateral=float(x[1]),
        roll=float(x[3]),
        yaw=float(x[4]),
    )
    peak_lateral = _invert_fixed_wheel_lateral(
        peak_y,
        wheel,
        lateral=float(x[1]),
        roll=float(x[3]),
        yaw=float(x[4]),
    )
    peak_x, _, peak_z, peak_angle = _fixed_wheel_trace_point(
        peak_lateral,
        wheel,
        lateral=float(x[1]),
        roll=float(x[3]),
        yaw=float(x[4]),
    )
    corrected_rail_z = _fixed_rail_height(center_y, rail)
    peak_rail_z = _fixed_rail_height(peak_y, rail)
    corrected_penetration = corrected_z - corrected_rail_z + float(x[2]) + float(x[5])
    peak_penetration = peak_z - peak_rail_z + float(x[2]) + float(x[5])
    if corrected_penetration <= 0.0:
        raise RuntimeError(
            "WRCP-Net A1 Direct projected corrected point has no positive penetration"
        )
    if peak_penetration < corrected_penetration:
        # The corrected centre is one of the two evaluated physical points.  If
        # the learned peak fraction selects a lower point, retain the centre as
        # the peak instead of inventing an independently regressed penetration.
        peak_y = center_y
        peak_lateral = corrected_lateral
        peak_x = corrected_x
        peak_z = corrected_z
        peak_angle = corrected_angle
        peak_rail_z = corrected_rail_z
        peak_penetration = corrected_penetration
        peak_fraction = np.clip(
            (center_y - start_y) / max(end_y - start_y, 2.0e-12),
            1.0e-7,
            1.0 - 1.0e-7,
        )
        result[8] = _logit(peak_fraction)
    peak_increment = max(peak_penetration - corrected_penetration, 1.0e-12)
    result[3] = corrected_x
    result[4] = corrected_rail_z
    result[5] = corrected_lateral
    result[6] = _softplus_inverse(corrected_penetration)
    result[7] = corrected_angle
    result[9] = peak_x
    result[10] = peak_rail_z
    result[11] = _softplus_inverse(peak_increment)
    result[12] = peak_angle
    return result


def _invert_fixed_wheel_lateral(
    target_y: float,
    table: np.ndarray,
    *,
    lateral: float,
    roll: float,
    yaw: float,
) -> float:
    lower = float(table[0, 0])
    upper = float(table[-1, 0])
    lower_y = _fixed_wheel_trace_point(
        lower, table, lateral=lateral, roll=roll, yaw=yaw
    )[1]
    upper_y = _fixed_wheel_trace_point(
        upper, table, lateral=lateral, roll=roll, yaw=yaw
    )[1]
    minimum_y = min(lower_y, upper_y)
    maximum_y = max(lower_y, upper_y)
    tolerance = 1.0e-12
    if not minimum_y - tolerance <= target_y <= maximum_y + tolerance:
        raise RuntimeError(
            "WRCP-Net A1 Direct final wheel point is outside the fixed wheel profile: "
            f"y={target_y:.9g}, support=[{minimum_y:.9g}, {maximum_y:.9g}]"
        )
    if abs(target_y - lower_y) <= tolerance:
        return lower
    if abs(target_y - upper_y) <= tolerance:
        return upper

    endpoint_residual = {
        lower: lower_y - target_y,
        upper: upper_y - target_y,
    }

    def residual(wheel_lateral: float) -> float:
        cached = endpoint_residual.get(wheel_lateral)
        if cached is not None:
            return cached
        return (
            _fixed_wheel_trace_point(
                wheel_lateral,
                table,
                lateral=lateral,
                roll=roll,
                yaw=yaw,
            )[1]
            - target_y
        )

    # The fixed wheel-to-track mapping is monotonic on the stored profile
    # support. Brent's bracketed root finder preserves the old bisection
    # robustness while converging in about 5–6 evaluations on production
    # traces instead of always performing 30 scalar bisection evaluations.
    return float(
        brentq(
            residual,
            lower,
            upper,
            xtol=5.0e-13,
            rtol=1.0e-14,
            maxiter=20,
        )
    )


def _fixed_wheel_trace_point(
    wheel_lateral: float,
    table: np.ndarray,
    *,
    lateral: float,
    roll: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    profile_lateral = table[:, 0]
    rolling_radius = float(np.interp(wheel_lateral, profile_lateral, table[:, 1]))
    contact_angle = float(np.interp(wheel_lateral, profile_lateral, table[:, 2]))
    lx = -float(np.cos(roll) * np.sin(yaw))
    ly = float(np.cos(roll) * np.cos(yaw))
    lz = float(np.sin(roll))
    tangent = float(np.tan(contact_angle))
    m_value = float(np.sqrt(max(0.0, 1.0 - lx**2 * (1.0 + tangent**2))))
    denominator = 1.0 - lx**2
    track_x = lx * (wheel_lateral + rolling_radius * tangent)
    track_y = (
        wheel_lateral * ly
        + lateral
        - rolling_radius * (lx**2 * ly * tangent + lz * m_value) / denominator
    )
    track_z = (
        wheel_lateral * lz
        - rolling_radius * (lx**2 * lz * tangent - ly * m_value) / denominator
    )
    return track_x, track_y, track_z, contact_angle


def _fixed_rail_height(target_y: float, table: np.ndarray) -> float:
    minimum_y = float(table[0, 0])
    maximum_y = float(table[-1, 0])
    if not minimum_y - 1.0e-12 <= target_y <= maximum_y + 1.0e-12:
        raise RuntimeError(
            "WRCP-Net A1 Direct final rail point is outside the fixed rail profile: "
            f"y={target_y:.9g}, support=[{minimum_y:.9g}, {maximum_y:.9g}]"
        )
    return float(np.interp(target_y, table[:, 0], table[:, 1]))


def _validate_projection_table(
    values: np.ndarray | None,
    *,
    columns: int,
    name: str,
) -> np.ndarray:
    table = np.asarray(values, dtype=float)
    if table.ndim != 2 or table.shape[1] != columns or table.shape[0] < 2:
        raise RuntimeError(f"WRCP-Net A1 Direct {name} has the wrong shape")
    if not np.isfinite(table).all() or np.any(np.diff(table[:, 0]) <= 0.0):
        raise RuntimeError(
            f"WRCP-Net A1 Direct {name} must be finite and strictly increasing"
        )
    return table


def _decode_patch(values: np.ndarray, *, delta_z: float, roll: float, d0: float) -> DirectContactPatch:
    value = np.asarray(values, dtype=float).reshape(len(TARGET_NAMES))
    center_y = float(value[0])
    left_width = float(_softplus(value[1]) + 1.0e-9)
    right_width = float(_softplus(value[2]) + 1.0e-9)
    start_y = center_y - left_width
    end_y = center_y + right_width
    corrected_penetration = float(_softplus(value[6]) + 1.0e-12)
    corrected_angle = float(value[7])
    peak_y = start_y + float(_sigmoid(value[8])) * (end_y - start_y)
    peak_penetration = corrected_penetration + float(_softplus(value[11]) + 1.0e-12)
    peak_angle = float(value[12])
    moments = np.asarray(_sigmoid(value[13:16]), dtype=float)
    corrected_rail = np.array([center_y, value[4]], dtype=float)
    peak_rail = np.array([peak_y, value[10]], dtype=float)
    corrected_wheel = np.array(
        [value[3], center_y, corrected_penetration + value[4] - delta_z - d0],
        dtype=float,
    )
    peak_wheel = np.array(
        [value[9], peak_y, peak_penetration + value[10] - delta_z - d0],
        dtype=float,
    )
    corrected_cosine = max(abs(float(np.cos(corrected_angle + roll))), np.finfo(float).eps)
    peak_cosine = max(abs(float(np.cos(peak_angle + roll))), np.finfo(float).eps)
    return DirectContactPatch(
        start_y=start_y,
        end_y=end_y,
        peak_wheel_point=peak_wheel,
        peak_rail_point=peak_rail,
        corrected_wheel_point=corrected_wheel,
        corrected_rail_point=corrected_rail,
        wheel_profile_lateral=float(value[5]),
        peak_vertical_penetration=peak_penetration,
        peak_normal_penetration=peak_penetration / peak_cosine,
        peak_contact_angle=peak_angle,
        corrected_vertical_penetration=corrected_penetration,
        corrected_normal_penetration=corrected_penetration / corrected_cosine,
        contact_angle=corrected_angle,
        shape_moments=moments,
    )


def geometry_from_training_targets(
    *,
    features: np.ndarray,
    targets: np.ndarray,
    patch_count: int,
) -> DirectContactGeometry:
    """Decode teacher targets with the same deterministic equations as runtime."""

    x = np.asarray(features, dtype=float).reshape(len(FEATURE_NAMES))
    values = np.asarray(targets, dtype=float).reshape(MAX_PATCHES, len(TARGET_NAMES))
    patches = tuple(
        _decode_patch(values[index], delta_z=x[2], roll=x[3], d0=x[5])
        for index in range(int(patch_count))
    )
    probability = np.zeros((3,), dtype=float)
    probability[int(patch_count)] = 1.0
    return DirectContactGeometry(
        has_contact=bool(patches),
        patches=patches,
        topology_probability=probability,
        in_distribution=True,
    )


def _bound_direct_geometry(
    geometry: DirectContactGeometry,
    *,
    roll: float,
    angle_min_rad: float,
    angle_max_rad: float,
    minimum_patch_separation_m: float,
    shape_moment_min: np.ndarray | None = None,
    shape_moment_max: np.ndarray | None = None,
) -> DirectContactGeometry:
    moment_min, moment_max = _validate_shape_moment_bounds(
        shape_moment_min,
        shape_moment_max,
    )
    bounded: list[DirectContactPatch] = []
    for patch in geometry.patches:
        corrected_angle = float(np.clip(patch.contact_angle, angle_min_rad, angle_max_rad))
        peak_angle = float(np.clip(patch.peak_contact_angle, angle_min_rad, angle_max_rad))
        moments = np.clip(
            np.asarray(patch.shape_moments, dtype=float),
            moment_min,
            moment_max,
        )
        moment_1 = float(moments[0])
        moment_1p5 = float(
            np.clip(
                moments[1],
                max(moment_1**1.5, moment_min[1]),
                min(moment_1, moment_max[1]),
            )
        )
        moment_2 = float(
            np.clip(
                moments[2],
                max(moment_1**2, moment_min[2]),
                min(moment_1p5, moment_max[2]),
            )
        )
        corrected_cosine = max(abs(float(np.cos(corrected_angle + roll))), 1.0e-3)
        peak_cosine = max(abs(float(np.cos(peak_angle + roll))), 1.0e-3)
        bounded.append(
            replace(
                patch,
                contact_angle=corrected_angle,
                peak_contact_angle=peak_angle,
                corrected_normal_penetration=float(
                    patch.corrected_vertical_penetration / corrected_cosine
                ),
                peak_normal_penetration=float(patch.peak_vertical_penetration / peak_cosine),
                shape_moments=np.array([moment_1, moment_1p5, moment_2], dtype=float),
            )
        )
    bounded.sort(key=lambda patch: patch.corrected_rail_point[0])
    if len(bounded) == 2:
        left, right = bounded
        left_center = float(left.corrected_rail_point[0])
        right_center = float(right.corrected_rail_point[0])
        # Leave a small numerical guard band.  The strict runtime validator
        # intentionally uses the requested separation without a tolerance, so
        # constructing two interval edges at exactly that distance can fail by
        # one floating-point ulp after serialization or arithmetic below.
        required_gap = minimum_patch_separation_m + 1.0e-9
        minimum_center_distance = required_gap + 4.0e-9
        if right_center - left_center <= minimum_center_distance:
            midpoint = 0.5 * (left_center + right_center)
            left = _shift_patch_lateral(left, midpoint - 0.5 * minimum_center_distance - left_center)
            right = _shift_patch_lateral(right, midpoint + 0.5 * minimum_center_distance - right_center)
            bounded = [left, right]
        if left.end_y + required_gap > right.start_y:
            left_center = float(left.corrected_rail_point[0])
            right_center = float(right.corrected_rail_point[0])
            separator = 0.5 * (left_center + right_center)
            left_end = min(left.end_y, separator - 0.5 * required_gap)
            right_start = max(right.start_y, separator + 0.5 * required_gap)
            left_end = max(left_end, left_center + 1.0e-9)
            right_start = min(right_start, right_center - 1.0e-9)
            if right_start - left_end < required_gap:
                # The centre-spacing branch above guarantees positive room on
                # both sides.  This construction is the final exact fallback
                # when very wide decoded patches consumed that room.
                available = right_center - left_center
                edge_margin = 0.25 * (available - required_gap)
                left_end = left_center + edge_margin
                right_start = right_center - edge_margin
            left = replace(left, end_y=float(left_end))
            right = replace(right, start_y=float(right_start))
            bounded = [left, right]
    bounded = [_clip_peak_to_patch_interval(patch) for patch in bounded]
    return DirectContactGeometry(
        has_contact=bool(bounded),
        patches=tuple(bounded),
        topology_probability=np.asarray(geometry.topology_probability, dtype=float).copy(),
        in_distribution=geometry.in_distribution,
    )


def _blend_shape_moments_with_accepted_history(
    geometry: DirectContactGeometry,
    *,
    history: np.ndarray,
    history_mask: np.ndarray,
    new_prediction_weight: float,
) -> DirectContactGeometry:
    weight = float(new_prediction_weight)
    if not 0.0 <= weight <= 1.0:
        raise ValueError("shape moment history blend must be in [0, 1]")
    previous_indexes = np.flatnonzero(np.asarray(history_mask, dtype=bool))
    if weight >= 1.0 or len(geometry.patches) != previous_indexes.size:
        return geometry
    previous = np.asarray(history, dtype=float)[previous_indexes]
    previous_order = np.argsort(previous[:, HISTORY_NAMES.index("corrected_rail_y_m")])
    previous = previous[previous_order]
    current = sorted(geometry.patches, key=lambda patch: patch.corrected_rail_point[0])
    blended = []
    for patch, accepted in zip(current, previous, strict=True):
        accepted_moments = accepted[
            [
                HISTORY_NAMES.index("shape_moment_1"),
                HISTORY_NAMES.index("shape_moment_1p5"),
                HISTORY_NAMES.index("shape_moment_2"),
            ]
        ]
        moments = (
            weight * np.asarray(patch.shape_moments, dtype=float)
            + (1.0 - weight) * accepted_moments
        )
        blended.append(replace(patch, shape_moments=moments))
    return DirectContactGeometry(
        has_contact=bool(blended),
        patches=tuple(blended),
        topology_probability=geometry.topology_probability,
        in_distribution=geometry.in_distribution,
    )


def _validate_shape_moment_bounds(
    minimum: np.ndarray | None,
    maximum: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray]:
    if minimum is None and maximum is None:
        return (
            np.full((3,), 1.0e-7, dtype=float),
            np.full((3,), 1.0 - 1.0e-7, dtype=float),
        )
    if minimum is None or maximum is None:
        raise ValueError("shape moment minimum and maximum must be configured together")
    lower = np.asarray(minimum, dtype=float).reshape(3)
    upper = np.asarray(maximum, dtype=float).reshape(3)
    if (
        not np.isfinite(lower).all()
        or not np.isfinite(upper).all()
        or np.any(lower <= 0.0)
        or np.any(upper >= 1.0)
        or np.any(lower >= upper)
    ):
        raise ValueError("shape moment bounds must be finite, ordered, and inside (0, 1)")
    # Every m1 value in the configured interval must leave a feasible interval
    # for m1.5 and m2 after the physical moment ordering is applied.
    if lower[1] > upper[0] or lower[2] > upper[1]:
        raise ValueError("shape moment bounds are incompatible with physical ordering")
    return lower, upper


def _clip_peak_to_patch_interval(patch: DirectContactPatch) -> DirectContactPatch:
    """Keep the derived peak inside an interval narrowed for patch separation."""

    peak_y = float(np.clip(patch.peak_rail_point[0], patch.start_y, patch.end_y))
    peak_wheel = np.asarray(patch.peak_wheel_point, dtype=float).copy()
    peak_rail = np.asarray(patch.peak_rail_point, dtype=float).copy()
    peak_wheel[1] = peak_y
    peak_rail[0] = peak_y
    return replace(
        patch,
        peak_wheel_point=peak_wheel,
        peak_rail_point=peak_rail,
    )


def _shift_patch_lateral(patch: DirectContactPatch, shift_y: float) -> DirectContactPatch:
    corrected_wheel = np.asarray(patch.corrected_wheel_point, dtype=float).copy()
    peak_wheel = np.asarray(patch.peak_wheel_point, dtype=float).copy()
    corrected_rail = np.asarray(patch.corrected_rail_point, dtype=float).copy()
    peak_rail = np.asarray(patch.peak_rail_point, dtype=float).copy()
    corrected_wheel[1] += shift_y
    peak_wheel[1] += shift_y
    corrected_rail[0] += shift_y
    peak_rail[0] += shift_y
    return replace(
        patch,
        start_y=float(patch.start_y + shift_y),
        end_y=float(patch.end_y + shift_y),
        corrected_wheel_point=corrected_wheel,
        peak_wheel_point=peak_wheel,
        corrected_rail_point=corrected_rail,
        peak_rail_point=peak_rail,
    )


def direct_patch_shape_features(geometry: DirectContactGeometry) -> np.ndarray:
    rows: list[np.ndarray] = []
    for patch in geometry.patches:
        width = max(float(patch.end_y - patch.start_y), np.finfo(float).eps)
        peak = max(float(patch.peak_vertical_penetration), np.finfo(float).eps)
        moment_1, moment_1p5, moment_2 = np.asarray(patch.shape_moments, dtype=float)
        area = width * peak * moment_1
        power_area = width * peak**1.5 * moment_1p5
        mean = peak * moment_1
        variance = max(peak**2 * moment_2 - mean**2, 0.0)
        rows.append(np.array([area, power_area, mean, np.sqrt(variance)], dtype=float))
    return np.stack(rows) if rows else np.zeros((0, 4), dtype=float)


def _silu(value: np.ndarray) -> np.ndarray:
    x = np.asarray(value, dtype=float)
    return x * _sigmoid(x)


def _sigmoid(value: np.ndarray | float) -> np.ndarray:
    x = np.asarray(value, dtype=float)
    result = np.empty_like(x)
    nonnegative = x >= 0.0
    result[nonnegative] = 1.0 / (1.0 + np.exp(-x[nonnegative]))
    exp_value = np.exp(x[~nonnegative])
    result[~nonnegative] = exp_value / (1.0 + exp_value)
    return result


def _softplus(value: np.ndarray | float) -> np.ndarray:
    x = np.asarray(value, dtype=float)
    return np.maximum(x, 0.0) + np.log1p(np.exp(-np.abs(x)))


def _softplus_inverse(value: np.ndarray | float) -> np.ndarray:
    x = np.maximum(np.asarray(value, dtype=float), 1.0e-12)
    return x + np.log(-np.expm1(-x))


def _logit(value: np.ndarray | float) -> np.ndarray:
    x = np.clip(np.asarray(value, dtype=float), 1.0e-7, 1.0 - 1.0e-7)
    return np.log(x) - np.log1p(-x)


def _positive_vector(value: np.ndarray, length: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(length)
    if not np.isfinite(result).all() or np.any(result <= 0.0):
        raise ValueError(f"{name} must contain finite positive values")
    return result.copy()


def _load_accepted_step_increment_limits(payload: Mapping[str, np.ndarray]) -> np.ndarray:
    _validate_names(payload, "accepted_increment_names", ACCEPTED_INCREMENT_NAMES)
    return _positive_vector(
        np.asarray(payload["accepted_step_increment_limits"], dtype=float),
        len(ACCEPTED_INCREMENT_NAMES),
        "accepted-step increment limits",
    )


def _softmax(value: np.ndarray) -> np.ndarray:
    x = np.asarray(value, dtype=float)
    shifted = x - np.max(x)
    exponential = np.exp(shifted)
    return exponential / np.sum(exponential)


def _default_vector(
    value: np.ndarray | None,
    size: int,
    default: float,
    *,
    positive: bool = False,
) -> np.ndarray:
    result = np.full((size,), default, dtype=float) if value is None else np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"calibration vector must have shape {(size,)} and contain finite values")
    if positive and np.any(result <= 0.0):
        raise ValueError("calibration scale must be positive")
    return result.copy()


def _runtime_ood_scale(value: np.ndarray | None, size: int) -> np.ndarray:
    if value is None:
        return np.ones((size,), dtype=float)
    result = np.asarray(value, dtype=float).reshape(size)
    if not np.isfinite(result).all() or np.any(result < 1.0):
        raise ValueError("OOD calibration scale must be finite and at least one")
    return result


def _canonical_side_target_transform(
    targets: np.ndarray,
    side_id: float | np.ndarray,
) -> np.ndarray:
    """Mirror left/right targets between physical and canonical coordinates.

    The transform is its own inverse.  It removes the artificial global
    ``-0.75/+0.75 m`` bimodality while keeping the deployed output contract in
    physical track coordinates.
    """

    result = np.asarray(targets, dtype=float).copy()
    single = result.ndim == 1
    if single:
        result = result[None, :]
    side = np.asarray(side_id, dtype=float)
    if side.ndim == 0:
        side = np.full((result.shape[0],), float(side))
    side = side.reshape(result.shape[0])
    lateral_sign = np.where(side >= 0.5, 1.0, -1.0)
    left = lateral_sign < 0.0
    result[:, 0] *= lateral_sign
    result[:, 5] *= lateral_sign
    result[:, 7] *= -lateral_sign
    result[:, 12] *= -lateral_sign
    result[:, 8] *= lateral_sign
    left_width = result[left, 1].copy()
    result[left, 1] = result[left, 2]
    result[left, 2] = left_width
    return result[0] if single else result


def _store_layers(
    values: dict[str, np.ndarray],
    prefix: str,
    weights: tuple[np.ndarray, ...],
    biases: tuple[np.ndarray, ...],
) -> None:
    values[f"{prefix}_count"] = np.array(len(weights), dtype=np.int64)
    for index, (weight, bias) in enumerate(zip(weights, biases, strict=True)):
        values[f"{prefix}_weight_{index}"] = weight
        values[f"{prefix}_bias_{index}"] = bias


def _load_layers(
    payload: Mapping[str, np.ndarray],
    prefix: str,
) -> tuple[tuple[np.ndarray, ...], tuple[np.ndarray, ...]]:
    count = int(payload[f"{prefix}_count"].item())
    weights = tuple(np.asarray(payload[f"{prefix}_weight_{index}"], dtype=np.float32) for index in range(count))
    biases = tuple(np.asarray(payload[f"{prefix}_bias_{index}"], dtype=np.float32) for index in range(count))
    return weights, biases


def _validate_names(payload: Mapping[str, np.ndarray], key: str, expected: tuple[str, ...]) -> None:
    actual = tuple(str(value) for value in payload[key])
    if actual != expected:
        raise ValueError(f"WRCP-Net A1 Direct {key} does not match runtime")


def _record_timing(
    timing: dict[str, float] | None,
    name: str,
    started: float | None,
) -> None:
    if timing is None or started is None:
        return
    timing[name] = timing.get(name, 0.0) + (time.perf_counter() - started)
