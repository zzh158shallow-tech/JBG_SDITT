from __future__ import annotations

import copy
import json
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from sditt.models.wrcp_net_a1_direct import (
    FEATURE_NAMES,
    HISTORY_NAMES,
    MAX_PATCHES,
    QUERY_OUTPUT_SIZE,
    TARGET_NAMES,
    WRCPNetA1DirectSet,
    _canonical_side_target_transform,
    geometry_to_history,
    save_wrcp_net_a1_direct,
)
from sditt.training_data.network_a_direct import (
    NetworkA1DirectDataset,
    dagger_source_mask,
    ensure_supervision_metadata,
)


RUNTIME_DAGGER_SAMPLE_WEIGHT = 1.0
FULL_CAL_TOPOLOGY_WEIGHT = 2.0
OTHER_ACCEPTED_TOPOLOGY_WEIGHT = 0.5
SYNTHETIC_TOPOLOGY_WEIGHT = 0.25
CAL1_GEOMETRY_WEIGHT = 12.0
EARLY_CAL_GEOMETRY_WEIGHT = 4.0
EARLY_CAL_MAX_STEP = 64
TRANSITION_CAL_GEOMETRY_WEIGHT = 2.0
TRANSITION_CAL_MAX_STEP = 256
OOD_CALIBRATION_QUANTILE = 1.0
OOD_CALIBRATION_TARGET_DISTANCE = 3.0
MOMENT_REGRESSION_WEIGHT = 8.0
CORRECTED_PENETRATION_REGRESSION_WEIGHT = 8.0
PEAK_PENETRATION_INCREMENT_REGRESSION_WEIGHT = 4.0
HERTZ_FORCE_PROXY_LOSS_WEIGHT = 1.0
VALIDATION_IRREGULARITY_SEED = 20260721
TEST_IRREGULARITY_SEED = 20260722
MINIMUM_FULL_CAL_TRAINING_SEEDS = 2


@dataclass(frozen=True)
class WRCPNetA1DirectTrainingResult:
    model: WRCPNetA1DirectSet
    model_path: Path
    metrics_path: Path
    best_epoch: int
    metrics: dict[str, Any]


def train_wrcp_net_a1_direct(
    train: NetworkA1DirectDataset,
    validation: NetworkA1DirectDataset,
    test: NetworkA1DirectDataset,
    *,
    output_dir: str | Path,
    epochs: int = 400,
    batch_size: int = 256,
    learning_rate: float = 3.0e-4,
    weight_decay: float = 1.0e-4,
    patience: int = 40,
    seed: int = 20260721,
    repo_root: str | Path | None = None,
    initial_model: WRCPNetA1DirectSet | None = None,
    maximum_rollout_steps: int = 32,
    final_teacher_probability: float = 0.1,
    rollout_start_fraction: float = 0.4,
    rollout_refresh_epochs: int = 5,
    moment_regression_weight: float = MOMENT_REGRESSION_WEIGHT,
) -> WRCPNetA1DirectTrainingResult:
    torch, nn, functional = _require_torch()
    train = ensure_supervision_metadata(train)
    validation = ensure_supervision_metadata(validation)
    test = ensure_supervision_metadata(test)
    if min(len(train), len(validation), len(test)) <= 0:
        raise ValueError("network-A1 Direct train/validation/test datasets must be non-empty")
    if epochs <= 0 or batch_size <= 0 or patience <= 0:
        raise ValueError("epochs, batch size, and patience must be positive")
    if maximum_rollout_steps <= 0 or rollout_refresh_epochs <= 0:
        raise ValueError("rollout steps and refresh interval must be positive")
    if not 0.0 <= final_teacher_probability <= 1.0:
        raise ValueError("final teacher probability must be in [0, 1]")
    if not 0.0 < rollout_start_fraction < 1.0:
        raise ValueError("rollout start fraction must be in (0, 1)")
    if moment_regression_weight <= 0.0:
        raise ValueError("moment regression weight must be positive")
    _validate_split_isolation(train, validation, test)
    supervision_summary = _validate_supervision_policy(train, validation, test)
    fixed_profile_projection = _load_fixed_profile_projection(repo_root)
    torch.manual_seed(seed)
    np.random.seed(seed)

    topology_weights_all = _direct_topology_weights(train, np.arange(len(train)))
    geometry_weights_all = _direct_sample_weights(train, np.arange(len(train)))
    supervised_rows = (topology_weights_all > 0.0) | (geometry_weights_all > 0.0)
    dataset_feature_mean, dataset_feature_scale = _mean_scale(train.features[supervised_rows])
    supervised_history_mask = train.history_mask & supervised_rows[:, None]
    valid_history = train.history[supervised_history_mask]
    dataset_history_mean, dataset_history_scale = _mean_scale(
        valid_history if valid_history.size else np.zeros((1, len(HISTORY_NAMES)), dtype=float)
    )
    canonical_train_targets = _canonicalize_batch_targets(train.targets, train.features[:, 0])
    supervised_patch_mask = train.patch_mask & (geometry_weights_all > 0.0)[:, None]
    valid_targets = canonical_train_targets[supervised_patch_mask]
    if not valid_targets.size:
        raise ValueError("network-A1 Direct training data have no eligible geometry labels")
    dataset_target_mean, dataset_target_scale = _mean_scale(valid_targets)
    if initial_model is None:
        feature_mean, feature_scale = dataset_feature_mean, dataset_feature_scale
        history_mean, history_scale = dataset_history_mean, dataset_history_scale
        target_mean, target_scale = dataset_target_mean, dataset_target_scale
    else:
        if not initial_model.canonical_side_targets:
            raise ValueError("warm-start WRCP-Net A1 Direct model must use canonical side targets")
        feature_mean = np.asarray(initial_model.feature_mean, dtype=float).copy()
        feature_scale = np.asarray(initial_model.feature_scale, dtype=float).copy()
        history_mean = np.asarray(initial_model.history_mean, dtype=float).copy()
        history_scale = np.asarray(initial_model.history_scale, dtype=float).copy()
        target_mean = np.asarray(initial_model.target_mean, dtype=float).copy()
        target_scale = np.asarray(initial_model.target_scale, dtype=float).copy()
    feature_ood_scale = _calibrated_ood_scale(
        (train.features[supervised_rows] - feature_mean) / feature_scale,
        threshold=OOD_CALIBRATION_TARGET_DISTANCE,
        quantile=OOD_CALIBRATION_QUANTILE,
    )
    history_ood_scale = _calibrated_ood_scale(
        (valid_history - history_mean) / history_scale,
        threshold=OOD_CALIBRATION_TARGET_DISTANCE,
        quantile=OOD_CALIBRATION_QUANTILE,
    )

    class_counts = np.maximum(
        np.bincount(
            train.patch_count,
            weights=topology_weights_all,
            minlength=3,
        ),
        1.0,
    )
    class_weights = np.sqrt(np.max(class_counts) / class_counts)
    class_weights /= np.mean(class_weights)
    class_weights_tensor = torch.as_tensor(class_weights, dtype=torch.float32)

    class TorchA1Direct(nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.state = nn.ModuleList((nn.Linear(len(FEATURE_NAMES), 128), nn.Linear(128, 128)))
            self.history = nn.ModuleList((nn.Linear(len(HISTORY_NAMES), 128), nn.Linear(128, 128)))
            self.fusion = nn.Linear(257, 256)
            self.residual = nn.ModuleList(
                nn.ModuleList((nn.Linear(256, 256), nn.Linear(256, 256))) for _ in range(3)
            )
            self.topology = nn.Linear(256, 3)
            self.query_embeddings = nn.Parameter(torch.empty(MAX_PATCHES, 32))
            nn.init.normal_(self.query_embeddings, std=0.02)
            self.decoder = nn.ModuleList(
                (nn.Linear(288, 128), nn.Linear(128, 128), nn.Linear(128, QUERY_OUTPUT_SIZE))
            )

        def forward(self, features: Any, history: Any, history_mask: Any) -> tuple[Any, Any]:
            state = features
            for layer in self.state:
                state = functional.silu(layer(state))
            encoded = history
            for layer in self.history:
                encoded = functional.silu(layer(encoded))
            encoded = encoded * history_mask[..., None]
            pooled = torch.sum(encoded, dim=1)
            count = torch.sum(history_mask, dim=1, keepdim=True) / MAX_PATCHES
            latent = functional.silu(self.fusion(torch.cat((state, pooled, count), dim=1)))
            for first, second in self.residual:
                latent = functional.silu(latent + second(functional.silu(first(latent))))
            topology = self.topology(latent)
            queries = self.query_embeddings[None, :, :].expand(latent.shape[0], -1, -1)
            decoded = torch.cat((latent[:, None, :].expand(-1, MAX_PATCHES, -1), queries), dim=2)
            for index, layer in enumerate(self.decoder):
                decoded = layer(decoded)
                if index + 1 < len(self.decoder):
                    decoded = functional.silu(decoded)
            return topology, decoded

    network = TorchA1Direct()
    if initial_model is not None:
        _load_runtime_weights_into_torch(torch, network, initial_model)
    optimizer = torch.optim.AdamW(
        network.parameters(),
        lr=learning_rate,
        weight_decay=weight_decay,
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(epochs, 1))
    rng = np.random.default_rng(seed)
    best_loss = float("inf")
    best_epoch = 0
    best_state = copy.deepcopy(network.state_dict())
    stale = 0
    scheduled_history = train.history.copy()
    scheduled_mask = train.history_mask.copy()
    history_noise_scale = np.zeros((len(HISTORY_NAMES),), dtype=float)
    schedule_boundary = int(np.floor(rollout_start_fraction * epochs))
    accepted_step_increment_limits = (
        None
        if initial_model is None or initial_model.accepted_step_increment_limits is None
        else np.asarray(initial_model.accepted_step_increment_limits, dtype=float).copy()
    )
    topology_hysteresis_min = (
        0.0 if initial_model is None else float(initial_model.topology_hysteresis_min)
    )

    for epoch in range(epochs):
        teacher_probability = _teacher_probability(
            epoch,
            epochs,
            final=final_teacher_probability,
            start_fraction=rollout_start_fraction,
        )
        if epoch >= schedule_boundary and (
            epoch == schedule_boundary or epoch % rollout_refresh_epochs == 0
        ):
            rollout_horizon = _training_rollout_horizon(
                epoch,
                epochs,
                maximum=maximum_rollout_steps,
                start_fraction=rollout_start_fraction,
            )
            runtime = _torch_to_runtime(
                network,
                feature_mean=feature_mean,
                feature_scale=feature_scale,
                history_mean=history_mean,
                history_scale=history_scale,
                target_mean=target_mean,
                target_scale=target_scale,
                topology_confidence_min=0.0,
                ood_threshold=1.0e12,
                fixed_profile_projection=fixed_profile_projection,
                accepted_step_increment_limits=accepted_step_increment_limits,
                topology_hysteresis_min=topology_hysteresis_min,
            )
            scheduled_history, scheduled_mask = _predicted_previous_histories(
                train,
                runtime,
                max_rollout_steps=rollout_horizon,
            )
            common_history = train.history_mask & scheduled_mask
            if np.any(common_history):
                normalized_residual = (
                    scheduled_history[common_history] - train.history[common_history]
                ) / history_scale
                history_noise_scale = np.minimum(
                    np.quantile(np.abs(normalized_residual), 0.68, axis=0),
                    0.5,
                )

        network.train()
        permutation = rng.permutation(len(train))
        for start in range(0, len(train), batch_size):
            indexes = permutation[start : start + batch_size]
            use_teacher = rng.random(indexes.size) < teacher_probability
            batch_history = np.where(
                use_teacher[:, None, None],
                train.history[indexes],
                scheduled_history[indexes],
            )
            batch_history_mask = np.where(
                use_teacher[:, None],
                train.history_mask[indexes],
                scheduled_mask[indexes],
            )
            x = torch.as_tensor(
                (train.features[indexes] - feature_mean) / feature_scale,
                dtype=torch.float32,
            )
            normalized_batch_history = (batch_history - history_mean) / history_scale
            teacher_noise_mask = use_teacher[:, None] & batch_history_mask
            if np.any(history_noise_scale > 0.0):
                normalized_batch_history = normalized_batch_history + (
                    rng.normal(size=normalized_batch_history.shape)
                    * history_noise_scale[None, None, :]
                    * teacher_noise_mask[:, :, None]
                )
            h = torch.as_tensor(normalized_batch_history, dtype=torch.float32)
            h_mask = torch.as_tensor(batch_history_mask, dtype=torch.float32)
            batch_targets = _canonicalize_batch_targets(
                train.targets[indexes],
                train.features[indexes, 0],
            )
            target = torch.as_tensor(
                (batch_targets - target_mean) / target_scale,
                dtype=torch.float32,
            )
            previous_targets = _canonicalize_batch_targets(
                _history_to_training_targets(batch_history),
                train.features[indexes, 0],
            )
            previous_target = torch.as_tensor(
                (previous_targets - target_mean) / target_scale,
                dtype=torch.float32,
            )
            counts = torch.as_tensor(train.patch_count[indexes], dtype=torch.long)
            topology_weights = torch.as_tensor(
                _direct_topology_weights(train, indexes),
                dtype=torch.float32,
            )
            geometry_weights = torch.as_tensor(
                _direct_sample_weights(train, indexes),
                dtype=torch.float32,
            )
            topology_logits, query_output = network(x, h, h_mask)
            loss = _training_loss(
                torch,
                functional,
                topology_logits,
                query_output,
                target,
                previous_target,
                h_mask,
                counts,
                topology_weights,
                geometry_weights,
                class_weights_tensor,
                torch.as_tensor(target_mean, dtype=torch.float32),
                torch.as_tensor(target_scale, dtype=torch.float32),
                moment_regression_weight,
            )
            mirror_features, mirror_history, mirror_history_mask, mirror_targets = _mirror_batch(
                train.features[indexes],
                batch_history,
                batch_history_mask,
                train.targets[indexes],
                train.patch_count[indexes],
            )
            mirror_topology, mirror_queries = network(
                torch.as_tensor(
                    (mirror_features - feature_mean) / feature_scale,
                    dtype=torch.float32,
                ),
                torch.as_tensor(
                    (mirror_history - history_mean) / history_scale,
                    dtype=torch.float32,
                ),
                torch.as_tensor(mirror_history_mask, dtype=torch.float32),
            )
            canonical_mirror_targets = _canonicalize_batch_targets(
                mirror_targets,
                mirror_features[:, 0],
            )
            mirror_target = torch.as_tensor(
                (canonical_mirror_targets - target_mean) / target_scale,
                dtype=torch.float32,
            )
            canonical_mirror_previous = _canonicalize_batch_targets(
                _history_to_training_targets(mirror_history),
                mirror_features[:, 0],
            )
            mirror_previous_target = torch.as_tensor(
                (canonical_mirror_previous - target_mean) / target_scale,
                dtype=torch.float32,
            )
            mirror_loss = _training_loss(
                torch,
                functional,
                mirror_topology,
                mirror_queries,
                mirror_target,
                mirror_previous_target,
                torch.as_tensor(mirror_history_mask, dtype=torch.float32),
                counts,
                topology_weights,
                geometry_weights,
                class_weights_tensor,
                torch.as_tensor(target_mean, dtype=torch.float32),
                torch.as_tensor(target_scale, dtype=torch.float32),
                moment_regression_weight,
            )
            loss = loss + 0.1 * mirror_loss
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(network.parameters(), max_norm=5.0)
            optimizer.step()
        scheduler.step()

        validation_runtime = _torch_to_runtime(
            network,
            feature_mean=feature_mean,
            feature_scale=feature_scale,
            history_mean=history_mean,
            history_scale=history_scale,
            target_mean=target_mean,
            target_scale=target_scale,
            topology_confidence_min=0.0,
            ood_threshold=1.0e12,
            fixed_profile_projection=fixed_profile_projection,
            accepted_step_increment_limits=accepted_step_increment_limits,
            topology_hysteresis_min=topology_hysteresis_min,
        )
        validation_history, validation_history_mask = _predicted_previous_histories(
            validation,
            validation_runtime,
        )
        validation_loss = _dataset_loss(
            torch,
            functional,
            network,
            validation,
            feature_mean,
            feature_scale,
            history_mean,
            history_scale,
            target_mean,
            target_scale,
            class_weights_tensor,
            batch_size,
            history=validation_history,
            history_mask=validation_history_mask,
            moment_regression_weight=moment_regression_weight,
        )
        if epoch == schedule_boundary:
            # Models selected before this point have never trained on their own
            # history distribution.  Reset early-stopping selection so a
            # teacher-forced checkpoint cannot win merely because the rollout
            # curriculum has not started yet.
            best_loss = float("inf")
            stale = 0
        improved = validation_loss < best_loss - 1.0e-6
        if improved:
            best_loss = validation_loss
            best_epoch = epoch
            best_state = copy.deepcopy(network.state_dict())
            stale = 0
        else:
            stale += 1
        if epoch == 0 or (epoch + 1) % 10 == 0 or improved and epoch == schedule_boundary:
            print(
                f"epoch {epoch + 1}/{epochs}: validation_loss={validation_loss:.6g}, "
                f"best_epoch={best_epoch + 1}, stale={stale}",
                flush=True,
            )
        if epoch >= schedule_boundary and stale >= patience:
            break

    network.load_state_dict(best_state)
    evaluation_runtime = _torch_to_runtime(
        network,
        feature_mean=feature_mean,
        feature_scale=feature_scale,
        history_mean=history_mean,
        history_scale=history_scale,
        target_mean=target_mean,
        target_scale=target_scale,
        topology_confidence_min=0.0,
        ood_threshold=1.0e12,
        fixed_profile_projection=fixed_profile_projection,
        accepted_step_increment_limits=accepted_step_increment_limits,
        topology_hysteresis_min=topology_hysteresis_min,
    )
    closed_loop_histories = {
        name: _predicted_previous_histories(dataset, evaluation_runtime)
        for name, dataset in {
            "train": train,
            "validation": validation,
            "test": test,
        }.items()
    }
    confidence_min = _calibrated_confidence(
        torch,
        network,
        validation,
        feature_mean,
        feature_scale,
        history_mean,
        history_scale,
        batch_size,
        history=closed_loop_histories["validation"][0],
        history_mask=closed_loop_histories["validation"][1],
    )
    # Runtime bounds are applied after canonical targets have been mirrored
    # back to physical left/right coordinates.  Calibrating them from the
    # canonical targets alone would retain only the positive branch and clip
    # every physical right-side angle to a positive lower bound.
    physical_valid_targets = np.asarray(train.targets, dtype=float)[supervised_patch_mask]
    angle_values = physical_valid_targets[:, [7, 12]].reshape(-1)
    model = _torch_to_runtime(
        network,
        feature_mean=feature_mean,
        feature_scale=feature_scale,
        history_mean=history_mean,
        history_scale=history_scale,
        target_mean=target_mean,
        target_scale=target_scale,
        topology_confidence_min=confidence_min,
        ood_threshold=4.0,
        angle_min_rad=max(-1.45, float(np.quantile(angle_values, 0.001) - 0.02)),
        angle_max_rad=min(1.45, float(np.quantile(angle_values, 0.999) + 0.02)),
        feature_ood_scale=feature_ood_scale,
        history_ood_scale=history_ood_scale,
        fixed_profile_projection=fixed_profile_projection,
        accepted_step_increment_limits=accepted_step_increment_limits,
        topology_hysteresis_min=topology_hysteresis_min,
    )
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    model_path = output / "model.npz"
    save_wrcp_net_a1_direct(model_path, model)
    metrics = {
        "model": "WRCP-Net A1 Direct Set",
        "schema": "wrcp-net-a1-direct-set-v1",
        "best_epoch": int(best_epoch),
        "best_validation_loss": float(best_loss),
        "topology_confidence_min": float(confidence_min),
        "ood_calibration": {
            "method": "per-field full training-support standardized envelope",
            "threshold": 4.0,
            "training_support_quantile": OOD_CALIBRATION_QUANTILE,
            "training_support_target_distance": OOD_CALIBRATION_TARGET_DISTANCE,
            "feature_scale": feature_ood_scale.tolist(),
            "history_scale": history_ood_scale.tolist(),
        },
        "evaluation_history": "closed-loop-autoregressive",
        "fixed_profile_projection": {
            "enabled": True,
            "method": "scalar inverse wheel trace and scalar rail-height interpolation",
            "reconstructed_gap_samples": 0,
        },
        "rollout_training": {
            "method": "stage-aware pushforward curriculum with residual-calibrated history noise",
            "maximum_rollout_steps": int(maximum_rollout_steps),
            "final_teacher_probability": float(final_teacher_probability),
            "rollout_start_fraction": float(rollout_start_fraction),
            "rollout_refresh_epochs": int(rollout_refresh_epochs),
            "history_noise_scale_normalized": history_noise_scale.tolist(),
        },
        "training_data": {
            "samples": len(train),
            "source_counts": {
                str(value): int(count)
                for value, count in zip(
                    *np.unique(np.asarray(train.metadata["source"], dtype=str), return_counts=True),
                    strict=True,
                )
            },
            "runtime_dagger_sample_weight": RUNTIME_DAGGER_SAMPLE_WEIGHT,
            "runtime_dagger_topology_weight": 0.0,
            "cal1_geometry_weight": CAL1_GEOMETRY_WEIGHT,
            "early_cal_geometry_weight": EARLY_CAL_GEOMETRY_WEIGHT,
            "early_cal_max_step": EARLY_CAL_MAX_STEP,
            "transition_cal_geometry_weight": TRANSITION_CAL_GEOMETRY_WEIGHT,
            "transition_cal_max_step": TRANSITION_CAL_MAX_STEP,
            "corrected_penetration_regression_weight": (
                CORRECTED_PENETRATION_REGRESSION_WEIGHT
            ),
            "peak_penetration_increment_regression_weight": (
                PEAK_PENETRATION_INCREMENT_REGRESSION_WEIGHT
            ),
            "hertz_force_proxy_loss_weight": HERTZ_FORCE_PROXY_LOSS_WEIGHT,
            "shape_moment_regression_weight": float(moment_regression_weight),
            "topology_supervision": supervision_summary,
            "warm_started": initial_model is not None,
        },
        "train": _classification_metrics(
            torch, network, train, feature_mean, feature_scale, history_mean, history_scale,
            history=closed_loop_histories["train"][0],
            history_mask=closed_loop_histories["train"][1],
        ),
        "validation": _classification_metrics(
            torch, network, validation, feature_mean, feature_scale, history_mean, history_scale,
            history=closed_loop_histories["validation"][0],
            history_mask=closed_loop_histories["validation"][1],
        ),
        "test": _classification_metrics(
            torch, network, test, feature_mean, feature_scale, history_mean, history_scale,
            history=closed_loop_histories["test"][0],
            history_mask=closed_loop_histories["test"][1],
        ),
    }
    metrics_path = output / "metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return WRCPNetA1DirectTrainingResult(model, model_path, metrics_path, best_epoch, metrics)


def _training_loss(
    torch: Any,
    functional: Any,
    topology_logits: Any,
    query_output: Any,
    target: Any,
    previous_target: Any,
    previous_mask: Any,
    counts: Any,
    topology_weights: Any,
    geometry_weights: Any,
    class_weights: Any,
    target_mean: Any,
    target_scale: Any,
    moment_regression_weight: float = MOMENT_REGRESSION_WEIGHT,
) -> Any:
    topology_values = functional.cross_entropy(
            topology_logits,
            counts,
            weight=class_weights,
            reduction="none",
    )
    topology_loss = _weighted_torch_mean(torch, topology_values, topology_weights)
    object_logits = query_output[:, :, 0]
    predictions = query_output[:, :, 1:]
    # Batched two-slot Hungarian matching.  For one patch, choose the cheaper
    # query; for two patches, compare the direct and swapped assignments.  The
    # former implementation performed the same operations in a Python loop for
    # every row, which dominated full-Cal training time.
    prediction_grid = predictions[:, :, None, :].expand(
        -1, -1, MAX_PATCHES, -1
    )
    target_grid = target[:, None, :, :].expand(
        -1, MAX_PATCHES, -1, -1
    )
    pairwise_cost = torch.mean(
        functional.smooth_l1_loss(
            prediction_grid,
            target_grid,
            reduction="none",
        ),
        dim=3,
    )
    query_rows = torch.arange(MAX_PATCHES, device=predictions.device)[None, :]
    teacher_indexes = query_rows.expand(predictions.shape[0], -1).clone()
    swapped = (
        pairwise_cost[:, 0, 1] + pairwise_cost[:, 1, 0]
        < pairwise_cost[:, 0, 0] + pairwise_cost[:, 1, 1]
    ).detach()
    swapped_indexes = torch.as_tensor(
        [1, 0], dtype=torch.long, device=predictions.device
    )[None, :].expand(predictions.shape[0], -1)
    teacher_indexes = torch.where(swapped[:, None], swapped_indexes, teacher_indexes)
    single_query = torch.argmin(pairwise_cost[:, :, 0].detach(), dim=1)
    single = counts == 1
    teacher_indexes[single] = 0
    matched = counts[:, None] == 2
    matched = matched | (single[:, None] & (query_rows == single_query[:, None]))
    matched = matched & (geometry_weights[:, None] > 0.0)
    assigned_targets = torch.gather(
        target,
        1,
        teacher_indexes[:, :, None].expand(-1, -1, target.shape[2]),
    )
    pair_weights = geometry_weights[:, None].expand_as(object_logits) * matched
    regression_field_weights = torch.ones(
        (target.shape[2],), dtype=predictions.dtype, device=predictions.device
    )
    regression_field_weights[6] = CORRECTED_PENETRATION_REGRESSION_WEIGHT
    regression_field_weights[11] = PEAK_PENETRATION_INCREMENT_REGRESSION_WEIGHT
    regression_field_weights[13:16] = float(moment_regression_weight)
    regression_values = torch.sum(
        functional.smooth_l1_loss(predictions, assigned_targets, reduction="none")
        * regression_field_weights[None, None, :],
        dim=2,
    ) / torch.sum(regression_field_weights)
    regression_loss = _weighted_torch_mean(torch, regression_values, pair_weights)
    object_targets = matched.to(dtype=object_logits.dtype)
    # Query objectness is an auxiliary topology signal.  DAgger rows with
    # topology weight zero must therefore not update it; their supervision is
    # limited to the matched local geometry coordinates below.
    object_weights = torch.where(
        topology_weights > 0.0,
        geometry_weights,
        torch.zeros_like(geometry_weights),
    )
    object_loss = _weighted_torch_mean(
        torch,
        torch.mean(
            functional.binary_cross_entropy_with_logits(
                object_logits,
                object_targets,
                reduction="none",
            ),
            dim=1,
        ),
        object_weights,
    )
    expected_count = torch.sum(torch.sigmoid(object_logits), dim=1)
    topology_count = torch.sum(torch.softmax(topology_logits, dim=1) * torch.arange(3, device=topology_logits.device), dim=1)
    consistency_loss = _weighted_torch_mean(
        torch,
        (expected_count - topology_count) ** 2,
        topology_weights,
    )
    shape_loss = torch.zeros((), dtype=topology_logits.dtype, device=topology_logits.device)
    hertz_force_proxy_loss = torch.zeros_like(shape_loss)
    continuity_loss = torch.zeros_like(shape_loss)
    if bool(torch.any(matched).item()):
        predicted = predictions[matched]
        weights = pair_weights[matched].to(dtype=predicted.dtype, device=predicted.device)
        physical = predicted * target_scale + target_mean
        teacher_physical = assigned_targets[matched] * target_scale + target_mean
        predicted_corrected_penetration = functional.softplus(physical[:, 6]) + 1.0e-12
        teacher_corrected_penetration = (
            functional.softplus(teacher_physical[:, 6]) + 1.0e-12
        )
        predicted_peak_penetration = (
            predicted_corrected_penetration
            + functional.softplus(physical[:, 11])
            + 1.0e-12
        )
        teacher_peak_penetration = (
            teacher_corrected_penetration
            + functional.softplus(teacher_physical[:, 11])
            + 1.0e-12
        )
        penetration_floor = 1.0e-5
        corrected_force_error = (
            predicted_corrected_penetration.pow(1.5)
            - teacher_corrected_penetration.pow(1.5)
        ) / (teacher_corrected_penetration.pow(1.5) + penetration_floor**1.5)
        peak_force_error = (
            predicted_peak_penetration.pow(1.5)
            - teacher_peak_penetration.pow(1.5)
        ) / (teacher_peak_penetration.pow(1.5) + penetration_floor**1.5)
        hertz_force_proxy_values = 0.5 * (
            functional.smooth_l1_loss(
                corrected_force_error,
                torch.zeros_like(corrected_force_error),
                reduction="none",
            )
            + functional.smooth_l1_loss(
                peak_force_error,
                torch.zeros_like(peak_force_error),
                reduction="none",
            )
        )
        hertz_force_proxy_loss = _weighted_torch_mean(
            torch,
            hertz_force_proxy_values,
            weights,
        )
        moments = torch.sigmoid(physical[:, 13:16])
        m1, m15, m2 = moments[:, 0], moments[:, 1], moments[:, 2]
        violations = (
            functional.relu(m15 - m1)
            + functional.relu(m2 - m15)
            + functional.relu(m1**1.5 - m15)
            + functional.relu(m1**2 - m2)
        )
        shape_loss = _weighted_torch_mean(torch, violations, weights)
        continuity_columns = torch.as_tensor(
            [0, 1, 2, 6, 8, 11, 13, 14, 15],
            device=predicted.device,
        )
        # Explicit accepted-step increment consistency.  The same accepted
        # history anchors prediction and teacher, while the nonlinear target
        # parameterization keeps this term distinct from raw slot regression.
        assigned_previous = torch.gather(
            previous_target,
            1,
            teacher_indexes[:, :, None].expand(-1, -1, previous_target.shape[2]),
        )
        assigned_previous_mask = torch.gather(previous_mask, 1, teacher_indexes)
        continuity_mask = matched & assigned_previous_mask.to(dtype=torch.bool)
        if bool(torch.any(continuity_mask).item()):
            prediction_increment = torch.index_select(
                predictions[continuity_mask] - assigned_previous[continuity_mask],
                1,
                continuity_columns,
            )
            teacher_increment = torch.index_select(
                assigned_targets[continuity_mask] - assigned_previous[continuity_mask],
                1,
                continuity_columns,
            )
            continuity_values = torch.mean(
                functional.smooth_l1_loss(
                    prediction_increment,
                    teacher_increment,
                    reduction="none",
                ),
                dim=1,
            )
            continuity_loss = _weighted_torch_mean(
                torch,
                continuity_values,
                pair_weights[continuity_mask],
            )
    return (
        topology_loss
        + regression_loss
        + object_loss
        + 0.25 * continuity_loss
        + 0.1 * shape_loss
        + HERTZ_FORCE_PROXY_LOSS_WEIGHT * hertz_force_proxy_loss
        + 0.1 * consistency_loss
    )


def _dataset_loss(
    torch: Any,
    functional: Any,
    network: Any,
    dataset: NetworkA1DirectDataset,
    feature_mean: np.ndarray,
    feature_scale: np.ndarray,
    history_mean: np.ndarray,
    history_scale: np.ndarray,
    target_mean: np.ndarray,
    target_scale: np.ndarray,
    class_weights: Any,
    batch_size: int,
    *,
    history: np.ndarray | None = None,
    history_mask: np.ndarray | None = None,
    moment_regression_weight: float = MOMENT_REGRESSION_WEIGHT,
) -> float:
    network.eval()
    losses = []
    history_values = dataset.history if history is None else np.asarray(history, dtype=float)
    mask_values = dataset.history_mask if history_mask is None else np.asarray(history_mask, dtype=bool)
    with torch.no_grad():
        for start in range(0, len(dataset), batch_size):
            stop = min(start + batch_size, len(dataset))
            topology, output = network(
                torch.as_tensor((dataset.features[start:stop] - feature_mean) / feature_scale, dtype=torch.float32),
                torch.as_tensor((history_values[start:stop] - history_mean) / history_scale, dtype=torch.float32),
                torch.as_tensor(mask_values[start:stop], dtype=torch.float32),
            )
            losses.append(float(_training_loss(
                torch,
                functional,
                topology,
                output,
                torch.as_tensor(
                    (
                        _canonicalize_batch_targets(
                            dataset.targets[start:stop],
                            dataset.features[start:stop, 0],
                        )
                        - target_mean
                    )
                    / target_scale,
                    dtype=torch.float32,
                ),
                torch.as_tensor(
                    (
                        _canonicalize_batch_targets(
                            _history_to_training_targets(history_values[start:stop]),
                            dataset.features[start:stop, 0],
                        )
                        - target_mean
                    )
                    / target_scale,
                    dtype=torch.float32,
                ),
                torch.as_tensor(mask_values[start:stop], dtype=torch.float32),
                torch.as_tensor(dataset.patch_count[start:stop], dtype=torch.long),
                torch.as_tensor(
                    _direct_topology_weights(dataset, np.arange(start, stop)),
                    dtype=torch.float32,
                ),
                torch.as_tensor(
                    _direct_sample_weights(dataset, np.arange(start, stop)),
                    dtype=torch.float32,
                ),
                class_weights,
                torch.as_tensor(target_mean, dtype=torch.float32),
                torch.as_tensor(target_scale, dtype=torch.float32),
                moment_regression_weight,
            ).item()))
    return float(np.mean(losses))


def _direct_sample_weights(
    dataset: NetworkA1DirectDataset,
    indexes: np.ndarray,
) -> np.ndarray:
    selected = np.asarray(indexes, dtype=int)
    counts = np.asarray(dataset.patch_count[selected], dtype=int)
    targets = np.asarray(dataset.targets[selected], dtype=float)
    active = np.arange(MAX_PATCHES)[None, :] < counts[:, None]
    penetration = np.logaddexp(0.0, targets[:, :, 6])
    shallow = np.any(active & (penetration <= 2.0e-5), axis=1)
    sample_class = np.asarray(
        dataset.metadata.get("sample_class", np.full((len(dataset),), "")),
        dtype=str,
    )[selected]
    emphasized = (counts == 2) | shallow | (sample_class == "boundary")
    weights = np.where(emphasized, 2.0, 1.0)
    source = np.asarray(
        dataset.metadata.get("source", np.full((len(dataset),), "")),
        dtype=str,
    )[selected]
    dagger = dagger_source_mask(source)
    metadata = dataset.metadata
    stage = np.asarray(
        metadata.get("stage", np.full((len(dataset),), "")),
        dtype=str,
    )[selected]
    step = np.asarray(
        metadata.get("step_index", np.zeros((len(dataset),), dtype=int)),
        dtype=int,
    )[selected]
    accepted = np.asarray(
        metadata.get("accepted_step_label", np.zeros((len(dataset),), dtype=bool)),
        dtype=bool,
    )[selected]
    cal1 = (
        accepted
        & (source == "full_cal_accepted")
        & (stage == "Cal")
        & (step == 1)
    )
    early_cal = (
        accepted
        & (source == "full_cal_accepted")
        & (stage == "Cal")
        & (step >= 2)
        & (step <= EARLY_CAL_MAX_STEP)
    )
    transition_cal = (
        accepted
        & (source == "full_cal_accepted")
        & (stage == "Cal")
        & (step > EARLY_CAL_MAX_STEP)
        & (step <= TRANSITION_CAL_MAX_STEP)
    )
    weights[cal1] = np.maximum(weights[cal1], CAL1_GEOMETRY_WEIGHT)
    weights[early_cal] = np.maximum(weights[early_cal], EARLY_CAL_GEOMETRY_WEIGHT)
    weights[transition_cal] = np.maximum(
        weights[transition_cal], TRANSITION_CAL_GEOMETRY_WEIGHT
    )
    eligible_dagger = (
        dagger
        & np.asarray(
            metadata.get("teacher_relabel", np.ones((len(dataset),), dtype=bool)),
            dtype=bool,
        )[selected]
        & np.asarray(
            metadata.get("topology_reference_available", np.zeros((len(dataset),), dtype=bool)),
            dtype=bool,
        )[selected]
        & np.asarray(
            metadata.get(
                "topology_matches_accepted_trajectory",
                np.zeros((len(dataset),), dtype=bool),
            ),
            dtype=bool,
        )[selected]
        & (counts > 0)
    )
    weights[dagger] = 0.0
    weights[eligible_dagger] = RUNTIME_DAGGER_SAMPLE_WEIGHT
    return weights


def _direct_topology_weights(
    dataset: NetworkA1DirectDataset,
    indexes: np.ndarray,
) -> np.ndarray:
    """Return topology-only weights; runtime DAgger is always excluded."""

    selected = np.asarray(indexes, dtype=int)
    all_source = np.asarray(
        dataset.metadata.get("source", np.full((len(dataset),), "")),
        dtype=str,
    )
    source = all_source[selected]
    accepted = np.asarray(
        dataset.metadata.get(
            "accepted_step_label",
            (all_source != "sobol") & ~dagger_source_mask(all_source),
        ),
        dtype=bool,
    )[selected]
    stage = np.asarray(
        dataset.metadata.get("stage", np.full((len(dataset),), "")),
        dtype=str,
    )[selected]
    full_cal = accepted & (source == "full_cal_accepted") & (stage == "Cal")
    other_accepted = accepted & ~full_cal
    dagger = dagger_source_mask(source)
    synthetic = ~accepted & ~dagger
    weights = np.zeros((selected.size,), dtype=float)
    weights[full_cal] = FULL_CAL_TOPOLOGY_WEIGHT
    weights[other_accepted] = OTHER_ACCEPTED_TOPOLOGY_WEIGHT
    weights[synthetic] = SYNTHETIC_TOPOLOGY_WEIGHT
    return weights


def _weighted_torch_mean(torch: Any, values: Any, weights: Any) -> Any:
    total = torch.sum(weights)
    return torch.sum(values * weights) / torch.clamp(total, min=1.0)


def _validate_split_isolation(
    train: NetworkA1DirectDataset,
    validation: NetworkA1DirectDataset,
    test: NetworkA1DirectDataset,
) -> None:
    datasets = {"train": train, "validation": validation, "test": test}
    groups = {
        name: set(np.asarray(dataset.metadata["group_id"], dtype=str))
        for name, dataset in datasets.items()
    }
    names = tuple(groups)
    for left_index, left in enumerate(names):
        for right in names[left_index + 1 :]:
            overlap = groups[left] & groups[right]
            if overlap:
                raise ValueError(
                    "network-A1 Direct train/validation/test groups must be disjoint: "
                    f"{left}/{right} overlap {sorted(overlap)[:3]}"
                )


def _validate_supervision_policy(
    train: NetworkA1DirectDataset,
    validation: NetworkA1DirectDataset,
    test: NetworkA1DirectDataset,
) -> dict[str, Any]:
    """Fail fast unless the formal topology supervision follows the locked policy."""

    if not 0.0 <= RUNTIME_DAGGER_SAMPLE_WEIGHT <= 1.0:
        raise ValueError("runtime DAgger geometry weight must be no greater than 1.0")

    train_source = np.asarray(train.metadata["source"], dtype=str)
    train_stage = np.asarray(train.metadata.get("stage", np.full((len(train),), "")), dtype=str)
    train_seed = np.asarray(train.metadata["irregularity_seed"], dtype=int)
    train_accepted = np.asarray(train.metadata["accepted_step_label"], dtype=bool)
    full_cal = (
        (train_source == "full_cal_accepted")
        & (train_stage == "Cal")
        & train_accepted
        & (train_seed >= 0)
    )
    full_cal_seeds = sorted(int(value) for value in np.unique(train_seed[full_cal]))
    if len(full_cal_seeds) < MINIMUM_FULL_CAL_TRAINING_SEEDS:
        raise ValueError(
            "formal topology training requires multi-seed full-Cal accepted steps; "
            f"found seeds {full_cal_seeds}"
        )

    topology_weights = _direct_topology_weights(train, np.arange(len(train)))
    full_cal_weight = float(np.sum(topology_weights[full_cal]))
    total_topology_weight = float(np.sum(topology_weights))
    if full_cal_weight <= 0.5 * total_topology_weight:
        raise ValueError(
            "full-Cal accepted steps must provide the majority of formal topology weight"
        )

    evaluation_seeds: dict[str, int] = {}
    for name, dataset, required_seed in (
        ("validation", validation, VALIDATION_IRREGULARITY_SEED),
        ("test", test, TEST_IRREGULARITY_SEED),
    ):
        seeds = np.asarray(dataset.metadata["irregularity_seed"], dtype=int)
        accepted = np.asarray(dataset.metadata["accepted_step_label"], dtype=bool)
        source = np.asarray(dataset.metadata["source"], dtype=str)
        if not np.all(seeds == required_seed):
            raise ValueError(
                f"{name} split must contain only irregularity seed {required_seed}; "
                f"found {sorted(int(value) for value in np.unique(seeds))}"
            )
        if not np.all(accepted & (source == "full_cal_accepted")):
            raise ValueError(
                f"{name} split must contain only full-Cal accepted-step labels"
            )
        evaluation_seeds[name] = required_seed

    dagger = dagger_source_mask(train_source)
    geometry_weights = _direct_sample_weights(train, np.arange(len(train)))
    return {
        "policy": "accepted-step-primary-topology-v2",
        "full_cal_training_seeds": full_cal_seeds,
        "full_cal_accepted_cal_rows": int(np.sum(full_cal)),
        "full_cal_topology_weight": full_cal_weight,
        "total_topology_weight": total_topology_weight,
        "full_cal_topology_weight_fraction": full_cal_weight / max(total_topology_weight, 1.0),
        "dagger_rows": int(np.sum(dagger)),
        "dagger_topology_weight": float(np.sum(topology_weights[dagger])),
        "dagger_geometry_rows": int(np.sum(geometry_weights[dagger] > 0.0)),
        "dagger_geometry_weight": float(np.sum(geometry_weights[dagger])),
        "evaluation_seeds": evaluation_seeds,
    }


def _history_to_training_targets(history: np.ndarray) -> np.ndarray:
    values = np.asarray(history, dtype=float)
    result = np.zeros(values.shape[:-1] + (len(TARGET_NAMES),), dtype=float)
    start = values[..., 0]
    end = values[..., 1]
    center = values[..., 5]
    width = np.maximum(end - start, 2.0e-12)
    result[..., 0] = center
    result[..., 1] = _softplus_inverse_array(np.maximum(center - start, 1.0e-12))
    result[..., 2] = _softplus_inverse_array(np.maximum(end - center, 1.0e-12))
    result[..., 3] = values[..., 2]
    result[..., 4] = values[..., 6]
    result[..., 5] = values[..., 7]
    result[..., 6] = _softplus_inverse_array(np.maximum(values[..., 8], 1.0e-12))
    result[..., 7] = values[..., 9]
    peak_fraction = np.clip((values[..., 13] - start) / width, 1.0e-7, 1.0 - 1.0e-7)
    result[..., 8] = np.log(peak_fraction) - np.log1p(-peak_fraction)
    result[..., 9] = values[..., 10]
    result[..., 10] = values[..., 14]
    peak_increment = np.maximum(values[..., 15] - values[..., 8], 1.0e-12)
    result[..., 11] = _softplus_inverse_array(peak_increment)
    result[..., 12] = values[..., 16]
    moments = np.clip(values[..., 17:20], 1.0e-7, 1.0 - 1.0e-7)
    result[..., 13:16] = np.log(moments) - np.log1p(-moments)
    return result


def _canonicalize_batch_targets(
    targets: np.ndarray,
    side_id: np.ndarray,
) -> np.ndarray:
    values = np.asarray(targets, dtype=float)
    flat = values.reshape(-1, values.shape[-1])
    repeated_side = np.repeat(
        np.asarray(side_id, dtype=float).reshape(values.shape[0]),
        int(np.prod(values.shape[1:-1], dtype=int)),
    )
    return _canonical_side_target_transform(flat, repeated_side).reshape(values.shape)


def _mirror_batch(
    features: np.ndarray,
    history: np.ndarray,
    history_mask: np.ndarray,
    targets: np.ndarray,
    counts: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    mirrored_features = np.asarray(features, dtype=float).copy()
    mirrored_features[:, 0] = 1.0 - mirrored_features[:, 0]
    mirrored_features[:, 1] *= -1.0
    mirrored_features[:, 3] *= -1.0
    mirrored_features[:, 4] *= -1.0

    mirrored_history = np.asarray(history, dtype=float).copy()
    start = mirrored_history[..., 0].copy()
    mirrored_history[..., 0] = -mirrored_history[..., 1]
    mirrored_history[..., 1] = -start
    mirrored_history[..., [3, 5, 7, 11, 13]] *= -1.0
    mirrored_history[..., [9, 16]] *= -1.0
    mirrored_history_mask = np.asarray(history_mask, dtype=bool).copy()

    mirrored_targets = np.asarray(targets, dtype=float).copy()
    mirrored_targets[..., 0] *= -1.0
    left_width = mirrored_targets[..., 1].copy()
    mirrored_targets[..., 1] = mirrored_targets[..., 2]
    mirrored_targets[..., 2] = left_width
    mirrored_targets[..., 5] *= -1.0
    mirrored_targets[..., [7, 8, 12]] *= -1.0

    two_patch = np.asarray(counts, dtype=int) == 2
    mirrored_targets[two_patch, :2] = mirrored_targets[two_patch, :2][:, ::-1]
    two_history = np.sum(mirrored_history_mask, axis=1) == 2
    mirrored_history[two_history, :2] = mirrored_history[two_history, :2][:, ::-1]
    mirrored_history_mask[two_history, :2] = mirrored_history_mask[two_history, :2][:, ::-1]
    return mirrored_features, mirrored_history, mirrored_history_mask, mirrored_targets


def _softplus_inverse_array(values: np.ndarray) -> np.ndarray:
    x = np.maximum(np.asarray(values, dtype=float), 1.0e-12)
    return x + np.log(-np.expm1(-x))


def _torch_to_runtime(
    network: Any,
    *,
    feature_mean: np.ndarray,
    feature_scale: np.ndarray,
    history_mean: np.ndarray,
    history_scale: np.ndarray,
    target_mean: np.ndarray,
    target_scale: np.ndarray,
    topology_confidence_min: float,
    ood_threshold: float,
    angle_min_rad: float = -1.45,
    angle_max_rad: float = 1.45,
    feature_ood_scale: np.ndarray | None = None,
    history_ood_scale: np.ndarray | None = None,
    fixed_profile_projection: dict[str, np.ndarray] | None = None,
    accepted_step_increment_limits: np.ndarray | None = None,
    topology_hysteresis_min: float = 0.0,
) -> WRCPNetA1DirectSet:
    def linear(layer: Any) -> tuple[np.ndarray, np.ndarray]:
        return (
            layer.weight.detach().cpu().numpy().T.astype(np.float32),
            layer.bias.detach().cpu().numpy().astype(np.float32),
        )

    state = tuple(linear(layer) for layer in network.state)
    history = tuple(linear(layer) for layer in network.history)
    fusion_weight, fusion_bias = linear(network.fusion)
    residual = tuple(tuple(linear(layer) for layer in block) for block in network.residual)
    topology_weight, topology_bias = linear(network.topology)
    decoder = tuple(linear(layer) for layer in network.decoder)
    return WRCPNetA1DirectSet(
        state_weights=tuple(value[0] for value in state),
        state_biases=tuple(value[1] for value in state),
        history_weights=tuple(value[0] for value in history),
        history_biases=tuple(value[1] for value in history),
        fusion_weight=fusion_weight,
        fusion_bias=fusion_bias,
        residual_weights=tuple((block[0][0], block[1][0]) for block in residual),
        residual_biases=tuple((block[0][1], block[1][1]) for block in residual),
        topology_weight=topology_weight,
        topology_bias=topology_bias,
        query_embeddings=network.query_embeddings.detach().cpu().numpy().astype(np.float32),
        decoder_weights=tuple(value[0] for value in decoder),
        decoder_biases=tuple(value[1] for value in decoder),
        feature_mean=feature_mean,
        feature_scale=feature_scale,
        history_mean=history_mean,
        history_scale=history_scale,
        target_mean=target_mean,
        target_scale=target_scale,
        feature_ood_scale=(
            np.ones((len(FEATURE_NAMES),), dtype=float)
            if feature_ood_scale is None
            else feature_ood_scale
        ),
        history_ood_scale=(
            np.ones((len(HISTORY_NAMES),), dtype=float)
            if history_ood_scale is None
            else history_ood_scale
        ),
        canonical_side_targets=True,
        topology_confidence_min=topology_confidence_min,
        topology_hysteresis_min=float(topology_hysteresis_min),
        ood_threshold=ood_threshold,
        angle_min_rad=angle_min_rad,
        angle_max_rad=angle_max_rad,
        accepted_step_increment_limits=(
            None
            if accepted_step_increment_limits is None
            else np.asarray(accepted_step_increment_limits, dtype=float).copy()
        ),
        **({} if fixed_profile_projection is None else fixed_profile_projection),
    )


def _load_runtime_weights_into_torch(
    torch: Any,
    network: Any,
    model: WRCPNetA1DirectSet,
) -> None:
    """Initialize the trainable network from an existing NumPy artifact."""

    def assign(layer: Any, weight: np.ndarray, bias: np.ndarray) -> None:
        layer.weight.copy_(torch.as_tensor(np.asarray(weight).T, dtype=layer.weight.dtype))
        layer.bias.copy_(torch.as_tensor(np.asarray(bias), dtype=layer.bias.dtype))

    with torch.no_grad():
        for layer, weight, bias in zip(
            network.state, model.state_weights, model.state_biases, strict=True
        ):
            assign(layer, weight, bias)
        for layer, weight, bias in zip(
            network.history, model.history_weights, model.history_biases, strict=True
        ):
            assign(layer, weight, bias)
        assign(network.fusion, model.fusion_weight, model.fusion_bias)
        for block, weights, biases in zip(
            network.residual,
            model.residual_weights,
            model.residual_biases,
            strict=True,
        ):
            for layer, weight, bias in zip(block, weights, biases, strict=True):
                assign(layer, weight, bias)
        assign(network.topology, model.topology_weight, model.topology_bias)
        for layer, weight, bias in zip(
            network.decoder, model.decoder_weights, model.decoder_biases, strict=True
        ):
            assign(layer, weight, bias)
        network.query_embeddings.copy_(
            torch.as_tensor(model.query_embeddings, dtype=network.query_embeddings.dtype)
        )


def _load_fixed_profile_projection(
    repo_root: str | Path | None,
) -> dict[str, np.ndarray]:
    """Build compact fixed-profile tables embedded in the runtime artifact."""

    from sditt.contact.geometry import _prepared_wheel_trace_profile
    from sditt.training_data.network_a import build_network_a_teacher_context

    context = build_network_a_teacher_context(repo_root)
    result: dict[str, np.ndarray] = {}
    for side, suffix in (("L", "left"), ("R", "right")):
        wheel_profile = (
            context.wheel_profiles.left if side == "L" else context.wheel_profiles.right
        )
        contact_angles = (
            context.wheel_profiles.contact_angle_left
            if side == "L"
            else context.wheel_profiles.contact_angle_right
        )
        prepared = _prepared_wheel_trace_profile(
            wheel_profile,
            contact_angles,
            dlb=context.dlb,
            discrete_len_flange=0.5e-5,
            discrete_len_tread=2.5e-5,
        )
        result[f"wheel_trace_profile_{suffix}"] = np.column_stack(
            (prepared.x_profile, prepared.rolling_radius, prepared.contact_angles)
        )
        rail = np.asarray(context.track_profiles.profile[side], dtype=float)
        result[f"rail_profile_{suffix}"] = rail[np.argsort(rail[:, 0])]
    return result


def _predicted_previous_histories(
    dataset: NetworkA1DirectDataset,
    model: WRCPNetA1DirectSet,
    *,
    max_rollout_steps: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    if (
        isinstance(model, WRCPNetA1DirectSet)
        and model.accepted_step_increment_limits is None
        and model.topology_hysteresis_min == 0.0
        and model.topology_confidence_min == 0.0
        and model.ood_threshold >= 1.0e10
    ):
        return _predicted_previous_histories_batched(
            dataset,
            model,
            max_rollout_steps=max_rollout_steps,
        )
    return _predicted_previous_histories_scalar(
        dataset,
        model,
        max_rollout_steps=max_rollout_steps,
    )


def _predicted_previous_histories_scalar(
    dataset: NetworkA1DirectDataset,
    model: WRCPNetA1DirectSet,
    *,
    max_rollout_steps: int | None,
) -> tuple[np.ndarray, np.ndarray]:
    history = dataset.history.copy()
    mask = dataset.history_mask.copy()
    predicted: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    rollout_depth: dict[int, int] = {}
    stage = np.asarray(
        dataset.metadata.get("stage", np.full((len(dataset),), "")),
        dtype=str,
    )
    for row, previous in enumerate(dataset.previous_row):
        previous = int(previous)
        # Deployment keeps the complete Preload stage traditional.  Cal-1 is
        # therefore anchored to the teacher's final accepted Preload patch;
        # model-induced history begins only at Cal-2.
        is_cal_rollout = (
            previous >= 0
            and stage[row] == "Cal"
            and stage[previous] == "Cal"
        )
        can_use_prediction = is_cal_rollout and previous in predicted and (
            max_rollout_steps is None
            or rollout_depth.get(previous, 0) < max_rollout_steps
        )
        if can_use_prediction:
            history[row], mask[row] = predicted[previous]
            current_depth = rollout_depth.get(previous, 0) + 1
        else:
            current_depth = 0
        try:
            result = model.predict(
                features=dataset.features[row],
                history=history[row],
                history_mask=mask[row],
            )
            predicted[row] = geometry_to_history(result.geometry)
            rollout_depth[row] = current_depth
        except RuntimeError:
            continue
    return history, mask


def _predicted_previous_histories_batched(
    dataset: NetworkA1DirectDataset,
    model: WRCPNetA1DirectSet,
    *,
    max_rollout_steps: int | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Propagate independent accepted-step windows in depth-wise batches."""

    history = dataset.history.copy()
    mask = dataset.history_mask.copy()
    stage = np.asarray(
        dataset.metadata.get("stage", np.full((len(dataset),), "")),
        dtype=str,
    )
    previous = np.asarray(dataset.previous_row, dtype=int)
    dependency = np.full((len(dataset),), -1, dtype=int)
    depth = np.zeros((len(dataset),), dtype=int)
    for row, previous_row in enumerate(previous):
        if (
            previous_row >= 0
            and stage[row] == "Cal"
            and stage[previous_row] == "Cal"
            and (
                max_rollout_steps is None
                or depth[previous_row] < max_rollout_steps
            )
        ):
            dependency[row] = previous_row
            depth[row] = depth[previous_row] + 1

    predicted_history = np.zeros_like(history)
    predicted_mask = np.zeros_like(mask)
    predicted_valid = np.zeros((len(dataset),), dtype=bool)
    for current_depth in range(int(np.max(depth, initial=0)) + 1):
        indexes = np.flatnonzero(depth == current_depth)
        if not indexes.size:
            continue
        dependencies = dependency[indexes]
        linked = dependencies >= 0
        usable = linked.copy()
        usable[linked] &= predicted_valid[dependencies[linked]]
        if np.any(usable):
            target_rows = indexes[usable]
            source_rows = dependencies[usable]
            history[target_rows] = predicted_history[source_rows]
            mask[target_rows] = predicted_mask[source_rows]
        batch_history, batch_mask, valid = _predict_history_batch(
            model,
            dataset.features[indexes],
            history[indexes],
            mask[indexes],
        )
        predicted_history[indexes] = batch_history
        predicted_mask[indexes] = batch_mask
        predicted_valid[indexes] = valid
    return history, mask


def _predict_history_batch(
    model: WRCPNetA1DirectSet,
    features: np.ndarray,
    history: np.ndarray,
    history_mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Decode runtime histories in bulk; unusual two-patch rows use exact fallback."""

    x = np.asarray(features, dtype=float)
    h = np.asarray(history, dtype=float)
    mask = np.asarray(history_mask, dtype=bool)
    topology_logits, query_raw = _runtime_forward_raw_batch(model, x, h, mask)
    counts = np.argmax(topology_logits, axis=1)
    query_order = np.argsort(-query_raw[:, :, 0], axis=1, kind="stable")
    result = np.zeros((x.shape[0], MAX_PATCHES, len(HISTORY_NAMES)), dtype=float)
    result_mask = np.zeros((x.shape[0], MAX_PATCHES), dtype=bool)
    valid = np.isfinite(x).all(axis=1) & np.all(
        np.where(mask[:, :, None], np.isfinite(h), True),
        axis=(1, 2),
    )

    one = np.flatnonzero((counts == 1) & valid)
    if one.size:
        selected = query_order[one, 0]
        decoded = (
            query_raw[one, selected, 1:] * model.target_scale[None, :]
            + model.target_mean[None, :]
        )
        if model.canonical_side_targets:
            decoded = _canonical_side_target_transform(decoded, x[one, 0])
        if model.has_fixed_profile_projection:
            decoded, projected = _project_fixed_profiles_batch(model, decoded, x[one])
        else:
            projected = np.ones((one.size,), dtype=bool)
        history_values = _targets_to_history_batch(
            decoded,
            x[one],
            angle_min=model.angle_min_rad,
            angle_max=model.angle_max_rad,
        )
        accepted = projected & np.isfinite(history_values).all(axis=1)
        accepted_rows = one[accepted]
        result[accepted_rows, 0] = history_values[accepted]
        result_mask[accepted_rows, 0] = True
        valid[one] &= accepted

    # No-contact rows produce a valid empty history.  The formal interval
    # dataset is single-patch; retain exact scalar semantics for the rare
    # two-patch prediction so the generic trainer remains correct.
    for row in np.flatnonzero((counts == 2) & valid):
        try:
            prediction = model.predict(
                features=x[row],
                history=h[row],
                history_mask=mask[row],
            )
        except RuntimeError:
            valid[row] = False
            continue
        result[row], result_mask[row] = geometry_to_history(prediction.geometry)
    return result, result_mask, valid


def _runtime_forward_raw_batch(
    model: WRCPNetA1DirectSet,
    features: np.ndarray,
    history: np.ndarray,
    history_mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    x = (features - model.feature_mean) / model.feature_scale
    h = (history - model.history_mean) / model.history_scale
    h = np.where(history_mask[:, :, None], h, 0.0)
    state = x
    for weight, bias in zip(model.state_weights, model.state_biases, strict=True):
        state = state @ weight + bias
        state = state / (1.0 + np.exp(-np.clip(state, -60.0, 60.0)))
    encoded = h
    for weight, bias in zip(model.history_weights, model.history_biases, strict=True):
        encoded = encoded @ weight + bias
        encoded = encoded / (1.0 + np.exp(-np.clip(encoded, -60.0, 60.0)))
    pooled = np.sum(encoded * history_mask[:, :, None], axis=1)
    count = np.sum(history_mask, axis=1, keepdims=True) / MAX_PATCHES
    latent = np.concatenate((state, pooled, count), axis=1)
    latent = latent @ model.fusion_weight + model.fusion_bias
    latent = latent / (1.0 + np.exp(-np.clip(latent, -60.0, 60.0)))
    for (weight_1, weight_2), (bias_1, bias_2) in zip(
        model.residual_weights,
        model.residual_biases,
        strict=True,
    ):
        residual = latent @ weight_1 + bias_1
        residual = residual / (1.0 + np.exp(-np.clip(residual, -60.0, 60.0)))
        latent = latent + residual @ weight_2 + bias_2
        latent = latent / (1.0 + np.exp(-np.clip(latent, -60.0, 60.0)))
    topology = latent @ model.topology_weight + model.topology_bias
    queries = np.broadcast_to(
        model.query_embeddings[None, :, :],
        (features.shape[0], MAX_PATCHES, model.query_embeddings.shape[1]),
    )
    decoded = np.concatenate(
        (np.broadcast_to(latent[:, None, :], (features.shape[0], MAX_PATCHES, latent.shape[1])), queries),
        axis=2,
    )
    for index, (weight, bias) in enumerate(
        zip(model.decoder_weights, model.decoder_biases, strict=True)
    ):
        decoded = decoded @ weight + bias
        if index + 1 < len(model.decoder_weights):
            decoded = decoded / (1.0 + np.exp(-np.clip(decoded, -60.0, 60.0)))
    return topology, decoded


def _project_fixed_profiles_batch(
    model: WRCPNetA1DirectSet,
    targets: np.ndarray,
    features: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    result = np.asarray(targets, dtype=float).copy()
    valid = np.ones((result.shape[0],), dtype=bool)
    for right_side, wheel, rail in (
        (False, model.wheel_trace_profile_left, model.rail_profile_left),
        (True, model.wheel_trace_profile_right, model.rail_profile_right),
    ):
        rows = np.flatnonzero((features[:, 0] >= 0.5) == right_side)
        if not rows.size:
            continue
        wheel_table = np.asarray(wheel, dtype=float)
        rail_table = np.asarray(rail, dtype=float)
        values = result[rows]
        x = features[rows]
        center = values[:, 0]
        left_width = np.logaddexp(0.0, values[:, 1]) + 1.0e-9
        right_width = np.logaddexp(0.0, values[:, 2]) + 1.0e-9
        start = center - left_width
        end = center + right_width
        peak = start + _sigmoid_array(values[:, 8]) * (end - start)
        corrected_lateral, corrected_x, corrected_z, corrected_angle, corrected_valid = (
            _invert_fixed_wheel_lateral_batch(center, wheel_table, x)
        )
        peak_lateral, peak_x, peak_z, peak_angle, peak_valid = (
            _invert_fixed_wheel_lateral_batch(peak, wheel_table, x)
        )
        rail_min = float(rail_table[0, 0]) - 1.0e-12
        rail_max = float(rail_table[-1, 0]) + 1.0e-12
        rail_valid = (
            (center >= rail_min)
            & (center <= rail_max)
            & (peak >= rail_min)
            & (peak <= rail_max)
        )
        corrected_rail_z = np.interp(center, rail_table[:, 0], rail_table[:, 1])
        peak_rail_z = np.interp(peak, rail_table[:, 0], rail_table[:, 1])
        corrected_penetration = corrected_z - corrected_rail_z + x[:, 2] + x[:, 5]
        peak_penetration = peak_z - peak_rail_z + x[:, 2] + x[:, 5]
        use_corrected_peak = peak_penetration < corrected_penetration
        peak[use_corrected_peak] = center[use_corrected_peak]
        peak_lateral[use_corrected_peak] = corrected_lateral[use_corrected_peak]
        peak_x[use_corrected_peak] = corrected_x[use_corrected_peak]
        peak_z[use_corrected_peak] = corrected_z[use_corrected_peak]
        peak_angle[use_corrected_peak] = corrected_angle[use_corrected_peak]
        peak_rail_z[use_corrected_peak] = corrected_rail_z[use_corrected_peak]
        peak_penetration[use_corrected_peak] = corrected_penetration[use_corrected_peak]
        center_fraction = np.clip(
            (center - start) / np.maximum(end - start, 2.0e-12),
            1.0e-7,
            1.0 - 1.0e-7,
        )
        values[use_corrected_peak, 8] = _logit_array(
            center_fraction[use_corrected_peak]
        )
        penetration_valid = corrected_penetration > 0.0
        peak_increment = np.maximum(
            peak_penetration - corrected_penetration,
            1.0e-12,
        )
        values[:, 3] = corrected_x
        values[:, 4] = corrected_rail_z
        values[:, 5] = corrected_lateral
        values[:, 6] = _softplus_inverse_array(corrected_penetration)
        values[:, 7] = corrected_angle
        values[:, 9] = peak_x
        values[:, 10] = peak_rail_z
        values[:, 11] = _softplus_inverse_array(peak_increment)
        values[:, 12] = peak_angle
        result[rows] = values
        valid[rows] &= corrected_valid & peak_valid & rail_valid & penetration_valid
    return result, valid


def _invert_fixed_wheel_lateral_batch(
    target_y: np.ndarray,
    table: np.ndarray,
    features: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    lower = np.full(target_y.shape, float(table[0, 0]))
    upper = np.full(target_y.shape, float(table[-1, 0]))
    _, lower_y, _, _ = _fixed_wheel_trace_point_batch(lower, table, features)
    _, upper_y, _, _ = _fixed_wheel_trace_point_batch(upper, table, features)
    minimum = np.minimum(lower_y, upper_y) - 1.0e-12
    maximum = np.maximum(lower_y, upper_y) + 1.0e-12
    valid = (target_y >= minimum) & (target_y <= maximum)
    increasing = upper_y >= lower_y
    for _ in range(30):
        middle = 0.5 * (lower + upper)
        _, middle_y, _, _ = _fixed_wheel_trace_point_batch(middle, table, features)
        move_lower = (middle_y < target_y) == increasing
        lower = np.where(move_lower, middle, lower)
        upper = np.where(move_lower, upper, middle)
    lateral = 0.5 * (lower + upper)
    track_x, _, track_z, angle = _fixed_wheel_trace_point_batch(
        lateral, table, features
    )
    return lateral, track_x, track_z, angle, valid


def _fixed_wheel_trace_point_batch(
    wheel_lateral: np.ndarray,
    table: np.ndarray,
    features: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    radius = np.interp(wheel_lateral, table[:, 0], table[:, 1])
    angle = np.interp(wheel_lateral, table[:, 0], table[:, 2])
    roll = features[:, 3]
    yaw = features[:, 4]
    lateral = features[:, 1]
    lx = -np.cos(roll) * np.sin(yaw)
    ly = np.cos(roll) * np.cos(yaw)
    lz = np.sin(roll)
    tangent = np.tan(angle)
    m_value = np.sqrt(np.maximum(0.0, 1.0 - lx**2 * (1.0 + tangent**2)))
    denominator = 1.0 - lx**2
    track_x = lx * (wheel_lateral + radius * tangent)
    track_y = (
        wheel_lateral * ly
        + lateral
        - radius * (lx**2 * ly * tangent + lz * m_value) / denominator
    )
    track_z = (
        wheel_lateral * lz
        - radius * (lx**2 * lz * tangent - ly * m_value) / denominator
    )
    return track_x, track_y, track_z, angle


def _targets_to_history_batch(
    targets: np.ndarray,
    features: np.ndarray,
    *,
    angle_min: float,
    angle_max: float,
) -> np.ndarray:
    values = np.asarray(targets, dtype=float)
    center = values[:, 0]
    left_width = np.logaddexp(0.0, values[:, 1]) + 1.0e-9
    right_width = np.logaddexp(0.0, values[:, 2]) + 1.0e-9
    start = center - left_width
    end = center + right_width
    corrected_penetration = np.logaddexp(0.0, values[:, 6]) + 1.0e-12
    peak = start + _sigmoid_array(values[:, 8]) * (end - start)
    peak_penetration = corrected_penetration + np.logaddexp(0.0, values[:, 11]) + 1.0e-12
    corrected_angle = np.clip(values[:, 7], angle_min, angle_max)
    peak_angle = np.clip(values[:, 12], angle_min, angle_max)
    moments = np.clip(_sigmoid_array(values[:, 13:16]), 1.0e-7, 1.0 - 1.0e-7)
    moments[:, 1] = np.clip(moments[:, 1], moments[:, 0] ** 1.5, moments[:, 0])
    moments[:, 2] = np.clip(moments[:, 2], moments[:, 0] ** 2, moments[:, 1])
    result = np.zeros((values.shape[0], len(HISTORY_NAMES)), dtype=float)
    result[:, 0] = start
    result[:, 1] = end
    result[:, 2] = values[:, 3]
    result[:, 3] = center
    result[:, 4] = corrected_penetration + values[:, 4] - features[:, 2] - features[:, 5]
    result[:, 5] = center
    result[:, 6] = values[:, 4]
    result[:, 7] = values[:, 5]
    result[:, 8] = corrected_penetration
    result[:, 9] = corrected_angle
    result[:, 10] = values[:, 9]
    result[:, 11] = peak
    result[:, 12] = peak_penetration + values[:, 10] - features[:, 2] - features[:, 5]
    result[:, 13] = peak
    result[:, 14] = values[:, 10]
    result[:, 15] = peak_penetration
    result[:, 16] = peak_angle
    result[:, 17:20] = moments
    return result


def _sigmoid_array(values: np.ndarray) -> np.ndarray:
    x = np.asarray(values, dtype=float)
    return 1.0 / (1.0 + np.exp(-np.clip(x, -60.0, 60.0)))


def _logit_array(values: np.ndarray) -> np.ndarray:
    x = np.clip(np.asarray(values, dtype=float), 1.0e-7, 1.0 - 1.0e-7)
    return np.log(x) - np.log1p(-x)


def _calibrated_confidence(
    torch: Any,
    network: Any,
    dataset: NetworkA1DirectDataset,
    feature_mean: np.ndarray,
    feature_scale: np.ndarray,
    history_mean: np.ndarray,
    history_scale: np.ndarray,
    batch_size: int,
    *,
    history: np.ndarray | None = None,
    history_mask: np.ndarray | None = None,
) -> float:
    probabilities, predictions = _topology_predictions(
        torch, network, dataset, feature_mean, feature_scale, history_mean, history_scale, batch_size,
        history=history,
        history_mask=history_mask,
    )
    correct = predictions == dataset.patch_count
    confidence = probabilities[np.arange(len(dataset)), predictions]
    return float(np.clip(np.quantile(confidence[correct], 0.01) if np.any(correct) else 0.5, 0.5, 0.95))


def _classification_metrics(
    torch: Any,
    network: Any,
    dataset: NetworkA1DirectDataset,
    feature_mean: np.ndarray,
    feature_scale: np.ndarray,
    history_mean: np.ndarray,
    history_scale: np.ndarray,
    *,
    history: np.ndarray | None = None,
    history_mask: np.ndarray | None = None,
) -> dict[str, Any]:
    probabilities, predicted = _topology_predictions(
        torch, network, dataset, feature_mean, feature_scale, history_mean, history_scale, 1024,
        history=history,
        history_mask=history_mask,
    )
    confusion = np.zeros((3, 3), dtype=int)
    for expected, actual in zip(dataset.patch_count, predicted, strict=True):
        confusion[int(expected), int(actual)] += 1
    support = np.sum(confusion, axis=1)
    recall = np.divide(np.diag(confusion), np.maximum(support, 1))
    precision = np.divide(np.diag(confusion), np.maximum(np.sum(confusion, axis=0), 1))
    f1 = np.divide(2 * precision * recall, np.maximum(precision + recall, np.finfo(float).eps))
    return {
        "accuracy": float(np.mean(predicted == dataset.patch_count)),
        "macro_f1": float(np.mean(f1)),
        "macro_f1_supported_classes": float(np.mean(f1[support > 0])),
        "class_support": support.tolist(),
        "per_class_recall": recall.tolist(),
        "confusion_matrix": confusion.tolist(),
        "mean_topology_confidence": float(np.mean(np.max(probabilities, axis=1))),
    }


def _topology_predictions(
    torch: Any,
    network: Any,
    dataset: NetworkA1DirectDataset,
    feature_mean: np.ndarray,
    feature_scale: np.ndarray,
    history_mean: np.ndarray,
    history_scale: np.ndarray,
    batch_size: int,
    *,
    history: np.ndarray | None = None,
    history_mask: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    network.eval()
    probabilities = []
    history_values = dataset.history if history is None else np.asarray(history, dtype=float)
    mask_values = dataset.history_mask if history_mask is None else np.asarray(history_mask, dtype=bool)
    with torch.no_grad():
        for start in range(0, len(dataset), batch_size):
            stop = min(start + batch_size, len(dataset))
            logits, _ = network(
                torch.as_tensor((dataset.features[start:stop] - feature_mean) / feature_scale, dtype=torch.float32),
                torch.as_tensor((history_values[start:stop] - history_mean) / history_scale, dtype=torch.float32),
                torch.as_tensor(mask_values[start:stop], dtype=torch.float32),
            )
            probabilities.append(torch.softmax(logits, dim=1).cpu().numpy())
    values = np.concatenate(probabilities)
    return values, np.argmax(values, axis=1)


def _teacher_probability(
    epoch: int,
    epochs: int,
    *,
    final: float = 0.1,
    start_fraction: float = 0.4,
) -> float:
    boundary = int(np.floor(start_fraction * epochs))
    if epoch < boundary:
        return 1.0
    progress = (epoch - boundary) / max(epochs - boundary - 1, 1)
    return 1.0 - (1.0 - final) * float(np.clip(progress, 0.0, 1.0))


def _training_rollout_horizon(
    epoch: int,
    epochs: int,
    *,
    maximum: int,
    start_fraction: float = 0.4,
) -> int:
    boundary = int(np.floor(start_fraction * epochs))
    progress = (epoch - boundary) / max(epochs - boundary - 1, 1)
    return 1 + int(np.floor(np.clip(progress, 0.0, 1.0) * max(maximum - 1, 0)))


def _mean_scale(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    array = np.asarray(values, dtype=float)
    mean = np.mean(array, axis=0)
    scale = np.std(array, axis=0)
    return mean, np.where(scale > 1.0e-12, scale, 1.0)


def _calibrated_ood_scale(
    standardized: np.ndarray,
    *,
    threshold: float,
    quantile: float = 0.9995,
) -> np.ndarray:
    values = np.asarray(standardized, dtype=float)
    if values.ndim != 2 or not values.shape[0]:
        return np.ones((values.shape[-1],), dtype=float)
    envelope = np.quantile(np.abs(values), quantile, axis=0)
    # The runtime still compares against the unchanged threshold.  This scale
    # calibrates each field to the empirical training support instead of
    # treating every marginal distribution as exactly Gaussian.
    return np.maximum(envelope / float(threshold), 1.0)


def _require_torch() -> tuple[Any, Any, Any]:
    try:
        import torch
        from torch import nn
        from torch.nn import functional
    except ImportError as exc:
        raise RuntimeError(
            "training WRCP-Net A1 Direct requires the optional 'train' dependency"
        ) from exc
    return torch, nn, functional
