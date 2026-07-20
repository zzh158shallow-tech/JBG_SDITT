from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from sditt.contact.geometry import ContactPatch, MultiPointContactGeometry
from sditt.models.wrcp_net_a1 import WRCPNetA1, _Adam, _sha256_file
from sditt.models.wrcp_net_a2g import (
    _canonical_features,
    _geometry_from_predicted_field,
    load_wrcp_net_a2g,
    predict_wrcp_net_a2g,
)
from sditt.training_data.network_a import (
    FEATURE_NAMES,
    geometry_to_network_a_labels,
    build_network_a_teacher_context,
    teacher_geometry_from_features,
)


MODEL_NAME = "WRCP-Net A2R"
MODEL_ID = "wrcp-net-a2r"
RESIDUAL_OUTPUT_NAMES = (
    "patch0_corrected_vertical_penetration_m",
    "patch0_peak_vertical_penetration_m",
    "patch1_corrected_vertical_penetration_m",
    "patch1_peak_vertical_penetration_m",
)


@dataclass(frozen=True)
class PenetrationResidualHeads:
    left: WRCPNetA1
    right: WRCPNetA1
    feature_mean: np.ndarray
    feature_std: np.ndarray
    target_scale: np.ndarray
    residual_gain: np.ndarray
    residual_limit_m: np.ndarray = field(
        default_factory=lambda: np.full((2, 4), 5.0e-5, dtype=np.float32)
    )

    def predict(self, features: np.ndarray, base_penetration_m: np.ndarray) -> np.ndarray:
        values = np.asarray(features, dtype=np.float32)
        if values.ndim == 1:
            values = values[None, :]
        inputs = _residual_inputs(values, base_penetration_m)
        normalized = (inputs - self.feature_mean[None, :]) / self.feature_std[None, :]
        output = np.zeros((len(values), len(RESIDUAL_OUTPUT_NAMES)), dtype=np.float32)
        left = values[:, 0] < 0.5
        if np.any(left):
            output[left] = np.asarray(self.left.forward(normalized[left]), dtype=np.float32)
        if np.any(~left):
            output[~left] = np.asarray(self.right.forward(normalized[~left]), dtype=np.float32)
        raw = output * self.target_scale[None, :]
        limits = np.where(
            left[:, None],
            self.residual_limit_m[0][None, :],
            self.residual_limit_m[1][None, :],
        ).astype(np.float32)
        bounded = limits * np.tanh(raw / np.maximum(limits, 1.0e-9))
        gain = np.where(left, self.residual_gain[0], self.residual_gain[1]).astype(np.float32)
        return bounded * gain[:, None]


def build_runtime_teacher_dataset(
    trace_dir: str | Path,
    output_dir: str | Path,
    *,
    repo_root: str | Path | None = None,
    split_counts: Sequence[int] = (14_000, 3_000, 3_000),
    seed: int = 20260721,
    base_model_path: str | Path = Path("outputs/wrcp_net_a2g/model.npz"),
) -> Path:
    trace_root = Path(trace_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    shards = sorted(trace_root.glob("trace_*.npz"))
    if not shards:
        raise FileNotFoundError(f"no runtime trace shards found in {trace_root}")
    keys = (
        "features",
        "labels",
        "patch_mask",
        "predicted_patch_count",
        "stage",
        "step_index",
        "iteration",
        "time_s",
        "dt_s",
        "front_mileage_m",
        "actual_mileage_m",
        "wheelset",
        "side",
    )
    loaded: dict[str, list[np.ndarray]] = {key: [] for key in keys}
    for path in shards:
        with np.load(path, allow_pickle=False) as archive:
            for key in keys:
                loaded[key].append(archive[key].copy())
    arrays = {key: np.concatenate(values, axis=0) for key, values in loaded.items()}
    accepted = _accepted_keys(trace_root / "accepted_steps.jsonl")
    accepted_mask = np.array(
        [
            _iteration_key(
                arrays["stage"][index],
                arrays["step_index"][index],
                arrays["iteration"][index],
                arrays["time_s"][index],
                arrays["dt_s"][index],
            )
            in accepted
            for index in range(len(arrays["features"]))
        ],
        dtype=bool,
    )
    nominal_dt = float(np.max(arrays["dt_s"]))
    priority = (
        1.0
        + 0.35 * np.maximum(arrays["iteration"].astype(float) - 1.0, 0.0)
        + 1.5 * np.sqrt(np.maximum(nominal_dt / arrays["dt_s"] - 1.0, 0.0))
        + 0.75 * (arrays["side"] == "L")
        + 0.75 * accepted_mask
    )
    steps = np.unique(arrays["step_index"])
    cut1 = steps[max(int(np.floor(0.70 * len(steps))) - 1, 0)]
    cut2 = steps[max(int(np.floor(0.85 * len(steps))) - 1, 0)]
    groups = {
        "train": np.flatnonzero(arrays["step_index"] <= cut1),
        "validation": np.flatnonzero(
            (arrays["step_index"] > cut1) & (arrays["step_index"] <= cut2)
        ),
        "test": np.flatnonzero(arrays["step_index"] > cut2),
    }
    rng = np.random.default_rng(seed)
    selected_by_split: dict[str, np.ndarray] = {}
    for (split, candidates), count in zip(groups.items(), split_counts, strict=True):
        if candidates.size == 0:
            raise RuntimeError(f"runtime trace split {split} is empty")
        take = min(int(count), candidates.size)
        probabilities = priority[candidates] / np.sum(priority[candidates])
        selected_by_split[split] = np.sort(
            rng.choice(candidates, size=take, replace=False, p=probabilities)
        )

    context = build_network_a_teacher_context(repo_root)
    base_model, base_normalization = load_wrcp_net_a2g(base_model_path)
    stats: dict[str, Any] = {}
    for split, selected in selected_by_split.items():
        count = len(selected)
        teacher_count = np.zeros((count,), dtype=np.int8)
        teacher_mask = np.zeros((count, 2), dtype=bool)
        teacher_labels = np.zeros((count, 2, 19), dtype=np.float32)
        base_count = np.zeros((count,), dtype=np.int8)
        base_mask = np.zeros((count, 2), dtype=bool)
        base_labels_all = np.zeros((count, 2, 19), dtype=np.float32)
        teacher_mismatch = np.zeros((count,), dtype=bool)
        base_prediction = predict_wrcp_net_a2g(
            base_model,
            arrays["features"][selected].astype(np.float32),
            base_normalization,
        )
        for local_index, source_index in enumerate(selected):
            geometry = teacher_geometry_from_features(
                context,
                arrays["features"][source_index].astype(float),
            )
            patch_count, patch_mask, labels = geometry_to_network_a_labels(geometry)
            teacher_count[local_index] = patch_count
            teacher_mask[local_index] = patch_mask
            teacher_labels[local_index] = labels
            base_geometry = _geometry_from_predicted_field(
                context,
                arrays["features"][source_index].astype(float),
                base_normalization["canonical_y_m"],
                base_prediction.gap_field_m[local_index],
                int(base_prediction.patch_count[local_index]),
                profile_refinement=False,
                candidate_profile_refinement=True,
            )
            b_count, b_mask, b_labels = geometry_to_network_a_labels(base_geometry)
            base_count[local_index] = b_count
            base_mask[local_index] = b_mask
            base_labels_all[local_index] = b_labels
            teacher_mismatch[local_index] = patch_count != b_count
        valid = ~teacher_mismatch
        selected = selected[valid]
        teacher_count = teacher_count[valid]
        teacher_mask = teacher_mask[valid]
        teacher_labels = teacher_labels[valid]
        base_labels = base_labels_all[valid]
        residual = np.zeros((len(selected), 4), dtype=np.float32)
        residual[:, 0] = teacher_labels[:, 0, 8] - base_labels[:, 0, 8]
        residual[:, 1] = teacher_labels[:, 0, 16] - base_labels[:, 0, 16]
        residual[:, 2] = teacher_labels[:, 1, 8] - base_labels[:, 1, 8]
        residual[:, 3] = teacher_labels[:, 1, 16] - base_labels[:, 1, 16]
        residual_mask = np.repeat(teacher_mask, 2, axis=1)
        penetration = np.column_stack(
            (
                teacher_labels[:, 0, 8],
                teacher_labels[:, 0, 16],
                teacher_labels[:, 1, 8],
                teacher_labels[:, 1, 16],
            )
        )
        positive = penetration[residual_mask]
        reference = max(float(np.median(positive)) if positive.size else 1.0e-5, 1.0e-6)
        force_sensitivity = np.clip(
            np.sqrt(np.maximum(penetration, 1.0e-8) / reference),
            0.5,
            4.0,
        ).astype(np.float32)
        sample_weight = priority[selected].astype(np.float32)
        payload: dict[str, np.ndarray] = {
            "features": arrays["features"][selected].astype(np.float32),
            "teacher_patch_count": teacher_count,
            "teacher_patch_mask": teacher_mask,
            "teacher_labels": teacher_labels,
            "base_labels": base_labels,
            "penetration_residual_m": residual,
            "residual_mask": residual_mask,
            "force_sensitivity": force_sensitivity,
            "sample_weight": sample_weight,
            "accepted": accepted_mask[selected],
            "stage": arrays["stage"][selected],
            "step_index": arrays["step_index"][selected],
            "iteration": arrays["iteration"][selected],
            "time_s": arrays["time_s"][selected],
            "dt_s": arrays["dt_s"][selected],
            "front_mileage_m": arrays["front_mileage_m"][selected],
            "actual_mileage_m": arrays["actual_mileage_m"][selected],
            "wheelset": arrays["wheelset"][selected],
            "side": arrays["side"][selected],
        }
        if split == "train":
            payload.update(
                _build_perturbation_pairs(
                    context=context,
                    base_model=base_model,
                    base_normalization=base_normalization,
                    features=payload["features"],
                    seed=seed + 101,
                )
            )
        np.savez_compressed(destination / f"{split}.npz", **payload)
        stats[split] = {
            "selected_before_teacher_filter": count,
            "saved": int(len(selected)),
            "teacher_count_mismatch_rejected": int(np.count_nonzero(teacher_mismatch)),
            "step_min": int(np.min(arrays["step_index"][selected])),
            "step_max": int(np.max(arrays["step_index"][selected])),
            "left": int(np.count_nonzero(arrays["side"][selected] == "L")),
            "accepted": int(np.count_nonzero(accepted_mask[selected])),
            "reduced_dt": int(np.count_nonzero(arrays["dt_s"][selected] < nominal_dt)),
            "iteration_ge_8": int(np.count_nonzero(arrays["iteration"][selected] >= 8)),
            "perturbation_pairs": int(
                np.count_nonzero(payload.get("perturbation_valid", np.zeros(0, dtype=bool)))
            ),
        }
        print(f"teacher_relabel split={split} saved={len(selected)}", flush=True)
    manifest = {
        "schema": "network-a-runtime-teacher-residual-v2",
        "source_trace": str(trace_root.resolve()),
        "source_trace_manifest_sha256": _sha256_file(trace_root / "manifest.json"),
        "teacher": "Python SDITT traditional multi_point_contact_geometry",
        "base_geometry": "A2G candidate-region fixed-profile refinement",
        "base_model_sha256": _sha256_file(Path(base_model_path)),
        "perturbations": {
            "train_pairs_per_state": 1,
            "delta_y_delta_z_m": "signed log-uniform 1e-8 to 2e-6",
            "roll_yaw_rad": "signed log-uniform 1e-8 to 2e-6",
        },
        "selection": {
            "seed": seed,
            "requested_split_counts": list(split_counts),
            "grouping": "contiguous step_index ranges 70/15/15",
            "priority": "iteration + reduced_dt + left_side + accepted_step",
            "nominal_dt_s": nominal_dt,
        },
        "splits": stats,
        "feature_names": list(FEATURE_NAMES),
        "residual_output_names": list(RESIDUAL_OUTPUT_NAMES),
    }
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return manifest_path


def train_penetration_residual_heads(
    dataset_dir: str | Path,
    base_model_path: str | Path,
    output_dir: str | Path,
    *,
    epochs: int = 400,
    batch_size: int = 256,
    learning_rate: float = 5.0e-4,
    hidden_sizes: Sequence[int] = (64, 64),
    patience: int = 60,
    seed: int = 20260722,
    residual_gain: Sequence[float] = (0.5, 0.5),
    continuity_alpha: float = 0.30,
    rate_quantile: float = 0.995,
    smooth_weight: float = 0.25,
    rate_reference_dataset: str | Path | None = Path(
        "outputs/network_a_dataset_v1/all_samples.npz"
    ),
) -> Path:
    dataset_root = Path(dataset_dir)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    data = {split: _load_residual_split(dataset_root / f"{split}.npz") for split in ("train", "validation", "test")}
    residual_inputs_train = _residual_inputs(
        data["train"]["features"],
        _base_penetration(data["train"]["base_labels"]),
    )
    feature_mean = np.mean(residual_inputs_train, axis=0).astype(np.float32)
    feature_std = np.maximum(np.std(residual_inputs_train, axis=0), 1.0e-6).astype(np.float32)
    target_values = data["train"]["residual"]
    target_mask = data["train"]["mask"]
    target_scale = np.ones((4,), dtype=np.float32)
    for column in range(4):
        values = target_values[target_mask[:, column], column]
        target_scale[column] = max(float(np.std(values)) if values.size else 0.0, 1.0e-6)
    models = {
        "L": WRCPNetA1(input_size=9, hidden_sizes=hidden_sizes, output_size=4, seed=seed),
        "R": WRCPNetA1(input_size=9, hidden_sizes=hidden_sizes, output_size=4, seed=seed + 1),
    }
    histories: dict[str, list[dict[str, float]]] = {}
    best_parameters: dict[str, tuple[list[np.ndarray], list[np.ndarray]]] = {}
    best_epochs: dict[str, int] = {}
    for side_index, side in enumerate(("L", "R")):
        model = models[side]
        optimizer = _Adam(model, learning_rate=learning_rate)
        rng = np.random.default_rng(seed + side_index)
        best_loss = float("inf")
        best_epoch = 0
        history: list[dict[str, float]] = []
        train_side = data["train"]["features"][:, 0] == float(side_index)
        validation_side = data["validation"]["features"][:, 0] == float(side_index)
        x_train_all = _residual_inputs(
            data["train"]["features"], _base_penetration(data["train"]["base_labels"])
        )
        x_train = (x_train_all[train_side] - feature_mean) / feature_std
        y_train = data["train"]["residual"][train_side] / target_scale
        m_train = data["train"]["mask"][train_side]
        w_train = data["train"]["weight"][train_side]
        f_train = data["train"]["force_sensitivity"][train_side]
        has_perturbations = "perturbed_features" in data["train"]
        if has_perturbations:
            x_perturbed_all = _residual_inputs(
                data["train"]["perturbed_features"],
                _base_penetration(data["train"]["perturbed_base_labels"]),
            )
            x_perturbed = (x_perturbed_all[train_side] - feature_mean) / feature_std
            y_perturbed = data["train"]["perturbed_residual_m"][train_side] / target_scale
            m_perturbed = data["train"]["perturbed_residual_mask"][train_side]
        x_val_all = _residual_inputs(
            data["validation"]["features"],
            _base_penetration(data["validation"]["base_labels"]),
        )
        x_val = (x_val_all[validation_side] - feature_mean) / feature_std
        y_val = data["validation"]["residual"][validation_side] / target_scale
        m_val = data["validation"]["mask"][validation_side]
        w_val = data["validation"]["weight"][validation_side]
        f_val = data["validation"]["force_sensitivity"][validation_side]
        for epoch in range(1, epochs + 1):
            order = rng.permutation(len(x_train))
            for start in range(0, len(order), batch_size):
                indexes = order[start : start + batch_size]
                if has_perturbations:
                    supervised_x = np.concatenate(
                        (x_train[indexes], x_perturbed[indexes]), axis=0
                    )
                    supervised_y = np.concatenate(
                        (y_train[indexes], y_perturbed[indexes]), axis=0
                    )
                    supervised_m = np.concatenate(
                        (m_train[indexes], m_perturbed[indexes]), axis=0
                    )
                    supervised_w = np.concatenate((w_train[indexes], w_train[indexes]))
                    supervised_f = np.concatenate((f_train[indexes], f_train[indexes]), axis=0)
                else:
                    supervised_x = x_train[indexes]
                    supervised_y = y_train[indexes]
                    supervised_m = m_train[indexes]
                    supervised_w = w_train[indexes]
                    supervised_f = f_train[indexes]
                output, cache = model.forward(supervised_x, return_cache=True)
                _, gradient = _weighted_huber(
                    np.asarray(output),
                    supervised_y,
                    supervised_m,
                    supervised_w,
                    supervised_f,
                )
                gradients = model.backward(cache, gradient, weight_decay=1.0e-5)
                if has_perturbations and smooth_weight > 0.0:
                    original_output, original_cache = model.forward(
                        x_train[indexes], return_cache=True
                    )
                    perturbed_output, perturbed_cache = model.forward(
                        x_perturbed[indexes], return_cache=True
                    )
                    smooth_mask = m_train[indexes] & m_perturbed[indexes]
                    _, smooth_gradient = _weighted_huber(
                        np.asarray(perturbed_output) - np.asarray(original_output),
                        y_perturbed[indexes] - y_train[indexes],
                        smooth_mask,
                        w_train[indexes],
                        f_train[indexes],
                    )
                    original_gradients = model.backward(
                        original_cache,
                        -float(smooth_weight) * smooth_gradient,
                        weight_decay=0.0,
                    )
                    perturbed_gradients = model.backward(
                        perturbed_cache,
                        float(smooth_weight) * smooth_gradient,
                        weight_decay=0.0,
                    )
                    gradients = _sum_model_gradients(
                        gradients, original_gradients, perturbed_gradients
                    )
                optimizer.step(model, *gradients)
            train_loss = _residual_loss(model, x_train, y_train, m_train, w_train, f_train)
            validation_loss = _residual_loss(model, x_val, y_val, m_val, w_val, f_val)
            history.append({"epoch": float(epoch), "train_loss": train_loss, "validation_loss": validation_loss})
            if validation_loss < best_loss:
                best_loss = validation_loss
                best_epoch = epoch
                best_parameters[side] = model.copy_parameters()
            elif epoch - best_epoch >= patience:
                break
        model.restore_parameters(best_parameters[side])
        best_epochs[side] = best_epoch
        histories[side] = history
        print(f"residual_head side={side} best_epoch={best_epoch} validation={best_loss:.6g}", flush=True)
    gain = np.asarray(residual_gain, dtype=np.float32)
    if gain.shape != (2,) or np.any(gain < 0.0) or np.any(gain > 1.0):
        raise ValueError("residual_gain must contain left/right values in [0, 1]")
    residual_limit = np.zeros((2, 4), dtype=np.float32)
    for side_index in range(2):
        selection = data["train"]["features"][:, 0] == float(side_index)
        for column in range(4):
            valid = selection & data["train"]["mask"][:, column]
            values = np.abs(data["train"]["residual"][valid, column])
            residual_limit[side_index, column] = max(
                float(np.quantile(values, 0.995)) if values.size else 0.0,
                5.0e-6,
            )
    heads = PenetrationResidualHeads(
        models["L"],
        models["R"],
        feature_mean,
        feature_std,
        target_scale,
        gain,
        residual_limit,
    )
    if rate_reference_dataset is not None and Path(rate_reference_dataset).is_file():
        penetration_rate, contact_location_rate = _coupled_teacher_rate_reference(
            Path(rate_reference_dataset), quantile=rate_quantile
        )
        rate_source = str(Path(rate_reference_dataset).resolve())
    else:
        penetration_rate = _adjacent_teacher_rates(data["train"], quantile=rate_quantile)
        contact_location_rate = np.array([6.5, 5.5], dtype=np.float32)
        rate_source = "runtime teacher residual train split"
    residual_rate = _adjacent_residual_rates(data["train"], quantile=rate_quantile)
    metrics = _evaluate_heads(heads, data)
    model_path = destination / "model.npz"
    _save_augmented_model(
        Path(base_model_path),
        model_path,
        heads,
        continuity_alpha=continuity_alpha,
        penetration_rate_m_per_s=penetration_rate,
        residual_rate_m_per_s=residual_rate,
        contact_location_rate_m_per_s=contact_location_rate,
    )
    (destination / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    manifest = {
        "model_name": MODEL_NAME,
        "model_id": MODEL_ID,
        "base_model": str(Path(base_model_path).resolve()),
        "base_model_sha256": _sha256_file(Path(base_model_path)),
        "dataset_manifest_sha256": _sha256_file(dataset_root / "manifest.json"),
        "hidden_sizes": list(hidden_sizes),
        "best_epochs": best_epochs,
        "target_scale_m": target_scale.tolist(),
        "residual_gain": {"L": float(gain[0]), "R": float(gain[1])},
        "residual_limit_m": residual_limit.tolist(),
        "residual_gain_reason": "closed-loop damping calibrated on an unseen irregularity short run",
        "continuity_alpha": float(continuity_alpha),
        "rate_quantile": float(rate_quantile),
        "penetration_rate_m_per_s": penetration_rate.tolist(),
        "contact_location_rate_m_per_s": contact_location_rate.tolist(),
        "rate_reference_source": rate_source,
        "residual_rate_m_per_s": residual_rate.tolist(),
        "smooth_weight": float(smooth_weight),
        "model_sha256": _sha256_file(model_path),
    }
    (destination / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return model_path


def load_penetration_residual_heads(path: str | Path) -> PenetrationResidualHeads | None:
    with np.load(Path(path), allow_pickle=False) as archive:
        if "a2r_feature_mean" not in archive.files:
            return None
        models = {}
        for side in ("L", "R"):
            sizes = tuple(int(value) for value in archive[f"a2r_{side}_layer_sizes"])
            model = WRCPNetA1(input_size=sizes[0], hidden_sizes=sizes[1:-1], output_size=sizes[-1])
            for index in range(len(model.weights)):
                model.weights[index][...] = archive[f"a2r_{side}_weight_{index}"]
                model.biases[index][...] = archive[f"a2r_{side}_bias_{index}"]
            models[side] = model
        return PenetrationResidualHeads(
            left=models["L"],
            right=models["R"],
            feature_mean=archive["a2r_feature_mean"].copy(),
            feature_std=archive["a2r_feature_std"].copy(),
            target_scale=archive["a2r_target_scale"].copy(),
            residual_gain=(
                archive["a2r_residual_gain"].copy()
                if "a2r_residual_gain" in archive.files
                else np.ones((2,), dtype=np.float32)
            ),
            residual_limit_m=(
                archive["a2r_residual_limit_m"].copy()
                if "a2r_residual_limit_m" in archive.files
                else np.full((2, 4), 5.0e-5, dtype=np.float32)
            ),
        )


def apply_penetration_residuals(
    heads: PenetrationResidualHeads,
    features: np.ndarray,
    geometry: MultiPointContactGeometry,
) -> MultiPointContactGeometry:
    base_penetration = np.zeros((1, 4), dtype=np.float32)
    for index, patch in enumerate(geometry.patches):
        base_penetration[0, 2 * index] = patch.corrected_vertical_penetration
        base_penetration[0, 2 * index + 1] = patch.peak_vertical_penetration
    residual = heads.predict(np.asarray(features, dtype=np.float32), base_penetration)[0]
    roll = float(features[3])
    patches: list[ContactPatch] = []
    for index, patch in enumerate(geometry.patches):
        corrected = max(0.0, patch.corrected_vertical_penetration + float(residual[2 * index]))
        peak = max(0.0, patch.peak_vertical_penetration + float(residual[2 * index + 1]))
        patches.append(
            replace(
                patch,
                corrected_vertical_penetration=corrected,
                corrected_normal_penetration=corrected / max(np.cos(patch.contact_angle + roll), np.finfo(float).eps),
                peak_vertical_penetration=peak,
                peak_normal_penetration=peak / max(np.cos(patch.peak_contact_angle + roll), np.finfo(float).eps),
            )
        )
    return replace(geometry, patches=tuple(patches), has_contact=bool(patches))


def _load_residual_split(path: Path) -> dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        result = {
            "features": archive["features"].copy(),
            "residual": archive["penetration_residual_m"].copy(),
            "mask": archive["residual_mask"].copy(),
            "weight": archive["sample_weight"].copy(),
            "force_sensitivity": archive["force_sensitivity"].copy(),
            "teacher_labels": archive["teacher_labels"].copy(),
            "base_labels": archive["base_labels"].copy(),
            "accepted": archive["accepted"].copy(),
            "stage": archive["stage"].copy(),
            "step_index": archive["step_index"].copy(),
            "time_s": archive["time_s"].copy(),
            "dt_s": archive["dt_s"].copy(),
            "wheelset": archive["wheelset"].copy(),
            "side": archive["side"].copy(),
        }
        for name in (
            "perturbed_features",
            "perturbed_teacher_labels",
            "perturbed_base_labels",
            "perturbed_residual_m",
            "perturbed_residual_mask",
            "perturbation_valid",
        ):
            if name in archive.files:
                result[name] = archive[name].copy()
        return result


def _weighted_huber(
    output: np.ndarray,
    target: np.ndarray,
    mask: np.ndarray,
    sample_weight: np.ndarray,
    force_sensitivity: np.ndarray,
) -> tuple[float, np.ndarray]:
    difference = np.asarray(output, dtype=np.float32) - np.asarray(target, dtype=np.float32)
    weights = mask.astype(np.float32) * sample_weight[:, None] * force_sensitivity
    absolute = np.abs(difference)
    point = np.where(absolute <= 1.0, 0.5 * difference**2, absolute - 0.5)
    derivative = np.where(absolute <= 1.0, difference, np.sign(difference))
    denominator = max(float(np.sum(weights)), 1.0)
    return float(np.sum(weights * point) / denominator), (weights * derivative / denominator).astype(np.float32)


def _residual_loss(model: WRCPNetA1, x: np.ndarray, y: np.ndarray, mask: np.ndarray, weight: np.ndarray, sensitivity: np.ndarray) -> float:
    output = np.asarray(model.forward(x), dtype=np.float32)
    return _weighted_huber(output, y, mask, weight, sensitivity)[0]


def _sum_model_gradients(
    *gradients: tuple[list[np.ndarray], list[np.ndarray]],
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    weights = [
        np.sum(np.stack(values), axis=0).astype(np.float32)
        for values in zip(*(item[0] for item in gradients), strict=True)
    ]
    biases = [
        np.sum(np.stack(values), axis=0).astype(np.float32)
        for values in zip(*(item[1] for item in gradients), strict=True)
    ]
    return weights, biases


def _evaluate_heads(heads: PenetrationResidualHeads, data: Mapping[str, Mapping[str, np.ndarray]]) -> dict[str, Any]:
    metrics: dict[str, Any] = {}
    for split, values in data.items():
        correction = heads.predict(values["features"], _base_penetration(values["base_labels"]))
        mask = values["mask"]
        base_error = np.abs(values["residual"])[mask]
        corrected_error = np.abs(values["residual"] - correction)[mask]
        by_side = {}
        for side_index, side in enumerate(("L", "R")):
            selection = values["features"][:, 0] == float(side_index)
            side_mask = mask[selection]
            side_error = np.abs(values["residual"][selection] - correction[selection])[side_mask]
            by_side[side] = {"mae_m": float(np.mean(side_error)), "p95_m": float(np.quantile(side_error, 0.95))}
        metrics[split] = {
            "samples": int(len(values["features"])),
            "base_mae_m": float(np.mean(base_error)),
            "corrected_mae_m": float(np.mean(corrected_error)),
            "corrected_p95_m": float(np.quantile(corrected_error, 0.95)),
            "by_side": by_side,
        }
    return metrics


def _save_augmented_model(
    base_path: Path,
    output_path: Path,
    heads: PenetrationResidualHeads,
    *,
    continuity_alpha: float = 0.30,
    penetration_rate_m_per_s: np.ndarray | None = None,
    residual_rate_m_per_s: np.ndarray | None = None,
    contact_location_rate_m_per_s: np.ndarray | None = None,
) -> None:
    with np.load(base_path, allow_pickle=False) as archive:
        payload = {name: archive[name].copy() for name in archive.files}
    payload.update(
        {
            "a2r_feature_mean": heads.feature_mean,
            "a2r_feature_std": heads.feature_std,
            "a2r_target_scale": heads.target_scale,
            "a2r_residual_gain": heads.residual_gain,
            "a2r_residual_limit_m": heads.residual_limit_m,
            "a2r_continuity_alpha": np.asarray(float(continuity_alpha)),
            "a2r_penetration_rate_m_per_s": np.asarray(
                [[0.22, 0.22], [0.06, 0.06]]
                if penetration_rate_m_per_s is None
                else penetration_rate_m_per_s,
                dtype=np.float32,
            ),
            "a2r_residual_rate_m_per_s": np.asarray(
                [[0.22, 0.22], [0.06, 0.06]]
                if residual_rate_m_per_s is None
                else residual_rate_m_per_s,
                dtype=np.float32,
            ),
            "a2r_contact_location_rate_m_per_s": np.asarray(
                [6.5, 5.5]
                if contact_location_rate_m_per_s is None
                else contact_location_rate_m_per_s,
                dtype=np.float32,
            ),
            "a2r_output_names": np.asarray(RESIDUAL_OUTPUT_NAMES),
        }
    )
    for side, model in (("L", heads.left), ("R", heads.right)):
        payload[f"a2r_{side}_layer_sizes"] = np.asarray(model.layer_sizes, dtype=np.int64)
        for index, (weight, bias) in enumerate(zip(model.weights, model.biases, strict=True)):
            payload[f"a2r_{side}_weight_{index}"] = weight
            payload[f"a2r_{side}_bias_{index}"] = bias
    np.savez_compressed(output_path, **payload)


def _base_penetration(base_labels: np.ndarray) -> np.ndarray:
    labels = np.asarray(base_labels, dtype=np.float32)
    return np.column_stack((labels[:, 0, 8], labels[:, 0, 16], labels[:, 1, 8], labels[:, 1, 16]))


def _residual_inputs(features: np.ndarray, base_penetration_m: np.ndarray) -> np.ndarray:
    values = np.asarray(features, dtype=np.float32)
    if values.ndim == 1:
        values = values[None, :]
    penetration = np.asarray(base_penetration_m, dtype=np.float32)
    if penetration.ndim == 1:
        penetration = penetration[None, :]
    if penetration.shape != (len(values), 4):
        raise ValueError("base_penetration_m must have shape (n, 4)")
    return np.column_stack((_canonical_features(values)[:, 1:], penetration)).astype(np.float32)


def _build_perturbation_pairs(
    *,
    context: Any,
    base_model: Any,
    base_normalization: Mapping[str, np.ndarray],
    features: np.ndarray,
    seed: int,
) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    perturbed = np.asarray(features, dtype=np.float32).copy()
    count = len(perturbed)
    for column, upper in ((1, 2.0e-6), (2, 2.0e-6), (3, 2.0e-6), (4, 2.0e-6)):
        magnitude = np.exp(
            rng.uniform(np.log(1.0e-8), np.log(upper), size=count)
        )
        perturbed[:, column] += (rng.choice((-1.0, 1.0), size=count) * magnitude).astype(
            np.float32
        )
    prediction = predict_wrcp_net_a2g(base_model, perturbed, base_normalization)
    teacher_labels = np.zeros((count, 2, 19), dtype=np.float32)
    base_labels = np.zeros((count, 2, 19), dtype=np.float32)
    mask = np.zeros((count, 4), dtype=bool)
    valid = np.zeros((count,), dtype=bool)
    for index, values in enumerate(perturbed):
        teacher = teacher_geometry_from_features(context, values.astype(float))
        teacher_count, teacher_mask, labels = geometry_to_network_a_labels(teacher)
        base = _geometry_from_predicted_field(
            context,
            values.astype(float),
            base_normalization["canonical_y_m"],
            prediction.gap_field_m[index],
            int(prediction.patch_count[index]),
            profile_refinement=False,
            candidate_profile_refinement=True,
        )
        base_count, _, base_values = geometry_to_network_a_labels(base)
        valid[index] = teacher_count == base_count
        teacher_labels[index] = labels
        base_labels[index] = base_values
        mask[index] = np.repeat(teacher_mask, 2)
    residual = _base_penetration(teacher_labels) - _base_penetration(base_labels)
    mask &= valid[:, None]
    return {
        "perturbed_features": perturbed,
        "perturbed_teacher_labels": teacher_labels,
        "perturbed_base_labels": base_labels,
        "perturbed_residual_m": residual.astype(np.float32),
        "perturbed_residual_mask": mask,
        "perturbation_valid": valid,
    }


def _adjacent_teacher_rates(data: Mapping[str, np.ndarray], *, quantile: float) -> np.ndarray:
    penetration = _base_penetration(data["teacher_labels"])
    return _adjacent_rates(data, penetration, quantile=quantile)


def _adjacent_residual_rates(data: Mapping[str, np.ndarray], *, quantile: float) -> np.ndarray:
    return _adjacent_rates(data, data["residual"], quantile=quantile)


def _adjacent_rates(
    data: Mapping[str, np.ndarray],
    values: np.ndarray,
    *,
    quantile: float,
) -> np.ndarray:
    result = np.zeros((2, 2), dtype=np.float32)
    fallbacks = np.array([[0.22, 0.22], [0.06, 0.06]], dtype=np.float32)
    for side_index, side in enumerate(("L", "R")):
        by_kind: list[list[float]] = [[], []]
        for wheelset in np.unique(data["wheelset"]):
            indexes = np.flatnonzero(
                data["accepted"]
                & (data["side"] == side)
                & (data["wheelset"] == wheelset)
            )
            indexes = indexes[np.argsort(data["step_index"][indexes])]
            for first, second in zip(indexes[:-1], indexes[1:], strict=True):
                if int(data["step_index"][second]) != int(data["step_index"][first]) + 1:
                    continue
                dt = max(
                    float(data["time_s"][second] - data["time_s"][first]),
                    float(data["dt_s"][second]),
                    np.finfo(float).eps,
                )
                for column in range(4):
                    if data["mask"][first, column] and data["mask"][second, column]:
                        by_kind[column % 2].append(
                            abs(float(values[second, column] - values[first, column])) / dt
                        )
        for kind in range(2):
            result[side_index, kind] = (
                max(float(np.quantile(by_kind[kind], quantile)), 1.0e-3)
                if by_kind[kind]
                else fallbacks[side_index, kind]
            )
    return result


def _coupled_teacher_rate_reference(
    path: Path,
    *,
    quantile: float,
) -> tuple[np.ndarray, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        source = np.char.startswith(archive["meta__source"].astype(str), "coupled")
        labels = archive["labels"]
        patch_mask = archive["patch_mask"]
        groups = archive["meta__group_id"]
        wheelsets = archive["meta__wheelset"]
        sides = archive["meta__side"]
        times = archive["meta__time_s"]
        dts = archive["meta__dt_s"]
        penetration_rate = np.zeros((2, 2), dtype=np.float32)
        location_rate = np.zeros((2,), dtype=np.float32)
        for side_index, side in enumerate(("L", "R")):
            penetration: list[list[float]] = [[], []]
            location: list[float] = []
            for group in np.unique(groups[source]):
                for wheelset in np.unique(wheelsets[source]):
                    indexes = np.flatnonzero(
                        source
                        & (groups == group)
                        & (wheelsets == wheelset)
                        & (sides == side)
                    )
                    indexes = indexes[np.argsort(times[indexes])]
                    for first, second in zip(indexes[:-1], indexes[1:], strict=True):
                        if not (patch_mask[first, 0] and patch_mask[second, 0]):
                            continue
                        dt = max(
                            float(times[second] - times[first]),
                            float(dts[second]),
                            np.finfo(float).eps,
                        )
                        penetration[0].append(
                            abs(float(labels[second, 0, 8] - labels[first, 0, 8])) / dt
                        )
                        penetration[1].append(
                            abs(float(labels[second, 0, 16] - labels[first, 0, 16])) / dt
                        )
                        location.append(
                            abs(float(labels[second, 0, 5] - labels[first, 0, 5])) / dt
                        )
            for kind in range(2):
                penetration_rate[side_index, kind] = max(
                    float(np.quantile(penetration[kind], quantile)), 1.0e-3
                )
            location_rate[side_index] = max(
                float(np.quantile(location, quantile)), 1.0e-3
            )
    return penetration_rate, location_rate


def _accepted_keys(path: Path) -> set[tuple[str, int, int, float, float]]:
    if not path.exists():
        return set()
    result = set()
    for line in path.read_text(encoding="utf-8").splitlines():
        value = json.loads(line)
        result.add(_iteration_key(value["stage"], value["step_index"], value["iteration"], value["time_s"], value["dt_s"]))
    return result


def _iteration_key(stage: Any, step: Any, iteration: Any, time_s: Any, dt_s: Any) -> tuple[str, int, int, float, float]:
    return str(stage), int(step), int(iteration), round(float(time_s), 15), round(float(dt_s), 15)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=f"Build and train {MODEL_NAME}.")
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("build-dataset")
    build.add_argument("--trace", type=Path, required=True)
    build.add_argument("--output", type=Path, default=Path("outputs/network_a_runtime_teacher_v1"))
    build.add_argument("--repo-root", type=Path, default=None)
    build.add_argument("--base-model", type=Path, default=Path("outputs/wrcp_net_a2g/model.npz"))
    train = sub.add_parser("train")
    train.add_argument("--dataset", type=Path, default=Path("outputs/network_a_runtime_teacher_v1"))
    train.add_argument("--base-model", type=Path, default=Path("outputs/wrcp_net_a2g/model.npz"))
    train.add_argument("--output", type=Path, default=Path("outputs/wrcp_net_a2r"))
    train.add_argument("--epochs", type=int, default=400)
    train.add_argument("--left-residual-gain", type=float, default=0.5)
    train.add_argument("--right-residual-gain", type=float, default=0.5)
    train.add_argument("--smooth-weight", type=float, default=0.25)
    train.add_argument(
        "--rate-reference-dataset",
        type=Path,
        default=Path("outputs/network_a_dataset_v1/all_samples.npz"),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.command == "build-dataset":
        path = build_runtime_teacher_dataset(
            args.trace,
            args.output,
            repo_root=args.repo_root,
            base_model_path=args.base_model,
        )
    else:
        path = train_penetration_residual_heads(
            args.dataset,
            args.base_model,
            args.output,
            epochs=args.epochs,
            residual_gain=(args.left_residual_gain, args.right_residual_gain),
            smooth_weight=args.smooth_weight,
            rate_reference_dataset=args.rate_reference_dataset,
        )
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
