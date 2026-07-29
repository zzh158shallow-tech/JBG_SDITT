from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np

from sditt.contact.geometry import MultiPointContactGeometry
from sditt.training_data.network_a import geometry_to_network_a_labels


class NetworkARuntimeTraceWriter:
    """Write restart-safe A2G diagnostics in full or selective NPZ shards.

    Selective mode buffers one coupled step in memory.  It always retains the
    accepted final iteration and retains every candidate iteration only when
    the step was retried, low-confidence, multi-patch, or selected by the
    mileage sampler.  Full mode is intended for explicit solver debugging.
    """

    def __init__(
        self,
        output_dir: str | Path,
        *,
        model_path: str | Path,
        metadata: Mapping[str, Any] | None = None,
        shard_size: int = 512,
        mode: str = "selective",
        low_confidence_threshold: float = 0.95,
        sample_interval_m: float | None = 1.0,
    ) -> None:
        if mode not in {"selective", "full"}:
            raise ValueError("network-A trace mode must be 'selective' or 'full'")
        if not 0.0 <= float(low_confidence_threshold) <= 1.0:
            raise ValueError("network-A trace low-confidence threshold must be in [0, 1]")
        if sample_interval_m is not None and float(sample_interval_m) <= 0.0:
            raise ValueError("network-A trace sample interval must be positive or None")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.mode = str(mode)
        self.low_confidence_threshold = float(low_confidence_threshold)
        self.sample_interval_m = (
            None if sample_interval_m is None else float(sample_interval_m)
        )
        manifest_path = self.output_dir / "manifest.json"
        existing: dict[str, Any] = {}
        if manifest_path.exists():
            existing = json.loads(manifest_path.read_text(encoding="utf-8"))
            if existing.get("schema") != "network-a-runtime-iteration-trace-v3":
                raise ValueError(
                    "network-A trace schema changed; use a new trace directory for v3 diagnostics"
                )
            expected = {
                "mode": self.mode,
                "low_confidence_threshold": self.low_confidence_threshold,
                "sample_interval_m": self.sample_interval_m,
            }
            if existing.get("selection") != expected:
                raise ValueError(
                    "network-A trace selection settings changed; use a new trace directory"
                )
        self.model_path = Path(model_path).resolve()
        self.model_sha256 = _sha256_file(self.model_path)
        self.metadata = dict(metadata or {})
        self.shard_size = int(shard_size)
        self._records: list[dict[str, Any]] = []
        self._pending_records: list[dict[str, Any]] = []
        shards = tuple(sorted(self.output_dir.glob("trace_*.npz")))
        self._shard_index = len(shards)
        self._sample_count = sum(_shard_sample_count(path) for path in shards)
        self._observed_count = int(existing.get("observed_samples", self._sample_count))
        self._reason_counts = {
            str(key): int(value)
            for key, value in dict(existing.get("retained_sample_reasons", {})).items()
        }
        last_bucket = existing.get("last_mileage_sample_bucket")
        self._last_sample_bucket = None if last_bucket is None else int(last_bucket)
        self._accepted_path = self.output_dir / "accepted_steps.jsonl"
        self._accepted_count = (
            0
            if not self._accepted_path.exists()
            else len(self._accepted_path.read_text(encoding="utf-8").splitlines())
        )
        self._write_manifest(complete=False)

    def record(
        self,
        *,
        context: Mapping[str, Any],
        features: np.ndarray,
        class_probability: np.ndarray,
        predicted_patch_count: int,
        geometry: MultiPointContactGeometry,
        gap_field_m: np.ndarray,
        continuity_diagnostics: Any | None = None,
    ) -> None:
        reconstructed_count, patch_mask, labels = geometry_to_network_a_labels(geometry)
        record = {
            "stage": str(context["stage"]),
            "step_index": int(context["step_index"]),
            "iteration": int(context["iteration"]),
            "time_s": float(context["time_s"]),
            "dt_s": float(context["dt_s"]),
            "front_mileage_m": float(context["front_mileage_m"]),
            "actual_mileage_m": float(context["actual_mileage_m"]),
            "wheelset": str(context["wheelset"]),
            "side": str(context["side"]),
            "features": np.asarray(features, dtype=np.float32).copy(),
            "class_probability": np.asarray(class_probability, dtype=np.float32).copy(),
            "predicted_patch_count": int(predicted_patch_count),
            "reconstructed_patch_count": int(reconstructed_count),
            "patch_mask": np.asarray(patch_mask, dtype=bool).copy(),
            "labels": np.asarray(labels, dtype=np.float32).copy(),
            "gap_min_m": float(np.min(gap_field_m)),
            "gap_max_m": float(np.max(gap_field_m)),
            "base_penetration_m": _diagnostic_array(
                continuity_diagnostics, "base_penetration_m", dtype=np.float32
            ),
            "bounded_residual_m": _diagnostic_array(
                continuity_diagnostics, "bounded_residual_m", dtype=np.float32
            ),
            "used_residual_m": _diagnostic_array(
                continuity_diagnostics, "used_residual_m", dtype=np.float32
            ),
            "network_penetration_m": _diagnostic_array(
                continuity_diagnostics, "network_penetration_m", dtype=np.float32
            ),
            "used_penetration_m": _diagnostic_array(
                continuity_diagnostics, "used_penetration_m", dtype=np.float32
            ),
            "residual_limited": _diagnostic_array(
                continuity_diagnostics, "residual_limited", dtype=bool
            ),
            "penetration_limited": _diagnostic_array(
                continuity_diagnostics, "penetration_limited", dtype=bool
            ),
            "branch_held": _diagnostic_array(
                continuity_diagnostics, "branch_held", dtype=bool, size=2
            ),
        }
        self._observed_count += 1
        if self.mode == "full":
            record["selection_reason"] = "full_iteration"
            self._append_retained((record,))
            return
        if self._pending_records:
            pending = self._pending_records[0]
            if (pending["stage"], pending["step_index"]) != (
                record["stage"],
                record["step_index"],
            ):
                raise RuntimeError(
                    "network-A selective trace reached a new step before the prior step was accepted"
                )
        self._pending_records.append(record)

    def mark_accepted(
        self,
        *,
        stage: str,
        step_index: int,
        iteration: int,
        time_s: float,
        dt_s: float,
        retry_count: int = 0,
        front_mileage_m: float | None = None,
    ) -> None:
        accepted_key = _iteration_key(stage, step_index, iteration, time_s, dt_s)
        retained_reasons: tuple[str, ...] = ()
        if self.mode == "selective":
            retained_reasons = self._select_pending_records(
                accepted_key=accepted_key,
                retry_count=int(retry_count),
                front_mileage_m=front_mileage_m,
            )
        payload = {
            "stage": str(stage),
            "step_index": int(step_index),
            "iteration": int(iteration),
            "time_s": float(time_s),
            "dt_s": float(dt_s),
            "retry_count": int(retry_count),
            "front_mileage_m": (
                None if front_mileage_m is None else float(front_mileage_m)
            ),
            "retained_reasons": list(retained_reasons),
        }
        with self._accepted_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False) + "\n")
        self._accepted_count += 1
        self._write_manifest(complete=False)

    def _select_pending_records(
        self,
        *,
        accepted_key: tuple[str, int, int, float, float],
        retry_count: int,
        front_mileage_m: float | None,
    ) -> tuple[str, ...]:
        pending = self._pending_records
        if not pending:
            raise RuntimeError("network-A selective trace has no records for the accepted step")
        accepted_records = [
            record for record in pending if _record_iteration_key(record) == accepted_key
        ]
        if not accepted_records:
            raise RuntimeError(
                "network-A selective trace could not match the accepted final iteration"
            )
        attempt_keys = {
            (round(float(record["time_s"]), 15), round(float(record["dt_s"]), 15))
            for record in pending
        }
        reasons: list[str] = []
        if retry_count > 0 or len(attempt_keys) > 1:
            reasons.append("retry")
        if any(
            float(np.max(record["class_probability"]))
            < self.low_confidence_threshold
            for record in pending
        ):
            reasons.append("low_confidence")
        if any(
            int(record["predicted_patch_count"]) > 1
            or int(record["reconstructed_patch_count"]) > 1
            for record in pending
        ):
            reasons.append("multi_patch")
        accepted_mileage = (
            float(accepted_records[0]["front_mileage_m"])
            if front_mileage_m is None
            else float(front_mileage_m)
        )
        if self.sample_interval_m is not None:
            bucket = math.floor(accepted_mileage / self.sample_interval_m + 1.0e-12)
            if bucket != self._last_sample_bucket:
                reasons.append("mileage_sample")
                self._last_sample_bucket = bucket

        exceptional = bool(reasons)
        selected = pending if exceptional else accepted_records
        reason_set = tuple(reasons)
        for record in selected:
            labels = list(reason_set)
            if _record_iteration_key(record) == accepted_key:
                labels.insert(0, "accepted_final")
            record["selection_reason"] = "+".join(labels)
        self._append_retained(selected)
        self._pending_records = []
        return reason_set

    def _append_retained(self, records: Iterable[dict[str, Any]]) -> None:
        for record in records:
            self._records.append(record)
            for reason in str(record["selection_reason"]).split("+"):
                self._reason_counts[reason] = self._reason_counts.get(reason, 0) + 1
        if len(self._records) >= self.shard_size:
            self.flush()

    def flush(self) -> None:
        if not self._records:
            return
        records = self._records
        path = self.output_dir / f"trace_{self._shard_index:06d}.npz"
        np.savez_compressed(
            path,
            stage=np.asarray([record["stage"] for record in records]),
            step_index=np.asarray([record["step_index"] for record in records], dtype=np.int64),
            iteration=np.asarray([record["iteration"] for record in records], dtype=np.int32),
            time_s=np.asarray([record["time_s"] for record in records], dtype=np.float64),
            dt_s=np.asarray([record["dt_s"] for record in records], dtype=np.float64),
            front_mileage_m=np.asarray([record["front_mileage_m"] for record in records], dtype=np.float64),
            actual_mileage_m=np.asarray([record["actual_mileage_m"] for record in records], dtype=np.float64),
            wheelset=np.asarray([record["wheelset"] for record in records]),
            side=np.asarray([record["side"] for record in records]),
            features=np.stack([record["features"] for record in records]),
            class_probability=np.stack([record["class_probability"] for record in records]),
            predicted_patch_count=np.asarray(
                [record["predicted_patch_count"] for record in records], dtype=np.int8
            ),
            reconstructed_patch_count=np.asarray(
                [record["reconstructed_patch_count"] for record in records], dtype=np.int8
            ),
            patch_mask=np.stack([record["patch_mask"] for record in records]),
            labels=np.stack([record["labels"] for record in records]),
            gap_min_m=np.asarray([record["gap_min_m"] for record in records], dtype=np.float32),
            gap_max_m=np.asarray([record["gap_max_m"] for record in records], dtype=np.float32),
            base_penetration_m=np.stack([record["base_penetration_m"] for record in records]),
            bounded_residual_m=np.stack([record["bounded_residual_m"] for record in records]),
            used_residual_m=np.stack([record["used_residual_m"] for record in records]),
            network_penetration_m=np.stack([record["network_penetration_m"] for record in records]),
            used_penetration_m=np.stack([record["used_penetration_m"] for record in records]),
            residual_limited=np.stack([record["residual_limited"] for record in records]),
            penetration_limited=np.stack([record["penetration_limited"] for record in records]),
            branch_held=np.stack([record["branch_held"] for record in records]),
            selection_reason=np.asarray([record["selection_reason"] for record in records]),
        )
        self._sample_count += len(records)
        self._shard_index += 1
        self._records = []
        self._write_manifest(complete=False)

    def close(self) -> None:
        if self.mode == "selective" and self._pending_records:
            self._pending_records = []
        self.flush()
        self._write_manifest(complete=True)

    def _write_manifest(self, *, complete: bool) -> None:
        payload = {
            "schema": "network-a-runtime-iteration-trace-v3",
            "role": (
                "selective accepted-final and exceptional-step diagnostics; not teacher labels"
                if self.mode == "selective"
                else "full diagnostic candidate pool; not teacher labels"
            ),
            "complete": bool(complete),
            "model_path": str(self.model_path),
            "model_sha256": self.model_sha256,
            "shard_size": self.shard_size,
            "completed_shards": self._shard_index,
            "completed_samples": self._sample_count,
            "buffered_samples": len(self._records),
            "pending_candidate_samples": len(self._pending_records),
            "observed_samples": self._observed_count,
            "accepted_step_keys": self._accepted_count,
            "selection": {
                "mode": self.mode,
                "low_confidence_threshold": self.low_confidence_threshold,
                "sample_interval_m": self.sample_interval_m,
            },
            "retained_sample_reasons": self._reason_counts,
            "last_mileage_sample_bucket": self._last_sample_bucket,
            "metadata": self.metadata,
            "usage_note": (
                "selection_reason identifies retained accepted-final or exceptional diagnostics. "
                "Join accepted_steps.jsonl by stage/step_index/iteration/time_s/dt_s. "
                "Rerun the traditional teacher before adding any selected row to training."
            ),
        }
        (self.output_dir / "manifest.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _diagnostic_array(
    diagnostics: Any | None,
    name: str,
    *,
    dtype: Any,
    size: int = 4,
) -> np.ndarray:
    if diagnostics is None:
        return np.zeros((size,), dtype=dtype)
    values = np.asarray(getattr(diagnostics, name), dtype=dtype).reshape(-1)
    result = np.zeros((size,), dtype=dtype)
    result[: min(size, values.size)] = values[:size]
    return result


def _shard_sample_count(path: Path) -> int:
    with np.load(path, allow_pickle=False) as archive:
        return int(len(archive["stage"]))


def _iteration_key(
    stage: Any,
    step_index: Any,
    iteration: Any,
    time_s: Any,
    dt_s: Any,
) -> tuple[str, int, int, float, float]:
    return (
        str(stage),
        int(step_index),
        int(iteration),
        round(float(time_s), 15),
        round(float(dt_s), 15),
    )


def _record_iteration_key(record: Mapping[str, Any]) -> tuple[str, int, int, float, float]:
    return _iteration_key(
        record["stage"],
        record["step_index"],
        record["iteration"],
        record["time_s"],
        record["dt_s"],
    )
