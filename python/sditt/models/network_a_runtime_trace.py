from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from sditt.contact.geometry import MultiPointContactGeometry
from sditt.training_data.network_a import geometry_to_network_a_labels


class NetworkARuntimeTraceWriter:
    """Write compact per-iteration A2G diagnostics in restart-safe NPZ shards."""

    def __init__(
        self,
        output_dir: str | Path,
        *,
        model_path: str | Path,
        metadata: Mapping[str, Any] | None = None,
        shard_size: int = 512,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        manifest_path = self.output_dir / "manifest.json"
        if manifest_path.exists():
            existing = json.loads(manifest_path.read_text(encoding="utf-8"))
            if existing.get("schema") != "network-a-runtime-iteration-trace-v2":
                raise ValueError(
                    "network-A trace schema changed; use a new trace directory for v2 diagnostics"
                )
        self.model_path = Path(model_path).resolve()
        self.metadata = dict(metadata or {})
        self.shard_size = int(shard_size)
        self._records: list[dict[str, Any]] = []
        shards = tuple(sorted(self.output_dir.glob("trace_*.npz")))
        self._shard_index = len(shards)
        self._sample_count = sum(_shard_sample_count(path) for path in shards)
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
        self._records.append(record)
        if len(self._records) >= self.shard_size:
            self.flush()

    def mark_accepted(
        self,
        *,
        stage: str,
        step_index: int,
        iteration: int,
        time_s: float,
        dt_s: float,
    ) -> None:
        payload = {
            "stage": str(stage),
            "step_index": int(step_index),
            "iteration": int(iteration),
            "time_s": float(time_s),
            "dt_s": float(dt_s),
        }
        with self._accepted_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, ensure_ascii=False) + "\n")
        self._accepted_count += 1
        self._write_manifest(complete=False)

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
        )
        self._sample_count += len(records)
        self._shard_index += 1
        self._records = []
        self._write_manifest(complete=False)

    def close(self) -> None:
        self.flush()
        self._write_manifest(complete=True)

    def _write_manifest(self, *, complete: bool) -> None:
        payload = {
            "schema": "network-a-runtime-iteration-trace-v2",
            "role": "diagnostic candidate pool; not a teacher-labelled training split",
            "complete": bool(complete),
            "model_path": str(self.model_path),
            "model_sha256": _sha256_file(self.model_path),
            "shard_size": self.shard_size,
            "completed_shards": self._shard_index,
            "completed_samples": self._sample_count,
            "buffered_samples": len(self._records),
            "accepted_step_keys": self._accepted_count,
            "metadata": self.metadata,
            "usage_note": (
                "Join accepted_steps.jsonl by stage/step_index/iteration/time_s/dt_s. "
                "Rerun the traditional teacher on selected feature rows before adding them to training."
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
