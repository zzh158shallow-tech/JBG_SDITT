from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from sditt.contact.geometry import DirectContactGeometry, DirectContactPatch


TRACE_SCHEMA = "network-a1-direct-runtime-trace-v1"


@dataclass
class NetworkA1DirectRuntimeTraceWriter:
    output_dir: Path
    model_path: Path
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __init__(
        self,
        output_dir: str | Path,
        *,
        model_path: str | Path,
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.model_path = Path(model_path).resolve()
        self.metadata = dict(metadata or {})
        self._iteration_path = self.output_dir / "iterations.jsonl"
        self._accepted_path = self.output_dir / "accepted_steps.jsonl"
        self._iteration_count = 0
        self._accepted_count = 0
        self._write_manifest(complete=False)

    def record(
        self,
        *,
        context: Mapping[str, Any],
        features: np.ndarray,
        history: np.ndarray,
        history_mask: np.ndarray,
        topology_probability: np.ndarray,
        query_probability: np.ndarray,
        decoded_targets: np.ndarray,
        geometry: DirectContactGeometry,
        feature_distance: float,
        history_distance: float,
    ) -> None:
        record = {
            "schema": TRACE_SCHEMA,
            "context": _json_value(context),
            "features": np.asarray(features, dtype=float).tolist(),
            "history": np.asarray(history, dtype=float).tolist(),
            "history_mask": np.asarray(history_mask, dtype=bool).tolist(),
            "topology_probability": np.asarray(topology_probability, dtype=float).tolist(),
            "query_probability": np.asarray(query_probability, dtype=float).tolist(),
            "decoded_targets": np.asarray(decoded_targets, dtype=float).tolist(),
            "feature_distance": float(feature_distance),
            "history_distance": float(history_distance),
            "hard_constraints_passed": True,
            "geometry": _geometry_record(geometry),
        }
        _append_jsonl(self._iteration_path, record)
        self._iteration_count += 1

    def mark_accepted(self, **values: Any) -> None:
        _append_jsonl(
            self._accepted_path,
            {"schema": TRACE_SCHEMA, **_json_value(values)},
        )
        self._accepted_count += 1

    def record_failure(
        self,
        *,
        context: Mapping[str, Any],
        features: np.ndarray,
        history: np.ndarray,
        history_mask: np.ndarray,
        topology_probability: np.ndarray,
        query_probability: np.ndarray,
        decoded_targets: np.ndarray,
        feature_distance: float,
        history_distance: float,
        in_distribution: bool,
        error: Exception,
    ) -> None:
        """Persist fail-fast OOD, confidence, or hard-constraint diagnostics."""

        record = {
            "schema": TRACE_SCHEMA,
            "context": _json_value(context),
            "features": np.asarray(features, dtype=float).tolist(),
            "history": np.asarray(history, dtype=float).tolist(),
            "history_mask": np.asarray(history_mask, dtype=bool).tolist(),
            "topology_probability": np.asarray(topology_probability, dtype=float).tolist(),
            "query_probability": np.asarray(query_probability, dtype=float).tolist(),
            "decoded_targets": np.asarray(decoded_targets, dtype=float).tolist(),
            "feature_distance": float(feature_distance),
            "history_distance": float(history_distance),
            "in_distribution": bool(in_distribution),
            "hard_constraints_passed": False,
            "failure": {
                "type": type(error).__name__,
                "message": str(error),
            },
            "geometry": None,
        }
        _append_jsonl(self._iteration_path, record)
        self._iteration_count += 1
        # A fail-fast exception may bypass the normal close path.  Refresh the
        # manifest immediately so the diagnostic remains discoverable.
        self._write_manifest(complete=False)

    def close(self) -> None:
        self._write_manifest(complete=True)

    def _write_manifest(self, *, complete: bool) -> None:
        payload = {
            "schema": TRACE_SCHEMA,
            "complete": bool(complete),
            "model_path": str(self.model_path),
            "model_sha256": _sha256_file(self.model_path),
            "iteration_records": int(getattr(self, "_iteration_count", 0)),
            "accepted_steps": int(getattr(self, "_accepted_count", 0)),
            "metadata": _json_value(self.metadata),
            "contents": {
                "iterations": self._iteration_path.name,
                "accepted_steps": self._accepted_path.name,
            },
        }
        (self.output_dir / "manifest.json").write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )


def _geometry_record(geometry: DirectContactGeometry) -> dict[str, Any]:
    return {
        "has_contact": bool(geometry.has_contact),
        "in_distribution": bool(geometry.in_distribution),
        "patches": [_patch_record(patch) for patch in geometry.patches],
    }


def _patch_record(patch: DirectContactPatch) -> dict[str, Any]:
    return {
        "start_y_m": float(patch.start_y),
        "end_y_m": float(patch.end_y),
        "corrected_wheel_point_m": np.asarray(patch.corrected_wheel_point, dtype=float).tolist(),
        "corrected_rail_point_m": np.asarray(patch.corrected_rail_point, dtype=float).tolist(),
        "corrected_vertical_penetration_m": float(patch.corrected_vertical_penetration),
        "corrected_normal_penetration_m": float(patch.corrected_normal_penetration),
        "corrected_contact_angle_rad": float(patch.contact_angle),
        "peak_wheel_point_m": np.asarray(patch.peak_wheel_point, dtype=float).tolist(),
        "peak_rail_point_m": np.asarray(patch.peak_rail_point, dtype=float).tolist(),
        "peak_vertical_penetration_m": float(patch.peak_vertical_penetration),
        "peak_normal_penetration_m": float(patch.peak_normal_penetration),
        "peak_contact_angle_rad": float(patch.peak_contact_angle),
        "shape_moments": np.asarray(patch.shape_moments, dtype=float).tolist(),
    }


def _append_jsonl(path: Path, payload: Mapping[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")


def _json_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
