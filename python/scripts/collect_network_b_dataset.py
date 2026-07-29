from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


PYTHON_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PYTHON_ROOT.parent
sys.path.insert(0, str(PYTHON_ROOT))

from sditt.config import DEFAULT_OPERATING_CASE, ProjectPaths
from sditt.simulation import FullDefaultCaseSettings, run_default_full_case_driver
from sditt.track import TrackIrregularitySettings
from sditt.training_data.network_b import (
    NetworkBAcceptedStepCollector,
    load_network_b_dataset,
    save_network_b_dataset,
    validate_network_b_dataset,
)


DEFAULT_DIRECT_NETWORK_A_MODEL = (
    PYTHON_ROOT
    / "outputs"
    / "wrcp_net_a1_direct_multiseed_topology_v22_dagger_strict_stabilized"
    / "model.npz"
)
DEFAULT_OUTPUT_DIR = PYTHON_ROOT / "outputs" / "network_b_direct_dataset_v3"


def main() -> int:
    args = _parse_args()
    output_dir = _resolve_output_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    direct = args.contact_geometry_mode == "network-a1-direct-after-preload"
    network_a_model = _resolve_input_path(
        args.network_a_model
        or (
            DEFAULT_DIRECT_NETWORK_A_MODEL
            if direct
            else Path("outputs/wrcp_net_a2r_continuity/model.npz")
        )
    )
    if args.contact_geometry_mode != "traditional" and not network_a_model.is_file():
        raise SystemExit(f"Network A model artifact does not exist: {network_a_model}")
    network_b_model = _resolve_input_path(args.network_b_model)
    if args.shadow_teacher and not direct and not network_b_model.is_file():
        raise SystemExit(f"Network B bootstrap model does not exist: {network_b_model}")

    configuration = _collection_configuration(
        args,
        network_a_model=network_a_model,
        network_b_model=network_b_model,
    )
    manifest_path = output_dir / "manifest.json"
    manifest = _load_or_create_manifest(
        manifest_path,
        configuration=configuration,
        resume=args.resume,
    )
    completed_seeds = {
        int(item["seed"])
        for item in manifest.get("datasets", [])
        if bool(item.get("complete", False))
    }

    for seed in args.seeds:
        if args.resume and int(seed) in completed_seeds:
            print(f"seed {seed} is already complete; skipping", flush=True)
            continue
        record = _collect_seed(
            args,
            seed=int(seed),
            output_dir=output_dir,
            network_a_model=network_a_model,
            network_b_model=network_b_model,
        )
        _replace_seed_record(manifest, record)
        manifest["complete"] = all(
            bool(item.get("complete", False)) for item in manifest["datasets"]
        ) and {int(item["seed"]) for item in manifest["datasets"]} >= set(args.seeds)
        _write_json_atomic(manifest_path, manifest)
        print(json.dumps(record, indent=2, ensure_ascii=False), flush=True)
        if not bool(record["complete"]):
            print(
                f"seed {seed} did not pass collection gates; "
                f"see {record['quality_report']}",
                file=sys.stderr,
            )
            return 1
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Collect final accepted-Cal patch labels for WRCP-Net B with "
            "seed-isolated, resumable shards and provenance."
        )
    )
    parser.add_argument("--seeds", nargs="+", type=int, required=True)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--cut-freq", type=float, default=50.0)
    parser.add_argument("--full-size", action="store_true")
    parser.add_argument("--matlab-mileage-endpoints", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--contact-geometry-mode",
        choices=(
            "traditional",
            "network-a-after-preload",
            "network-a1-direct-after-preload",
        ),
        default="network-a1-direct-after-preload",
    )
    parser.add_argument("--network-a-model", type=Path, default=None)
    parser.add_argument(
        "--network-b-model",
        type=Path,
        default=Path("outputs/wrcp_net_b/model.npz"),
    )
    parser.add_argument(
        "--shadow-teacher",
        action="store_true",
        help=(
            "For non-Direct Network A, run an existing Network B model while "
            "retaining traditional labels. Direct A1 always enables its "
            "parallel traditional teacher."
        ),
    )
    parser.add_argument("--shard-interval-m", type=float, default=10.0)
    parser.add_argument("--checkpoint-interval-m", type=float, default=10.0)
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume compatible solver checkpoints and append deduplicated shards.",
    )
    parser.add_argument(
        "--enable-network-a-trace",
        action="store_true",
        help="Enable selective A1 iteration tracing for explicit diagnostics.",
    )
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.cut_freq <= 0.0:
        parser.error("--cut-freq must be positive")
    if args.shard_interval_m <= 0.0 or args.checkpoint_interval_m <= 0.0:
        parser.error("shard and checkpoint intervals must be positive")
    if args.full_size and not args.matlab_mileage_endpoints:
        parser.error("--full-size data collection requires --matlab-mileage-endpoints")
    if len(set(args.seeds)) != len(args.seeds):
        parser.error("--seeds must not contain duplicates")
    return args


def _collect_seed(
    args: argparse.Namespace,
    *,
    seed: int,
    output_dir: Path,
    network_a_model: Path,
    network_b_model: Path,
) -> dict[str, object]:
    direct = args.contact_geometry_mode == "network-a1-direct-after-preload"
    seed_dir = output_dir / f"seed_{seed}"
    seed_dir.mkdir(parents=True, exist_ok=True)
    collector = NetworkBAcceptedStepCollector(
        irregularity_seed=seed,
        require_shadow_teacher=direct,
        shard_dir=seed_dir / "shards",
        shard_interval_m=float(args.shard_interval_m),
        resume=bool(args.resume),
    )
    trace_dir = seed_dir / "network_a_trace" if args.enable_network_a_trace else None
    force_mode = (
        "hertz"
        if direct
        else "network-b" if args.shadow_teacher else "traditional"
    )
    shadow_teacher = direct or bool(args.shadow_teacher)
    settings = FullDefaultCaseSettings(
        cut_freq=None if args.full_size else float(args.cut_freq),
        n_steps_per_stage=int(args.steps),
        stage_end_mileage=(
            None
            if args.matlab_mileage_endpoints
            else {"Preload": 47.6} if direct else None
        ),
        use_matlab_mileage_endpoints=bool(args.matlab_mileage_endpoints),
        history_retention_steps=2,
        accepted_contact_callback=collector,
        preload_cache_dir=seed_dir / "preload_cache",
        checkpoint_dir=seed_dir / "checkpoints",
        save_checkpoints=bool(args.matlab_mileage_endpoints),
        resume_checkpoint=bool(args.resume),
        checkpoint_interval_m=float(args.checkpoint_interval_m),
        contact_geometry_mode=args.contact_geometry_mode,
        network_a_model_path=network_a_model,
        network_a_trace_dir=trace_dir,
        network_a_trace_mode="selective",
        network_a_trace_low_confidence=0.95,
        network_a_trace_sample_interval_m=1.0,
        network_a_force_mode=force_mode,
        network_b_model_path=network_b_model,
        network_b_shadow_teacher=shadow_teacher,
        network_b_ood_fallback="hertz" if direct else "traditional",
        track_irregularity=TrackIrregularitySettings(
            model="china-ballastless",
            seed=seed,
        ),
    )
    print(
        f"collecting seed {seed}: geometry={args.contact_geometry_mode}, "
        f"force={force_mode}, full_size={args.full_size}, "
        f"full_mileage={args.matlab_mileage_endpoints}",
        flush=True,
    )
    result = None
    error: Exception | None = None
    try:
        result = run_default_full_case_driver(
            repo_root=REPO_ROOT,
            settings=settings,
            operating_case=DEFAULT_OPERATING_CASE,
        )
    except Exception as exc:
        error = exc
    finally:
        dataset = collector.finalize()
        consolidated_path = output_dir / f"seed_{seed}.npz"
        if len(dataset):
            temporary = consolidated_path.with_suffix(".tmp.npz")
            save_network_b_dataset(temporary, dataset)
            temporary.replace(consolidated_path)
        quality = _quality_report(
            dataset,
            expected_seed=seed,
            require_direct=direct,
            require_all_wheel_sides=bool(args.matlab_mileage_endpoints),
        )
        collection = collector.summary()
        final_front_mileage = collection["last_front_mileage_m"]
        reaches_endpoint = (
            (
                final_front_mileage is not None
                and abs(float(final_front_mileage) - 130.0) <= 1.0e-9
            )
            if args.matlab_mileage_endpoints
            else int(collection["accepted_steps"]) == int(args.steps)
        )
        complete = bool(
            error is None
            and result is not None
            and len(dataset) > 0
            and quality["valid"]
            and reaches_endpoint
        )
        record: dict[str, object] = {
            "seed": seed,
            "complete": complete,
            "dataset_path": str(consolidated_path),
            "dataset_sha256": (
                _sha256_file(consolidated_path) if consolidated_path.is_file() else None
            ),
            "quality_report": str(seed_dir / "quality_report.json"),
            "collection": collection,
            "quality": quality,
            "reaches_requested_cal_endpoint": reaches_endpoint,
            "error": None if error is None else f"{type(error).__name__}: {error}",
        }
        _write_json_atomic(seed_dir / "quality_report.json", record)
    return record


def _quality_report(
    dataset: Any,
    *,
    expected_seed: int,
    require_direct: bool,
    require_all_wheel_sides: bool,
) -> dict[str, object]:
    try:
        return validate_network_b_dataset(
            dataset,
            expected_seed=expected_seed,
            require_direct_schema=require_direct,
            require_all_wheel_sides=require_all_wheel_sides,
        )
    except ValueError as exc:
        from sditt.training_data.network_b import network_b_dataset_quality_report

        report = network_b_dataset_quality_report(dataset)
        report["valid"] = False
        report["validation_error"] = str(exc)
        return report


def _collection_configuration(
    args: argparse.Namespace,
    *,
    network_a_model: Path,
    network_b_model: Path,
) -> dict[str, object]:
    paths = ProjectPaths.from_repo_root(REPO_ROOT)
    tracked_inputs = {
        "network_a_model": network_a_model,
        "standard_basic_rail_profile": paths.standard_basic_rail_profile,
        "vehicle_parameters": paths.default_vehicle_parameters,
        "collector_script": Path(__file__),
        "network_b_dataset_module": PYTHON_ROOT / "sditt" / "training_data" / "network_b.py",
        "contact_solver_module": PYTHON_ROOT / "sditt" / "contact" / "full_case.py",
    }
    if args.shadow_teacher and network_b_model.is_file():
        tracked_inputs["network_b_bootstrap_model"] = network_b_model
    return {
        "schema": "network-b-direct-collection-manifest-v1",
        "teacher": "Python traditional STRIPES + Kalker",
        "accepted_cal_steps_only": True,
        "contact_geometry_mode": args.contact_geometry_mode,
        "force_mode": (
            "hertz"
            if args.contact_geometry_mode == "network-a1-direct-after-preload"
            else "network-b" if args.shadow_teacher else "traditional"
        ),
        "shadow_teacher": bool(
            args.shadow_teacher
            or args.contact_geometry_mode == "network-a1-direct-after-preload"
        ),
        "rail_layout": "interval L1+R1",
        "track_irregularity": "china-ballastless",
        "steps_per_stage": int(args.steps),
        "full_preload": bool(
            args.matlab_mileage_endpoints
            or args.contact_geometry_mode == "network-a1-direct-after-preload"
        ),
        "full_size": bool(args.full_size),
        "matlab_mileage_endpoints": bool(args.matlab_mileage_endpoints),
        "cut_freq_Hz": None if args.full_size else float(args.cut_freq),
        "shard_interval_m": float(args.shard_interval_m),
        "checkpoint_interval_m": float(args.checkpoint_interval_m),
        "friction_coefficient": 0.40,
        "seeds": [int(seed) for seed in args.seeds],
        "git": _git_provenance(),
        "inputs": {
            name: _file_provenance(path)
            for name, path in tracked_inputs.items()
            if path.is_file()
        },
    }


def _load_or_create_manifest(
    path: Path,
    *,
    configuration: dict[str, object],
    resume: bool,
) -> dict[str, Any]:
    if not path.exists():
        manifest: dict[str, Any] = {
            **configuration,
            "complete": False,
            "datasets": [],
        }
        _write_json_atomic(path, manifest)
        return manifest
    if not resume:
        raise FileExistsError(
            f"output manifest already exists; use --resume or a new output directory: {path}"
        )
    manifest = json.loads(path.read_text(encoding="utf-8"))
    for key, value in configuration.items():
        if key == "seeds":
            continue
        if manifest.get(key) != value:
            raise ValueError(f"resume configuration mismatch for {key}")
    manifest["seeds"] = sorted(
        set(int(seed) for seed in manifest.get("seeds", []))
        | set(int(seed) for seed in configuration["seeds"])
    )
    return manifest


def _replace_seed_record(manifest: dict[str, Any], record: dict[str, object]) -> None:
    datasets = [
        item for item in manifest.get("datasets", []) if int(item["seed"]) != int(record["seed"])
    ]
    datasets.append(record)
    datasets.sort(key=lambda item: int(item["seed"]))
    manifest["datasets"] = datasets


def _resolve_input_path(value: Path) -> Path:
    if value.is_absolute():
        return value.resolve()
    python_candidate = (PYTHON_ROOT / value).resolve()
    repo_candidate = (REPO_ROOT / value).resolve()
    if python_candidate.exists() or not repo_candidate.exists():
        return python_candidate
    return repo_candidate


def _resolve_output_path(value: Path) -> Path:
    if value.is_absolute():
        return value.resolve()
    if value.parts and value.parts[0] == "python":
        return (REPO_ROOT / value).resolve()
    return (PYTHON_ROOT / value).resolve()


def _file_provenance(path: Path) -> dict[str, object]:
    resolved = path.resolve()
    stat = resolved.stat()
    return {
        "path": str(resolved),
        "size": int(stat.st_size),
        "mtime_ns": int(stat.st_mtime_ns),
        "sha256": _sha256_file(resolved),
    }


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _git_provenance() -> dict[str, object]:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    status = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return {"commit": commit, "dirty": bool(status.strip())}


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


if __name__ == "__main__":
    raise SystemExit(main())
