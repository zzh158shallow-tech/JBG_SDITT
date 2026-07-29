from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


PYTHON_ROOT = Path(__file__).resolve().parents[1]
NUMERIC_COLUMNS = (
    "step",
    "time_s",
    "dt_s",
    "front_mileage_m",
    "iterations",
    "wheelset_mileage_m",
    "contact_patch_index",
    "patch_id",
    "force_on_wheel_x_N",
    "force_on_wheel_y_N",
    "force_on_wheel_z_N",
    "resultant_force_on_wheel_N",
    "normal_force_N",
    "wheel_contact_x_m",
    "wheel_contact_y_m",
    "wheel_contact_z_m",
    "rail_contact_x_m",
    "rail_contact_y_m",
    "rail_contact_z_m",
    "contact_angle_rad",
    "vertical_penetration_m",
    "normal_penetration_m",
)


def _resolve(path: Path) -> Path:
    return path if path.is_absolute() else (PYTHON_ROOT / path).resolve()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _strictly_increasing(values: list[float]) -> bool:
    return all(right > left for left, right in zip(values, values[1:], strict=False))


def _stage_summary(rows: list[dict[str, str]]) -> dict[str, Any]:
    by_step: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        by_step[int(row["step"])].append(row)
    steps = sorted(by_step)
    mileages = [float(by_step[step][0]["front_mileage_m"]) for step in steps]
    times = [float(by_step[step][0]["time_s"]) for step in steps]
    fallbacks = {
        step: any(row["network_b_fallback"].lower() == "true" for row in step_rows)
        for step, step_rows in by_step.items()
    }
    expected_wheel_sides = {
        ("FF", "L"),
        ("FF", "R"),
        ("FR", "L"),
        ("FR", "R"),
        ("RF", "L"),
        ("RF", "R"),
        ("RR", "L"),
        ("RR", "R"),
    }
    covered = {(row["wheelset"], row["side"]) for row in rows}
    complete_steps = sum(
        {(row["wheelset"], row["side"]) for row in step_rows}
        == expected_wheel_sides
        for step_rows in by_step.values()
    )
    accepted_steps = len(steps)
    return {
        "rows": len(rows),
        "accepted_steps": accepted_steps,
        "step_min": min(steps),
        "step_max": max(steps),
        "skipped_step_indices": max(steps) - min(steps) + 1 - accepted_steps,
        "front_mileage_start_m": mileages[0],
        "front_mileage_end_m": mileages[-1],
        "time_start_s": times[0],
        "time_end_s": times[-1],
        "mileage_strictly_increasing": _strictly_increasing(mileages),
        "time_strictly_increasing": _strictly_increasing(times),
        "wheel_side_coverage": sorted(f"{wheelset}-{side}" for wheelset, side in covered),
        "complete_eight_wheel_side_steps": complete_steps,
        "rows_per_step_histogram": {
            str(key): value
            for key, value in sorted(Counter(len(value) for value in by_step.values()).items())
        },
        "iterations_histogram": {
            str(key): value
            for key, value in sorted(
                Counter(int(by_step[step][0]["iterations"]) for step in steps).items()
            )
        },
        "fallback_rows": sum(
            row["network_b_fallback"].lower() == "true"
            for row in rows
        ),
        "fallback_steps": sum(fallbacks.values()),
        "fallback_step_fraction": sum(fallbacks.values()) / accepted_steps,
        "maximum_contact_patch_index": max(int(row["contact_patch_index"]) for row in rows),
        "minimum_normal_force_N": min(float(row["normal_force_N"]) for row in rows),
        "maximum_normal_force_N": max(float(row["normal_force_N"]) for row in rows),
        "maximum_resultant_force_on_wheel_N": max(
            float(row["resultant_force_on_wheel_N"])
            for row in rows
        ),
    }


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _write_report(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# Network A-1 + Network B 闭环短跑摘要",
        "",
        f"- CSV：`{summary['input']['csv_path']}`",
        f"- CSV SHA256：`{summary['input']['csv_sha256']}`",
        f"- 总行数：{summary['quality']['rows']}",
        f"- 数值全部有限：`{str(summary['quality']['all_numeric_values_finite']).lower()}`",
        f"- Network B 模型：`{summary['models']['network_b']['path']}`",
        f"- Network B SHA256：`{summary['models']['network_b']['sha256']}`",
        "",
        "| 阶段 | 接受步 | 前端终点里程 (m) | 完整 8 轮/侧步 | 最大迭代 | 回退步 |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for stage_name, stage in summary["stages"].items():
        maximum_iterations = max(int(key) for key in stage["iterations_histogram"])
        lines.append(
            f"| {stage_name} | {stage['accepted_steps']} | "
            f"{stage['front_mileage_end_m']:.12g} | "
            f"{stage['complete_eight_wheel_side_steps']} | "
            f"{maximum_iterations} | {stage['fallback_steps']} |"
        )
    cal = summary["stages"].get("Cal", {})
    lines.extend(
        [
            "",
            "## 结论",
            "",
            f"- Cal 时间严格递增：`{str(cal.get('time_strictly_increasing', False)).lower()}`",
            f"- Cal 里程严格递增：`{str(cal.get('mileage_strictly_increasing', False)).lower()}`",
            f"- Cal 轮/侧覆盖：`{', '.join(cal.get('wheel_side_coverage', []))}`",
            f"- Cal Network B OOD 回退比例："
            f"`{100.0 * float(cal.get('fallback_step_fraction', 0.0)):.6f}%`",
            f"- 最大接触斑序号：`{cal.get('maximum_contact_patch_index', 0)}`；"
            "本次只验证单接触斑。",
            "",
            "该摘要只证明验证 seed 上的有界闭环稳定性，不替代封存测试 seed 的一次性盲测，"
            "也不代表完整里程或多接触斑生产门槛已经通过。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Summarize an accepted-step Network A-1 + Network B rollout CSV."
    )
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--network-a-model", type=Path, required=True)
    parser.add_argument("--network-b-model", type=Path, required=True)
    parser.add_argument("--irregularity-seed", type=int, required=True)
    args = parser.parse_args()

    csv_path = _resolve(args.csv)
    output_dir = _resolve(args.output_dir)
    network_a_model = _resolve(args.network_a_model)
    network_b_model = _resolve(args.network_b_model)
    for path in (csv_path, network_a_model, network_b_model):
        if not path.is_file():
            parser.error(f"input does not exist: {path}")

    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        parser.error("rollout CSV contains no data rows")
    missing = set(NUMERIC_COLUMNS) - set(rows[0])
    if missing:
        parser.error("rollout CSV is missing columns: " + ", ".join(sorted(missing)))
    finite = all(
        math.isfinite(float(row[column]))
        for row in rows
        for column in NUMERIC_COLUMNS
    )
    stages = {
        stage: _stage_summary([row for row in rows if row["stage"] == stage])
        for stage in ("Preload", "Cal")
        if any(row["stage"] == stage for row in rows)
    }
    summary = {
        "schema": "network-ab-rollout-summary-v1",
        "input": {
            "csv_path": str(csv_path),
            "csv_sha256": _sha256(csv_path),
            "irregularity_seed": int(args.irregularity_seed),
        },
        "models": {
            "network_a": {
                "path": str(network_a_model),
                "sha256": _sha256(network_a_model),
            },
            "network_b": {
                "path": str(network_b_model),
                "sha256": _sha256(network_b_model),
            },
        },
        "quality": {
            "rows": len(rows),
            "all_numeric_values_finite": finite,
        },
        "stages": stages,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "closed_loop_summary.json"
    report_path = output_dir / "closed_loop_report.md"
    _write_json(json_path, summary)
    _write_report(report_path, summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"wrote {json_path}")
    print(f"wrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
