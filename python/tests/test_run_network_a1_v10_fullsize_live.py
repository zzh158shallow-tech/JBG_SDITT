from __future__ import annotations

from pathlib import Path

from scripts.run_network_a1_v10_fullsize_live import build_full_case_arguments


def test_v10_live_runner_defaults_to_full_size_and_full_mileage(tmp_path: Path) -> None:
    arguments = build_full_case_arguments(
        output_dir=tmp_path / "output",
        model_path=tmp_path / "model.npz",
        irregularity_seed=20260716,
        preview_steps=None,
        live_window=True,
        resume_checkpoint=False,
        full_trace=False,
    )
    assert "--full-size" in arguments
    assert "--matlab-mileage-endpoints" in arguments
    assert "--live-window" in arguments
    assert "--steps" not in arguments
    assert arguments[arguments.index("--contact-geometry-mode") + 1] == (
        "network-a1-direct-after-preload"
    )
    assert arguments[arguments.index("--network-a-force-mode") + 1] == "hertz"
    assert arguments[arguments.index("--network-a-trace-mode") + 1] == "selective"


def test_v10_live_runner_only_shortens_when_preview_is_explicit(tmp_path: Path) -> None:
    arguments = build_full_case_arguments(
        output_dir=tmp_path / "output",
        model_path=tmp_path / "model.npz",
        irregularity_seed=20260716,
        preview_steps=3,
        live_window=False,
        resume_checkpoint=True,
        full_trace=True,
    )
    assert "--matlab-mileage-endpoints" not in arguments
    assert arguments[arguments.index("--steps") + 1] == "3"
    assert "--live-window" not in arguments
    assert "--resume-checkpoint" in arguments
    assert arguments[arguments.index("--network-a-trace-mode") + 1] == "full"
