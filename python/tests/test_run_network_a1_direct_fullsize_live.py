from pathlib import Path

import scripts.run_network_a1_direct_fullsize_live as live_runner
from scripts.run_network_a1_direct_fullsize_live import (
    DEFAULT_MODEL,
    DEFAULT_NETWORK_B_MODEL,
    DEFAULT_OUTPUT_PREFIX,
    DEFAULT_PRELOAD_CACHE_DIR,
    DEFAULT_SEED,
    build_full_case_arguments,
)


def test_default_live_runner_uses_v22_strict_artifact() -> None:
    assert DEFAULT_MODEL.parent.name == (
        "wrcp_net_a1_direct_multiseed_topology_v22_dagger_strict_stabilized"
    )
    assert DEFAULT_OUTPUT_PREFIX == "network_a1_direct_v22_fullsize_live"
    assert DEFAULT_SEED == 20260721
    assert DEFAULT_NETWORK_B_MODEL.parent.name == (
        "wrcp_net_b_direct_round0_residual_128x128_weight010"
    )
    assert DEFAULT_PRELOAD_CACHE_DIR.name == "shared_preload_cache"


def test_production_arguments_use_full_size_live_window_without_trace() -> None:
    arguments = build_full_case_arguments(
        output_dir=Path("outputs/case"),
        model_path=Path("outputs/model/model.npz"),
        irregularity_seed=20260722,
        preview_steps=None,
        live_window=True,
        resume_checkpoint=True,
        cut_freq=None,
        enable_trace=False,
    )

    assert "--live-window" in arguments
    assert "--full-size" in arguments
    assert "--matlab-mileage-endpoints" in arguments
    assert "--resume-checkpoint" in arguments
    assert "--network-a-trace-dir" not in arguments
    assert arguments[arguments.index("--contact-geometry-mode") + 1] == (
        "network-a1-direct-after-preload"
    )
    assert arguments[arguments.index("--network-a-force-mode") + 1] == "hertz"
    assert arguments[arguments.index("--network-b-ood-fallback") + 1] == "hertz"
    assert arguments[arguments.index("--snapshot-history-limit") + 1] == "1"
    assert "--network-b-model" not in arguments


def test_network_b_arguments_enable_guarded_cal_force_model() -> None:
    arguments = build_full_case_arguments(
        output_dir=Path("outputs/case"),
        model_path=Path("outputs/network_a/model.npz"),
        network_b_model_path=Path("outputs/network_b/model.npz"),
        network_b_ood_threshold=4.0,
        network_b_ood_fallback="hertz",
        irregularity_seed=20260721,
        preview_steps=None,
        live_window=True,
        resume_checkpoint=True,
        cut_freq=None,
        enable_trace=False,
    )

    assert arguments[arguments.index("--network-a-force-mode") + 1] == "network-b"
    assert arguments[arguments.index("--network-b-model") + 1] == (
        "outputs/network_b/model.npz"
    )
    assert arguments[arguments.index("--network-b-ood-threshold") + 1] == "4"
    assert arguments[arguments.index("--network-b-ood-fallback") + 1] == "hertz"


def test_shared_preload_cache_can_be_independent_of_output_directory() -> None:
    arguments = build_full_case_arguments(
        output_dir=Path("outputs/new_result"),
        model_path=Path("outputs/network_a/model.npz"),
        irregularity_seed=20260721,
        preview_steps=None,
        live_window=False,
        resume_checkpoint=False,
        cut_freq=None,
        enable_trace=False,
        preload_cache_dir=Path("outputs/shared_preload_cache"),
    )

    assert arguments[arguments.index("--preload-cache-dir") + 1] == (
        "outputs/shared_preload_cache"
    )
    assert arguments[arguments.index("--checkpoint-dir") + 1] == (
        "outputs/new_result/checkpoints"
    )


def test_main_passes_existing_network_b_model_to_full_case(
    tmp_path,
    monkeypatch,
) -> None:
    network_a_model = tmp_path / "network_a.npz"
    network_b_model = tmp_path / "network_b.npz"
    network_a_model.touch()
    network_b_model.touch()
    captured: dict[str, list[str]] = {}

    def fake_full_case(arguments: list[str]) -> int:
        captured["arguments"] = arguments
        return 0

    monkeypatch.setattr(live_runner, "run_full_case", fake_full_case)
    result = live_runner.main(
        [
            "--network-a-model",
            str(network_a_model),
            "--network-b-model",
            str(network_b_model),
            "--irregularity-seed",
            "20260721",
            "--preview-steps",
            "2",
            "--cut-freq",
            "50",
            "--headless",
            "--no-resume-checkpoint",
            "--output-dir",
            str(tmp_path / "output"),
        ]
    )

    arguments = captured["arguments"]
    assert result == 0
    assert arguments[arguments.index("--network-a-force-mode") + 1] == "network-b"
    assert arguments[arguments.index("--network-b-model") + 1] == str(network_b_model)
    assert arguments[arguments.index("--irregularity-seed") + 1] == "20260721"
    assert arguments[arguments.index("--preload-cache-dir") + 1] == str(
        DEFAULT_PRELOAD_CACHE_DIR.resolve()
    )


def test_diagnostic_arguments_can_be_short_headless_and_selectively_traced() -> None:
    arguments = build_full_case_arguments(
        output_dir=Path("outputs/case"),
        model_path=Path("outputs/model/model.npz"),
        irregularity_seed=7,
        preview_steps=2,
        live_window=False,
        resume_checkpoint=False,
        cut_freq=50.0,
        enable_trace=True,
    )

    assert "--live-window" not in arguments
    assert "--full-size" not in arguments
    assert "--matlab-mileage-endpoints" not in arguments
    assert arguments[arguments.index("--steps") + 1] == "2"
    assert arguments[arguments.index("--cut-freq") + 1] == "50.0"
    assert "--no-resume-checkpoint" in arguments
    assert arguments[arguments.index("--network-a-trace-mode") + 1] == "selective"
