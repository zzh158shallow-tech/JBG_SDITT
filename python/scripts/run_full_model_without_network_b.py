"""Run the full interval model with Network A geometry and Network B disabled.

The run writes only accepted-step wheel/rail force and contact-location key data.
Execute this file from the ``python`` directory so model artifact paths resolve in
the same way as the main validation command.
"""

from __future__ import annotations

import sys

from sditt.validation.full_case_short_run import main


def run() -> int:
    return main(
        [
            "--rail-layout",
            "interval",
            "--contact-geometry-mode",
            "network-a-after-preload",
            "--network-a-model",
            "outputs/wrcp_net_a2r_continuity/model.npz",
            "--disable-network-b",
            "--contact-key-data-only",
            "--live-window",
            "--track-irregularity",
            "china-ballastless",
            "--irregularity-seed",
            "20260716",
            "--full-size",
            "--matlab-mileage-endpoints",
            "--history-retention-steps",
            "1",
            "--contact-force-tolerance",
            "0.0025",
            "--output-dir",
            "outputs/full_model_network_a_without_network_b",
            *sys.argv[1:],
        ]
    )


if __name__ == "__main__":
    raise SystemExit(run())
