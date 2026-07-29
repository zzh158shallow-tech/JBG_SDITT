"""Diagnostic runner that gives every wheel/side a single-patch Cal anchor.

This is intentionally not a production route.  During traditional Preload it
retains the latest accepted one-patch geometry for every wheel/side instead of
overwriting that diagnostic anchor with a later no-contact state.  Use it with
an A1 Direct artifact whose topology hysteresis is above one to keep that
single-patch topology fixed throughout Cal.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np


PYTHON_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYTHON_ROOT))

from sditt.models.network_a1_direct_full_case import (
    NetworkA1DirectContactGeometryAdapter,
    direct_geometry_from_traditional,
)
from sditt.validation.full_case_short_run import main as run_full_case


def retain_latest_single_patch_preload_anchor(
    adapter: NetworkA1DirectContactGeometryAdapter,
    snapshot: Any,
) -> None:
    """Retain the latest traditional one-patch anchor for each wheel/side."""

    if snapshot is None:
        return
    for wheelset, by_side in snapshot.wheel_rail_contact.geometry_by_wheelset_side.items():
        for side, geometry in by_side.items():
            if geometry is None or len(geometry.patches) != 1:
                continue
            shift = np.asarray(
                snapshot.effective_rail_displacement_by_wheelset_side[wheelset][side],
                dtype=float,
            ).reshape(2)
            direct = direct_geometry_from_traditional(geometry, rail_shift_yz=shift)
            if len(direct.patches) == 1:
                adapter._accepted[f"{wheelset}:{side}"] = direct


def main() -> int:
    NetworkA1DirectContactGeometryAdapter.seed_from_traditional_snapshot = (
        retain_latest_single_patch_preload_anchor
    )
    return run_full_case(sys.argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
