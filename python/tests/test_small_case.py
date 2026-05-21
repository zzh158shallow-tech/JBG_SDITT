from __future__ import annotations

import numpy as np

from sditt.simulation import SmallRigidWheelsetCase, export_small_rigid_wheelset_curve, run_small_rigid_wheelset_case


def test_small_rigid_wheelset_case_outputs_finite_force_curve() -> None:
    case = SmallRigidWheelsetCase(distance=0.25, dt=2.0e-4, irregularity_start=0.05, irregularity_length=0.10)

    result = run_small_rigid_wheelset_case(case)

    assert result.mileage[0] == 0.0
    assert result.mileage[-1] >= case.distance
    assert result.normal_force.shape == result.mileage.shape
    assert result.iterations.shape == result.mileage.shape
    assert np.all(np.isfinite(result.normal_force))
    assert np.all(result.normal_force >= 0.0)
    assert np.max(result.normal_force) > case.static_wheel_load
    assert np.max(result.iterations[1:]) <= 30


def test_export_small_rigid_wheelset_curve_writes_csv_and_svg(tmp_path) -> None:
    result, csv_path, svg_path = export_small_rigid_wheelset_curve(
        tmp_path,
        SmallRigidWheelsetCase(distance=0.10, dt=2.0e-4, irregularity_start=0.02, irregularity_length=0.04),
    )

    assert csv_path.exists()
    assert svg_path.exists()
    assert "normal_force_N" in csv_path.read_text(encoding="utf-8").splitlines()[0]
    assert "<polyline" in svg_path.read_text(encoding="utf-8")
    assert result.normal_force.size > 2
