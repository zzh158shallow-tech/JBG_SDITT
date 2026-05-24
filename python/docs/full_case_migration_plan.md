# Full Default Case Migration Plan

This document preserves the handoff context for continuing the Python migration
of the SDITT vehicle-turnout rigid-flexible coupled dynamics model.

Use this prompt in a new Codex conversation:

```text
Please continue the SDITT Python full-case migration according to
python/docs/full_case_migration_plan.md. Use the project virtual environment at
python/.venv/bin/python, preserve existing changes, and validate with pytest.
```

## Current State

- Python code lives under `python/`.
- Use the project interpreter:

```bash
cd /Users/zhou/Documents/Codex/SDITT/python
.venv/bin/python -m pytest
```

- Current test status after wiring the default full-case contact main route,
  CRH380A_v6 nonlinear dampers, and the default straight-layout gating:

```text
75 passed, 1 skipped
```

- The current full default driver is in:
  - `python/sditt/simulation/full_case.py`
  - exported through `python/sditt/simulation/__init__.py`

- The current driver can:
  - Build the MATLAB default `07(009)` / `Face` / `350 km/h` / `FT-Modal` /
    `CRH380A_v6` system.
  - Build the full 2000 Hz sparse route with 6248 total DOF:
    6197 modal track DOF plus 51 rigid vehicle DOF.
  - Build and apply the modal flexible-turnout gravity preload vector
    `Pxt_Gravity`, including vehicle bodies, rails, baseplates, and space
    irons.
  - Build the default `07(009)` / `Face` `RailPro_ProCS` profile-coordinate
    rail selections for `L1`, `R1`, `R2`, and `R3`.
  - Build default `Cal_ShapeFunction_Beam188_FWV` beam shape functions and
    `RailBeam_Motion` inputs for the current front mileage.
  - Recover contact-point rail displacement, velocity, acceleration, and
    rail-node dynamic status through `RailDyn_ModalFT`.
  - Assemble the default rigid-wheel `Multi_Con_250812` contact route and
    emit MATLAB-style `Con_WS`, `Pjc`, `Pjch`, `Pjcc`, `Prhx`, and `Prhxf`
    structures for the four wheelsets.
  - Evaluate the CRH380A_v6 nonlinear damper branches
    (`Judge_DamperNL.m` / `NonLinear_DampingForce_v4Re.m`) for the current
    full-system state, including damper marks, updated damping branches, and
    the equivalent force correction needed by the Python coupled loop.
  - Map frozen wheel-rail force inputs into both the modal turnout DOF block
    and the rigid vehicle DOF block through the existing force-mapping
    functions.
  - Execute the two-stage `Preload -> Cal` coupled loop.
  - Preserve stage handoff of displacement, velocity, acceleration, and contact
    force.
  - Carry the MATLAB default `Type_Layout = Straight` flag into `InpPar` and
    skip the curve/layout external-force branch for the default route.
  - Leave `missing_stages` empty for the default straight FT-Modal route while
    still guarding the non-default curve/layout external-force branch.

- The current default driver is physically complete for the MATLAB straight
  FT-Modal route covered by this migration plan. It now computes wheel-rail
  contact, applies the CRH380A_v6 nonlinear vehicle dampers, stores
  MATLAB-style iteration histories and accepted-step outputs, and maps those
  forces into the coupled system. The curve/layout external-force stage still
  remains a non-default branch because the main MATLAB case fixes
  `Type_Layout = Straight`.

## Useful Entry Points

Validated low-cutoff full-case smoke run:

```python
from sditt.simulation import FullDefaultCaseSettings, run_default_full_case_driver

result = run_default_full_case_driver(
    settings=FullDefaultCaseSettings(
        cut_freq=50.0,
        dt=1e-4,
        n_steps_per_stage=2,
    )
)
print(result.preparation.total_dof)
print([stage.stage for stage in result.stages])
print(result.preparation.missing_stages)
```

Strict mode for the straight default route:

```python
from sditt.simulation import FullDefaultCaseSettings, run_default_full_case_driver

run_default_full_case_driver(
    settings=FullDefaultCaseSettings(fail_on_missing_physics=True)
)
```

Full-size preparation smoke check:

```bash
cd /Users/zhou/Documents/Codex/SDITT/python
.venv/bin/python -c "from sditt.simulation import prepare_default_full_case; p=prepare_default_full_case(); print(p.total_dof, p.system.Mxt.shape, len(p.missing_stages))"
```

Expected output shape:

```text
6248 (6248, 6248) 1
```

## Missing Physical Models

### 1. Gravity Preload

- Status: implemented in `python/sditt/track/gravity.py` and injected into the
  full-case driver as `preparation.gravity_preload.pxt_gravity`.
- Former missing stage: `gravity_load_modal_ft`
- MATLAB reference: `Gravity_Load_ModalFT.m`
- MATLAB baseline export helper:
  `python/matlab/export_gravity_preload_baseline.m`
- Implemented behavior:
  - Build the static gravity/preload force vector for the modal flexible turnout
    route.
  - Inject it into the full driver as the baseline `Pxt_Gravity`.
- Validation:
  - `cut_freq=50` short case compared against MATLAB R2026a
    `Gravity_Load_ModalFT`: `max_abs_err = 0.0`.
  - Vehicle body force DOF locations and modal track force injection are covered
    by `python/tests/test_full_case_driver.py`.

### 2. Main Rail Profile Selection

- Status: implemented in `python/sditt/profiles/selection.py` and exposed
  through `preparation.profile_selector`.
- Former missing stage: `main_profile_selection`
- MATLAB references:
  - `Get_Profile_P1_Through_210624.m`
  - `Get_Profile_P1_Through_CN18_250813.m`
- MATLAB baseline export helper:
  `python/matlab/export_profile_selection_baseline.m`
- Implemented behavior:
  - Given mileage `j1` and wheelset positions, construct `RailPro_ProCS`.
  - Support the default `07(009)` / `Face` route for `L1`, `R1`, `R2`, and
    `R3`.
- Remaining scope:
  - Add CN18 after the default route is stable.
- Validation:
  - `j1=54 m` compared against MATLAB R2026a
    `Get_Profile_P1_Through_210624`: profile numbers, profile row counts,
    radius row counts, selected FF profiles, and FF front/rear source profiles
    match; profile arrays differ only at floating roundoff
    (`R1_FF max_abs_err = 2.1e-16`, `R2_FF max_abs_err = 5.9e-17`).

### 3. Beam Shape Functions

- Status: implemented in `python/sditt/track/shape_function.py` and exposed
  through `preparation.shape_function_context`.
- MATLAB reference: `Cal_ShapeFunction_Beam188_FWV.m`
- MATLAB baseline export helper:
  `python/matlab/export_shape_function_baseline.m`
- Implemented behavior:
  - Compute `ShapeFunction` and `RailBeam_Motion` for the current mileage.
  - Load cut-frequency-trimmed `ModeShape`, `Pos_Node`, `N_Node`, and
    `DOF_Node` inputs for the default modal turnout.
  - Build the active default `RailBeam.Pos_Z.R2_zjg` /
    `RailBeam.Vel_Z.R2_zjg` tables from `Cal_RailBeam_230518.m`.
  - Provide mapping vectors and interpolation weights used by:
    - `wr_force_modal_ft`
    - `rail_dyn_modal_ft`
- Validation:
  - Low-cutoff preparation confirms modal shape dimensions match the default
    5-mode system.
  - `j1=54 m` checks cover the `qjbg` shape vectors, 1-based mapping indices,
    empty out-of-range contact branches, and `R2` rail-beam interpolation.

### 4. Modal Rail Response Recovery

- Status: implemented in `python/sditt/track/dynamics.py` and wired into
  `python/sditt/simulation/full_case.py`.
- MATLAB reference: `RailDyn_ModalFT.m`
- MATLAB baseline export helper:
  `python/matlab/export_rail_dynamics_baseline.m`
- Implemented behavior:
  - Recover `Dis_Rail`, `Vel_Rail`, and `Acc_Rail` at each contact patch.
  - Recover per-rail node dynamic status arrays matching MATLAB's six-column
    `DynStatus_Rail` layout.
  - Evaluate `ShapeFunction` at the current front-wheel mileage inside the
    full-case `recover_track_response` callback and carry the results in
    `FullCaseRailRecovery` for downstream stages.
- Validation:
  - Full-case diagnostic tests verify the driver now reports
    `physical_rail_recovered=True`.
  - The accepted `Cal` step rail response is checked against a direct
    `rail_dyn_modal_ft` call using the same modal state and evaluated shape
    function.

### 5. Wheel-Rail Force Mapping

- Status: implemented through the full-case frozen-contact path in
  `python/sditt/simulation/full_case.py`.
- MATLAB references:
  - `WR_Force_VehicleSys_RotationIII.m`
  - `WR_Force_ModalFT.m`
- MATLAB baseline export helper:
  `python/matlab/export_force_mapping_baseline.m`
- Implemented behavior:
  - Map wheel-rail forces into both:
    - rigid vehicle DOF force vector
    - flexible turnout modal force vector
  - Allow `run_default_full_case_driver` to accept frozen `Pjcc`, `Pjch`,
    `Prhxf`, `Con_WS`, and `xlcs` through
    `FullDefaultCaseSettings.frozen_contact_input`.
  - Combine `wr_force_modal_ft` and `wr_force_vehicle_sys_rotation_iii`
    directly in the full-case `contact_force` callback.
- Validation:
  - Full-case tests verify the accepted `Cal` step contact-force vector equals
    the direct composition of `wr_force_modal_ft` and
    `wr_force_vehicle_sys_rotation_iii` for the same frozen input.
  - The mapped contact force is confirmed to affect both the track modal block
    and the rigid vehicle block.

### 6. Main Wheel-Rail Contact Routine

- Status: implemented for the default rigid-wheel full-case route in
  `python/sditt/contact/full_case.py` and wired into
  `python/sditt/simulation/full_case.py`.
- MATLAB reference: `Multi_Con_250812.m`
- Related Python pieces used by the assembled route:
  - contact geometry helpers in `python/sditt/contact/geometry.py`
  - Hertz, STRIPES, Hu-Guo damping, and Kalker helpers in
    `python/sditt/contact/forces.py`
- Implemented behavior:
  - Build per-wheelset merged track-coordinate rail profiles from
    `RailPro_ProCS` selections and recovered rail contact-point states.
  - Detect left/right contact patches with the migrated multi-point geometry
    helper and assign them back onto `L1/R1/R2/R3`.
  - Emit MATLAB-style `Con_WS`, `Pjc`, `Pjch`, `Pjcc`, `Prhx`, and `Prhxf`
    structures that plug directly into the existing force-mapping functions.
  - Map the rigid wheelset DOF block back into MATLAB-style wheel pose and
    rate terms (`Yw`, `Zw`, `Roll`, `Yaw`, `vel_WS_track`) for the default
    straight route.
  - Rebuild the rigid-wheel local kinematics used by
    `Multi_Con_250812.m` section 6.2a-II / 6.3, including `Vjd`, `Vjd_r`,
    `Vsdc`, `Vjsdc`, and `RHLv`.
  - Feed recovered rail displacement and velocity, plus the default `R2`
    rail-beam lift velocity, back into profile offsets and creepage inputs.
  - Feed the generated contact result into the full-case `contact_force`
    callback so the default route no longer depends on frozen contact input.
- Current remaining simplification:
  - The default route now uses the main rigid-wheel local kinematics, but the
    historical `ZP_Con.RelVel_max` memory used by Hu-Guo damping is still
    approximated by current-step gating rather than the full MATLAB history
    table.
- Validation:
  - Full-case diagnostic tests verify the accepted `Cal` step contact-force
    vector equals the direct composition of `wr_force_modal_ft` and
    `wr_force_vehicle_sys_rotation_iii` for the generated contact result.
  - The default driver now reports contact enabled with populated
    `wheel_rail_contact` diagnostics and non-zero `Pjcc`.
  - `tests/test_full_case_contact.py` locks the rigid wheelset DOF mapping and
    the default `Vjd / Vjd_r / Vsdc / Vjsdc` kinematics, including the `R2`
    rail-beam velocity branch.

### 7. Nonlinear Vehicle Dampers

- Status: implemented in `python/sditt/vehicle/nonlinear_dampers.py` and wired
  into `python/sditt/simulation/full_case.py`.
- MATLAB references:
  - `Judge_DamperNL.m`
  - `NonLinear_DampingForce_v4Re.m`
- Implemented behavior:
  - Update nonlinear damper states.
  - Rebuild the vehicle damping block with the active nonlinear branches.
  - Add nonlinear damper force terms into `Pxt`.
  - Feed the coupled driver an equivalent force correction
    `(C_vehicle_base - C_vehicle_nl) @ v + Pxt_nl` so the nonlinear branch is
    honored without mutating the stepper matrices mid-iteration.
- Validation:
  - `tests/test_vehicle_nonlinear_dampers.py` checks frozen-state
    `DamperNL`, switched damping diagonals, direct `Pxt` terms, and the
    equivalent-force identity.
  - `tests/test_full_case_driver.py` verifies the full driver includes the
    nonlinear correction in the accepted `Cal` step force vector.

### 8. Curve/Layout External Force

- Status for the default route: not triggered.
- MATLAB reference: `External_Force_Curve_210227_v4.m`
- Implemented/default-route behavior:
  - Carry `Type_Layout` through `DefaultOperatingCase.to_inp_par()`.
  - Match the MATLAB main script, which sets `InpPar.Type_Layout = 'Straight'`
    for the default `SDITT_CR400_NoStrTIrr_250728_Face.m` route.
  - Omit `curve_external_force` from `missing_stages` when the layout is
    straight, since the branch is unreachable for the default route.
- Validation:
  - `tests/test_readers.py` checks the default operating case exports
    `Type_Layout = Straight`.
  - `tests/test_full_case_driver.py` verifies the prepared default route no
    longer reports `curve_external_force` as missing, that strict mode accepts
    the straight default route, and that the non-default curve route still
    fails.

### 9. Iteration Storage And Output

- Completed stage: `iteration_storage_and_output`
- MATLAB references:
  - `Storage_Iteration_P1_220713.m`
  - `Storage_Iteration_P2_220713.m`
  - `Output_ZP_FW_230517.m`
- Needed behavior:
  - Reproduce MATLAB-compatible iteration storage for convergence checks and
    post-processing.
  - Store contact force, normal/tangential error, vehicle state, rail response,
    and dynamic outputs.
- Validation target:
  - Compare short-run output tables and convergence histories against MATLAB.
  - MATLAB helper: `python/matlab/export_iteration_output_baseline.m`
  - Optional Python regression:
    `SDITT_RUN_MATLAB_BASELINES=1 .venv/bin/pytest tests/test_full_case_driver.py -q`

## Recommended Implementation Order

1. Export a short MATLAB baseline case with frozen intermediate variables.
2. Implement `Gravity_Load_ModalFT`.
3. Assemble `Multi_Con_250812` from the existing Python contact helpers.
4. Add nonlinear vehicle damper updates.
5. Add curve/layout external force if the default route needs it.
6. Add MATLAB-compatible iteration storage and output.

## Development Rules For Future Turns

- Use `.venv/bin/python`, not system `python3`.
- Keep new Python code under `python/`.
- Prefer sparse matrices for full 2000 Hz FT-Modal runs.
- Keep `fail_on_missing_physics=True` failing until all missing physical stages
  are genuinely wired and tested.
- Add focused tests for each migrated MATLAB stage before connecting it to the
  full driver.
- Compare each stage against MATLAB exported data before moving to the next
  stage.

## Current Completion Checklist

- [x] Raw `.mat`, text, and profile readers.
- [x] CRH380A_v6 vehicle parameters and RW matrix builder.
- [x] FT-Modal track matrix builder.
- [x] Sparse full-system block assembly.
- [x] Park/Newmark linear integrator.
- [x] Coupled iteration skeleton.
- [x] Diagnostic full default case driver with `Preload -> Cal`.
- [x] Gravity preload.
- [x] Rail profile selection by mileage.
- [x] Beam shape functions.
- [x] Rail response recovery wired to the driver.
- [x] Vehicle and track wheel-rail force mapping wired to the driver.
- [ ] Full `Multi_Con_250812` contact routine.
- [x] Nonlinear vehicle dampers.
- [x] Curve/layout external force (default route verified not triggered).
- [x] MATLAB-compatible iteration storage and output.
