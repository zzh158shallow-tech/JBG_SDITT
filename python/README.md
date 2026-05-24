# Python Code

Place all Python code for this project in this directory.

This includes new Python scripts, modules, packages, notebooks converted to
Python files, and future generated Python code.

## Layout

- `sditt/config`: project path configuration.
- `sditt/io`: MATLAB `.mat` and numeric text readers.
- `sditt/vehicle`: MATLAB vehicle-parameter reader and CRH380A vehicle matrix
  reproduction.
- `sditt/track`: raw turnout/track data readers.
- `sditt/profiles`: wheel/rail profile discovery and loading.
- `sditt/contact`, `sditt/integrators`, `sditt/simulation`: reserved package
  namespaces for later physical-model migration.
- `tests`: reader-level tests.

## Scope

The current code reads source data and reproduces the linear vehicle dynamics
and flexible turnout modal matrix skeletons from:

- `Par_Vehicle_CRH380A_v6.m`
- `Matrix_Vehicle_RW_230409.m`
- `Matrix_Modal_FT_230313.m`

It now includes the first linear time-integration loop without wheel-rail
contact. It does not yet perform wheel-rail contact or full coupled
vehicle-track simulation.

The migration handoff plan for the full default coupled model is in
[`docs/full_case_migration_plan.md`](docs/full_case_migration_plan.md).

Useful smoke check for the raw MATLAB/text/profile data layer:

```bash
python -m sditt.io.inspect_data
```

Programmatic entry point:

```python
from sditt.io import inspect_raw_inputs, load_sditt_raw_inputs

manifest = inspect_raw_inputs()
raw = load_sditt_raw_inputs(max_profiles_per_kind=2)
```

This reads/summarizes `Mat_FT_S8b.mat`, loads `ModeFreq.FT_All`, parses
`Par_Vehicle_CRH380A_v6.m`, discovers wheel/rail profile files, and loads
numeric text tables such as damping-ratio and mileage files. It is intentionally
only a data layer.

MATLAB full default operating case:

```python
from sditt.config import MATLAB_FULL_DEFAULT_CASE

case = MATLAB_FULL_DEFAULT_CASE
inp_par = case.to_inp_par()
```

This aligns the Python defaults with `SDITT_CR400_NoStrTIrr_250728_Face.m`:
`07(009)`, `Face`, `350 km/h`, `FT-Modal`, `STRIPES&ConDamp`, Hu-Guo contact
damping coefficient `0.83`, Park integration, `CRH380A_v6`, `NM_FW = 0`,
`Type_Layout = Straight`, and the MATLAB two-stage loop `Preload` then `Cal`.

Build and export the CRH380A vehicle matrices:

```bash
python -m sditt.vehicle.export_matrices vehicle_matrices_rw_230409.npz
```

The archive stores both Python-facing names (`M_vehicle`, `K_vehicle`,
`C_vehicle`) and MATLAB-compatible names (`Mlc`, `Klc`, `Clc`, `Clc_0`).

Build and export the flexible turnout modal matrices:

```bash
python -m sditt.track.export_matrices modal_track_matrices_ft_230313.npz
```

The archive stores `M_track`, `K_track`, `C_track`, `DR`, `ModeFreq_FT`, and
`omega`. With the default `--cut-freq 2000`, the dense modal matrices are
`6197 x 6197`.

Build and export the uncoupled full system block matrices:

```bash
python -m sditt.simulation.export_system_matrices system_matrices_modal_rw.npz
```

This follows the main-script ordering
`[track DOFs, flexible wheel DOFs, rigid vehicle DOFs]`. For the current RW
vehicle route, `nm_fw = 0`, so `Mxt/Kxt/Cxt` are the track block followed
directly by the 51-DOF vehicle block.

For large FT-Modal runs, prefer the sparse builder when a downstream solver can
consume SciPy sparse matrices:

```python
from sditt.simulation import build_default_sparse_modal_rw_system_matrices

system, track, vehicle = build_default_sparse_modal_rw_system_matrices()
Mxt = system.Mxt
Kxt = system.Kxt
Cxt = system.Cxt
```

The sparse route keeps modal track `M/K/C` as diagonal sparse matrices and
assembles the full system with sparse block diagonals instead of dense zero
blocks.

Run the first no-contact forced-response loop:

```python
from sditt.simulation import run_default_no_contact_smoke

result = run_default_no_contact_smoke(cut_freq=50.0, dt=1e-4, n_steps=100)
time = result.history.time
displacement = result.history.displacement
velocity = result.history.velocity
acceleration = result.history.acceleration
force = result.force
```

This uses a simple prescribed harmonic force and advances the uncoupled
vehicle-track system with the Park/Newmark integrator. It does not compute
wheel-rail contact forces.

Run the current full-default driver:

```python
from sditt.simulation import FullDefaultCaseSettings, run_default_full_case_driver

result = run_default_full_case_driver(
    settings=FullDefaultCaseSettings(
        cut_freq=50.0,
        dt=1e-4,
        n_steps_per_stage=2,
    )
)
stages = result.stages
missing = result.preparation.missing_stages
```

This builds the MATLAB default `07(009)` / `Face` / `350 km/h` /
`FT-Modal` / `CRH380A_v6` system and executes the two-stage
`Preload -> Cal` coupled loop with the modal FT gravity preload
applied as the baseline external force and the default `07(009)` / `Face`
profile selector available through `result.preparation.profile_selector`.
For the MATLAB default straight-layout route, the current driver evaluates the
CRH380A_v6 nonlinear vehicle damper stage, the default rigid-wheel contact
route, and the MATLAB-style iteration/output storage path. For this straight
FT-Modal route, `missing_stages` is expected to be empty; the remaining
guarded branch is the non-default curve/layout external-force stage, which the
main MATLAB default case does not enter because `Type_Layout = Straight`.

Install the package in editable mode from this directory when using a fresh
Python environment:

```bash
python -m pip install -e ".[test]"
```
