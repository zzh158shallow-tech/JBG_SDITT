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

It does not yet perform wheel-rail contact, time integration, or full coupled
vehicle-track simulation.

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

Install the package in editable mode from this directory when using a fresh
Python environment:

```bash
python -m pip install -e ".[test]"
```
