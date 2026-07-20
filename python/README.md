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

Operating-case layouts:

```python
from sditt.config import DEFAULT_OPERATING_CASE, MATLAB_FULL_DEFAULT_CASE

interval_inp_par = DEFAULT_OPERATING_CASE.to_inp_par()
turnout_inp_par = MATLAB_FULL_DEFAULT_CASE.to_inp_par()
```

`DEFAULT_OPERATING_CASE` uses two constant basic-rail contact slots, `L1/R1`.
`MATLAB_FULL_DEFAULT_CASE` preserves the original turnout `L1/R1/R2/R3`
contact layout aligned with `SDITT_CR400_NoStrTIrr_250728_Face.m`.

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

This builds the default `07(009)` / `Face` / `350 km/h` /
`FT-Modal` / `CRH380A_v6` system and executes the two-stage
`Preload -> Cal` coupled loop with the modal FT gravity preload
applied as the baseline external force. The default contact selector uses the
same constant measured basic-rail section on both sides. The matrices, gravity
preload, and structural dynamics still come from the flexible-turnout model,
so this is an interval contact-geometry surrogate rather than a complete
interval track dynamics model.

The original turnout contact route remains available from the command line:

```bash
python -m sditt.validation.full_case_short_run --rail-layout turnout --steps 2 --cut-freq 50
```

Track irregularity is off by default so existing baselines remain unchanged.
Enable the Chinese high-speed ballastless-track spectrum with a reproducible
random-phase trigonometric-series reconstruction as follows:

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --track-irregularity china-ballastless \
  --irregularity-seed 20260716 \
  --full-size \
  --matlab-mileage-endpoints \
  --save-progress
```

The model synthesizes vertical profile (高低), alignment (轨向), cross level
(水平), and gauge (轨距) over wavelengths 2–200 m. It applies both displacement
and the corresponding velocity to wheel–rail contact. With `--save-progress`,
the `progress/` directory also contains `track_irregularity.csv`,
`track_irregularity_spectrum.csv`, and `track_irregularity.svg`.

## Network A training data

The interval-layout contact-geometry surrogate dataset uses one accepted
time-step / wheelset / wheel-side environment per sample. Generate the
2,000-sample pilot first, then extend the same output to the 20,000-sample
production dataset:

```bash
python -m sditt.training_data.network_a generate --mode pilot
python -m sditt.training_data.network_a generate --mode production
```

Outputs are written under `outputs/network_a_dataset_v1/` and are ignored by
Git. The production dataset contains 6,000 accepted coupled samples and 14,000
parameter samples: 7,600 ordinary contact, 800 targeted flange-related
multi-contact, 3,500 contact/separation boundary, and 2,100 no-contact samples.
The targeted samples use Sobol perturbations around profile-specific discovery
anchors and are accepted only after the traditional geometry teacher confirms
two separated contact patches with at least one flange-angle patch. All samples
are grouped into `14,000 / 3,000 / 3,000` train/validation/test splits. Inspect
the manifest and quality results with:

```bash
python -m sditt.training_data.network_a inspect outputs/network_a_dataset_v1
```

Programmatic loading converts floating arrays to `float32` by default while
preserving masks and metadata:

```python
from sditt.training_data import load_network_a_dataset

train = load_network_a_dataset("outputs/network_a_dataset_v1", split="train")
```

## WRCP-Net A1 training

`WRCP-Net A1` means Wheel–Rail Contact Point Network A1 (轮轨接触点网络A1).
It is the first network-A baseline: a dependency-free NumPy multi-task MLP
that predicts the contact-patch count and two masked patch-label slots. Train
and inspect it from the `python/` directory:

```bash
python -m sditt.models.wrcp_net_a1 train
python -m sditt.models.wrcp_net_a1 inspect outputs/wrcp_net_a1
```

The ignored `outputs/wrcp_net_a1/` directory contains the compressed model,
training history, validation/test metrics, test predictions, and provenance
manifest. The first version uses class-weighted cross entropy for contact state
and masked Huber loss (胡贝尔损失，兼顾平方误差与异常值鲁棒性) for continuous
patch labels, with additional weight on the second contact slot.

## WRCP-Net A2G geometry-aware training

`WRCP-Net A2G` is the geometry-aware successor. It predicts a 257-point
canonical gap/penetration field and the contact topology, then reconstructs
patch boundaries, wheel/rail points, penetration and contact angle from the
fixed profiles. When the predicted topology agrees with the native profile
topology, a deterministic profile-consistency refinement removes compact-grid
quantisation from the final coordinates without overriding the predicted
contact class. Generate the field cache and train in one command:

```bash
python -m sditt.models.wrcp_net_a2g train
python -m sditt.models.wrcp_net_a2g inspect outputs/wrcp_net_a2g
```

The field cache is written to `outputs/network_a_gap_field_v1/`; the trained
model and comparison metrics are written to `outputs/wrcp_net_a2g/`. Both are
ignored by Git.

Use A2G as a strict replacement for the traditional multi-point geometry
search in the interval full case with:

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --contact-geometry-mode network-a \
  --network-a-model outputs/wrcp_net_a2g/model.npz \
  --steps 2 \
  --cut-freq 50
```

This mode has no traditional-geometry fallback. A predicted/reconstructed
patch-count mismatch, non-finite geometry, a missing model, or a non-interval
layout raises an error. STRIPES/Hertz/Kalker force calculation and the coupled
dynamics remain unchanged after A2G supplies the contact geometry.

To establish the preload equilibrium with traditional geometry and switch to
strict A2G geometry only for the subsequent `Cal` stage, use:

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --contact-geometry-mode network-a-after-preload \
  --network-a-model outputs/wrcp_net_a2g/model.npz \
  --network-a-trace-dir outputs/network_a_runtime_trace \
  --track-irregularity china-ballastless \
  --irregularity-seed 20260716 \
  --steps 2 \
  --cut-freq 50
```

The optional trace directory contains compressed per-iteration shards and
`accepted_steps.jsonl`. Repeated nonlinear iterations and reduced-`dt` retries
are retained as diagnostic candidate states. They are not teacher labels: join
accepted keys as needed and rerun the traditional geometry teacher before
promoting selected rows into a future training dataset.

## WRCP-Net A2R continuity refinement

`WRCP-Net A2R` adds independent left/right bounded penetration-residual heads,
accepted-step continuity state, contact-branch hysteresis, and candidate-region
fixed-profile interpolation to the frozen A2G gap-field model. Build its v2
teacher dataset and micron-scale perturbation pairs, then train with higher
weights for high-iteration, reduced-`dt`, left-side, and accepted states:

```bash
python -m sditt.models.wrcp_net_a2r build-dataset \
  --trace outputs/network_a_runtime_trace_irregularity_full_seed20260716 \
  --output outputs/network_a_runtime_teacher_v2_continuity \
  --repo-root .. \
  --base-model outputs/wrcp_net_a2g/model.npz

python -m sditt.models.wrcp_net_a2r train \
  --dataset outputs/network_a_runtime_teacher_v2_continuity \
  --base-model outputs/wrcp_net_a2g/model.npz \
  --output outputs/wrcp_net_a2r_continuity \
  --left-residual-gain 1.0 \
  --right-residual-gain 1.0 \
  --smooth-weight 0.25 \
  --rate-reference-dataset outputs/network_a_dataset_v1/all_samples.npz
```

The deployed continuity envelope uses the 99.5th percentile of adjacent
traditional coupled steps. Ordinary changes pass through unchanged; only
out-of-envelope changes activate `alpha=0.3` under-relaxation and rate clipping.
Every nonlinear retry remains anchored to the preceding accepted step, and
history is committed only after the integrator accepts the step. The model is
still an A2G-compatible artifact and introduces no traditional fallback.

Run it after traditional preload with a new v2 trace directory:

```bash
python -m sditt.validation.full_case_short_run \
  --rail-layout interval \
  --contact-geometry-mode network-a-after-preload \
  --network-a-model outputs/wrcp_net_a2r_continuity/model.npz \
  --network-a-trace-dir outputs/network_a_runtime_trace_a2r_continuity \
  --track-irregularity china-ballastless \
  --irregularity-seed 20260716 \
  --steps 200 \
  --cut-freq 50 \
  --save-progress
```

The v2 runtime trace separately records base penetration, bounded residual,
slew-limited residual, network penetration, final used penetration, branch
hysteresis, and continuity-limit flags. Do not append v2 records to a v1 trace
directory.

For the straight-layout route, the current driver evaluates the
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
