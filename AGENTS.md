# Project Memory

## Python Code Location

- All existing and future Python code for this project should live under
  `python/`.
- If new Python scripts, modules, packages, or generated `.py` files are
  created, place them in `python/` unless the user explicitly asks for a
  different location.
- The Python package root is `python/sditt`.
- The Python project metadata is `python/pyproject.toml`; install from inside
  `python/` with editable mode.

## Fresh Machine Setup Through GitHub

Use this section when Codex or a developer is setting up the project on a new
computer from GitHub.

1. Clone the private repository:
   - `git clone https://github.com/zzh158shallow-tech/JBG_SDITT.git JBG_SDITT`
   - `cd JBG_SDITT`
2. Configure the GitHub SSH key if the machine uses the project key:
   - `git config core.sshCommand "ssh -i ~/.ssh/id_ed25519_github -o IdentitiesOnly=yes"`
3. Create and activate a Python virtual environment:
   - `cd python`
   - `python3 -m venv .venv`
   - `source .venv/bin/activate`
4. Install the project and test dependencies:
   - `python -m pip install --upgrade pip`
   - `python -m pip install -e ".[test]"`
5. Return to the repository root for commands that expect repo-relative data:
   - `cd ..`

If `tkinter` is missing on the new computer, the realtime window cannot open.
The non-GUI validation commands still work.

## Runtime Data Placement On A Fresh Machine

GitHub contains source code, not the large private MATLAB/model data. After
cloning, copy the private runtime data folder into the repository root so paths
match this layout:

- `SDITT/SDITT-RW-FT-250728/Mat_FT_S8b.mat`
- `SDITT/SDITT-RW-FT-250728/Mat_FT_CN18_T4.mat`
- `SDITT/SDITT-RW-FT-250728/Pre_CRH380A_009_Face_V350_zgygPf2_*.mat`
- `SDITT/SDITT-RW-FT-250728/CRH380A_009_Face_V350_zgygPf2.mat`
- `SDITT/SDITT-RW-FT-250728/WRProfile-07(009)-1_18/...`

Do not try to fetch these large/private files from GitHub; they are ignored on
purpose. If the new machine lacks them, profile reading and full FT-Modal runs
will fail even though the source code is installed correctly.

## Current Supported Operating Cases

- The default Python route uses `interval` contact geometry: left and right are
  the same constant standard basic-rail section, represented by `L1 + R1`.
- This is currently a contact-geometry variant only. It still uses the
  `07(009)` flexible-turnout FT-Modal matrices, gravity preload, and structural
  dynamics as a surrogate; it is not yet a complete interval track model.
- The original `07(009)` turnout contact route remains available with
  `--rail-layout turnout`. Its right profile is the combined `R1 + R2 + R3`
  profile matching MATLAB `Get_Profile_P2_v2.m`.
- The realtime profile display is intentionally stabilized on the front
  wheelset (`FF`) so the displayed mileage/profile does not jump between
  wheelsets.
- The realtime force display defaults to the first wheelset only. Interval mode
  shows `FF-L1` and `FF-R1`; turnout mode also shows `FF-R2` and `FF-R3`.
  The progress CSV still records all patch force columns.

## Common Run Commands

Run commands from the repository root unless noted otherwise. If using the
virtual environment from the setup section, activate it first:

- Short validation run:
  - `cd python`
  - `python -m sditt.validation.full_case_short_run --steps 2 --cut-freq 50`
- Original turnout validation run:
  - `cd python`
  - `python -m sditt.validation.full_case_short_run --rail-layout turnout --steps 2 --cut-freq 50`
- Realtime window short run:
  - `cd python`
  - `python -m sditt.validation.full_case_short_run --live-window --steps 2 --cut-freq 50`
- Full MATLAB-mileage endpoint run with realtime window:
  - `cd python`
  - `python -m sditt.validation.full_case_short_run --live-window --full-size --matlab-mileage-endpoints`
- Full MATLAB-mileage endpoint run with realtime window, final progress output,
  and run checkpoints:
  - Windows PowerShell:
    - `cd G:\codex_program\sditt_py\python`
    - ```powershell
      .\.venv\Scripts\python.exe -m sditt.validation.full_case_short_run `
        --live-window `
        --full-size `
        --matlab-mileage-endpoints `
        --save-progress `
        --checkpoint-dir G:\codex_program\sditt_py\python\outputs\full_case_short_run\checkpoints `
        --no-resume-checkpoint
      ```
  - macOS/Linux shell:
    - `cd /path/to/SDITT/python`
    - ```bash
      ./.venv/bin/python -m sditt.validation.full_case_short_run \
        --live-window \
        --full-size \
        --matlab-mileage-endpoints \
        --save-progress \
        --checkpoint-dir /path/to/SDITT/python/outputs/full_case_short_run/checkpoints \
        --no-resume-checkpoint
      ```
  - Replace `--no-resume-checkpoint` with `--resume-checkpoint` to continue
    from the latest matching checkpoint.
- Save final progress CSV/SVG without GUI:
  - `cd python`
  - `python -m sditt.validation.full_case_short_run --save-progress --steps 2 --cut-freq 50`
- Export combined right rail `R1 + R2 + R3` 3D/top-view profiles:
  - `python python/scripts/export_combined_right_profile_3d.py`
- Plot mileage profile top-view helper:
  - `python python/scripts/plot_mileage_profiles_3d.py`

Generated outputs are intentionally ignored under `python/outputs/` and
`python/python/outputs/`.

## Useful Verification Commands

- Syntax check changed Python files:
  - `python -m py_compile <file.py>`
- Run the test suite from inside `python/`:
  - `python -m pytest`
- If the global `python3` cannot import `scipy` or `pytest`, use the virtual
  environment created above.

## GitHub Push Target

- This project is a private GitHub repository.
- Default remote: `origin`
- Remote URL: `https://github.com/zzh158shallow-tech/JBG_SDITT.git`
- Default branch: `main`
- Use the local SSH key at `~/.ssh/id_ed25519_github` for GitHub access.
- The repo is configured with:
  - `core.sshCommand=ssh -i ~/.ssh/id_ed25519_github -o IdentitiesOnly=yes`

## Normal Push Workflow

1. Check changes:
   - `git status -sb`
2. Commit intentionally:
   - `git add <files>`
   - `git commit -m "<message>"`
3. Push to GitHub:
   - `git push -u origin main`

Do not force-push unless the user explicitly asks for it.
Before committing, avoid staging runtime outputs, caches, private `.mat` files,
or generated `__pycache__` files. Prefer targeted `git add <files>` over broad
adds.

## Large And Private Data

Large runtime data files are intentionally ignored and should not be committed
to GitHub unless the user explicitly asks to set up a separate storage strategy
such as Git LFS, private object storage, or an external transfer method.

Known ignored examples include:

- `SDITT-RW-FT-250728/Mat_FT_S8b.mat`
  - Flexible turnout modal/input data for the FT-Modal track model.
  - MATLAB v5 `.mat`, about 984 MB, with top-level variables such as
    `DOF_Node`, `ModeFreq`, `ModeShape`, `ModeShape_Mapping`, `N_Node`,
    `Pos_Node`, and `Type_SpaceIron`.
  - Treat as large private model/runtime data, not source code.
- `SDITT-RW-FT-250728/Mat_FT_CN18_T4.mat`
- `SDITT-RW-FT-250728/Pre_CRH380A_009_Face_V350_zgygPf2_*.mat`
- `SDITT-RW-FT-250728/CRH380A_009_Face_V350_zgygPf2.mat`
