# Python Code

Place all Python code for this project in this directory.

This includes new Python scripts, modules, packages, notebooks converted to
Python files, and future generated Python code.

## Phase 1 Layout

- `sditt/config`: project path configuration.
- `sditt/io`: MATLAB `.mat` and numeric text readers.
- `sditt/vehicle`: MATLAB vehicle-parameter script reader.
- `sditt/track`: raw turnout/track data readers.
- `sditt/profiles`: wheel/rail profile discovery and loading.
- `sditt/contact`, `sditt/integrators`, `sditt/simulation`: reserved package
  namespaces for later physical-model migration.
- `tests`: reader-level tests.

## Reader Scope

The current code only reads source data. It does not perform wheel-rail contact,
vehicle dynamics, track dynamics, integration, or simulation calculations.

Useful smoke check:

```bash
python -m sditt.io.inspect_data
```

Install the package in editable mode from this directory when using a fresh
Python environment:

```bash
python -m pip install -e ".[test]"
```
