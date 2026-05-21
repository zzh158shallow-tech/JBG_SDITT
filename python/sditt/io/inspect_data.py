from __future__ import annotations

import argparse
from pathlib import Path

from sditt.config import ProjectPaths
from sditt.io.dataset import inspect_raw_inputs, load_sditt_raw_inputs


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect first-stage SDITT input readers.")
    parser.add_argument("--root", type=Path, default=None, help="Repository root.")
    args = parser.parse_args()

    paths = ProjectPaths.from_repo_root(args.root)
    manifest = inspect_raw_inputs(paths.root)
    raw_sample = load_sditt_raw_inputs(paths.root, max_profiles_per_kind=1)

    print(f"MAT: {manifest.modal_turnout_summary.path}")
    print(f"  variables: {len(manifest.modal_turnout_summary.variables)}")
    print(f"  ModeFreq.FT_All: {manifest.modal_frequencies_shape}")
    for name, summary in list(manifest.modal_turnout_summary.variables.items())[:10]:
        print(f"  - {name}: {summary}")
    print(f"Vehicle: {raw_sample.vehicle_parameters.path}")
    print(f"  parsed parameters: {len(raw_sample.vehicle_parameters.values)}")
    print(f"  unsupported statements: {len(raw_sample.vehicle_parameters.unsupported_statements)}")
    print(f"Profiles: {paths.profile_dir}")
    print(f"  files: {len(manifest.profile_files)}")
    print(f"  wheel files: {len(manifest.wheel_profile_files)}")
    print(f"  rail files: {len(manifest.rail_profile_files)}")
    for name, profiles in (("wheel", raw_sample.wheel_profiles), ("rail", raw_sample.rail_profiles)):
        if profiles:
            path, profile = next(iter(profiles.items()))
            print(f"  sample {name}: {path}, points={profile.points.shape}")
    print(f"Numeric text tables: {len(manifest.numeric_text_files)}")


if __name__ == "__main__":
    main()
