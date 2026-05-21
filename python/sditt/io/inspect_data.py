from __future__ import annotations

import argparse
from pathlib import Path

from sditt.config import ProjectPaths
from sditt.io.matlab import summarize_mat_file
from sditt.profiles import discover_profile_files, load_profile_file
from sditt.vehicle import load_vehicle_parameters


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect first-stage SDITT input readers.")
    parser.add_argument("--root", type=Path, default=None, help="Repository root.")
    args = parser.parse_args()

    paths = ProjectPaths.from_repo_root(args.root)
    mat_summary = summarize_mat_file(paths.modal_turnout_mat)
    vehicle = load_vehicle_parameters(paths.default_vehicle_parameters)
    profile_files = discover_profile_files(paths.profile_dir)
    sample_profile = load_profile_file(profile_files[0]) if profile_files else None

    print(f"MAT: {mat_summary.path}")
    print(f"  variables: {len(mat_summary.variables)}")
    for name, summary in list(mat_summary.variables.items())[:10]:
        print(f"  - {name}: {summary}")
    print(f"Vehicle: {vehicle.path}")
    print(f"  parsed parameters: {len(vehicle.values)}")
    print(f"  unsupported statements: {len(vehicle.unsupported_statements)}")
    print(f"Profiles: {paths.profile_dir}")
    print(f"  files: {len(profile_files)}")
    if sample_profile is not None:
        print(f"  sample: {sample_profile.path.name}, points={sample_profile.points.shape}")


if __name__ == "__main__":
    main()
