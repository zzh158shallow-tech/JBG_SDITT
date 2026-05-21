from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ProjectPaths:
    """Filesystem locations used by the first-stage Python readers."""

    root: Path
    matlab_dir: Path
    profile_dir: Path

    @classmethod
    def from_repo_root(cls, root: str | Path | None = None) -> "ProjectPaths":
        repo_root = Path(root) if root is not None else Path(__file__).resolve().parents[3]
        repo_root = repo_root.resolve()
        matlab_dir = repo_root / "SDITT-RW-FT-250728"
        return cls(
            root=repo_root,
            matlab_dir=matlab_dir,
            profile_dir=matlab_dir / "WRProfile-07(009)-1_18",
        )

    @property
    def modal_turnout_mat(self) -> Path:
        return self.matlab_dir / "Mat_FT_S8b.mat"

    @property
    def default_vehicle_parameters(self) -> Path:
        return self.matlab_dir / "Par_Vehicle_CRH380A_v6.m"
