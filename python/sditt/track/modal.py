from __future__ import annotations

from pathlib import Path

from sditt.io.matlab import MatFile, MatSummary, load_mat_file, summarize_mat_file


def load_modal_turnout_data(path: str | Path) -> MatFile:
    """Load raw flexible turnout modal data from a MATLAB ``.mat`` file."""

    return load_mat_file(path)


def summarize_modal_turnout_data(path: str | Path) -> MatSummary:
    """Read variable names, shapes, and MATLAB classes without expanding arrays."""

    return summarize_mat_file(path)
