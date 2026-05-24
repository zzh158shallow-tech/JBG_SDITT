"""Validation helpers for Python/MATLAB migration checks."""

__all__ = [
    "build_full_case_short_run_report",
    "build_python_short_run_snapshot",
    "compare_short_run_snapshots",
    "write_full_case_short_run_report",
]


def __getattr__(name: str):
    if name in __all__:
        from . import full_case_short_run

        return getattr(full_case_short_run, name)
    raise AttributeError(name)
