"""Input/output readers for MATLAB and text-based SDITT data."""

from .matlab import MatFile, MatSummary, load_mat_file, load_mat_variables, summarize_mat_file
from .text import NumericTextData, load_numeric_text

__all__ = [
    "MatFile",
    "MatSummary",
    "NumericTextData",
    "RawInputManifest",
    "SDITTRawInputs",
    "inspect_raw_inputs",
    "load_mat_file",
    "load_mat_variables",
    "load_numeric_text",
    "load_sditt_raw_inputs",
    "summarize_mat_file",
]


def __getattr__(name: str):
    if name in {
        "RawInputManifest",
        "SDITTRawInputs",
        "inspect_raw_inputs",
        "load_sditt_raw_inputs",
    }:
        from .dataset import (
            RawInputManifest,
            SDITTRawInputs,
            inspect_raw_inputs,
            load_sditt_raw_inputs,
        )

        return {
            "RawInputManifest": RawInputManifest,
            "SDITTRawInputs": SDITTRawInputs,
            "inspect_raw_inputs": inspect_raw_inputs,
            "load_sditt_raw_inputs": load_sditt_raw_inputs,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
