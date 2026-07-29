"""Training-data preparation utilities for SDITT surrogate models."""

from importlib import import_module

__all__ = [
    "FEATURE_NAMES",
    "LABEL_NAMES",
    "MAX_PATCHES",
    "NetworkADataset",
    "generate_network_a_dataset",
    "load_network_a_dataset",
]


def __getattr__(name: str):
    if name in __all__:
        return getattr(import_module(".network_a", __name__), name)
    raise AttributeError(name)
