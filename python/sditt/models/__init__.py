"""Trainable surrogate models for SDITT."""

from importlib import import_module

__all__ = [
    "WRCPNetA1",
    "WRCPNetA1Prediction",
    "load_wrcp_net_a1",
    "predict_wrcp_net_a1",
    "train_wrcp_net_a1",
    "WRCPNetA2G",
    "WRCPNetA2GPrediction",
    "load_wrcp_net_a2g",
    "predict_wrcp_net_a2g",
    "train_wrcp_net_a2g",
    "PenetrationResidualHeads",
    "apply_penetration_residuals",
    "build_runtime_teacher_dataset",
    "load_penetration_residual_heads",
    "train_penetration_residual_heads",
    "NetworkAContactGeometryAdapter",
    "NetworkA1DirectContactGeometryAdapter",
    "WRCPNetA1DirectSet",
    "WRCPNetA1DirectPrediction",
    "load_wrcp_net_a1_direct",
    "save_wrcp_net_a1_direct",
]


def __getattr__(name: str):
    if name == "NetworkAContactGeometryAdapter":
        return getattr(import_module(".network_a_full_case", __name__), name)
    if name == "NetworkA1DirectContactGeometryAdapter":
        return getattr(import_module(".network_a1_direct_full_case", __name__), name)
    if name in {
        "WRCPNetA1DirectSet",
        "WRCPNetA1DirectPrediction",
        "load_wrcp_net_a1_direct",
        "save_wrcp_net_a1_direct",
    }:
        return getattr(import_module(".wrcp_net_a1_direct", __name__), name)
    if name.endswith("A2G") or name.endswith("a2g"):
        return getattr(import_module(".wrcp_net_a2g", __name__), name)
    if name in {
        "PenetrationResidualHeads",
        "apply_penetration_residuals",
        "build_runtime_teacher_dataset",
        "load_penetration_residual_heads",
        "train_penetration_residual_heads",
    }:
        return getattr(import_module(".wrcp_net_a2r", __name__), name)
    if name in __all__:
        return getattr(import_module(".wrcp_net_a1", __name__), name)
    raise AttributeError(name)
