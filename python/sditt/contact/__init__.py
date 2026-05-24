"""Wheel-rail contact geometry helpers."""

from .geometry import (
    BoundaryExtrema,
    ContactPatch,
    MultiPointContactGeometry,
    SinglePointContact,
    WheelPose2D,
    WheelTrace,
    extreme_boundary,
    multi_point_contact_geometry,
    quasi_elastic_correction,
    single_point_contact_geometry,
    trace_wheel_profile,
)
from .forces import (
    KalkerTangentialForceResult,
    StripePatchResult,
    StripesNormalForceResult,
    add_hu_guo_stripes_damping,
    contact_to_track_matrix,
    hertz_normal_force,
    hu_guo_normal_damping_force,
    kalker_linear_saturated_creep_force,
    normal_damping_window,
    stripes_normal_force,
)
from .full_case import (
    DefaultTrackContactParameters,
    FullCaseWheelRailContactResult,
    solve_default_wheel_rail_contact,
)

__all__ = [
    "BoundaryExtrema",
    "ContactPatch",
    "DefaultTrackContactParameters",
    "FullCaseWheelRailContactResult",
    "KalkerTangentialForceResult",
    "MultiPointContactGeometry",
    "SinglePointContact",
    "StripePatchResult",
    "StripesNormalForceResult",
    "WheelPose2D",
    "WheelTrace",
    "add_hu_guo_stripes_damping",
    "contact_to_track_matrix",
    "extreme_boundary",
    "hertz_normal_force",
    "hu_guo_normal_damping_force",
    "kalker_linear_saturated_creep_force",
    "multi_point_contact_geometry",
    "normal_damping_window",
    "quasi_elastic_correction",
    "single_point_contact_geometry",
    "solve_default_wheel_rail_contact",
    "stripes_normal_force",
    "trace_wheel_profile",
]
