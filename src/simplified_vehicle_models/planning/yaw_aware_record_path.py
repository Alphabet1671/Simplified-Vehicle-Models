"""Export yaw-aware optimized-line samples as animation/run-record geometry."""
from __future__ import annotations

import numpy as np

from simplified_vehicle_models.planning.yaw_aware_line import YawAwareLinePlan
from simplified_vehicle_models.tracks.types import PathCoordinates


def _immutable(values) -> np.ndarray:
    result = np.array(values, dtype=float, copy=True)
    result.setflags(write=False)
    return result


def path_coordinates_from_yaw_line(plan: YawAwareLinePlan) -> PathCoordinates:
    """Sample the exact optimized trajectory at its physical speed-plan stations.

    ``SpeedPlan.distance`` is path arclength, not reference-curve station.
    Map it back through the trajectory's station map before evaluating analytic
    geometry; never reconstruct the optimized path from its discretized samples.
    """
    if not isinstance(plan, YawAwareLinePlan):
        raise TypeError("plan must be a YawAwareLinePlan")
    distance = np.asarray(plan.speed_plan.distance, dtype=float)
    if distance.ndim != 1 or distance.size < 2 or not np.all(np.isfinite(distance)):
        raise ValueError("plan speed stations must be a finite one-dimensional sequence")
    if np.any(np.diff(distance) <= 0.0):
        raise ValueError("plan speed stations must be strictly increasing")
    source = np.asarray(plan.trajectory.station_map.to_reference(distance), dtype=float)
    values = plan.trajectory.evaluate(source)
    return PathCoordinates(
        _immutable(distance),
        _immutable(values.north),
        _immutable(values.east),
        _immutable(np.unwrap(np.asarray(values.heading, dtype=float))),
        _immutable(values.curvature),
        is_closed=bool(plan.speed_plan.is_closed),
        reference_distance=_immutable(source),
    )
