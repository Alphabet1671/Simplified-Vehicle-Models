"""Closed-loop validation of yaw-aware line plans with the forward driver.

This Phase-4a helper reuses :func:`solve_forward` and evaluates containment at
its accepted output states. The driven lateral error is added to the planned
signed offset in the track frame; positive offsets retain the track convention
(the path normal points right for the documented north/east frame).
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from simplified_vehicle_models.planning.yaw_aware_line import YawAwareLinePlan
from simplified_vehicle_models.solvers.forward_driver import (
    ForwardPlant, ForwardResult, PreviewDriver, solve_forward,
)


@dataclass(frozen=True, slots=True)
class YawAwareDriveResult:
    """Immutable result of driving and checking a yaw-aware plan.

    ``completed`` reports whether the forward solver reached the finish event.
    ``max_corridor_violation`` and ``sampled_center_contained`` summarize only
    accepted output states, using the center point (not a footprint); they do
    not establish between-step containment, periodicity, refinement, or full
    Phase-4 acceptance. ``driven_time`` and the comparison are ``None`` unless
    the finish event was reached. The station is physical path arclength.
    """
    forward_result: ForwardResult
    max_corridor_violation: float
    worst_violation_station: float
    completed: bool
    sampled_center_contained: bool
    predicted_time: float
    driven_time: float | None

    @property
    def predicted_vs_driven_time(self) -> float | None:
        """Return driven minus predicted duration, or None if incomplete."""
        if not self.completed or self.driven_time is None:
            return None
        return self.driven_time - self.predicted_time


def trajectory_corridor_containment(plan: YawAwareLinePlan, path_distance,
                                    lateral_error) -> tuple[float, float, bool]:
    """Measure sampled center-point containment at supplied accepted states.

    ``path_distance`` and ``lateral_error`` are equal-length vectors. Station
    mapping is performed via the plan's exact ``OffsetTrajectory.station_map``;
    no identity or resampled mapping between path and reference stations is
    assumed. Corridor bounds are signed reference-frame offsets. This sampled
    center-point check is not a footprint, between-step, or periodicity gate.
    """
    if not isinstance(plan, YawAwareLinePlan):
        raise TypeError("plan must be a YawAwareLinePlan")
    distance, error = np.asarray(path_distance, float), np.asarray(lateral_error, float)
    if (distance.ndim != 1 or error.shape != distance.shape or distance.size == 0
            or not np.all(np.isfinite(distance)) or not np.all(np.isfinite(error))):
        raise ValueError("distance and lateral_error must be matching finite vectors")
    station_map = plan.trajectory.station_map
    if not station_map.is_closed and (np.any(distance < station_map.path_start)
                                       or np.any(distance > station_map.path_end)):
        raise ValueError("driven path stations are outside the plan station map")
    reference_station = np.asarray(station_map.to_reference(distance), dtype=float)
    planned = np.asarray(plan.trajectory.evaluate(reference_station).offset, dtype=float)
    driven_offset = planned + error
    corridor = plan.corridor
    axis = np.asarray(corridor.distance, float)
    if corridor.is_closed:
        reference_station = np.mod(reference_station - axis[0], axis[-1] - axis[0]) + axis[0]
        reference_station = np.where(np.isclose(reference_station, axis[-1], rtol=0., atol=1e-12), axis[-1], reference_station)
    elif np.any(reference_station < axis[0]) or np.any(reference_station > axis[-1]):
        raise ValueError("plan reference stations are outside corridor coverage")
    lower = np.interp(reference_station, axis, corridor.minimum)
    upper = np.interp(reference_station, axis, corridor.maximum)
    signed_violation = np.maximum(lower - driven_offset, driven_offset - upper)
    index = int(np.argmax(signed_violation))
    maximum = max(0., float(signed_violation[index]))
    return maximum, float(distance[index]), maximum <= 1e-9


def drive_yaw_aware_plan(plan: YawAwareLinePlan, plant: ForwardPlant,
                         driver: PreviewDriver, initial_state, *,
                         duration=300., wall_seconds=290., maximum_step=0.02,
                         rtol=1.e-6, atol=1.e-8, method="Radau", **forward_options
                         ) -> YawAwareDriveResult:
    """Drive a yaw-aware line and check sampled center-point containment.

    ``completed`` distinguishes reaching the forward solver's finish event from
    a duration-limited partial run. Only completed runs report a driven lap
    time or predicted-versus-driven delta. This helper does not evaluate
    periodicity, refinement, between-step containment, or vehicle footprint.

    The provided driver must track ``plan.speed_plan`` (normally by
    construction). Additional supported ``solve_forward`` keyword options,
    such as a command mapper, are forwarded unchanged.
    """
    if not isinstance(plan, YawAwareLinePlan):
        raise TypeError("plan must be a YawAwareLinePlan")
    if not isinstance(driver, PreviewDriver) or driver.plan is not plan.speed_plan:
        raise ValueError("driver must use this plan's exact SpeedPlan")
    forward = solve_forward(plant, driver, initial_state, duration=duration,
                            wall_seconds=wall_seconds, maximum_step=maximum_step,
                            rtol=rtol, atol=atol, method=method, **forward_options)
    observations = [plant.observe(state) for state in forward.state.T]
    distance = np.asarray([item.distance for item in observations], float)
    lateral_error = np.asarray([item.lateral_error for item in observations], float)
    violation, station, contained = trajectory_corridor_containment(plan, distance, lateral_error)
    completed = bool(forward.lap_completed)
    driven_time = float(forward.time[-1]) if completed else None
    predicted_time = float(plan.profile.lap_time)
    if not math.isfinite(predicted_time) or predicted_time <= 0.:
        raise ValueError("plan predicted time must be finite and positive")
    return YawAwareDriveResult(forward, violation, station, completed,
                              contained, predicted_time, driven_time)
