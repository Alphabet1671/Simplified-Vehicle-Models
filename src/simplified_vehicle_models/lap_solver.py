from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np
import numpy.typing as npt

from simplified_vehicle_models.parameters import PointMassVehicleParameters
from simplified_vehicle_models.point_mass import (
    acceleration_limits,
    maximum_corner_speed,
)
from simplified_vehicle_models.tracks import TrackProfile


FloatArray = npt.NDArray[np.float64]


class LapMode(Enum):
    """Identify the boundary condition for a lap."""

    FLYING = "flying"
    STANDING_START = "standing_start"


@dataclass(frozen=True, slots=True)
class LapResult:
    """Store the calculated lap state and control traces."""

    speed: FloatArray
    elapsed_time: FloatArray
    longitudinal_acceleration: FloatArray
    lateral_acceleration: FloatArray
    throttle: FloatArray
    brake: FloatArray
    lap_time: float
    converged: bool
    iterations: int
    maximum_speed_change: float


def solve_lap(
    vehicle: PointMassVehicleParameters,
    track: TrackProfile,
    mode: LapMode,
    tolerance: float = 1.0e-6,
    maximum_iterations: int = 200,
) -> LapResult:
    """Calculate a minimum-time speed profile on a prescribed path."""

    if tolerance <= 0.0 or not math.isfinite(tolerance):
        raise ValueError("tolerance must be finite and positive")
    if maximum_iterations < 1:
        raise ValueError("maximum_iterations must be positive")
    if len(track.distance) != len(track.radius):
        raise ValueError("track distance and radius counts must match")
    if len(track.segment_length) != len(track.distance) - 1:
        raise ValueError("track segment count must be one less than node count")

    speed_limit = np.asarray(
        [maximum_corner_speed(vehicle, float(radius)) for radius in track.radius],
        dtype=np.float64,
    )
    speed = speed_limit.copy()
    if mode is LapMode.FLYING:
        _apply_periodic_boundary(speed)
    elif mode is LapMode.STANDING_START:
        speed[0] = 0.0
    else:
        raise ValueError(f"unsupported lap mode {mode!r}")

    converged = False
    maximum_change = math.inf
    iterations = 0
    for iterations in range(1, maximum_iterations + 1):
        previous = speed.copy()
        _forward_sweep(vehicle, track, speed)
        if mode is LapMode.FLYING:
            _apply_periodic_boundary(speed)
        else:
            speed[0] = 0.0
        _backward_sweep(vehicle, track, speed)
        if mode is LapMode.FLYING:
            _apply_periodic_boundary(speed)
        else:
            speed[0] = 0.0
        np.minimum(speed, speed_limit, out=speed)
        maximum_change = float(np.max(np.abs(speed - previous)))
        if maximum_change <= tolerance:
            converged = True
            break

    return _build_result(
        vehicle,
        track,
        speed,
        converged,
        iterations,
        maximum_change,
    )


def _apply_periodic_boundary(speed: FloatArray) -> None:
    boundary_speed = min(float(speed[0]), float(speed[-1]))
    speed[0] = boundary_speed
    speed[-1] = boundary_speed


def _forward_sweep(
    vehicle: PointMassVehicleParameters,
    track: TrackProfile,
    speed: FloatArray,
) -> None:
    for index, segment_length in enumerate(track.segment_length):
        start_speed = float(speed[index])
        upper_speed = float(speed[index + 1])
        if upper_speed <= start_speed:
            continue
        segment_curvature = max(
            abs(float(track.curvature[index])),
            abs(float(track.curvature[index + 1])),
        )

        candidate = upper_speed
        for _ in range(6):
            limits = acceleration_limits(
                vehicle,
                0.5 * (start_speed + candidate),
                segment_curvature,
            )
            reachable = math.sqrt(
                start_speed * start_speed
                + 2.0 * limits.maximum_acceleration * segment_length
            )
            updated = min(candidate, reachable)
            if abs(updated - candidate) <= 1.0e-7:
                break
            candidate = updated
        speed[index + 1] = min(candidate, updated)


def _backward_sweep(
    vehicle: PointMassVehicleParameters,
    track: TrackProfile,
    speed: FloatArray,
) -> None:
    for index in range(len(track.segment_length) - 1, -1, -1):
        next_speed = float(speed[index + 1])
        upper_speed = float(speed[index])
        if upper_speed <= next_speed:
            continue
        segment_length = float(track.segment_length[index])
        segment_curvature = max(
            abs(float(track.curvature[index])),
            abs(float(track.curvature[index + 1])),
        )

        candidate = upper_speed
        for _ in range(6):
            limits = acceleration_limits(
                vehicle,
                0.5 * (candidate + next_speed),
                segment_curvature,
            )
            reachable = math.sqrt(
                next_speed * next_speed
                + 2.0 * limits.maximum_deceleration * segment_length
            )
            updated = min(candidate, reachable)
            if abs(updated - candidate) <= 1.0e-7:
                break
            candidate = updated
        speed[index] = min(candidate, updated)


def _build_result(
    vehicle: PointMassVehicleParameters,
    track: TrackProfile,
    speed: FloatArray,
    converged: bool,
    iterations: int,
    maximum_change: float,
) -> LapResult:
    start_speed = speed[:-1]
    end_speed = speed[1:]
    acceleration = (
        end_speed * end_speed - start_speed * start_speed
    ) / (2.0 * track.segment_length)
    segment_curvature = np.maximum(
        np.abs(track.curvature[:-1]),
        np.abs(track.curvature[1:]),
    )
    lateral_acceleration = start_speed * start_speed * segment_curvature
    throttle = np.zeros_like(acceleration)
    brake = np.zeros_like(acceleration)
    for index, required_acceleration in enumerate(acceleration):
        evaluation_speed = 0.5 * (start_speed[index] + end_speed[index])
        limits = acceleration_limits(
            vehicle,
            float(evaluation_speed),
            float(segment_curvature[index]),
        )
        if required_acceleration > 0.0 and limits.maximum_acceleration > 0.0:
            throttle[index] = min(
                required_acceleration / limits.maximum_acceleration,
                1.0,
            )
        elif required_acceleration < 0.0 and limits.maximum_deceleration > 0.0:
            brake[index] = min(
                -required_acceleration / limits.maximum_deceleration,
                1.0,
            )

    denominator = start_speed + end_speed
    if np.any(denominator <= 0.0):
        raise RuntimeError("a lap segment has zero average speed")
    time_step = 2.0 * track.segment_length / denominator
    elapsed_time = np.concatenate(
        (np.asarray([0.0], dtype=np.float64), np.cumsum(time_step))
    )
    arrays = (
        speed,
        elapsed_time,
        acceleration,
        lateral_acceleration,
        throttle,
        brake,
    )
    for array in arrays:
        array.setflags(write=False)
    return LapResult(
        speed=speed,
        elapsed_time=elapsed_time,
        longitudinal_acceleration=acceleration,
        lateral_acceleration=lateral_acceleration,
        throttle=throttle,
        brake=brake,
        lap_time=float(elapsed_time[-1]),
        converged=converged,
        iterations=iterations,
        maximum_speed_change=maximum_change,
    )
