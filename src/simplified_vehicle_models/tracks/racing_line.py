"""Reconstruct signed paths from lateral offsets.

Offsets are positive to the right of travel. In the repository's north/east
frame, the right normal at heading theta is (-sin(theta), cos(theta)).
"""

from __future__ import annotations

import math

import numpy as np

from simplified_vehicle_models.tracks.types import PathCoordinates


def _periodic_derivative(values, distance):
    """Differentiate periodic samples, excluding the repeated terminal node."""

    length = float(distance[-1])
    unique_distance = np.asarray(distance, dtype=float)[:-1]
    nodes = np.asarray(values, dtype=float)[:-1]
    forward = np.diff(np.r_[unique_distance, unique_distance[0] + length])
    backward = np.r_[forward[-1], forward[:-1]]
    derivative = (np.roll(nodes, -1) - np.roll(nodes, 1)) / (forward + backward)
    return np.r_[derivative, derivative[0]]


def offset_path(coordinates: PathCoordinates, offset) -> PathCoordinates:
    """Displace path nodes laterally and recompute their signed geometry."""

    values = np.asarray(offset, dtype=float)
    distance0 = np.asarray(coordinates.distance, dtype=float)
    if values.shape != distance0.shape:
        raise ValueError("offset must match the path node count")
    if not np.all(np.isfinite(values)):
        raise ValueError("offset must be finite")
    heading = np.asarray(coordinates.heading, dtype=float)
    north = np.asarray(coordinates.x_north, dtype=float) - values * np.sin(heading)
    east = np.asarray(coordinates.y_east, dtype=float) + values * np.cos(heading)
    distance = np.r_[0.0, np.cumsum(np.hypot(np.diff(north), np.diff(east)))]

    if coordinates.is_closed:
        tangent_north = _periodic_derivative(north, distance)
        tangent_east = _periodic_derivative(east, distance)
    else:
        tangent_north = np.gradient(north, distance)
        tangent_east = np.gradient(east, distance)
    angles = np.arctan2(tangent_east, tangent_north)
    new_heading = np.unwrap(angles)
    if coordinates.is_closed:
        unique_angles = angles[:-1]
        delta = np.roll(unique_angles, -1) - np.roll(unique_angles, 1)
        delta = (delta + np.pi) % (2.0 * np.pi) - np.pi
        length = float(distance[-1])
        unique_distance = distance[:-1]
        forward = np.diff(np.r_[unique_distance, unique_distance[0] + length])
        backward = np.r_[forward[-1], forward[:-1]]
        curvature = delta / (forward + backward)
        curvature = np.r_[curvature, curvature[0]]
    else:
        curvature = np.gradient(new_heading, distance)

    for array in (north, east, new_heading, curvature, distance):
        array.setflags(write=False)
    return PathCoordinates(distance, north, east, new_heading, curvature, coordinates.is_closed)


def _periodic_smooth(values, sigma_samples: float):
    """Apply a periodic Gaussian average to unique samples."""

    values = np.asarray(values, dtype=float)
    half = int(max(1, round(4.0 * sigma_samples)))
    taps = np.arange(-half, half + 1)
    kernel = np.exp(-0.5 * (taps / sigma_samples) ** 2)
    kernel /= kernel.sum()
    padded = np.r_[values[-half:], values, values[:half]]
    return np.convolve(padded, kernel, mode="valid")


def path_from_solution(track, solution, *, smoothing: float = 0.0) -> PathCoordinates:
    """Rebuild a track-grid path from a QSS solution's lateral offset."""

    if not math.isfinite(smoothing) or smoothing < 0.0:
        raise ValueError("smoothing must be finite and nonnegative")
    if track.path_coordinates is None:
        raise ValueError("track has no signed path coordinates")
    if not track.is_closed:
        raise ValueError("path_from_solution requires a closed track")
    from simplified_vehicle_models.solvers.qss_line_ocp import periodic_sample

    offset = periodic_sample(
        np.asarray(solution.offset, dtype=float)[:-1],
        np.asarray(solution.distance, dtype=float)[:-1],
        np.asarray(track.distance, dtype=float),
        period=float(track.lap_length),
    )
    if smoothing > 0.0:
        spacing = float(track.lap_length) / float(track.distance.size - 1)
        offset = _periodic_smooth(offset[:-1], smoothing / spacing)
        offset = np.r_[offset, offset[0]]
    return offset_path(track.path_coordinates, offset)
