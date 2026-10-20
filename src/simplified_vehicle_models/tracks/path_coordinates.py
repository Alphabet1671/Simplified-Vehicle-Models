"""Evaluate one signed path at a progress distance.

The stored geometry is a table of nodes.
Every node holds a position, a heading, and a signed curvature.
This module interpolates that table. It holds no state and needs only NumPy.

A closed path is periodic, so the progress distance wraps. An open path has a
start and an end, so the progress distance does not wrap. Outside an open
course, the path continues as a straight line along the tangent of the nearest
end. That extension is a numerical convenience for an integrator trial that
steps slightly past an end. It is not a part of the course.

Interpolation is linear, so the returned curvature is continuous and its
derivative is not. The optimal-control formulation needs a smoother
representation. Fit that representation from these nodes, and check the fit
error against this module. Do not assume that a linear curvature is smooth.
"""

from __future__ import annotations

import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.tracks.types import PathCoordinates


def evaluate_path(
    coordinates: PathCoordinates,
    distance: FloatArray,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    """Return position, heading, and curvature at the given distances.

    ``distance`` uses metres and may hold any value.

    For a closed path this function wraps the distance into one lap, so a
    negative value or a value past the lap end is accepted.

    For an open path this function does not wrap. A value before the start or
    past the end continues along the end tangent with zero curvature and a
    constant heading.

    The outputs use the broadcast shape of ``distance``.
    """

    query = np.asarray(distance, dtype=np.float64)
    length = coordinates.lap_length
    if not np.isfinite(length) or length <= 0.0:
        raise ValueError("the path must have a positive finite length")

    if not coordinates.is_closed:
        return _evaluate_open(coordinates, query)

    laps = np.floor(query / length)
    wrapped = query - laps * length
    x_north = np.interp(wrapped, coordinates.distance, coordinates.x_north)
    y_east = np.interp(wrapped, coordinates.distance, coordinates.y_east)
    curvature = np.interp(wrapped, coordinates.distance, coordinates.curvature)
    # The stored heading is unwrapped and falls by one turn across a lap.
    # Continue that trend for a query that leaves the first lap.
    heading = np.interp(wrapped, coordinates.distance, coordinates.heading)
    heading_turn = coordinates.heading[-1] - coordinates.heading[0]
    heading = heading + laps * heading_turn
    return (
        np.asarray(x_north, dtype=np.float64),
        np.asarray(y_east, dtype=np.float64),
        np.asarray(heading, dtype=np.float64),
        np.asarray(curvature, dtype=np.float64),
    )


def _evaluate_open(
    coordinates: PathCoordinates,
    query: FloatArray,
) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    """Evaluate an open path and extend it with straight end tangents."""

    start = float(coordinates.distance[0])
    end = float(coordinates.distance[-1])
    inside = np.clip(query, start, end)
    x_north = np.interp(inside, coordinates.distance, coordinates.x_north)
    y_east = np.interp(inside, coordinates.distance, coordinates.y_east)
    heading = np.interp(inside, coordinates.distance, coordinates.heading)
    curvature = np.interp(inside, coordinates.distance, coordinates.curvature)

    # Signed distance beyond an end: negative before the start, positive past the end.
    overshoot = np.where(query < start, query - start, np.where(query > end, query - end, 0.0))
    # The heading is measured from north toward east, so the tangent is
    # (cos, sin) in (north, east).
    x_north = x_north + overshoot * np.cos(heading)
    y_east = y_east + overshoot * np.sin(heading)
    curvature = np.where(overshoot != 0.0, 0.0, curvature)
    return (
        np.asarray(x_north, dtype=np.float64),
        np.asarray(y_east, dtype=np.float64),
        np.asarray(heading, dtype=np.float64),
        np.asarray(curvature, dtype=np.float64),
    )


def heading_modulo_two_pi(heading: FloatArray) -> FloatArray:
    """Return a heading folded into the interval from minus pi to pi."""

    folded = np.mod(np.asarray(heading, dtype=np.float64) + np.pi, 2.0 * np.pi)
    return folded - np.pi
