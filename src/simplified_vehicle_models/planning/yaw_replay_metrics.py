"""Time-weighted replay metrics and local geometric projection helpers."""
from __future__ import annotations

import numpy as np


def _validated(time, values, name):
    time = np.asarray(time, dtype=float)
    values = np.asarray(values)
    if time.ndim != 1 or values.ndim != 1 or time.shape != values.shape:
        raise ValueError("time and values must be equal-length one-dimensional arrays")
    if time.size < 2:
        raise ValueError("at least two samples are required")
    if not np.all(np.isfinite(time)) or not np.all(np.isfinite(values)):
        raise ValueError("time and values must be finite")
    if np.any(np.diff(time) <= 0):
        raise ValueError("time must be strictly increasing")
    return time, values


def time_weighted_rms(time, error) -> float:
    """RMS of a sampled signal using the explicit trapezoidal time integral."""
    time, error = _validated(time, error, "error")
    dt = np.diff(time)
    integral = np.sum(dt * (error[:-1] ** 2 + error[1:] ** 2) * 0.5)
    return float(np.sqrt(integral / (time[-1] - time[0])))


def time_weighted_fraction(time, mask) -> float:
    """Fraction of elapsed time for which a sampled boolean condition is true."""
    time, mask = _validated(time, mask, "mask")
    if mask.dtype.kind != "b":
        raise ValueError("mask must contain booleans")
    dt = np.diff(time)
    integral = np.sum(dt * (mask[:-1].astype(float) + mask[1:].astype(float)) * 0.5)
    return float(integral / (time[-1] - time[0]))


def project_xy_to_reference(reference, x, y, station_hint, *, window=30.0, samples=121):
    """Project driven positions onto the station-local monotone reference branch.

    Each sample searches a bounded interval around its prior station hint and
    carries the selected station forward, preventing a nearby hairpin branch
    from winning an unrestricted global nearest-point query.
    """
    x, y, hint = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float),
                                     np.asarray(station_hint, float))
    if x.ndim != 1 or not np.all(np.isfinite(np.r_[x, y, hint])):
        raise ValueError("positions and station hints must be finite one-dimensional arrays")
    if window <= 0 or samples < 3:
        raise ValueError("window must be positive and samples at least three")
    result = np.empty_like(hint)
    signed = np.empty_like(hint)
    previous = float(hint[0])
    lap = float(reference.end-reference.start)
    selected_previous = None
    for i, (px, py, expected) in enumerate(zip(x, y, hint)):
        # Unwrap periodic hints into the forward-running source-station domain.
        expected = float(expected)
        if getattr(reference, "is_closed", False):
            expected += round((previous-expected)/lap)*lap
        center = max(expected, previous)
        search_center = (selected_previous if selected_previous is not None and i > 0 else center)
        stations = np.linspace(max(reference.start, search_center-window),
                               min(reference.end, search_center+window), samples)
        curve = reference.evaluate(stations)
        distance2 = (np.asarray(curve.north)-px)**2 + (np.asarray(curve.east)-py)**2
        k = int(np.argmin(distance2)); s = float(stations[k])
        if selected_previous is not None:
            s = max(s, selected_previous)
        selected_previous = s
        # Enforce forward-monotone branch selection while permitting stationary samples.
        if s < previous:
            s = previous
            curve = reference.evaluate(s)
        dx, dy = px-float(np.asarray(curve.north).reshape(-1)[0]), py-float(np.asarray(curve.east).reshape(-1)[0])
        heading = float(np.asarray(curve.heading).reshape(-1)[0])
        result[i] = s
        signed[i] = dx*(-np.sin(heading)) + dy*np.cos(heading)
        previous = s
    return result, signed
