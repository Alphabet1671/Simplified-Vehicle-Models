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


def periodic_interpolate(station, x, values, *, closed=False):
    """Interpolate geometry by station, wrapping physical station for closed paths."""
    station = np.asarray(station, dtype=float)
    x = np.asarray(x, dtype=float)
    values = np.asarray(values, dtype=float)
    if x.ndim != 1 or values.shape[0] != x.size or x.size < 2 or np.any(np.diff(x) <= 0):
        raise ValueError("interpolation stations and values must have matching increasing leading axis")
    query = station
    if closed:
        period = float(x[-1] - x[0])
        if not np.isfinite(period) or period <= 0:
            raise ValueError("closed path must have positive station period")
        query = (np.mod(station - x[0], period) + x[0])
    else:
        query = np.clip(station, x[0], x[-1])
    if values.ndim == 1:
        return np.interp(query, x, values)
    return np.stack([np.interp(query, x, values[:, j]) for j in range(values.shape[1])], axis=-1)


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
    closed = bool(getattr(reference, "is_closed", False))
    selected_previous = None
    for i, (px, py, expected) in enumerate(zip(x, y, hint)):
        # Work in an unwrapped station domain, but evaluate periodic references
        # modulo their period. This keeps a seam-crossing branch continuous.
        expected = float(expected)
        if closed:
            expected += round((previous-expected)/lap)*lap
        center = max(expected, previous)
        search_center = selected_previous if selected_previous is not None and i > 0 else center
        lo, hi = search_center-window, search_center+window
        if not closed:
            lo, hi = max(reference.start, lo), min(reference.end, hi)
        stations = np.linspace(lo, hi, samples)
        eval_stations = (np.mod(stations-reference.start, lap)+reference.start) if closed else stations
        curve = reference.evaluate(eval_stations)
        distance2 = (np.asarray(curve.north)-px)**2 + (np.asarray(curve.east)-py)**2
        k = int(np.argmin(distance2)); s = float(stations[k])
        s = max(s, selected_previous if selected_previous is not None else previous)
        selected_previous = s
        eval_s = ((s-reference.start) % lap)+reference.start if closed else s
        curve = reference.evaluate(eval_s)
        dx, dy = px-float(np.asarray(curve.north).reshape(-1)[0]), py-float(np.asarray(curve.east).reshape(-1)[0])
        heading = float(np.asarray(curve.heading).reshape(-1)[0])
        result[i] = s
        signed[i] = dx*(-np.sin(heading)) + dy*np.cos(heading)
        previous = s
    return result, signed
