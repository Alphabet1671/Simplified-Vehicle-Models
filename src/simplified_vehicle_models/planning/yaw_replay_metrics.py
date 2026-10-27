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


def periodic_centered_derivative(values, spacing):
    """Second-order centered derivative of a uniformly sampled periodic signal."""
    values = np.asarray(values, dtype=float)
    if values.ndim != 1 or values.size < 3 or not np.all(np.isfinite(values)):
        raise ValueError("values must be a finite one-dimensional array with at least three points")
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("spacing must be positive and finite")
    return (np.roll(values, -1) - np.roll(values, 1)) / (2.0 * spacing)


def tangential_acceleration_from_body(vx, vy, ax_body, ay_body):
    """Project body-frame acceleration onto velocity to obtain scalar a_t."""
    vx, vy, ax, ay = np.broadcast_arrays(*[np.asarray(v, float) for v in (vx, vy, ax_body, ay_body)])
    speed = np.hypot(vx, vy)
    if not np.all(np.isfinite(np.r_[vx.ravel(), vy.ravel(), ax.ravel(), ay.ravel()])) or np.any(speed <= 0):
        raise ValueError("finite body velocity and acceleration with positive speed required")
    return (vx * ax + vy * ay) / speed


def _periodic_nonuniform_derivative(values, station, period):
    """Three-point derivative on nonuniform stations with periodic seam neighbors."""
    values = np.asarray(values, float)
    station = np.asarray(station, float)
    n = len(values)
    out = np.empty(n)
    for i in range(n):
        xm = station[i - 1] if i else station[-1] - period
        xp = station[i + 1] if i < n - 1 else station[0] + period
        hm = station[i] - xm
        hp = xp - station[i]
        if hm <= 0 or hp <= 0:
            raise ValueError("invalid periodic physical station spacing")
        out[i] = (-hp / (hm * (hm + hp)) * values[i - 1]
                  + (hp - hm) / (hm * hp) * values[i]
                  + hm / (hp * (hm + hp)) * values[(i + 1) % n])
    return out


def yaw_approximation_diagnostics(time, station, vx, vy, yaw_rate, yaw_acceleration,
                                  tangential_acceleration, curvature, *, filter_width=0,
                                  lap_length=None):
    """Audit body yaw-rate and course-rate approximations on a periodic trace.

    ``station`` must be unwrapped and monotonically increasing. ``lap_length``
    supplies periodic physical-distance neighbors. Tangential acceleration must
    be independently projected onto velocity; pass None to withhold the yaw
    acceleration residual rather than treating body-longitudinal acceleration
    as tangential acceleration.
    """
    required = (time, station, vx, vy, yaw_rate, yaw_acceleration, curvature)
    arrays = [np.asarray(v, dtype=float) for v in required]
    if tangential_acceleration is None:
        at = None
    else:
        at = np.asarray(tangential_acceleration, dtype=float)
    n = arrays[0].size
    if any(a.ndim != 1 or a.size != n or not np.all(np.isfinite(a)) for a in arrays) or n < 3:
        raise ValueError("diagnostic inputs must be equal-length finite one-dimensional arrays")
    time, station, vx, vy, yaw_rate, yaw_acceleration, curvature = arrays
    if at is not None and (at.shape != (n,) or not np.all(np.isfinite(at))):
        raise ValueError("tangential acceleration must be finite and match diagnostic trace")
    dt = np.diff(time)
    ds = np.diff(station)
    if np.any(dt <= 0) or np.any(ds <= 0):
        raise ValueError("time and unwrapped physical station must increase")
    if not np.allclose(dt, dt[0], rtol=1e-6, atol=1e-10):
        raise ValueError("beta time derivative requires uniform time resampling")
    period = float(lap_length) if lap_length is not None else float(station[-1] - station[0] + np.median(ds))
    if not np.isfinite(period) or period <= station[-1] - station[0]:
        raise ValueError("lap_length must exceed the sampled station span")
    beta = np.arctan2(vy, vx)
    beta_dot = periodic_centered_derivative(beta, dt[0])
    course_rate = yaw_rate + beta_dot
    speed = np.hypot(vx, vy)
    curvature_prime = _periodic_nonuniform_derivative(curvature, station, period)
    body_yaw_rate_residual = yaw_rate - speed * curvature
    course_rate_residual = course_rate - speed * curvature
    raw = {"body_yaw_rate_minus_Vkappa": body_yaw_rate_residual,
           "course_rate_minus_Vkappa": course_rate_residual}
    yaw_accel_demand = None
    if at is not None:
        yaw_accel_demand = at * curvature + speed ** 2 * curvature_prime
        raw["yaw_acceleration_minus_path_demand"] = yaw_acceleration - yaw_accel_demand
    filtered = {}
    if filter_width:
        if filter_width < 3 or filter_width % 2 != 1 or filter_width > n:
            raise ValueError("filter_width must be an odd integer in [3, sample_count]")
        filtered = {name: sum(np.roll(value, j) for j in range(-(filter_width // 2), filter_width // 2 + 1)) / filter_width
                    for name, value in raw.items()}

    def stats(value):
        absolute = np.abs(value)
        return {"rms": float(np.sqrt(np.mean(value ** 2))),
                "p95_abs": float(np.percentile(absolute, 95)), "max_abs": float(np.max(absolute))}

    return {"beta_rad": beta, "beta_dot_rad_s": beta_dot, "course_rate_rad_s": course_rate,
            "speed_mps": speed, "curvature_prime_per_m2": curvature_prime,
            "yaw_acceleration_demand_rad_s2": yaw_accel_demand,
            "raw": {k: stats(v) for k, v in raw.items()},
            "filtered": {k: stats(v) for k, v in filtered.items()},
            "yaw_accel_residual_status": "computed" if at is not None else "unknown_tangential_acceleration_unavailable",
            "filter_width_samples": int(filter_width), "filter_kind": "periodic_boxcar" if filter_width else "none",
            "filter_dt_s": float(dt[0] * filter_width) if filter_width else 0.0,
            "diagnostic_only_not_force_balance": True}


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
    lap = float(reference.end - reference.start)
    closed = bool(getattr(reference, "is_closed", False))
    selected_previous = None
    for i, (px, py, expected) in enumerate(zip(x, y, hint)):
        expected = float(expected)
        if closed:
            expected += round((previous - expected) / lap) * lap
        center = max(expected, previous)
        search_center = selected_previous if selected_previous is not None and i > 0 else center
        lo, hi = search_center - window, search_center + window
        if not closed:
            lo, hi = max(reference.start, lo), min(reference.end, hi)
        stations = np.linspace(lo, hi, samples)
        eval_stations = (np.mod(stations - reference.start, lap) + reference.start) if closed else stations
        curve = reference.evaluate(eval_stations)
        distance2 = (np.asarray(curve.north) - px) ** 2 + (np.asarray(curve.east) - py) ** 2
        k = int(np.argmin(distance2))
        s = float(stations[k])
        s = max(s, selected_previous if selected_previous is not None else previous)
        selected_previous = s
        eval_s = ((s - reference.start) % lap) + reference.start if closed else s
        curve = reference.evaluate(eval_s)
        dx = px - float(np.asarray(curve.north).reshape(-1)[0])
        dy = py - float(np.asarray(curve.east).reshape(-1)[0])
        heading = float(np.asarray(curve.heading).reshape(-1)[0])
        result[i] = s
        signed[i] = dx * (-np.sin(heading)) + dy * np.cos(heading)
        previous = s
    return result, signed
