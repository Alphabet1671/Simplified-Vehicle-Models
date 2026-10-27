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


def yaw_approximation_diagnostics(time, station, vx, vy, yaw_rate, yaw_acceleration,
                                  tangential_acceleration, curvature, *, filter_width=0):
    """Audit r≈Vκ and r_dot≈a_tκ+V²κ' on a uniform periodic record.

    Curvature derivative is with respect to physical path distance. Optional
    filtering is a periodic moving average with an odd number of samples.
    Residuals are kinematic diagnostics, not force-balance validation.
    """
    arrays = [np.asarray(v, dtype=float) for v in
              (time, station, vx, vy, yaw_rate, yaw_acceleration, tangential_acceleration, curvature)]
    n = arrays[0].size
    if any(a.ndim != 1 or a.size != n or not np.all(np.isfinite(a)) for a in arrays) or n < 3:
        raise ValueError("diagnostic inputs must be equal-length finite one-dimensional arrays")
    time, station, vx, vy, yaw_rate, yaw_acceleration, at, curvature = arrays
    dt = np.diff(time)
    ds = np.diff(station)
    if np.any(dt <= 0) or np.any(ds <= 0):
        raise ValueError("time and physical station must increase")
    if not np.allclose(dt, dt[0], rtol=1e-6, atol=1e-10):
        raise ValueError("beta time derivative requires uniform time resampling")
    if np.any(ds <= 0):
        raise ValueError("physical station must increase without wrapping")
    beta = np.arctan2(vy, vx)
    beta_dot = periodic_centered_derivative(beta, dt[0])
    course_rate = yaw_rate + beta_dot
    speed = np.hypot(vx, vy)
    curvature_prime = np.gradient(curvature, station, edge_order=2)
    # Periodic derivative across the seam uses the physical lap length and
    # centered neighboring samples, not the endpoint-clustered solver stations.
    period=float(station[-1]-station[0] + np.median(ds))
    curvature_prime[0]=(curvature[1]-curvature[-1])/(station[1]-(station[-1]-period))
    curvature_prime[-1]=(curvature[0]-curvature[-2])/((station[0]+period)-station[-2])
    yaw_rate_residual = course_rate - speed * curvature
    yaw_accel_demand = at * curvature + speed ** 2 * curvature_prime
    yaw_accel_residual = yaw_acceleration - yaw_accel_demand
    raw = {"course_rate_minus_Vkappa": yaw_rate_residual,
           "yaw_acceleration_minus_path_demand": yaw_accel_residual}
    filtered = {}
    if filter_width:
        if filter_width < 3 or filter_width % 2 != 1 or filter_width > n:
            raise ValueError("filter_width must be an odd integer in [3, sample_count]")
        kernel = np.ones(filter_width) / filter_width
        filtered = {name: sum(np.roll(value, j) for j in range(-(filter_width//2), filter_width//2+1)) / filter_width
                    for name, value in raw.items()}
    def stats(value):
        absolute = np.abs(value)
        return {"rms": float(np.sqrt(np.mean(value**2))),
                "p95_abs": float(np.percentile(absolute, 95)), "max_abs": float(np.max(absolute))}
    return {"beta_rad": beta, "beta_dot_rad_s": beta_dot, "course_rate_rad_s": course_rate,
            "speed_mps": speed, "raw": {k: stats(v) for k, v in raw.items()},
            "filtered": {k: stats(v) for k, v in filtered.items()},
            "filter_width_samples": int(filter_width), "filter_kind": "periodic_boxcar" if filter_width else "none",
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
    """Project driven positions onto the station-local monotone reference branch."""
    x, y, hint = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float), np.asarray(station_hint, float))
    if x.ndim != 1 or not np.all(np.isfinite(np.r_[x, y, hint])): raise ValueError("positions and station hints must be finite one-dimensional arrays")
    if window <= 0 or samples < 3: raise ValueError("window must be positive and samples at least three")
    result=np.empty_like(hint); signed=np.empty_like(hint); previous=float(hint[0]); lap=float(reference.end-reference.start)
    closed=bool(getattr(reference,"is_closed",False)); selected_previous=None
    for i,(px,py,expected) in enumerate(zip(x,y,hint)):
        expected=float(expected)
        if closed: expected+=round((previous-expected)/lap)*lap
        center=max(expected,previous); search_center=selected_previous if selected_previous is not None and i>0 else center
        lo,hi=search_center-window,search_center+window
        if not closed:lo,hi=max(reference.start,lo),min(reference.end,hi)
        stations=np.linspace(lo,hi,samples); evals=(np.mod(stations-reference.start,lap)+reference.start) if closed else stations
        curve=reference.evaluate(evals); d2=(np.asarray(curve.north)-px)**2+(np.asarray(curve.east)-py)**2
        s=max(float(stations[int(np.argmin(d2))]),selected_previous if selected_previous is not None else previous)
        selected_previous=s; es=((s-reference.start)%lap)+reference.start if closed else s; curve=reference.evaluate(es)
        dx=px-float(np.asarray(curve.north).reshape(-1)[0]);dy=py-float(np.asarray(curve.east).reshape(-1)[0]);heading=float(np.asarray(curve.heading).reshape(-1)[0])
        result[i]=s;signed[i]=dx*(-np.sin(heading))+dy*np.cos(heading);previous=s
    return result,signed
