"""Independent fixed-path capability-profile reference (no torque vectoring).

This module is deliberately separate from the production lap solver.  It solves
on prescribed path arclength using a sampled capability table; it is a numerical
reference, not an empirical validation of the original vehicle plant.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Callable

import numpy as np


@dataclass(frozen=True, slots=True)
class CapabilityProfile:
    """An immutable sampled fixed-path speed reference."""
    distance: np.ndarray
    speed: np.ndarray
    elapsed_time: np.ndarray
    lap_time: float
    converged: bool
    status: str
    iterations: int
    max_change: float
    reference_distance: np.ndarray | None = None
    diagnostics: dict | None = None

    def __post_init__(self) -> None:
        for name in ("distance", "speed", "elapsed_time", "reference_distance"):
            value = getattr(self, name)
            if value is not None:
                value = np.array(value, dtype=float, copy=True)
                value.setflags(write=False)
                object.__setattr__(self, name, value)
        object.__setattr__(self, "diagnostics", MappingProxyType(dict(self.diagnostics or {})))


def _finite_positive(value, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _strict_linear(axis, values, query: float, name: str) -> float:
    axis = np.asarray(axis, dtype=float)
    values = np.asarray(values, dtype=float)
    if not math.isfinite(query) or query < axis[0] or query > axis[-1]:
        raise ValueError(f"{name} query outside supported physical domain")
    return float(np.interp(query, axis, values))


def _legacy_limits(surface, speed: float, curvature: float) -> tuple[float, float]:
    """Bilinear legacy-table query without endpoint clipping/extrapolation."""
    speeds = np.asarray(surface.speed, dtype=float)
    curves = np.asarray(surface.curvature, dtype=float)
    if speed < speeds[0] or speed > speeds[-1] or curvature < curves[0] or curvature > curves[-1]:
        raise ValueError("capability query outside supported physical domain")
    def bilinear(values):
        values = np.asarray(values, dtype=float)
        # np.interp in curvature, then speed, is bilinear on a rectilinear grid.
        by_speed = np.asarray([np.interp(curvature, curves, row) for row in values])
        return float(np.interp(speed, speeds, by_speed))
    return bilinear(surface.acceleration_min), bilinear(surface.acceleration_max)


def _surface_interface(surface):
    """Return strict cap/limit functions and the demonstrated speed floor."""
    if hasattr(surface, "domain_speed") and hasattr(surface, "speed_floor"):
        def cap(k):
            return float(surface.domain_speed(float(k)))
        def limits(v, k):
            return tuple(float(x) for x in surface.limits(float(v), float(k)))
        return cap, limits, float(surface.speed_floor)
    required = ("speed", "curvature", "speed_ceiling", "acceleration_min", "acceleration_max")
    if not all(hasattr(surface, name) for name in required):
        raise TypeError("surface must be CapabilitySurface or SignedTransientCapability")
    def cap(k):
        return _strict_linear(surface.curvature, surface.speed_ceiling, float(k), "capability")
    return cap, lambda v, k: _legacy_limits(surface, v, k), float(np.asarray(surface.speed, dtype=float)[0])


def _signed_affine_cells(surface, curvature: float, which: str):
    """Return physical-speed affine chart pieces at one fixed curvature.

    A signed chart is bilinear in ``(u, curvature)``.  Once curvature is
    fixed, each u-cell is exactly affine in *physical* speed; this exposes the
    transition equations without extrapolating a sampled chart.
    """
    values = np.asarray(getattr(surface, which), dtype=float)
    u = np.asarray(surface.u, dtype=float)
    domain = float(surface.domain_speed(float(curvature)))
    samples = np.asarray([np.interp(curvature, surface.curvature, row) for row in values])
    scale = domain - float(surface.speed_floor)
    cells = []
    for index in range(u.size - 1):
        lower = float(surface.speed_floor + u[index]*scale)
        upper = float(surface.speed_floor + u[index + 1]*scale)
        slope = float((samples[index + 1] - samples[index])/(upper-lower))
        cells.append((lower, upper, slope, float(samples[index] - slope*lower)))
    return cells


def _quadratic_roots(a: float, b: float, c: float) -> list[float]:
    """Return finite real roots, retaining the linear degenerate case."""
    if abs(a) <= 1.e-14:
        return [] if abs(b) <= 1.e-14 else [-c/b]
    discriminant = b*b - 4.*a*c
    if discriminant < -1.e-12:
        return []
    root = math.sqrt(max(0., discriminant))
    return [(-b-root)/(2.*a), (-b+root)/(2.*a)]


def _signed_transition(surface, *, vi: float, bound: float, curvature: float, ds: float,
                       forward: bool, tolerance: float) -> float:
    """Greatest chart-supported energy endpoint from affine cell enumeration.

    The returned endpoint is the greatest enumerated feasible endpoint whose
    signed direction is continuable from the actual current state.  Ambiguous
    remote branches are refused rather than selected by a seed-dependent point.
    """
    if min(vi, bound) < surface.speed_floor:
        raise RuntimeError("signed affine transition below chart floor is unsupported")
    w_fixed, w_bound = vi*vi, bound*bound
    lower = _signed_affine_cells(surface, curvature, "acceleration_min")
    upper = _signed_affine_cells(surface, curvature, "acceleration_max")
    edges = set()
    for lo, hi, _, _ in lower + upper:
        for speed in (lo, hi):
            w = 2.*speed*speed-w_fixed
            if 0. <= w <= w_bound:
                edges.add(math.sqrt(.5*(w+w_fixed)))
    m_min = math.sqrt(.5*w_fixed)
    m_anchor = math.sqrt(w_fixed)  # identical endpoint energy: w_other == w_fixed
    m_max = math.sqrt(.5*(w_fixed+w_bound))
    edges.update((m_min, m_anchor, m_max))
    # All chart boundaries are shared; use the exact lookup at every candidate
    # to retain the public signed-axis/domain semantics.
    for cells in (lower, upper):
        for lo, hi, slope, intercept in cells:
            if forward:
                coefficients = (1., -ds*slope, -ds*intercept-w_fixed)
            else:
                coefficients = (1., ds*slope, ds*intercept-w_fixed)
            for root in _quadratic_roots(*coefficients):
                if m_min-tolerance <= root <= m_max+tolerance and lo-tolerance <= root <= hi+tolerance:
                    edges.add(min(m_max, max(m_min, root)))
    ordered = sorted(edges)

    def feasible(m: float) -> bool:
        if forward:
            w = 2.*m*m-w_fixed
            acceleration = (w-w_fixed)/(2.*ds)
        else:
            w = 2.*m*m-w_fixed
            acceleration = (w_fixed-w)/(2.*ds)
        if w < -tolerance or w > w_bound+tolerance:
            return False
        amin, amax = surface.limits(m, curvature)
        return amin-tolerance <= acceleration <= amax+tolerance

    intervals = []
    for lo, hi in zip(ordered[:-1], ordered[1:]):
        middle = .5*(lo+hi)
        if feasible(middle):
            intervals.append([lo, hi])
    # A feasible endpoint can be the cap/boundary even when its neighbouring
    # interval is infeasible.  It cannot provide a positive-length transition.
    if not intervals:
        # A stationary cap/bound has no open midpoint interval, but remains a
        # valid zero-energy transition when it exactly satisfies the limits.
        if feasible(m_max):
            return math.sqrt(max(0., min(w_bound, 2.*m_max*m_max-w_fixed)))
        raise RuntimeError("signed affine transition has no feasible energy interval")
    merged = []
    for lo, hi in intervals:
        if merged and lo <= merged[-1][1] + tolerance:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    if len(merged) != 1:
        raise RuntimeError("signed affine transition has disconnected feasible branches")
    m = merged[0][1]
    endpoint = math.sqrt(max(0., min(w_bound, 2.*m*m-w_fixed)))
    # A local ds->0 continuation starts with a in [amin(vi), amax(vi)].
    # Consequently a branch cannot reverse its permitted local energy direction
    # merely because a remote chart cell has a favorable root.  This is not an
    # a==0 anchor test: a strictly negative amax permits braking and a strictly
    # positive amin permits acceleration.  The exact chart-cell zero barriers
    # additionally prohibit crossing a speed where the relevant signed limit
    # eliminates that direction.
    amin_now, amax_now = surface.limits(vi, curvature)
    if forward:
        if endpoint > vi + tolerance:
            if amax_now <= tolerance:
                raise RuntimeError("signed affine transition lacks a connected current-state branch")
            direction_cells = upper
        elif endpoint < vi - tolerance:
            if amin_now >= -tolerance:
                raise RuntimeError("signed affine transition lacks a connected current-state branch")
            direction_cells = lower
        else:
            return endpoint
    else:
        if endpoint > vi + tolerance:
            if amin_now >= -tolerance:
                raise RuntimeError("signed affine transition lacks a connected current-state branch")
            direction_cells = lower
        elif endpoint < vi - tolerance:
            if amax_now <= tolerance:
                raise RuntimeError("signed affine transition lacks a connected current-state branch")
            direction_cells = upper
        else:
            return endpoint
    # Every sign change is affine within a chart cell; its analytic zero is a
    # direction barrier, so no residual search or sampled continuation is used.
    for lo, hi, slope, intercept in direction_cells:
        if (min(vi, endpoint) < hi-tolerance and max(vi, endpoint) > lo+tolerance
                and abs(slope) > 1.e-14):
            root = -intercept/slope
            if min(vi, endpoint)+tolerance < root < max(vi, endpoint)-tolerance:
                raise RuntimeError("signed affine transition crosses a signed-direction barrier")
    return endpoint


def _bracketed_energy_transition(limits, *, vi: float, bound: float, curvature: float,
                                ds: float, forward: bool, tolerance: float,
                                maximum_iterations: int = 64,
                                segment_limit_sampling: str = "midpoint",
                                curvature_samples=None) -> float:
    """Return a feasible bracketed endpoint under sampled monotone residuals.

    Monotonicity is checked at fixed fractions and during bisection; it is not a
    continuous-domain certificate. The returned endpoint is always checked
    against the actual acceleration callback.
    """
    if segment_limit_sampling not in ("midpoint", "speed_endpoints_and_midpoint"):
        raise ValueError("segment_limit_sampling must be 'midpoint' or 'speed_endpoints_and_midpoint'")
    scalars = (vi, bound, curvature, ds, tolerance)
    if any(isinstance(x, bool) or not isinstance(x, (int, float, np.number))
           or not math.isfinite(float(x)) for x in scalars):
        raise ValueError("bracketed transition inputs must be finite real scalars")
    vi, bound, curvature, ds, tolerance = map(float, scalars)
    if vi < 0. or bound < 0. or ds <= 0. or tolerance <= 0.:
        raise ValueError("bracketed transition requires nonnegative speeds and positive ds/tolerance")
    if isinstance(maximum_iterations, bool) or not isinstance(maximum_iterations, (int, np.integer)) or maximum_iterations < 1:
        raise ValueError("maximum_iterations must be a positive integer")
    if curvature_samples is not None:
        if not isinstance(curvature_samples, (tuple, list, np.ndarray)):
            raise ValueError("curvature_samples must be a nonempty finite real sequence")
        frozen_curvatures = tuple(curvature_samples)
        if not frozen_curvatures or any(isinstance(k, (bool, np.bool_)) or
                not isinstance(k, (int, float, np.integer, np.floating)) or not math.isfinite(float(k))
                for k in frozen_curvatures):
            raise ValueError("curvature_samples must be a nonempty finite real sequence")
        frozen_curvatures = tuple(float(k) for k in frozen_curvatures)
    w0, wb = vi*vi, bound*bound
    if not math.isfinite(w0) or not math.isfinite(wb):
        raise ValueError("bracketed transition squared speeds overflow")
    energy_tol = 2.*ds*tolerance + max(w0, wb, 1.)*32.*np.finfo(float).eps
    if not math.isfinite(energy_tol):
        raise ValueError("bracketed transition energy tolerance overflow")

    def residual(endpoint):
        midpoint = math.sqrt(.5*(w0 + endpoint*endpoint))
        query_speeds = ((midpoint,) if segment_limit_sampling == "midpoint"
                        else (vi, endpoint, midpoint))
        queried = []
        for speed in query_speeds:
            for k_query in ((curvature,) if curvature_samples is None else frozen_curvatures):
                pair = limits(speed, k_query)
                if not isinstance(pair, (tuple, list)) or len(pair) != 2:
                    raise ValueError("transition limits must return two accelerations")
                real_types = (int, float, np.integer, np.floating)
                if any(isinstance(x, (bool, np.bool_)) or not isinstance(x, real_types) for x in pair):
                    raise ValueError("transition limits must be real non-boolean scalars")
                amin_i, amax_i = float(pair[0]), float(pair[1])
                if not math.isfinite(amin_i) or not math.isfinite(amax_i) or amin_i > amax_i:
                    raise ValueError("transition limits are nonfinite or unordered")
                queried.append((amin_i, amax_i))
        amin = max(item[0] for item in queried)
        amax = min(item[1] for item in queried)
        value = (endpoint*endpoint - w0 - 2.*ds*amax if forward
                 else endpoint*endpoint - w0 + 2.*ds*amin)
        if not math.isfinite(value):
            raise ValueError("transition energy residual overflow")
        return value

    points = [bound*f for f in (0., .25, .5, .75, 1.)]
    values = [residual(x) for x in points]
    if any(right < left-energy_tol for left, right in zip(values, values[1:])):
        raise RuntimeError("sampled energy residual is nonmonotone; bracketed transition is not eligible")
    if values[0] > energy_tol:
        raise RuntimeError("no feasible bracketed energy transition")
    if values[-1] <= 0.:
        return bound
    brackets = [(points[i], points[i+1], values[i], values[i+1])
                for i in range(4) if values[i] <= 0. < values[i+1]]
    if not brackets:
        raise RuntimeError("no feasible bracketed energy transition")
    lo, hi, flo, fhi = brackets[-1]
    for _ in range(maximum_iterations):
        if hi-lo <= tolerance and abs(flo)/(2.*ds) <= tolerance:
            return lo
        mid = .5*(lo+hi)
        if mid == lo or mid == hi:
            break
        fm = residual(mid)
        if fm < flo-energy_tol or fm > fhi+energy_tol:
            raise RuntimeError("energy residual became nonmonotone during bisection")
        if fm <= 0.:
            lo, flo = mid, fm
        else:
            hi, fhi = mid, fm
    if hi-lo <= tolerance and abs(flo)/(2.*ds) <= tolerance:
        return lo
    raise RuntimeError("bracketed energy transition did not meet physical residual guards")


def _path_samples(path, step: float, trajectory):
    for name in ("distance", "curvature", "is_closed"):
        if not hasattr(path, name):
            raise TypeError("path must provide distance, curvature, and is_closed")
    raw_distance = np.asarray(path.distance, dtype=float)
    raw_curvature = np.asarray(path.curvature, dtype=float)
    if raw_distance.ndim != 1 or raw_curvature.shape != raw_distance.shape or raw_distance.size < 2:
        raise ValueError("path distance and curvature must be matching one-dimensional arrays")
    if not np.all(np.isfinite(raw_distance)) or not np.all(np.isfinite(raw_curvature)) or np.any(np.diff(raw_distance) <= 0.0):
        raise ValueError("path samples must be finite and strictly increasing")
    distance = np.r_[np.arange(raw_distance[0], raw_distance[-1], step), raw_distance[-1]]
    if distance[0] != raw_distance[0]:
        distance = np.r_[raw_distance[0], distance]
    distance = np.unique(distance)
    reference = getattr(path, "reference_distance", None)
    if reference is not None:
        reference = np.asarray(reference, dtype=float)
        if (reference.shape != raw_distance.shape or not np.all(np.isfinite(reference))
                or np.any(np.diff(reference) <= 0.0)):
            raise ValueError("path reference_distance must match a strictly increasing path grid")
        mapped = np.interp(distance, raw_distance, reference)
    else:
        mapped = None
    if trajectory is None:
        curvature = np.interp(distance, raw_distance, raw_curvature)
    else:
        station_map = getattr(trajectory, "station_map", None)
        if station_map is None or not callable(getattr(station_map, "to_reference", None)):
            raise ValueError("trajectory curvature requires an explicit station_map.to_reference")
        # The authoritative analytic map, rather than interpolation through a
        # sampled path table, establishes ell -> source-station correspondence.
        try:
            mapped = np.asarray(station_map.to_reference(distance), dtype=float)
        except (TypeError, ValueError) as error:
            raise ValueError("sampled path arclength is outside trajectory station-map coverage") from error
        if mapped.shape != distance.shape or not np.all(np.isfinite(mapped)) or np.any(np.diff(mapped) <= 0.0):
            raise ValueError("trajectory station map must cover every sampled path station monotonically")
        values = trajectory.evaluate(mapped)
        curvature = np.asarray(values.curvature, dtype=float)
        if curvature.shape != distance.shape or not np.all(np.isfinite(curvature)):
            raise ValueError("trajectory did not return finite curvature at every path station")
    return distance, curvature, mapped


def _closed_duplicate(path, curvature: np.ndarray) -> bool:
    if not bool(path.is_closed) or not np.isclose(curvature[0], curvature[-1], rtol=0., atol=1.e-10):
        return False
    # Periodicity is physical, not a boolean/table-curvature convention: XY
    # endpoint evidence is required for a duplicated-end ring.
    if not (hasattr(path, "x_north") and hasattr(path, "y_east")):
        return False
    x, y = np.asarray(path.x_north, dtype=float), np.asarray(path.y_east, dtype=float)
    if x.ndim != 1 or y.shape != x.shape or x.size < 2 or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        return False
    return bool(np.isclose(x[0], x[-1], rtol=0., atol=1.e-9) and np.isclose(y[0], y[-1], rtol=0., atol=1.e-9))


def capability_profile(path, surface, *, boundary: str = "flying", start_speed=None,
                       step: float = .25, tolerance: float = 1.e-7,
                       maximum_iterations: int = 200, rest_limits: Callable | None = None,
                       trajectory=None, transition_method: str = "legacy_fixed_point",
                        segment_limit_sampling: str = "midpoint",
                        segment_curvature_sampling: str = "node_average") -> CapabilityProfile:
    """Solve a bounded forward/backward energy profile on exact path arclength.

    ``rest_limits(speed, curvature)`` is an explicit original-physics low-speed
    extension returning ``(minimum_acceleration, maximum_acceleration)``.  It is
    required for standing starts below a table's demonstrated speed floor.
    """
    step = _finite_positive(step, "step")
    tolerance = _finite_positive(tolerance, "tolerance")
    if isinstance(maximum_iterations, bool) or not isinstance(maximum_iterations, (int, np.integer)) or maximum_iterations < 1:
        raise ValueError("maximum_iterations must be a positive integer")
    if not isinstance(transition_method, str) or transition_method not in ("legacy_fixed_point", "bracketed_energy"):
        raise ValueError("transition_method must be 'legacy_fixed_point' or 'bracketed_energy'")
    if not isinstance(segment_curvature_sampling, str) or segment_curvature_sampling not in ("node_average", "trajectory_five_fractions"):
        raise ValueError("segment_curvature_sampling must be 'node_average' or 'trajectory_five_fractions'")
    if segment_curvature_sampling == "trajectory_five_fractions":
        if trajectory is None:
            raise ValueError("trajectory_five_fractions requires a trajectory")
        if transition_method != "bracketed_energy":
            raise ValueError("trajectory_five_fractions requires transition_method='bracketed_energy'")
        if hasattr(surface, "u") and hasattr(surface, "domain_speed"):
            raise ValueError("trajectory_five_fractions is unsupported for signed charts")
    if not isinstance(segment_limit_sampling, str) or segment_limit_sampling not in ("midpoint", "speed_endpoints_and_midpoint"):
        raise ValueError("segment_limit_sampling must be 'midpoint' or 'speed_endpoints_and_midpoint'")
    if segment_limit_sampling != "midpoint" and transition_method != "bracketed_energy":
        raise ValueError("speed endpoint sampling requires transition_method='bracketed_energy'")
    if boundary not in ("flying", "rolling", "standing"):
        raise ValueError("boundary must be 'flying', 'rolling', or 'standing'")
    geometry_wall_guard = None
    if segment_curvature_sampling == "trajectory_five_fractions":
        geometry_wall_guard = getattr(surface, "ensure_wall_budget", None)
        if callable(geometry_wall_guard):
            geometry_wall_guard()
    distance, curvature, reference = _path_samples(path, step, trajectory)
    if callable(geometry_wall_guard):
        geometry_wall_guard()
    cap, chart_limits, floor = _surface_interface(surface)
    segment = np.diff(distance)
    curvature_samples = None
    if segment_curvature_sampling == "trajectory_five_fractions":
        station_map = getattr(trajectory, "station_map", None)
        inverse = getattr(station_map, "to_reference", None)
        evaluate = getattr(trajectory, "evaluate", None)
        if not callable(inverse) or not callable(evaluate):
            raise ValueError("trajectory_five_fractions requires station_map.to_reference and trajectory.evaluate")
        sample_rows = []
        wall_guard = geometry_wall_guard
        if callable(wall_guard):
            wall_guard()
        for i, ds_i in enumerate(segment):
            physical = np.asarray((distance[i], distance[i] + .25*ds_i, distance[i] + .5*ds_i,
                                   distance[i] + .75*ds_i, distance[i+1]), dtype=float)
            try:
                mapped = np.asarray(inverse(physical), dtype=float)
            except (TypeError, ValueError) as error:
                raise ValueError("trajectory geometry sample is outside station-map coverage") from error
            if mapped.shape != physical.shape or not np.all(np.isfinite(mapped)):
                raise ValueError("trajectory station map returned malformed geometry samples")
            source_bounds = np.asarray(getattr(getattr(trajectory, "reference", None), "source_station", ()), dtype=float)
            if source_bounds.ndim != 1 or source_bounds.size < 2 or not np.all(np.isfinite(source_bounds)) or np.any(np.diff(source_bounds) <= 0.) or np.any(mapped < source_bounds[0]) or np.any(mapped > source_bounds[-1]):
                raise ValueError("trajectory geometry sample is outside strict source station bounds")
            values = evaluate(mapped)
            sampled = np.asarray(getattr(values, "curvature", None), dtype=float)
            if sampled.shape != physical.shape or not np.all(np.isfinite(sampled)):
                raise ValueError("trajectory did not return finite curvature at every segment fraction")
            sample_rows.append(tuple(float(k) for k in np.r_[sampled, .5*(curvature[i]+curvature[i+1])]))
        curvature_samples = tuple(sample_rows)
        if callable(wall_guard):
            wall_guard()
    if boundary == "flying":
        if start_speed is not None:
            raise ValueError("flying boundary does not accept start_speed")
        if not _closed_duplicate(path, curvature):
            raise ValueError("flying boundary requires a closed path with an equal duplicated end")
    else:
        if bool(path.is_closed):
            raise ValueError("rolling and standing boundaries require an open path")
        if boundary == "rolling":
            if start_speed is None:
                raise ValueError("rolling boundary requires start_speed")
            start_speed = float(start_speed)
            if not math.isfinite(start_speed) or start_speed <= 0.0:
                raise ValueError("rolling start_speed must be finite and positive")
        else:
            if start_speed not in (None, 0, 0.0):
                raise ValueError("standing boundary has an exact zero start speed")
            if rest_limits is None:
                raise ValueError("standing start below chart floor requires rest_limits")
            if not callable(rest_limits):
                raise TypeError("rest_limits must be callable")
    try:
        caps = np.asarray([cap(k) for k in curvature], dtype=float)
    except ValueError as error:
        raise ValueError("path curvature is outside the declared capability domain") from error
    # A segment's selected curvature caps constrain both endpoint speeds.
    if curvature_samples is not None:
        try:
            segment_caps = np.asarray([min(cap(k) for k in row) for row in curvature_samples], dtype=float)
        except ValueError as error:
            raise ValueError("sampled path curvature is outside the declared capability domain") from error
    else:
        segment_caps = np.asarray([cap(.5*(curvature[i] + curvature[i+1]))
                                   for i in range(curvature.size - 1)], dtype=float)
    caps[:-1] = np.minimum(caps[:-1], segment_caps)
    caps[1:] = np.minimum(caps[1:], segment_caps)
    if np.any(~np.isfinite(caps)) or np.any(caps <= 0.0):
        raise ValueError("capability speed caps must be finite and positive")
    if hasattr(surface, "speed") and np.any(caps > float(np.max(np.asarray(surface.speed, dtype=float)))):
        raise ValueError("capability speed ceiling exceeds sampled speed domain")
    if boundary == "rolling" and start_speed > caps[0] + tolerance:
        raise ValueError("rolling start_speed exceeds the initial capability domain")
    if boundary == "rolling" and start_speed < floor:
        raise ValueError("rolling start_speed is below the demonstrated capability domain")
    if boundary == "standing":
        # A rest extension must connect to the chart at every encountered
        # curvature rather than create an unverified discontinuity at the floor.
        selected_k = (np.asarray(curvature_samples, dtype=float).reshape(-1) if curvature_samples is not None else
                      np.r_[curvature, .5*(curvature[:-1] + curvature[1:])])
        for k in np.unique(selected_k):
            lo, hi = rest_limits(floor, float(k))
            clo, chi = chart_limits(floor, float(k))
            if not all(math.isfinite(float(x)) for x in (lo, hi)) or lo > hi:
                raise ValueError("rest_limits returned invalid acceleration limits")
            if abs(float(lo)-clo) > 1.e-6 or abs(float(hi)-chi) > 1.e-6:
                raise ValueError("rest_limits must continuously match chart limits at its floor")
    speed = caps.copy()
    if boundary == "rolling":
        speed[0] = start_speed
    elif boundary == "standing":
        speed[0] = 0.0
    else:
        speed[-1] = speed[0] = min(speed[0], speed[-1])

    def limits(v, k):
        if v < floor:
            if boundary != "standing" or rest_limits is None:
                raise ValueError("capability query below demonstrated speed floor")
            answer = rest_limits(float(v), float(k))
            if len(answer) != 2:
                raise ValueError("rest_limits must return two acceleration limits")
            lo, hi = (float(answer[0]), float(answer[1]))
            if not math.isfinite(lo) or not math.isfinite(hi) or lo > hi:
                raise ValueError("rest_limits returned invalid acceleration limits")
            return lo, hi
        return chart_limits(float(v), float(k))

    def failed(status, iteration, change, detail):
        details = {"detail": detail, "boundary": boundary, "floor": floor}
        if transition_method == "bracketed_energy":
            details.update({"transition_method": transition_method,
                            "root_assumption": "sampled_non_decreasing_energy_residual_not_continuous_certificate",
                            "segment_limit_sampling": segment_limit_sampling})
            if curvature_samples is not None:
                details.update({"segment_curvature_sampling": segment_curvature_sampling,
                                "curvature_samples_per_segment": 6,
                                "geometry_query_stations_kind": "physical_fraction_via_computational_station_map_inverse",
                                "continuous_feasibility_certified": False})
        return CapabilityProfile(distance, speed, np.full(distance.shape, np.nan), math.nan,
                                 False, status, iteration, change, reference, details)

    signed_chart = hasattr(surface, "u") and hasattr(surface, "domain_speed")
    if signed_chart and transition_method == "bracketed_energy":
        raise ValueError("bracketed_energy is only supported for original-live/generic surfaces, not signed charts")
    max_change = math.inf
    for iteration in range(1, maximum_iterations + 1):
        previous = speed.copy()
        try:
            # Forward transition: decreasing fixed point from the current cap.
            for i, ds in enumerate(segment):
                vi, candidate = float(speed[i]), float(speed[i+1])
                k = .5*(curvature[i] + curvature[i+1])
                if signed_chart and vi >= floor and candidate >= floor:
                    speed[i+1] = _signed_transition(surface, vi=vi, bound=candidate,
                                                    curvature=k, ds=ds, forward=True,
                                                    tolerance=tolerance)
                elif transition_method == "bracketed_energy":
                    kwargs = ({"curvature_samples": curvature_samples[i]} if curvature_samples is not None else {})
                    speed[i+1] = _bracketed_energy_transition(
                        limits, vi=vi, bound=candidate, curvature=k, ds=ds,
                        forward=True, tolerance=tolerance,
                        segment_limit_sampling=segment_limit_sampling, **kwargs)
                else:
                    for _ in range(32):
                        vm = math.sqrt(.5*(vi*vi + candidate*candidate))
                        _, amax = limits(vm, k)
                        radicand = vi*vi + 2.*amax*ds
                        if radicand < -tolerance:
                            raise RuntimeError("forward transition has no real energy state")
                        updated = min(candidate, math.sqrt(max(0., radicand)))
                        if abs(updated-candidate) <= tolerance:
                            candidate = updated; break
                        candidate = updated
                    else:
                        return failed("transition-failed", iteration, max_change, "forward fixed point did not converge")
                    speed[i+1] = candidate
            if boundary == "flying":
                speed[0] = speed[-1] = min(speed[0], speed[-1])
            elif boundary == "rolling": speed[0] = start_speed
            else: speed[0] = 0.0
            # Backward braking preview propagates future caps dynamically.
            for i in range(len(segment)-1, -1, -1):
                vj, candidate = float(speed[i+1]), float(speed[i])
                k = .5*(curvature[i] + curvature[i+1])
                if signed_chart and vj >= floor and candidate >= floor:
                    speed[i] = _signed_transition(surface, vi=vj, bound=candidate,
                                                  curvature=k, ds=segment[i], forward=False,
                                                  tolerance=tolerance)
                elif transition_method == "bracketed_energy":
                    kwargs = ({"curvature_samples": curvature_samples[i]} if curvature_samples is not None else {})
                    speed[i] = _bracketed_energy_transition(
                        limits, vi=vj, bound=candidate, curvature=k, ds=segment[i],
                        forward=False, tolerance=tolerance,
                        segment_limit_sampling=segment_limit_sampling, **kwargs)
                else:
                    for _ in range(32):
                        vm = math.sqrt(.5*(candidate*candidate + vj*vj))
                        amin, _ = limits(vm, k)
                        radicand = vj*vj - 2.*amin*segment[i]
                        if radicand < -tolerance:
                            raise RuntimeError("backward transition has no real energy state")
                        updated = min(candidate, math.sqrt(max(0., radicand)))
                        if abs(updated-candidate) <= tolerance:
                            candidate = updated; break
                        candidate = updated
                    else:
                        return failed("transition-failed", iteration, max_change, "backward fixed point did not converge")
                    speed[i] = candidate
        except (ValueError, RuntimeError) as error:
            return failed("transition-domain-failed", iteration, max_change, str(error))
        if boundary == "flying": speed[0] = speed[-1] = min(speed[0], speed[-1])
        elif boundary == "rolling": speed[0] = start_speed
        else: speed[0] = 0.
        max_change = float(np.max(np.abs(speed-previous)))
        if max_change <= tolerance:
            break
    else:
        return failed("not-converged", maximum_iterations, max_change, "outer profile sweeps exhausted")

    # Verify every achieved segment against the same signed physical limits.
    for i, ds in enumerate(segment):
        vi, vj = float(speed[i]), float(speed[i+1])
        if vi + vj <= 0.0:
            return failed("launch-failed", iteration, max_change, "first segment has no positive speed")
        a = (vj*vj-vi*vi)/(2.*ds)
        k = .5*(curvature[i]+curvature[i+1])
        verify_speeds = ((math.sqrt(.5*(vi*vi+vj*vj)),) if segment_limit_sampling == "midpoint"
                         else (vi, vj, math.sqrt(.5*(vi*vi+vj*vj))))
        verify_curvatures = curvature_samples[i] if curvature_samples is not None else (k,)
        checked = [limits(v, k_query) for v in verify_speeds for k_query in verify_curvatures]
        lo = max(pair[0] for pair in checked)
        hi = min(pair[1] for pair in checked)
        if a < lo-tolerance or a > hi+tolerance:
            return failed("dynamically-infeasible", iteration, max_change, "segment acceleration outside physical limits")
    elapsed = np.r_[0., np.cumsum(2.*segment/(speed[:-1]+speed[1:]))]
    return CapabilityProfile(distance, speed, elapsed, float(elapsed[-1]), True, "converged",
                             iteration, max_change, reference,
                             {"boundary": boundary, "floor": floor, "reference_kind": "capability-table",
                               **({"transition_method": transition_method,
                                  "root_assumption": "sampled_non_decreasing_energy_residual_not_continuous_certificate",
                                       "segment_limit_sampling": segment_limit_sampling,
                                        **({"segment_curvature_sampling": segment_curvature_sampling, "curvature_samples_per_segment": 6, "geometry_query_stations_kind": "physical_fraction_via_computational_station_map_inverse", "continuous_feasibility_certified": False} if curvature_samples is not None else {})}
                                  if transition_method == "bracketed_energy" else {})})


def matched_profile_replay(trajectory, profile: CapabilityProfile):
    """Convert a converged profile to a replay sample with explicit station map."""
    if profile.reference_distance is None:
        raise ValueError("matched profile replay requires explicit reference_distance")
    from simplified_vehicle_models.solvers.closed_loop_line import ReplaySample
    return ReplaySample(profile.distance, profile.speed, profile.converged, profile.lap_time,
                        reference_distance=profile.reference_distance)


__all__ = ["CapabilityProfile", "capability_profile", "matched_profile_replay"]
