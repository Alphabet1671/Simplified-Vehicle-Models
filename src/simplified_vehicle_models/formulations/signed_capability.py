"""Opt-in, physically signed transient capability charts.

This representation deliberately does not share the legacy ``CapabilitySurface``
cache format.  Its first axis is the dimensionless coordinate
``u=(v-speed_floor)/(domain_speed(k)-speed_floor)``, not a speed axis.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Callable

import numpy as np

from simplified_vehicle_models.formulations.point_mass import evaluate_equilibrium, maximum_vehicle_speed
from simplified_vehicle_models.formulations.point_mass_capability import capability_hash

REPRESENTATION = "signed_transient_normalized_v1"


def _readonly(value, *, name: str, ndim: int = 1) -> np.ndarray:
    result = np.array(value, dtype=float, copy=True)
    if result.ndim != ndim or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite {ndim}-dimensional array")
    result.setflags(write=False)
    return result


def _linear(axis: np.ndarray, values: np.ndarray, query: float) -> float:
    if not math.isfinite(query) or query < axis[0] or query > axis[-1]:
        raise ValueError("capability query outside supported physical domain")
    return float(np.interp(query, axis, values))


def _bilinear(values: np.ndarray, xaxis: np.ndarray, yaxis: np.ndarray, x: float, y: float) -> float:
    if x < xaxis[0] or x > xaxis[-1] or y < yaxis[0] or y > yaxis[-1]:
        raise ValueError("capability query outside supported physical domain")
    i = int(np.clip(np.searchsorted(xaxis, x, side="right") - 1, 0, xaxis.size - 2))
    j = int(np.clip(np.searchsorted(yaxis, y, side="right") - 1, 0, yaxis.size - 2))
    tx = (x-xaxis[i])/(xaxis[i+1]-xaxis[i]); ty = (y-yaxis[j])/(yaxis[j+1]-yaxis[j])
    return float((1-tx)*(1-ty)*values[i,j] + tx*(1-ty)*values[i+1,j] + (1-tx)*ty*values[i,j+1] + tx*ty*values[i+1,j+1])


@dataclass(frozen=True, slots=True)
class SignedTransientCapability:
    """Versioned signed transient limits in a shifted normalized-speed chart."""
    u: np.ndarray
    curvature: np.ndarray
    acceleration_max: np.ndarray
    acceleration_min: np.ndarray
    domain_speed_values: np.ndarray
    speed_floor: float
    provenance_hash: str
    build_settings: dict = field(default_factory=dict)
    validation: dict = field(default_factory=dict)
    representation: str = REPRESENTATION

    def __post_init__(self) -> None:
        u = _readonly(self.u, name="u"); curvature = _readonly(self.curvature, name="curvature")
        amax = _readonly(self.acceleration_max, name="acceleration_max", ndim=2)
        amin = _readonly(self.acceleration_min, name="acceleration_min", ndim=2)
        domain = _readonly(self.domain_speed_values, name="domain_speed_values")
        if u.size < 2 or curvature.size < 2 or np.any(np.diff(u) <= 0) or np.any(np.diff(curvature) <= 0):
            raise ValueError("u and curvature must be strictly increasing axes")
        if not np.isclose(u[0], 0.0) or not np.isclose(u[-1], 1.0):
            raise ValueError("u axis must span exactly [0, 1]")
        if amax.shape != (u.size, curvature.size) or amin.shape != amax.shape:
            raise ValueError("signed acceleration arrays must match (u, curvature)")
        if domain.shape != curvature.shape or np.any(domain <= self.speed_floor):
            raise ValueError("domain_speed_values must exceed speed_floor")
        if not math.isfinite(self.speed_floor) or self.speed_floor < 0 or np.any(amin > amax):
            raise ValueError("invalid signed capability bounds")
        if self.representation != REPRESENTATION:
            raise ValueError(f"unsupported signed representation {self.representation!r}")
        for name, value in (("u", u), ("curvature", curvature), ("acceleration_max", amax),
                            ("acceleration_min", amin), ("domain_speed_values", domain)):
            object.__setattr__(self, name, value)

    def domain_speed(self, curvature: float) -> float:
        """Maximum physical speed with sampled signed-domain evidence."""
        return _linear(self.curvature, self.domain_speed_values, float(curvature))

    def normalized_speed(self, speed: float, curvature: float) -> float:
        domain = self.domain_speed(curvature)
        if not math.isfinite(speed) or speed < self.speed_floor or speed > domain:
            raise ValueError("speed outside signed transient capability domain")
        return (float(speed)-self.speed_floor)/(domain-self.speed_floor)

    def limits(self, speed: float, curvature: float) -> tuple[float, float]:
        """Return ``(minimum_acceleration, maximum_acceleration)`` physically."""
        u = self.normalized_speed(speed, curvature)
        return (_bilinear(self.acceleration_min, self.u, self.curvature, u, curvature),
                _bilinear(self.acceleration_max, self.u, self.curvature, u, curvature))

    def validate_physical_query(self, speed: float, curvature: float, acceleration: float | None = None) -> None:
        amin, amax = self.limits(speed, curvature)
        if acceleration is not None and (acceleration < amin or acceleration > amax):
            raise ValueError("acceleration outside signed transient capability bounds")


def signed_feasible_interval(equilibrium: Callable[[float], object], *, acceleration_min: float,
                             acceleration_max: float, scan_points: int = 401,
                             refinement_steps: int = 32) -> tuple[float, float] | None:
    """Find one signed feasible interval by bounded scan and inward bisection.

    Multiple sampled feasible runs are rejected rather than bridged.  A ``None``
    result means no sampled evidence of a feasible interval; it is not a proof of
    physical infeasibility.
    """
    if not (math.isfinite(acceleration_min) and math.isfinite(acceleration_max)
            and acceleration_min < acceleration_max and isinstance(scan_points, int)
            and scan_points >= 3 and isinstance(refinement_steps, int) and refinement_steps > 0):
        raise ValueError("invalid signed interval scan settings")
    scan = np.linspace(acceleration_min, acceleration_max, scan_points)
    def ok(a: float) -> bool:
        value = equilibrium(a)
        return bool(value if isinstance(value, (bool, np.bool_)) else value.is_feasible)
    feasible = np.array([ok(float(a)) for a in scan])
    edges = np.flatnonzero(np.diff(np.r_[False, feasible, False].astype(int)) != 0)
    runs = list(zip(edges[::2], edges[1::2]))
    if not runs:
        return None
    if len(runs) != 1:
        raise ValueError("disconnected signed feasible acceleration set")
    begin, end = runs[0]  # end is exclusive
    lo, hi = float(scan[begin]), float(scan[end-1])
    # Refine only an observed feasible/infeasible bracket; retain the feasible side.
    if begin:
        bad, good = float(scan[begin-1]), lo
        for _ in range(refinement_steps):
            middle = 0.5*(bad+good)
            if ok(middle): good = middle
            else: bad = middle
        lo = good
    if end < scan.size:
        good, bad = hi, float(scan[end])
        for _ in range(refinement_steps):
            middle = 0.5*(good+bad)
            if ok(middle): good = middle
            else: bad = middle
        hi = good
    return lo, hi


def _speed_domain(vehicle, curvature: float, floor: float, upper: float, acceleration_range: tuple[float, float], scan_points: int, refinement_steps: int, speed_scan_points: int) -> float:
    speeds = np.linspace(floor, upper, speed_scan_points)
    exists = []
    for speed in speeds:
        interval = signed_feasible_interval(lambda a: evaluate_equilibrium(vehicle, float(speed), curvature, a), acceleration_min=acceleration_range[0], acceleration_max=acceleration_range[1], scan_points=scan_points, refinement_steps=refinement_steps)
        exists.append(interval is not None)
    runs = np.flatnonzero(np.diff(np.r_[False, exists, False].astype(int)) != 0)
    paired = list(zip(runs[::2], runs[1::2]))
    if len(paired) != 1 or paired[0][0] != 0:
        raise ValueError(f"unsupported disconnected/nonmonotone signed speed domain at curvature {curvature:.12g}")
    stop = paired[0][1]
    if stop == len(speeds):
        return float(speeds[-1])
    # Refine the final existence bracket and retain a feasible speed.
    good, bad = float(speeds[stop-1]), float(speeds[stop])
    for _ in range(refinement_steps):
        middle = 0.5*(good+bad)
        interval = signed_feasible_interval(lambda a: evaluate_equilibrium(vehicle, middle, curvature, a), acceleration_min=acceleration_range[0], acceleration_max=acceleration_range[1], scan_points=scan_points, refinement_steps=refinement_steps)
        if interval is None: bad = middle
        else: good = middle
    return good


def build_signed_transient_capability(vehicle, *, curvature: np.ndarray, u: np.ndarray | None = None,
                                       speed_floor: float = 0.5, maximum_speed: float | None = None,
                                       acceleration_range: tuple[float, float] = (-30.0, 10.0),
                                       scan_points: int = 401, refinement_steps: int = 32,
                                       speed_scan_points: int = 81, interior_subdivisions: int = 2,
                                       endpoint_buffer: float = 1.0e-4, domain_backoff: float = 1.0e-3,
                                       refinement_attempts: int = 3) -> SignedTransientCapability:
    """Build a conservatively reconstructed signed chart from original physics.

    Every cell's full edge/interior stencil contributes a common inner interval.
    That requirement is propagated to its four shared vertices, so reconstructed
    bilinear bounds cannot be wider than the live interval at a sampled probe.
    Failed cells refine the true axes/domain then explicitly refuse if unresolved.
    """
    curvature = _readonly(curvature, name="curvature")
    u = np.linspace(0., 1., 9) if u is None else _readonly(u, name="u")
    if curvature.size < 2 or u.size < 2 or np.any(np.diff(curvature) <= 0) or np.any(np.diff(u) <= 0) or not np.isclose(u[0], 0) or not np.isclose(u[-1], 1):
        raise ValueError("curvature and u must be increasing axes with u=[0,1]")
    if (not math.isfinite(speed_floor) or speed_floor <= 0 or speed_scan_points < 3
            or interior_subdivisions < 1 or not math.isfinite(endpoint_buffer)
            or endpoint_buffer < 0 or not math.isfinite(domain_backoff) or domain_backoff < 0
            or not isinstance(refinement_attempts, int) or refinement_attempts < 0):
        raise ValueError("invalid signed capability build domain")
    hardware_top = min(float(maximum_vehicle_speed(vehicle)), 60.0)
    top = hardware_top if maximum_speed is None else float(maximum_speed)
    if not math.isfinite(top) or top <= speed_floor:
        raise ValueError("maximum_speed must exceed speed_floor")
    if top > hardware_top + 1.0e-12:
        raise ValueError("maximum_speed exceeds declared vehicle mechanical limit")

    axes_u, axes_k, total_samples, last_failure = u, curvature, 0, None
    for attempt in range(refinement_attempts + 1):
        # Shared cell edges are deliberately queried more than once.  Cache the
        # original-physics result by exact float inputs; it changes cost only,
        # never the sampled evidence.
        interval_cache = {}
        def interval(speed, k):
            key = (float(speed), float(k))
            if key not in interval_cache:
                interval_cache[key] = signed_feasible_interval(
                    lambda a: evaluate_equilibrium(vehicle, key[0], key[1], a),
                    acceleration_min=acceleration_range[0], acceleration_max=acceleration_range[1],
                    scan_points=scan_points, refinement_steps=refinement_steps)
            return interval_cache[key]
        raw_domains = np.array([_speed_domain(vehicle, float(k), speed_floor, top, acceleration_range,
                                               scan_points, refinement_steps, speed_scan_points) for k in axes_k])
        backoff = domain_backoff * (attempt + 1)
        domains = raw_domains - backoff
        if np.any(domains <= speed_floor):
            raise ValueError("signed speed domain vanished under requested conservative backoff")
        lows = np.empty((axes_u.size, axes_k.size)); highs = np.empty_like(lows)
        bad = None
        for j, k in enumerate(axes_k):
            for i, coordinate in enumerate(axes_u):
                live = interval(speed_floor + coordinate*(domains[j]-speed_floor), k)
                if live is None or live[1]-live[0] <= 2*endpoint_buffer:
                    bad = {"kind":"vertex", "u":float(coordinate), "curvature":float(k)}; break
                lows[i,j], highs[i,j] = live[0]+endpoint_buffer, live[1]-endpoint_buffer
            if bad: break
        if bad is None:
            req_low, req_high = lows.copy(), highs.copy()
            probes = 0
            for i in range(axes_u.size-1):
                for j in range(axes_k.size-1):
                    cell_low, cell_high = [], []
                    for uu in np.linspace(axes_u[i], axes_u[i+1], interior_subdivisions+2):
                        for kk in np.linspace(axes_k[j], axes_k[j+1], interior_subdivisions+2):
                            domain = float(np.interp(kk, axes_k, domains))
                            speed = speed_floor + uu*(domain-speed_floor)
                            live = interval(speed, kk); probes += 1
                            if live is None:
                                bad = {"kind":"domain", "u":float(uu), "curvature":float(kk), "speed":float(speed)}; break
                            cell_low.append(live[0]+endpoint_buffer); cell_high.append(live[1]-endpoint_buffer)
                        if bad: break
                    if bad: break
                    common_low, common_high = max(cell_low), min(cell_high)
                    if common_low > common_high:
                        bad = {"kind":"empty_common_interval", "cell":[i,j], "lower":common_low, "upper":common_high}; break
                    req_low[i:i+2,j:j+2] = np.maximum(req_low[i:i+2,j:j+2], common_low)
                    req_high[i:i+2,j:j+2] = np.minimum(req_high[i:i+2,j:j+2], common_high)
                if bad: break
            total_samples += probes
            if bad is None and np.all(req_low <= req_high):
                settings = {"acceleration_range":list(acceleration_range), "scan_points":scan_points,
                    "refinement_steps":refinement_steps, "speed_scan_points":speed_scan_points,
                    "interior_subdivisions":interior_subdivisions, "maximum_speed":top,
                    "endpoint_buffer":endpoint_buffer, "domain_backoff":domain_backoff,
                    "refinement_attempt":attempt, "refinement_attempts":refinement_attempts,
                    "actual_u":axes_u.tolist(), "actual_curvature":axes_k.tolist(),
                    "actual_domain_backoff":backoff}
                chart = SignedTransientCapability(axes_u, axes_k, req_high, req_low, domains, speed_floor, capability_hash(vehicle), settings)
                checks, failures = 0, []
                for i in range(axes_u.size-1):
                    for j in range(axes_k.size-1):
                        for uu in np.linspace(axes_u[i], axes_u[i+1], interior_subdivisions+2):
                            for kk in np.linspace(axes_k[j], axes_k[j+1], interior_subdivisions+2):
                                speed = speed_floor + uu*(chart.domain_speed(float(kk))-speed_floor)
                                low, high = chart.limits(speed, float(kk)); checks += 1
                                if not evaluate_equilibrium(vehicle, speed, float(kk), low).is_feasible or not evaluate_equilibrium(vehicle, speed, float(kk), high).is_feasible:
                                    failures.append({"u":float(uu), "curvature":float(kk), "speed":float(speed), "limits":[low,high]})
                if not failures:
                    loss = float(np.mean((highs-lows)-(req_high-req_low)))
                    return SignedTransientCapability(axes_u, axes_k, req_high, req_low, domains, speed_floor, chart.provenance_hash, settings,
                        {"sampled":True, "construction_stencil_count":total_samples, "validation_stencil_count":checks,
                         "live_interval_evaluations":len(interval_cache), "failures":[], "mean_interval_conservative_loss":loss,
                         "claim":"sampled certification, not continuum proof"})
                bad = {"kind":"revalidation", **failures[0]}
            elif bad is None:
                bad = {"kind":"propagated_empty_vertex"}
        last_failure = bad
        if attempt < refinement_attempts:
            axes_u = np.sort(np.unique(np.r_[axes_u, .5*(axes_u[:-1]+axes_u[1:])]))
            axes_k = np.sort(np.unique(np.r_[axes_k, .5*(axes_k[:-1]+axes_k[1:])]))
    raise ValueError("signed chart refused after conservative refinement: " + json.dumps(last_failure, sort_keys=True))

def save_signed_transient_capability(chart: SignedTransientCapability, path: str | Path) -> Path:
    """Serialize a distinct, versioned archive without touching legacy caches."""
    destination = Path(path); destination.parent.mkdir(parents=True, exist_ok=True)
    # Do not overwrite any cache, including a same-version signed archive.
    with destination.open("xb") as handle:
        np.savez_compressed(handle, representation=np.asarray(chart.representation), u=chart.u, curvature=chart.curvature,
            acceleration_max=chart.acceleration_max, acceleration_min=chart.acceleration_min,
            domain_speed_values=chart.domain_speed_values, speed_floor=np.asarray(chart.speed_floor),
            provenance_hash=np.asarray(chart.provenance_hash), build_settings=np.asarray(json.dumps(chart.build_settings, sort_keys=True)), validation=np.asarray(json.dumps(chart.validation, sort_keys=True)))
    return destination


def load_signed_transient_capability(path: str | Path) -> SignedTransientCapability:
    with np.load(Path(path), allow_pickle=False) as data:
        if "representation" not in data.files or str(data["representation"]) != REPRESENTATION:
            raise ValueError("not a signed transient capability archive")
        return SignedTransientCapability(data["u"], data["curvature"], data["acceleration_max"], data["acceleration_min"], data["domain_speed_values"], float(data["speed_floor"]), str(data["provenance_hash"]), json.loads(str(data["build_settings"])), json.loads(str(data["validation"])))


def symbolic_lookup(chart: SignedTransientCapability, prefix: str = "signed"):
    """Return CasADi ``(amin, amax, domain)`` callables for physical ``(v,k)``."""
    import casadi as ca
    domain = ca.interpolant(prefix+"_domain", "linear", [chart.curvature.tolist()], chart.domain_speed_values.tolist())
    amin_grid = ca.interpolant(prefix+"_amin", "linear", [chart.u.tolist(), chart.curvature.tolist()], chart.acceleration_min.ravel(order="F").tolist())
    amax_grid = ca.interpolant(prefix+"_amax", "linear", [chart.u.tolist(), chart.curvature.tolist()], chart.acceleration_max.ravel(order="F").tolist())
    def limits(speed, curvature):
        cap = domain(curvature); coordinate = (speed-chart.speed_floor)/(cap-chart.speed_floor)
        # Clamp lookup coordinates solely to prevent an extrapolated table value
        # during rejected NLP iterates; callers add physical-domain constraints.
        coordinate = ca.fmin(1.0, ca.fmax(0.0, coordinate))
        return amin_grid(ca.vertcat(coordinate, curvature)), amax_grid(ca.vertcat(coordinate, curvature)), cap
    return limits
