"""Sample the quasi-steady capability of the point-mass vehicle onto a grid.

Every physics value is evaluated by the NumPy point-mass implementation.
"""
from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import brentq

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.formulations.point_mass import (
    _equilibrium_limits, acceleration_limits, maximum_acceleration_limit,
    maximum_vehicle_speed, 
)


@dataclass(frozen=True, slots=True)
class CapabilitySurface:
    """Sampled acceleration limits indexed by speed and curvature."""
    speed: FloatArray
    curvature: FloatArray
    acceleration_max: FloatArray
    acceleration_min: FloatArray
    speed_ceiling: FloatArray
    provenance_hash: str

    def __post_init__(self) -> None:
        for name in ("speed", "curvature", "acceleration_max", "acceleration_min", "speed_ceiling"):
            value = np.array(getattr(self, name), dtype=float, copy=True)
            if not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be finite")
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        if self.acceleration_max.shape != (self.speed.size, self.curvature.size):
            raise ValueError("acceleration_max must have shape (speed, curvature)")
        if self.acceleration_min.shape != self.acceleration_max.shape:
            raise ValueError("acceleration_min must match acceleration_max")
        if self.speed_ceiling.shape != self.curvature.shape:
            raise ValueError("speed_ceiling must match curvature")


def _canonical(value):
    if is_dataclass(value):
        return {f.name: _canonical(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, np.ndarray):
        return {"dtype": str(value.dtype), "shape": list(value.shape), "values": value.tolist()}
    if isinstance(value, np.generic):
        return _canonical(value.item())
    if isinstance(value, dict):
        return {str(k): _canonical(value[k]) for k in sorted(value, key=str)}
    if isinstance(value, (tuple, list)):
        return [_canonical(v) for v in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            return str(value)
        return value
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if hasattr(value, "__dict__"):
        return {k: _canonical(v) for k, v in sorted(vars(value).items()) if not k.startswith("_")}
    slots = []
    for cls in type(value).__mro__:
        declared = getattr(cls, "__slots__", ())
        if isinstance(declared, str):
            declared = (declared,)
        for name in declared:
            if not name.startswith("_") and hasattr(value, name):
                slots.append((name, _canonical(getattr(value, name))))
    if slots:
        return {name: item for name, item in sorted(slots)}
    if hasattr(value, "value") and isinstance(value.value, (str, int, float)):
        return {"enum": type(value).__qualname__, "value": _canonical(value.value)}
    raise TypeError(f"cannot canonically serialize capability input {type(value).__qualname__}")


def capability_hash(vehicle) -> str:
    """Hash top-level and nested physics parameters deterministically."""
    return hashlib.sha256(json.dumps(_canonical(vehicle), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _ceiling_speed(vehicle, curvature: float, maximum: float, tolerance: float) -> float:
    """Find the highest zero-longitudinal feasible speed, not a zero sentinel."""
    def feasible(speed: float) -> bool:
        contact, lateral, *_ = _equilibrium_limits(vehicle, speed, curvature, 0.0)
        return contact and lateral

    # Establish a feasible lower endpoint; zero curvature should always permit
    # low speed, while severe curvature may be infeasible even at the floor.
    low = 1.0e-3
    if not feasible(low):
        return 0.0
    if feasible(maximum):
        return maximum
    # Lateral/contact feasibility can fail before the acceleration residual
    # reaches zero. Find that boundary explicitly using equilibrium states.
    lo, hi = low, maximum
    while hi - lo > tolerance:
        mid = (lo + hi) * 0.5
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    boundary = lo
    # Keep the reported ceiling just inside the feasible side of the tolerance
    # bracket so floating point tolerance in the force checks cannot classify it out.
    boundary = max(low, boundary - max(tolerance, 1.0e-7))
    # Feasibility from _equilibrium_limits is lateral/contact only. Acceleration
    # reserve may be zero before this boundary; bracket its zero within feasible
    # endpoints, without mistaking the zero sentinel for a feasible top speed.
    def residual(speed: float) -> float:
        return maximum_acceleration_limit(vehicle, speed, curvature)
    if residual(boundary) <= 1.0e-10:
        return boundary
    if boundary < maximum:
        # A laterally infeasible upper point can yield the max-limit zero
        # sentinel. Its sign cannot be used to infer a root; the cap is boundary.
        if residual(maximum) == 0.0:
            return boundary
        if residual(maximum) > 0.0:
            return boundary
        return brentq(residual, low, boundary, xtol=tolerance, rtol=1e-12)
    if residual(maximum) >= 0.0:
        return maximum
    return brentq(residual, low, maximum, xtol=tolerance, rtol=1e-12)


def build_capability_surface(vehicle, *, curvature_limit: float, speed_points: int = 121,
                             curvature_points: int = 81, minimum_speed: float = 0.5,
                             tolerance: float = 1e-4) -> CapabilitySurface:
    """Sample NumPy point-mass acceleration limits on a regular grid."""
    if not math.isfinite(curvature_limit) or curvature_limit <= 0:
        raise ValueError("curvature_limit must be finite and positive")
    if speed_points < 3 or curvature_points < 3:
        raise ValueError("at least three samples per axis are required")
    if not math.isfinite(minimum_speed) or minimum_speed <= 0 or not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("minimum_speed and tolerance must be finite and positive")
    top = min(float(maximum_vehicle_speed(vehicle)), 60.0)
    if top <= minimum_speed:
        raise ValueError("maximum vehicle speed must exceed minimum_speed")
    speed = np.linspace(minimum_speed, top, speed_points)
    curvature = np.linspace(-curvature_limit, curvature_limit, curvature_points)
    amax = np.empty((speed.size, curvature.size)); amin = np.empty_like(amax)
    ceiling = np.empty(curvature.size)
    for j, k in enumerate(curvature):
        for i, v in enumerate(speed):
            limits = acceleration_limits(vehicle, float(v), float(k))
            amax[i,j] = limits.maximum_acceleration
            amin[i,j] = -limits.maximum_deceleration
        ceiling[j] = _ceiling_speed(vehicle, float(k), top, tolerance)
    return CapabilitySurface(speed, curvature, amax, amin, ceiling, capability_hash(vehicle))


def save_capability_surface(surface: CapabilitySurface, path: str | Path) -> Path:
    destination = Path(path); destination.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(destination, speed=surface.speed, curvature=surface.curvature,
        acceleration_max=surface.acceleration_max, acceleration_min=surface.acceleration_min,
        speed_ceiling=surface.speed_ceiling, provenance_hash=np.asarray(surface.provenance_hash))
    return destination


def load_capability_surface(path: str | Path) -> CapabilitySurface:
    with np.load(Path(path), allow_pickle=False) as data:
        return CapabilitySurface(data["speed"], data["curvature"], data["acceleration_max"],
            data["acceleration_min"], data["speed_ceiling"], str(data["provenance_hash"]))
