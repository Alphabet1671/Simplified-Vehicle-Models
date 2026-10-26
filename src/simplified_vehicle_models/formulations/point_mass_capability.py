"""Sample the quasi-steady capability of the point-mass vehicle onto a grid.

Every physics value is evaluated by the NumPy point-mass implementation.
"""
from __future__ import annotations

from collections.abc import Mapping
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


#: Control modes a capability surface can be sampled for.
CAPABILITY_MODES = ("ideal", "allocation", "closed_loop")


@dataclass(frozen=True, slots=True)
class CapabilitySurface:
    """Sampled acceleration limits indexed by speed and curvature.

    ``mode``, ``controller_hash``, and ``feasible_mask`` record the provenance
    of a sampled surface.  They default to an ideal surface (mode ``ideal``, no
    controller, every cell feasible) so a six-positional-argument construction,
    and a cache written before these fields existed, stays valid.
    """
    speed: FloatArray
    curvature: FloatArray
    acceleration_max: FloatArray
    acceleration_min: FloatArray
    speed_ceiling: FloatArray
    provenance_hash: str
    mode: str = "ideal"
    controller_hash: str = ""
    feasible_mask: FloatArray | None = None

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
        if not isinstance(self.mode, str) or self.mode not in CAPABILITY_MODES:
            raise ValueError(f"mode must be one of {CAPABILITY_MODES}")
        if not isinstance(self.controller_hash, str):
            raise ValueError("controller_hash must be a string")
        if self.feasible_mask is not None:
            mask = np.array(self.feasible_mask, dtype=bool, copy=True)
            if mask.shape != self.acceleration_max.shape:
                raise ValueError("feasible_mask must match the capability grid")
            mask.setflags(write=False)
            object.__setattr__(self, "feasible_mask", mask)


def _type_tag(value) -> str:
    cls = type(value)
    return f"{cls.__module__}.{cls.__qualname__}"


def _canonical(value):
    if is_dataclass(value):
        return {"__type__": _type_tag(value), "fields": [[f.name, _canonical(getattr(value, f.name))] for f in fields(value)]}
    if isinstance(value, np.ndarray):
        return {"__type__": _type_tag(value), "dtype": str(value.dtype), "shape": list(value.shape), "values": _canonical(value.tolist())}
    if isinstance(value, np.generic):
        return {"__type__": _type_tag(value), "value": _canonical(value.item())}
    if isinstance(value, Mapping):
        pairs = [[_canonical(k), _canonical(v)] for k, v in value.items()]
        pairs.sort(key=lambda pair: json.dumps(pair[0], sort_keys=True, separators=(",", ":"), allow_nan=False))
        return {"__type__": _type_tag(value), "items": pairs}
    if isinstance(value, (tuple, list)):
        return {"__type__": _type_tag(value), "items": [_canonical(v) for v in value]}
    if isinstance(value, float):
        if not math.isfinite(value):
            return {"__type__": _type_tag(value), "value": str(value)}
        return {"__type__": _type_tag(value), "value": value}
    if value is None or isinstance(value, (str, int, bool)):
        return {"__type__": _type_tag(value), "value": value}
    if hasattr(value, "__dict__"):
        return {"__type__": _type_tag(value), "attributes": [[k, _canonical(v)] for k, v in sorted(vars(value).items()) if not k.startswith("_")]}
    slots = []
    for cls in type(value).__mro__:
        declared = getattr(cls, "__slots__", ())
        if isinstance(declared, str):
            declared = (declared,)
        for name in declared:
            if not name.startswith("_") and hasattr(value, name):
                slots.append((name, _canonical(getattr(value, name))))
    if slots:
        return {"__type__": _type_tag(value), "slots": [[name, item] for name, item in sorted(slots)]}
    if hasattr(value, "value") and isinstance(value.value, (str, int, float)):
        return {"__type__": _type_tag(value), "value": _canonical(value.value)}
    raise TypeError(f"cannot canonically serialize capability input {type(value).__qualname__}")


def capability_hash(vehicle) -> str:
    """Hash top-level and nested physics parameters deterministically."""
    return hashlib.sha256(json.dumps(_canonical(vehicle), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _ceiling_speed(vehicle, curvature: float, maximum: float, tolerance: float) -> float:
    """Return highest contact/lateral feasible speed, or acceleration-zero root.

    The feasibility boundary is the cap when reached before a power-reserve
    root. It is backed off into the feasible side by the requested tolerance.
    """
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
    low_residual = residual(low)
    boundary_residual = residual(boundary)
    if boundary_residual > 1.0e-10:
        # Lateral/contact feasibility is the limiting surface before drive reserve.
        return boundary
    if boundary_residual >= -1.0e-10:
        return boundary
    if low_residual <= 0.0:
        # No positive-reserve interval starts at the low endpoint.
        return low
    if boundary <= low:
        return boundary
    # Explicitly establish the sign bracket before invoking Brent. The upper
    # endpoint is inside lateral/contact feasibility, avoiding the zero sentinel.
    if not (low_residual > 0.0 and boundary_residual < 0.0):
        return boundary
    return brentq(residual, low, boundary, xtol=tolerance, rtol=1e-12)


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
    feasible_mask = (
        np.empty((0, 0), dtype=bool) if surface.feasible_mask is None
        else surface.feasible_mask
    )
    np.savez_compressed(destination, speed=surface.speed, curvature=surface.curvature,
        acceleration_max=surface.acceleration_max, acceleration_min=surface.acceleration_min,
        speed_ceiling=surface.speed_ceiling, provenance_hash=np.asarray(surface.provenance_hash),
        mode=np.asarray(surface.mode), controller_hash=np.asarray(surface.controller_hash),
        feasible_mask=feasible_mask)
    return destination


def load_capability_surface(path: str | Path) -> CapabilitySurface:
    with np.load(Path(path), allow_pickle=False) as data:
        keys = set(data.files)
        # Caches written before provenance fields existed are ideal surfaces.
        mode = str(data["mode"]) if "mode" in keys else "ideal"
        controller_hash = str(data["controller_hash"]) if "controller_hash" in keys else ""
        feasible_mask = data["feasible_mask"] if "feasible_mask" in keys else None
        if feasible_mask is not None and feasible_mask.size == 0:
            feasible_mask = None
        return CapabilitySurface(data["speed"], data["curvature"], data["acceleration_max"],
            data["acceleration_min"], data["speed_ceiling"], str(data["provenance_hash"]),
            mode, controller_hash, feasible_mask)
