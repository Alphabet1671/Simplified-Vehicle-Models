"""Deterministic quasi-static yaw-moment capability maps.

The physical builder uses SciPy's HiGHS LP backend (provided by the optional
``forward`` extra). Importing the package does not require SciPy; attempting to
build a map without it gives an actionable error.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.formulations.point_mass import (
    aerodynamic_forces, normal_loads, tire_force_capacities,
)
from simplified_vehicle_models.formulations.point_mass_capability import capability_hash
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters


SOURCE_KINDS = ("physical_no_tv_v1", "physical_tv_v1", "physical_ideal_tv_v1")


def _axis(value, name: str) -> FloatArray:
    array = np.array(value, dtype=float, copy=True)
    if array.ndim != 1 or array.size < 1 or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a nonempty finite one-dimensional grid")
    if array.size > 1 and np.any(np.diff(array) <= 0):
        raise ValueError(f"{name} must be strictly increasing")
    array.setflags(write=False)
    return array


@dataclass(frozen=True, slots=True)
class YawCapabilityGrid:
    speed: FloatArray
    longitudinal_acceleration: FloatArray
    lateral_acceleration: FloatArray

    def __post_init__(self):
        for name in ("speed", "longitudinal_acceleration", "lateral_acceleration"):
            value = _axis(getattr(self, name), name)
            if name == "speed" and np.any(value < 0):
                raise ValueError("speed must be nonnegative")
            object.__setattr__(self, name, value)


@dataclass(frozen=True, slots=True)
class TvLimits:
    max_delta_torque: float
    regen_vectoring_allowed: bool = False
    brake_vectoring_allowed: bool = False

    def __post_init__(self):
        if not math.isfinite(self.max_delta_torque) or self.max_delta_torque < 0:
            raise ValueError("max_delta_torque must be finite and nonnegative")
        if not isinstance(self.regen_vectoring_allowed, bool) or not isinstance(self.brake_vectoring_allowed, bool):
            raise ValueError("vectoring flags must be bool")


@dataclass(frozen=True, slots=True)
class YawCapabilityMap:
    speed: FloatArray
    longitudinal_acceleration: FloatArray
    lateral_acceleration: FloatArray
    yaw_moment_into_turn: FloatArray
    yaw_moment_out_of_turn: FloatArray
    feasible_mask: np.ndarray
    yaw_inertia: float
    source_kind: str
    source_identity: str

    def __post_init__(self):
        for name in ("speed", "longitudinal_acceleration", "lateral_acceleration"):
            value = _axis(getattr(self, name), name)
            object.__setattr__(self, name, value)
        shape = (len(self.speed), len(self.longitudinal_acceleration), len(self.lateral_acceleration))
        for name in ("yaw_moment_into_turn", "yaw_moment_out_of_turn"):
            arr = np.array(getattr(self, name), dtype=float, copy=True)
            if arr.shape != shape or not np.all(np.isfinite(arr)) or np.any(arr < 0):
                raise ValueError(f"{name} must be finite, nonnegative, and have shape {shape}")
            arr.setflags(write=False)
            object.__setattr__(self, name, arr)
        mask = np.array(self.feasible_mask, dtype=bool, copy=True)
        if mask.shape != shape:
            raise ValueError(f"feasible_mask must have shape {shape}")
        if not math.isfinite(self.yaw_inertia) or self.yaw_inertia <= 0:
            raise ValueError("yaw_inertia must be finite and positive")
        if self.source_kind not in SOURCE_KINDS or not self.source_identity:
            raise ValueError("invalid source provenance")
        mask.setflags(write=False)
        object.__setattr__(self, "feasible_mask", mask)

    def steady_envelope(self) -> tuple[FloatArray, FloatArray]:
        """Return (speed, lateral acceleration) samples feasible at ax=0."""
        zero = int(np.argmin(np.abs(self.longitudinal_acceleration)))
        valid = self.feasible_mask[:, zero, :]
        return self.speed, np.where(valid, self.lateral_acceleration[None, :], np.nan)


def _require_linprog():
    try:
        from scipy.optimize import linprog
    except ImportError as exc:
        raise RuntimeError("physical yaw maps require SciPy; install simplified-vehicle-models[forward]") from exc
    return linprog


def _cell_lp(vehicle, speed, ax, ay, tv_limits, direction, *, ideal=False):
    linprog = _require_linprog()
    loads = normal_loads(vehicle, speed, ax, ay)
    if np.any(loads < -1e-9):
        return None
    capx, capy = tire_force_capacities(vehicle, loads)
    _, drag = aerodynamic_forces(vehicle, speed)
    target_x, target_y = vehicle.total_mass * ax + drag, vehicle.total_mass * ay
    # variables: four wheel Fx and two axle Fy. Lateral axle force is shared
    # over its wheels in proportion to normal load.
    n = 6
    aub, bub, aeq, beq = [], [], [], []
    aeq.append([1,1,1,1,0,0]); beq.append(target_x)
    aeq.append([0,0,0,0,1,1]); beq.append(target_y)
    # 16-sided inscribed friction polygon at each tire.
    for i in range(4):
        axle = 0 if i < 2 else 1
        load_sum = max(float(loads[2*axle:2*axle+2].sum()), 1e-30)
        share = float(loads[i] / load_sum)
        for theta in np.arange(16) * (2 * math.pi / 16):
            row = np.zeros(n)
            row[i] = math.cos(theta) / max(float(capx[i]), 1e-30)
            row[4+axle] = share * math.sin(theta) / max(float(capy[i]), 1e-30)
            aub.append(row); bub.append(math.cos(math.pi/16))
    # Wheel drive/brake force constraints (force caps are a conservative
    # reduction of the available motor envelope for this point-mass model).
    drive_shares = None
    drivetrain = vehicle.drivetrain
    if hasattr(drivetrain, "driven_wheels"):
        driven = set(drivetrain.driven_wheels)
        for i, label in enumerate(("fl", "fr", "rl", "rr")):
            if label not in driven:
                row=np.zeros(n); row[i]=1; aeq.append(row); beq.append(0.)
    if tv_limits is None:
        for left, right in ((0,1),(2,3)):
            row=np.zeros(n); row[left]=1; row[right]=-1; aeq.append(row); beq.append(0.)
    else:
        if not ideal:
            torque = tv_limits.max_delta_torque / max(vehicle.tire.effective_radius, 1e-12)
            for left,right in ((0,1),(2,3)):
                row=np.zeros(n); row[left]=1; row[right]=-1
                aub.extend((row,-row)); bub.extend((torque,torque))
    # Moment convention: x-front/rear lateral forces and lateral arms for Fx.
    front_arm = vehicle.wheelbase * (1 - vehicle.front_mass_fraction)
    rear_arm = vehicle.wheelbase * vehicle.front_mass_fraction
    y = vehicle.track_width / 2
    c = np.array([y*direction, -y*direction, y*direction, -y*direction,
                  -front_arm*direction, rear_arm*direction])
    bounds = [(None,None)]*n
    # torque vectoring constraints apply only to powered/allowed direction
    # all cells have physical tire bounds; optimize the signed directional extreme.
    result = linprog(-c, A_ub=np.asarray(aub) if aub else None, b_ub=np.asarray(bub) if bub else None,
                     A_eq=np.asarray(aeq), b_eq=np.asarray(beq), bounds=bounds, method="highs")
    if not result.success:
        return None
    return float(-result.fun)


def build_physical_yaw_map(vehicle: PointMassVehicleParameters, grid: YawCapabilityGrid,
                           tv_limits: TvLimits | None = None, *, yaw_inertia: float) -> YawCapabilityMap:
    """Build directional yaw authority for all requested acceleration cells.

    ``tv_limits=None`` builds equal-left/right no-TV. ``TvLimits`` builds bounded
    physical TV. Pass ``tv_limits`` with zero torque but ``ideal=True`` is not
    exposed; use :func:`build_ideal_tv_yaw_map` for the ideal upper bound.
    """
    return _build(vehicle, grid, tv_limits, yaw_inertia, ideal=False)


def _build(vehicle, grid, tv_limits, yaw_inertia, ideal):
    if not math.isfinite(yaw_inertia) or yaw_inertia <= 0:
        raise ValueError("yaw_inertia must be finite and positive")
    if tv_limits is not None and not isinstance(tv_limits, TvLimits):
        raise TypeError("tv_limits must be TvLimits or None")
    shape=(len(grid.speed),len(grid.longitudinal_acceleration),len(grid.lateral_acceleration))
    into=np.zeros(shape); out=np.zeros(shape); feasible=np.zeros(shape,dtype=bool)
    for i,v in enumerate(grid.speed):
      for j,ax in enumerate(grid.longitudinal_acceleration):
       for k,ay in enumerate(grid.lateral_acceleration):
        # Feasibility requires a zero yaw moment; query both optimization directions.
        positive=_cell_lp(vehicle,float(v),float(ax),float(ay),tv_limits,1.,ideal=ideal)
        negative=_cell_lp(vehicle,float(v),float(ax),float(ay),tv_limits,-1.,ideal=ideal)
        if positive is not None and negative is not None and positive >= -1e-7 and negative >= -1e-7:
            feasible[i,j,k]=True
            into[i,j,k]=max(0.,positive); out[i,j,k]=max(0.,negative)
    kind="physical_no_tv_v1" if tv_limits is None else "physical_ideal_tv_v1" if ideal else "physical_tv_v1"
    identity=capability_hash((vehicle, grid, tv_limits, yaw_inertia, "ideal" if ideal else "physical"))
    return YawCapabilityMap(grid.speed,grid.longitudinal_acceleration,grid.lateral_acceleration,
                            into,out,feasible,yaw_inertia,kind,identity)


def build_ideal_tv_yaw_map(vehicle, grid: YawCapabilityGrid, *, yaw_inertia: float) -> YawCapabilityMap:
    """Build unconstrained left/right torque TV upper bound for comparisons."""
    return _build(vehicle, grid, TvLimits(0.), yaw_inertia, ideal=True)
