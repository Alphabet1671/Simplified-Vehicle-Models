"""Exact algebraic fast path for rear-driven, no-TV yaw LP cells.

The current no-TV LP fixes left/right longitudinal force equality.  With exactly
both rear wheels driven, longitudinal equilibrium fixes each wheel force, while
only front and rear axle lateral totals remain.  Intersecting the same 16-sided
per-tyre polygon rows used by :mod:`yaw_capability` produces an exact one-
dimensional interval, not a circular-tyre approximation.  Other topologies and
TV configurations fall back to ``direct_yaw_cell`` unchanged.  Near an active
algebraic boundary this module also falls back to the canonical LP rather than
changing its numerical feasible set with reduced-path tolerances.
"""
from __future__ import annotations

import math
import numpy as np

from simplified_vehicle_models.formulations.point_mass import (
    aerodynamic_forces, normal_loads, tire_force_capacities,
)
from simplified_vehicle_models.formulations.yaw_capability import (
    DirectYawCell, direct_yaw_cell,
)
from simplified_vehicle_models.formulations.yaw_map_domain import ProbeStatus
from simplified_vehicle_models.powertrain.assembly import FullDrivetrainRuntime, maximum_drive_force
from simplified_vehicle_models.powertrain.energy_storage import StorageState

_EPS = 1e-30
_POLYGON_COSINE = math.cos(math.pi / 16.0)
# This is a fallback band, never a feasibility relaxation.  The canonical LP
# decides cells whose reduced subtraction/division is near an active boundary.
_AMBIGUOUS_FORCE = 1e-7


class _ReducedAmbiguous(RuntimeError):
    pass


def _rear_only_no_tv(vehicle, tv_limits) -> bool:
    runtime = getattr(vehicle, "drivetrain", None)
    return (tv_limits is None and isinstance(runtime, FullDrivetrainRuntime)
            and set(runtime.drivetrain.transmission.driven_wheels) == {"rl", "rr"})


def _interval_for_axle(loads, capx, capy, wheel_indices, fx) -> tuple[float, float] | None:
    """Exact lateral-total interval from the current 16 polygon rows.

    No nonzero ``q`` row is discarded.  Near a row contact we defer to HiGHS,
    avoiding a reduced arithmetic tolerance that could enlarge or shrink the LP.
    """
    total_load = max(float(np.asarray(loads)[list(wheel_indices)].sum()), _EPS)
    lower, upper = -math.inf, math.inf
    for index in wheel_indices:
        share = float(loads[index] / total_load)
        for theta in np.arange(16) * (2.0 * math.pi / 16.0):
            q = math.sin(float(theta)) * share / max(float(capy[index]), _EPS)
            rhs = _POLYGON_COSINE - math.cos(float(theta)) * fx[index] / max(float(capx[index]), _EPS)
            if q == 0.0:
                if rhs < 0.0:
                    return None
                if abs(rhs) <= _AMBIGUOUS_FORCE:
                    raise _ReducedAmbiguous("polygon longitudinal row near active")
            elif q > 0.0:
                candidate = rhs / q
                if math.isfinite(upper) and abs(candidate - upper) <= _AMBIGUOUS_FORCE:
                    raise _ReducedAmbiguous("polygon lateral upper rows near active")
                upper = min(upper, candidate)
            else:
                candidate = rhs / q
                if math.isfinite(lower) and abs(candidate - lower) <= _AMBIGUOUS_FORCE:
                    raise _ReducedAmbiguous("polygon lateral lower rows near active")
                lower = max(lower, candidate)
    if lower > upper:
        return None
    if math.isfinite(lower) and math.isfinite(upper) and upper - lower <= _AMBIGUOUS_FORCE:
        raise _ReducedAmbiguous("polygon lateral interval near empty")
    return lower, upper


def _reduced_rear_only_cell(vehicle, speed, ax, ay) -> DirectYawCell:
    """Solve the rear-drive no-TV polygon LP by its exact one-axis reduction."""
    loads = normal_loads(vehicle, speed, ax, ay)
    # Preserve the canonical LP's own normal-load threshold exactly.
    if np.any(loads < -1e-9):
        return DirectYawCell(ProbeStatus.INFEASIBLE)
    capx, capy = tire_force_capacities(vehicle, loads)
    _, drag = aerodynamic_forces(vehicle, speed)
    target_x = float(vehicle.total_mass * ax + drag)
    target_y = float(vehicle.total_mass * ay)
    fx = np.zeros(4, dtype=float)
    if target_x >= 0.0:
        runtime = vehicle.drivetrain
        from simplified_vehicle_models.formulations.point_mass import _default_motor_shares
        shares = _default_motor_shares(runtime.powertrain.power_plant, runtime.drivetrain.transmission)
        drive = maximum_drive_force(runtime.powertrain, runtime.drivetrain, speed,
            StorageState(runtime.powertrain.energy_storage.initial_state_of_charge),
            effective_radius=vehicle.tire.effective_radius, wheel_shares=shares).maximum_drive_force
        fx[2:] = target_x / 2.0
        # Do not broaden the drive inequality.  Near equality use canonical LP.
        difference = target_x - drive
        if abs(difference) <= _AMBIGUOUS_FORCE:
            raise _ReducedAmbiguous("rear drive limit near active")
        if difference > 0.0:
            return DirectYawCell(ProbeStatus.INFEASIBLE)
    else:
        braking = -target_x
        fx[:2] = -float(vehicle.brake_bias) * braking / 2.0
        fx[2:] = -(1.0 - float(vehicle.brake_bias)) * braking / 2.0
    if not np.all(np.isfinite(fx)):
        raise _ReducedAmbiguous("nonfinite reduced longitudinal force")
    front = _interval_for_axle(loads, capx, capy, (0, 1), fx)
    rear = _interval_for_axle(loads, capx, capy, (2, 3), fx)
    if front is None or rear is None:
        return DirectYawCell(ProbeStatus.INFEASIBLE)
    front_low = max(front[0], target_y - rear[1])
    front_high = min(front[1], target_y - rear[0])
    if front_low > front_high:
        return DirectYawCell(ProbeStatus.INFEASIBLE)
    if front_high - front_low <= _AMBIGUOUS_FORCE:
        raise _ReducedAmbiguous("combined lateral interval near empty")
    rear_arm = float(vehicle.wheelbase * vehicle.front_mass_fraction)
    maximum = rear_arm * target_y - float(vehicle.wheelbase) * front_low
    minimum = rear_arm * target_y - float(vehicle.wheelbase) * front_high
    if not (math.isfinite(maximum) and math.isfinite(minimum)):
        raise _ReducedAmbiguous("nonfinite reduced yaw interval")
    # Retain the canonical direct evaluator's zero-containment tolerance exactly.
    if maximum < -1e-7 or -minimum < -1e-7:
        return DirectYawCell(ProbeStatus.INFEASIBLE)
    return DirectYawCell(ProbeStatus.FEASIBLE, max(0.0, maximum), max(0.0, -minimum))


def direct_reduced_cell(vehicle, speed, ax, ay, tv_limits=None, *, ideal=False) -> DirectYawCell:
    """Return reduced rear/no-TV result, with canonical LP on ambiguous cells."""
    if ideal or not _rear_only_no_tv(vehicle, tv_limits):
        return direct_yaw_cell(vehicle, speed, ax, ay, tv_limits, ideal=ideal)
    if not all(math.isfinite(float(value)) for value in (speed, ax, ay)) or speed < 0.0:
        raise ValueError("direct yaw query requires finite accelerations and nonnegative speed")
    try:
        return _reduced_rear_only_cell(vehicle, float(speed), float(ax), float(ay))
    except _ReducedAmbiguous:
        return direct_yaw_cell(vehicle, speed, ax, ay, tv_limits, ideal=ideal)
    except Exception:
        # Any unexpected reduced-model error is delegated to canonical LP, whose
        # unresolved status remains the single numerical truth.
        return direct_yaw_cell(vehicle, speed, ax, ay, tv_limits, ideal=ideal)
