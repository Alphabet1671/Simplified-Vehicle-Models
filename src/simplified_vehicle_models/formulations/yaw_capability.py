"""Deterministic quasi-static yaw-moment capability maps.

The physical builder uses SciPy's HiGHS LP backend (provided by the optional
``forward`` extra). Importing the package does not require SciPy; attempting to
build a map without it gives an actionable error.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.formulations.point_mass import (
    aerodynamic_forces, normal_loads, tire_force_capacities,
)
from simplified_vehicle_models.formulations.point_mass_capability import capability_hash
from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
from simplified_vehicle_models.powertrain.assembly import FullDrivetrainRuntime
from simplified_vehicle_models.powertrain.motor_envelope import MotorEnvelopeParameters
from simplified_vehicle_models.powertrain.energy_storage import StorageState
from simplified_vehicle_models.powertrain.assembly import maximum_drive_force


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
    # Maximum left/right hydraulic wheel-force difference, in N. Independent
    # of motor torque authority; zero means no hydraulic brake vectoring.
    max_hydraulic_force_difference: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.max_delta_torque) or self.max_delta_torque < 0:
            raise ValueError("max_delta_torque must be finite and nonnegative")
        if (not math.isfinite(self.max_hydraulic_force_difference)
                or self.max_hydraulic_force_difference < 0):
            raise ValueError("max_hydraulic_force_difference must be finite and nonnegative")
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
            if name == "speed" and np.any(value < 0):
                raise ValueError("speed must be nonnegative")
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
        if (self.source_kind not in SOURCE_KINDS or not isinstance(self.source_identity, str)
                or not self.source_identity.strip()):
            raise ValueError("invalid source provenance")
        mask.setflags(write=False)
        object.__setattr__(self, "feasible_mask", mask)

    def steady_envelope(self) -> tuple[FloatArray, FloatArray]:
        """Return (speed, lateral acceleration) samples feasible at exact ax=0."""
        matches = np.flatnonzero(np.isclose(self.longitudinal_acceleration, 0.0, atol=1e-12, rtol=0.0))
        if matches.size == 0:
            raise ValueError("steady envelope requires an exact zero longitudinal-acceleration grid cell")
        zero = int(matches[0])
        valid = self.feasible_mask[:, zero, :]
        return self.speed, np.where(valid, self.lateral_acceleration[None, :], np.nan)


def _require_linprog():
    try:
        from scipy.optimize import linprog
    except ImportError as exc:
        raise RuntimeError("physical yaw maps require SciPy; install simplified-vehicle-models[forward]") from exc
    return linprog


class YawCapabilityNumericalError(RuntimeError):
    """A direct capability LP was unresolved, rather than physically infeasible."""


def _cell_lp(vehicle, speed, ax, ay, tv_limits, direction, *, ideal=False, strict_status=False):
    linprog = _require_linprog()
    loads = normal_loads(vehicle, speed, ax, ay)
    if np.any(loads < -1e-9):
        return None
    capx, capy = tire_force_capacities(vehicle, loads)
    _, drag = aerodynamic_forces(vehicle, speed)
    target_x, target_y = vehicle.total_mass * ax + drag, vehicle.total_mass * ay
    # Variables: four nonnegative drive forces, four nonnegative regen
    # magnitudes, four nonnegative hydraulic brake magnitudes, two axle Fy.
    # Net wheel force = drive - regen - hydraulic.
    n = 14
    aub, bub, aeq, beq = [], [], [], []
    aeq.append([1]*4 + [-1]*8 + [0,0]); beq.append(target_x)
    aeq.append([0]*12 + [1,1]); beq.append(target_y)
    # 16-sided inscribed friction polygon at each tire.
    for i in range(4):
        axle = 0 if i < 2 else 1
        load_sum = max(float(loads[2*axle:2*axle+2].sum()), 1e-30)
        share = float(loads[i] / load_sum)
        for theta in np.arange(16) * (2 * math.pi / 16):
            row = np.zeros(n)
            row[i] = math.cos(theta) / max(float(capx[i]), 1e-30)
            row[4+i] = -math.cos(theta) / max(float(capx[i]), 1e-30)
            row[8+i] = -math.cos(theta) / max(float(capx[i]), 1e-30)
            row[12+axle] = share * math.sin(theta) / max(float(capy[i]), 1e-30)
            aub.append(row); bub.append(math.cos(math.pi/16))
    # Resolve driven wheels from the assembled transmission (or legacy model).
    drivetrain = vehicle.drivetrain
    runtime = isinstance(drivetrain, FullDrivetrainRuntime)
    if not runtime:
        raise TypeError("physical yaw maps require a FullDrivetrainRuntime with explicit driven wheels")
    driven = set(drivetrain.drivetrain.transmission.driven_wheels)
    for i, label in enumerate(("fl", "fr", "rl", "rr")):
        if label not in driven:
            for offset in (0, 4):
                row=np.zeros(n); row[offset+i]=1; aeq.append(row); beq.append(0.)
    # Regen is conservatively disabled until a charge-power limit is modeled.
    for i in range(4):
        row=np.zeros(n); row[4+i]=1; aeq.append(row); beq.append(0.)
    # Recover conservative total drive limit for every acceleration cell.
    if runtime:
        from simplified_vehicle_models.formulations.point_mass import _default_motor_shares
        shares = _default_motor_shares(drivetrain.powertrain.power_plant, drivetrain.drivetrain.transmission)
        drive_limit = maximum_drive_force(drivetrain.powertrain, drivetrain.drivetrain, speed,
            StorageState(drivetrain.powertrain.energy_storage.initial_state_of_charge),
            effective_radius=vehicle.tire.effective_radius, wheel_shares=shares).maximum_drive_force
    # Each driven wheel receives an equal conservative share of aggregate
    # motor torque/power capability; no motoring force is permitted elsewhere.
    share = 1.0 / len(driven)
    for i in range(4):
        row=np.zeros(n); row[i]=1.; aub.append(row); bub.append(drive_limit*share if ("fl","fr","rl","rr")[i] in driven else 0.)
    for i in range(4):
        row=np.zeros(n); row[4+i]=1.; aeq.append(row); beq.append(0.)
    # Hydraulic brakes act on every wheel, driven or not; the axle split is
    # fixed by the brake-bias equality below. (Restricting hydraulic force to
    # driven wheels while also enforcing a nonzero front bias forced all
    # braking to zero on rear-driven vehicles.)
    # Nonnegative motor drive, zero regen, nonnegative hydraulic braking.
    bounds=[(0,None)]*12 + [(None,None)]*2
    braking = target_x < 0.0
    # Avoid simultaneous motoring and hydraulic braking in this LP cell.
    if target_x >= 0.0:
        for i in range(4):
            row=np.zeros(n); row[8+i]=1.; aeq.append(row); beq.append(0.)
    else:
        for i in range(4):
            row=np.zeros(n); row[i]=1.; aeq.append(row); beq.append(0.)
    # Preserve front/rear axle-total brake-bias equality for every physical
    # cell, including hydraulic TV cells. Hydraulic TV may only redistribute
    # braking left/right within each axle under its separate force-difference cap.
    row=np.zeros(n); row[8:10]=1-vehicle.brake_bias; row[10:12]=-vehicle.brake_bias
    aeq.append(row); beq.append(0.)
    # Each axle's hydraulic braking is left/right equal absent hydraulic TV.
    # When enabled, the dedicated force-difference cap below is applied.
    if not (tv_limits is not None and tv_limits.brake_vectoring_allowed and not ideal):
        for left,right in ((0,1),(2,3)):
            row=np.zeros(n); row[8+left]=1; row[8+right]=-1; aeq.append(row); beq.append(0.)
    # Regen vectoring flag does not unlock hydraulic force; regen is currently zero.
    # Motor torque vectoring is bounded by max_delta_torque; absent a TV
    # configuration, motor force remains equal left/right. Hydraulic vectoring
    # is independently enabled and cannot be authorized by regen_vectoring.
    torque = (tv_limits.max_delta_torque / max(vehicle.tire.effective_radius, 1e-12)
              if tv_limits is not None else 0.0)
    hydraulic_difference = (tv_limits.max_hydraulic_force_difference
                            if tv_limits is not None else 0.0)
    for left,right in ((0,1),(2,3)):
        if tv_limits is None:
            row=np.zeros(n); row[left]=1; row[right]=-1; aeq.append(row); beq.append(0.)
        elif not ideal:
            row=np.zeros(n); row[left]=1; row[right]=-1
            aub.extend((row,-row)); bub.extend((torque,torque))
        if not (tv_limits is not None and tv_limits.brake_vectoring_allowed and not ideal):
            row=np.zeros(n); row[8+left]=1; row[8+right]=-1; aeq.append(row); beq.append(0.)
        elif not ideal:
            row=np.zeros(n); row[8+left]=1; row[8+right]=-1
            aub.extend((row,-row)); bub.extend((hydraulic_difference,hydraulic_difference))
    # Moment convention: net Fx = drive - regen - hydraulic. Lateral force
    # moment uses the two axle-force variables and SAE x-axis lever arms.
    front_arm = vehicle.wheelbase * (1 - vehicle.front_mass_fraction)
    rear_arm = vehicle.wheelbase * vehicle.front_mass_fraction
    y = vehicle.track_width / 2
    lateral_arms = np.array([y, -y, y, -y])
    c = np.concatenate((lateral_arms, -lateral_arms, -lateral_arms,
                        [-front_arm, rear_arm])) * direction
    # Torque vectoring constraints apply only to permitted components; all
    # components remain subject to tire friction polygons.
    result = linprog(-c, A_ub=np.asarray(aub) if aub else None, b_ub=np.asarray(bub) if bub else None,
                     A_eq=np.asarray(aeq), b_eq=np.asarray(beq), bounds=bounds, method="highs")
    if not result.success:
        if strict_status and getattr(result, "status", None) != 2:
            raise YawCapabilityNumericalError(
                f"yaw LP unresolved at {(speed, ax, ay)}: status={getattr(result, 'status', None)} "
                f"{getattr(result, 'message', '')}")
        return None
    value = float(-result.fun)
    if not math.isfinite(value):
        if strict_status:
            raise YawCapabilityNumericalError(f"yaw LP returned nonfinite objective at {(speed, ax, ay)}")
        return None
    return value


@dataclass(frozen=True, slots=True)
class DirectYawCell:
    status: object
    into_turn: float | None = None
    out_of_turn: float | None = None
    detail: str = ""


def direct_yaw_cell(vehicle, speed, ax, ay, tv_limits=None, *, ideal=False):
    """Direct LP evidence, keeping numerical failure distinct from infeasibility."""
    from simplified_vehicle_models.formulations.yaw_map_domain import ProbeStatus
    if not all(math.isfinite(float(x)) for x in (speed, ax, ay)) or speed < 0:
        raise ValueError("direct yaw query requires finite accelerations and nonnegative speed")
    try:
        positive = _cell_lp(vehicle, speed, ax, ay, tv_limits, 1., ideal=ideal, strict_status=True)
        negative = _cell_lp(vehicle, speed, ax, ay, tv_limits, -1., ideal=ideal, strict_status=True)
    except YawCapabilityNumericalError as error:
        return DirectYawCell(ProbeStatus.UNRESOLVED, detail=str(error))
    if positive is None or negative is None or min(positive, negative) < -1e-7:
        return DirectYawCell(ProbeStatus.INFEASIBLE)
    return DirectYawCell(ProbeStatus.FEASIBLE, max(0., positive), max(0., negative))


def direct_yaw_probe(vehicle, speed, ax, ay, tv_limits=None, *, ideal=False):
    from simplified_vehicle_models.formulations.yaw_map_domain import ProbeResult
    cell = direct_yaw_cell(vehicle, speed, ax, ay, tv_limits, ideal=ideal)
    return ProbeResult(cell.status, cell.detail)


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
        positive=_cell_lp(vehicle,float(v),float(ax),float(ay),tv_limits,1.,ideal=ideal,strict_status=True)
        negative=_cell_lp(vehicle,float(v),float(ax),float(ay),tv_limits,-1.,ideal=ideal,strict_status=True)
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
