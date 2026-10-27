"""Compile the planar 7-DOF vehicle and evaluate its quasi-static loads.

The model has seven mechanical degrees of freedom:

- body longitudinal motion ``v_x``;
- body lateral motion ``v_y``;
- body yaw rate ``r``;
- four wheel angular speeds.

The body has no heave, roll, or pitch state. The vertical tire loads are
therefore quasi static. They depend on the longitudinal and lateral
acceleration at the same instant.

Every axis uses the SAE vehicle frame. See ``core.conventions``.
A positive lateral acceleration points to the right.
A positive yaw rate turns the vehicle to the right.
Track curvature is positive in a right turn.

Wheel order is always FL, FR, RL, RR.
"""

from __future__ import annotations

import csv
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import TypeVar, cast

import numpy as np

from simplified_vehicle_models.aero import fixed_aero
from simplified_vehicle_models.controls.brakes import PedalRatioCurve
from simplified_vehicle_models.controls.steering import AckermannTable
from simplified_vehicle_models.compiler.adapters import ResolutionResult
from simplified_vehicle_models.compiler.capabilities import (
    AERO_LOADS,
    DRIVE_FORCE_LIMIT,
    STEADY_COMBINED_SLIP,
)
from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.parameters.aero import AeroParameters
from simplified_vehicle_models.parameters.drivetrain import DrivetrainParameters
from simplified_vehicle_models.parameters.tire import UniTireSimpleParameters
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.powertrain.energy_storage import EnergyStorageParameters
from simplified_vehicle_models.powertrain.motor_envelope import (
    TORQUE_ENVELOPE_SUBTYPE,
    MotorEnvelopeParameters,
)
from simplified_vehicle_models.suspension import quasi_static
from simplified_vehicle_models.tires.unitire_simple import (
    evaluate_unitire,
    loaded_radius,
)
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    require_closed_unit_interval,
    require_nonnegative,
    require_open_unit_interval,
    require_positive,
    require_signed_brake_bias,
    required_parameters,
)


PLANAR_7DOF_REQUESTED_CAPABILITIES: tuple[str, ...] = (
    STEADY_COMBINED_SLIP,
    AERO_LOADS,
    DRIVE_FORCE_LIMIT,
)

PLANAR_7DOF_VEHICLE_PARAMETER_UNITS: dict[str, str] = {
    "total_mass": "kg",
    "mass_without_driver": "kg",
    "driver_mass": "kg",
    "wheelbase": "m",
    "track_width": "m",
    "cg_height": "m",
    "front_mass_fraction": "1",
    "lateral_load_transfer_distribution": "1",
    "yaw_radius_of_gyration": "m",
    "wheel_inertia": "kg*m^2",
    "steer_ratio": "1",
    "steering_maximum_angle": "rad",
    "steering_maximum_rate": "rad/s",
    "rack_travel_per_revolution": "m",
    "steer_arm_length": "m",
    "caster_angle": "rad",
    "kingpin_inclination_angle": "rad",
    "brake_bias": "1",
    "maximum_pedal_force": "N",
    "hydraulic_ratio_front": "1",
    "hydraulic_ratio_rear": "1",
    "maximum_brake_pressure": "Pa",
    "caliper_area": "m^2",
    "pad_mu_front": "1",
    "pad_mu_rear": "1",
    "brake_effective_radius_front": "m",
    "brake_effective_radius_rear": "m",
}

_CapabilityValue = TypeVar("_CapabilityValue")

#: Wheel positions in the order FL, FR, RL, RR.
_WHEEL_ORDER = ("fl", "fr", "rl", "rr")


def compile_planar_7dof_vehicle(
    assembly: Model,
    resolutions: Mapping[str, ResolutionResult],
) -> PlanarVehicleParameters:
    """Compile one assembly for the planar 7-DOF formulation.

    Consume the assembly parameters that the planar model needs.
    Resolve each requested capability through the capability resolutions.
    """

    values, _ = required_parameters(assembly, PLANAR_7DOF_VEHICLE_PARAMETER_UNITS)
    require_positive(
        assembly,
        values,
        "total_mass",
        "driver_mass",
        "wheelbase",
        "track_width",
        "yaw_radius_of_gyration",
        "wheel_inertia",
        "steer_ratio",
        "steering_maximum_angle",
        "steering_maximum_rate",
        "rack_travel_per_revolution",
        "steer_arm_length",
        "maximum_pedal_force",
        "hydraulic_ratio_front",
        "hydraulic_ratio_rear",
        "maximum_brake_pressure",
        "caliper_area",
        "pad_mu_front",
        "pad_mu_rear",
        "brake_effective_radius_front",
        "brake_effective_radius_rear",
    )
    require_nonnegative(
        assembly,
        values,
        "mass_without_driver",
        "cg_height",
        "caster_angle",
        "kingpin_inclination_angle",
    )
    require_open_unit_interval(
        assembly,
        values,
        "front_mass_fraction",
    )
    require_signed_brake_bias(assembly, values)
    require_closed_unit_interval(
        assembly,
        values,
        "lateral_load_transfer_distribution",
    )
    if values["mass_without_driver"] + values["driver_mass"] > values["total_mass"] + 1.0e-9:
        raise ValueError(
            f"{assembly.path}: total_mass must cover the driver and the car"
        )
    drivetrain = _capability_value(
        assembly,
        resolutions,
        DRIVE_FORCE_LIMIT,
        DrivetrainParameters,
    )
    # The assembled runtime carries the nested motor and battery. The planar
    # model needs the smooth envelope and the storage curves, not the compact
    # legacy drive-force contract.
    nested = getattr(drivetrain, "powertrain", None)
    if nested is None:
        raise ValueError(
            f"{assembly.path}: the drivetrain must be an assembled powertrain"
        )
    motor = nested.power_plant
    storage = nested.energy_storage
    if not isinstance(motor, MotorEnvelopeParameters):
        raise ValueError(
            f"{assembly.path}: the planar model needs a "
            f"{TORQUE_ENVELOPE_SUBTYPE!r} power plant, received "
            f"{type(motor).__name__}"
        )
    return PlanarVehicleParameters(
        total_mass=values["total_mass"],
        mass_without_driver=values["mass_without_driver"],
        driver_mass=values["driver_mass"],
        wheelbase=values["wheelbase"],
        track_width=values["track_width"],
        cg_height=values["cg_height"],
        front_mass_fraction=values["front_mass_fraction"],
        lateral_load_transfer_distribution=values[
            "lateral_load_transfer_distribution"
        ],
        yaw_radius_of_gyration=values["yaw_radius_of_gyration"],
        wheel_inertia=values["wheel_inertia"],
        steer_ratio=values["steer_ratio"],
        steering_maximum_angle=values["steering_maximum_angle"],
        steering_maximum_rate=values["steering_maximum_rate"],
        caster_angle=values["caster_angle"],
        kingpin_inclination_angle=values["kingpin_inclination_angle"],
        rack_travel_per_revolution=values["rack_travel_per_revolution"],
        steer_arm_length=values["steer_arm_length"],
        ackermann=_read_ackermann_table(assembly),
        brake_bias=values["brake_bias"],
        maximum_pedal_force=values["maximum_pedal_force"],
        hydraulic_ratio_front=values["hydraulic_ratio_front"],
        hydraulic_ratio_rear=values["hydraulic_ratio_rear"],
        maximum_brake_pressure=values["maximum_brake_pressure"],
        caliper_area=values["caliper_area"],
        pad_mu_front=values["pad_mu_front"],
        pad_mu_rear=values["pad_mu_rear"],
        brake_effective_radius_front=values["brake_effective_radius_front"],
        brake_effective_radius_rear=values["brake_effective_radius_rear"],
        pedal_curve=_read_pedal_curve(assembly),
        tire=_capability_value(
            assembly,
            resolutions,
            STEADY_COMBINED_SLIP,
            UniTireSimpleParameters,
        ),
        aero=_capability_value(
            assembly,
            resolutions,
            AERO_LOADS,
            AeroParameters,
        ),
        drivetrain=drivetrain,
        controller=drivetrain.drivetrain.controller,
        motor=motor,
        storage=storage,
    )


def _capability_value(
    assembly: Model,
    resolutions: Mapping[str, ResolutionResult],
    capability: str,
    expected_type: type[_CapabilityValue],
) -> _CapabilityValue:
    resolution = resolutions.get(capability)
    if resolution is None:
        raise ValueError(
            f"{assembly.path}: the compiler did not resolve capability {capability!r}"
        )
    if not isinstance(resolution.value, expected_type):
        raise ValueError(
            f"{assembly.path}: capability {capability!r} produced "
            f"{type(resolution.value).__name__}, expected "
            f"{expected_type.__name__}"
        )
    return cast(_CapabilityValue, resolution.value)


def _table_path(assembly: Model, role: str) -> Path:
    """Return the path of one named data table of the assembly."""

    for reference in assembly.data:
        if reference.role == role:
            if reference.format != "csv":
                raise ValueError(
                    f"{assembly.path}: table {role!r} must use the csv format"
                )
            return reference.path
    raise ValueError(f"{assembly.path}: missing the {role!r} data table")


def _read_number_table(
    assembly: Model,
    role: str,
    columns: tuple[str, str],
) -> tuple[FloatArray, FloatArray]:
    """Read a two column numeric table from a named assembly data file."""

    path = _table_path(assembly, role)
    first: list[float] = []
    second: list[float] = []
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            if header is None or [item.strip() for item in header] != list(columns):
                raise ValueError(
                    f"{path}: columns must be {', '.join(columns)}"
                )
            for number, row in enumerate(reader, start=2):
                if not row or all(not cell.strip() for cell in row):
                    continue
                if len(row) != 2:
                    raise ValueError(f"{path}: row {number} must hold two values")
                try:
                    first.append(float(row[0]))
                    second.append(float(row[1]))
                except ValueError as error:
                    raise ValueError(
                        f"{path}: row {number} holds a non-numeric value"
                    ) from error
    except OSError as error:
        raise ValueError(f"cannot read the {role!r} table at {path}: {error}") from error
    return (
        np.asarray(first, dtype=np.float64),
        np.asarray(second, dtype=np.float64),
    )


def _read_ackermann_table(assembly: Model) -> AckermannTable:
    """Read the Ackermann table of one assembly."""

    inner, outer = _read_number_table(
        assembly, "ackermann", ("inner_angle_rad", "outer_offset_rad")
    )
    return AckermannTable(inner_angle=inner, outer_offset=outer)


def _read_pedal_curve(assembly: Model) -> PedalRatioCurve:
    """Read the pedal ratio curve of one assembly."""

    force, ratio = _read_number_table(
        assembly, "pedal-ratio", ("pedal_force_n", "pedal_ratio")
    )
    constant = float(ratio[0]) if ratio.size else 1.0
    if ratio.size and float(np.ptp(ratio)) == 0.0:
        # A constant curve needs no table.
        return PedalRatioCurve(
            pedal_force=np.asarray([], dtype=np.float64),
            pedal_ratio=np.asarray([], dtype=np.float64),
            constant_ratio=constant,
        )
    return PedalRatioCurve(pedal_force=force, pedal_ratio=ratio, constant_ratio=constant)


def wheel_positions(vehicle: PlanarVehicleParameters) -> FloatArray:
    """Return the wheel position of each wheel in body axes.

    Return a ``(4, 2)`` array of ``(x, y)`` positions in the order FL, FR, RL,
    RR. ``x`` points forward and ``y`` points right, so the front axle sits at
    a positive ``x`` and the right wheels sit at a positive ``y``.
    """

    front = vehicle.front_axle_to_cg
    rear = -vehicle.rear_axle_to_cg
    half_track = vehicle.half_track
    return np.asarray(
        [
            (front, -half_track),
            (front, half_track),
            (rear, -half_track),
            (rear, half_track),
        ],
        dtype=np.float64,
    )


def aerodynamic_forces(
    vehicle: PlanarVehicleParameters,
    speed: float,
) -> tuple[float, float]:
    """Return the downforce and the drag at one speed."""

    return fixed_aero.aerodynamic_forces(vehicle.aero, speed)


def evaluate_normal_loads(
    vehicle: PlanarVehicleParameters,
    speed: float,
    longitudinal_acceleration: float,
    lateral_acceleration: float,
) -> FloatArray:
    """Return the quasi-static vertical load of each tire.

    The lateral acceleration uses the SAE vehicle frame, so a positive value is
    a right turn and moves load to the left wheels. Unlike the point-mass
    model, the sign is significant even though the axle total is not.

    An inside wheel lifts at zero load instead of taking a negative load; the
    roll moment it can no longer react moves to the other axle. A negative
    load is returned only for a two-wheel (rollover) state.

    Return the loads in the order FL, FR, RL, RR.
    """

    downforce, _ = aerodynamic_forces(vehicle, speed)
    return quasi_static.normal_loads(
        total_mass=vehicle.total_mass,
        wheelbase=vehicle.wheelbase,
        track_width=vehicle.track_width,
        cg_height=vehicle.cg_height,
        front_mass_fraction=vehicle.front_mass_fraction,
        lateral_transfer_distribution=vehicle.lateral_load_transfer_distribution,
        downforce=downforce,
        aero_balance=vehicle.aero.aero_balance,
        longitudinal_acceleration=longitudinal_acceleration,
        lateral_acceleration=lateral_acceleration,
        wheel_lift=True,
    )


@dataclass(frozen=True, slots=True)
class PlanarState:
    """Store one planar vehicle state.

    The state holds seven mechanical degrees of freedom plus the electrical
    state of charge. Position and heading are pose coordinates, and they do not
    add a mechanical degree of freedom.
    """

    v_x: float
    v_y: float
    yaw_rate: float
    wheel_speeds: FloatArray
    state_of_charge: float = 1.0


@dataclass(frozen=True, slots=True)
class PlanarControls:
    """Store one planar vehicle control.

    ``throttle`` and ``brake`` are fractions from zero to one.
    ``steering`` is the road-wheel angle in radians.
    """

    throttle: float
    brake: float
    steering: float


def body_lateral_acceleration(
    lateral_velocity_rate: float,
    yaw_rate: float,
    v_x: float,
) -> float:
    """Return the body lateral acceleration of one state.

    The lateral acceleration is ``v_y_dot + r * v_x``.
    Use it for the quasi-static load transfer.
    In a steady turn the term ``v_y_dot`` is zero.
    """

    return lateral_velocity_rate + yaw_rate * v_x


def wheel_steering_angles(
    steering: float | FloatArray,
    wheel_count: int = 4,
    steering_axle: int = 2,
) -> FloatArray:
    """Return one steering angle for each wheel.

    A scalar applies to the steering axle and leaves the other wheels straight.
    A full array is validated and returned unchanged. Use the full form for an
    Ackermann pair, where the two front wheels differ.
    """

    values = np.asarray(steering, dtype=np.float64)
    if values.ndim == 0:
        if not np.isfinite(float(values)):
            raise ValueError("steering must be finite")
        angles = np.zeros(wheel_count, dtype=np.float64)
        angles[:steering_axle] = float(values)
        return angles
    if values.shape != (wheel_count,):
        raise ValueError(
            f"steering must be a scalar or hold {wheel_count} wheel angles"
        )
    if not np.all(np.isfinite(values)):
        raise ValueError("steering must be finite")
    return values


def contact_velocities(
    state: PlanarState,
    positions: FloatArray,
    steering: float | FloatArray,
) -> tuple[FloatArray, FloatArray]:
    """Return the contact velocity of each wheel in its own tire axes.

    Return the longitudinal and the lateral velocity.
    The first ``steering_axle`` wheels steer by ``steering``.
    The contact velocity of a rigid body at position ``(x, y)`` in body axes is
    ``(v_x - r*y, v_y + r*x)``.

    A positive steering angle turns the wheel to the right, which follows the
    SAE frame. Express each body velocity in the rotated wheel axes.
    """

    body_longitudinal = state.v_x - state.yaw_rate * positions[:, 1]
    body_lateral = state.v_y + state.yaw_rate * positions[:, 0]
    angle = wheel_steering_angles(steering, positions.shape[0])
    cosine = np.cos(angle)
    sine = np.sin(angle)
    longitudinal = body_longitudinal * cosine + body_lateral * sine
    lateral = -body_longitudinal * sine + body_lateral * cosine
    return longitudinal, lateral


def slip_quantities(
    wheel_speeds: FloatArray,
    effective_radii: FloatArray,
    longitudinal_velocity: FloatArray,
    lateral_velocity: FloatArray,
    speed_floor: float,
) -> tuple[FloatArray, FloatArray]:
    """Return the slip ratio and the slip angle of each wheel.

    The slip ratio is ``(Re*w - v_long) / max(|v_long|, speed_floor)``.
    A driving wheel therefore has a positive slip ratio and a braking wheel has
    a negative one.

    The slip angle is ``-atan2(v_lat, max(|v_long|, speed_floor))``.
    A positive angle steers the wheel to the right, which is the sense that the
    tire model documents. A right-steered wheel therefore has a positive slip
    angle and gives a positive lateral force.

    ``speed_floor`` regularizes the low-speed singularity. It is a numerical
    device, not a physical quantity.
    """

    if speed_floor <= 0.0:
        raise ValueError("the speed floor must be positive")
    denominator = np.maximum(np.abs(longitudinal_velocity), speed_floor)
    slip_ratio = (
        effective_radii * wheel_speeds - longitudinal_velocity
    ) / denominator
    slip_angle = -np.arctan2(lateral_velocity, denominator)
    return slip_ratio, slip_angle


def rotate_tire_forces_to_body(
    longitudinal_force: FloatArray,
    lateral_force: FloatArray,
    steering: float | FloatArray,
) -> tuple[FloatArray, FloatArray]:
    """Rotate each tire force from its tire axes into the body axes.

    The moment about the vertical axis does not change under this rotation.
    """

    angle = wheel_steering_angles(steering, longitudinal_force.shape[0])
    cosine = np.cos(angle)
    sine = np.sin(angle)
    body_longitudinal = longitudinal_force * cosine - lateral_force * sine
    body_lateral = longitudinal_force * sine + lateral_force * cosine
    return body_longitudinal, body_lateral


@dataclass(frozen=True, slots=True)
class PlanarEvaluation:
    """Store one planar vehicle evaluation.

    Every four-element array uses the wheel order FL, FR, RL, RR.
    The tire forces and moments use the tire axes.
    The body forces use the vehicle axes.

    The three accelerations and the four wheel accelerations are the state
    derivative. Position and heading are not part of this evaluation.
    """

    normal_loads: FloatArray
    longitudinal_forces: FloatArray
    lateral_forces: FloatArray
    aligning_moments: FloatArray
    body_longitudinal_forces: FloatArray
    body_lateral_forces: FloatArray
    slip_ratios: FloatArray
    slip_angles: FloatArray
    effective_radii: FloatArray
    drag_force: float
    yaw_moment: float
    longitudinal_acceleration: float
    lateral_acceleration: float
    yaw_acceleration: float
    wheel_accelerations: FloatArray
    load_iterations: int
    load_residual: float
    load_converged: bool
    #: How the load closure ended: "newton", "fixed-point" or "unconverged".
    load_method: str = "fixed-point"
    #: ``|det(I - J)|`` of the load map's finite-difference Jacobian at the
    #: returned acceleration (``nan`` when no Jacobian was formed). A value away
    #: from zero means the closure is locally a smooth, unique solve.
    load_jacobian_determinant: float = float("nan")

    @property
    def is_feasible(self) -> bool:
        """Return true when no tire load is negative."""

        return bool(np.all(self.normal_loads >= 0.0))

    @property
    def contact_feasible(self) -> bool:
        """Return true when the load solve converged and no tire lifted."""

        return self.load_converged and self.is_feasible


def parse_wheel_torques(wheel_torques: FloatArray) -> FloatArray:
    """Validate and copy one four-wheel torque vector."""

    torques = np.asarray(wheel_torques, dtype=np.float64)
    if torques.shape != (4,):
        raise ValueError("wheel_torques must hold four values in FL, FR, RL, RR order")
    if not np.all(np.isfinite(torques)):
        raise ValueError("wheel_torques must be finite")
    return torques


def yaw_moment_of(
    positions: FloatArray,
    body_longitudinal_forces: FloatArray,
    body_lateral_forces: FloatArray,
    aligning_moments: FloatArray,
) -> float:
    """Return the total yaw moment about the vertical axis.

    The moment of one wheel is ``x * Fy - y * Fx + Mz``.
    The right wheels sit at a positive ``y``, so a forward force on the right
    wheels gives a negative moment, which turns the vehicle to the left.
    """

    return float(
        np.sum(
            positions[:, 0] * body_lateral_forces
            - positions[:, 1] * body_longitudinal_forces
            + aligning_moments
        )
    )


@dataclass(frozen=True, slots=True)
class _LoadStep:
    """Hold one evaluation of the coupled load and force balance."""

    loads: FloatArray
    radii: FloatArray
    slip_ratios: FloatArray
    slip_angles: FloatArray
    longitudinal_forces: FloatArray
    lateral_forces: FloatArray
    aligning_moments: FloatArray
    rolling_moments: FloatArray
    body_longitudinal: FloatArray
    body_lateral: FloatArray
    drag: float
    implied_longitudinal_acceleration: float
    implied_lateral_acceleration: float


def _load_step(
    vehicle: PlanarVehicleParameters,
    state: PlanarState,
    steering: float,
    speed: float,
    rolling_speed: FloatArray,
    longitudinal_velocity: FloatArray,
    lateral_velocity: FloatArray,
    longitudinal_acceleration: float,
    lateral_acceleration: float,
    speed_floor: float,
) -> _LoadStep:
    """Evaluate the loads and the tire forces at one acceleration guess.

    The vertical load follows from the acceleration guess.
    The tire force follows from the load and the slip.
    The implied acceleration follows from the tire force.
    The caller compares the two accelerations to judge convergence.
    """

    loads = evaluate_normal_loads(
        vehicle,
        speed,
        longitudinal_acceleration,
        lateral_acceleration,
    )
    radii = loaded_radius(vehicle.tire, loads)
    slip_ratio, slip_angle = slip_quantities(
        state.wheel_speeds,
        radii,
        longitudinal_velocity,
        lateral_velocity,
        speed_floor,
    )
    tire = evaluate_unitire(
        vehicle.tire,
        slip_angle,
        slip_ratio,
        loads,
        rolling_speed,
    )
    body_longitudinal, body_lateral = rotate_tire_forces_to_body(
        tire.longitudinal_force,
        tire.lateral_force,
        steering,
    )
    _, drag = aerodynamic_forces(vehicle, speed)
    mass = vehicle.total_mass
    # Load transfer responds to inertial acceleration, not the derivative of
    # body-frame velocity. In a steady turn dv_y/dt can vanish while a_y=r*v_x.
    implied_longitudinal = (float(np.sum(body_longitudinal)) - drag) / mass
    implied_lateral = float(np.sum(body_lateral)) / mass
    return _LoadStep(
        loads=loads,
        radii=radii,
        slip_ratios=slip_ratio,
        slip_angles=slip_angle,
        longitudinal_forces=tire.longitudinal_force,
        lateral_forces=tire.lateral_force,
        aligning_moments=tire.aligning_moment,
        rolling_moments=tire.rolling_resistance_moment,
        body_longitudinal=body_longitudinal,
        body_lateral=body_lateral,
        drag=drag,
        implied_longitudinal_acceleration=implied_longitudinal,
        implied_lateral_acceleration=implied_lateral,
    )


#: Damping applied when the load fixed point stops contracting.
_LOAD_DAMPING = 0.5
#: Forward-difference increment for the load map's Jacobian (m/s^2).
_LOAD_JACOBIAN_STEP = 1.0e-6


def evaluate_planar_dynamics(
    vehicle: PlanarVehicleParameters,
    state: PlanarState,
    wheel_torques: FloatArray,
    steering: float,
    speed_floor: float = 1.0,
    load_tolerance: float = 1.0e-9,
    load_maximum_iterations: int = 60,
) -> PlanarEvaluation:
    """Evaluate the planar vehicle dynamics at one state.

    ``wheel_torques`` holds the **net** torque at each wheel in the order FL,
    FR, RL, RR. A positive torque drives the wheel forward; a braking torque is
    negative. The caller owns the split between drive and brake. The wheel
    torque does not change the tire force at the current state; it changes the
    wheel acceleration, and therefore the next state.

    ``steering`` is the road-wheel angle in radians and turns the front wheels.

    The vertical load depends on the body acceleration, and the tire force
    depends on the vertical load. Solve that two-unknown loop with a damped
    Newton iteration (finite-difference Jacobian, backtracking line search).
    If Newton stalls at a kink of the map, continue with the bounded fixed
    point that applies the undamped update while the residual contracts and
    halves the step while it does not. Then evaluate once more at the
    converged acceleration, so every returned quantity belongs to the same
    state.

    A large slip can make the loop non-contractive. The guard keeps the loop
    bounded, and the returned ``load_converged``, ``load_residual`` and
    ``load_method`` report whether and how it converged. Never treat an
    unconverged answer as a converged one.
    """

    torques = parse_wheel_torques(wheel_torques)
    wheel_angles = wheel_steering_angles(steering)
    if not np.isfinite(speed_floor) or speed_floor <= 0.0:
        raise ValueError("speed_floor must be finite and positive")
    if not np.isfinite(load_tolerance) or load_tolerance <= 0.0:
        raise ValueError("load_tolerance must be finite and positive")
    if load_maximum_iterations < 1:
        raise ValueError("load_maximum_iterations must be positive")
    if vehicle.wheel_inertia <= 0.0:
        raise ValueError("the wheel inertia must be positive")
    if vehicle.yaw_inertia <= 0.0:
        raise ValueError("the yaw inertia must be positive")

    positions = wheel_positions(vehicle)
    speed = float(np.hypot(state.v_x, state.v_y))
    longitudinal_velocity, lateral_velocity = contact_velocities(
        state, positions, wheel_angles
    )
    # The tire model recovers the rolling speed from the hub speed magnitude.
    rolling_speed = np.abs(state.wheel_speeds)

    def step(longitudinal: float, lateral: float) -> _LoadStep:
        return _load_step(
            vehicle,
            state,
            wheel_angles,
            speed,
            rolling_speed,
            longitudinal_velocity,
            lateral_velocity,
            longitudinal,
            lateral,
            speed_floor,
        )

    def residual_of(evaluation: _LoadStep, guess: FloatArray) -> FloatArray:
        return np.array([
            evaluation.implied_longitudinal_acceleration - guess[0],
            evaluation.implied_lateral_acceleration - guess[1],
        ])

    guess = np.zeros(2)
    evaluation = step(0.0, 0.0)
    residual_vector = residual_of(evaluation, guess)
    residual = float(np.max(np.abs(residual_vector)))
    iteration = 0
    load_converged = residual <= load_tolerance
    method = "newton" if load_converged else "unconverged"
    determinant = float("nan")

    # Newton on F(a) = g(a) - a with a forward-difference Jacobian and a
    # backtracking line search on the residual norm. The load map has kinks (a
    # lifted wheel, the tire's small-slip branch), so the line search keeps the
    # iteration monotone rather than trusting a full step.
    newton_budget = max(1, load_maximum_iterations // 2)
    # One finite-difference Jacobian per solve; afterwards a Broyden rank-one
    # update from each accepted step (no extra load evaluations). If a stale
    # Broyden matrix cannot produce a descent step, rebuild it once and retry.
    jacobian = None
    rebuilt = False
    while not load_converged and iteration < newton_budget:
        iteration += 1
        if not np.all(np.isfinite(residual_vector)):
            break
        if jacobian is None:
            jacobian = np.empty((2, 2))
            for column in range(2):
                increment = _LOAD_JACOBIAN_STEP * max(1.0, abs(guess[column]))
                probe = guess.copy()
                probe[column] += increment
                probed = step(probe[0], probe[1])
                jacobian[:, column] = (residual_of(probed, probe) - residual_vector) / increment
            determinant = abs(float(np.linalg.det(jacobian)))
            fresh = True
        else:
            fresh = False
        if not np.isfinite(determinant) or determinant < 1.0e-12:
            break
        direction = -np.linalg.solve(jacobian, residual_vector)
        factor = 1.0
        accepted = False
        for _ in range(8):
            trial = guess + factor * direction
            trial_evaluation = step(trial[0], trial[1])
            trial_vector = residual_of(trial_evaluation, trial)
            trial_residual = float(np.max(np.abs(trial_vector)))
            if np.isfinite(trial_residual) and trial_residual < residual:
                accepted = True
                break
            factor *= 0.5
        if not accepted:
            if fresh or rebuilt:
                break
            jacobian, rebuilt = None, True
            continue
        move = trial - guess
        change = trial_vector - residual_vector
        jacobian = jacobian + np.outer(change - jacobian @ move, move) / float(move @ move)
        determinant = abs(float(np.linalg.det(jacobian)))
        guess, evaluation = trial, trial_evaluation
        residual_vector, residual = trial_vector, trial_residual
        if residual <= load_tolerance:
            load_converged = True
            method = "newton"

    if not load_converged:
        # Fallback: the original bounded fixed point, continued from the best
        # Newton iterate. Apply the undamped update while the residual
        # contracts, and halve the step while it does not.
        previous_residual = float("inf")
        remaining = max(1, load_maximum_iterations - iteration)
        for local in range(1, remaining + 1):
            iteration += 1
            residual_vector = residual_of(evaluation, guess)
            residual = float(np.max(np.abs(residual_vector)))
            if residual <= load_tolerance:
                load_converged = True
                method = "fixed-point"
                break
            if local > 1 and residual >= previous_residual:
                guess = guess + _LOAD_DAMPING * residual_vector
            else:
                guess = np.array([evaluation.implied_longitudinal_acceleration,
                                  evaluation.implied_lateral_acceleration])
            previous_residual = residual
            evaluation = step(guess[0], guess[1])

    longitudinal_acceleration, lateral_acceleration = float(guess[0]), float(guess[1])
    # Evaluate once more at the returned acceleration, so the loads, the slip,
    # and the forces all describe the same state.
    evaluation = step(longitudinal_acceleration, lateral_acceleration)
    final_residual = max(
        abs(
            evaluation.implied_longitudinal_acceleration
            - longitudinal_acceleration
        ),
        abs(evaluation.implied_lateral_acceleration - lateral_acceleration),
    )
    load_residual = max(residual, final_residual)
    load_converged = load_converged and final_residual <= load_tolerance
    if not load_converged:
        method = "unconverged"

    moment = yaw_moment_of(
        positions,
        evaluation.body_longitudinal,
        evaluation.body_lateral,
        evaluation.aligning_moments,
    )
    yaw_acceleration = moment / vehicle.yaw_inertia
    # A positive wheel speed rolls the wheel forward. The tire force resists the
    # drive torque, and the rolling resistance moment retards a forward roll.
    wheel_accelerations = (
        torques
        - evaluation.radii * evaluation.longitudinal_forces
        + evaluation.rolling_moments
    ) / vehicle.wheel_inertia

    return PlanarEvaluation(
        normal_loads=evaluation.loads,
        longitudinal_forces=evaluation.longitudinal_forces,
        lateral_forces=evaluation.lateral_forces,
        aligning_moments=evaluation.aligning_moments,
        body_longitudinal_forces=evaluation.body_longitudinal,
        body_lateral_forces=evaluation.body_lateral,
        slip_ratios=evaluation.slip_ratios,
        slip_angles=evaluation.slip_angles,
        effective_radii=evaluation.radii,
        drag_force=evaluation.drag,
        yaw_moment=moment,
        longitudinal_acceleration=(
            longitudinal_acceleration + state.yaw_rate * state.v_y
        ),
        lateral_acceleration=(
            lateral_acceleration - state.yaw_rate * state.v_x
        ),
        yaw_acceleration=yaw_acceleration,
        wheel_accelerations=wheel_accelerations,
        load_iterations=iteration,
        load_residual=load_residual,
        load_converged=load_converged,
        load_method=method,
        load_jacobian_determinant=determinant,
    )
