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

from collections.abc import Mapping
from dataclasses import dataclass
from typing import TypeVar, cast

import numpy as np

from simplified_vehicle_models.aero import fixed_aero
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
from simplified_vehicle_models.suspension import quasi_static
from simplified_vehicle_models.xml.model import Model
from simplified_vehicle_models.xml.schema import (
    require_closed_unit_interval,
    require_nonnegative,
    require_open_unit_interval,
    require_positive,
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
    "caster_angle": "rad",
    "kingpin_inclination_angle": "rad",
    "brake_bias": "1",
    "maximum_brake_torque_front": "N*m",
    "maximum_brake_torque_rear": "N*m",
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
        "maximum_brake_torque_front",
        "maximum_brake_torque_rear",
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
        "brake_bias",
    )
    require_closed_unit_interval(
        assembly,
        values,
        "lateral_load_transfer_distribution",
    )
    if values["mass_without_driver"] + values["driver_mass"] > values["total_mass"] + 1.0e-9:
        raise ValueError(
            f"{assembly.path}: total_mass must cover the driver and the car"
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
        brake_bias=values["brake_bias"],
        maximum_brake_torque_front=values["maximum_brake_torque_front"],
        maximum_brake_torque_rear=values["maximum_brake_torque_rear"],
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
        drivetrain=_capability_value(
            assembly,
            resolutions,
            DRIVE_FORCE_LIMIT,
            DrivetrainParameters,
        ),
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
