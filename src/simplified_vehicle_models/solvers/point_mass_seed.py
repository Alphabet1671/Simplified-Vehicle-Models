"""Map a point-mass flying-lap profile to a planar OCP primal seed."""

from __future__ import annotations

import numpy as np

from simplified_vehicle_models.core.types import FloatArray
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.results.traces import LapResult


def point_mass_planar_primal_seed(
    lap: LapResult,
    source_distance: FloatArray,
    target_path: dict[str, FloatArray],
    vehicle: PlanarVehicleParameters,
) -> dict[str, np.ndarray]:
    """Map point-mass speed, acceleration, and pedal profiles to OCP states.

    The point-mass profile has no yaw, steering, path errors, or wheel-level
    quantities. Those are initialized kinematically; callers should pass the
    returned arrays through the planar OCP's local algebraic projection before
    IPOPT. The mapping intentionally does not copy point-mass wheel loads or
    forces into the planar algebraics.
    """

    source = np.asarray(source_distance, dtype=np.float64)
    distance = np.asarray(target_path["distance"], dtype=np.float64)
    curvature = np.asarray(target_path["curvature"], dtype=np.float64)
    source_speed = np.asarray(lap.speed, dtype=np.float64)
    source_acceleration = np.asarray(
        lap.longitudinal_acceleration, dtype=np.float64
    )
    if source.ndim != 1 or source.shape != source_speed.shape:
        raise ValueError("source_distance must match the point-mass speed profile")
    if source_acceleration.shape != (source.size - 1,):
        raise ValueError("point-mass acceleration must have one value per segment")
    if distance.ndim != 1 or curvature.shape != distance.shape or distance.size < 2:
        raise ValueError("target path distance and curvature must be matching vectors")
    if (
        not np.all(np.isfinite(source))
        or not np.all(np.isfinite(source_speed))
        or not np.all(np.isfinite(source_acceleration))
        or not np.all(np.isfinite(distance))
        or not np.all(np.isfinite(curvature))
        or np.any(np.diff(source) <= 0.0)
        or np.any(np.diff(distance) <= 0.0)
    ):
        raise ValueError("source and target distances must be finite and increasing")
    if distance[0] < source[0] - 1.0e-9 or distance[-1] > source[-1] + 1.0e-9:
        raise ValueError("target path must lie within source distance coverage")

    speed = np.interp(distance, source, source_speed)
    speed = np.maximum(speed, 0.25)
    state = np.zeros((7, distance.size), dtype=np.float64)
    state[2] = speed
    state[4] = speed * curvature
    state[5] = vehicle.storage.initial_state_of_charge
    state[6, 1:] = np.cumsum(
        0.5 * np.diff(distance) * (1.0 / speed[:-1] + 1.0 / speed[1:])
    )

    control = np.zeros((3, distance.size), dtype=np.float64)
    control[2] = np.arctan(vehicle.wheelbase * curvature)
    source_midpoint = 0.5 * (source[:-1] + source[1:])
    node_acceleration = np.interp(
        distance,
        source_midpoint,
        source_acceleration,
        left=float(source_acceleration[0]),
        right=float(source_acceleration[-1]),
    )
    control[0] = np.clip(node_acceleration / 6.0, 0.0, 1.0)
    control[1] = np.clip(-node_acceleration / 12.0, 0.0, 1.0)
    return {"state": state, "control": control}
