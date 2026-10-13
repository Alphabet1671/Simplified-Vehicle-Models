"""Construct a low-speed, locally balanced starting trajectory for planar IPOPT.

At each path node solve the vertical loads, steady wheel rotation, lateral
sideslip, steering and small positive drive needed for a constant-speed turn.
This is only a *seed*: curvature and sideslip still change between nodes, so
spatial dynamics must be satisfied by the NLP before a lap can be trusted.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from scipy.optimize import least_squares

from simplified_vehicle_models.formulations.planar_7dof import evaluate_normal_loads
from simplified_vehicle_models.parameters.vehicle import PlanarVehicleParameters
from simplified_vehicle_models.powertrain.energy_storage import StorageState, solve_current
from simplified_vehicle_models.powertrain.motor_envelope import total_bus_power
from simplified_vehicle_models.tires.unitire_simple import loaded_radius


def equilibrium_start(
    vehicle: PlanarVehicleParameters,
    path: dict[str, np.ndarray],
    node_functions: Any,
    cruise_speed: float = 8.0,
) -> dict[str, np.ndarray]:
    """Return a full-length quasi-steady seed with exact local tire balances.

    The 11 local unknowns are body lateral velocity, steering, throttle, four
    normal loads and four angular speeds. Eleven equations enforce load and
    wheel consistency plus zero local body-x, body-y and yaw accelerations.
    At each node the heading cancels sideslip and the yaw rate follows the path,
    so lateral progress and heading-error derivatives are zero. The clock and
    state of charge are integrated along the path after the mechanical solve.
    """

    import casadi

    if not np.isfinite(cruise_speed) or cruise_speed <= 2.0:
        raise ValueError("equilibrium cruise speed must exceed 2 m/s")
    motor = vehicle.motor
    storage = vehicle.storage
    omega_limit = motor.maximum_motor_speed / motor.gear_ratio
    if cruise_speed >= 0.85 * omega_limit * min(
        loaded_radius(vehicle.tire, evaluate_normal_loads(vehicle, cruise_speed, 0.0, 0.0))
    ):
        raise ValueError("equilibrium cruise speed leaves no motor slip reserve")
    speed = float(cruise_speed)
    distance = np.asarray(path["distance"], dtype=float)
    curvature = np.asarray(path["curvature"], dtype=float)
    count = distance.size
    if count < 2 or np.any(np.diff(distance) <= 0.0):
        raise ValueError("the equilibrium path must have at least two increasing nodes")

    unknown = casadi.SX.sym("equilibrium_unknown", 11)
    speed_symbol = casadi.SX.sym("equilibrium_speed")
    curvature_symbol = casadi.SX.sym("equilibrium_curvature")
    soc_symbol = casadi.SX.sym("equilibrium_soc")
    side = unknown[0]
    progress = casadi.sqrt(speed_symbol * speed_symbol + side * side)
    state_expr = casadi.vertcat(
        0.0, -casadi.atan2(side, speed_symbol),
        speed_symbol, side, curvature_symbol * progress, soc_symbol, 0.0,
    )
    control_expr = casadi.vertcat(unknown[2], 0.0, unknown[1])
    algebraic_expr = casadi.vertcat(unknown[3:11], 0.0)
    balance = casadi.vertcat(
        node_functions.algebraic(
            state_expr, control_expr, algebraic_expr, curvature_symbol
        )[:8],
        node_functions.dynamics(
            state_expr, control_expr, algebraic_expr, curvature_symbol
        )[2:5] * progress,
    )
    arguments = [unknown, speed_symbol, curvature_symbol, soc_symbol]
    residual = casadi.Function("equilibrium_residual", arguments, [balance])
    jacobian = casadi.Function(
        "equilibrium_jacobian", arguments, [casadi.jacobian(balance, unknown)]
    )
    scales = np.asarray([1000.0] * 4 + [100.0] * 4 + [4.0, 10.0, 5.0])
    lower = np.asarray([-5.0, -vehicle.steering_maximum_angle, 0.0]
                       + [1.0] * 4 + [0.0] * 4)
    upper = np.asarray([5.0, vehicle.steering_maximum_angle, 1.0]
                       + [20000.0] * 4 + [omega_limit] * 4)
    state = np.zeros((7, count))
    control = np.zeros((3, count))
    algebraic = np.zeros((9, count))
    static_loads = evaluate_normal_loads(vehicle, speed, 0.0, 0.0)
    previous = np.r_[0.0, 0.0, 0.02, static_loads,
                     speed / loaded_radius(vehicle.tire, static_loads)]
    for index, k in enumerate(curvature):
        lateral_loads = evaluate_normal_loads(vehicle, speed, 0.0, speed**2 * k)
        # The previous node balances the same branch when the mesh is fine.
        # A coarse mesh can change curvature sharply, so fall back to a guess
        # built from the current curvature's lateral load transfer.
        candidates = [previous]
        candidates.append(np.r_[
            0.0, np.arctan(vehicle.wheelbase * k), 0.03,
            lateral_loads, speed / loaded_radius(vehicle.tire, lateral_loads),
        ])
        fitted = None
        for candidate in candidates:
            at = np.clip(candidate, lower + 1.0e-9, upper - 1.0e-9)

            def evaluate(z: np.ndarray) -> np.ndarray:
                return np.asarray(residual(z, speed, float(k),
                                           storage.initial_state_of_charge)).ravel() / scales

            def gradient(z: np.ndarray) -> np.ndarray:
                return np.asarray(jacobian(z, speed, float(k),
                                           storage.initial_state_of_charge)) / scales[:, None]

            attempt = least_squares(
                evaluate, at, jac=gradient, bounds=(lower, upper),
                max_nfev=80, ftol=1.0e-10, xtol=1.0e-10, gtol=1.0e-10,
            )
            if np.max(np.abs(attempt.fun)) <= 1.0e-4:
                fitted = attempt
                break
        if fitted is None:
            raise RuntimeError(
                f"cannot find a balanced equilibrium at node {index} "
                f"(s={distance[index]:.2f} m, kappa={k:.4f} 1/m): "
                f"scaled residual={np.max(np.abs(attempt.fun)):.3g}"
            )
        previous = fitted.x
        side, steer, throttle = previous[:3]
        state[:5, index] = (
            0.0, -np.arctan2(side, speed), speed,
            side, k * np.hypot(speed, side),
        )
        control[:, index] = (throttle, 0.0, steer)
        algebraic[:8, index] = previous[3:11]

    peak = motor.peak_motor_torque * motor.gear_ratio * motor.motor_efficiency
    shares = np.asarray([0.1, 0.1, 0.4, 0.4])
    capacity = 3600.0 * storage.pack_capacity_ah
    state[5, 0] = storage.initial_state_of_charge
    for index in range(count):
        drive = shares * 4.0 * peak * control[0, index]
        power = total_bus_power(motor, algebraic[4:8, index], drive)
        current, feasible = solve_current(
            storage, StorageState(float(state[5, index])), power
        )
        if not feasible:
            raise RuntimeError(f"battery cannot supply seed at node {index}")
        algebraic[8, index] = current
        if index == count - 1:
            break
        step = distance[index + 1] - distance[index]
        here_progress = np.hypot(state[2, index], state[3, index])
        next_progress = np.hypot(state[2, index + 1], state[3, index + 1])
        next_drive = shares * 4.0 * peak * control[0, index + 1]
        next_power = total_bus_power(motor, algebraic[4:8, index + 1], next_drive)
        predicted_soc = state[5, index] - step * current / (capacity * here_progress)
        if predicted_soc < 0.02:
            raise RuntimeError("equilibrium seed would exhaust the battery")
        next_current, feasible = solve_current(
            storage, StorageState(float(predicted_soc)), next_power
        )
        if not feasible:
            raise RuntimeError(f"battery cannot supply seed at node {index + 1}")
        state[5, index + 1] = state[5, index] - step * 0.5 * (
            current / here_progress + next_current / next_progress
        ) / capacity
        state[6, index + 1] = state[6, index] + step * 0.5 * (
            1.0 / here_progress + 1.0 / next_progress
        )
    return {"state": state, "control": control, "algebraic": algebraic}
