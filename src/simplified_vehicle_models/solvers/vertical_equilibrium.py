"""Static and steady-corner vertical equilibrium for the 14-DoF vehicle model."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from simplified_vehicle_models.aero.fixed_aero import aerodynamic_forces
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState, contact_velocities, slip_quantities, wheel_positions,
    wheel_steering_angles,
)
from simplified_vehicle_models.formulations.vertical_14dof import (
    VerticalEvaluation, VerticalVehicleParameters, VerticalState,
    evaluate_vertical_dynamics, sprung_wheel_positions,
)
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.tires.unitire_transient import steady_rolling_state
from simplified_vehicle_models.tires.vertical import tire_deflection_load


def solve_static_equilibrium(
    vehicle: VerticalVehicleParameters,
    speed: float = 0.0,
) -> VerticalState:
    """Solve static ride equilibrium in heave, roll, pitch, and four jounces.

    Spring preload is defined at zero jounce; consequently the equilibrium
    coordinates are solved while each spring force remains referenced to that
    design position. Tire forces and aero load are included in the force/moment
    balance. This solve deliberately uses only NumPy.
    """
    speed = float(speed)
    if not np.isfinite(speed) or speed < 0.0:
        raise ValueError("speed must be finite and nonnegative")

    planar = vehicle.planar
    downforce, _ = aerodynamic_forces(planar.aero, speed)
    # Heave/pitch are about the sprung CG, so body-point arms are sprung-CG arms.
    positions = sprung_wheel_positions(vehicle)
    gravity = 9.80665

    def residual(q):
        heave, roll, pitch, *jounce = q
        z = np.asarray(jounce)
        # Positive-down heave; positive roll lowers the right side (y points
        # right); positive pitch is nose up. Jounce is positive in compression
        # (hub moves up relative to the body), so the hub moves down by
        # body displacement minus jounce.
        wheel_vertical = heave + positions[:, 1] * roll - positions[:, 0] * pitch
        deflection = wheel_vertical - z
        tire = tire_deflection_load(planar.tire, deflection, np.zeros(4), 0.0)
        spring = _spring_forces(vehicle, z)
        wheel_load = tire.load
        body_vertical = np.sum(spring) - vehicle.sprung_mass * gravity - downforce
        roll_moment = spring[0] + spring[2] - spring[1] - spring[3]
        front_aero = downforce * planar.aero.aero_balance
        front_target = (vehicle.sprung_mass * gravity * vehicle.sprung_front_fraction
                        + front_aero)
        pitch_moment = np.sum(spring[:2]) - front_target
        unsprung = np.array([
            vehicle.unsprung_mass_front, vehicle.unsprung_mass_front,
            vehicle.unsprung_mass_rear, vehicle.unsprung_mass_rear,
        ])
        wheel_balance = wheel_load - spring - unsprung * gravity
        return np.r_[body_vertical, roll_moment, pitch_moment, wheel_balance]

    # Seven coordinates: body heave/roll/pitch and four jounces. Newton steps
    # use a scaled finite-difference Jacobian and backtracking for robustness.
    q = np.zeros(7)  # heave, roll, pitch, and four jounces
    for _ in range(80):
        r = residual(q)
        if np.linalg.norm(r, ord=np.inf) < 1e-8:
            break
        jac = np.empty((7, 7))
        for column in range(7):
            step = 1e-6 if column in (0, 3, 4, 5, 6) else 1e-7
            trial = q.copy()
            trial[column] += step
            jac[:, column] = (residual(trial) - r) / step
        delta, *_ = np.linalg.lstsq(jac, -r, rcond=None)
        scale = 1.0
        norm = np.linalg.norm(r)
        # A trial outside the suspension travel or the damper table raises
        # DomainError; count it as a rejected step, exactly like a step that
        # fails to reduce the residual.
        while scale > 1e-6:
            try:
                trial = residual(q + scale * delta)
            except ValueError:
                trial_norm = np.inf
            else:
                trial_norm = np.linalg.norm(trial)
            if trial_norm < norm:
                break
            scale *= 0.5
        q += scale * delta
    else:
        raise RuntimeError("static vertical equilibrium did not converge")
    if np.linalg.norm(residual(q), ord=np.inf) > 1e-6:
        raise RuntimeError("static vertical equilibrium residual exceeds tolerance")

    state = np.zeros(21)
    state[0] = speed
    state[6:9] = q[:3]
    state[9:13] = speed / planar.tire.r_1
    state[13:17] = q[3:]
    return VerticalState.from_vector(state)


#: Unknown order of the steady-corner Newton solve.
STEADY_CORNER_UNKNOWN_NAMES: tuple[str, ...] = (
    "steering", "sideslip", "driven_torque",
    "heave", "roll", "pitch",
    "jounce_fl", "jounce_fr", "jounce_rl", "jounce_rr",
    "slip_ratio_fl", "slip_ratio_fr", "slip_ratio_rl", "slip_ratio_rr",
)

#: Unknown scales: Newton runs in ``y = x / STEADY_CORNER_SCALES``.
STEADY_CORNER_SCALES = np.array(
    [0.1, 0.05, 20.0, 0.01, 0.01, 0.005,
     0.01, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.05]
)

#: Physical central-difference step of each unknown.
STEADY_CORNER_STEPS = np.array(
    [1e-6, 1e-6, 1e-4, 1e-6, 1e-7, 1e-7,
     1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6]
)

#: Residual rows taken from ``VerticalEvaluation.rates`` in the unknown order.
STEADY_CORNER_RATE_INDEX = np.array([0, 1, 5, 2, 3, 4, 17, 18, 19, 20, 9, 10, 11, 12])

#: Smallest speed [m/s] the solver supports (the contact points stay above the
#: tire model's 1 m/s slip-ratio speed floor).
MINIMUM_STEADY_CORNER_SPEED = 3.0

#: Default continuation step [m/s^2] and its floor after step halving.
DEFAULT_CONTINUATION_STEP = 0.1 * 9.80665
MINIMUM_CONTINUATION_STEP = 0.005 * 9.80665

#: Scaled-residual target of one Newton solve and the infinity norm a
#: continuation step must reach to be accepted as converged.
NEWTON_TOLERANCE = 1e-10
CONTINUATION_ACCEPTANCE = 1e-9
MAXIMUM_NEWTON_ITERATIONS = 40
MINIMUM_LINE_SEARCH_SCALE = 1.0 / 1024.0

#: Trial bounds. Jounce is the DSS travel, the load floor keeps all four tires
#: in contact, and the sideslip/slip-ratio caps reject wild Newton excursions.
MAXIMUM_TRIAL_JOUNCE = 0.0318
MINIMUM_TRIAL_LOAD = 2.0
MAXIMUM_TRIAL_SIDESLIP = 0.5
MAXIMUM_TRIAL_SLIP_RATIO = 0.5

#: Final validation tolerances of the returned point.
VALIDATION_RATE_TOLERANCE = 1e-6
VALIDATION_TIRE_RATE_TOLERANCE = 1e-8
VALIDATION_BODY_LATERAL_TOLERANCE = 1e-6

_WHEEL_INDEX = {"fl": 0, "fr": 1, "rl": 2, "rr": 3}


class _RejectedTrial(ValueError):
    """A Newton trial point outside the admissible steady-corner box."""


@dataclass(frozen=True, slots=True, eq=False)
class SteadyCornerSolution:
    """One converged steady-corner solve with its steering, torque and tires.

    ``state`` is the returned 21-value mechanical state; ``steering``,
    ``driven_torque``, ``wheel_torques`` and ``tire_states`` are the actuator and
    tire-state context that ``VerticalState`` cannot carry. ``residual_inf`` and
    ``iterations`` describe the Newton solve (scaled residual infinity norm and
    total Newton iterations including the straight seed solve), and
    ``continuation_steps`` counts the accepted lateral-acceleration steps.
    ``evaluation`` is the final ``VerticalEvaluation`` at the returned point.
    """

    state: VerticalState
    steering: float
    driven_torque: float
    wheel_torques: np.ndarray
    tire_states: tuple
    residual_inf: float
    iterations: int
    continuation_steps: int
    evaluation: VerticalEvaluation


def solve_steady_corner(
    vehicle: VerticalVehicleParameters,
    speed: float,
    lateral_acceleration: float,
) -> VerticalState:
    """Return the steady-corner vertical state at ``speed`` and ``ay``.

    ``speed`` is in m/s and ``lateral_acceleration`` in m/s^2, positive to the
    right (SAE +y: it loads the left wheels and produces negative SAE roll). The
    path target is ``V * r``, so the yaw rate is ``lateral_acceleration / speed``
    and the body-axis lateral acceleration is the target times ``cos(beta)``.

    Raises ``ValueError`` for a non-finite input or ``speed < 3 m/s``, and
    ``RuntimeError`` when no steady corner exists inside the admissible box
    (non-convergence, a lifted wheel, an out-of-range suspension point, a
    steering or torque limit, or a failed final validation). Never returns a
    partial state.
    """

    return _solve_steady_corner_detail(vehicle, speed, lateral_acceleration).state


def _solve_steady_corner_detail(
    vehicle: VerticalVehicleParameters,
    speed: float,
    lateral_acceleration: float,
    *,
    continuation_step: float | None = DEFAULT_CONTINUATION_STEP,
    minimum_step: float = MINIMUM_CONTINUATION_STEP,
    maximum_iterations: int = MAXIMUM_NEWTON_ITERATIONS,
    tolerance: float = NEWTON_TOLERANCE,
) -> SteadyCornerSolution:
    """Solve one steady corner and return the full solution context.

    ``continuation_step`` is the lateral-acceleration increment. Its default
    continues in 0.1 g steps, warm-started, and halves the step down to
    ``minimum_step`` when a step fails. ``continuation_step=None`` jumps
    straight to the target from the straight solution (used to cross-check the
    continuation path in tests).
    """

    speed = float(speed)
    lateral_acceleration = float(lateral_acceleration)
    if not (np.isfinite(speed) and np.isfinite(lateral_acceleration)):
        raise ValueError("speed and lateral_acceleration must be finite")
    if speed < MINIMUM_STEADY_CORNER_SPEED:
        raise ValueError(
            f"speed must be at least {MINIMUM_STEADY_CORNER_SPEED} m/s, received {speed}"
        )
    if continuation_step is not None and not (
        np.isfinite(continuation_step) and 0.0 < minimum_step <= continuation_step
    ):
        raise ValueError("continuation_step must be finite and at least minimum_step")
    if maximum_iterations < 1:
        raise ValueError("maximum_iterations must be positive")

    scale = _residual_scale(vehicle)
    straight = _static_guess(vehicle, speed)
    x, residual_inf, evaluation, iterations = _newton_solve(
        vehicle, straight, speed, 0.0, scale,
        tolerance=tolerance, maximum_iterations=maximum_iterations,
    )
    if not residual_inf < CONTINUATION_ACCEPTANCE:
        raise RuntimeError(
            f"steady corner straight solve failed to converge at speed={speed} "
            f"(residual {residual_inf:.3e})"
        )
    continuation_steps = 0

    if lateral_acceleration != 0.0:
        sign = 1.0 if lateral_acceleration > 0.0 else -1.0
        target = abs(lateral_acceleration)
        reached = 0.0
        if continuation_step is None:
            seed = _kinematic_seed(vehicle, speed, lateral_acceleration, straight)
            x, residual_inf, evaluation, step_iterations = _attempt(
                vehicle, seed, speed, lateral_acceleration, scale,
                tolerance, maximum_iterations,
            )
            iterations += step_iterations
            if not residual_inf < CONTINUATION_ACCEPTANCE:
                raise RuntimeError(
                    f"steady corner failed to converge at ay={lateral_acceleration:.6g} "
                    f"(residual {residual_inf:.3e})"
                )
            continuation_steps = 1
        else:
            step = min(continuation_step, target)
            while reached < target - 1e-12:
                trial_target = min(reached + step, target)
                ay = sign * trial_target
                # Never overwrite the last converged point with a failed
                # attempt: the smaller-step retry continues from it.
                trial_x, trial_residual, trial_evaluation, step_iterations = _attempt(
                    vehicle, x, speed, ay, scale, tolerance, maximum_iterations,
                )
                iterations += step_iterations
                if not trial_residual < CONTINUATION_ACCEPTANCE:
                    seed = _kinematic_seed(vehicle, speed, ay, straight)
                    trial_x, trial_residual, trial_evaluation, step_iterations = _attempt(
                        vehicle, seed, speed, ay, scale, tolerance, maximum_iterations,
                    )
                    iterations += step_iterations
                if not trial_residual < CONTINUATION_ACCEPTANCE:
                    if step <= minimum_step * (1.0 + 1e-12):
                        raise RuntimeError(
                            f"steady corner failed to converge at ay={ay:.6g} "
                            f"(residual {trial_residual:.3e})"
                        )
                    step = max(0.5 * step, minimum_step)
                    continue
                x = trial_x
                residual_inf = trial_residual
                evaluation = trial_evaluation
                reached = trial_target
                continuation_steps += 1
                step = min(continuation_step, target)

    _validate_solution(evaluation, lateral_acceleration, float(x[1]))
    state, torques, steering, tire_states = _corner_point(
        vehicle, x, speed, lateral_acceleration
    )
    return SteadyCornerSolution(
        state=state,
        steering=float(steering),
        driven_torque=float(x[2]),
        wheel_torques=torques,
        tire_states=tire_states,
        residual_inf=float(residual_inf),
        iterations=int(iterations),
        continuation_steps=int(continuation_steps),
        evaluation=evaluation,
    )


def _residual_scale(vehicle: VerticalVehicleParameters) -> np.ndarray:
    """Return the independent residual scale of each steady-corner row."""

    planar = vehicle.planar
    front_hop = planar.total_mass * 9.80665 / 4.0 / vehicle.unsprung_mass_front
    wheel_acceleration = (
        planar.tire.r_1 * planar.total_mass * 9.80665 / 4.0 / planar.wheel_inertia
    )
    return np.array(
        [
            9.80665,
            9.80665,
            planar.total_mass * 9.80665 * planar.wheelbase / planar.yaw_inertia,
            9.80665,
            vehicle.sprung_mass * 9.80665 * planar.half_track / vehicle.roll_inertia,
            vehicle.sprung_mass * 9.80665 * planar.wheelbase / 2.0 / vehicle.pitch_inertia,
            front_hop, front_hop, front_hop, front_hop,
            wheel_acceleration, wheel_acceleration, wheel_acceleration, wheel_acceleration,
        ]
    )


def _driven_wheels(vehicle: VerticalVehicleParameters) -> np.ndarray:
    """Return the driven-wheel mask in FL, FR, RL, RR order."""

    transmission = getattr(
        getattr(vehicle.planar.drivetrain, "drivetrain", None), "transmission", None
    )
    driven = getattr(transmission, "driven_wheels", ("rl", "rr"))
    mask = np.zeros(4, dtype=bool)
    for name in driven:
        if name not in _WHEEL_INDEX:
            raise ValueError(f"unknown driven wheel {name!r}")
        mask[_WHEEL_INDEX[name]] = True
    return mask


def _wheel_torque_limit(vehicle: VerticalVehicleParameters) -> float:
    """Return the per-wheel drive torque limit [N*m] of the compiled drivetrain."""

    limit = getattr(vehicle.planar.drivetrain, "peak_motor_torque", None)
    if limit is None:
        return np.inf
    return float(limit)


def _wheel_torques(vehicle: VerticalVehicleParameters, driven_torque: float) -> np.ndarray:
    """Return the four wheel torques for one shared driven-wheel torque."""

    return np.where(_driven_wheels(vehicle), driven_torque, 0.0)


def _static_guess(vehicle: VerticalVehicleParameters, speed: float) -> np.ndarray:
    """Return the straight-motion starting point of the steady-corner solve."""

    equilibrium = solve_static_equilibrium(vehicle, speed).as_vector()
    x = np.zeros(14)
    x[3:6] = equilibrium[6:9]
    x[6:10] = equilibrium[13:17]
    return x


def _kinematic_seed(
    vehicle: VerticalVehicleParameters,
    speed: float,
    lateral_acceleration: float,
    straight: np.ndarray,
) -> np.ndarray:
    """Return the Ackermann-free kinematic-steer seed used as a step fallback."""

    x = np.asarray(straight, dtype=float).copy()
    x[0] = vehicle.planar.wheelbase * lateral_acceleration / speed**2
    x[1] = 0.0
    x[2] = 0.0
    x[10:14] = 0.0
    return x


def _corner_point(
    vehicle: VerticalVehicleParameters,
    x: np.ndarray,
    speed: float,
    lateral_acceleration: float,
) -> tuple[VerticalState, np.ndarray, float, tuple]:
    """Build the state, torques, steering and steady tire states at ``x``.

    Raises ``_RejectedTrial`` for a point outside the admissible box (steering,
    torque, jounce, sideslip and slip-ratio bounds, and a tire out of contact).
    """

    planar = vehicle.planar
    steering, sideslip, driven_torque = float(x[0]), float(x[1]), float(x[2])
    heave, roll, pitch = float(x[3]), float(x[4]), float(x[5])
    jounce = np.asarray(x[6:10], dtype=float)
    kappa = np.asarray(x[10:14], dtype=float)
    if abs(steering) > planar.steering_maximum_angle:
        raise _RejectedTrial("steering beyond the maximum angle")
    if abs(driven_torque) > _wheel_torque_limit(vehicle):
        raise _RejectedTrial("drive torque beyond the wheel torque limit")
    if abs(sideslip) > MAXIMUM_TRIAL_SIDESLIP:
        raise _RejectedTrial("sideslip beyond its bound")
    if np.any(np.abs(kappa) > MAXIMUM_TRIAL_SLIP_RATIO):
        raise _RejectedTrial("slip ratio beyond its bound")
    if np.any(np.abs(jounce) > MAXIMUM_TRIAL_JOUNCE):
        raise _RejectedTrial("jounce beyond the suspension travel")

    yaw_rate = lateral_acceleration / speed
    v_x = speed * np.cos(sideslip)
    v_y = speed * np.sin(sideslip)
    positions = wheel_positions(planar)
    body_positions = sprung_wheel_positions(vehicle)
    body_vertical = heave + body_positions[:, 1] * roll - body_positions[:, 0] * pitch
    loads = np.maximum(
        np.asarray(
            tire_deflection_load(
                planar.tire,
                body_vertical - jounce - vehicle.suspension.kinematics.steering_lift(
                    wheel_steering_angles(steering, 4))[0],
                np.zeros(4), 0.0,
            ).load
        ),
        0.0,
    )
    if np.any(loads <= MINIMUM_TRIAL_LOAD):
        raise _RejectedTrial("a tire left the admissible load window")

    radii = loaded_radius(planar.tire, loads)
    angles = wheel_steering_angles(steering, 4) + vehicle.suspension.kinematics.toe(jounce)
    longitudinal, lateral = contact_velocities(
        PlanarState(v_x, v_y, yaw_rate, np.zeros(4)), positions, angles
    )
    denominator = np.maximum(np.abs(longitudinal), 1.0)
    wheel_speed = (kappa * denominator + longitudinal) / radii
    slip_ratio, slip_angle = slip_quantities(wheel_speed, radii, longitudinal, lateral, 1.0)
    state = VerticalState(
        v_x, v_y, 0.0, 0.0, 0.0, yaw_rate,
        heave, roll, pitch, *wheel_speed, *jounce, 0.0, 0.0, 0.0, 0.0,
    )
    tire_state = tuple(
        steady_rolling_state(
            planar.tire, vehicle.transient_tire,
            float(slip_angle[i]), float(slip_ratio[i]), float(loads[i]),
        )
        for i in range(4)
    )
    return state, _wheel_torques(vehicle, driven_torque), steering, tire_state


def _scaled_residual(
    vehicle: VerticalVehicleParameters,
    x: np.ndarray,
    speed: float,
    lateral_acceleration: float,
    scale: np.ndarray,
) -> tuple[np.ndarray, VerticalEvaluation]:
    """Return the scaled steady-corner residual and its evaluation."""

    state, torques, steering, tire_state = _corner_point(vehicle, x, speed, lateral_acceleration)
    evaluation = evaluate_vertical_dynamics(vehicle, state, torques, steering, tire_state)
    residual = np.asarray(evaluation.rates)[STEADY_CORNER_RATE_INDEX] / scale
    return residual, evaluation


def _newton_solve(
    vehicle: VerticalVehicleParameters,
    x0: np.ndarray,
    speed: float,
    lateral_acceleration: float,
    scale: np.ndarray,
    *,
    tolerance: float,
    maximum_iterations: int,
) -> tuple[np.ndarray, float, VerticalEvaluation, int]:
    """Run scaled central-difference Newton with a backtracking line search.

    Returns ``(x, residual_inf, evaluation, iterations)`` at the final point,
    which may be unconverged (the caller decides). Raises when a required
    evaluation leaves the admissible box.
    """

    y = np.asarray(x0, dtype=float) / STEADY_CORNER_SCALES
    residual, evaluation = _scaled_residual(
        vehicle, y * STEADY_CORNER_SCALES, speed, lateral_acceleration, scale
    )
    iterations = 0
    for iteration in range(maximum_iterations):
        norm = float(np.max(np.abs(residual)))
        if norm < tolerance:
            break
        jacobian = np.empty((14, 14))
        for column in range(14):
            step = STEADY_CORNER_STEPS[column]
            plus = (y * STEADY_CORNER_SCALES).copy()
            minus = plus.copy()
            plus[column] += step
            minus[column] -= step
            forward, _ = _scaled_residual(vehicle, plus, speed, lateral_acceleration, scale)
            backward, _ = _scaled_residual(vehicle, minus, speed, lateral_acceleration, scale)
            jacobian[:, column] = (
                (forward - backward) / (2.0 * step) * STEADY_CORNER_SCALES[column]
            )
        delta = _linear_step(jacobian, -residual)
        iterations += 1
        line_scale = 1.0
        accepted = False
        candidate_residual = residual
        candidate_evaluation = evaluation
        while line_scale > MINIMUM_LINE_SEARCH_SCALE:
            trial = (y + line_scale * delta) * STEADY_CORNER_SCALES
            try:
                trial_residual, trial_evaluation = _scaled_residual(
                    vehicle, trial, speed, lateral_acceleration, scale
                )
            except (ValueError, FloatingPointError, OverflowError, ZeroDivisionError):
                trial_residual = None
            if (
                trial_residual is not None
                and np.all(np.isfinite(trial_residual))
                and float(np.max(np.abs(trial_residual))) < norm * (1.0 - 1e-4 * line_scale)
            ):
                accepted = True
                candidate_residual = trial_residual
                candidate_evaluation = trial_evaluation
                break
            line_scale *= 0.5
        if not accepted:
            break
        y = y + line_scale * delta
        residual = candidate_residual
        evaluation = candidate_evaluation
    return (
        y * STEADY_CORNER_SCALES,
        float(np.max(np.abs(residual))),
        evaluation,
        iterations,
    )


def _linear_step(jacobian: np.ndarray, right_hand_side: np.ndarray) -> np.ndarray:
    """Solve the Newton system, falling back to least squares when ill-conditioned."""

    try:
        if np.linalg.cond(jacobian) <= 1e12:
            delta = np.linalg.solve(jacobian, right_hand_side)
            if np.all(np.isfinite(delta)):
                return delta
    except np.linalg.LinAlgError:
        pass
    delta, *_ = np.linalg.lstsq(jacobian, right_hand_side, rcond=None)
    return delta


def _attempt(
    vehicle: VerticalVehicleParameters,
    x0: np.ndarray,
    speed: float,
    lateral_acceleration: float,
    scale: np.ndarray,
    tolerance: float,
    maximum_iterations: int,
) -> tuple[np.ndarray, float, VerticalEvaluation, int]:
    """Run one Newton solve, treating a rejected starting point as a failure."""

    try:
        return _newton_solve(
            vehicle, x0, speed, lateral_acceleration, scale,
            tolerance=tolerance, maximum_iterations=maximum_iterations,
        )
    except (ValueError, FloatingPointError, OverflowError, ZeroDivisionError):
        return np.asarray(x0, dtype=float), np.inf, _empty_evaluation(), 0


def _empty_evaluation() -> VerticalEvaluation:
    """Return a placeholder evaluation for a rejected Newton attempt."""

    return VerticalEvaluation(
        rates=np.full(21, np.nan), mass_matrix=None,
        tire_state_rates=np.full(12, np.nan), normal_loads=np.zeros(4),
        tire_forces=np.zeros((4, 3)), corner_forces=None, lifted=np.ones(4, dtype=bool),
        out_of_range_count=0, body_accelerations=np.full(2, np.nan),
    )


def _validate_solution(
    evaluation: VerticalEvaluation,
    lateral_acceleration: float,
    sideslip: float,
) -> None:
    """Raise ``RuntimeError`` unless the converged point passes every gate."""

    rows = np.asarray(evaluation.rates)[
        [0, 1, 5, 2, 3, 4, 17, 18, 19, 20]
    ]
    if not np.all(np.isfinite(rows)) or float(np.max(np.abs(rows))) > VALIDATION_RATE_TOLERANCE:
        raise RuntimeError(
            f"steady corner rates exceed {VALIDATION_RATE_TOLERANCE:g} "
            f"(maximum {float(np.nanmax(np.abs(rows))):.3e})"
        )
    if int(evaluation.out_of_range_count) != 0:
        raise RuntimeError(
            f"steady corner left the damper table at {evaluation.out_of_range_count} corners"
        )
    if bool(np.any(evaluation.lifted)):
        raise RuntimeError("steady corner lifts a wheel")
    tire_rates = np.asarray(evaluation.tire_state_rates)
    if not np.all(np.isfinite(tire_rates)) or (
        float(np.max(np.abs(tire_rates))) > VALIDATION_TIRE_RATE_TOLERANCE
    ):
        raise RuntimeError(
            f"steady corner tire-state rates exceed {VALIDATION_TIRE_RATE_TOLERANCE:g} "
            f"(maximum {float(np.nanmax(np.abs(tire_rates))):.3e})"
        )
    body_lateral = float(evaluation.body_accelerations[1])
    # The path target is V*r, so the body-axis lateral acceleration is the target
    # times cos(beta) (the evaluator's yaw equation gives ay - r*v_x = 0).
    expected = lateral_acceleration * np.cos(sideslip)
    if not np.isfinite(body_lateral) or abs(body_lateral - expected) > VALIDATION_BODY_LATERAL_TOLERANCE:
        raise RuntimeError(
            f"steady corner body lateral acceleration {body_lateral:.6g} does not match "
            f"{expected:.6g}"
        )


def _spring_forces(vehicle: VerticalVehicleParameters, jounce: np.ndarray) -> np.ndarray:
    """Evaluate only the preload-bearing (rate-free) corner forces of the network."""
    return vehicle.suspension.network.static_forces(jounce)
