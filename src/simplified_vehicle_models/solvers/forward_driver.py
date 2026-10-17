"""Drive a complete vehicle state forward along a prescribed path.

The plant owns its state layout and local algebraic equations.
The driver uses physical observations instead of plant state indices.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol
from time import perf_counter

import numpy as np


class TrialStateError(ValueError):
    """Reject an integrator trial outside the plant state domain."""


@dataclass(frozen=True)
class RateSystem:
    """Store the system M * state_rate = forcing at one evaluation."""

    forcing: np.ndarray
    mass_matrix: np.ndarray | None = None

    def solve(self) -> np.ndarray:
        forcing = np.asarray(self.forcing, dtype=float)
        if forcing.ndim != 1 or not np.all(np.isfinite(forcing)):
            raise ValueError("forcing must be a finite vector")
        if self.mass_matrix is None:
            return forcing.copy()
        mass = np.asarray(self.mass_matrix, dtype=float)
        if mass.shape != (forcing.size, forcing.size) or not np.all(np.isfinite(mass)):
            raise ValueError("mass matrix must be finite and match the state")
        try:
            rate = np.linalg.solve(mass, forcing)
        except np.linalg.LinAlgError as error:
            raise ValueError("singular mass system requires a DAE backend") from error
        if not np.all(np.isfinite(rate)):
            raise ValueError("mass solve returned nonfinite rates")
        return rate


@dataclass(frozen=True)
class DriverObservation:
    distance: float
    lateral_error: float
    heading_error: float
    speed: float
    yaw_rate: float
    steering: float
    lateral_speed: float = 0.
    #: The most-locked wheel's braking slip ratio plus one, i.e. its distance
    #: above full lock (slip ratio -1, wheel stationary while the body still
    #: moves). Dimensionless; 1.0 or above means no wheel is meaningfully
    #: braking-slipped, 0.0 means a wheel has fully locked. A point-mass-
    #: style plant with no wheel states reports +inf, meaning "no wheel-lock
    #: risk is modeled," so PreviewDriver's brake governor is a no-op for it.
    wheel_speed_margin: float = float("inf")


@dataclass(frozen=True)
class DriverCommand:
    throttle: float
    brake_force: float
    steering_rate: float


class ForwardPlant(Protocol):
    """Supply observations and a complete, nonsingular implicit rate system.

    Each plant checks its local algebraic closure before returning rates.
    Singular contact systems require a separate DAE integration backend.
    """

    state_size: int

    def observe(self, state: np.ndarray) -> DriverObservation: ...

    def evaluate(self, time: float, state: np.ndarray,
                 command: DriverCommand) -> RateSystem: ...


#: Arc-length half-window for smoothing the feedforward acceleration profile.
_ACCELERATION_SMOOTHING_HALF_WINDOW_M = 4.0


def _smooth_along_distance(distance: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Return a periodic arc-length moving average of ``values``.

    ``distance`` is assumed strictly increasing with ``distance[-1]`` the lap
    length (``values[0]`` and ``values[-1]`` describe the same closed-path
    point). The average uses a symmetric window of
    ``_ACCELERATION_SMOOTHING_HALF_WINDOW_M`` on each side, wrapped around the
    seam, so the result stays periodic and introduces no boundary bias.
    """

    lap_length = float(distance[-1])
    half_window = _ACCELERATION_SMOOTHING_HALF_WINDOW_M
    if lap_length <= 4. * half_window:
        # A very short path cannot support this averaging window; return the
        # input unchanged rather than averaging the whole lap into one value.
        return np.array(values, dtype=float, copy=True)
    count = distance.size - 1
    body = values[:-1]
    offsets = distance[:-1]
    smoothed = np.empty_like(body)
    for index in range(count):
        delta = (offsets - offsets[index] + lap_length / 2.) % lap_length - lap_length / 2.
        weight = np.abs(delta) <= half_window
        smoothed[index] = np.mean(body[weight])
    return np.r_[smoothed, smoothed[0]]


@dataclass(frozen=True)
class SpeedPlan:
    """Store a requested speed envelope, not a proven vehicle capability."""

    distance: np.ndarray
    speed: np.ndarray
    curvature: np.ndarray

    def __post_init__(self):
        arrays = [np.array(value, dtype=float, copy=True)
                  for value in (self.distance, self.speed, self.curvature)]
        distance, speed, curvature = arrays
        if (distance.ndim != 1 or distance.size < 2 or distance[0] != 0.
                or speed.shape != distance.shape or curvature.shape != distance.shape
                or any(not np.all(np.isfinite(a)) for a in arrays)
                or np.any(np.diff(distance) <= 0) or np.any(speed <= 0)):
            raise ValueError("plan needs increasing distance and positive finite speed")
        for name, array in zip(("distance", "speed", "curvature"), arrays):
            array.setflags(write=False)
            object.__setattr__(self, name, array)
        acceleration = np.gradient(speed**2, distance) / 2.
        # The forward-accelerate / backward-brake sweep that builds the speed
        # envelope is a piecewise min() over two one-sided passes. Its speed is
        # continuous but its derivative (this feedforward acceleration) has a
        # real kink everywhere a forward-limited segment meets a backward-
        # limited one, i.e. at every corner entry. A raw np.gradient of that
        # kinked array can jump by several m/s^2 within one grid spacing.
        # PreviewDriver.command() feeds this feedforward straight into brake
        # force with no actuation lag, so an unsmoothed jump commands a near
        # step change in brake torque. Given this car's wheel inertia
        # (~0.2 kg*m^2) and brake torque (~350 N*m front), that is enough to
        # lock and reverse-spin a wheel within a few integrator substeps,
        # which then trips the forward plant's domain guard. Smooth the
        # feedforward with a short arc-length moving average so it varies
        # continuously, the way a real driver's brake application does.
        acceleration = _smooth_along_distance(distance, acceleration)
        acceleration.setflags(write=False)
        object.__setattr__(self, "acceleration", acceleration)

    def sample(self, distance: float) -> tuple[float, float, float]:
        position = float(distance) % self.distance[-1]
        return tuple(float(np.interp(position, self.distance, array))
                     for array in (self.speed, self.curvature, self.acceleration))


def _build_live_limit_speed_plan(distance, curvature, vehicle, storage_state=None,
                                 lateral_acceleration=None, tolerance=1.0e-8,
                                 maximum_speed=60.0) -> SpeedPlan:
    """Build a speed plan from the point-mass lap solution of the same vehicle.

    A planar 7DoF vehicle is reduced with ``point_mass_view_of_planar``: same
    mass, geometry, wheel-lift load transfer, aero, brake bias, and assembled
    powertrain, with the vehicle's own simplified UniTire reduced to its
    per-load pure-slip peak envelope. A point-mass vehicle is used as given.

    The plan is the converged flying-lap profile of the point-mass forward /
    backward solver. That solver evaluates the corner speed at every node and
    the acceleration and deceleration limits at the local segment speed, so
    aero load, drag, and the motor and battery power limits follow the speed
    the car is actually doing.

    ``lateral_acceleration`` is an optional explicit cap on the planned
    cornering acceleration. ``None`` (the default) applies no cap beyond the
    vehicle's own limits. ``maximum_speed`` caps the plan.
    """
    from simplified_vehicle_models.formulations.point_mass import point_mass_view_of_planar
    from simplified_vehicle_models.parameters.vehicle import PointMassVehicleParameters
    from simplified_vehicle_models.results.traces import LapMode
    from simplified_vehicle_models.solvers.forward_backward import solve_lap
    from simplified_vehicle_models.tracks.types import TrackProfile

    distance = np.asarray(distance, dtype=float)
    curvature = np.asarray(curvature, dtype=float)
    if (distance.ndim != 1 or distance.size < 3 or curvature.shape != distance.shape
            or not np.all(np.isfinite(distance)) or not np.all(np.isfinite(curvature))
            or np.any(np.diff(distance) <= 0.)):
        raise ValueError("live-limit planner needs an increasing finite distance grid")
    if not np.isfinite(maximum_speed) or maximum_speed <= 0:
        raise ValueError("live-limit planner maximum speed must be finite and positive")
    if lateral_acceleration is not None and (
            not np.isfinite(lateral_acceleration) or lateral_acceleration <= 0):
        raise ValueError("live-limit planner lateral acceleration cap must be finite and positive")

    if isinstance(vehicle, PointMassVehicleParameters):
        point_mass = vehicle
    else:
        point_mass = point_mass_view_of_planar(vehicle)
    magnitude = np.abs(curvature)
    # The point-mass solver consumes a finite radius magnitude.
    radius = 1. / np.maximum(magnitude, 1. / _PLANNER_MAXIMUM_RADIUS_M)
    track = TrackProfile(distance=distance, segment_length=np.diff(distance),
                         radius=radius, curvature=curvature, is_closed=True,
                         title="forward-driver plan")
    storage_trace = None
    if storage_state is not None:
        storage_trace = np.full(distance.size, float(storage_state.state_of_charge))
    lap = solve_lap(point_mass, track, LapMode.FLYING, root_tolerance=tolerance,
                    _storage_trace=storage_trace)
    if not lap.converged:
        raise RuntimeError("live-limit speed plan did not converge")
    speed = np.minimum(np.asarray(lap.speed, dtype=float), maximum_speed)
    if lateral_acceleration is not None:
        speed = np.minimum(speed, np.sqrt(lateral_acceleration / np.maximum(magnitude, 1.e-12)))
    speed[-1] = speed[0] = min(speed[0], speed[-1])
    return SpeedPlan(distance, speed, curvature)


#: Radius used for a straight by the live planner's point-mass solve.
_PLANNER_MAXIMUM_RADIUS_M = 1.0e4


def build_speed_plan(distance, curvature, maximum_speed=8., lateral_acceleration=None,
                      drive_acceleration=2., brake_acceleration=3., vehicle=None,
                      storage_state=None, live_limit=False) -> SpeedPlan:
    """Propagate a speed request around a closed path.

    With live_limit=False, use constant kinematic acceleration limits.
    With live_limit=True, use the vehicle's point-mass lap solution (see
    ``_build_live_limit_speed_plan``); ``lateral_acceleration`` is then an
    optional cap and ``None`` applies none.

    The forward plant must validate the requested trajectory independently.
    """
    if live_limit:
        if vehicle is None:
            raise ValueError("live_limit=True requires a vehicle parameter object")
        return _build_live_limit_speed_plan(distance, curvature, vehicle, storage_state,
                                             lateral_acceleration=lateral_acceleration,
                                            maximum_speed=maximum_speed)

    if lateral_acceleration is None:
        lateral_acceleration = 3.
    limits = np.asarray([maximum_speed, lateral_acceleration,
                         drive_acceleration, brake_acceleration], dtype=float)
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0):
        raise ValueError("planner limits must be finite and positive")
    distance = np.asarray(distance, dtype=float)
    curvature = np.asarray(curvature, dtype=float)
    initial = np.minimum(maximum_speed, np.sqrt(
        lateral_acceleration / np.maximum(np.abs(curvature), 1.e-12)))
    plan = SpeedPlan(distance, initial, curvature)
    speed_squared = plan.speed[:-1].copy()**2
    speed_squared[0] = min(speed_squared[0], plan.speed[-1]**2)
    spacing = np.diff(plan.distance)
    count = speed_squared.size
    for _ in range(2 * count + 2):
        previous = speed_squared.copy()
        for index in range(count):
            following = (index + 1) % count
            speed_squared[following] = min(speed_squared[following],
                speed_squared[index] + 2 * drive_acceleration * spacing[index])
        for index in range(count - 1, -1, -1):
            following = (index + 1) % count
            speed_squared[index] = min(speed_squared[index],
                speed_squared[following] + 2 * brake_acceleration * spacing[index])
        if np.max(np.abs(previous - speed_squared)) < 1.e-10:
            break
    else:
        raise RuntimeError("speed envelope did not converge")
    speed = np.sqrt(np.r_[speed_squared, speed_squared[0]])
    return SpeedPlan(plan.distance, speed, plan.curvature)


@dataclass(frozen=True)
class PreviewDriver:
    plan: SpeedPlan
    wheelbase: float
    maximum_steering: float
    maximum_steering_rate: float
    maximum_brake_force: float
    preview_seconds: float = 0.15
    speed_gain: float = 2.
    heading_gain: float = 1.5
    lateral_gain: float = 1.5
    steering_time_constant: float = 0.08
    live_limit: bool = False

    #: The brake governor ramps brake force down to zero as
    #: ``observation.wheel_speed_margin`` (the most-locked wheel's slip ratio
    #: plus one, i.e. its distance above full lock at slip ratio -1) falls
    #: from this value to zero. The default of 0.85 starts backing off once
    #: slip ratio passes about -0.15, in the range where braking force
    #: typically peaks for a real tire; it is a coarse proportional governor,
    #: not a slip-ratio ABS controller tuned to this tire's actual peak. Its
    #: job is only to keep the forward driver's Newton-trial domain guard
    #: from ever seeing a reversed wheel, not to reproduce real ABS behavior.
    #: A point-mass-style plant with no wheel states reports a +inf margin,
    #: so the governor is always fully open (a no-op) for it.
    brake_governor_margin: float = 0.85

    #: Time for the foot to take a pedal through its full travel (0 to 100 %
    #: throttle, or 0 to maximum brake force), in seconds. A real driver needs
    #: about 0.15--0.20 s. ``None`` keeps the historic instantaneous pedals.
    #: When set, ``solve_forward`` integrates one signed pedal position
    #: (+1 full throttle, -1 full brake) as an extra driver state, so the
    #: throttle and brake can never overlap and moving from full throttle to
    #: full brake takes two full travels.
    pedal_travel_time: float | None = None
    #: First-order time constant of the foot tracking its pedal target inside
    #: the slew-rate limit. Keeps the pedal rate continuous in the state.
    pedal_time_constant: float = 0.02

    def __post_init__(self):
        values = [self.wheelbase, self.maximum_steering, self.maximum_steering_rate,
                  self.maximum_brake_force, self.speed_gain, self.heading_gain,
                  self.lateral_gain, self.steering_time_constant, self.brake_governor_margin]
        if (not np.all(np.isfinite(values)) or min(values) <= 0
                or not np.isfinite(self.preview_seconds) or self.preview_seconds < 0):
            raise ValueError("driver gains and limits must be finite and positive")
        if self.pedal_travel_time is not None and (
                not np.isfinite(self.pedal_travel_time) or self.pedal_travel_time <= 0
                or not np.isfinite(self.pedal_time_constant) or self.pedal_time_constant <= 0):
            raise ValueError("pedal travel time and time constant must be finite and positive")

    @property
    def has_pedal_dynamics(self) -> bool:
        return self.pedal_travel_time is not None

    def pedal_target(self, command: DriverCommand) -> float:
        """Map a requested command to the signed pedal position."""
        return float(command.throttle - command.brake_force / self.maximum_brake_force)

    def pedal_rate(self, position: float, target: float) -> float:
        """Return the slew-limited rate of the signed pedal position."""
        limit = 1. / self.pedal_travel_time
        return float(np.clip((target - position) / self.pedal_time_constant, -limit, limit))

    def realized_command(self, position: float, requested: DriverCommand) -> DriverCommand:
        """Return the command the plant sees at a pedal position.

        Steering keeps the requested rate; the pedals follow the integrated
        foot position, clipped to the physical travel.
        """
        position = float(np.clip(position, -1., 1.))
        return DriverCommand(max(position, 0.), max(-position, 0.) * self.maximum_brake_force,
                             requested.steering_rate)

    def command(self, observation: DriverObservation) -> DriverCommand:
        observed = vars(observation)
        if not all(np.isfinite(value) or name == "wheel_speed_margin"
                  for name, value in observed.items()):
            raise ValueError("driver observations must be finite")
        if np.isnan(observation.wheel_speed_margin):
            raise ValueError("driver observations must be finite")
        target, curvature, feedforward = self.plan.sample(observation.distance)
        _, preview_curvature, _ = self.plan.sample(
            observation.distance + max(observation.speed, 0.) * self.preview_seconds)
        course_error = observation.heading_error + np.arctan2(
            observation.lateral_speed, max(observation.speed, 0.25))
        desired_steering = (
            np.arctan(self.wheelbase * preview_curvature)
            - self.heading_gain * course_error
            - np.arctan2(self.lateral_gain * observation.lateral_error,
                         max(observation.speed, 1.))
            - 0.1 * (observation.yaw_rate - observation.speed * curvature))
        desired_steering = float(np.clip(desired_steering,
                                         -self.maximum_steering, self.maximum_steering))
        steering_rate = float(np.clip(
            (desired_steering - observation.steering) / self.steering_time_constant,
            -self.maximum_steering_rate, self.maximum_steering_rate))
        acceleration = feedforward + self.speed_gain * (target - observation.speed)
        throttle = float(np.clip(acceleration / 6., 0., 1.))
        brake_request = float(np.clip(-acceleration / 6., 0., 1.))
        # Govern the brake request against how close the most-locked wheel's
        # slip ratio is to full lock. A point-mass-style plant reports +inf
        # here, so clip(..., 1) below saturates at 1 (a no-op) for it; the
        # planar plant reports a finite margin that shrinks toward zero as a
        # wheel approaches lock.
        governor = float(np.clip(observation.wheel_speed_margin / self.brake_governor_margin,
                                 0., 1.))
        brake_force = brake_request * governor * self.maximum_brake_force
        return DriverCommand(throttle, brake_force, steering_rate)


@dataclass(frozen=True)
class ForwardResult:
    time: np.ndarray
    state: np.ndarray
    lap_completed: bool
    wall_seconds: float
    evaluations: int
    message: str
    rejected_domain_trials: int = 0
    #: Integrated signed pedal position (+1 full throttle, -1 full brake) at
    #: each output time, or ``None`` when the driver has instantaneous pedals.
    pedal: np.ndarray | None = None
    #: Physical checks at the dense-output midpoint of each accepted interval.
    #: Sampling one interior point does not certify the continuous interval.
    midpoint_sample_validation_complete: bool = False
    #: Conservative ``[min, max]`` of every state component (and the pedal
    #: coordinate) over each accepted interval, from the exact BDF dense-output
    #: polynomial. ``None`` for a method without a verifiable interpolant. It
    #: bounds the reported interpolant, not the true ODE trajectory.
    dense_bounds: np.ndarray | None = None
    #: Exact per-interval BDF dense polynomials (``DensePolynomials``) or ``None``.
    dense_polynomials: object | None = None
    #: Midpoint samples taken, and the largest scaled defect
    #: ``|dy/dt - f(t, y)| * step / (atol + rtol * |y|)`` among them. The defect
    #: is informational evidence about the interpolant, not a bound on it.
    dense_samples: int = 0
    dense_defect_scaled_max: float = float("nan")


def solve_forward(plant: ForwardPlant, driver: PreviewDriver, initial_state,
                  duration=300., wall_seconds=290., maximum_step=0.02,
                  rtol=1.e-6, atol=1.e-8, method="Radau",
                   initial_pedal: float | None = None,
                   maximum_wall_seconds: float = 300.) -> ForwardResult:
    """Integrate all states with a stiff method and stop at the path end.

    ``wall_seconds`` is this call's wall budget and ``maximum_wall_seconds`` is
    the ceiling that budget may not exceed. The ceiling defaults to 300 s so the
    7DoF acceptance limit is preserved; a rung that may run longer (the vertical
    14DoF solver entry point) must raise it explicitly. Both are wall-clock
    seconds of the caller's process. The caller must also apply a
    parent-process timeout to include setup. Completion does not assert physical
    validity or hot-lap periodicity.
    """
    from scipy.integrate import solve_ivp

    limits = np.asarray([duration, wall_seconds, maximum_step, rtol, atol,
                         maximum_wall_seconds], dtype=float)
    if not np.all(np.isfinite(limits)) or np.any(limits <= 0):
        raise ValueError("positive finite limits including maximum_wall_seconds are required")
    if wall_seconds > maximum_wall_seconds:
        raise ValueError("wall_seconds must not exceed maximum_wall_seconds")
    if method not in ("Radau", "BDF"):
        raise ValueError("forward method must be Radau or BDF")
    plant_initial = np.array(initial_state, dtype=float, copy=True)
    plant_size = plant.state_size
    if plant_initial.shape != (plant_size,) or not np.all(np.isfinite(plant_initial)):
        raise ValueError("initial state must match the complete plant layout")
    pedal_dynamics = bool(getattr(driver, "has_pedal_dynamics", False))
    if initial_pedal is not None and (not pedal_dynamics or not np.isfinite(initial_pedal)
                                      or not -1. <= initial_pedal <= 1.):
        raise ValueError("initial pedal requires enabled pedal dynamics and position in [-1, 1]")
    if pedal_dynamics:
        # The foot starts where the driver wants it, so the run does not open
        # with a pedal transient.
        requested = driver.command(plant.observe(plant_initial))
        initial = np.r_[plant_initial, driver.pedal_target(requested)
                        if initial_pedal is None else initial_pedal]
    else:
        initial = plant_initial
    start = perf_counter()
    evaluations = 0
    rejected_domain_trials = 0

    def state_rates(time, state):
        plant_state = state[:plant_size]
        requested = driver.command(plant.observe(plant_state))
        if not pedal_dynamics:
            return plant.evaluate(time, plant_state, requested).solve()
        position = float(state[plant_size])
        realized = driver.realized_command(position, requested)
        rates = plant.evaluate(time, plant_state, realized).solve()
        return np.r_[rates, driver.pedal_rate(position, driver.pedal_target(requested))]

    # Reject an invalid initial state before the integrator can retry trials.
    state_rates(0., initial)

    def rhs(time, state):
        nonlocal evaluations, rejected_domain_trials
        if perf_counter() - start >= wall_seconds:
            raise TimeoutError("forward integration exceeded its wall budget")
        evaluations += 1
        try:
            rates = state_rates(time, state)
        except TrialStateError:
            rejected_domain_trials += 1
            # Radau rejects this Newton trial and reduces its integration step.
            return np.full(initial.shape, np.nan)
        if rates.shape != initial.shape:
            raise ValueError("plant rates must retain the complete state")
        return rates

    def finish(time, state):
        return plant.observe(state[:plant_size]).distance - driver.plan.distance[-1]

    finish.terminal = True
    finish.direction = 1
    def jacobian(time, state):
        # Local closure tolerance can obscure machine-scale perturbations.
        baseline = rhs(time, state)
        matrix = np.zeros((state.size, state.size))
        if not np.all(np.isfinite(baseline)):
            # The predictor state is outside the domain; return a zero Jacobian
            # so the integrator rejects the step and retries with a smaller step.
            return matrix
        for index in range(state.size):
            step = 1.e-5 * max(abs(state[index]), 1.)
            rates = None
            # Try progressively smaller perturbations; keep the first admissible one.
            for _ in range(12):
                trial = state.copy()
                trial[index] += step
                rates = rhs(time, trial)
                if np.all(np.isfinite(rates)):
                    break
                trial[index] = state[index] - step
                rates = rhs(time, trial)
                if np.all(np.isfinite(rates)):
                    step = -step
                    break
                rates = None
                step *= 0.25
            if rates is not None and np.all(np.isfinite(rates)):
                matrix[:, index] = (rates - baseline) / step
            # If no admissible perturbation is found, leave the column as zero.
        return matrix

    result = solve_ivp(rhs, (0., duration), initial, method=method, jac=jacobian,
                       rtol=rtol, atol=atol, max_step=maximum_step, events=finish, dense_output=True)
    if not result.success:
        raise RuntimeError(result.message)
    # Bound every component of the dense polynomial over each accepted interval
    # before the replay loops, so a later timeout cannot hide this result.
    dense_bounds = None
    dense_polynomials = None
    if method == "BDF":
        from simplified_vehicle_models.solvers.dense_domain import (
            UnsupportedDenseOutputError, dense_component_bounds, dense_polynomial_coefficients)
        try:
            dense_bounds = dense_component_bounds(result.sol)
            dense_polynomials = dense_polynomial_coefficients(result.sol)
        except UnsupportedDenseOutputError:
            dense_bounds = None
            dense_polynomials = None
    # Recheck every accepted state without the Newton-trial rejection path.
    # Checking a sparse subset cannot certify contact across the lap.
    for time_value, state in zip(result.t, result.y.T):
        if perf_counter() - start >= wall_seconds:
            raise TimeoutError("accepted-state validation exceeded its wall budget")
        state_rates(time_value, state)
    # Endpoints are not enough: reconstruct each interval's interior state
    # from the integrator's dense solution, including its pedal coordinate.
    defect_max = 0.
    samples = 0
    for left, right in zip(result.t[:-1], result.t[1:]):
        if perf_counter() - start >= wall_seconds:
            raise TimeoutError("between-step validation exceeded its wall budget")
        midpoint = (left + right) / 2.
        dense_state = result.sol(midpoint)
        rates = state_rates(midpoint, dense_state)
        # The BDF interpolant is a polynomial, so a small central difference
        # recovers its slope to rounding. Compare it with the model slope.
        width = right - left
        probe = 1.e-3 * width
        slope = (result.sol(midpoint + probe) - result.sol(midpoint - probe)) / (2. * probe)
        scaled = np.abs(slope - rates) * abs(width) / (atol + rtol * np.abs(dense_state))
        defect_max = max(defect_max, float(np.max(scaled)))
        samples += 1
    elapsed = perf_counter() - start
    if elapsed >= wall_seconds:
        raise TimeoutError("forward integration exceeded its wall budget")
    pedal = np.clip(result.y[plant_size], -1., 1.) if pedal_dynamics else None
    return ForwardResult(result.t, result.y[:plant_size], bool(result.t_events[0].size),
                         elapsed, evaluations, result.message, rejected_domain_trials, pedal, True,
                         dense_bounds, dense_polynomials, samples, defect_max)
