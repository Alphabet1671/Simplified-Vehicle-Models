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


class ForwardIntegrationError(RuntimeError):
    """An IVP solver stopped without producing an accepted forward result."""
    def __init__(self, message, diagnostics, last_state, last_time):
        super().__init__(message)
        snapshot=np.array(last_state,dtype=float,copy=True)
        snapshot.setflags(write=False)
        self.last_state=snapshot
        self.last_time=float(last_time)
        self.diagnostics=dict(diagnostics)


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


def _smooth_open_along_distance(distance: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Return an arc-length moving average of ``values`` along an open path.

    The window is symmetric and shrinks towards each end of the course so it
    never reaches past the first or last node. The end nodes keep their own
    value, so the average introduces no one-sided bias there.
    """

    length = float(distance[-1] - distance[0])
    half_window = _ACCELERATION_SMOOTHING_HALF_WINDOW_M
    if length <= 4. * half_window:
        return np.array(values, dtype=float, copy=True)
    smoothed = np.empty_like(values, dtype=float)
    for index, position in enumerate(distance):
        reach = min(half_window, position - distance[0], distance[-1] - position)
        smoothed[index] = np.mean(values[np.abs(distance - position) <= reach + 1.e-12])
    return smoothed


@dataclass(frozen=True)
class SpeedPlan:
    """Store a requested speed envelope, not a proven vehicle capability.

    A closed plan is one periodic lap: ``sample`` wraps and every speed is
    positive. An open plan (``is_closed=False``) is a course with a start and an
    end: ``sample`` holds the end values outside the course, and the first
    speed may be zero for a standing start.
    """

    distance: np.ndarray
    speed: np.ndarray
    curvature: np.ndarray
    is_closed: bool = True

    def __post_init__(self):
        arrays = [np.array(value, dtype=float, copy=True)
                  for value in (self.distance, self.speed, self.curvature)]
        distance, speed, curvature = arrays
        if self.is_closed:
            speed_valid = not np.any(speed <= 0)
        else:
            speed_valid = speed.size > 0 and speed[0] >= 0. and not np.any(speed[1:] <= 0)
        if (distance.ndim != 1 or distance.size < 2 or distance[0] != 0.
                or speed.shape != distance.shape or curvature.shape != distance.shape
                or any(not np.all(np.isfinite(a)) for a in arrays)
                or np.any(np.diff(distance) <= 0) or not speed_valid):
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
        if self.is_closed:
            acceleration = _smooth_along_distance(distance, acceleration)
        else:
            acceleration = _smooth_open_along_distance(distance, acceleration)
        acceleration.setflags(write=False)
        object.__setattr__(self, "acceleration", acceleration)

    def sample(self, distance: float) -> tuple[float, float, float]:
        if self.is_closed:
            position = float(distance) % self.distance[-1]
        else:
            # An open course does not repeat: np.interp holds the end values.
            position = float(distance)
        return tuple(float(np.interp(position, self.distance, array))
                     for array in (self.speed, self.curvature, self.acceleration))


def _build_live_limit_speed_plan(distance, curvature, vehicle, storage_state=None,
                                 lateral_acceleration=None, tolerance=1.0e-8,
                                 maximum_speed=60.0, open_course=False,
                                 lateral_margin=1.0) -> SpeedPlan:
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

    ``lateral_margin`` (above 0, at most 1; default 1) is the fraction of the
    vehicle's point-mass cornering grip that the plan uses. The point-mass limit
    is a steady-state limit, so a car planned at 100 % has no grip left to
    change direction or to recover from a transient. The corner speed is solved
    on the corner radius multiplied by the margin, so it falls roughly with the
    square root of the margin and the lateral acceleration used falls roughly
    with the margin itself. The acceleration and braking limits are re-solved
    on that profile and straights are unchanged. At 1 the plan is what it was.

    ``open_course=True`` plans a standing start on an open course instead: the
    point-mass ``STANDING_START`` profile, with speed 0 at the first node, a free
    finish speed, and a non-periodic ``SpeedPlan``.
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
    if not np.isfinite(lateral_margin) or not 0. < lateral_margin <= 1.:
        raise ValueError("live-limit planner lateral margin must be above 0 and at most 1")

    if isinstance(vehicle, PointMassVehicleParameters):
        point_mass = vehicle
    else:
        point_mass = point_mass_view_of_planar(vehicle)
    magnitude = np.abs(curvature)
    # The point-mass solver consumes a finite radius magnitude.
    radius = lateral_margin / np.maximum(magnitude, 1. / _PLANNER_MAXIMUM_RADIUS_M)
    track = TrackProfile(distance=distance, segment_length=np.diff(distance),
                         radius=radius, curvature=curvature, is_closed=not open_course,
                         title="forward-driver plan")
    storage_trace = None
    if storage_state is not None:
        storage_trace = np.full(distance.size, float(storage_state.state_of_charge))
    mode = LapMode.STANDING_START if open_course else LapMode.FLYING
    lap = solve_lap(point_mass, track, mode, root_tolerance=tolerance,
                    _storage_trace=storage_trace)
    if not lap.converged:
        raise RuntimeError("live-limit speed plan did not converge")
    speed = np.minimum(np.asarray(lap.speed, dtype=float), maximum_speed)
    if lateral_acceleration is not None:
        speed = np.minimum(speed, np.sqrt(lateral_acceleration / np.maximum(magnitude, 1.e-12)))
    if open_course:
        # A standing start begins at rest and the finish speed is free.
        speed[0] = 0.
        return SpeedPlan(distance, speed, curvature, is_closed=False)
    speed[-1] = speed[0] = min(speed[0], speed[-1])
    return SpeedPlan(distance, speed, curvature)


#: Radius used for a straight by the live planner's point-mass solve.
_PLANNER_MAXIMUM_RADIUS_M = 1.0e4


def build_speed_plan(distance, curvature, maximum_speed=8., lateral_acceleration=None,
                      drive_acceleration=2., brake_acceleration=3., vehicle=None,
                      storage_state=None, live_limit=False, open_course=False,
                      lateral_margin=1.0) -> SpeedPlan:
    """Propagate a speed request around a closed path, or along an open course.

    With live_limit=False, use constant kinematic acceleration limits.
    With live_limit=True, use the vehicle's point-mass lap solution (see
    ``_build_live_limit_speed_plan``); ``lateral_acceleration`` is then an
    optional cap and ``None`` applies none.

    ``lateral_margin`` is the fraction of the point-mass cornering grip a
    live-limit plan uses (see ``_build_live_limit_speed_plan``); the constant
    planner takes its limit from ``lateral_acceleration`` and ignores it.

    ``open_course=True`` plans a standing start: speed 0 at the first node, no
    wrap at the end, and a free finish speed. The result is not periodic.

    The forward plant must validate the requested trajectory independently.
    """
    if live_limit:
        if vehicle is None:
            raise ValueError("live_limit=True requires a vehicle parameter object")
        return _build_live_limit_speed_plan(distance, curvature, vehicle, storage_state,
                                             lateral_acceleration=lateral_acceleration,
                                            maximum_speed=maximum_speed,
                                            open_course=open_course,
                                            lateral_margin=lateral_margin)

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
    if open_course:
        if np.any(np.diff(distance) <= 0.) or distance.size < 2:
            raise ValueError("plan needs increasing distance and positive finite speed")
        speed_squared = initial**2
        speed_squared[0] = 0.
        spacing = np.diff(distance)
        for index in range(speed_squared.size - 1):
            speed_squared[index + 1] = min(speed_squared[index + 1],
                speed_squared[index] + 2 * drive_acceleration * spacing[index])
        for index in range(speed_squared.size - 2, -1, -1):
            speed_squared[index] = min(speed_squared[index],
                speed_squared[index + 1] + 2 * brake_acceleration * spacing[index])
        speed_squared[0] = 0.
        return SpeedPlan(distance, np.sqrt(speed_squared), curvature, is_closed=False)
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
class HistoricalPreviewDecision:
    """Actual historical speed-law values; no envelope or forecast is implied."""
    distance: float
    target_speed: float
    feedforward_acceleration: float
    acceleration_request: float
    requested_throttle: float
    ungoverned_brake_force: float
    governor_factor: float
    governed_brake_force: float

    def to_dict(self):
        from dataclasses import asdict
        return {"schema": "historical-preview-v1", **asdict(self)}


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
    #: Optional signed-pedal slew bounds (normalized pedal units per second).
    throttle_apply_rate: float | None = None
    throttle_release_rate: float | None = None
    brake_apply_rate: float | None = None
    brake_release_rate: float | None = None
    envelope_policy: object | None = None
    brake_planning_context: object | None = None
    brake_planning_context_cache: object | None = None
    brake_planning_context_factory: object | None = None
    brake_current_context_factory: object | None = None
    brake_current_predictor: object | None = None
    hybrid_policy: object | None = None
    hybrid_kinematic_context_factory: object | None = None
    #: Opt-in first-order actuator inverse; preview preserves existing behavior.
    steering_feedforward: str = "preview"

    def __post_init__(self):
        providers=sum(item is not None for item in (self.brake_planning_context,self.brake_planning_context_cache,self.brake_planning_context_factory))
        if providers>1:
            raise ValueError("configure only one brake planning context provider")
        if self.brake_planning_context_cache is not None and not callable(self.brake_planning_context_cache):
            raise TypeError("brake_planning_context_cache must be callable")
        if self.brake_planning_context_factory is not None and not callable(self.brake_planning_context_factory):
            raise TypeError("brake_planning_context_factory must be callable")
        if self.brake_current_context_factory is not None and not callable(self.brake_current_context_factory):
            raise TypeError("brake_current_context_factory must be callable")
        if self.brake_current_predictor is not None and not callable(getattr(self.brake_current_predictor,"request_current",None)):
            raise TypeError("brake_current_predictor must provide request_current")
        if self.brake_current_predictor is not None and self.brake_current_context_factory is None and self.hybrid_policy is None:
            raise ValueError("current inverse requires current-context factory")
        if self.steering_feedforward not in ("preview", "actuator-inverse"):
            raise ValueError("steering_feedforward must be preview or actuator-inverse")
        if self.hybrid_policy is not None:
            from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy
            if not isinstance(self.hybrid_policy, HybridLongitudinalPolicy): raise TypeError("hybrid_policy must be HybridLongitudinalPolicy")
            if self.envelope_policy is not None: raise ValueError("configure only one longitudinal policy")
            pc=self.hybrid_policy.config.pedal_context
            if not self.has_pedal_dynamics: raise ValueError("hybrid policy requires signed pedal dynamics")
            if self.has_pedal_dynamics:
                release=self.throttle_release_rate or 1./self.pedal_travel_time
                apply=self.brake_apply_rate or 1./self.pedal_travel_time
                throttle_apply=self.throttle_apply_rate or 1./self.pedal_travel_time
                brake_release=self.brake_release_rate or 1./self.pedal_travel_time
                actual=__import__('simplified_vehicle_models.solvers.envelope_driver',fromlist=['PedalPolicyContext']).PedalPolicyContext(release,apply,self.pedal_time_constant,self.maximum_brake_force,self.brake_governor_margin,throttle_apply,brake_release)
                if pc is None or actual!=pc: raise ValueError("hybrid pedal context must exactly match PreviewDriver dynamics")
            if not np.isclose(self.hybrid_policy.config.maximum_brake_force_n,self.maximum_brake_force): raise ValueError("hybrid maximum brake force must match PreviewDriver")
            if self.brake_current_predictor is not None and self.brake_current_predictor is not self.hybrid_policy.predictor: raise ValueError("adapter and policy current predictors must match")
        if self.hybrid_kinematic_context_factory is not None and not callable(self.hybrid_kinematic_context_factory): raise TypeError("hybrid_kinematic_context_factory must be callable")
        if self.envelope_policy is not None:
            from simplified_vehicle_models.solvers.envelope_driver import EnvelopePolicy
            if not isinstance(self.envelope_policy, EnvelopePolicy):
                raise TypeError("envelope_policy must be EnvelopePolicy or None")
            envelope = self.envelope_policy.envelope
            if (bool(envelope.is_closed) != bool(self.plan.is_closed)
                    or not np.isclose(envelope.distance[-1], self.plan.distance[-1])):
                raise ValueError("envelope and speed plan path topology/length must match")
        values = [self.wheelbase, self.maximum_steering, self.maximum_steering_rate,
                  self.maximum_brake_force, self.speed_gain, self.heading_gain,
                  self.lateral_gain, self.steering_time_constant, self.brake_governor_margin]
        if (not np.all(np.isfinite(values)) or min(values) <= 0
                or not np.isfinite(self.preview_seconds) or self.preview_seconds < 0):
            raise ValueError("driver gains and limits must be finite and positive")
        rates = (self.throttle_apply_rate, self.throttle_release_rate,
                 self.brake_apply_rate, self.brake_release_rate)
        configured = tuple(rate is not None for rate in rates)
        if any(configured) and not all(configured):
            raise ValueError("all four directional pedal rates must be configured together")
        if all(configured) and (not np.all(np.isfinite(rates)) or min(rates) <= 0.):
            raise ValueError("directional pedal rates must be finite and positive")
        if self.pedal_travel_time is not None and (
                not np.isfinite(self.pedal_travel_time) or self.pedal_travel_time <= 0):
            raise ValueError("pedal travel time must be finite and positive")
        if (self.pedal_travel_time is not None or all(configured)) and (
                not np.isfinite(self.pedal_time_constant) or self.pedal_time_constant <= 0):
            raise ValueError("pedal time constant must be finite and positive")

    @property
    def has_pedal_dynamics(self) -> bool:
        return self.pedal_travel_time is not None or self.throttle_apply_rate is not None

    def pedal_target(self, command: DriverCommand) -> float:
        """Map a requested command to the signed pedal position."""
        return float(command.throttle - command.brake_force / self.maximum_brake_force)

    def pedal_rate(self, position: float, target: float) -> float:
        """Return the slew-limited rate of the signed pedal position."""
        if not self.has_pedal_dynamics:
            raise ValueError("pedal dynamics disabled")
        delta = target - position
        if self.throttle_apply_rate is None:
            limit = 1. / self.pedal_travel_time
        elif position > 0.:
            limit = self.throttle_release_rate if delta < 0. else self.throttle_apply_rate
        elif position < 0.:
            limit = self.brake_release_rate if delta > 0. else self.brake_apply_rate
        else:
            limit = self.throttle_apply_rate if delta >= 0. else self.brake_apply_rate
        return float(np.clip(delta / self.pedal_time_constant, -limit, limit))

    def realized_command(self, position: float, requested: DriverCommand) -> DriverCommand:
        """Return the command the plant sees at a pedal position.

        Steering keeps the requested rate; the pedals follow the integrated
        foot position, clipped to the physical travel.
        """
        position = float(np.clip(position, -1., 1.))
        return DriverCommand(max(position, 0.), max(-position, 0.) * self.maximum_brake_force,
                             requested.steering_rate)

    def _steering_command(self, observation: DriverObservation) -> float:
        _, curvature, _ = self.plan.sample(observation.distance)
        _, preview_curvature, _ = self.plan.sample(
            observation.distance + max(observation.speed, 0.) * self.preview_seconds)
        course_error = observation.heading_error + np.arctan2(
            observation.lateral_speed, max(observation.speed, 0.25))
        feedforward = np.arctan(self.wheelbase * preview_curvature)
        if self.steering_feedforward == "actuator-inverse":
            # Invert delta_dot=(target-delta)/tau for the nominal curvature
            # steering: target=delta_ref+tau*v*d(delta_ref)/ds. A centered
            # derivative over 0.2 m bounds sensitivity to piecewise-plan knots;
            # this is a numerical stencil, not a track-specific gain. The
            # physical angle and slew limits remain applied below.
            half_span = 0.1
            before = self.plan.sample(observation.distance - half_span)[1]
            after = self.plan.sample(observation.distance + half_span)[1]
            span = 2. * half_span
            if not self.plan.is_closed:
                low = max(float(self.plan.distance[0]), observation.distance - half_span)
                high = min(float(self.plan.distance[-1]), observation.distance + half_span)
                before = self.plan.sample(low)[1]
                after = self.plan.sample(high)[1]
                span = max(high - low, np.finfo(float).eps)
            gradient = (np.arctan(self.wheelbase * after)
                        - np.arctan(self.wheelbase * before)) / span
            feedforward = (np.arctan(self.wheelbase * curvature)
                           + self.steering_time_constant * max(observation.speed, 0.) * gradient)
        desired_steering = (
            feedforward
            - self.heading_gain * course_error
            - np.arctan2(self.lateral_gain * observation.lateral_error,
                         max(observation.speed, 1.))
            - 0.1 * (observation.yaw_rate - observation.speed * curvature))
        desired_steering = float(np.clip(desired_steering, -self.maximum_steering, self.maximum_steering))
        return float(np.clip((desired_steering-observation.steering)/self.steering_time_constant,
                             -self.maximum_steering_rate,self.maximum_steering_rate))

    def build_brake_planning_context(self, plant, observation, pedal_context, plant_state):
        if self.brake_planning_context_factory is None:
            return self.brake_planning_context
        return self.brake_planning_context_factory(plant,observation,pedal_context,plant_state)

    def command_with_context(self, observation, pedal_position=None, plant_state=None, plant=None, planning_context_override=None, use_resolved_context=False):
        if self.hybrid_policy is not None:
            if plant_state is None or plant is None:
                raise ValueError("hybrid policy requires plant and complete current state")
            from simplified_vehicle_models.solvers.hybrid_driver import HybridLongitudinalPolicy
            if self.hybrid_kinematic_context_factory is not None:
                kin=self.hybrid_kinematic_context_factory(plant, observation, plant_state)
            elif callable(getattr(plant,"hybrid_kinematic_context",None)):
                kin=plant.hybrid_kinematic_context(plant_state)
            else:
                raise ValueError("plant has no pure hybrid kinematic context factory")
            ctx=planning_context_override
            force_context=None
            predictor=self.brake_current_predictor or self.hybrid_policy.predictor
            if self.brake_current_predictor is not None and predictor is not self.hybrid_policy.predictor: raise ValueError("adapter and policy predictors must match")
            if self.brake_current_context_factory is not None:
                force_context=self.brake_current_context_factory(plant,observation,None,plant_state)
            elif plant is not None and callable(getattr(plant,"brake_authority_context",None)):
                force_context=plant.brake_authority_context(plant_state,predictor=predictor)
            elif ctx is not None:
                force_context=getattr(ctx,"current_authority_context",ctx)
            result=self.hybrid_policy.decide(observation=observation,pedal_position=0. if pedal_position is None else pedal_position,
                kinematics=kin,soc=float(getattr(force_context,"soc",float('nan'))),brake_context=force_context)
            return DriverCommand(result.throttle,result.brake_force_n,self._steering_command(observation)),result.decision
        if self.envelope_policy is None:
            if any(not np.isfinite(value) for name, value in vars(observation).items()
                   if name != "wheel_speed_margin") or np.isnan(observation.wheel_speed_margin):
                raise ValueError("driver observations must be finite")
            target, _, feedforward = self.plan.sample(observation.distance)
            acceleration = feedforward + self.speed_gain * (target - observation.speed)
            throttle = float(np.clip(acceleration / 6., 0., 1.))
            ungoverned = float(np.clip(-acceleration / 6., 0., 1.)) * self.maximum_brake_force
            governor = float(np.clip(observation.wheel_speed_margin / self.brake_governor_margin, 0., 1.))
            brake = ungoverned * governor
            decision = HistoricalPreviewDecision(float(observation.distance), target, feedforward,
                float(acceleration), throttle, ungoverned, governor, brake)
            return DriverCommand(throttle, brake, self._steering_command(observation)), decision
        from simplified_vehicle_models.solvers.envelope_driver import PedalPolicyContext, PhysicalBrakePlanningContext
        context = None
        configured_contexts=sum(x is not None for x in (self.brake_planning_context,self.brake_planning_context_cache,self.brake_planning_context_factory))
        if self.brake_current_predictor is not None and self.brake_current_context_factory is None and self.hybrid_policy is None:
            raise ValueError("current inverse requires current-context factory")
        if planning_context_override is not None and self.brake_planning_context_factory is not None:
            context = planning_context_override
        if configured_contexts>1:
            raise ValueError("configure one fixed context, cache provider, or setup factory")
        if self.brake_planning_context_factory is not None and not callable(self.brake_planning_context_factory):
            raise TypeError("brake_planning_context_factory must be callable")
        if self.has_pedal_dynamics:
            release = self.throttle_release_rate if self.throttle_release_rate is not None else 1./self.pedal_travel_time
            apply = self.brake_apply_rate if self.brake_apply_rate is not None else 1./self.pedal_travel_time
            throttle_apply = self.throttle_apply_rate if self.throttle_apply_rate is not None else 1./self.pedal_travel_time
            brake_release = self.brake_release_rate if self.brake_release_rate is not None else 1./self.pedal_travel_time
            context = PedalPolicyContext(release, apply, self.pedal_time_constant,
                                         self.maximum_brake_force,self.brake_governor_margin,
                                         throttle_apply,brake_release)
        authority=None
        if self.envelope_policy.settings.mode == "physical-reachable-v1" and self.brake_current_context_factory is not None:
            if plant is None or plant_state is None:
                raise ValueError("current brake context requires plant and state")
            authority=self.brake_current_context_factory(plant,observation,context,plant_state)
            if planning_context_override is not None:
                from dataclasses import replace as dc_replace
                context=dc_replace(planning_context_override,current_authority_context=authority)
            elif self.brake_planning_context is not None:
                from dataclasses import replace as dc_replace
                context=dc_replace(self.brake_planning_context,current_authority_context=authority)
            else:
                raise ValueError("current-state physical inverse requires a forecast context")
        if (self.envelope_policy.settings.mode == "physical-reachable-v1"
                and (context is not None or self.brake_current_context_factory is not None)
                and (self.brake_planning_context is not None or planning_context_override is not None or use_resolved_context)):
            if self.brake_planning_context_factory is not None:
                context = planning_context_override
            elif callable(self.brake_planning_context_cache):
                context = planning_context_override if use_resolved_context else self.brake_planning_context_cache(observation, context, plant_state)
            elif self.brake_current_context_factory is None:
                context = self.brake_planning_context
            if self.brake_current_context_factory is not None:
                from dataclasses import replace as dc_replace
                context=dc_replace(context,current_authority_context=authority)
            elif context is None:
                raise ValueError("current-state inverse requires forecast context wrapper")
            if self.brake_current_predictor is not None:
                from dataclasses import replace as dc_replace
                context=dc_replace(context,current_predictor=self.brake_current_predictor)
        decision = self.envelope_policy.decide(observation,pedal_position,context)
        if context is None:
            from dataclasses import replace
            governor = float(np.clip(observation.wheel_speed_margin / self.brake_governor_margin, 0., 1.))
            decision = replace(decision,
                ungoverned_brake_force=decision.ungoverned_brake_force*self.maximum_brake_force,
                governor_factor=governor,
                governed_brake_force=decision.ungoverned_brake_force*self.maximum_brake_force*governor,
                governor_shortfall=decision.ungoverned_brake_force*self.maximum_brake_force > decision.ungoverned_brake_force*self.maximum_brake_force*governor+1e-12)
        return DriverCommand(decision.requested_throttle,decision.governed_brake_force,
                             self._steering_command(observation)), decision

    def command(self, observation: DriverObservation) -> DriverCommand:
        return self.command_with_context(observation, None)[0]


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
    #: Empirical midpoint times and interpolated plant states; not interval bounds.
    dense_sample_time: np.ndarray | None = None
    dense_sample_state: np.ndarray | None = None


@dataclass(frozen=True)
class AppliedPowertrainDiagnostics:
    """Immutable snapshot of algebraic powertrain output for one evaluation."""

    wheel_motor_torque_nm: tuple[float, ...]

    def to_dict(self):
        return {"wheel_motor_torque_nm": list(self.wheel_motor_torque_nm)}


@dataclass(frozen=True)
class StepSample:
    """One accepted integrator step, as handed to a ``step_sink``.

    ``state`` and ``rates`` hold the plant layout only (the pedal coordinate is
    reported separately). ``command`` is the *realized* command the plant was
    evaluated with, i.e. after the pedal lag when the driver has one. Algebraic
    powertrain diagnostics are copied into an immutable snapshot during that
    evaluation, so later trial/Jacobian calls cannot change this row.
    """

    index: int
    time: float
    state: np.ndarray
    rates: np.ndarray
    command: DriverCommand
    pedal: float | None
    requested_command: DriverCommand | None = None
    mapped_command: DriverCommand | None = None
    policy_diagnostics: object | None = None
    applied_powertrain_diagnostics: object | None = None


def solve_forward(plant: ForwardPlant, driver: PreviewDriver, initial_state,
                  duration=300., wall_seconds=290., maximum_step=0.02,
                  rtol=1.e-6, atol=1.e-8, method="Radau",
                   initial_pedal: float | None = None,
                   maximum_wall_seconds: float = 300.,
                   step_sink=None, *, diagnostics_sink=None, policy_sink=None,
                   stop_speed_mps: float | None = None) -> ForwardResult:
    """Integrate all states with a stiff method and stop at the path end.

    When ``stop_speed_mps`` is set, also terminate at the downward crossing
    of that positive body-speed threshold; this is an operational event, not
    a claim that the vehicle is at rest or that its wheels are stopped.

    ``wall_seconds`` is this call's wall budget and ``maximum_wall_seconds`` is
    the ceiling that budget may not exceed. The ceiling defaults to 300 s so the
    7DoF acceptance limit is preserved; a rung that may run longer (the vertical
    14DoF solver entry point) must raise it explicitly. Both are wall-clock
    seconds of the caller's process. The caller must also apply a
    parent-process timeout to include setup. Completion does not assert physical
    validity or hot-lap periodicity.

    ``step_sink`` is an optional callable that receives one :class:`StepSample`
    per accepted step, in order, from the accepted-state validation loop below.
    ``policy_sink`` is an alternative observer used when ``step_sink`` is absent;
    they are mutually exclusive and passing both raises ``ValueError`` before
    plant work. The validation loop already evaluates the plant at each accepted
    state, so an observer adds no plant evaluation and never sees a rejected trial.
    No observer (the default) leaves every result bit-for-bit unchanged.
    """
    from scipy.integrate import solve_ivp

    if stop_speed_mps is not None:
        if (isinstance(stop_speed_mps, (bool, np.bool_))
                or not isinstance(stop_speed_mps, (int, float, np.integer, np.floating))
                or not np.isfinite(stop_speed_mps) or stop_speed_mps <= 0.):
            raise ValueError("stop_speed_mps must be finite and strictly positive")
        stop_speed_mps = float(stop_speed_mps)
    if step_sink is not None and policy_sink is not None:
        raise ValueError("step_sink and policy_sink are alternative observers; pass only one")
    policy_enabled = getattr(driver, "envelope_policy", None) is not None
    hybrid_enabled = getattr(driver, "hybrid_policy", None) is not None
    historical_enabled = isinstance(driver, PreviewDriver) and not (policy_enabled or hybrid_enabled)
    if not (policy_enabled or hybrid_enabled or historical_enabled):
        raise ValueError("solve_forward requires a configured PreviewDriver")
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
    if stop_speed_mps is not None:
        initial_speed = plant.observe(plant_initial).speed
        if initial_speed <= stop_speed_mps:
            raise ValueError("initial body speed must be above stop_speed_mps")
    if (getattr(driver,"brake_planning_context_factory",None) is not None
            and getattr(driver,"envelope_policy",None) is not None
            and driver.envelope_policy.settings.mode!="physical-reachable-v1"):
        raise ValueError("physical context factory requires physical-reachable-v1 mode")
    pedal_dynamics = bool(getattr(driver, "has_pedal_dynamics", False))
    if initial_pedal is not None and (not pedal_dynamics or not np.isfinite(initial_pedal)
                                      or not -1. <= initial_pedal <= 1.):
        raise ValueError("initial pedal requires enabled pedal dynamics and position in [-1, 1]")
    if pedal_dynamics:
        # The foot starts where the driver wants it, so the run does not open
        # with a pedal transient.
        observation = plant.observe(plant_initial)
        if policy_enabled or hybrid_enabled:
            if driver.brake_planning_context_factory is not None or driver.brake_current_context_factory is not None:
                pedal_context = None
                if driver.has_pedal_dynamics:
                    release=driver.throttle_release_rate if driver.throttle_release_rate is not None else 1./driver.pedal_travel_time
                    apply=driver.brake_apply_rate if driver.brake_apply_rate is not None else 1./driver.pedal_travel_time
                    pedal_context=__import__('simplified_vehicle_models.solvers.envelope_driver',fromlist=['PedalPolicyContext']).PedalPolicyContext(
                        release,apply,driver.pedal_time_constant,driver.maximum_brake_force,driver.brake_governor_margin,
                        driver.throttle_apply_rate or release,driver.brake_release_rate or apply)
                if driver.brake_planning_context_factory is not None:
                    fixed_context=driver.brake_planning_context_factory(plant,observation,pedal_context,plant_initial)
                else:
                    fixed_context=driver.brake_planning_context
                resolved_planning_context=[fixed_context]
            else:
                fixed_context=None
            requested, _ = driver.command_with_context(observation, initial_pedal, plant_state=plant_initial,
                                                        plant=plant,planning_context_override=fixed_context)
        else:
            requested = driver.command(observation)
        initial = np.r_[plant_initial, driver.pedal_target(requested)
                        if initial_pedal is None else initial_pedal]
    else:
        initial = plant_initial
    start = perf_counter()
    evaluations = 0
    rejected_domain_trials = 0
    last_domain_rejection = [None]

    #: Commands from the latest ``state_rates`` call, read only by ``step_sink``.
    applied = [None]
    command_stages = [None]
    policy_stage = [None]
    # Immutable diagnostics captured inside the exact evaluate() call used by
    # the latest RHS. Accepted-state validation consumes this immediately;
    # later Jacobian/trial calls cannot change its provenance.
    evaluated_powertrain = [None]

    def capture_powertrain_snapshot():
        powertrain = getattr(plant, "last_powertrain", None)
        if powertrain is None:
            evaluated_powertrain[0] = None
            return
        torques = tuple(float(value) for value in
                        np.asarray(powertrain.motor_wheel_torques, dtype=float).copy())
        evaluated_powertrain[0] = AppliedPowertrainDiagnostics(torques)

    resolved_planning_context=[fixed_context if pedal_dynamics and (policy_enabled or hybrid_enabled) else getattr(driver,"brake_planning_context",None)]
    if pedal_dynamics and policy_enabled and fixed_context is None and driver.brake_planning_context_cache is not None:
        pedal_context_for_cache=__import__('simplified_vehicle_models.solvers.envelope_driver',fromlist=['PedalPolicyContext']).PedalPolicyContext(
            driver.throttle_release_rate or 1./driver.pedal_travel_time,
            driver.brake_apply_rate or 1./driver.pedal_travel_time,
            driver.pedal_time_constant,driver.maximum_brake_force,driver.brake_governor_margin,
            driver.throttle_apply_rate or 1./driver.pedal_travel_time,
            driver.brake_release_rate or 1./driver.pedal_travel_time)
        resolved_planning_context[0]=driver.brake_planning_context_cache(plant,observation,pedal_context_for_cache,plant_initial)
    def raw_state_rates(time, state):
        plant_state = state[:plant_size]
        observation = plant.observe(plant_state)
        if policy_enabled or hybrid_enabled or historical_enabled:
            requested, decision = driver.command_with_context(
                observation, float(np.clip(state[plant_size], -1., 1.)) if pedal_dynamics else None,
                plant_state=plant_state, plant=plant,
                planning_context_override=resolved_planning_context[0],
                use_resolved_context=resolved_planning_context[0] is not None)
            policy_stage[0] = decision
        else:
            requested = driver.command(observation)
            policy_stage[0] = None
        mapped = requested
        command_stages[0] = (requested, mapped)
        if not pedal_dynamics:
            applied[0] = mapped
            system = plant.evaluate(time, plant_state, mapped)
            capture_powertrain_snapshot()
            return system.solve()
        position = float(state[plant_size])
        realized = driver.realized_command(position, mapped)
        applied[0] = realized
        system = plant.evaluate(time, plant_state, realized)
        capture_powertrain_snapshot()
        rates = system.solve()
        return np.r_[rates, driver.pedal_rate(position, driver.pedal_target(mapped))]

    def state_rates(time, state):
        return raw_state_rates(time, state)

    # Reject an invalid initial state before the integrator can retry trials.
    state_rates(0., initial)

    def rhs(time, state):
        nonlocal evaluations, rejected_domain_trials
        if perf_counter() - start >= wall_seconds:
            raise TimeoutError("forward integration exceeded its wall budget")
        evaluations += 1
        try:
            rates = raw_state_rates(time, state)
        except TrialStateError as error:
            rejected_domain_trials += 1
            last_domain_rejection[0]={"time":float(time),"message":str(error)}
            # Radau rejects this Newton trial and reduces its integration step.
            return np.full(initial.shape, np.nan)
        if rates.shape != initial.shape:
            raise ValueError("plant rates must retain the complete state")
        return rates

    def finish(time, state):
        return plant.observe(state[:plant_size]).distance - driver.plan.distance[-1]

    finish.terminal = True
    finish.direction = 1

    if stop_speed_mps is not None:
        def speed_stop(time, state):
            return plant.observe(state[:plant_size]).speed - stop_speed_mps

        speed_stop.terminal = True
        speed_stop.direction = -1
        integration_events = (finish, speed_stop)
    else:
        integration_events = finish

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
                       rtol=rtol, atol=atol, max_step=maximum_step,
                       events=integration_events, dense_output=True)
    if not result.success:
        accepted_time=float(result.t[-1]) if len(result.t) else 0.
        accepted_state=np.asarray(result.y[:,-1],dtype=float) if result.y.ndim == 2 and result.y.shape[1] else initial
        state_kind="solver_accepted_unvalidated" if len(result.t) else "initial"
        diagnostics={"failure_kind":"solver_unsuccessful","solver_message":str(result.message),
                     "last_state_kind":state_kind,"last_accepted_time":accepted_time,
                     "accepted_step_count":max(0,len(result.t)-1),
                     "rhs_evaluations":evaluations,"rejected_domain_trials":rejected_domain_trials,
                     "last_domain_rejection":last_domain_rejection[0],"duration":float(duration),"wall_seconds":float(wall_seconds),"method":method,
                     "elapsed_wall_seconds":perf_counter()-start}
        raise ForwardIntegrationError(str(result.message),diagnostics,accepted_state,accepted_time)
    integration_done = perf_counter()
    # Bound every component of the dense polynomial over each accepted interval
    # before the replay loops, so a later timeout cannot hide this result.
    dense_bounds = None
    dense_polynomials = None
    timings = {"integration_seconds": integration_done - start, "accepted_state_validation_seconds": 0.0,
               "dense_state_domain_validation_seconds": 0.0, "defect_checking_seconds": 0.0,
               "record_writing_seconds": 0.0, "summary_construction_seconds": 0.0}
    counts = {"accepted_states_validated": 0, "dense_probes_validated": 0,
              "defect_checks": 0, "records_written": 0}
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
    for step_number, (time_value, state) in enumerate(zip(result.t, result.y.T)):
        if perf_counter() - start >= wall_seconds:
            diagnostics = {"failure_kind": "validation_timeout", "last_state_kind": "solver_accepted_unvalidated",
                "last_accepted_time": float(result.t[-1]), "accepted_step_count": max(0, len(result.t)-1),
                "rhs_evaluations": evaluations, "rejected_domain_trials": rejected_domain_trials,
                "last_domain_rejection": last_domain_rejection[0], "duration": float(duration),
                "wall_seconds": float(wall_seconds), "method": method, "elapsed_wall_seconds": perf_counter()-start,
                "validation_timings": dict(timings), "validation_counts": dict(counts)}
            raise ForwardIntegrationError("accepted-state validation exceeded its wall budget", diagnostics,
                                          result.y[:, -1], result.t[-1])
        validation_started = perf_counter()
        accepted_rates = state_rates(time_value, state)
        timings["accepted_state_validation_seconds"] += perf_counter() - validation_started
        counts["accepted_states_validated"] += 1
        sample = StepSample(
            step_number, float(time_value), np.array(state[:plant_size], dtype=float),
            np.array(accepted_rates[:plant_size], dtype=float), applied[0],
            float(np.clip(state[plant_size], -1., 1.)) if pedal_dynamics else None,
            command_stages[0][0], command_stages[0][1], policy_stage[0],
            evaluated_powertrain[0])
        sinks = (step_sink,) if step_sink is not None else (policy_sink,)
        for sink in sinks:
            if sink is not None:
                record_started = perf_counter()
                sink(sample)
                timings["record_writing_seconds"] += perf_counter() - record_started
                counts["records_written"] += 1
    # Endpoints are not enough: reconstruct each interval's interior state
    # from the integrator's dense solution, including its pedal coordinate.
    defect_max = 0.
    samples = 0
    dense_sample_times = []
    dense_sample_states = []
    for left, right in zip(result.t[:-1], result.t[1:]):
        if perf_counter() - start >= wall_seconds:
            diagnostics = {"failure_kind": "validation_timeout", "last_state_kind": "solver_accepted_unvalidated",
                "last_accepted_time": float(result.t[-1]), "accepted_step_count": max(0, len(result.t)-1),
                "rhs_evaluations": evaluations, "rejected_domain_trials": rejected_domain_trials,
                "last_domain_rejection": last_domain_rejection[0], "duration": float(duration),
                "wall_seconds": float(wall_seconds), "method": method, "elapsed_wall_seconds": perf_counter()-start,
                "validation_timings": dict(timings), "validation_counts": dict(counts)}
            raise ForwardIntegrationError("between-step validation exceeded its wall budget", diagnostics,
                                          result.y[:, -1], result.t[-1])
        midpoint = (left + right) / 2.
        dense_state = result.sol(midpoint)
        dense_sample_times.append(float(midpoint))
        dense_sample_states.append(np.array(dense_state[:plant_size], dtype=float, copy=True))
        domain_started = perf_counter()
        rates = state_rates(midpoint, dense_state)
        timings["dense_state_domain_validation_seconds"] += perf_counter() - domain_started
        counts["dense_probes_validated"] += 1
        # The BDF interpolant is a polynomial, so a small central difference
        # recovers its slope to rounding. Compare it with the model slope.
        defect_started = perf_counter()
        width = right - left
        probe = 1.e-3 * width
        slope = (result.sol(midpoint + probe) - result.sol(midpoint - probe)) / (2. * probe)
        scaled = np.abs(slope - rates) * abs(width) / (atol + rtol * np.abs(dense_state))
        defect_max = max(defect_max, float(np.max(scaled)))
        samples += 1
        counts["defect_checks"] += 1
        timings["defect_checking_seconds"] += perf_counter() - defect_started
    elapsed = perf_counter() - start
    if elapsed >= wall_seconds:
        diagnostics = {"failure_kind": "validation_timeout", "last_state_kind": "solver_accepted_unvalidated",
            "last_accepted_time": float(result.t[-1]), "accepted_step_count": max(0, len(result.t)-1),
            "rhs_evaluations": evaluations, "rejected_domain_trials": rejected_domain_trials,
            "last_domain_rejection": last_domain_rejection[0], "duration": float(duration),
            "wall_seconds": float(wall_seconds), "method": method, "elapsed_wall_seconds": elapsed,
            "validation_timings": dict(timings), "validation_counts": dict(counts)}
        raise ForwardIntegrationError("post-integration validation exceeded its wall budget", diagnostics,
                                      result.y[:, -1], result.t[-1])
    pedal = np.clip(result.y[plant_size], -1., 1.) if pedal_dynamics else None
    summary_started = perf_counter()
    lap_completed = bool(result.t_events[0].size)
    forward_result = ForwardResult(result.t, result.y[:plant_size], lap_completed,
                         elapsed, evaluations, result.message, rejected_domain_trials, pedal, True,
                         dense_bounds, dense_polynomials, samples, defect_max,
                          np.asarray(dense_sample_times), np.asarray(dense_sample_states))
    timings["summary_construction_seconds"] = perf_counter() - summary_started
    if diagnostics_sink is not None:
        diagnostics_sink({"timings": dict(timings), "counts": dict(counts),
                          "evaluations": evaluations, "accepted_steps": max(0, len(result.t)-1),
                          "total_wall_seconds": perf_counter()-start})
    return forward_result
