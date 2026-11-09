"""Adapt the vertical 14-DoF vehicle to the shared forward driver.

The plant owns 38 first-order states:

===  ==========================================================
0-2  path coordinates ``s``, ``n`` and the heading error
3-23 the 21 mechanical values of ``VerticalState`` (body velocities
     and Euler rates, heave/roll/pitch, four wheel speeds, four
     jounces, four jounce rates)
24   battery state of charge
25   road-wheel steering angle
26-37 the 12 transient tire states, three per wheel in FL, FR, RL, RR
     order: effective longitudinal slip, lateral carcass
     deformation, and trail angle
===  ==========================================================

A suspension network with its own states (a hydraulic circuit) appends them after
the 38 rows: ``vertical_plant_class(vehicle)`` returns a subclass with
``state_size = 38 + n_states`` and the network's state names, tolerances and
bounds. A stateless network keeps this class, so every earlier layout, trace
header and committed evidence is unchanged.

The mechanical rows come from ``evaluate_vertical_dynamics`` through
``RateSystem(out.forcing, out.mass_matrix)``; the plant never re-solves
``out.rates``. When the compiled inertance is zero the evaluator returns
``mass_matrix is None`` and ``forcing`` already holds the solved rates, so the
plant embeds it directly; when the inertance is positive the 21x21 physical
block is embedded in a 38x38 identity by the state-name index map.

``initial_state`` starts from the steady corner at the planned speed and the
first-node curvature (``solve_steady_corner``). The body sideslip, yaw rate,
steering, roll, heave, pitch, jounces, wheel speeds and tire states are then in
lateral, yaw and vertical balance at ``t = 0``. The driver still owns the wheel
torque, so the longitudinal balance follows the first command. Below the steady-corner
minimum speed (a standing start) the start uses ``solve_static_equilibrium`` and
refines the seven vertical coordinates on the evaluator's generalized forces.
``initial_condition`` records which start was used.
"""
from __future__ import annotations

from dataclasses import fields

import numpy as np

from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState, contact_velocities, slip_quantities, wheel_positions,
    wheel_steering_angles,
)
from simplified_vehicle_models.formulations.vertical_14dof import (
    VerticalState, evaluate_vertical_dynamics, sprung_wheel_positions,
)
from simplified_vehicle_models.powertrain.motor_envelope import (
    INVERTER_SPEED_REGULATION_BAND, motor_speed_limit,
)
from simplified_vehicle_models.solvers.forward_driver import (
    DriverCommand, DriverObservation, RateSystem, SpeedPlan, TrialStateError,
)
from simplified_vehicle_models.solvers.forward_powertrain import (
    ForwardPowertrain, evaluate_forward_powertrain,
)
from simplified_vehicle_models.solvers.vertical_equilibrium import (
    MINIMUM_STEADY_CORNER_SPEED, _solve_steady_corner_detail, solve_static_equilibrium,
)
from simplified_vehicle_models.suspension.errors import DomainError
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.tires.vertical import tire_deflection_load
from simplified_vehicle_models.tires.unitire_transient import (
    TransientTireState, steady_rolling_state,
)

#: Wheel order used by every per-corner array in this module.
WHEELS: tuple[str, ...] = ("fl", "fr", "rl", "rr")

#: Build-time state name prefixes.
MECHANICAL_NAMES: tuple[str, ...] = tuple(field.name for field in fields(VerticalState))
TIRE_STATE_NAMES: tuple[str, ...] = ("kappa_eff", "lateral_deformation", "trail_angle")

#: Plant-layout offsets. The index of a mechanical row is its
#: ``VerticalState`` index plus ``MECHANICAL_OFFSET``.
PATH_NAMES: tuple[str, ...] = ("s", "n", "heading_error")
MECHANICAL_OFFSET = len(PATH_NAMES)
SOC_INDEX = MECHANICAL_OFFSET + len(MECHANICAL_NAMES)
STEERING_INDEX = SOC_INDEX + 1
TIRE_OFFSET = STEERING_INDEX + 1
_CORNER_COUNT = len(WHEELS)
MECHANICAL_INDEX: tuple[int, ...] = tuple(
    range(MECHANICAL_OFFSET, MECHANICAL_OFFSET + len(MECHANICAL_NAMES))
)
WHEEL_SPEED_INDEX: tuple[int, ...] = tuple(
    range(MECHANICAL_OFFSET + 9, MECHANICAL_OFFSET + 13)
)
JOUNCE_INDEX: tuple[int, ...] = tuple(
    range(MECHANICAL_OFFSET + 13, MECHANICAL_OFFSET + 17)
)
TIRE_INDEX: tuple[int, ...] = tuple(
    range(TIRE_OFFSET, TIRE_OFFSET + 3 * _CORNER_COUNT)
)

#: ``VerticalState`` indices of the seven vertical coordinates the plant
#: refines: heave, roll, pitch and the four jounces.
VERTICAL_POSITION_INDEX: tuple[int, ...] = (6, 7, 8, 13, 14, 15, 16)

_CONTACT_SPEED_FLOOR = 1.0
#: First column of the suspension network's own states.
NETWORK_OFFSET = TIRE_OFFSET + 3 * _CORNER_COUNT


def _periodic_tolerance(name: str) -> float:
    """Return the closed-lap periodicity tolerance of one state name.

    Three tiers follow the planar runner's vector, which uses about 1e-3 m for
    the lateral error, 1e-3 rad for angles, and 1e-2 for velocities, refined
    with the plan's Task 9 14DoF values: 0.5 mm for lengths, 0.005 for angles
    and angular rates, 0.01 for linear velocities and wheel speeds, and the
    tire-state tolerances 0.002 (effective kappa), 0.2 mm (deformation) and
    0.002 rad (trail angle).
    """
    if name.startswith("jounce_rate"):
        return 1.0e-2
    if name in ("n", "heave") or name.startswith("jounce_"):
        return 5.0e-4
    if name.startswith("kappa_eff"):
        return 2.0e-3
    if name.startswith("lateral_deformation"):
        return 2.0e-4
    if name.startswith("trail_angle"):
        return 2.0e-3
    if name in ("heading_error", "roll", "pitch", "steering",
                "roll_rate", "pitch_rate", "yaw_rate"):
        return 5.0e-3
    return 1.0e-2


def _periodic_tolerances(names: tuple[str, ...], indices: tuple[int, ...]) -> np.ndarray:
    """Return one tolerance per periodic index, in the same order."""

    return np.array([_periodic_tolerance(names[index]) for index in indices], dtype=float)


def body_accelerations(state: np.ndarray, rates: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return road-plane body-axis accelerations from 38-state rows.

    A plant with network states has more columns; only the first 38 are read.

    This mirrors ``planar_forward.body_accelerations`` for the vertical layout:
    the velocity derivatives alone omit the rotating-frame terms, so a steady
    corner has ``v_y_dot == 0`` yet ``a_y == yaw_rate * v_x``. The road is flat,
    so the body-axis accelerations are the road-plane ones.
    """
    state = np.asarray(state, dtype=float)
    rates = np.asarray(rates, dtype=float)
    if state.shape != rates.shape or state.shape[-1] < 38:
        raise ValueError("vertical states and rates must have matching 38-state layouts")
    return (rates[..., 3] - state[..., 8] * state[..., 4],
            rates[..., 4] + state[..., 8] * state[..., 3])


class VerticalForwardPlant:
    """Own the 38-state vertical 14-DoF plant of the forward driver."""

    state_size = 38
    #: Number of suspension-network states appended after the 38 rows.
    network_states = 0
    state_names: tuple[str, ...] = (
        *PATH_NAMES,
        *MECHANICAL_NAMES,
        "soc",
        "steering",
        *(f"{name}_{wheel}" for wheel in WHEELS for name in TIRE_STATE_NAMES),
    )
    #: Periodic states of a closed lap: everything except the arc length ``s``
    #: (which starts over each lap) and the state of charge (consumed, not
    #: periodic), matching the planar plant's exclusion of both.
    periodic_indices: tuple[int, ...] = tuple(
        index for index in range(state_size) if index not in (0, SOC_INDEX)
    )
    #: One tolerance per ``periodic_indices`` entry, in the same order.
    periodic_tolerances: np.ndarray = _periodic_tolerances(state_names, periodic_indices)

    def __init__(self, vehicle, plan: SpeedPlan):
        self.vehicle = vehicle
        self.plan = plan
        #: The method and values of the last ``initial_state`` call, or ``None``.
        self.initial_condition = None
        #: See ``PlanarForwardPlant``: an open plan may start from rest.
        self.minimum_speed = 0.25 if plan.is_closed else 0.0
        self.minimum_progress = 0.25 if plan.is_closed else 0.0
        self.driven, self.gearing, self.gear_efficiency = _drivetrain_context(vehicle.planar)
        # Jounce limits are design-range metadata, not mechanical guards.
        # Diagnostic counters and extremes, matching ``PlanarForwardPlant`` names.
        # The vertical load path is algebraic (no inner load loop), so the load
        # closure diagnostics stay at their trivial values and are counted under
        # the ``direct`` method.
        self.maximum_load_residual = 0.0
        self.minimum_contact_load = float("inf")
        self.maximum_power_residual = 0.0
        self.maximum_load_iterations = 0
        self.load_method_counts = {"direct": 0}
        self.minimum_load_jacobian_determinant = float("inf")
        self.lifted_wheel_evaluations = 0
        self.speed_regulated_evaluations = 0
        self.maximum_motor_speed_ratio = 0.0
        self.evaluations = 0
        self.last_time = 0.0
        self.last_distance = 0.0
        #: Vertical-specific diagnostics. ``out_of_range_evaluations`` counts
        #: every evaluation that left a compiled table domain: a damper velocity
        #: outside its table (reported by the evaluator's counter) or a spring,
        #: bump-stop, or jounce-steer query outside its spline domain (raised as
        #: ``DomainError`` and converted to a rejected trial below).
        self.out_of_range_evaluations = 0
        self.maximum_jounce = -float("inf")
        self.minimum_jounce = float("inf")
        self.maximum_damper_velocity = 0.0
        #: Range reached by the network's own states over every evaluation, one
        #: entry per state (empty for a stateless network).
        self.network_state_minimum = np.full(self.network_states, np.inf)
        self.network_state_maximum = np.full(self.network_states, -np.inf)
        #: Road-plane body accelerations ``(ax, ay)`` of the last evaluation,
        #: which the driver layer reads from the plant. ``DriverObservation``
        #: has no acceleration fields, so they are exposed here, exactly as the
        #: planar plant exposes them through ``planar_forward.body_accelerations``.
        self.body_accelerations = np.zeros(2)
        #: The ``VerticalEvaluation`` and ``ForwardPowertrain`` of the latest
        #: ``evaluate`` call, for a run recorder (see ``PlanarForwardPlant``).
        self.last_evaluation = None
        self.last_powertrain = None

    # -- plant layout -----------------------------------------------------

    def mechanical_state(self, state) -> VerticalState:
        """Return the 21-value mechanical state of the 38-state plant vector."""

        values = np.asarray(state, dtype=float)
        return VerticalState.from_vector(values[list(MECHANICAL_INDEX)])

    def network_state(self, state):
        """Return the suspension-network states of the plant vector, or None."""

        if self.network_states == 0:
            return None
        values = np.asarray(state, dtype=float)
        return values[NETWORK_OFFSET:NETWORK_OFFSET + self.network_states]

    def tire_states(self, state) -> tuple[TransientTireState, ...]:
        """Return the four transient tire states of the plant vector."""

        values = np.asarray(state, dtype=float)
        return tuple(
            TransientTireState(*(float(value) for value in values[
                TIRE_OFFSET + 3 * wheel: TIRE_OFFSET + 3 * wheel + 3]))
            for wheel in range(_CORNER_COUNT)
        )

    def initial_state(self, speed: float) -> np.ndarray:
        """Return the steady (planned-curvature) initial state.

        At or above ``MINIMUM_STEADY_CORNER_SPEED`` the start is the steady
        corner at ``speed`` and ``ay = speed**2 * curvature(0)``. Sideslip, yaw
        rate, steering, body attitude, jounces, wheel speeds and the 12 tire
        states come from that solve, so the lateral, yaw and vertical rates are
        zero at ``t = 0``. If no steady corner exists at the first node, this
        raises ``ValueError``. It does not fall back to an unbalanced start.

        Below that speed (a standing start) the seven vertical coordinates are
        refined on the evaluator's generalized forces at zero wheel torque, and
        the tire states are set to their steady rolling values.
        """
        if (np.isfinite(speed) and speed >= MINIMUM_STEADY_CORNER_SPEED):
            return self._steady_corner_initial_state(float(speed))
        return self._static_initial_state(speed)

    def _steady_corner_initial_state(self, speed: float) -> np.ndarray:
        """Return the initial state from the steady corner at the first node."""
        vehicle = self.vehicle
        planar = vehicle.planar
        _, curvature, _ = self.plan.sample(0.0)
        lateral_acceleration = speed * speed * float(curvature)
        try:
            solution = _solve_steady_corner_detail(vehicle, speed, lateral_acceleration)
        except RuntimeError as error:
            raise ValueError(
                f"no steady corner at the first node (speed {speed:.6g} m/s, "
                f"curvature {float(curvature):.6g} 1/m, ay {lateral_acceleration:.6g} m/s^2): "
                f"{error}"
            ) from error
        steering = float(solution.steering)
        if abs(steering) > planar.steering_maximum_angle:
            raise ValueError("initial curvature exceeds steering authority")
        q = solution.state.as_vector()
        self.initial_condition = {
            "method": "steady-corner",
            "speed_mps": speed,
            "curvature_1pm": float(curvature),
            "lateral_acceleration_mps2": lateral_acceleration,
            "steering_rad": steering,
            "steady_driven_wheel_torque_nm": float(solution.driven_torque),
            "residual_inf": float(solution.residual_inf),
            "continuation_steps": int(solution.continuation_steps),
        }
        return self._assemble_state(q, steering, solution.tire_states)

    def _assemble_state(self, q: np.ndarray, steering: float, tires) -> np.ndarray:
        """Place the mechanical, tire, network, SOC and path values in the plant layout."""
        vehicle = self.vehicle
        state = np.zeros(self.state_size)
        if self.network_states:
            state[NETWORK_OFFSET:] = vehicle.suspension.network.initial_state(q[13:17])
        state[2] = -np.arctan2(q[1], q[0])
        state[list(MECHANICAL_INDEX)] = q
        state[SOC_INDEX] = vehicle.planar.storage.initial_state_of_charge
        state[STEERING_INDEX] = steering
        for wheel, tire in enumerate(tires):
            state[TIRE_OFFSET + 3 * wheel:TIRE_OFFSET + 3 * wheel + 3] = (
                tire.longitudinal_slip, tire.lateral_deformation, tire.trail_angle
            )
        return state

    def _static_initial_state(self, speed: float) -> np.ndarray:
        """Return the straight static-equilibrium start used below the corner-solve speed."""
        if self.plan.is_closed:
            if not np.isfinite(speed) or speed <= 1.0:
                raise ValueError("initial forward speed must exceed 1 m/s")
        elif not np.isfinite(speed) or speed < 0.0:
            raise ValueError("initial forward speed must be finite and nonnegative")
        vehicle = self.vehicle
        planar = vehicle.planar
        _, curvature, _ = self.plan.sample(0.0)
        steering = float(np.arctan(planar.wheelbase * curvature))
        if abs(steering) > planar.steering_maximum_angle:
            raise ValueError("initial curvature exceeds steering authority")

        static = solve_static_equilibrium(vehicle, speed)
        q = static.as_vector()
        q[0] = float(speed)
        q[5] = float(speed) * curvature
        # Align the rear contact velocity before the dynamic tire forces develop.
        q[1] = planar.front_mass_fraction * planar.wheelbase * q[5]
        q[9:13] = float(speed) / planar.tire.r_1
        wheel_torques = np.zeros(_CORNER_COUNT)
        tires = (TransientTireState(0.0, 0.0, 0.0),) * _CORNER_COUNT
        for _ in range(12):
            previous = q.copy()
            state = self._refine_vertical(
                VerticalState.from_vector(q), tires, wheel_torques, steering
            )
            q = state.as_vector()
            evaluation = evaluate_vertical_dynamics(
                vehicle, state, wheel_torques, steering, tires
            )
            loads = np.asarray(evaluation.normal_loads, dtype=float)
            radii = loaded_radius(planar.tire, loads)
            angles = wheel_steering_angles(steering, _CORNER_COUNT) + (
                vehicle.suspension.kinematics.toe(q[13:17])
            )
            longitudinal, lateral = contact_velocities(
                PlanarState(q[0], q[1], q[5], q[9:13]), wheel_positions(planar), angles
            )
            q[9:13] = longitudinal / radii
            slip_ratio, slip_angle = slip_quantities(
                q[9:13], radii, longitudinal, lateral, _CONTACT_SPEED_FLOOR
            )
            tires = tuple(
                steady_rolling_state(planar.tire, vehicle.transient_tire,
                                     float(slip_angle[wheel]), float(slip_ratio[wheel]),
                                     float(loads[wheel]))
                for wheel in range(_CORNER_COUNT)
            )
            if float(np.max(np.abs(q - previous))) < 1e-12:
                break

        self.initial_condition = {
            "method": "static-refined",
            "speed_mps": float(speed),
            "curvature_1pm": float(curvature),
            "steering_rad": steering,
        }
        return self._assemble_state(q, steering, tires)

    def _refine_vertical(self, state: VerticalState, tires, wheel_torques, steering: float,
                         iterations: int = 40) -> VerticalState:
        """Return the vertical coordinates that zero the generalized forces.

        A small private NumPy Newton solve on the seven vertical coordinates
        (heave, roll, pitch and the four jounces) of the evaluator's
        ``generalized_forces``. The mass block is invertible, so zero
        generalized force is zero vertical acceleration. Backtracking keeps the
        trial inside the spring and damper domains.
        """
        vehicle = self.vehicle
        q = state.as_vector()
        position = list(VERTICAL_POSITION_INDEX)

        def residual(x: np.ndarray) -> np.ndarray:
            trial = q.copy()
            trial[position] = x
            return np.asarray(
                evaluate_vertical_dynamics(
                    vehicle, VerticalState.from_vector(trial), wheel_torques, steering, tires
                ).generalized_forces,
                dtype=float,
            )

        x = q[position].copy()
        for _ in range(iterations):
            r = residual(x)
            if not np.all(np.isfinite(r)) or float(np.max(np.abs(r))) < 1e-11:
                break
            jacobian = np.empty((7, 7))
            for column in range(7):
                step = 1e-7
                plus, minus = x.copy(), x.copy()
                plus[column] += step
                minus[column] -= step
                jacobian[:, column] = (residual(plus) - residual(minus)) / (2.0 * step)
            delta, *_ = np.linalg.lstsq(jacobian, -r, rcond=None)
            norm = float(np.linalg.norm(r))
            scale = 1.0
            accepted = False
            while scale > 1e-6:
                try:
                    trial = residual(x + scale * delta)
                except (ValueError, FloatingPointError, OverflowError, ZeroDivisionError):
                    trial_norm = np.inf
                else:
                    trial_norm = (float(np.linalg.norm(trial))
                                  if np.all(np.isfinite(trial)) else np.inf)
                if trial_norm < norm:
                    accepted = True
                    break
                scale *= 0.5
            if not accepted:
                break
            x = x + scale * delta
        refined = q.copy()
        refined[position] = x
        return VerticalState.from_vector(refined)

    # -- driver interface -------------------------------------------------

    def hybrid_kinematic_context(self, state):
        """Pure path-aligned quasi-steady context; vy_dot=0 is explicit assumption."""
        from simplified_vehicle_models.solvers.longitudinal_capability import PathKinematicContext
        values=np.asarray(state,dtype=float); obs=self.observe(values)
        _,curv,_=self.plan.sample(obs.distance)
        gradient=float(np.interp(obs.distance,self.plan.distance,np.gradient(self.plan.curvature,self.plan.distance)))
        valid=(abs(float(values[2]))<.05 and abs(float(values[1]))<.25 and abs(float(values[4]))<.5
               and abs(float(curv))<.08 and abs(gradient)<.02)
        return PathKinematicContext(float(values[3]),float(values[4]),float(values[2]),float(values[1]),
            float(values[8]),float(curv),gradient,0.,valid)

    def brake_authority_context(self, state, *, predictor=None):
        """Build an immutable authority context from this exact plant state."""
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
        values=np.asarray(state,dtype=float)
        observation=self.observe(values)
        if predictor is None:
            predictor=getattr(self,"brake_authority_predictor",None)
        if predictor is None:
            raise ValueError("a prebuilt BrakeAuthorityPredictor is required")
        mechanical=self.mechanical_state(values)
        jounce=values[list(JOUNCE_INDEX)]
        positions=sprung_wheel_positions(self.vehicle)
        z_body=mechanical.heave+mechanical.roll*positions[:,1]-mechanical.pitch*positions[:,0]
        lift,_,_=self.vehicle.suspension.kinematics.steering_lift(
            wheel_steering_angles(float(values[STEERING_INDEX]),_CORNER_COUNT))
        loads=np.maximum(np.asarray(tire_deflection_load(
            self.vehicle.planar.tire,z_body-jounce-lift,np.zeros(_CORNER_COUNT),0.).load),1.)
        radii=loaded_radius(self.vehicle.planar.tire,loads)
        _,curvature,_=self.plan.sample(observation.distance)
        return BrakeAuthorityContext(float(observation.speed),float(curvature),
            tuple(float(x) for x in radii),tuple(float(x) for x in loads),
            soc=float(np.clip(values[SOC_INDEX],0.,1.)),regen_enabled=False,
            v_y_mps=float(observation.lateral_speed),yaw_rate_rps=float(observation.yaw_rate),
            steering_rad=float(observation.steering))

    def brake_planning_context(self, state, pedal_context, *, predictor=None, knot_count=8):
        """Construct explicit state-valid authority context, never using last RHS data.

        Vertical load estimates use the current mechanical jounce/attitude state;
        per-wheel capacities are conservatively omitted, so callers must treat
        the resulting fixed-bias forecast as a quasi-steady lower-confidence
        estimate, not combined-slip certification.
        """
        from simplified_vehicle_models.solvers.brake_authority import BrakeAuthorityContext
        from simplified_vehicle_models.solvers.envelope_driver import PhysicalBrakePlanningContext
        values=np.asarray(state,dtype=float)
        observation=self.observe(values)
        if predictor is None:
            predictor=getattr(self,"brake_authority_predictor",None)
        if predictor is None:
            raise ValueError("a prebuilt BrakeAuthorityPredictor is required")
        mechanical=self.mechanical_state(values)
        jounce=values[list(JOUNCE_INDEX)]
        positions=sprung_wheel_positions(self.vehicle)
        z_body=mechanical.heave+mechanical.roll*positions[:,1]-mechanical.pitch*positions[:,0]
        lift,_,_=self.vehicle.suspension.kinematics.steering_lift(
            wheel_steering_angles(float(values[STEERING_INDEX]),_CORNER_COUNT))
        loads=np.maximum(np.asarray(tire_deflection_load(
            self.vehicle.planar.tire,z_body-jounce-lift,np.zeros(_CORNER_COUNT),0.).load),1.)
        radii=loaded_radius(self.vehicle.planar.tire,loads)
        _,curvature,_=self.plan.sample(observation.distance)
        authority=BrakeAuthorityContext(float(observation.speed),
            float(curvature),tuple(float(x) for x in radii),tuple(float(x) for x in loads),
            soc=float(np.clip(values[SOC_INDEX],0.,1.)),regen_enabled=False,
            v_y_mps=float(observation.lateral_speed),yaw_rate_rps=float(observation.yaw_rate),
            steering_rad=float(observation.steering))
        context=PhysicalBrakePlanningContext.from_predictor(predictor,authority,pedal_context,knot_count=knot_count)
        from dataclasses import replace
        return replace(context,current_authority_context=authority,current_predictor=predictor)

    def observe(self, state) -> DriverObservation:
        """Return the driver observation from this state.

        The road-plane body accelerations of the last evaluation stay on
        ``self.body_accelerations``; ``DriverObservation`` itself carries no
        acceleration fields. The wheel-speed margin is the most-locked wheel's
        kinematic slip ratio plus one, exactly as the planar plant computes it.
        It uses wheel speed against contact velocity at the static loaded
        radius, so the brake governor sees wheel lock when it happens. The
        transient tire's slip state lags the wheel by the relaxation length,
        so using it made the governor release late.
        """
        values = np.asarray(state, dtype=float)
        mechanical = self.mechanical_state(values)
        planar = self.vehicle.planar
        jounce = values[list(JOUNCE_INDEX)]
        angles = (wheel_steering_angles(float(values[STEERING_INDEX]), _CORNER_COUNT)
                  + self.vehicle.suspension.kinematics.toe(jounce))
        longitudinal, lateral = contact_velocities(
            PlanarState(mechanical.v_x, mechanical.v_y, mechanical.yaw_rate,
                        values[list(WHEEL_SPEED_INDEX)]),
            wheel_positions(planar), angles)
        # The loaded radius follows each tire's current load (from its own
        # deflection), as in the planar plant; a single nominal radius was off by
        # several percent of slip under load transfer.
        positions = sprung_wheel_positions(self.vehicle)
        z_body = (mechanical.heave + mechanical.roll * positions[:, 1]
                  - mechanical.pitch * positions[:, 0])
        lift, _, _ = self.vehicle.suspension.kinematics.steering_lift(
            wheel_steering_angles(float(values[STEERING_INDEX]), _CORNER_COUNT))
        loads = np.maximum(np.asarray(tire_deflection_load(
            planar.tire, z_body - jounce - lift, np.zeros(_CORNER_COUNT), 0.0).load), 0.0)
        slip_ratio, _ = slip_quantities(values[list(WHEEL_SPEED_INDEX)],
                                        loaded_radius(planar.tire, loads),
                                        longitudinal, lateral, 1.0)
        wheel_speed_margin = float(np.min(slip_ratio)) + 1.0
        return DriverObservation(float(values[0]), float(values[1]), float(values[2]),
                                 float(values[3]), float(values[8]),
                                 float(values[STEERING_INDEX]), float(values[4]),
                                 wheel_speed_margin)

    def powertrain(self, state, command: DriverCommand) -> ForwardPowertrain:
        """Return the shared powertrain closure at this state and command."""

        values = np.asarray(state, dtype=float)
        return evaluate_forward_powertrain(self.vehicle.planar, self.driven, self.gearing,
                                           self.gear_efficiency,
                                           values[list(WHEEL_SPEED_INDEX)],
                                           values[SOC_INDEX], command)

    def evaluate(self, time, state, command) -> RateSystem:
        """Return the 38-state implicit rate system of one evaluation."""
        values = np.asarray(state, dtype=float)
        if values.shape != (self.state_size,) or not np.all(np.isfinite(values)):
            raise ValueError("invalid vertical forward state")
        self.last_time = float(time)
        self.last_distance = float(values[0])
        if not 0.0 <= values[SOC_INDEX] <= 1.0:
            raise TrialStateError(
                f"trial state of charge outside bounds: {values[SOC_INDEX]:.6g}"
            )
        if values[3] < self.minimum_speed or (
                self.plan.is_closed and values[3] == self.minimum_speed) or np.any(
                values[list(WHEEL_SPEED_INDEX)] < 0.0):
            raise TrialStateError("prototype requires forward body and wheel motion")
        if abs(values[STEERING_INDEX]) > self.vehicle.planar.steering_maximum_angle + 1.0e-7:
            raise TrialStateError("steering angle exceeds the physical bound")
        # State finiteness is checked above; finite suspension displacement is unlimited.
        point = self.powertrain(values, command)
        mechanical = self.mechanical_state(values)
        tires = self.tire_states(values)
        # The driver supplies steering rate but not acceleration. The omitted
        # unsprung inertial term m_u*h'*steer_accel is below 1 N for the supported
        # lift magnitudes (h' <~ 5 mm, steer_accel ~ 10 rad/s^2, m_u ~ 13 kg).
        try:
            evaluation = evaluate_vertical_dynamics(
                self.vehicle, mechanical, point.wheel_torques,
                float(values[STEERING_INDEX]), tires, self.network_state(values),
                steering_rate=float(command.steering_rate),
            )
        except DomainError as error:
            # A trial outside a spring, bump-stop, or jounce-steer table is the
            # same kind of state-domain rejection as the travel guard above; the
            # integrator must be able to retry with a smaller step instead of
            # failing hard. Count it in ``out_of_range_evaluations`` so the
            # table-domain gate sees spring and jounce-steer exits, not only
            # damper-velocity exits.
            self.out_of_range_evaluations += 1
            raise TrialStateError(f"trial left a suspension table domain: {error}") from error
        except np.linalg.LinAlgError as error:
            # A singular vertical mass block at a trial state is a rejected step.
            raise TrialStateError(f"trial vertical mass block is singular: {error}") from error
        if not (np.all(np.isfinite(evaluation.forcing))
                and np.all(np.isfinite(evaluation.tire_state_rates))):
            # A finite trial state can still overflow the tire or contact law.
            # The integrator must retry with a smaller step, not stop.
            raise TrialStateError("trial produced nonfinite plant rates")
        _, curvature, _ = self.plan.sample(values[0])
        denominator = 1.0 - curvature * values[1]
        if denominator < 0.5:
            # A trial state off the path-coordinate domain is a step the
            # integrator must retry smaller, not a failure of the run.
            raise TrialStateError("path coordinate denominator is invalid")
        progress = (values[3] * np.cos(values[2]) - values[4] * np.sin(values[2])) / denominator
        if progress < self.minimum_progress or (
                self.plan.is_closed and progress == self.minimum_progress):
            raise TrialStateError("positive path progress constraint failed")

        forcing = np.zeros(self.state_size)
        forcing[0] = progress
        forcing[1] = values[3] * np.sin(values[2]) + values[4] * np.cos(values[2])
        forcing[2] = values[8] - curvature * progress
        forcing[list(MECHANICAL_INDEX)] = evaluation.forcing
        forcing[SOC_INDEX] = -point.current / self.vehicle.planar.storage.pack_capacity
        forcing[STEERING_INDEX] = command.steering_rate
        forcing[list(TIRE_INDEX)] = evaluation.tire_state_rates
        if self.network_states:
            forcing[NETWORK_OFFSET:] = evaluation.corner_forces.ds_dt
            self.network_state_minimum = np.minimum(self.network_state_minimum,
                                                    self.network_state(values))
            self.network_state_maximum = np.maximum(self.network_state_maximum,
                                                    self.network_state(values))
        mass_matrix = None
        if evaluation.mass_matrix is not None:
            mass_matrix = np.eye(self.state_size)
            block = list(MECHANICAL_INDEX)
            mass_matrix[np.ix_(block, block)] = evaluation.mass_matrix

        self.maximum_power_residual = max(self.maximum_power_residual, point.power_residual)
        self.load_method_counts["direct"] += 1
        self.minimum_contact_load = min(self.minimum_contact_load,
                                        float(np.min(evaluation.normal_loads)))
        jounce = values[list(JOUNCE_INDEX)]
        self.maximum_jounce = max(self.maximum_jounce, float(np.max(jounce)))
        self.minimum_jounce = min(self.minimum_jounce, float(np.min(jounce)))
        damper_velocity = np.asarray(evaluation.corner_forces.damper_velocity, dtype=float)
        self.maximum_damper_velocity = max(self.maximum_damper_velocity,
                                           float(np.max(np.abs(damper_velocity))))
        if bool(np.any(evaluation.lifted)):
            self.lifted_wheel_evaluations += 1
        if int(evaluation.out_of_range_count) > 0:
            self.out_of_range_evaluations += 1
        motor = self.vehicle.planar.motor
        speed_ratio = float(
            np.max(motor.gear_ratio * values[list(WHEEL_SPEED_INDEX)][self.driven])
            / motor_speed_limit(motor, point.voltage)
        )
        if speed_ratio > 1.0 - INVERTER_SPEED_REGULATION_BAND:
            self.speed_regulated_evaluations += 1
        self.maximum_motor_speed_ratio = max(self.maximum_motor_speed_ratio, speed_ratio)
        self.body_accelerations = np.array(evaluation.body_accelerations, dtype=float, copy=True)
        self.evaluations += 1
        self.last_evaluation = evaluation
        self.last_powertrain = point
        return RateSystem(forcing, mass_matrix)


def vertical_plant_class(vehicle) -> type:
    """Return the plant class whose state layout matches the vehicle's suspension.

    A stateless network (every layout but the hydraulic one) returns
    ``VerticalForwardPlant`` itself. A stateful network returns a subclass that
    appends the network's states: names, periodic indices and tolerances are
    class attributes, as the runner, the gates and the reports read them there.
    """

    network = vehicle.suspension.network
    count = int(network.n_states)
    if count == 0:
        return VerticalForwardPlant
    names = (*VerticalForwardPlant.state_names, *network.state_names)
    if len(names) != VerticalForwardPlant.state_size + count:
        raise ValueError("the network must name each of its states")
    size = len(names)
    indices = tuple(index for index in range(size) if index not in (0, SOC_INDEX))
    tolerances = np.concatenate((
        _periodic_tolerances(VerticalForwardPlant.state_names,
                             VerticalForwardPlant.periodic_indices),
        np.asarray(network.state_tolerances, dtype=float)))
    return type("NetworkVerticalForwardPlant", (VerticalForwardPlant,), {
        "state_size": size, "network_states": count, "state_names": names,
        "periodic_indices": indices, "periodic_tolerances": tolerances,
    })


def _drivetrain_context(planar):
    """Return ``(driven, gearing, gear_efficiency)`` of the compiled planar vehicle.

    Mirrors ``PlanarForwardPlant.__init__``: ``gearing`` is the motor gear ratio
    already multiplied by the gear efficiency, exactly as the shared powertrain
    closure expects.
    """
    transmission = planar.drivetrain.drivetrain.transmission
    wheel_index = {name: index for index, name in enumerate(WHEELS)}
    driven = np.array([wheel_index[name] for name in transmission.driven_wheels])
    if len(driven) != planar.motor.unit_count:
        raise ValueError("motor count must match driven wheels")
    gear_efficiency = transmission.gear_efficiency
    return driven, planar.motor.gear_ratio * gear_efficiency, gear_efficiency


__all__ = ["VerticalForwardPlant", "body_accelerations", "vertical_plant_class"]
