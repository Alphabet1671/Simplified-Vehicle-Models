"""Task 7 focus tests: the vertical 14-DoF forward plant.

Oracles and where they come from:

* **Protocol / layout.** ``ForwardPlant`` is a static-only ``Protocol`` (it is
  not ``@runtime_checkable``), so ``isinstance(plant, ForwardPlant)`` raises for
  the planar plant too. Conformance is checked structurally and by driving the
  plant through ``solve_forward``, exactly as the planar plant is checked.
* **Initial state.** The ledger ruling: ``solve_static_equilibrium`` carries no
  longitudinal force and is not stationary at speed, so the plant refines the
  seven vertical coordinates on the evaluator's generalized forces.
* **Step test.** The plan's 4.42 Hz front ride frequency and 0.49 damping ratio
  are body-fixed quarter-car references; the ledger records that the
  full-vehicle pitch mode is 7.3 Hz. A 2 000 N front-axle step excites heave and
  pitch together, so the valid oracle is the seven-coordinate linear model built
  from the closed-form mass, stiffness, and damping assemblies (the same
  closed-form construction the Task 6 acceptance gates validate against the
  evaluator's finite-difference linearization). The quarter-car reference is
  reported for context, not used as the gate.
* **Refactor gate.** Rate and powertrain outputs are compared bitwise with a
  fixture recorded from the pre-move code.
"""
import math
import time
import unittest
from dataclasses import dataclass, fields, replace
from pathlib import Path
from unittest.mock import patch

import numpy as np
import scipy.integrate
from scipy.integrate import solve_ivp

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.vertical_14dof import (
    VerticalState, evaluate_vertical_dynamics, sprung_wheel_positions,
    unsprung_masses,
)
from simplified_vehicle_models.solvers.forward_driver import (
    DriverCommand, ForwardPlant, RateSystem, SpeedPlan, TrialStateError, solve_forward,
)
from simplified_vehicle_models.solvers.forward_powertrain import evaluate_forward_powertrain
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.solvers.vertical_forward import (
    MECHANICAL_INDEX, SOC_INDEX, STEERING_INDEX, TIRE_INDEX, TIRE_OFFSET, WHEELS,
    VerticalForwardPlant, body_accelerations,
)
from simplified_vehicle_models.tires.unitire_transient import (
    relaxation_lengths, steady_rolling_state,
)
from simplified_vehicle_models.tires.vertical import tire_deflection_load

ROOT = Path(__file__).resolve().parents[1]
FORMULATION_XML = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"
PLANAR_FORMULATION_XML = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
POWERTRAIN_FIXTURE = ROOT / "tests/fixtures/planar_forward_powertrain_states.npz"

#: ``VerticalState`` rows of the seven vertical coordinates and their rates.
POSITION_ROWS = (6, 7, 8, 13, 14, 15, 16)
RATE_ROWS = (2, 3, 4, 17, 18, 19, 20)
#: The same rows in the 38-state plant layout.
PLANT_RATE_ROWS = tuple(3 + row for row in RATE_ROWS)
PLANT_POSITION_ROWS = tuple(3 + row for row in POSITION_ROWS)

#: Constant-throttle trial settings.
TRIAL_SPEED = 20.0
TRIAL_DISTANCE = 100.0
TRIAL_THROTTLE = 0.10
TRIAL_MAXIMUM_STEP = 0.005
JOUNCE_LIMIT = 0.0318

#: Step-test settings: a 2 000 N downward step on the front axle's sprung-body
#: points (1 000 N per corner).
STEP_FORCE = 2000.0
STEP_DURATION = 1.2
STEP_INERTANCE = 0.5

_VEHICLE_CACHE: dict = {}


def compiled_vehicle():
    """Return the compiled IEM24 14-DoF vehicle, cached across the module."""

    if "vehicle" not in _VEHICLE_CACHE:
        _VEHICLE_CACHE["vehicle"] = compile_formulation(FORMULATION_XML).model
    return _VEHICLE_CACHE["vehicle"]


def without_bump_steer(vehicle):
    """Return ``vehicle`` with flat (zero) jounce-steer curves, keeping everything else.

    These tests check identities that hold for any bump steer only when the toe
    change is zero (exact static rates, oracle force balances), so they isolate the
    physics under test from the derived curves, which have their own tests.
    """

    from dataclasses import replace
    from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline

    kinematics = vehicle.suspension.kinematics
    for axle in (0, 1):
        low, high = kinematics.axle(axle).jounce_limits
        kinematics = kinematics.with_axle(axle, jounce_steer=MonotoneSpline([low, 0.0, high], [0.0, 0.0, 0.0]))
    suspension = replace(vehicle.suspension, kinematics=kinematics,
                         network=vehicle.suspension.network.with_kinematics(kinematics))
    return replace(vehicle, suspension=suspension)


def flat_vehicle():
    """Return the compiled IEM24 vehicle with flat bump steer, cached across the module."""

    if "flat" not in _VEHICLE_CACHE:
        _VEHICLE_CACHE["flat"] = without_bump_steer(compiled_vehicle())
    return _VEHICLE_CACHE["flat"]


def straight_plan(speed: float = TRIAL_SPEED, distance: float = TRIAL_DISTANCE) -> SpeedPlan:
    """Return a straight closed plan at a constant requested speed."""

    return SpeedPlan(np.array([0.0, distance]), np.full(2, float(speed)), np.zeros(2))


@dataclass(frozen=True)
class ConstantThrottleDriver:
    """A minimal ``ForwardPlant`` driver stub: constant throttle, no braking."""

    plan: SpeedPlan
    throttle: float

    def command(self, observation):
        return DriverCommand(self.throttle, 0.0, 0.0)


def with_inertance(vehicle, inertance: float = STEP_INERTANCE):
    """Return the vehicle with a hub-to-chassis inerter on both axles.

    The inerter makes the evaluator expose its physical 21x21 mass block, which
    is what lets the plant embed a 38x38 mass matrix (and what lets a test
    superpose an external generalized force on the implicit system).
    """

    suspension = vehicle.suspension
    return replace(
        vehicle,
        suspension=suspension.with_network(suspension.network.with_inertance(inertance, inertance)),
    )


def closed_form_mass(vehicle) -> np.ndarray:
    """Independent 7x7 physical mass block, including the inerter diagonal."""

    positions = sprung_wheel_positions(vehicle)
    masses = unsprung_masses(vehicle)
    matrix = np.zeros((7, 7))
    matrix[0, 0] = vehicle.sprung_mass
    matrix[1, 1] = vehicle.roll_inertia
    matrix[2, 2] = vehicle.pitch_inertia
    for index in range(4):
        column = np.zeros(7)
        column[:3] = (1.0, positions[index, 1], -positions[index, 0])
        column[3 + index] = -1.0
        matrix += masses[index] * np.outer(column, column)
    for index in range(4):
        inertance = vehicle.suspension.network.inertance[0 if index < 2 else 1]
        if inertance > 0.0:
            ratio = vehicle.suspension.kinematics.motion_ratio(index, 0.0)
            matrix[3 + index, 3 + index] += inertance / ratio**2
    return matrix


def closed_form_stiffness(vehicle, x0) -> np.ndarray:
    """Independent 7x7 stiffness from the compiled spring, ARB, and tire data.

    ``K[0]`` is the tire-only heave stiffness because the suspension spring acts
    between body and hub and therefore only on the jounce coordinates in this
    coordinate choice (see the Task 6 acceptance gates).
    """

    positions = sprung_wheel_positions(vehicle)
    x0 = np.asarray(x0, dtype=float)
    body = x0[0] + positions[:, 1] * x0[1] - positions[:, 0] * x0[2]
    deflection = body - x0[3:]
    tire = np.asarray(
        tire_deflection_load(vehicle.planar.tire, deflection, np.zeros(4), 0.0).stiffness,
        dtype=float,
    )
    spring = np.empty(4)
    network = vehicle.suspension.network
    for axle, (left, right) in ((0, (0, 1)), (1, (2, 3))):
        spring[[left, right]] = network.springs.axle(axle).derivative(x0[3:][[left, right]])
    matrix = np.zeros((7, 7))
    for index in range(4):
        column = np.zeros(7)
        column[0] = 1.0
        column[1] = positions[index, 1]
        column[2] = -positions[index, 0]
        column[3 + index] = -1.0
        matrix += tire[index] * np.outer(column, column)
        matrix[3 + index, 3 + index] += spring[index]
    for axle, (left, right) in ((0, (0, 1)), (1, (2, 3))):
        half = 0.5 * network.arb.axle(axle)
        matrix[3 + left, 3 + left] += half
        matrix[3 + left, 3 + right] -= half
        matrix[3 + right, 3 + left] -= half
        matrix[3 + right, 3 + right] += half
    return matrix


def closed_form_damping(vehicle) -> np.ndarray:
    """Independent 7x7 damping from the tire vertical damping and damper tables.

    The tire damper acts between the hub and the ground, so it has the tire
    stiffness structure; the suspension damper acts between body and hub, so it
    contributes only to the jounce diagonal, with ``slope / MR^2`` from the
    damper table's own force derivative at zero velocity.
    """

    positions = sprung_wheel_positions(vehicle)
    matrix = np.zeros((7, 7))
    for index in range(4):
        column = np.zeros(7)
        column[0] = 1.0
        column[1] = positions[index, 1]
        column[2] = -positions[index, 0]
        column[3 + index] = -1.0
        matrix += vehicle.tire_vertical_damping * np.outer(column, column)
    for index in range(4):
        damper = vehicle.suspension.network.dampers[0 if index < 2 else 1]
        ratio = vehicle.suspension.kinematics.motion_ratio(index, 0.0)
        slope = float(np.asarray(damper.force(np.zeros(1)).slope)[0])
        matrix[3 + index, 3 + index] += slope / ratio**2
    return matrix


def first_overshoot(times, response):
    """Return ``(first local maximum, its time)`` of a step response."""

    for index in range(1, len(times) - 1):
        if response[index] >= response[index - 1] and response[index] > response[index + 1]:
            return float(response[index]), float(times[index])
    peak = int(np.argmax(response))
    return float(response[peak]), float(times[peak])


class VerticalForwardLayoutTests(unittest.TestCase):
    """The plant satisfies the driver's protocol and documents its layout."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compiled_vehicle()
        cls.plan = straight_plan()
        cls.plant = VerticalForwardPlant(cls.vehicle, cls.plan)

    def test_lift_configuration_uses_steer_rate_for_hub_velocity(self):
        from dataclasses import replace
        kin = self.vehicle.suspension.kinematics.with_axle(0, steering_lift_sine_left=.002)
        suspension = replace(self.vehicle.suspension, kinematics=kin,
                             network=self.vehicle.suspension.network.with_kinematics(kin))
        plant = VerticalForwardPlant(replace(self.vehicle, suspension=suspension), self.plan)
        state = plant.initial_state(15.0)
        state[STEERING_INDEX] = 0.2
        still = plant.evaluate(0.0, state, DriverCommand(0.0, 0.0, 0.0)).forcing
        moving = plant.evaluate(0.0, state, DriverCommand(0.0, 0.0, 0.3)).forcing
        self.assertTrue(np.all(np.isfinite(moving)))
        self.assertFalse(np.array_equal(still, moving))

    def test_forward_plant_protocol_is_checked_the_same_way_as_the_planar_plant(self):
        planar = PlanarForwardPlant(
            compile_formulation(PLANAR_FORMULATION_XML).model, straight_plan(8.0)
        )
        # ForwardPlant is a static-only Protocol, so neither plant is
        # runtime-checkable and isinstance raises for both.
        with self.assertRaises(TypeError):
            isinstance(planar, ForwardPlant)
        with self.assertRaises(TypeError):
            isinstance(self.plant, ForwardPlant)
        for attribute in ("state_size", "observe", "evaluate", "powertrain", "initial_state"):
            self.assertTrue(hasattr(self.plant, attribute), attribute)
        self.assertTrue(callable(self.plant.observe))
        self.assertTrue(callable(self.plant.evaluate))
        self.assertEqual(self.plant.state_size, 38)

    def test_state_names_match_the_documented_layout(self):
        names = self.plant.state_names
        self.assertEqual(len(names), 38)
        self.assertEqual(names[:3], ("s", "n", "heading_error"))
        self.assertEqual(names[3:24], tuple(field.name for field in fields(VerticalState)))
        self.assertEqual(names[SOC_INDEX], "soc")
        self.assertEqual(names[STEERING_INDEX], "steering")
        self.assertEqual(names[TIRE_OFFSET:TIRE_OFFSET + 3], (
            "kappa_eff_fl", "lateral_deformation_fl", "trail_angle_fl"))
        self.assertEqual(names[-1], "trail_angle_rr")
        self.assertEqual(len(MECHANICAL_INDEX), 21)
        self.assertEqual(len(TIRE_INDEX), 12)
        for wheel, name in enumerate(WHEELS):
            offset = TIRE_OFFSET + 3 * wheel
            self.assertEqual(names[offset:offset + 3],
                             (f"kappa_eff_{name}", f"lateral_deformation_{name}",
                              f"trail_angle_{name}"))
        self.assertEqual(tuple(MECHANICAL_INDEX),
                         tuple(range(3, 24)))
        self.assertEqual(tuple(TIRE_INDEX), tuple(range(TIRE_OFFSET, TIRE_OFFSET + 12)))

    def test_periodic_metadata_covers_every_tire_state(self):
        indices = self.plant.periodic_indices
        tolerances = self.plant.periodic_tolerances
        self.assertEqual(len(indices), len(tolerances))
        self.assertEqual(len(indices), 36)
        self.assertNotIn(0, indices)          # arc length restarts each lap
        self.assertNotIn(SOC_INDEX, indices)  # charge is consumed
        for index in TIRE_INDEX:
            self.assertIn(index, indices)
        self.assertTrue(np.all(np.asarray(tolerances) > 0.0))
        self.assertTrue(np.all(np.isfinite(np.asarray(tolerances))))
        by_name = dict(zip((self.plant.state_names[index] for index in indices), tolerances))
        self.assertEqual(by_name["n"], 5.0e-4)
        self.assertEqual(by_name["heave"], 5.0e-4)
        self.assertEqual(by_name["jounce_rate_fl"], 1.0e-2)
        self.assertEqual(by_name["roll"], 5.0e-3)
        self.assertEqual(by_name["v_x"], 1.0e-2)
        self.assertEqual(by_name["kappa_eff_fl"], 2.0e-3)
        self.assertEqual(by_name["lateral_deformation_fl"], 2.0e-4)
        self.assertEqual(by_name["trail_angle_rr"], 2.0e-3)

    def test_powertrain_uses_the_shared_closure(self):
        state = self.plant.initial_state(15.0)
        command = DriverCommand(0.3, 0.0, 0.0)
        point = self.plant.powertrain(state, command)
        expected = evaluate_forward_powertrain(
            self.vehicle.planar, self.plant.driven, self.plant.gearing,
            self.plant.gear_efficiency, state[12:16], state[SOC_INDEX], command,
        )
        np.testing.assert_array_equal(point.wheel_torques, expected.wheel_torques)
        self.assertEqual(point.voltage, expected.voltage)
        self.assertEqual(point.current, expected.current)
        self.assertEqual(point.bus_power, expected.bus_power)

    def test_trial_domain_rejects_out_of_range_trials(self):
        state = self.plant.initial_state(15.0)
        command = DriverCommand(0.0, 0.0, 0.0)
        self.assertTrue(np.all(np.isfinite(self.plant.evaluate(0.0, state, command).forcing)))
        beyond = state.copy()
        beyond[16] = JOUNCE_LIMIT + 0.011
        with self.assertRaises(TrialStateError):
            self.plant.evaluate(0.0, beyond, command)
        inside = state.copy()
        inside[16] = 0.020
        self.assertTrue(np.all(np.isfinite(self.plant.evaluate(0.0, inside, command).forcing)))
        # Inside the trial margin but outside the compiled jounce-steer table
        # domain: the plant reports a rejected trial, not a hard error, and counts
        # the table exit in out_of_range_evaluations (the damper-only counter the
        # evaluator reports cannot see spring or jounce-steer table exits).
        self.assertLess(JOUNCE_LIMIT + 0.009, self.plant.jounce_maximum[0])
        fresh = VerticalForwardPlant(compiled_vehicle(), self.plan)
        # A damper-table exit does not raise: the table extends linearly and is
        # flagged, and the plant counts the flagged evaluation.
        beyond_damper = state.copy()
        beyond_damper[20] = 5.0  # jounce rate far beyond the damper velocity axis
        damper_system = fresh.evaluate(0.0, beyond_damper, command)
        self.assertTrue(np.all(np.isfinite(damper_system.forcing)))
        self.assertEqual(fresh.out_of_range_evaluations, 1)
        # A spring/jounce-steer table exit is a rejected trial and is counted too.
        table_edge = state.copy()
        table_edge[16] = JOUNCE_LIMIT + 0.009
        with self.assertRaises(TrialStateError):
            fresh.evaluate(0.0, table_edge, command)
        self.assertEqual(fresh.out_of_range_evaluations, 2)
        charge = state.copy()
        charge[SOC_INDEX] = 1.05
        with self.assertRaises(TrialStateError):
            self.plant.evaluate(0.0, charge, command)
        reverse = state.copy()
        reverse[12] = -1.0
        with self.assertRaises(TrialStateError):
            self.plant.evaluate(0.0, reverse, command)
        steering = state.copy()
        steering[STEERING_INDEX] = self.vehicle.planar.steering_maximum_angle + 0.1
        with self.assertRaises(ValueError):
            self.plant.evaluate(0.0, steering, command)

    def test_diagnostics_and_observation_come_from_the_last_evaluation(self):
        # A fresh plant: the counters below must not depend on test order.
        vehicle = compiled_vehicle()
        plant = VerticalForwardPlant(vehicle, self.plan)
        state = plant.initial_state(15.0)
        command = DriverCommand(0.0, 0.0, 0.0)
        plant.evaluate(0.0, state, command)
        observation = plant.observe(state)
        self.assertAlmostEqual(observation.distance, float(state[0]))
        self.assertAlmostEqual(observation.speed, float(state[3]))
        self.assertAlmostEqual(observation.steering, float(state[STEERING_INDEX]))
        self.assertTrue(np.isfinite(observation.wheel_speed_margin))
        for attribute in ("maximum_load_residual", "minimum_contact_load",
                          "maximum_power_residual", "maximum_load_iterations",
                          "load_method_counts", "minimum_load_jacobian_determinant",
                          "lifted_wheel_evaluations", "speed_regulated_evaluations",
                          "maximum_motor_speed_ratio", "evaluations", "last_time",
                          "last_distance", "out_of_range_evaluations", "maximum_jounce",
                          "minimum_jounce", "maximum_damper_velocity"):
            self.assertTrue(hasattr(plant, attribute), attribute)
        self.assertEqual(plant.evaluations, 1)
        self.assertEqual(plant.out_of_range_evaluations, 0)
        self.assertGreaterEqual(plant.maximum_jounce, plant.minimum_jounce)
        self.assertGreaterEqual(plant.maximum_damper_velocity, 0.0)
        # The road-plane accelerations of the last evaluation are exposed on the
        # plant (DriverObservation has no acceleration fields).
        evaluation = evaluate_vertical_dynamics(
            vehicle, plant.mechanical_state(state),
            plant.powertrain(state, command).wheel_torques,
            float(state[STEERING_INDEX]), plant.tire_states(state),
        )
        np.testing.assert_allclose(plant.body_accelerations,
                                   evaluation.body_accelerations, rtol=0.0, atol=0.0)
        # The module-level helper reproduces them from state and rates.
        rates = plant.evaluate(0.0, state, command).solve()
        self.assertEqual(plant.evaluations, 2)
        ax, ay = body_accelerations(state, rates)
        self.assertAlmostEqual(float(ax), float(plant.body_accelerations[0]))
        self.assertAlmostEqual(float(ay), float(plant.body_accelerations[1]))
        with self.assertRaises(ValueError):
            body_accelerations(np.zeros(38), np.zeros(12))

    def test_mechanical_and_tire_state_accessors_round_trip(self):
        state = self.plant.initial_state(15.0)
        mechanical = self.plant.mechanical_state(state)
        np.testing.assert_array_equal(mechanical.as_vector(), state[list(MECHANICAL_INDEX)])
        tires = self.plant.tire_states(state)
        self.assertEqual(len(tires), 4)
        np.testing.assert_allclose(
            [value for tire in tires for value in
             (tire.longitudinal_slip, tire.lateral_deformation, tire.trail_angle)],
            state[list(TIRE_INDEX)],
        )


class VerticalForwardInitialStateTests(unittest.TestCase):
    """The plant opens with no artificial vertical or tire transient."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = flat_vehicle()

    def _assert_steady(self, speed: float, curvature: float = 0.0):
        plan = SpeedPlan(np.array([0.0, 200.0]), np.full(2, speed), np.full(2, curvature))
        plant = VerticalForwardPlant(self.vehicle, plan)
        state = plant.initial_state(speed)
        # "Steady speed" command: the plan's constant speed with a zero
        # throttle/brake command. The wheel torques are zero there and the
        # initial state is refined so the seven vertical generalized forces
        # vanish; drag still decelerates the body longitudinally, which is not
        # a vertical rate.
        rates = plant.evaluate(0.0, state, DriverCommand(0.0, 0.0, 0.0)).solve()
        vertical = rates[list(PLANT_RATE_ROWS) + list(PLANT_POSITION_ROWS)]
        self.assertLess(float(np.max(np.abs(vertical))), 1e-6)
        self.assertLess(float(np.max(np.abs(rates[list(TIRE_INDEX)]))), 1e-6)
        return plant, state

    def test_initial_state_has_no_vertical_rates_at_15_m_s(self):
        self._assert_steady(15.0)

    def test_initial_state_is_steady_at_30_m_s_and_on_a_curve(self):
        self._assert_steady(30.0)
        self._assert_steady(15.0, curvature=0.02)

    def test_tire_states_start_at_their_steady_rolling_values(self):
        plant, state = self._assert_steady(15.0)
        loads = evaluate_vertical_dynamics(
            self.vehicle, plant.mechanical_state(state), np.zeros(4),
            float(state[STEERING_INDEX]), plant.tire_states(state),
        ).normal_loads
        for wheel, tire in enumerate(plant.tire_states(state)):
            expected = steady_rolling_state(
                self.vehicle.planar.tire, self.vehicle.transient_tire,
                0.0, 0.0, float(loads[wheel]),
            )
            np.testing.assert_allclose(
                [tire.longitudinal_slip, tire.lateral_deformation, tire.trail_angle],
                [expected.longitudinal_slip, expected.lateral_deformation,
                 expected.trail_angle],
                rtol=0.0, atol=1e-12,
            )
        self.assertAlmostEqual(float(state[2]), 0.0, places=12)

    def test_initial_speed_must_exceed_one_m_s(self):
        plant = VerticalForwardPlant(self.vehicle, straight_plan())
        for speed in (0.0, 1.0, -3.0, float("nan")):
            with self.assertRaises(ValueError):
                plant.initial_state(speed)


class VerticalForwardTireStateTests(unittest.TestCase):
    """The plant's tire-state rows relax at the planned relaxation rates."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = flat_vehicle()
        cls.plant = VerticalForwardPlant(cls.vehicle, straight_plan(20.0))
        cls.state = cls.plant.initial_state(20.0)

    def test_perturbed_lateral_deformation_has_the_planned_decay_rate(self):
        state = self.state.copy()
        perturbation = 2.0e-3
        state[TIRE_OFFSET + 1::3] += perturbation  # lateral_deformation, all wheels
        command = DriverCommand(0.0, 0.0, 0.0)
        rates = self.plant.evaluate(0.0, state, command).solve()
        loads = evaluate_vertical_dynamics(
            self.vehicle, self.plant.mechanical_state(state), np.zeros(4),
            float(state[STEERING_INDEX]), self.plant.tire_states(state),
        ).normal_loads
        speed = float(state[3])
        for wheel in range(4):
            sigma_x, sigma_y = relaxation_lengths(
                self.vehicle.planar.tire, self.vehicle.transient_tire, float(loads[wheel])
            )
            # alpha_effective grows by perturbation / sigma_y, so the
            # deformation rate is -|Vx| * perturbation / sigma_y.
            np.testing.assert_allclose(
                rates[TIRE_OFFSET + 3 * wheel + 1],
                -speed * perturbation / sigma_y,
                rtol=1e-9, atol=1e-15,
            )
            # The longitudinal slip state relaxes with sigma_x / |Vx|; a
            # perturbation decays at exactly that rate.
            self.assertGreater(sigma_x, 0.0)
            self.assertGreater(sigma_y, 0.0)

    def test_lateral_deformation_decays_at_the_planned_time_constant(self):
        state = self.state.copy()
        perturbation = 2.0e-3
        state[TIRE_OFFSET + 1::3] += perturbation
        command = DriverCommand(0.0, 0.0, 0.0)
        # Freeze the mechanical state and integrate only the 12 tire-state rows
        # through the plant's own rate path. The load is then constant, so the
        # deformation decays as a single exponential.
        loads = evaluate_vertical_dynamics(
            self.vehicle, self.plant.mechanical_state(state), np.zeros(4),
            float(state[STEERING_INDEX]), self.plant.tire_states(state),
        ).normal_loads
        # Each corner has its own load, hence its own relaxation length.
        taus = np.array([
            relaxation_lengths(self.vehicle.planar.tire, self.vehicle.transient_tire,
                               float(loads[wheel]))[1] / float(state[3])
            for wheel in range(4)
        ])
        duration = 4.0 * float(np.min(taus))

        def rhs(_time, tire_values):
            trial = state.copy()
            trial[list(TIRE_INDEX)] = tire_values
            return self.plant.evaluate(0.0, trial, command).solve()[list(TIRE_INDEX)]

        solution = solve_ivp(rhs, (0.0, duration), state[list(TIRE_INDEX)],
                             method="Radau", rtol=1e-10, atol=1e-14)
        self.assertTrue(solution.success)
        final = solution.y[1::3, -1]
        np.testing.assert_allclose(final, perturbation * np.exp(-duration / taus),
                                   rtol=1e-3, atol=1e-12)
        # The decay rate from two samples matches each corner's 1/tau.
        measured = -np.log(final / perturbation) / duration
        np.testing.assert_allclose(measured, 1.0 / taus, rtol=0.02)


class VerticalForwardStraightTrialTests(unittest.TestCase):
    """A straight 100 m trial at constant throttle completes inside the domain."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compiled_vehicle()
        cls.plan = straight_plan()
        cls.plant = VerticalForwardPlant(cls.vehicle, cls.plan)
        cls.driver = ConstantThrottleDriver(cls.plan, TRIAL_THROTTLE)
        cls.initial = cls.plant.initial_state(TRIAL_SPEED)
        started = time.perf_counter()
        cls.result = solve_forward(
            cls.plant, cls.driver, cls.initial, duration=60.0, wall_seconds=280.0,
            maximum_step=TRIAL_MAXIMUM_STEP, rtol=1e-6, atol=1e-8, method="BDF",
        )
        cls.wall_seconds = time.perf_counter() - started

    def test_trial_completes_and_stays_inside_the_travel(self):
        result = self.result
        self.assertTrue(result.lap_completed)
        self.assertGreaterEqual(float(result.state[0, -1]), TRIAL_DISTANCE - 1e-6)
        jounce = result.state[16:20]
        self.assertLessEqual(float(np.max(np.abs(jounce))), JOUNCE_LIMIT)
        if result.dense_bounds is not None:
            # Shape (intervals, components, 2): the dense-polynomial bounds of
            # every accepted interval must also stay inside the travel.
            bounds = np.asarray(result.dense_bounds)[:, 16:20, :]
            self.assertLessEqual(float(np.max(bounds[:, :, 1])), JOUNCE_LIMIT)
            self.assertGreaterEqual(float(np.min(bounds[:, :, 0])), -JOUNCE_LIMIT)
        self.assertEqual(self.plant.out_of_range_evaluations, 0)
        self.assertEqual(self.plant.lifted_wheel_evaluations, 0)
        self.assertGreaterEqual(self.plant.minimum_contact_load, 0.0)
        self.assertLessEqual(self.plant.maximum_jounce, JOUNCE_LIMIT)
        self.assertGreaterEqual(self.plant.minimum_jounce, -JOUNCE_LIMIT)

    def test_tire_states_stay_finite_and_small(self):
        result = self.result
        tires = result.state[26:38]
        self.assertTrue(np.all(np.isfinite(tires)))
        self.assertLess(float(np.max(np.abs(tires))), 5.0e-3)
        initial = self.initial[26:38, None]
        for wheel in range(4):
            self.assertLess(abs(float(np.max(np.abs(tires[3 * wheel:3 * wheel + 3])))),
                            0.02)
        self.assertLess(float(np.max(np.abs(tires - initial))), 0.02)

    def test_trial_used_the_mass_matrix_free_path(self):
        # The compiled vehicle has zero inertance, so the evaluator returns
        # mass_matrix is None and the plant embeds the solved rates directly.
        state = self.plant.initial_state(TRIAL_SPEED)
        system = self.plant.evaluate(0.0, state, DriverCommand(0.0, 0.0, 0.0))
        self.assertIsNone(system.mass_matrix)


class VerticalForwardDriverLoopTests(unittest.TestCase):
    """The PreviewDriver closes the loop on a curved plan without rejections."""

    def test_short_curved_closed_loop_trial_runs_cleanly(self):
        from simplified_vehicle_models.solvers.forward_driver import PreviewDriver

        vehicle = compiled_vehicle()
        planar = vehicle.planar
        curvature = -0.01
        plan = SpeedPlan(np.array([0.0, 300.0]), np.full(2, 15.0), np.full(2, curvature))
        plant = VerticalForwardPlant(vehicle, plan)
        driver = PreviewDriver(plan, planar.wheelbase, planar.steering_maximum_angle,
                               planar.steering_maximum_rate, planar.maximum_pedal_force)
        initial = plant.initial_state(15.0)
        result = solve_forward(plant, driver, initial, duration=6.0, wall_seconds=280.0,
                               maximum_step=TRIAL_MAXIMUM_STEP, rtol=1e-6, atol=1e-8,
                               method="BDF")
        self.assertTrue(result.midpoint_sample_validation_complete)
        self.assertEqual(result.rejected_domain_trials, 0)
        self.assertEqual(plant.out_of_range_evaluations, 0)
        self.assertEqual(plant.lifted_wheel_evaluations, 0)
        self.assertGreater(float(result.state[0, -1]), 50.0)
        self.assertTrue(np.all(np.isfinite(result.state)))
        self.assertLess(float(np.max(np.abs(result.state[1]))), 0.25)
        self.assertLessEqual(float(np.max(np.abs(result.state[16:20]))), JOUNCE_LIMIT)
        self.assertTrue(np.isfinite(plant.observe(result.state[:, -1]).wheel_speed_margin))


class VerticalForwardMassMatrixTests(unittest.TestCase):
    """The embedded mass-matrix path reproduces the evaluator's rate system."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compiled_vehicle()
        cls.inerted = with_inertance(cls.vehicle)
        cls.plan = straight_plan()
        cls.plant = VerticalForwardPlant(cls.inerted, cls.plan)
        cls.state = cls.plant.initial_state(15.0)
        cls.command = DriverCommand(0.0, 0.0, 0.0)

    def test_positive_inertance_returns_the_embedded_38x38_mass_matrix(self):
        system = self.plant.evaluate(0.0, self.state, self.command)
        self.assertIsNotNone(system.mass_matrix)
        matrix = np.asarray(system.mass_matrix)
        self.assertEqual(matrix.shape, (38, 38))
        zero_inertance = VerticalForwardPlant(compiled_vehicle(), self.plan)
        plain = zero_inertance.evaluate(0.0, zero_inertance.initial_state(15.0),
                                        self.command)
        self.assertIsNone(plain.mass_matrix)

    def test_embedded_mechanical_rows_equal_the_evaluator_rate_system(self):
        state = self.state
        system = self.plant.evaluate(0.0, state, self.command)
        point = self.plant.powertrain(state, self.command)
        evaluation = evaluate_vertical_dynamics(
            self.inerted, self.plant.mechanical_state(state), point.wheel_torques,
            float(state[STEERING_INDEX]), self.plant.tire_states(state),
        )
        expected = RateSystem(evaluation.forcing, evaluation.mass_matrix).solve()
        solved = system.solve()
        # The plant solves the embedded 38x38 block, so the mechanical rows
        # round differently from a direct 21x21 solve by a few ulp.
        np.testing.assert_allclose(solved[list(MECHANICAL_INDEX)], expected,
                                   rtol=1e-12, atol=1e-15)
        # The plant must not re-solve the evaluator's already-solved rates.
        np.testing.assert_array_equal(np.asarray(system.forcing)[list(MECHANICAL_INDEX)],
                                      evaluation.forcing)
        block = list(MECHANICAL_INDEX)
        np.testing.assert_array_equal(
            np.asarray(system.mass_matrix)[np.ix_(block, block)], evaluation.mass_matrix)
        # Everything outside the mechanical block is an identity row.
        outside = [index for index in range(38) if index not in MECHANICAL_INDEX]
        np.testing.assert_array_equal(np.asarray(system.mass_matrix)[np.ix_(outside, outside)],
                                      np.eye(len(outside)))
        np.testing.assert_array_equal(np.asarray(system.mass_matrix)[np.ix_(outside, block)],
                                      np.zeros((len(outside), len(block))))

    def test_values_outside_the_mechanical_block_are_the_documented_rates(self):
        """Non-straight state, curvature, and steering rate make every row live.

        On the straight initial state rows 1, 2 and the steering row are all zero,
        so an assertion there would pass for the wrong reason. This case uses
        ``n``, ``v_y``, ``yaw_rate``, ``heading_error`` and a curved plan, and the
        expected values are the planar path-coordinate formulas written out here
        rather than read back from the plant.
        """
        curvature = -0.02
        plan = SpeedPlan(np.array([0.0, 200.0]), np.full(2, 15.0), np.full(2, curvature))
        plant = VerticalForwardPlant(self.inerted, plan)
        state = plant.initial_state(15.0)
        state[1] = 0.35          # lateral error n
        state[2] = 0.04          # heading error
        state[3] = 15.0
        state[4] = 0.9           # body lateral speed v_y
        state[8] = -0.06         # yaw rate
        steering_rate = 0.003
        command = DriverCommand(0.2, 0.0, steering_rate)
        forcing = np.asarray(plant.evaluate(0.0, state, command).forcing)

        denominator = 1.0 - curvature * state[1]
        progress = (state[3] * math.cos(state[2]) - state[4] * math.sin(state[2])) / denominator
        expected_row_1 = state[3] * math.sin(state[2]) + state[4] * math.cos(state[2])
        expected_row_2 = state[8] - curvature * progress
        # Every asserted quantity must be materially nonzero first.
        self.assertGreater(abs(progress), 0.1)
        self.assertGreater(abs(expected_row_1), 0.1)
        self.assertGreater(abs(expected_row_2), 0.1)
        self.assertEqual(steering_rate, command.steering_rate)
        self.assertAlmostEqual(float(forcing[0]), float(progress), places=12)
        self.assertAlmostEqual(float(forcing[1]), float(expected_row_1), places=12)
        self.assertAlmostEqual(float(forcing[2]), float(expected_row_2), places=12)
        self.assertAlmostEqual(float(forcing[STEERING_INDEX]), steering_rate, places=15)
        self.assertAlmostEqual(
            float(forcing[SOC_INDEX]),
            -float(plant.powertrain(state, command).current)
            / self.inerted.planar.storage.pack_capacity, places=15)


class VerticalForwardStepTests(unittest.TestCase):
    """A 2 000 N front-axle step against the valid seven-coordinate oracle."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = with_inertance(compiled_vehicle())
        cls.plan = straight_plan(15.0)
        cls.plant = VerticalForwardPlant(cls.vehicle, cls.plan)
        cls.initial = cls.plant.initial_state(15.0)
        positions = sprung_wheel_positions(cls.vehicle)
        cls.x_front = float(positions[0, 0])
        cls.y_front = float(positions[0, 1])
        cls.offset = np.zeros(38)
        cls.offset[PLANT_RATE_ROWS[0]] += STEP_FORCE
        cls.offset[PLANT_RATE_ROWS[2]] += -STEP_FORCE * cls.x_front
        cls.command = DriverCommand(0.0, 0.0, 0.0)
        cls.times = np.linspace(0.0, STEP_DURATION, 12001)
        mechanical = cls.initial[list(MECHANICAL_INDEX)]
        cls.mechanical = mechanical
        cls.vertical0 = mechanical[list(POSITION_ROWS)]
        cls.front_body0 = cls._front_body(cls.vertical0)
        cls.nonlinear = cls._nonlinear_response()
        cls.linear = cls._linear_response()

    @classmethod
    def _front_body(cls, coordinates):
        return float(coordinates[0] + cls.y_front * coordinates[1]
                     - cls.x_front * coordinates[2])

    @classmethod
    def _step_rates(cls, time, vertical):
        state = cls.initial.copy()
        state[list(PLANT_RATE_ROWS)] = vertical[:7]
        state[list(PLANT_POSITION_ROWS)] = vertical[7:]
        system = cls.plant.evaluate(time, state, cls.command)
        rates = RateSystem(system.forcing + cls.offset, system.mass_matrix).solve()
        return np.r_[rates[list(PLANT_RATE_ROWS)], rates[list(PLANT_POSITION_ROWS)]]

    @classmethod
    def _nonlinear_response(cls):
        initial = np.r_[np.zeros(7), cls.vertical0]
        solution = solve_ivp(cls._step_rates, (0.0, STEP_DURATION), initial, method="BDF",
                             rtol=1e-8, atol=1e-10, max_step=0.002, t_eval=cls.times)
        assert solution.success, solution.message
        return ((solution.y[7] + cls.y_front * solution.y[8] - cls.x_front * solution.y[9])
                - cls.front_body0)

    @classmethod
    def _linear_response(cls):
        mass = closed_form_mass(cls.vehicle)
        stiffness = closed_form_stiffness(cls.vehicle, cls.vertical0)
        damping = closed_form_damping(cls.vehicle)
        generalized = np.array([STEP_FORCE, 0.0, -STEP_FORCE * cls.x_front,
                                0.0, 0.0, 0.0, 0.0])

        def rhs(_time, y):
            position, velocity = y[:7], y[7:]
            return np.r_[velocity,
                         np.linalg.solve(mass, generalized - stiffness @ position
                                         - damping @ velocity)]

        solution = solve_ivp(rhs, (0.0, STEP_DURATION), np.zeros(14), method="DOP853",
                            rtol=1e-12, atol=1e-14, t_eval=cls.times)
        assert solution.success, solution.message
        return solution.y[0] + cls.y_front * solution.y[1] - cls.x_front * solution.y[2]

    def test_first_overshoot_matches_the_seven_coordinate_linear_oracle(self):
        peak_nonlinear, time_nonlinear = first_overshoot(self.times, self.nonlinear)
        peak_linear, time_linear = first_overshoot(self.times, self.linear)
        final_nonlinear = float(self.nonlinear[-1])
        final_linear = float(self.linear[-1])
        # The steady responses agree (the nonlinear force path is close to
        # linear over this amplitude).
        self.assertLess(abs(final_nonlinear / final_linear - 1.0), 0.05)
        # The ±15 % gate is on the first-peak deviation from the initial state.
        self.assertLess(abs(peak_nonlinear / peak_linear - 1.0), 0.15,
                        f"nonlinear peak {peak_nonlinear * 1e3:.3f} mm vs "
                        f"linear {peak_linear * 1e3:.3f} mm")
        self.assertLess(abs(time_nonlinear / time_linear - 1.0), 0.15)
        # Overshoot fractions agree within ±15 % relative.
        overshoot_nonlinear = peak_nonlinear / final_nonlinear - 1.0
        overshoot_linear = peak_linear / final_linear - 1.0
        self.assertLess(abs(overshoot_nonlinear / overshoot_linear - 1.0), 0.15)

    def test_quarter_car_reference_is_reported_not_gated(self):
        """The plan's body-fixed quarter-car overshoot does not describe this step.

        The 4.42 Hz / 0.49 front reference predicts a 17.1 % first overshoot for
        a quarter car, but a front-axle step on the whole vehicle excites the
        coupled heave (3.83 Hz) and pitch (7.30 Hz) modes and peaks near 0.08 s.
        The check below only records the mismatch; the gate above is the valid
        seven-coordinate oracle.
        """
        peak_nonlinear, _ = first_overshoot(self.times, self.nonlinear)
        overshoot = peak_nonlinear / float(self.nonlinear[-1]) - 1.0
        # The quarter-car first overshoot fraction for zeta = 0.49 is +0.171
        # (the peak rises 17.1 % above the steady value), not 0.171 - 1.
        quarter_car = math.exp(-math.pi * 0.49 / math.sqrt(1.0 - 0.49**2))
        self.assertAlmostEqual(quarter_car, 0.171, places=3)
        self.assertGreater(overshoot, 1.4 * quarter_car)
        self.assertLess(overshoot, 0.40)
        # The quarter-car reference misses the ±15 % gate; this is the disclosed
        # reason it is not used as the pass/fail oracle.
        self.assertGreater(abs(overshoot / quarter_car - 1.0), 0.15)


class VerticalForwardPerformanceReportTests(unittest.TestCase):
    """Report evaluation cost and projected lap wall time; do not gate on time."""

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compiled_vehicle()
        cls.plan = straight_plan()
        cls.plant = VerticalForwardPlant(cls.vehicle, cls.plan)
        cls.state = cls.plant.initial_state(TRIAL_SPEED)
        cls.command = DriverCommand(TRIAL_THROTTLE, 0.0, 0.0)

    def test_report_evaluation_cost_and_projected_lap_time(self):
        """Print cost numbers; never fail on a machine-dependent threshold."""
        plant = VerticalForwardPlant(self.vehicle, self.plan)
        state = plant.initial_state(TRIAL_SPEED)
        command = DriverCommand(TRIAL_THROTTLE, 0.0, 0.0)
        # 1000 timed calls after a warm-up call, so import and cache effects are
        # not part of the measurement.
        plant.evaluate(0.0, state, command)
        samples = np.empty(1000)
        for index in range(1000):
            started = time.perf_counter()
            plant.evaluate(0.0, state, command)
            samples[index] = time.perf_counter() - started
        median = float(np.median(samples))
        p95 = float(np.percentile(samples, 95))
        self.assertGreater(median, 0.0)
        self.assertGreaterEqual(p95, median)

        # Count every plant evaluation of the trial, including the accepted-state
        # and between-step replay calls that ``ForwardResult.evaluations`` does
        # not see, and capture the integrator's own call counts.
        calls = {"total": 0}
        real_evaluate = plant.evaluate

        def counting(*args, **kwargs):
            calls["total"] += 1
            return real_evaluate(*args, **kwargs)

        plant.evaluate = counting
        counts = {}
        original = scipy.integrate.solve_ivp

        def spy(*args, **kwargs):
            result = original(*args, **kwargs)
            counts["njev"] = int(getattr(result, "njev", 0))
            counts["nfev"] = int(getattr(result, "nfev", 0))
            return result

        try:
            with patch.object(scipy.integrate, "solve_ivp", spy):
                trial = solve_forward(
                    plant, ConstantThrottleDriver(self.plan, TRIAL_THROTTLE),
                    state, duration=60.0, wall_seconds=280.0,
                    maximum_step=TRIAL_MAXIMUM_STEP, rtol=1e-6, atol=1e-8, method="BDF",
                )
        finally:
            plant.evaluate = real_evaluate
        self.assertTrue(trial.lap_completed)
        self.assertGreater(counts["njev"], 0)
        self.assertGreater(calls["total"], trial.evaluations)
        # Each finite-difference Jacobian costs the baseline call plus one call
        # per state (38), and the rest of the total is the accepted-state and
        # between-step replay.
        columns = 39
        replay = calls["total"] - counts["nfev"] - counts["njev"] * columns
        calls_per_metre = calls["total"] / float(trial.state[0, -1])
        lap_length = _michigan_lap_length()
        projected_calls = calls_per_metre * lap_length
        projected_seconds = projected_calls * median
        print(
            "PERFORMANCE REPORT: median evaluate "
            f"{median * 1e3:.3f} ms, p95 {p95 * 1e3:.3f} ms; "
            f"100 m BDF trial: {trial.evaluations} solver rate-path calls, "
            f"{counts['njev']} Jacobian evaluations, {counts['nfev']} rhs calls, "
            f"{trial.time.size - 1} accepted steps, {calls['total']} total plant "
            f"evaluate calls including {replay} replay calls; "
            f"projected full lap {lap_length:.1f} m -> {projected_calls:.0f} calls, "
            f"{projected_seconds:.0f} s at the median rate"
        )
        self.assertTrue(np.isfinite(projected_seconds))
        self.assertGreater(projected_seconds, 0.0)


def _michigan_lap_length() -> float:
    """Return the closed Michigan endurance lap length [m] used for projection."""

    from simplified_vehicle_models.tracks.profile import load_track

    track = load_track(ROOT / "models/tracks/mis_ev_endurance_2024_linear.xml")
    return float(track.lap_length)


class PowertrainRefactorGateTests(unittest.TestCase):
    """Bitwise gate for the Task 7 powertrain extraction.

    The fixture was recorded from the pre-move code (generator:
    ``/tmp/record_planar_powertrain_fixture.py``, seed 20261016) before
    ``PlanarForwardPlant.powertrain``'s body moved to
    ``solvers/forward_powertrain.py``.
    """

    @classmethod
    def setUpClass(cls):
        cls.recorded = np.load(POWERTRAIN_FIXTURE)
        cls.vehicle = compile_formulation(PLANAR_FORMULATION_XML).model
        cls.plan = straight_plan(20.0)
        cls.plant = PlanarForwardPlant(cls.vehicle, cls.plan)

    def test_fixture_holds_the_recorded_in_domain_states(self):
        recorded = self.recorded
        count = recorded["state"].shape[0]
        # The first 50 rows are the original recording; the speed-regulation
        # rows appended from the same pre-move code pin the ``limited`` flag.
        self.assertGreaterEqual(count, 50)
        self.assertEqual(recorded["state"].shape, (count, 12))
        self.assertEqual(recorded["command"].shape, (count, 3))
        self.assertEqual(recorded["rate"].shape, (count, 12))
        self.assertEqual(str(recorded["vehicle_xml"][0]),
                         "models/formulations/iem24_murphy_real_planar_7dof.xml")
        self.assertEqual(int(recorded["seed"][0]), 20261016)
        self.assertTrue(np.all(np.isfinite(recorded["rate"])))
        # At least one recorded state must be limited, or the flag is unpinned.
        self.assertGreaterEqual(int(np.count_nonzero(recorded["limited"])), 1)
        self.assertFalse(bool(np.all(recorded["limited"])))

    def test_planar_rates_and_powertrain_outputs_are_bitwise_unchanged(self):
        recorded = self.recorded
        for index in range(recorded["state"].shape[0]):
            state = recorded["state"][index]
            command = DriverCommand(*(float(value) for value in recorded["command"][index]))
            rate = self.plant.evaluate(0.0, state, command).solve()
            self.assertTrue(np.array_equal(rate, recorded["rate"][index]),
                            f"rate mismatch at recorded state {index}")
            point = self.plant.powertrain(state, command)
            self.assertTrue(np.array_equal(point.wheel_torques,
                                           recorded["wheel_torques"][index]))
            self.assertTrue(np.array_equal(point.motor_wheel_torques,
                                           recorded["motor_wheel_torques"][index]))
            self.assertEqual(point.bus_power, float(recorded["bus_power"][index]))
            self.assertEqual(point.voltage, float(recorded["voltage"][index]))
            self.assertEqual(point.current, float(recorded["current"][index]))
            self.assertEqual(point.power_residual, float(recorded["power_residual"][index]))
            self.assertEqual(bool(point.limited), bool(recorded["limited"][index]))

    def test_thin_wrapper_matches_the_shared_function_directly(self):
        recorded = self.recorded
        for index in range(recorded["state"].shape[0]):
            state = recorded["state"][index]
            command = DriverCommand(*(float(value) for value in recorded["command"][index]))
            point = self.plant.powertrain(state, command)
            expected = evaluate_forward_powertrain(
                self.plant.vehicle, self.plant.driven, self.plant.gearing,
                self.plant.gear_efficiency, state[6:10], state[10], command,
            )
            self.assertTrue(np.array_equal(point.wheel_torques, expected.wheel_torques))


if __name__ == "__main__":
    unittest.main()
