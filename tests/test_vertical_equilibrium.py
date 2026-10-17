"""Oracle tests for static vertical equilibrium and the steady corner solve."""

from pathlib import Path
import unittest

import numpy as np

import dataclasses

from simplified_vehicle_models.aero.fixed_aero import aerodynamic_forces
from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState,
    contact_velocities,
    slip_quantities,
    wheel_positions,
    wheel_steering_angles,
)
from simplified_vehicle_models.solvers.vertical_equilibrium import (
    _solve_steady_corner_detail,
    solve_static_equilibrium,
    solve_steady_corner,
)
from simplified_vehicle_models.suspension.corner import corner_forces, corner_toe_angles
from simplified_vehicle_models.suspension.monotone_spline import MonotoneSpline
from simplified_vehicle_models.tires.unitire_simple import loaded_radius
from simplified_vehicle_models.tires.unitire_transient import steady_rolling_state
from simplified_vehicle_models.tires.vertical import tire_deflection_load
from simplified_vehicle_models.formulations.vertical_14dof import (
    VerticalState,
    evaluate_vertical_dynamics,
    sprung_wheel_positions,
)


ROOT = Path(__file__).resolve().parents[1]
FORMULATION = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"


class VerticalEquilibriumOracleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(str(FORMULATION)).model

    def test_static_equilibrium_load_oracle(self):
        vehicle = self.vehicle
        state = solve_static_equilibrium(vehicle)
        self.assertAlmostEqual(state.jounce_fl, 0.0, delta=1e-6)
        self.assertAlmostEqual(state.jounce_fr, 0.0, delta=1e-6)
        self.assertAlmostEqual(state.jounce_rl, 0.0, delta=1e-6)
        self.assertAlmostEqual(state.jounce_rr, 0.0, delta=1e-6)
        q = state.as_vector()
        positions = sprung_wheel_positions(vehicle)
        jounce = q[13:17]
        tire_deflection = q[6] + positions[:, 1] * q[7] - positions[:, 0] * q[8] - jounce
        loads = tire_deflection_load(vehicle.planar.tire, tire_deflection, np.zeros(4), 0.0).load
        self.assertAlmostEqual(float(np.sum(loads)), vehicle.planar.total_mass * 9.80665, delta=1e-6)
        self.assertAlmostEqual(float(np.sum(loads[:2])), 0.45 * vehicle.planar.total_mass * 9.80665, delta=1e-6)

    def test_speed_equilibrium_includes_static_aero_load(self):
        vehicle = self.vehicle
        speed = 12.0
        # Nonzero-speed downforce compresses the preloaded springs; the zero-
        # jounce oracle applies only at the default (zero-speed) design point.
        state = solve_static_equilibrium(vehicle, speed=speed)
        q = state.as_vector()
        positions = sprung_wheel_positions(vehicle)
        deflection = q[6] + positions[:, 1] * q[7] - positions[:, 0] * q[8] - q[13:17]
        loads = tire_deflection_load(vehicle.planar.tire, deflection, np.zeros(4), 0.0).load
        downforce = 0.5 * vehicle.planar.aero.air_density * speed**2 * vehicle.planar.aero.downforce_area
        self.assertAlmostEqual(float(np.sum(loads)), vehicle.planar.total_mass * 9.80665 + downforce, delta=1e-6)
        self.assertAlmostEqual(
            float(np.sum(loads[:2])),
            0.45 * vehicle.planar.total_mass * 9.80665 + vehicle.planar.aero.aero_balance * downforce,
            delta=1e-6,
        )

    def _steady_speed_evaluation(self, speed):
        """Evaluate the vertical dynamics at the steady state at ``speed``.

        Straight motion (v_y = yaw rate = steering = 0), zero-data baseline toe,
        wheel speeds aligned to the loaded radii (zero slip ratio), and steady
        rolling tire states. The static equilibrium solver has no longitudinal
        tire force, so at speed drag leaves it slightly off the evaluator's own
        steady state: drag acts at the sprung CG, only ~21 mm above the total CG,
        so the refined state pitches a touch nose-up and moves a little load
        rearward. The state is refined on the evaluator's generalized forces, and
        both load vectors are returned: the static solver's and the refined one.
        """

        from simplified_vehicle_models.formulations.vertical_14dof import sprung_wheel_positions
        from tests.test_vertical_14dof import solve_vertical_steady_state

        vehicle = self.vehicle
        planar = vehicle.planar
        equilibrium = solve_static_equilibrium(vehicle, speed=speed)
        q = equilibrium.as_vector()
        positions = sprung_wheel_positions(vehicle)
        jounce = q[13:17]
        deflection = q[6] + positions[:, 1] * q[7] - positions[:, 0] * q[8] - jounce
        loads = tire_deflection_load(planar.tire, deflection, np.zeros(4), 0.0).load
        self.assertTrue(np.all(loads > 2.0))  # inside the full contact window
        static_loads = loads.copy()

        # Zero baseline toe at the equilibrium jounces: no jounce-steer offsets.
        # This is the input-level check on the helper; the evaluator-level gate
        # is test_zero_baseline_toe_is_inactive_in_the_evaluator_at_static_state.
        np.testing.assert_array_equal(corner_toe_angles(vehicle.suspension, jounce), np.zeros(4))

        # Align each wheel with its own loaded radius: slip ratio exactly zero.
        # The tire states depend on the load, so refine the vertical state and
        # rebuild the steady tire states together until both stop moving.
        state = VerticalState.from_vector(q)
        tires = tuple(
            steady_rolling_state(planar.tire, vehicle.transient_tire, 0.0, 0.0, float(load))
            for load in loads
        )
        refined_loads = loads
        refined_state = state
        for _ in range(8):
            state = solve_vertical_steady_state(vehicle, state, tires)
            refined = state.as_vector()
            positions = sprung_wheel_positions(vehicle)
            deflection = (refined[6] + positions[:, 1] * refined[7]
                          - positions[:, 0] * refined[8] - refined[13:17])
            refined_loads = tire_deflection_load(planar.tire, deflection,
                                                 np.zeros(4), 0.0).load
            refined[9:13] = speed / loaded_radius(planar.tire, refined_loads)
            state = VerticalState.from_vector(refined)
            refined_state = state
            previous = tires
            tires = tuple(
                steady_rolling_state(planar.tire, vehicle.transient_tire, 0.0, 0.0, float(load))
                for load in refined_loads
            )
            if max(abs(a.longitudinal_slip - b.longitudinal_slip)
                   for a, b in zip(tires, previous)) < 1e-12:
                break
        evaluation = evaluate_vertical_dynamics(
            vehicle, state, np.zeros(4), 0.0, tires
        )
        return evaluation, equilibrium, refined_state, static_loads, refined_loads

    def test_initialized_steady_tire_states_have_zero_rates_in_straight_motion(self):
        """Oracle: Task 6 static case, four steady-initialized tire states are stationary.

        At constant straight motion (v_y = yaw rate = steering = 0), the
        zero-data baseline toe, and wheel speeds aligned to the loaded radii
        (zero slip ratio), the lateral slip command is 0 and the longitudinal
        slip command is 0. The steady tire states are (0, 0, 0), so every
        relaxation rate must vanish. This checks only the tire-state rates;
        the vertical balance is covered by the evaluator consistency test
        below. Longitudinal rates are deliberately not asserted: at 8 m/s the
        drag and rolling resistance remain.
        """

        evaluation, _, _, _, refined_loads = self._steady_speed_evaluation(8.0)

        np.testing.assert_allclose(evaluation.normal_loads, refined_loads, rtol=0.0, atol=1e-9)
        self.assertFalse(np.any(evaluation.lifted))
        self.assertEqual(evaluation.tire_state_rates.shape, (12,))
        np.testing.assert_allclose(evaluation.tire_state_rates, np.zeros(12), rtol=0.0, atol=1e-12)

    def test_evaluator_vertical_rates_vanish_at_nonzero_speed_equilibrium(self):
        """The evaluator's own steady state at speed is stationary in all seven.

        Aerodynamic downforce is positive down, like heave, so it adds to
        gravity in the heave equation. At the refined 8 m/s steady state the
        heave, roll and pitch accelerations (rates[2:5]) and the four jounce
        accelerations (rates[17:21]) must vanish. Not all rates are asserted
        zero: drag and rolling resistance still act longitudinally and on the
        wheel speeds at 8 m/s.
        """

        speed = 8.0
        vehicle = self.vehicle
        evaluation, static_state, refined_state, static_loads, _ = self._steady_speed_evaluation(speed)

        np.testing.assert_allclose(evaluation.rates[2:5], np.zeros(3), rtol=0.0, atol=1e-6)
        np.testing.assert_allclose(evaluation.rates[17:21], np.zeros(4), rtol=0.0, atol=1e-6)

        # The static solver has no longitudinal tire force, so its state is not
        # the evaluator's steady state. The refinement is a small pitch and load
        # redistribution, not a change of total load: the vertical balance
        # Sigma N = m_total*g + downforce only depends on the speed.
        np.testing.assert_allclose(float(np.sum(evaluation.normal_loads)),
                                   float(np.sum(static_loads)), rtol=0.0, atol=1e-5)
        # The signed front-axle change matters: drag at the sprung CG, only
        # ~21 mm above the total CG, leaves a nose-up pitch, and the front axle
        # LOSES load to the rear (-0.83 N at 8 m/s, -1.87 N at 12 m/s). An
        # absolute bound would also accept a wrong-direction shift, so assert
        # the sign.
        gained = float(np.sum(evaluation.normal_loads[:2]) - np.sum(static_loads[:2]))
        self.assertLess(gained, -0.1)
        self.assertGreater(gained, -20.0)
        # Drag acts at the sprung CG above the ground, so the evaluator's steady
        # state pitches a little nose-up relative to the static solution.
        ax = float(evaluation.body_accelerations[0])
        self.assertLess(ax, 0.0)
        self.assertGreater(refined_state.pitch, static_state.pitch)
        self.assertLess(refined_state.pitch, 5e-3)

    def test_zero_baseline_toe_is_inactive_in_the_evaluator_at_static_state(self):
        """Oracle: at the static state the evaluator uses the pure steering angles.

        ``corner_toe_angles`` is only a helper and could agree with itself while
        the evaluator used something else, so the gate goes through
        ``VerticalEvaluation``. Two independent statements are checked.

        First, a copy of the vehicle whose front and rear jounce-steer curves are
        identically zero must evaluate bitwise identically. The compiled vehicle's
        jounce-steer data is zero anyway, so this control does not by itself
        prove the evaluator reads it; it does prove that no jounce-steer toe is
        being added at these jounces (a curve that was nonzero at a small
        compression would move the FL rates and break the equality).

        Second, the evaluator's commanded slip angle must equal the pure steering
        slip angle. The returned tire states are rebuilt from the contact
        kinematics of ``wheel_steering_angles(0.0, 4)`` alone, and the reported
        lateral-deformation rates vanish. The rate law is
        ``speed*(alpha_command - deformation/sigma_y)``, so a vanishing rate puts
        the commanded slip angle at the state's effective slip angle, which the
        rebuilt steady state has already fixed at the pure steering slip. The
        same test attaches a nonzero toe curve and confirms the rates do move, so
        the gate is not vacuous.
        """

        speed = 8.0
        evaluation, _, refined_state, _, refined_loads = self._steady_speed_evaluation(speed)
        vehicle = self.vehicle
        planar = vehicle.planar
        q = refined_state.as_vector()
        # The helper builds these from the same refined loads, so this rebuild is
        # exactly the set that produced ``evaluation``.
        tires = tuple(
            steady_rolling_state(planar.tire, vehicle.transient_tire, 0.0, 0.0, float(load))
            for load in refined_loads
        )
        np.testing.assert_array_equal(evaluation.normal_loads, refined_loads)

        def with_toe(toe):
            """Return the vehicle with a jounce-steer curve ramping to ``toe``.

            The curve spans the jounce limits, is zero at the zero-jounce knot
            (the model requires that) and reaches ``toe`` by the upper limit, so
            it is identically zero for ``toe = 0``.
            """

            def curve(axle):
                low, high = axle.jounce_limits
                return MonotoneSpline([low, 0.0, high], [0.0, 0.0, toe])

            suspension = dataclasses.replace(
                vehicle.suspension,
                front=dataclasses.replace(vehicle.suspension.front,
                                          jounce_steer=curve(vehicle.suspension.front)),
                rear=dataclasses.replace(vehicle.suspension.rear,
                                         jounce_steer=curve(vehicle.suspension.rear)),
            )
            return dataclasses.replace(vehicle, suspension=suspension)

        zero_toe = evaluate_vertical_dynamics(with_toe(0.0), refined_state, np.zeros(4), 0.0,
                                              tires)
        self.assertIsNone(zero_toe.mass_matrix)
        for name in ("rates", "forcing", "generalized_forces", "tire_state_rates",
                     "normal_loads", "tire_forces", "lifted", "body_accelerations"):
            with self.subTest(field=name):
                np.testing.assert_array_equal(getattr(zero_toe, name), getattr(evaluation, name))
        self.assertEqual(zero_toe.out_of_range_count, evaluation.out_of_range_count)
        for name in ("spring", "arb", "damper", "total", "damper_velocity"):
            with self.subTest(corner_force=name):
                np.testing.assert_array_equal(getattr(zero_toe.corner_forces, name),
                                              getattr(evaluation.corner_forces, name))

        # The evaluator's commanded slip is the pure steering slip: rebuild the
        # contact kinematics with the steering angles alone and check the
        # returned tire states are the steady rolling states of those slips.
        angles = wheel_steering_angles(0.0, 4)
        np.testing.assert_array_equal(angles, np.zeros(4))
        longitudinal, lateral = contact_velocities(
            PlanarState(refined_state.v_x, refined_state.v_y, refined_state.yaw_rate, np.zeros(4)),
            wheel_positions(planar),
            angles,
        )
        radii = loaded_radius(planar.tire, evaluation.normal_loads)
        slip_ratio, slip_angle = slip_quantities(q[9:13], radii, longitudinal, lateral, 1.0)
        np.testing.assert_allclose(slip_ratio, 0.0, rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(evaluation.tire_state_rates, np.zeros(12), rtol=0.0, atol=1e-12)
        for index, tire_state in enumerate(tires):
            expected = steady_rolling_state(
                planar.tire, vehicle.transient_tire, float(slip_angle[index]),
                float(slip_ratio[index]), float(evaluation.normal_loads[index]),
            )
            with self.subTest(wheel=index):
                self.assertAlmostEqual(tire_state.longitudinal_slip,
                                       expected.longitudinal_slip, delta=1e-12)
                self.assertAlmostEqual(tire_state.lateral_deformation,
                                       expected.lateral_deformation, delta=1e-12)
                self.assertAlmostEqual(tire_state.trail_angle, expected.trail_angle, delta=1e-12)

        # Non-vacuity: a genuinely nonzero toe curve does move the tire rates at
        # a state whose jounces put it well onto the curve.
        bumped = q.copy()
        bumped[13:17] += 5e-3
        perturbed = VerticalState.from_vector(bumped)
        inactive = evaluate_vertical_dynamics(with_toe(0.0), perturbed, np.zeros(4), 0.0, tires)
        active = evaluate_vertical_dynamics(with_toe(np.radians(0.5)), perturbed,
                                           np.zeros(4), 0.0, tires)
        moved = np.asarray(active.tire_state_rates) - np.asarray(inactive.tire_state_rates)
        self.assertGreater(float(np.max(np.abs(moved[[1, 4, 7, 10]]))), 1e-3)


GRAVITY = 9.80665

#: Tire stiffening of the rigid-tire reference vehicle. The loaded-radius
#: inverse blends a quadratic low-load ramp up to a fixed 0.5 mm deflection;
#: dividing ``r_2`` and ``r_3`` by more than about 470 turns the ramp's lower
#: knot positive and the law stops being a monotone spring. 300 is the largest
#: well-formed stiffening used here: about 1 % of the baseline tire roll
#: compliance survives (measured roll gradient 0.2916 deg/g at 15 m/s against
#: the 0.2897 deg/g closed form derived in the reference test below).
REFERENCE_TIRE_STIFFENING = 300.0


def _static_roll_centre(height: float) -> MonotoneSpline:
    """Return a constant roll-centre height spline over the measured roll range."""

    return MonotoneSpline([0.0, np.radians(5.0)], [height, height])


def _reference_rigid_tire_vehicle(vehicle):
    """Return the rigid-tire, static-roll-centre reference vehicle.

    Task 6's roll oracle (0.289 deg/g, front LLTD 0.473) is defined for a rigid
    tire and the static roll-centre heights 41.275 mm front / 57.15 mm rear. The
    reference case keeps every other part of the compiled vehicle, including the
    rear ARB and both axles' wheel-rate springs, and only replaces the tire
    loaded-radius coefficients and the measured roll-centre map.
    """

    tire = dataclasses.replace(
        vehicle.planar.tire,
        r_2=vehicle.planar.tire.r_2 / REFERENCE_TIRE_STIFFENING,
        r_3=vehicle.planar.tire.r_3 / REFERENCE_TIRE_STIFFENING,
    )
    planar = dataclasses.replace(vehicle.planar, tire=tire)
    suspension = dataclasses.replace(
        vehicle.suspension,
        front=dataclasses.replace(
            vehicle.suspension.front,
            roll_centre_height=_static_roll_centre(0.041275),
        ),
        rear=dataclasses.replace(
            vehicle.suspension.rear,
            roll_centre_height=_static_roll_centre(0.05715),
        ),
    )
    return dataclasses.replace(vehicle, planar=planar, suspension=suspension)


def _front_load_transfer_ratio(loads) -> float:
    """Return the front share of the total lateral load transfer."""

    front = float(loads[0] - loads[1])
    rear = float(loads[2] - loads[3])
    return front / (front + rear)


def _axle_roll_stiffness(vehicle, indices) -> float:
    """Return one axle's spring+ARB roll stiffness at the wheels [N*m/rad].

    Independent oracle: apply a pure roll jounce pattern ``z_i = y_i * eps`` and
    read the anti-symmetric spring force back from the public corner model.
    """

    eps = 1e-7
    positions = sprung_wheel_positions(vehicle)
    y = positions[:, 1]
    zero = corner_forces(
        vehicle.suspension, np.zeros(4), np.zeros(4), vehicle.dampers
    ).total
    pattern = np.zeros(4)
    pattern[indices] = y[indices] * eps
    loaded = corner_forces(vehicle.suspension, pattern, np.zeros(4), vehicle.dampers).total
    return float(np.sum((loaded - zero)[indices] * y[indices]) / eps)


class SteadyCornerTests(unittest.TestCase):
    """Oracle tests for ``solve_steady_corner`` (Task 6, the LLTD gate).

    The oracles are the static equilibrium solver, Newton's laws for the whole
    vehicle (load sum and lateral load-transfer moment), mirrored-vehicle
    symmetry, an independent recomputation of the tire slips, and the plan's
    reference roll numbers (D4-D7) for the rigid-tire reference vehicle. The
    production vehicle's compliant-tire roll gradient and LLTD are reported in
    the task report and are deliberately not asserted here.
    """

    @classmethod
    def setUpClass(cls):
        cls.vehicle = compile_formulation(str(FORMULATION)).model
        cls.speeds = (12.0, 15.0, 25.0)
        cls.one_g = {
            speed: _solve_steady_corner_detail(cls.vehicle, speed, GRAVITY)
            for speed in cls.speeds
        }
        cls.minus_one_g = {
            speed: _solve_steady_corner_detail(cls.vehicle, speed, -GRAVITY)
            for speed in cls.speeds
        }
        cls.straight = _solve_steady_corner_detail(cls.vehicle, 15.0, 0.0)
        cls.reference = _reference_rigid_tire_vehicle(cls.vehicle)
        cls.reference_one_g = {
            speed: _solve_steady_corner_detail(cls.reference, speed, GRAVITY)
            for speed in cls.speeds
        }

    def test_straight_solve_reduces_to_static_equilibrium(self):
        """Oracle: with no longitudinal force, ay = 0 is the static solve.

        The settled evaluator carries the drag and thrust longitudinal load
        transfer, so the production straight solve is *not* the static solution:
        the drive force squats the rear and moves load forward (measured at
        15 m/s: pitch +6.6e-4 rad, jounces up to 2.8e-4 m, heave +8.3e-6 m,
        front axle load -44.5 N against ``solve_static_equilibrium``). Removing
        the aero drag removes that whole coupling: no wheel carries any
        longitudinal force, the driven torque is exactly zero, and the
        evaluator's vertical block becomes the static system. The two
        independent solvers must then agree to 1e-6 at every speed.
        """

        planar = self.vehicle.planar
        glide = dataclasses.replace(
            self.vehicle,
            planar=dataclasses.replace(
                planar, aero=dataclasses.replace(planar.aero, drag_area=0.0)
            ),
        )
        for speed in (5.0, 15.0, 25.0):
            equilibrium = solve_static_equilibrium(glide, speed).as_vector()
            detail = _solve_steady_corner_detail(glide, speed, 0.0)
            state = detail.state.as_vector()
            self.assertAlmostEqual(detail.driven_torque, 0.0, delta=1e-9)
            np.testing.assert_allclose(state[6:9], equilibrium[6:9], rtol=0.0, atol=1e-6)
            np.testing.assert_allclose(
                state[13:17], equilibrium[13:17], rtol=0.0, atol=1e-6
            )

    def test_production_straight_solve_balance_and_thrust_shift(self):
        """Oracle: Newton's laws at the straight solve, with the thrust shift.

        The normal loads must still sum to gravity plus downforce. The drive
        force at the contact patches is below the sprung CG, so the straight
        solve sits a documented amount away from the static solve: the band
        below is 2e-3 (rad or m) against the measured 6.6e-4 rad pitch and
        2.8e-4 m jounce shifts at 15 m/s, and the front axle load drops because
        the thrust line pushes load rearward.
        """

        speed = 15.0
        detail = self.straight
        state = detail.state
        loads = detail.evaluation.normal_loads
        downforce, drag = aerodynamic_forces(self.vehicle.planar.aero, speed)
        self.assertAlmostEqual(state.yaw_rate, 0.0, delta=1e-12)
        self.assertAlmostEqual(state.v_y, 0.0, delta=1e-12)
        self.assertAlmostEqual(detail.steering, 0.0, delta=1e-12)
        self.assertAlmostEqual(
            float(np.sum(loads)),
            self.vehicle.planar.total_mass * GRAVITY + downforce,
            delta=1e-6,
        )
        # The driven wheels alone carry the drag: T balances the drag and the
        # tire rolling-resistance moment at zero-ish slip.
        self.assertAlmostEqual(
            float(np.sum(detail.evaluation.tire_forces[:, 0])), drag, delta=1e-6
        )
        self.assertGreater(detail.driven_torque, 0.0)
        self.assertAlmostEqual(
            detail.driven_torque,
            0.5 * drag * self.vehicle.planar.tire.r_1,
            delta=0.1 * 0.5 * drag * self.vehicle.planar.tire.r_1,
        )
        static_state = solve_static_equilibrium(self.vehicle, speed).as_vector()
        np.testing.assert_allclose(
            state.as_vector()[[6, 7, 8]], static_state[[6, 7, 8]], rtol=0.0, atol=2e-3
        )
        np.testing.assert_allclose(
            state.as_vector()[13:17], static_state[13:17], rtol=0.0, atol=2e-3
        )
        static_loads = tire_deflection_load(
            self.vehicle.planar.tire,
            static_state[6]
            + sprung_wheel_positions(self.vehicle)[:, 1] * static_state[7]
            - sprung_wheel_positions(self.vehicle)[:, 0] * static_state[8]
            - static_state[13:17],
            np.zeros(4),
            0.0,
        ).load
        self.assertLess(float(np.sum(loads[:2])), float(np.sum(static_loads[:2])))

    def test_one_g_and_minus_one_g_converge_and_mirror(self):
        """Oracle: convergence gates plus mirrored-vehicle symmetry.

        A mirrored vehicle (ay -> -ay) must give the same solution with left
        and right wheels swapped, steer and sideslip negated, roll negated and
        pitch and heave unchanged. The 1e-8 tolerance is the design's mirror
        gate; the measured mismatch is about 1e-14.
        """

        for speed in self.speeds:
            for detail in (self.one_g[speed], self.minus_one_g[speed]):
                self.assertLess(detail.residual_inf, 1e-6)
                self.assertEqual(detail.evaluation.out_of_range_count, 0)
                self.assertFalse(bool(np.any(detail.evaluation.lifted)))
                self.assertLess(
                    float(np.max(np.abs(detail.evaluation.tire_state_rates))), 1e-8
                )
                non_wheel = np.asarray(detail.evaluation.rates)[
                    [0, 1, 2, 3, 4, 5, 17, 18, 19, 20]
                ]
                np.testing.assert_allclose(non_wheel, np.zeros(10), rtol=0.0, atol=1e-6)

            positive = self.one_g[speed].state.as_vector()
            negative = self.minus_one_g[speed].state.as_vector()
            mirrored = np.array([
                negative[0], -negative[1], negative[2], -negative[3],
                negative[4], -negative[5], negative[6], -negative[7], negative[8],
                negative[10], negative[9], negative[12], negative[11],
                negative[14], negative[13], negative[16], negative[15],
                negative[18], negative[17], negative[20], negative[19],
            ])
            np.testing.assert_allclose(positive, mirrored, rtol=0.0, atol=1e-8)
            self.assertAlmostEqual(
                self.one_g[speed].steering, -self.minus_one_g[speed].steering, delta=1e-8
            )
            self.assertAlmostEqual(
                self.one_g[speed].driven_torque,
                self.minus_one_g[speed].driven_torque,
                delta=1e-8,
            )

    def test_positive_lateral_acceleration_signs_and_load_transfer(self):
        """Oracle: SAE signs. +ay (right turn) loads the left tires and rolls left.

        Positive SAE roll is right-side-down, so a right turn gives negative
        roll, more load on FL and RL than FR and RR, a positive (right) steering
        angle, and a front load-transfer distribution inside the physical band.
        """

        for speed in self.speeds:
            detail = self.one_g[speed]
            loads = detail.evaluation.normal_loads
            self.assertLess(detail.state.roll, 0.0)
            self.assertGreater(float(loads[0]), float(loads[1]))
            self.assertGreater(float(loads[2]), float(loads[3]))
            self.assertGreater(detail.steering, 0.0)
            ratio = _front_load_transfer_ratio(loads)
            self.assertGreater(ratio, 0.3)
            self.assertLess(ratio, 0.7)
            sideslip = float(np.arctan2(detail.state.v_y, detail.state.v_x))
            self.assertAlmostEqual(
                float(detail.evaluation.body_accelerations[1]),
                GRAVITY * np.cos(sideslip),
                delta=1e-6,
            )

    def test_whole_vehicle_lateral_moment_identity(self):
        """Oracle: Sigma y_i * N_i = -m_total * h_cg * ay, in body axes.

        Taking moments about the ground contact line for the whole steady
        vehicle: the lateral inertia of the sprung mass at h_s and of the four
        unsprung masses at r_1 give exactly -m_total*h_cg*ay. It holds only with
        the body-axis lateral acceleration: the path target is V*r, and using it
        instead leaves m_total*h_cg*ay*(1-cos beta), measured 0.15 N*m at
        12 m/s (beta = 1.09 deg). The tolerance below is 1e-6 N*m; the measured
        residual is up to 2.8e-8 N*m at 25 m/s, set by the solver's scaled roll
        residual (3e-11) times m_s*g*half_track = 163 N*m.
        """

        y = wheel_positions(self.vehicle.planar)[:, 1]
        for speed in self.speeds:
            for detail in (self.one_g[speed], self.minus_one_g[speed]):
                ay = float(detail.evaluation.body_accelerations[1])
                moment = float(np.sum(y * detail.evaluation.normal_loads))
                expected = -self.vehicle.planar.total_mass * self.vehicle.planar.cg_height * ay
                self.assertAlmostEqual(moment, expected, delta=1e-6)

    def test_tire_states_are_steady_and_the_slips_recompute(self):
        """Oracle: the returned tire states are steady at independently derived slips.

        The transient tire is in equilibrium only when its state is the steady
        rolling state of the slips the evaluator re-derives. This test rebuilds
        the contact kinematics, slip ratios and slip angles from the public
        position/steering/slip functions and the returned state, then compares
        the solver's tire states with ``steady_rolling_state`` at those slips.
        """

        planar = self.vehicle.planar
        positions = wheel_positions(planar)
        for speed in self.speeds:
            detail = self.one_g[speed]
            state = detail.state
            evaluation = detail.evaluation
            np.testing.assert_allclose(
                evaluation.tire_state_rates, np.zeros(12), rtol=0.0, atol=1e-8
            )
            self.assertEqual(evaluation.out_of_range_count, 0)
            self.assertFalse(bool(np.any(evaluation.corner_forces.out_of_range)))
            self.assertFalse(bool(np.any(evaluation.lifted)))
            angles = wheel_steering_angles(detail.steering, 4) + corner_toe_angles(
                self.vehicle.suspension, state.as_vector()[13:17]
            )
            longitudinal, lateral = contact_velocities(
                PlanarState(state.v_x, state.v_y, state.yaw_rate, np.zeros(4)),
                positions,
                angles,
            )
            radii = loaded_radius(planar.tire, evaluation.normal_loads)
            slip_ratio, slip_angle = slip_quantities(
                state.as_vector()[9:13], radii, longitudinal, lateral, 1.0
            )
            for index, tire_state in enumerate(detail.tire_states):
                expected = steady_rolling_state(
                    planar.tire,
                    self.vehicle.transient_tire,
                    float(slip_angle[index]),
                    float(slip_ratio[index]),
                    float(evaluation.normal_loads[index]),
                )
                self.assertAlmostEqual(
                    tire_state.longitudinal_slip, expected.longitudinal_slip, delta=1e-12
                )
                self.assertAlmostEqual(
                    tire_state.lateral_deformation, expected.lateral_deformation, delta=1e-12
                )
                self.assertAlmostEqual(
                    tire_state.trail_angle, expected.trail_angle, delta=1e-12
                )

    def test_direct_jump_matches_continuation(self):
        """Oracle: the 0.1 g continuation path reaches the same root as a jump."""

        for speed, target in ((15.0, GRAVITY), (12.0, -GRAVITY)):
            direct = _solve_steady_corner_detail(
                self.vehicle, speed, target, continuation_step=None
            )
            stepped = self.one_g[speed] if target > 0.0 else self.minus_one_g[speed]
            self.assertLess(direct.residual_inf, 1e-6)
            self.assertEqual(direct.continuation_steps, 1)
            self.assertEqual(stepped.continuation_steps, 10)
            np.testing.assert_allclose(
                direct.state.as_vector(), stepped.state.as_vector(), rtol=0.0, atol=1e-8
            )

    def test_input_validation_and_unreachable_steering(self):
        """Oracle: ValueError for bad inputs, RuntimeError past the steering lock.

        At 3 m/s a 1 g corner needs about 2.9 rad of road-wheel angle, far past
        the vehicle's 0.524 rad steering maximum, so no admissible steady corner
        exists and the solver must raise instead of returning a partial state.
        """

        with self.assertRaises(ValueError):
            solve_steady_corner(self.vehicle, 2.9, 0.0)
        with self.assertRaises(ValueError):
            solve_steady_corner(self.vehicle, float("nan"), GRAVITY)
        with self.assertRaises(ValueError):
            solve_steady_corner(self.vehicle, 15.0, float("inf"))
        with self.assertRaises(RuntimeError):
            _solve_steady_corner_detail(self.vehicle, 3.0, GRAVITY)

    def test_three_g_target_raises(self):
        """Oracle: the tire-limited feasible region ends before 3 g everywhere.

        At 25 m/s the 0.1 g continuation reaches about 2.09 g and then fails to
        converge even after halving the step to 0.005 g, so a 3 g request must
        raise RuntimeError rather than return a partial solution.
        """

        with self.assertRaises(RuntimeError):
            solve_steady_corner(self.vehicle, 25.0, 3.0 * GRAVITY)

    def test_reference_roll_stiffness_matches_source_numbers(self):
        """Oracle: the plan's D4-D7 roll stiffnesses and closed-form roll gradient.

        Finite-differencing the public corner model with a pure roll jounce
        pattern must give 1057 N*m/deg front and 1104 N*m/deg rear, the source
        spreadsheet numbers. The rigid-tire roll gradient then follows in closed
        form from the sprung mass, the sprung CG height, the roll-stiffness
        weighted roll-centre height and the total wheel-rate roll stiffness.
        """

        vehicle = self.vehicle
        front = _axle_roll_stiffness(vehicle, [0, 1])
        rear = _axle_roll_stiffness(vehicle, [2, 3])
        self.assertAlmostEqual(front / np.degrees(1.0), 1057.4, delta=1.0)
        self.assertAlmostEqual(rear / np.degrees(1.0), 1104.4, delta=1.0)
        total = front + rear
        roll_centre = (front * 0.041275 + rear * 0.05715) / total
        gradient = np.degrees(
            vehicle.sprung_mass
            * (vehicle.sprung_cg_height - roll_centre)
            / total
            * GRAVITY
        )
        self.assertAlmostEqual(gradient, 0.289, delta=0.006)

    def test_reference_rigid_tire_roll_gradient_and_lltd(self):
        """Oracle: the rigid-tire/static-RC reference case (0.289 deg/g, LLTD 0.473).

        The reference vehicle is built by ``dataclasses.replace`` from the
        compiled vehicle: stiff tire loaded-radius coefficients (300x) and the
        constant static roll-centre heights 41.275 mm front / 57.15 mm rear,
        keeping the rear ARB. The plan's reference numbers are a roll gradient
        of 0.289 deg/g (+/-10 %) and a front LLTD of 0.473 (+/-0.02). The
        compliant-tire production numbers are reported separately in the task
        report and are not used as oracles here.
        """

        front = _axle_roll_stiffness(self.reference, [0, 1])
        rear = _axle_roll_stiffness(self.reference, [2, 3])
        total = front + rear
        roll_centre = (front * 0.041275 + rear * 0.05715) / total
        phi = (
            self.reference.sprung_mass
            * (self.reference.sprung_cg_height - roll_centre)
            * GRAVITY
            / total
        )
        half_track = self.reference.planar.half_track
        elastic = front * phi / half_track
        geometric = 0.041275 * self.reference.sprung_mass * self.reference.sprung_front_fraction * GRAVITY / half_track
        unsprung = (
            2.0
            * self.reference.unsprung_mass_front
            * GRAVITY
            * self.reference.unsprung_cg_height
            / half_track
        )
        transfer = (
            self.reference.planar.total_mass
            * self.reference.planar.cg_height
            * GRAVITY
            / half_track
        )
        closed_form_lltd = (elastic + geometric + unsprung) / transfer
        self.assertAlmostEqual(closed_form_lltd, 0.473, delta=0.01)
        for speed in self.speeds:
            detail = self.reference_one_g[speed]
            gradient = -np.degrees(detail.state.roll)
            self.assertLess(gradient, 1.1 * 0.289)
            self.assertGreater(gradient, 0.9 * 0.289)
            ratio = _front_load_transfer_ratio(detail.evaluation.normal_loads)
            self.assertAlmostEqual(ratio, 0.473, delta=0.02)
            self.assertAlmostEqual(ratio, closed_form_lltd, delta=0.02)

if __name__ == "__main__":
    unittest.main()
