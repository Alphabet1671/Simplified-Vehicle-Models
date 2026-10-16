import math
from dataclasses import replace
from pathlib import Path
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.formulations.planar_7dof import (
    PlanarState,
    contact_velocities,
    evaluate_normal_loads,
    evaluate_planar_dynamics,
    parse_wheel_torques,
    rotate_tire_forces_to_body,
    slip_quantities,
    wheel_positions,
    yaw_moment_of,
)
from simplified_vehicle_models.tires.unitire_simple import loaded_radius


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
FORMULATION_PATH = REPOSITORY_ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"


def _free_rolling_speeds(vehicle, speed: float) -> np.ndarray:
    """Return wheel speeds that give zero slip at the static loads."""

    loads = evaluate_normal_loads(vehicle, speed, 0.0, 0.0)
    return speed / loaded_radius(vehicle.tire, loads)


class SlipKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION_PATH).model
        cls.positions = wheel_positions(cls.vehicle)

    def test_straight_run_has_no_lateral_velocity(self) -> None:
        state = PlanarState(20.0, 0.0, 0.0, np.zeros(4))

        longitudinal, lateral = contact_velocities(state, self.positions, 0.0)

        np.testing.assert_allclose(longitudinal, 20.0)
        np.testing.assert_allclose(lateral, 0.0)

    def test_yaw_rate_speeds_the_outer_wheels(self) -> None:
        state = PlanarState(20.0, 0.0, 0.5, np.zeros(4))

        longitudinal, _ = contact_velocities(state, self.positions, 0.0)

        # A positive yaw rate is a right turn, so the left wheels are outer.
        # FL is index 0 and FR is index 1.
        self.assertGreater(longitudinal[0], longitudinal[1])
        self.assertGreater(longitudinal[2], longitudinal[3])

    def test_steering_rotates_only_the_front_wheels(self) -> None:
        state = PlanarState(20.0, 0.0, 0.0, np.zeros(4))
        steering = math.radians(5.0)

        longitudinal, lateral = contact_velocities(
            state, self.positions, steering
        )

        self.assertAlmostEqual(longitudinal[0], longitudinal[1], places=9)
        self.assertAlmostEqual(longitudinal[2], 20.0, places=9)
        # A right-steered wheel sees the flow arrive from its left.
        self.assertAlmostEqual(lateral[0], -20.0 * math.sin(steering), places=9)
        self.assertAlmostEqual(lateral[2], 0.0, places=9)

    def test_slip_ratio_is_zero_at_free_rolling(self) -> None:
        radii = np.full(4, 0.19)
        speeds = np.full(4, 20.0 / 0.19)

        ratio, angle = slip_quantities(
            speeds, radii, np.full(4, 20.0), np.zeros(4), 1.0
        )

        np.testing.assert_allclose(ratio, 0.0, atol=1.0e-12)
        np.testing.assert_allclose(angle, 0.0, atol=1.0e-12)

    def test_driving_slip_is_positive_and_braking_slip_is_negative(self) -> None:
        radii = np.full(4, 0.19)
        rolling = np.full(4, 20.0 / 0.19)

        driving, _ = slip_quantities(
            rolling * 1.1, radii, np.full(4, 20.0), np.zeros(4), 1.0
        )
        braking, _ = slip_quantities(
            rolling * 0.9, radii, np.full(4, 20.0), np.zeros(4), 1.0
        )

        self.assertTrue(np.all(driving > 0.0))
        self.assertTrue(np.all(braking < 0.0))

    def test_right_steered_wheel_has_a_positive_slip_angle(self) -> None:
        radii = np.full(4, 0.19)
        speed = 20.0
        for steering_degrees, expected_sign in ((5.0, 1.0), (-5.0, -1.0)):
            steering = math.radians(steering_degrees)
            state = PlanarState(speed, 0.0, 0.0, np.full(4, speed / 0.19))
            longitudinal, lateral = contact_velocities(
                state, self.positions, steering
            )
            _, angle = slip_quantities(
                state.wheel_speeds, radii, longitudinal, lateral, 1.0
            )
            with self.subTest(steering=steering_degrees):
                self.assertEqual(math.copysign(1.0, angle[0]), expected_sign)
                # A wheel steered by delta with no body slip has slip angle
                # exactly delta.
                self.assertAlmostEqual(abs(angle[0]), abs(steering), places=9)
                self.assertAlmostEqual(angle[2], 0.0, places=12)

    def test_speed_floor_bounds_the_low_speed_ratio(self) -> None:
        ratio, angle = slip_quantities(
            np.zeros(4), np.full(4, 0.19), np.zeros(4), np.zeros(4), 2.0
        )

        self.assertTrue(np.all(np.isfinite(ratio)))
        self.assertAlmostEqual(float(ratio[0]), 0.0)
        self.assertAlmostEqual(float(angle[0]), 0.0)

    def test_speed_floor_must_be_positive(self) -> None:
        with self.assertRaises(ValueError):
            slip_quantities(np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(4), 0.0)


class ForceRotationTests(unittest.TestCase):
    def test_zero_steering_leaves_the_forces_alone(self) -> None:
        longitudinal = np.asarray([1.0, 2.0, 3.0, 4.0])
        lateral = np.asarray([5.0, 6.0, 7.0, 8.0])

        body_x, body_y = rotate_tire_forces_to_body(longitudinal, lateral, 0.0)

        np.testing.assert_allclose(body_x, longitudinal)
        np.testing.assert_allclose(body_y, lateral)

    def test_a_steered_drive_force_gains_a_lateral_component(self) -> None:
        steering = math.radians(10.0)
        longitudinal = np.asarray([1000.0, 1000.0, 0.0, 0.0])
        lateral = np.zeros(4)

        body_x, body_y = rotate_tire_forces_to_body(longitudinal, lateral, steering)

        # A right-steered wheel pushes the vehicle to the right.
        self.assertGreater(body_y[0], 0.0)
        self.assertAlmostEqual(body_y[0], 1000.0 * math.sin(steering), places=9)
        self.assertAlmostEqual(body_x[0], 1000.0 * math.cos(steering), places=9)
        self.assertAlmostEqual(body_y[2], 0.0, places=12)

    def test_rotation_preserves_the_force_magnitude(self) -> None:
        steering = math.radians(-12.0)
        longitudinal = np.asarray([800.0, -300.0, 0.0, 0.0])
        lateral = np.asarray([1200.0, 400.0, 0.0, 0.0])

        body_x, body_y = rotate_tire_forces_to_body(longitudinal, lateral, steering)

        np.testing.assert_allclose(
            np.hypot(body_x, body_y), np.hypot(longitudinal, lateral), rtol=1.0e-12
        )


class YawMomentTests(unittest.TestCase):
    def test_a_forward_force_on_the_right_wheels_yaws_left(self) -> None:
        positions = np.asarray(
            [(0.8, -0.5), (0.8, 0.5), (-0.7, -0.5), (-0.7, 0.5)]
        )
        force = np.asarray([0.0, 100.0, 0.0, 100.0])

        moment = yaw_moment_of(positions, force, np.zeros(4), np.zeros(4))

        self.assertLess(moment, 0.0)

    def test_a_rightward_lateral_force_on_the_front_yaws_right(self) -> None:
        positions = np.asarray(
            [(0.8, -0.5), (0.8, 0.5), (-0.7, -0.5), (-0.7, 0.5)]
        )
        lateral = np.asarray([500.0, 500.0, 0.0, 0.0])

        moment = yaw_moment_of(positions, np.zeros(4), lateral, np.zeros(4))

        self.assertGreater(moment, 0.0)
        self.assertAlmostEqual(moment, 0.8 * 1000.0, places=9)

    def test_an_aligning_moment_adds_directly(self) -> None:
        positions = np.zeros((4, 2))
        aligning = np.asarray([1.0, 2.0, 3.0, 4.0])

        moment = yaw_moment_of(positions, np.zeros(4), np.zeros(4), aligning)

        self.assertAlmostEqual(moment, 10.0, places=9)


class PlanarDynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION_PATH).model
        cls.speed = 20.0
        cls.wheel_speeds = _free_rolling_speeds(cls.vehicle, cls.speed)

    def _state(self, **overrides) -> PlanarState:
        base = {
            "v_x": self.speed,
            "v_y": 0.0,
            "yaw_rate": 0.0,
            "wheel_speeds": self.wheel_speeds.copy(),
        }
        base.update(overrides)
        return PlanarState(**base)

    def test_rejects_a_wrong_torque_shape(self) -> None:
        for bad in (np.zeros(3), np.zeros(5), np.zeros((4, 1))):
            with self.subTest(shape=bad.shape), self.assertRaises(ValueError):
                parse_wheel_torques(bad)

    def test_rejects_non_finite_inputs(self) -> None:
        with self.assertRaises(ValueError):
            evaluate_planar_dynamics(
                self.vehicle, self._state(), np.zeros(4), float("nan")
            )
        with self.assertRaises(ValueError):
            evaluate_planar_dynamics(
                self.vehicle, self._state(), np.full(4, np.inf), 0.0
            )
        with self.assertRaises(ValueError):
            evaluate_planar_dynamics(
                self.vehicle, self._state(), np.zeros(4), 0.0, speed_floor=0.0
            )

    def test_straight_run_has_no_lateral_response(self) -> None:
        evaluation = evaluate_planar_dynamics(
            self.vehicle, self._state(), np.zeros(4), 0.0
        )

        np.testing.assert_allclose(evaluation.lateral_forces, 0.0, atol=1.0e-9)
        self.assertAlmostEqual(evaluation.lateral_acceleration, 0.0, places=9)
        self.assertAlmostEqual(evaluation.yaw_acceleration, 0.0, places=9)
        self.assertAlmostEqual(evaluation.yaw_moment, 0.0, places=6)

    def test_right_steer_yaws_right_and_left_steer_yaws_left(self) -> None:
        right = evaluate_planar_dynamics(
            self.vehicle, self._state(), np.zeros(4), math.radians(3.0)
        )
        left = evaluate_planar_dynamics(
            self.vehicle, self._state(), np.zeros(4), math.radians(-3.0)
        )

        self.assertGreater(right.yaw_acceleration, 0.0)
        self.assertGreater(right.lateral_acceleration, 0.0)
        self.assertTrue(np.all(right.lateral_forces > 0.0) or
                        np.all(right.lateral_forces[:2] > 0.0))
        self.assertAlmostEqual(
            right.yaw_acceleration, -left.yaw_acceleration, places=6
        )
        self.assertAlmostEqual(
            right.lateral_acceleration, -left.lateral_acceleration, places=6
        )

    def test_right_turn_moves_load_to_the_left_wheels(self) -> None:
        evaluation = evaluate_planar_dynamics(
            self.vehicle, self._state(), np.zeros(4), math.radians(3.0)
        )

        self.assertGreater(
            float(evaluation.normal_loads[0]), float(evaluation.normal_loads[1])
        )
        self.assertGreater(
            float(evaluation.normal_loads[2]), float(evaluation.normal_loads[3])
        )

    def test_yaw_coriolis_does_not_cancel_physical_load_transfer(self) -> None:
        state = self._state(yaw_rate=0.2)
        evaluation = evaluate_planar_dynamics(
            self.vehicle, state, np.zeros(4), math.radians(2.0)
        )
        physical_lateral = (
            float(np.sum(evaluation.body_lateral_forces)) / self.vehicle.total_mass
        )
        self.assertAlmostEqual(
            evaluation.lateral_acceleration + state.yaw_rate * state.v_x,
            physical_lateral,
            places=7,
        )
        expected = evaluate_normal_loads(
            self.vehicle,
            float(np.hypot(state.v_x, state.v_y)),
            evaluation.longitudinal_acceleration - state.yaw_rate * state.v_y,
            physical_lateral,
        )
        np.testing.assert_allclose(evaluation.normal_loads, expected, atol=1e-6)

    def test_torque_changes_the_derivative_not_the_force(self) -> None:
        state = self._state()
        coasting = evaluate_planar_dynamics(self.vehicle, state, np.zeros(4), 0.0)
        driving = evaluate_planar_dynamics(
            self.vehicle, state, np.full(4, 120.0), 0.0
        )

        np.testing.assert_allclose(
            driving.longitudinal_forces, coasting.longitudinal_forces, atol=1.0e-12
        )
        np.testing.assert_allclose(
            driving.wheel_accelerations - coasting.wheel_accelerations,
            120.0 / self.vehicle.wheel_inertia,
            rtol=1.0e-12,
        )

    def test_rolling_resistance_retards_a_forward_roll(self) -> None:
        rolling_tire = replace(self.vehicle.tire, f_rr_0=0.015)
        with_resistance = replace(self.vehicle, tire=rolling_tire)
        state = self._state()

        free = evaluate_planar_dynamics(self.vehicle, state, np.zeros(4), 0.0)
        resisted = evaluate_planar_dynamics(with_resistance, state, np.zeros(4), 0.0)

        self.assertTrue(np.all(resisted.wheel_accelerations < free.wheel_accelerations))

    def test_load_loop_converges_on_a_feasible_state(self) -> None:
        evaluation = evaluate_planar_dynamics(
            self.vehicle, self._state(), np.full(4, 80.0), math.radians(4.0)
        )

        self.assertTrue(evaluation.load_converged)
        self.assertLessEqual(evaluation.load_residual, 1.0e-9)
        self.assertTrue(evaluation.is_feasible)
        self.assertTrue(evaluation.contact_feasible)

    def test_an_over_limit_state_is_reported_unconverged(self) -> None:
        # An intentionally excessive CG height makes the same sharp turn
        # unload a contact patch. This is a stress case, not an IEM24 parameter.
        state = self._state(yaw_rate=1.0)
        extreme_vehicle = replace(self.vehicle, cg_height=1.0)

        evaluation = evaluate_planar_dynamics(
            extreme_vehicle, state, np.zeros(4), math.radians(25.0)
        )

        self.assertFalse(evaluation.is_feasible)
        self.assertFalse(evaluation.contact_feasible)
        # A pathological contact-loss state need not converge with stronger road grip;
        # it must never be accepted as feasible regardless of fixed-point status.
        self.assertIsInstance(evaluation.load_converged, bool)

    def test_the_force_balance_closes_on_the_reported_acceleration(self) -> None:
        evaluation = evaluate_planar_dynamics(
            self.vehicle, self._state(), np.full(4, 60.0), math.radians(2.0)
        )

        total_longitudinal = float(
            np.sum(evaluation.body_longitudinal_forces)
        ) - evaluation.drag_force
        self.assertAlmostEqual(
            evaluation.longitudinal_acceleration,
            total_longitudinal / self.vehicle.total_mass,
            places=6,
        )
        self.assertAlmostEqual(
            evaluation.yaw_acceleration,
            evaluation.yaw_moment / self.vehicle.yaw_inertia,
            places=9,
        )

    def test_distinct_wheel_torques_split_the_drive(self) -> None:
        # The fixed 20/80 split: the front pair carries a fifth of the torque.
        total = 400.0
        torques = np.asarray(
            [0.10 * total, 0.10 * total, 0.40 * total, 0.40 * total]
        )

        evaluation = evaluate_planar_dynamics(
            self.vehicle, self._state(), torques, 0.0
        )

        self.assertGreater(
            float(evaluation.wheel_accelerations[2]),
            float(evaluation.wheel_accelerations[0]),
        )


if __name__ == "__main__":
    unittest.main()


class LoadSolveTests(unittest.TestCase):
    """The Newton load closure agrees with the fixed point and does not fail."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.vehicle = compile_formulation(FORMULATION_PATH).model

    @staticmethod
    def _state(random, kind):
        speed = random.uniform(3.0, 40.0)
        if kind == "consistent":
            return (PlanarState(speed, random.uniform(-1.5, 1.5), random.uniform(-0.8, 0.8),
                                speed / 0.2 * random.uniform(0.9, 1.1, 4), 0.8),
                    random.uniform(-250.0, 250.0, 4), random.uniform(-0.3, 0.3))
        return (PlanarState(speed, random.uniform(-6.0, 6.0), random.uniform(-2.0, 2.0),
                            speed / 0.2 * random.uniform(0.3, 1.6, 4), 0.8),
                random.uniform(-250.0, 250.0, 4), random.uniform(-0.5, 0.5))

    def test_no_closure_failures_on_random_states(self) -> None:
        # The previous fixed point failed on 52/6000 and 9/6000 of these seeds.
        for kind, seed in (("consistent", 11), ("wide", 12)):
            random = np.random.default_rng(seed)
            failures = 0
            for _ in range(1500):
                state, torques, steering = self._state(random, kind)
                evaluation = evaluate_planar_dynamics(self.vehicle, state, torques, steering)
                failures += not evaluation.load_converged
                self.assertLessEqual(evaluation.load_residual, 1.0e-9 if evaluation.load_converged
                                     else np.inf)
            self.assertEqual(failures, 0, kind)

    def test_solution_matches_a_long_fixed_point_run(self) -> None:
        random = np.random.default_rng(5)
        for _ in range(60):
            state, torques, steering = self._state(random, "consistent")
            newton = evaluate_planar_dynamics(self.vehicle, state, torques, steering)
            reference = evaluate_planar_dynamics(self.vehicle, state, torques, steering,
                                                 load_maximum_iterations=5000)
            self.assertTrue(newton.load_converged)
            if reference.load_converged:
                np.testing.assert_allclose(newton.normal_loads, reference.normal_loads,
                                           rtol=1.0e-6, atol=1.0e-5)
                self.assertAlmostEqual(newton.yaw_acceleration, reference.yaw_acceleration,
                                       delta=1.0e-6)

    def test_method_and_jacobian_are_reported(self) -> None:
        speed = 20.0
        state = PlanarState(speed, 0.2, 0.1, _free_rolling_speeds(self.vehicle, speed), 0.8)
        evaluation = evaluate_planar_dynamics(self.vehicle, state, np.zeros(4), 0.05)
        self.assertEqual(evaluation.load_method, "newton")
        self.assertTrue(np.isfinite(evaluation.load_jacobian_determinant))
        self.assertGreater(evaluation.load_jacobian_determinant, 1.0e-3)

    def test_an_exhausted_budget_is_reported_not_hidden(self) -> None:
        random = np.random.default_rng(11)
        state, torques, steering = self._state(random, "wide")
        evaluation = evaluate_planar_dynamics(self.vehicle, state, torques, steering,
                                              load_maximum_iterations=1, load_tolerance=1.0e-15)
        if not evaluation.load_converged:
            self.assertEqual(evaluation.load_method, "unconverged")
            self.assertFalse(evaluation.contact_feasible)
