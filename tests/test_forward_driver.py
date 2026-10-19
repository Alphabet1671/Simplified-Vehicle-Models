"""Check the shared forward-driver numerical contract."""
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.forward_driver import (
    RateSystem, DriverObservation, PreviewDriver, build_speed_plan, solve_forward,
)


class RateSystemTests(unittest.TestCase):
    def test_requested_wheel_inertia(self):
        vehicle = compile_formulation(
            "models/formulations/iem24_murphy_real_planar_7dof.xml"
        ).model
        self.assertAlmostEqual(vehicle.wheel_inertia, 0.5 * 10 * (16 * 0.0254 / 2)**2)

    def test_mass_matrix_and_no_mutation(self):
        force = np.array([2., 6.])
        mass = np.diag([2., 3.])
        np.testing.assert_allclose(RateSystem(force, mass).solve(), [1., 2.])
        np.testing.assert_array_equal(force, [2., 6.])
        np.testing.assert_array_equal(mass, np.diag([2., 3.]))

    def test_invalid_system_rejected(self):
        for force, mass in [(np.ones(2), np.zeros((2, 2))),
                            (np.ones(2), np.eye(3)),
                            (np.array([np.nan]), None)]:
            with self.assertRaises(ValueError):
                RateSystem(force, mass).solve()


class DriverTests(unittest.TestCase):
    def setUp(self):
        distance = np.linspace(0, 100, 101)
        curvature = np.zeros(101)
        curvature[45:56] = 0.2
        self.plan = build_speed_plan(distance, curvature, 15., 4., 2., 3.)
        self.driver = PreviewDriver(self.plan, 1.5, 0.5, 2., 1000.)

    def test_braking_before_corner_and_periodic_seam(self):
        self.assertLess(self.plan.speed[40], self.plan.speed[20])
        self.assertAlmostEqual(self.plan.speed[0], self.plan.speed[-1])
        self.assertLessEqual(self.plan.speed[50], np.sqrt(4. / .2))

    def test_live_plan_honors_configured_maximum_speed(self):
        vehicle = compile_formulation("models/formulations/iem24_murphy_real_planar_7dof.xml").model
        distance = np.array([0., 5., 10.])
        curvature = np.zeros(3)
        plan = build_speed_plan(distance, curvature, maximum_speed=5.,
                                lateral_acceleration=3., vehicle=vehicle, live_limit=True)
        np.testing.assert_allclose(plan.speed, 5.)
        with self.assertRaises(ValueError):
            build_speed_plan(distance, curvature, maximum_speed=np.nan,
                             vehicle=vehicle, live_limit=True)

    def test_explicit_steering_tuning_changes_request_and_respects_limits(self):
        observation = DriverObservation(45., .05, .02, 10., 0., 0.)
        tuned = PreviewDriver(self.plan, 1.5, 0.5, 20., 1000.,
                              preview_seconds=.35, heading_gain=2.5, lateral_gain=.75)
        legacy = self.driver.command(observation)
        command = tuned.command(observation)
        self.assertTrue(np.all(np.isfinite((command.throttle, command.brake_force,
                                             command.steering_rate))))
        self.assertLessEqual(abs(command.steering_rate), tuned.maximum_steering_rate)
        self.assertNotEqual(command.steering_rate, legacy.steering_rate)

    def test_direct_preview_driver_defaults_remain_legacy_values(self):
        self.assertEqual(self.driver.preview_seconds, .15)
        self.assertEqual(self.driver.heading_gain, 1.5)
        self.assertEqual(self.driver.lateral_gain, 1.5)

    def test_driver_sign_limits_and_overlap(self):
        command = self.driver.command(DriverObservation(50., 0., 0., 15., 0., 0.))
        self.assertGreater(command.steering_rate, 0)
        self.assertLessEqual(command.steering_rate, 2)
        self.assertEqual(command.throttle, 0)
        self.assertGreater(command.brake_force, 0)
        self.assertLessEqual(command.brake_force, 1000)

    def test_brake_governor_reduces_pedal_force_as_wheel_locks(self):
        def command(margin):
            return self.driver.command(DriverObservation(50., 0., 0., 15., 0., 0.,
                                                         wheel_speed_margin=margin))
        full = command(float("inf")).brake_force
        self.assertGreater(full, 0.)
        self.assertAlmostEqual(command(.85).brake_force, full)
        self.assertAlmostEqual(command(.425).brake_force, .5 * full)
        self.assertEqual(command(0.).brake_force, 0.)

    def test_driver_tracks_course_not_zero_body_sideslip(self):
        straight = build_speed_plan(np.array([0., 100.]), np.zeros(2))
        driver = PreviewDriver(straight, 1.5, .5, 2., 1000.)
        zero = driver.command(DriverObservation(0., 0., 0., 8., 0., 0.))
        angled = driver.command(DriverObservation(0., 0., -np.arctan2(1., 8.),
                                                  8., 0., 0., 1.))
        self.assertAlmostEqual(zero.steering_rate, angled.steering_rate)

    def test_nonfinite_and_negative_plan_rejected(self):
        with self.assertRaises(ValueError):
            build_speed_plan(np.array([0., 1.]), np.array([0., np.nan]), 10., 3., 2., 2.)
        with self.assertRaises(ValueError):
            build_speed_plan(np.array([0., 1.]), np.zeros(2), -1., 3., 2., 2.)

    def test_every_accepted_state_is_revalidated(self):
        class Plant:
            state_size = 1
            def __init__(self):
                self.seen = []
            def observe(self, state):
                return DriverObservation(state[0], 0., 0., 1., 0., 0.)
            def evaluate(self, time, state, command):
                self.seen.append(float(time))
                return RateSystem(np.array([1.]))
        plant = Plant()
        result = solve_forward(plant, self.driver, np.array([0.]),
                               duration=2., maximum_step=.005, method="BDF")
        # Every accepted result sample must be rechecked *after* integration.
        np.testing.assert_allclose(
            plant.seen[-(2 * len(result.time) - 1):-(len(result.time) - 1)],
            result.time,
        )

    def test_initial_pedal_state_is_preserved_for_warm_start(self):
        class Plant:
            state_size = 1
            def observe(self, state):
                return DriverObservation(state[0], 0., 0., 1., 0., 0.)
            def evaluate(self, time, state, command):
                return RateSystem(np.array([1.]))
        driver = PreviewDriver(self.plan, 1.5, 0.5, 2., 1000.,
                               pedal_travel_time=.175)
        result = solve_forward(Plant(), driver, np.array([0.]), duration=.01,
                               initial_pedal=-.25)
        self.assertAlmostEqual(result.pedal[0], -.25)

    def test_dense_midpoints_are_checked(self):
        class Plant:
            state_size = 1
            def __init__(self):
                self.seen = []
            def observe(self, state):
                return DriverObservation(state[0], 0., 0., 1., 0., 0.)
            def evaluate(self, time, state, command):
                self.seen.append(float(time))
                return RateSystem(np.array([1.]))
        plant = Plant()
        result = solve_forward(plant, self.driver, np.array([0.]),
                               duration=.1, maximum_step=.01, method="BDF")
        for midpoint in (result.time[1:] + result.time[:-1]) / 2.:
            self.assertTrue(any(abs(midpoint - sampled) < 1.e-12 for sampled in plant.seen))
        self.assertTrue(result.midpoint_sample_validation_complete)

    def test_extra_states_and_mass_matrix_survive_integration(self):
        class Plant:
            state_size = 32

            def observe(self, state):
                return DriverObservation(state[0], 0., 0., 1., 0., 0.)

            def evaluate(self, time, state, command):
                forcing = -state.copy()
                forcing[0] = 2.
                return RateSystem(forcing, 2. * np.eye(32))

        initial = np.ones(32)
        initial[0] = 0.
        result = solve_forward(Plant(), self.driver, initial, duration=.2, maximum_step=.05)
        self.assertEqual(result.state.shape[0], 32)
        self.assertLess(result.state[-1, -1], initial[-1])
        self.assertTrue(np.all(np.diff(result.time) > 0))
        self.assertAlmostEqual(result.state[-1, -1], np.exp(-.1), places=6)
        self.assertFalse(result.lap_completed)
        bdf = solve_forward(Plant(), self.driver, initial, duration=.2,
                            maximum_step=.05, method="BDF")
        np.testing.assert_allclose(bdf.state[:, -1], result.state[:, -1], rtol=2.e-6)
        with self.assertRaises(ValueError):
            solve_forward(Plant(), self.driver, initial, method="unknown")
        with self.assertRaises(TimeoutError):
            solve_forward(Plant(), self.driver, initial, wall_seconds=1.e-9)
        np.testing.assert_array_equal(initial[1:], np.ones(31))
        with self.assertRaises(ValueError):
            solve_forward(Plant(), self.driver, initial[:-1], duration=.1)
        with self.assertRaises(ValueError):
            solve_forward(Plant(), self.driver, initial, wall_seconds=301.)


if __name__ == "__main__":
    unittest.main()
