"""Standing-start support for open courses in the forward-driver stack."""
from pathlib import Path
import tempfile
import unittest

import numpy as np

from simplified_vehicle_models.compiler.compile import compile_formulation
from simplified_vehicle_models.solvers.forward_driver import (
    PreviewDriver, SpeedPlan, build_speed_plan, solve_forward)
from simplified_vehicle_models.solvers.planar_dense_certificate import (
    certify_planar_dense_domain, curvature_enclosure, open_curvature_enclosure)
from simplified_vehicle_models.solvers.planar_forward import PlanarForwardPlant
from simplified_vehicle_models.solvers.vertical_forward import VerticalForwardPlant
from simplified_vehicle_models.tracks.path_coordinates import evaluate_path
from simplified_vehicle_models.tracks.profile import load_track
from tools.run.run_iem24_forward_driver import acceptance_gate, event_times

ROOT = Path(__file__).resolve().parents[1]
ACCELERATION = ROOT / "models/tracks/fsae_2026_acceleration.xml"
SKIDPAD = ROOT / "models/tracks/fsae_2026_skidpad.xml"
PLANAR = ROOT / "models/formulations/iem24_murphy_real_planar_7dof.xml"
VERTICAL = ROOT / "models/formulations/iem24_murphy_real_14dof_simple.xml"


def control(vehicle):
    """Return the planar parameters of a planar or vertical vehicle."""
    return getattr(vehicle, "planar", vehicle)


def open_plan(track_path, vehicle, live=True, maximum_speed=40.):
    track = load_track(track_path)
    distance = np.linspace(0., track.lap_length, int(np.ceil(track.lap_length / .5)) + 1)
    curvature = evaluate_path(track.path_coordinates, distance)[-1]
    return build_speed_plan(distance, curvature, maximum_speed, None, 1., 2.,
                            vehicle=vehicle if live else None, live_limit=live,
                            open_course=True)


class OpenSpeedPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vehicle = control(compile_formulation(str(PLANAR)).model)

    def test_open_plan_allows_rest_only_at_the_start(self):
        distance = np.array([0., 1., 2.])
        plan = SpeedPlan(distance, np.array([0., 1., 2.]), np.zeros(3), is_closed=False)
        self.assertFalse(plan.is_closed)
        with self.assertRaises(ValueError):
            SpeedPlan(distance, np.array([0., 0., 2.]), np.zeros(3), is_closed=False)
        with self.assertRaises(ValueError):
            SpeedPlan(distance, np.array([0., 1., 2.]), np.zeros(3))

    def test_open_plan_does_not_wrap(self):
        plan = SpeedPlan(np.array([0., 10., 20.]), np.array([0., 5., 6.]),
                         np.array([0., 0., .1]), is_closed=False)
        speed, curvature, _ = plan.sample(25.)
        self.assertAlmostEqual(speed, 6.)
        self.assertAlmostEqual(curvature, .1)
        speed, _, _ = plan.sample(-3.)
        self.assertAlmostEqual(speed, 0.)

    def test_live_plan_starts_at_rest_and_accelerates(self):
        plan = open_plan(ACCELERATION, self.vehicle)
        self.assertFalse(plan.is_closed)
        self.assertEqual(plan.speed[0], 0.)
        self.assertTrue(np.all(plan.speed[1:] > 0.))
        self.assertGreater(plan.speed[-1], plan.speed[len(plan.speed) // 2])

    def test_conservative_plan_starts_at_rest_and_brakes_into_the_skidpad(self):
        plan = open_plan(SKIDPAD, None, live=False, maximum_speed=12.)
        self.assertEqual(plan.speed[0], 0.)
        self.assertLessEqual(plan.speed.max(), 12. + 1.e-9)
        self.assertFalse(plan.is_closed)

    def test_lateral_margin_one_is_the_unmargined_plan(self):
        default = open_plan(SKIDPAD, self.vehicle)
        track = load_track(SKIDPAD)
        distance = np.linspace(0., track.lap_length, int(np.ceil(track.lap_length / .5)) + 1)
        curvature = evaluate_path(track.path_coordinates, distance)[-1]
        explicit = build_speed_plan(distance, curvature, 40., None, 1., 2., vehicle=self.vehicle,
                                    live_limit=True, open_course=True, lateral_margin=1.)
        np.testing.assert_array_equal(default.speed, explicit.speed)

    def test_lateral_margin_lowers_corner_speed_and_not_the_straights(self):
        track = load_track(SKIDPAD)
        distance = np.linspace(0., track.lap_length, int(np.ceil(track.lap_length / .5)) + 1)
        curvature = evaluate_path(track.path_coordinates, distance)[-1]
        full = build_speed_plan(distance, curvature, 40., None, 1., 2., vehicle=self.vehicle,
                                live_limit=True, open_course=True)
        margin = build_speed_plan(distance, curvature, 40., None, 1., 2., vehicle=self.vehicle,
                                  live_limit=True, open_course=True, lateral_margin=.9)
        circle = np.abs(curvature) > .1
        # Only nodes held at the corner limit slow down. On the entry the car is still
        # accelerating and the lower lateral demand leaves it slightly more drive force.
        held = circle & (full.speed ** 2 * np.abs(curvature) > 0.99 * (full.speed ** 2 * np.abs(curvature)).max())
        self.assertGreater(held.sum(), 100)
        self.assertTrue(np.all(margin.speed[held] < full.speed[held]))
        lateral = margin.speed[circle] ** 2 * np.abs(curvature[circle])
        ratio = lateral.max() / (full.speed[circle] ** 2 * np.abs(curvature[circle])).max()
        self.assertLess(ratio, .95)
        self.assertGreater(ratio, .85)
        straight = distance < 10.
        np.testing.assert_allclose(margin.speed[straight], full.speed[straight], rtol=1.e-9)
        self.assertEqual(margin.speed[0], 0.)

    def test_lateral_margin_must_be_in_range(self):
        distance = np.linspace(0., 50., 101)
        for bad in (0., -.1, 1.01, float("nan")):
            with self.assertRaises(ValueError):
                build_speed_plan(distance, np.zeros(101), 40., None, 1., 2.,
                                 vehicle=self.vehicle, live_limit=True, open_course=True,
                                 lateral_margin=bad)

    def test_closed_plan_is_unchanged_by_the_open_option_default(self):
        distance = np.linspace(0., 100., 201)
        curvature = np.full_like(distance, .02)
        plan = build_speed_plan(distance, curvature, 8., None, 1., 2.)
        self.assertTrue(plan.is_closed)
        self.assertTrue(np.all(plan.speed > 0.))


class StandingStartPlantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planar_vehicle = compile_formulation(str(PLANAR)).model
        cls.vertical_vehicle = compile_formulation(str(VERTICAL)).model
        cls.plan = open_plan(ACCELERATION, control(cls.planar_vehicle))

    def test_planar_plant_starts_from_rest(self):
        plant = PlanarForwardPlant(self.planar_vehicle, self.plan)
        self.assertEqual(plant.minimum_speed, 0.)
        initial = plant.initial_state(0.)
        self.assertEqual(initial[3], 0.)
        self.assertTrue(np.all(np.isfinite(initial)))

    def test_closed_plan_still_needs_a_rolling_start(self):
        distance = np.linspace(0., 100., 201)
        closed = SpeedPlan(distance, np.full_like(distance, 8.), np.zeros_like(distance))
        plant = PlanarForwardPlant(self.planar_vehicle, closed)
        self.assertEqual(plant.minimum_speed, .25)
        with self.assertRaises(ValueError):
            plant.initial_state(0.)

    def test_vertical_plant_starts_from_rest(self):
        plant = VerticalForwardPlant(self.vertical_vehicle, self.plan)
        self.assertEqual(plant.minimum_speed, 0.)
        self.assertEqual(plant.initial_state(0.)[3], 0.)

    def test_planar_launch_accelerates_forward(self):
        plant = PlanarForwardPlant(self.planar_vehicle, self.plan)
        car = control(self.planar_vehicle)
        driver = PreviewDriver(self.plan, car.wheelbase, car.steering_maximum_angle,
                               car.steering_maximum_rate, car.maximum_pedal_force,
                               speed_gain=10., pedal_travel_time=.175)
        result = solve_forward(plant, driver, plant.initial_state(0.), duration=1.5,
                               wall_seconds=60., method="BDF", initial_pedal=0.)
        speed = result.state[3]
        self.assertEqual(speed[0], 0.)
        self.assertGreater(speed[-1], 2.)
        self.assertGreaterEqual(speed.min(), -1.e-9)
        self.assertLess(np.abs(result.state[1]).max(), 1.e-6)

    def test_vertical_launch_accelerates_forward(self):
        plant = VerticalForwardPlant(self.vertical_vehicle, self.plan)
        car = control(self.vertical_vehicle)
        driver = PreviewDriver(self.plan, car.wheelbase, car.steering_maximum_angle,
                               car.steering_maximum_rate, car.maximum_pedal_force,
                               speed_gain=10., pedal_travel_time=.175)
        result = solve_forward(plant, driver, plant.initial_state(0.), duration=1.5,
                               wall_seconds=120., method="BDF", initial_pedal=0.,
                               maximum_wall_seconds=300.)
        self.assertGreater(result.state[3, -1], 2.)
        self.assertGreaterEqual(result.state[3].min(), -1.e-9)


class OpenCertificateTests(unittest.TestCase):
    def test_open_enclosure_holds_the_end_values(self):
        distance = np.array([0., 10., 20.])
        curvature = np.array([0., .1, .2])
        low, high = open_curvature_enclosure(distance, curvature, np.array([15., 25.]),
                                             np.array([30., 30.]))
        np.testing.assert_allclose(low, [.15, .2])
        np.testing.assert_allclose(high, [.2, .2])

    def test_closed_enclosure_still_wraps(self):
        distance = np.array([0., 10., 20.])
        curvature = np.array([0., .1, 0.])
        low, high = curvature_enclosure(distance, curvature, np.array([15.]), np.array([25.]))
        self.assertAlmostEqual(float(low[0]), 0.)
        self.assertAlmostEqual(float(high[0]), .05)

    def test_rest_start_passes_for_an_open_plan_and_fails_for_a_closed_one(self):
        vehicle = compile_formulation(str(PLANAR)).model
        distance = np.array([0., 10., 20.])
        speed = np.array([1., 5., 6.])
        bounds = np.zeros((2, 13, 2))
        bounds[:, 10, :] = .8
        bounds[0, 0] = [0., 1.]
        bounds[1, 0] = [1., 5.]
        bounds[0, 3] = [-1.e-19, 1.]
        bounds[1, 3] = [1., 5.]
        for column in range(6, 10):
            bounds[0, column] = [-1.e-19, 1.]
            bounds[1, column] = [1., 5.]
        open_plan_ = SpeedPlan(distance, np.array([0., 5., 6.]), np.zeros(3), is_closed=False)
        closed_plan = SpeedPlan(distance, speed, np.zeros(3))
        opened = certify_planar_dense_domain(vehicle, open_plan_, bounds, True)
        closed = certify_planar_dense_domain(vehicle, closed_plan, bounds, True)
        self.assertNotIn("speed_over_minimum", opened.get("failed", []))
        self.assertIn("speed_over_minimum", closed.get("failed", []))

    def test_real_reversal_still_fails_an_open_plan(self):
        vehicle = compile_formulation(str(PLANAR)).model
        plan = SpeedPlan(np.array([0., 10., 20.]), np.array([0., 5., 6.]), np.zeros(3),
                         is_closed=False)
        bounds = np.zeros((1, 13, 2))
        bounds[:, 10, :] = .8
        bounds[0, 3] = [-.5, 1.]
        result = certify_planar_dense_domain(vehicle, plan, bounds, True)
        self.assertIn("speed_over_minimum", result["failed"])


class EventTimeAndGateTests(unittest.TestCase):
    def test_acceleration_window_excludes_the_staging_offset(self):
        time = np.linspace(0., 8., 801)
        distance = 75.3 * (time / 8.) ** 2
        times = event_times(ACCELERATION, time, distance)
        expected = 8. - 8. * np.sqrt(.3 / 75.3)
        self.assertAlmostEqual(times["timing"], expected, places=2)

    def test_skidpad_reports_both_timed_laps(self):
        track = load_track(SKIDPAD)
        time = np.linspace(0., 25., 2501)
        distance = track.lap_length * time / 25.
        times = event_times(SKIDPAD, time, distance)
        self.assertEqual(set(times), {"right_timed_lap", "left_timed_lap"})
        for value in times.values():
            self.assertAlmostEqual(value, 25. * 57.334 / track.lap_length, places=2)

    def test_window_beyond_the_run_is_none(self):
        time = np.linspace(0., 3., 31)
        times = event_times(ACCELERATION, time, 10. * time)
        self.assertIsNone(times["timing"])

    def test_open_course_gate_needs_no_periodic_closure(self):
        summary = dict(lap_completed=True, track_is_closed=False, integration_completed=True,
                       path_tracking_valid=True, resolution_valid=True,
                       checked_rhs_physics_valid=True, between_step_validation_complete=True,
                       refinement_validated=True, contact_valid=True)
        self.assertTrue(acceptance_gate(summary))
        self.assertFalse(acceptance_gate({**summary, "lap_completed": False}))
        self.assertFalse(acceptance_gate({**summary, "path_tracking_valid": False}))
        self.assertFalse(acceptance_gate({**summary, "track_is_closed": True}))

    def test_open_course_gate_still_needs_the_other_gates(self):
        summary = dict(lap_completed=True, track_is_closed=False, integration_completed=True,
                       path_tracking_valid=True, resolution_valid=True,
                       checked_rhs_physics_valid=True, between_step_validation_complete=False,
                       refinement_validated=True, contact_valid=True)
        self.assertFalse(acceptance_gate(summary))


if __name__ == "__main__":
    unittest.main()
