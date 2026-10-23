import json
import math
import time
import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.solvers.live_standing_profile import LiveStandingSettings, _LiveSurface, solve_live_standing_profile
from simplified_vehicle_models.tracks.types import PathCoordinates


class Ref:
    is_closed = False
    source_station = np.array([0., 75.3])


class StationMap:
    path_start = 0.
    path_end = 75.3
    def to_reference(self, x): return np.asarray(x, dtype=float)


class Trajectory:
    reference = Ref()
    station_map = StationMap()
    def to_path(self, source):
        s = np.asarray(source, dtype=float)
        return PathCoordinates(s, s, np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), False, s)
    def evaluate(self, source):
        return type("Geometry", (), {"curvature": np.zeros_like(source, dtype=float)})()


def constant_limits(vehicle, speed, curvature): return (-3., 2.)
def constant_cap(vehicle, curvature): return 100.


class LiveStandingProfileTests(unittest.TestCase):
    def test_constant_acceleration_analytic_profile_and_full_open_course(self):
        for step in (10., .5):
            profile, report = solve_live_standing_profile(
                object(), Trajectory(), settings=LiveStandingSettings(step=step),
                limit_evaluator=constant_limits, speed_cap_evaluator=constant_cap)
            self.assertIsNotNone(profile, report)
            self.assertEqual(profile.speed[0], 0.)
            self.assertAlmostEqual(profile.speed[-1], math.sqrt(4*75.3), places=6)
            self.assertAlmostEqual(profile.lap_time, math.sqrt(75.3), places=6)
            self.assertEqual(profile.distance[0], 0.)
            self.assertAlmostEqual(profile.distance[-1], 75.3)
            self.assertTrue(report["same_physics_all_speeds"])
            self.assertFalse(report["chart_queried"])
            self.assertTrue(report["no_transition_join"])
            self.assertFalse(report["physical_qualified"])
            self.assertFalse(report["all_accepted"])
            self.assertFalse(report["accepted"])
            self.assertFalse(report["event_qualified"])
            self.assertFalse(report["U3_qualified"])
            np.testing.assert_allclose(np.diff(profile.elapsed_time),
                2*np.diff(profile.distance)/(profile.speed[:-1]+profile.speed[1:]))

    def test_invalid_source_path_coverage_and_unclosed_inputs_fail_json_safely(self):
        class BadSource(Trajectory):
            class Ref:
                is_closed = False
                source_station = np.array([0., float("nan")])
            reference = Ref()
        for trajectory in (BadSource(), type("Closed", (Trajectory,), {
                "reference": type("ClosedRef", (), {"is_closed": True, "source_station": np.array([0., 1.])})()})()):
            profile, report = solve_live_standing_profile(object(), trajectory)
            self.assertIsNone(profile)
            self.assertIn("error", report)
            json.dumps(report, allow_nan=False)

        class WrongCoverage(Trajectory):
            station_map = type("Map", (), {"path_start": 0., "path_end": 70.,
                "to_reference": lambda self, x: np.asarray(x, dtype=float)})()
        profile, report = solve_live_standing_profile(object(), WrongCoverage())
        self.assertIsNone(profile)
        self.assertIn("cover exact full map length", report["error"]["message"] )

    def test_invalid_callbacks_budget_and_unconverged_never_return_candidate(self):
        for limits, cap, settings in (
            (lambda *a: (float("nan"), 2.), constant_cap, LiveStandingSettings()),
            (constant_limits, lambda *a: 0., LiveStandingSettings()),
            (constant_limits, constant_cap, LiveStandingSettings(maximum_queries=1)),
            (constant_limits, constant_cap, LiveStandingSettings(maximum_iterations=1, tolerance=1e-14)),
        ):
            profile, report = solve_live_standing_profile(object(), Trajectory(), settings=settings,
                limit_evaluator=limits, speed_cap_evaluator=cap)
            self.assertIsNone(profile)
            self.assertFalse(report["all_accepted"])
            self.assertIn("error", report)

    def test_exact_query_budget_allows_nth_call_and_rejects_n_plus_one(self):
        # Directly exercise backend accounting independent of the sweep's cache pattern.
        settings = LiveStandingSettings(maximum_queries=2)
        surface = _LiveSurface(object(), settings, lambda *a: (-1., 1.), lambda *a: 20., time.monotonic())
        self.assertEqual(surface.domain_speed(0.), 20.)
        self.assertEqual(surface.limits(1., 0.), (-1., 1.))
        self.assertEqual(surface.query_count, 2)
        with self.assertRaisesRegex(RuntimeError, "query budget"):
            surface.domain_speed(.1)
        self.assertEqual(surface.query_count, 2)

    def test_cached_limit_and_cap_still_check_wall_deadline(self):
        settings = LiveStandingSettings(maximum_queries=10, maximum_wall_seconds=1.)
        clock = [0.]
        def now(): return clock[0]
        surface = _LiveSurface(object(), settings, lambda *a: (-1., 1.), lambda *a: 20., 0.)
        with patch("simplified_vehicle_models.solvers.live_standing_profile.time.monotonic", now):
            self.assertEqual(surface.domain_speed(0.), 20.)
            self.assertEqual(surface.limits(1., 0.), (-1., 1.))
            clock[0] = 2.
            with self.assertRaises(TimeoutError):
                surface.domain_speed(0.)
            with self.assertRaises(TimeoutError):
                surface.limits(1., 0.)

    def test_callback_values_must_be_real_nonboolean_scalars(self):
        for bad in (True, "2", np.array(2.), np.float64(float("nan"))):
            settings = LiveStandingSettings(maximum_queries=20)
            surface = _LiveSurface(object(), settings, lambda *a: (bad, 2.), lambda *a: 20., time.monotonic())
            with self.assertRaises(ValueError):
                surface.limits(1., 0.)
        for bad in (True, "20", np.array(20.), float("inf")):
            settings = LiveStandingSettings(maximum_queries=20)
            surface = _LiveSurface(object(), settings, lambda *a: (-1., 1.), lambda *a: bad, time.monotonic())
            with self.assertRaises(ValueError):
                surface.domain_speed(0.)

    def test_false_callback_override_is_rejected_not_replaced_by_default(self):
        profile, report = solve_live_standing_profile(object(), Trajectory(),
            limit_evaluator=False, speed_cap_evaluator=constant_cap)
        self.assertIsNone(profile)
        self.assertEqual(report["error"]["type"], "TypeError")
        json.dumps(report, allow_nan=False)

    def test_query_cache_exact_and_curvature_specific_caps(self):
        calls = {"limit": [], "cap": []}
        def lim(v, s, k): calls["limit"].append((s,k)); return (-4., 1.)
        def cap(v, k): calls["cap"].append(k); return 50.
        profile, report = solve_live_standing_profile(object(), Trajectory(),
            settings=LiveStandingSettings(step=5.), limit_evaluator=lim, speed_cap_evaluator=cap)
        self.assertIsNotNone(profile, report)
        self.assertGreater(report["queries"], 0)
        self.assertGreaterEqual(report["cache_hits"], 0)
        self.assertEqual(len(calls["limit"]), len(set(calls["limit"])))

    def test_curved_cap_triggers_braking_preview_and_signed_limit_queries(self):
        class Curved(Trajectory):
            class Ref:
                is_closed = False
                source_station = np.array([0., 20.])
            reference = Ref()
            class Map:
                path_start = 0.
                path_end = 20.
                def to_reference(self, x): return np.asarray(x, dtype=float)
            station_map = Map()
            def to_path(self, source):
                s = np.asarray(source, dtype=float)
                return PathCoordinates(s, s, np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), False, s)
            def evaluate(self, source):
                k = np.where(np.asarray(source) >= 10., -.2, 0.)
                return type("Geometry", (), {"curvature": k})()
        queried = []
        def limits(vehicle, speed, curvature):
            queried.append((speed, curvature))
            return (-2., 4.)
        def cap(vehicle, curvature): return 100. if curvature == 0. else 4.
        profile, report = solve_live_standing_profile(object(), Curved(),
            settings=LiveStandingSettings(step=1.), limit_evaluator=limits, speed_cap_evaluator=cap)
        self.assertIsNotNone(profile, report)
        self.assertTrue(any(k < 0 for _, k in queried)) # signed curvature values are preserved
        self.assertTrue(any(k == -.2 for _, k in queried))
        self.assertLess(profile.speed[np.searchsorted(profile.distance, 9.)], math.sqrt(2*4*9.))
        self.assertFalse(report["physical_qualified"])

    def test_actual_compiled_iem24_vehicle_on_short_straight(self):
        from pathlib import Path
        from simplified_vehicle_models.compiler.compile import compile_formulation
        from simplified_vehicle_models.formulations.point_mass import maximum_vehicle_speed
        root = Path(__file__).resolve().parents[1]
        model = compile_formulation(str(root / "models/formulations/iem24_murphy_real_point_mass.xml")).model
        class Short(Trajectory):
            reference = type("OpenReference", (), {"is_closed": False, "source_station": np.array([0., 75.3])})()
            station_map = type("Map", (), {"path_start": 0., "path_end": 75.3,
                "to_reference": lambda self, x: np.asarray(x, dtype=float)})()
            def to_path(self, source):
                s = np.asarray(source, dtype=float)
                return PathCoordinates(s, s, np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), False, s)
        result, report = solve_live_standing_profile(model, Short(),
            settings=LiveStandingSettings(step=10., maximum_queries=5000, maximum_wall_seconds=30.))
        self.assertIsNotNone(result, report)
        self.assertEqual(result.speed[0], 0.)
        self.assertAlmostEqual(result.distance[-1], 75.3)
        self.assertTrue(math.isfinite(result.lap_time))
        self.assertGreater(report["queries"], 0)
        self.assertLess(result.speed.max(), maximum_vehicle_speed(model) + 1e-8)
        self.assertFalse(report["chart_queried"])
        self.assertFalse(report["accepted"])
        self.assertFalse(report["physical_qualified"])
        self.assertFalse(report["numerically_validated"])
