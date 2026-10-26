import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.solvers.capability_profile import capability_profile
from simplified_vehicle_models.solvers.live_standing_profile import LiveStandingSettings, solve_live_standing_profile
from simplified_vehicle_models.tracks.types import PathCoordinates


class _Map:
    path_start = 0.0
    path_end = 10.0

    def to_reference(self, x):
        return np.asarray(x, dtype=float)


class _Trajectory:
    station_map = _Map()
    reference = type("Ref", (), {"is_closed": False, "source_station": np.array([0., 10.])})()

    def to_path(self, source):
        s = np.asarray(source, dtype=float)
        return PathCoordinates(s, s, np.zeros_like(s), np.zeros_like(s), np.zeros_like(s), False, s)

    def evaluate(self, source):
        x = np.asarray(source, dtype=float)
        # Endpoints are straight; an interior physical bump is invisible to node averaging.
        k = np.where(np.isclose(x, 5.0), .2, 0.0)
        return type("Geometry", (), {"curvature": k})()


class _Path:
    distance = np.array([0., 10.])
    curvature = np.array([0., 0.])
    is_closed = False


class _Surface:
    speed_floor = 0.0
    speed = np.array([0., 100.])
    curvature = np.array([-.3, .3])
    acceleration_min = np.tile(np.array([-10., -10.]), (2, 1))
    acceleration_max = np.array([[4., 0.], [4., 0.]])
    speed_ceiling = np.array([100., 4.])

    def domain_speed(self, k):
        return 100. if abs(k) < .1 else 4.

    def limits(self, v, k):
        return (-10., 4. - 10. * abs(k))


class SegmentCurvatureSamplingTests(unittest.TestCase):
    def test_default_metadata_and_profile_match_explicit_node_average(self):
        a = capability_profile(_Path(), _Surface(), boundary="rolling", start_speed=1., step=10., transition_method="bracketed_energy")
        b = capability_profile(_Path(), _Surface(), boundary="rolling", start_speed=1., step=10., transition_method="bracketed_energy", segment_curvature_sampling="node_average")
        np.testing.assert_array_equal(a.speed, b.speed)
        np.testing.assert_array_equal(a.elapsed_time, b.elapsed_time)
        self.assertEqual(dict(a.diagnostics), dict(b.diagnostics))

    def test_new_mode_queries_actual_five_fractions_and_applies_guard(self):
        seen = []
        class Recording(_Surface):
            def evaluate(self, source):
                values = _Trajectory().evaluate(source)
                self.assert_samples = np.asarray(values.curvature).copy()
                return values
            def limits(self, v, k):
                seen.append((v, k))
                return super().limits(v, k)
        trajectory = _Trajectory()
        profile = capability_profile(_Path(), Recording(), boundary="rolling", start_speed=1., step=10.,
            transition_method="bracketed_energy", trajectory=trajectory,
            segment_curvature_sampling="trajectory_five_fractions")
        self.assertTrue(any(np.isclose(k, .2) for _, k in seen), seen)
        self.assertLess(profile.speed[-1], np.sqrt(1. + 2.*4.*10.))
        self.assertEqual(profile.diagnostics["curvature_samples_per_segment"], 6)
        self.assertFalse(profile.diagnostics["continuous_feasibility_certified"])

    def test_new_mode_requires_trajectory_bracketed_and_non_signed_before_queries(self):
        queries = []
        class Count(_Surface):
            def domain_speed(self, k):
                queries.append(k)
                return super().domain_speed(k)
        for kwargs in ({}, {"transition_method": "legacy_fixed_point", "trajectory": _Trajectory()}):
            with self.assertRaises(ValueError):
                capability_profile(_Path(), Count(), boundary="rolling", start_speed=1., step=10.,
                    segment_curvature_sampling="trajectory_five_fractions", **kwargs)
        self.assertEqual(queries, [])
        from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
        chart = SignedTransientCapability(np.array([0., 1.]), np.array([-.3, .3]), np.full((2,2), 2.), np.full((2,2), -3.), np.full(2,20.), .5, "test")
        with self.assertRaisesRegex(ValueError, "signed"):
            capability_profile(_Path(), chart, boundary="rolling", start_speed=1., step=10.,
                transition_method="bracketed_energy", trajectory=_Trajectory(),
                segment_curvature_sampling="trajectory_five_fractions")
        self.assertEqual(queries, [])

    def test_bracket_transition_samples_each_curvature_at_each_speed(self):
        from simplified_vehicle_models.solvers.capability_profile import _bracketed_energy_transition
        seen = []
        samples = (0., .1, .2, .1, 0., 0.)
        def limits(v, k):
            seen.append((v, k))
            return (-2.+10.*abs(k), 4.-10.*abs(k))
        forward = _bracketed_energy_transition(limits, vi=1., bound=10., curvature=0., ds=2.,
            forward=True, tolerance=1.e-8, segment_limit_sampling="speed_endpoints_and_midpoint",
            curvature_samples=samples)
        self.assertLess(forward, np.sqrt(1.+8.*2.))
        self.assertTrue(all(any(k == sample for _, k in seen) for sample in samples))
        seen.clear()
        backward = _bracketed_energy_transition(limits, vi=4., bound=10., curvature=0., ds=2.,
            forward=False, tolerance=1.e-8, segment_limit_sampling="speed_endpoints_and_midpoint",
            curvature_samples=samples)
        self.assertGreater(backward, np.sqrt(16.-4.*2.))
        self.assertTrue(any(k == .2 for _, k in seen))
        baseline_braking = _bracketed_energy_transition(limits, vi=4., bound=10., curvature=0., ds=2.,
            forward=False, tolerance=1.e-8, segment_limit_sampling="speed_endpoints_and_midpoint")
        self.assertLess(backward, baseline_braking)

    def test_final_verifier_checks_interior_curvature(self):
        with patch("simplified_vehicle_models.solvers.capability_profile._bracketed_energy_transition", return_value=10.):
            profile = capability_profile(_Path(), _Surface(), boundary="rolling", start_speed=1., step=10.,
                transition_method="bracketed_energy", trajectory=_Trajectory(),
                segment_curvature_sampling="trajectory_five_fractions")
        self.assertFalse(profile.converged)
        self.assertEqual(profile.status, "dynamically-infeasible")

    def test_bad_mapped_geometry_and_wall_guard_fail_without_fallback(self):
        class BadMap(_Trajectory):
            class Map:
                path_start = 0.
                path_end = 10.
                def to_reference(self, x): return np.full(5, np.nan)
            station_map = Map()
        with self.assertRaisesRegex(ValueError, "station map"):
            capability_profile(_Path(), _Surface(), boundary="rolling", start_speed=1., step=10.,
                transition_method="bracketed_energy", trajectory=BadMap(),
                segment_curvature_sampling="trajectory_five_fractions")

        class Guard(_Surface):
            def ensure_wall_budget(self): raise TimeoutError("geometry wall budget exhausted")
        with self.assertRaisesRegex(TimeoutError, "wall budget"):
            capability_profile(_Path(), Guard(), boundary="rolling", start_speed=1., step=10.,
                transition_method="bracketed_energy", trajectory=_Trajectory(),
                segment_curvature_sampling="trajectory_five_fractions")

    def test_slow_node_geometry_is_bracketed_before_five_fraction_queries(self):
        clock = [0.]
        calls = [0]
        class ClockSurface(_Surface):
            maximum_wall_seconds = 1.
            started = 0.
            def ensure_wall_budget(self):
                if clock[0] - self.started > self.maximum_wall_seconds:
                    raise TimeoutError("geometry wall budget exhausted")
        class SlowTrajectory(_Trajectory):
            def evaluate(self, source):
                calls[0] += 1
                clock[0] += 2.
                return super().evaluate(source)
        with self.assertRaisesRegex(TimeoutError, "wall budget"):
            capability_profile(_Path(), ClockSurface(), boundary="rolling", start_speed=1., step=10.,
                transition_method="bracketed_energy", trajectory=SlowTrajectory(),
                segment_curvature_sampling="trajectory_five_fractions")
        # One evaluation is the initial _path_samples node query; no fraction query follows it.
        self.assertEqual(calls[0], 1)

    def test_settings_and_live_wrapper_report_sampled_mode_unqualified(self):
        with self.assertRaises(ValueError):
            LiveStandingSettings(segment_curvature_sampling="unknown")
        settings = LiveStandingSettings(step=10., transition_method="bracketed_energy",
            segment_curvature_sampling="trajectory_five_fractions")
        profile, report = solve_live_standing_profile(object(), _Trajectory(), settings=settings,
            limit_evaluator=lambda vehicle, v, k: (-10., 4.-10.*abs(k)),
            speed_cap_evaluator=lambda vehicle, k: 100. if abs(k) < .1 else 4.)
        self.assertIsNotNone(profile, report)
        self.assertEqual(report["segment_curvature_sampling"], "trajectory_five_fractions")
        self.assertFalse(report["physical_qualified"])
        self.assertFalse(report["accepted"])
        self.assertFalse(report["continuous_feasibility_certified"])
        default, default_report = solve_live_standing_profile(object(), _Trajectory(),
            settings=LiveStandingSettings(step=10.), limit_evaluator=lambda *a: (-10., 4.),
            speed_cap_evaluator=lambda *a: 100.)
        self.assertIsNotNone(default, default_report)
        self.assertNotIn("segment_curvature_sampling", default_report)
        self.assertNotIn("continuous_feasibility_certified", default_report)


if __name__ == "__main__":
    unittest.main()
