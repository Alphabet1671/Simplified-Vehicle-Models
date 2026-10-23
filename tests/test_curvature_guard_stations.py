import unittest

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
        # Narrow physical bump deliberately missed by the five legacy fractions.
        k = np.where(np.isclose(x, 1.25, rtol=0., atol=1.e-12), .2, 0.)
        return type("Geometry", (), {"curvature": k})()


class _Path:
    distance = np.array([0., 10.])
    curvature = np.array([0., 0.])
    is_closed = False


class _Surface:
    speed_floor = 0.
    speed = np.array([0., 100.])
    curvature = np.array([-.3, .3])
    acceleration_min = np.tile(np.array([-10., -10.]), (2, 1))
    acceleration_max = np.array([[4., 0.], [4., 0.]])

    def __init__(self):
        self.seen = []

    def domain_speed(self, k):
        return 100. if abs(k) < .1 else 4.

    def limits(self, v, k):
        self.seen.append((v, k))
        return (-10., 4. - 10. * abs(k))


class CurvatureGuardStationTests(unittest.TestCase):
    def _solve(self, stations=None, surface=None, step=10.):
        surface = _Surface() if surface is None else surface
        profile = capability_profile(_Path(), surface, boundary="rolling", start_speed=1., step=step,
            transition_method="bracketed_energy", trajectory=_Trajectory(),
            segment_curvature_sampling="trajectory_five_fractions",
            curvature_guard_stations=stations)
        return profile, surface

    def test_local_guard_catches_bump_missed_by_old_fractions(self):
        baseline, _ = self._solve()
        guarded, surface = self._solve([1.25])
        self.assertTrue(baseline.converged)
        self.assertTrue(guarded.converged, guarded.diagnostics)
        self.assertLess(guarded.speed[-1], baseline.speed[-1])
        self.assertTrue(any(k == .2 for _, k in surface.seen))
        self.assertEqual(guarded.diagnostics["curvature_guard_stations"], (1.25,))
        self.assertEqual(guarded.diagnostics["additional_curvature_samples_per_segment"], (1,))
        self.assertEqual(guarded.diagnostics["total_curvature_samples_per_segment"], (7,))

    def test_guard_validation_precedes_model_callbacks_and_empty_matches_default(self):
        calls = []
        class Count(_Surface):
            def domain_speed(self, k):
                calls.append(k)
                return super().domain_speed(k)
        from fractions import Fraction
        for bad in ([True], [np.bool_(False)], [True, 1.25], [np.bool_(True), 1.25], [np.inf], ["1.25"], [1+0j], [Fraction(1, 3)], [[1.25]], np.zeros(4097)):
            with self.assertRaises(ValueError):
                capability_profile(_Path(), Count(), boundary="rolling", start_speed=1., step=10.,
                    transition_method="bracketed_energy", trajectory=_Trajectory(),
                    segment_curvature_sampling="trajectory_five_fractions", curvature_guard_stations=bad)
        self.assertEqual(calls, [])
        default, _ = self._solve()
        empty, _ = self._solve([])
        np.testing.assert_array_equal(default.speed, empty.speed)
        self.assertEqual(dict(default.diagnostics), dict(empty.diagnostics))

    def test_duplicates_sort_and_copy_input(self):
        stations = np.array([7.5, 1.25, 1.25])
        profile, _ = self._solve(stations)
        stations[:] = 2.
        self.assertEqual(profile.diagnostics["curvature_guard_stations"], (1.25, 7.5))
        # 7.5 is already a base quarter-fraction, so only 1.25 is additional.
        self.assertEqual(profile.diagnostics["additional_curvature_samples_per_segment"], (1,))
        self.assertEqual(profile.diagnostics["total_curvature_samples_per_segment"], (7,))

    def test_per_cell_sample_counts_and_mesh_node_is_shared_by_both_cells(self):
        class TwoCellPath:
            distance = np.array([0., 10., 20.])
            curvature = np.zeros(3)
            is_closed = False
        class TwoCellMap:
            path_start = 0.
            path_end = 20.
            def to_reference(self, x): return np.asarray(x, dtype=float)
        class TwoCellTrajectory(_Trajectory):
            station_map = TwoCellMap()
            reference = type("Ref", (), {"is_closed": False, "source_station": np.array([0., 20.])})()
            def evaluate(self, source):
                x = np.asarray(source, dtype=float)
                return type("Geometry", (), {"curvature": np.zeros_like(x)})()
        profile = capability_profile(TwoCellPath(), _Surface(), boundary="rolling", start_speed=1., step=10.,
            transition_method="bracketed_energy", trajectory=TwoCellTrajectory(),
            segment_curvature_sampling="trajectory_five_fractions", curvature_guard_stations=[1.25, 10., 11.25])
        self.assertEqual(profile.diagnostics["additional_curvature_samples_per_segment"], (1, 1))
        self.assertEqual(profile.diagnostics["total_curvature_samples_per_segment"], (7, 7))

    def test_domain_boundaries_accepted_and_outside_rejected(self):
        for station in (0., 10.):
            profile, _ = self._solve([station])
            self.assertTrue(profile.converged)
        for station in (-np.nextafter(0., 1.), np.nextafter(10., np.inf)):
            with self.assertRaisesRegex(ValueError, "domain"):
                self._solve([station])

    def test_guard_requires_physical_five_fraction_mode(self):
        with self.assertRaisesRegex(ValueError, "require"):
            capability_profile(_Path(), _Surface(), boundary="rolling", start_speed=1.,
                curvature_guard_stations=[1.25])

    def test_live_wrapper_reports_json_safe_station_counts_and_preserves_input(self):
        stations = np.array([1.25])
        settings = LiveStandingSettings(step=10., transition_method="bracketed_energy",
            segment_curvature_sampling="trajectory_five_fractions")
        profile, report = solve_live_standing_profile(object(), _Trajectory(), settings=settings,
            curvature_guard_stations=stations,
            limit_evaluator=lambda vehicle, v, k: (-10., 4.-10.*abs(k)),
            speed_cap_evaluator=lambda vehicle, k: 100. if abs(k) < .1 else 4.)
        self.assertIsNotNone(profile, report)
        self.assertEqual(report["curvature_guard_stations"], [1.25])
        self.assertEqual(report["additional_curvature_samples_per_segment"], [1])
        self.assertEqual(report["total_curvature_samples_per_segment"], [7])
        stations[0] = 8.
        self.assertEqual(report["curvature_guard_stations"], [1.25])
        # JSON encoding confirms report metadata contains only plain values.
        import json
        json.dumps(report)

    def test_wrapper_rejects_guard_before_model_queries_without_opt_in_mode(self):
        calls = []
        profile, report = solve_live_standing_profile(object(), _Trajectory(),
            curvature_guard_stations=[1.25], limit_evaluator=lambda *a: calls.append(a) or (-1., 1.),
            speed_cap_evaluator=lambda *a: calls.append(a) or 10.)
        self.assertIsNone(profile)
        self.assertEqual(calls, [])
        self.assertIn("require", report["error"]["message"])


if __name__ == "__main__":
    unittest.main()
