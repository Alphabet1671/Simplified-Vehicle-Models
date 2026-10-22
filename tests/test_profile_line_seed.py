import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.solvers import profile_line_seed as seeds


class ProfileLineSeedTest(unittest.TestCase):
    def test_energy_interpolation_uses_physical_path_station(self):
        profile = SimpleNamespace(distance=np.array([0., 10.]), speed=np.array([1., 3.]))
        energy, acceleration = seeds._interpolate_energy(profile, np.array([0., 2., 10.]))
        np.testing.assert_allclose(energy, [1., 2.6, 9.])
        np.testing.assert_allclose(acceleration, [.4, .4, .4])

    def test_hs_repair_is_exact_for_nonunit_metric(self):
        mesh = np.array([0., 2., 5.])
        energy = np.array([4., 8., 17.])
        acceleration = np.array([1., 1.5, 2.])
        metric = np.array([2., 3., 4.])
        mid_metric = np.array([2.5, 3.5])
        middle = seeds._hs_midpoint_acceleration(energy, acceleration, metric, mid_metric, mesh)
        residual = (energy[1:] - energy[:-1] - np.diff(mesh) / 6 *
                    (2 * metric[:-1] * acceleration[:-1] + 8 * mid_metric * middle +
                     2 * metric[1:] * acceleration[1:]))
        np.testing.assert_allclose(residual, 0., atol=1e-14)

    def test_failed_profile_returns_explicit_unavailable_reason(self):
        track = SimpleNamespace(is_closed=False)
        spline = SimpleNamespace(control_count=2)
        settings = SimpleNamespace(start_speed=0., minimum_speed=.5, tolerance=1e-6)
        with patch.object(seeds, "_reference_curve", return_value=SimpleNamespace(source_station=np.array([0., 1.]))), \
             patch.object(seeds, "OffsetTrajectory", side_effect=ValueError("fixture geometry failure")):
            seed, diagnostics = seeds.build_profile_line_seed(track, object(), object(), np.array([0., 1.]), spline, settings)
        self.assertIsNone(seed)
        self.assertFalse(diagnostics["available"])
        self.assertEqual(diagnostics["reason"], "unsupported-or-invalid")

    def test_legacy_surface_is_reported_without_fallback_seed(self):
        track = SimpleNamespace(is_closed=True)
        spline = SimpleNamespace(control_count=2)
        settings = SimpleNamespace(start_speed=0., minimum_speed=.5, tolerance=1e-6)
        trajectory = SimpleNamespace(to_path=lambda stations: object())
        profile = SimpleNamespace(converged=False, status="legacy-unsupported", diagnostics={"detail": "legacy"}, iterations=0)
        with patch.object(seeds, "_reference_curve", return_value=SimpleNamespace(source_station=np.array([0., 1.]))), \
             patch.object(seeds, "OffsetTrajectory", return_value=trajectory), \
             patch.object(seeds, "fixed_path_energy_profile", return_value=profile) as fixed:
            seed, diagnostics = seeds.build_profile_line_seed(track, object(), object(), np.array([0., 1.]), spline, settings)
        self.assertIsNone(seed)
        self.assertEqual(diagnostics["reason"], "profile-not-converged")
        self.assertEqual(fixed.call_args.kwargs["boundary"], "flying")

    def test_open_boundary_uses_explicit_floor_or_start_speed(self):
        track = SimpleNamespace(is_closed=False)
        spline = SimpleNamespace(control_count=2)
        settings = SimpleNamespace(start_speed=.2, minimum_speed=.5, tolerance=1e-6)
        trajectory = SimpleNamespace(to_path=lambda stations: object())
        profile = SimpleNamespace(converged=False, status="failed", diagnostics={}, iterations=0)
        with patch.object(seeds, "_reference_curve", return_value=SimpleNamespace(source_station=np.array([0., 1.]))), \
             patch.object(seeds, "OffsetTrajectory", return_value=trajectory), \
             patch.object(seeds, "fixed_path_energy_profile", return_value=profile) as fixed:
            seeds.build_profile_line_seed(track, object(), object(), np.array([0., 1.]), spline, settings)
        self.assertEqual(fixed.call_args.kwargs["boundary"], "rolling")
        self.assertEqual(fixed.call_args.kwargs["start_speed"], .5)


if __name__ == "__main__":
    unittest.main()
