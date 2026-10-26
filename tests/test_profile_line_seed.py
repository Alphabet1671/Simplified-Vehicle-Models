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

    def test_tiny_variable_bound_excess_is_rejected_without_tolerance_slack(self):
        surface = SimpleNamespace(representation="signed_transient_normalized_v1",
                                  domain_speed_values=np.array([4.]),
                                  acceleration_min=np.array([[-2.]]), acceleration_max=np.array([[3.]]))
        spline = SimpleNamespace(control_count=2)
        settings = SimpleNamespace(minimum_speed=.5)
        seed = np.array([0., 0., 1., 1., 0., 3.000000000001, 0.])
        accepted, reason = seeds._within_bounds(seed, surface, spline, np.array([0., 1.]), settings)
        self.assertFalse(accepted)
        self.assertIn("variable-bound-violation", reason)

    def test_control_projection_selector_rejects_unknown_methods(self):
        with self.assertRaisesRegex(ValueError, "control_projection"):
            seeds.build_profile_line_seed(None, None, None, np.array([0., 1.]),
                                          SimpleNamespace(control_count=2), None,
                                          control_projection="clipped")

    def test_bounded_projection_preserves_energy_and_uses_hs_controls(self):
        track = SimpleNamespace(is_closed=True)
        spline = SimpleNamespace(control_count=2)
        settings = SimpleNamespace(start_speed=0., minimum_speed=.5, tolerance=1e-6)
        mesh = np.array([0., 1., 2.])
        surface = SimpleNamespace(acceleration_min=np.array([-.1]), acceleration_max=np.array([.8]),
                                  speed=np.array([.5, 20.]))
        def geometry(stations):
            stations = np.asarray(stations)
            return SimpleNamespace(metric=1.0 + stations)
        trajectory = SimpleNamespace(
            to_path=lambda stations: object(),
            station_map=SimpleNamespace(to_path=lambda stations: np.asarray(stations)),
            evaluate=geometry,
        )
        profile = SimpleNamespace(converged=True, status="ok", diagnostics={}, iterations=1,
                                  lap_time=2., distance=np.array([0., 2.]), speed=np.array([2., np.sqrt(6.)]))
        bounded = np.array([0., 0., 0., .5, .3])
        projection_report = {"available": True, "maximum_hs_energy_defect": 0.}
        with patch.object(seeds, "_reference_curve", return_value=SimpleNamespace(source_station=mesh)), \
             patch.object(seeds, "OffsetTrajectory", return_value=trajectory), \
             patch.object(seeds, "fixed_path_energy_profile", return_value=profile), \
             patch("simplified_vehicle_models.solvers.energy_seed_projection.project_energy_seed_controls",
                   return_value=(bounded, projection_report)) as projector, \
             patch.object(seeds, "_within_bounds", return_value=(True, None)):
            seed, diagnostics = seeds.build_profile_line_seed(
                track, object(), surface, mesh, spline, settings, control_projection="bounded_energy_l1")
        self.assertIsNotNone(seed)
        np.testing.assert_allclose(seed[2:5], [2., np.sqrt(5.), np.sqrt(6.)])
        np.testing.assert_allclose(seed[5:], bounded)
        self.assertEqual(diagnostics["projection_method"], "bounded_energy_l1")
        self.assertEqual(diagnostics["nested_control_projection_diagnostics"], projection_report)
        energy = np.array([4., 5., 6.])
        node_metric = np.array([1., 2., 3.])
        mid_metric = np.array([1.5, 2.5])
        expected_residual = np.diff(energy) - np.diff(mesh) / 6.0 * (
            2.0 * node_metric[:-1] * bounded[:2]
            + 8.0 * mid_metric * bounded[3:]
            + 2.0 * node_metric[1:] * bounded[1:3])
        self.assertAlmostEqual(diagnostics["maximum_hs_energy_defect"],
                               float(np.max(np.abs(expected_residual))))
        self.assertLess(diagnostics["maximum_hs_energy_defect"], 1e-12)
        self.assertTrue(diagnostics["projection_attempted"])
        self.assertTrue(diagnostics["projection_used_by_seed"])
        self.assertFalse(diagnostics["solver_feasibility_claimed"])
        np.testing.assert_allclose(diagnostics["midpoint_energy_minimum"], 4.5)
        self.assertFalse(diagnostics["midpoint_capability_checked"])
        self.assertTrue(projector.call_args.kwargs["settings"].periodic_endpoint)

    def test_bounded_projection_failure_does_not_fall_back(self):
        track = SimpleNamespace(is_closed=False)
        spline = SimpleNamespace(control_count=2)
        settings = SimpleNamespace(start_speed=0., minimum_speed=.5, tolerance=1e-6)
        mesh = np.array([0., 1.])
        trajectory = SimpleNamespace(to_path=lambda stations: object(),
                                     station_map=SimpleNamespace(to_path=lambda stations: np.asarray(stations)),
                                     evaluate=lambda stations: SimpleNamespace(metric=np.ones_like(np.asarray(stations))))
        profile = SimpleNamespace(converged=True, status="ok", diagnostics={}, iterations=1,
                                  distance=mesh, speed=np.array([2., 2.]))
        with patch.object(seeds, "_reference_curve", return_value=SimpleNamespace(source_station=mesh)), \
             patch.object(seeds, "OffsetTrajectory", return_value=trajectory), \
             patch.object(seeds, "fixed_path_energy_profile", return_value=profile), \
             patch("simplified_vehicle_models.solvers.energy_seed_projection.project_energy_seed_controls",
                   return_value=(None, {"reason": "infeasible"})):
            seed, diagnostics = seeds.build_profile_line_seed(
                track, object(), SimpleNamespace(acceleration_min=np.array([-1.]), acceleration_max=np.array([1.])),
                mesh, spline, settings, control_projection="bounded_energy_l1")
        self.assertIsNone(seed)
        self.assertEqual(diagnostics["reason"], "bounded-control-projection-failed")
        self.assertEqual(diagnostics["nested_control_projection_diagnostics"]["reason"], "infeasible")

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

    def test_unexpected_backend_error_is_normalized_to_diagnostics(self):
        track = SimpleNamespace(is_closed=False)
        spline = SimpleNamespace(control_count=2)
        settings = SimpleNamespace(start_speed=0., minimum_speed=.5, tolerance=1e-6)
        with patch.object(seeds, "_reference_curve", side_effect=RuntimeError("backend fixture failure")):
            seed, diagnostics = seeds.build_profile_line_seed(track, object(), object(), np.array([0., 1.]), spline, settings)
        self.assertIsNone(seed)
        self.assertEqual(diagnostics["reason"], "unsupported-or-invalid")
        self.assertIn("RuntimeError", diagnostics["error"])

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
