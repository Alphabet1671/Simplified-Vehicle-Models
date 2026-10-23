"""Opt-in spline coefficient coordinate scaling regressions."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


def _surface():
    speed = np.linspace(.5, 40., 81)
    curvature = np.linspace(-.3, .3, 61)
    return CapabilitySurface(speed, curvature, np.full((81, 61), 3.),
                             np.full((81, 61), -8.), np.full(61, 25.), "scale-test")


def _casadi():
    try:
        import casadi
    except ImportError:
        return False
    return True


@unittest.skipUnless(_casadi(), "CasADi is required")
class SplineCoefficientScalingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        track = load_track(_write_circle_track(Path(self.tmp.name), radius=40., nodes=72,
                                               half_width=3.))
        self.track = replace(track, is_closed=False)
        self.corridor = corridor_from_track(self.track)
        self.spline = OffsetSpline(self.track, control_spacing=20.)
        self.mesh = np.array([0., self.track.lap_length])
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        self.settings = QssSettings(start_speed=10., maximum_iterations=4, tolerance=1.e-6)

    def tearDown(self):
        self.tmp.cleanup()

    def test_invalid_scale_rejected_before_build(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
        invalid = [True, np.bool_(False), 0, -1., float("nan"), float("inf"),
                   1 + 2j, "0.1", None, np.array([.1]), 10**10000]
        with patch("simplified_vehicle_models.solvers.qss_spline_ocp.build_spline_ocp",
                   side_effect=AssertionError("build called")) as build:
            for index, value in enumerate(invalid):
                with self.subTest(index=index), self.assertRaises(ValueError):
                    solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                      self.spline, self.settings,
                                      coefficient_coordinate_scale=value)
            build.assert_not_called()

    def test_actual_solver_api_preserves_physical_problem_and_solution(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            build_spline_ocp, solve_spline_line,
        )
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh
        mesh = build_mesh(self.track, 20.)
        built = build_spline_ocp(self.track, self.corridor, _surface(), mesh,
                                 self.spline, self.settings)
        self.mesh = mesh
        controls, nodes = built["control_count"], built["mesh"].size
        seed = np.zeros(int(built["nlp"]["x"].numel()))
        seed[:controls] = .1  # nonzero constant offset, independent of scale
        seed[controls:controls + nodes] = 10.
        base_f = ca.Function("api_base_f", [built["nlp"]["x"]], [built["nlp"]["f"]])
        base_g = ca.Function("api_base_g", [built["nlp"]["x"]], [built["nlp"]["g"]])
        outputs = []
        for scale in (None, 1., .1, 2.):
            captures = []
            def fake_nlpsol(_name, _plugin, nlp, _options):
                xarg = nlp["x"]
                f_eval = ca.Function("captured_f", [xarg], [nlp["f"]])
                g_eval = ca.Function("captured_g", [xarg], [nlp["g"]])
                class Solver:
                    def __call__(self, **kwargs):
                        z = kwargs["x0"]
                        captures.append({**{k: np.asarray(v, dtype=float).ravel().copy()
                                            for k, v in kwargs.items()
                                            if k in ("x0", "lbx", "ubx")},
                                         "f": float(f_eval(z)),
                                         "g": np.asarray(g_eval(z)).ravel().copy()})
                        return {"x": z, "f": captures[-1]["f"], "g": captures[-1]["g"],
                                "lam_x": np.zeros(len(z)), "lam_g": np.zeros(len(kwargs["lbg"]))}
                    def stats(self):
                        return {"return_status": "Solve_Succeeded", "iter_count": 0}
                return Solver()
            with patch("casadi.nlpsol", side_effect=fake_nlpsol):
                solve_kwargs = {} if scale is None else {"coefficient_coordinate_scale": scale}
                solution = solve_spline_line(
                    self.track, self.corridor, _surface(), self.mesh, self.spline,
                    self.settings, seed=seed, **solve_kwargs,
                )
            self.assertTrue(captures)
            captured = captures[0]
            center = np.r_[np.zeros(controls), np.full(nodes, 20.), np.zeros(2*nodes-1)]
            physical = np.r_[solution.coefficients, solution.speed,
                             solution.acceleration, solution.midpoint_acceleration]
            np.testing.assert_allclose(physical, seed, atol=1.e-12, rtol=0)
            requested = 1. if scale is None else scale
            effective = np.r_[np.full(controls, requested), np.full(nodes, 10.),
                              np.full(2*nodes-1, 5.)]
            np.testing.assert_allclose(captured["x0"], (seed-center)/effective,
                                       atol=1.e-12, rtol=0)
            np.testing.assert_allclose(center + effective*captured["lbx"], built["x_lower"],
                                       atol=1.e-12, rtol=0)
            np.testing.assert_allclose(center + effective*captured["ubx"], built["x_upper"],
                                       atol=1.e-12, rtol=0)
            np.testing.assert_allclose(captured["f"], float(base_f(physical)),
                                       rtol=1.e-12, atol=1.e-12)
            np.testing.assert_allclose(captured["g"], np.asarray(base_g(physical)).ravel(),
                                       rtol=1.e-12, atol=1.e-12)
            np.testing.assert_allclose(solution.objective_value, captured["f"],
                                       rtol=1.e-12, atol=1.e-12)
            requested = 1. if scale is None else scale
            self.assertEqual(solution.coefficient_coordinate_scale, requested)
            self.assertEqual(solution.coefficient_coordinate_scale_applied, requested != 1.)
            outputs.append(solution)
        for solution in outputs[1:]:
            np.testing.assert_allclose(solution.coefficients, outputs[0].coefficients)
            np.testing.assert_allclose(solution.speed, outputs[0].speed)
            np.testing.assert_allclose(solution.acceleration, outputs[0].acceleration)
            np.testing.assert_allclose(solution.midpoint_acceleration,
                                       outputs[0].midpoint_acceleration)
            self.assertAlmostEqual(solution.lap_time, outputs[0].lap_time, places=12)

    def test_free_failure_metadata_records_scaled_dual_coordinates(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            SplineSolveError, build_spline_ocp, solve_spline_line,
        )
        built = build_spline_ocp(self.track, self.corridor, _surface(), self.mesh,
                                 self.spline, self.settings)
        controls, nodes = built["control_count"], self.mesh.size
        seed = np.zeros(int(built["nlp"]["x"].numel()))
        seed[:controls] = .1
        seed[controls:controls+nodes] = 10.
        def fake_nlpsol(_name, _plugin, nlp, _options):
            f_eval = ca.Function("failure_f", [nlp["x"]], [nlp["f"]])
            g_eval = ca.Function("failure_g", [nlp["x"]], [nlp["g"]])
            class Solver:
                def __call__(self, **kwargs):
                    z = kwargs["x0"]
                    return {"x": z, "f": float(f_eval(z)), "g": np.asarray(g_eval(z)).ravel(),
                            "lam_x": np.ones(len(z)), "lam_g": np.ones(len(kwargs["lbg"]))}
                def stats(self):
                    return {"return_status": "Maximum_Iterations_Exceeded", "iter_count": 1}
            return Solver()
        with patch("casadi.nlpsol", side_effect=fake_nlpsol):
            with self.assertRaises(SplineSolveError) as caught:
                solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                  self.spline, self.settings, seed=seed,
                                  coefficient_coordinate_scale=.1)
        error = caught.exception
        self.assertEqual(error.diagnostics["coefficient_coordinate_scale"], .1)
        self.assertTrue(error.diagnostics["coefficient_coordinate_scale_applied"])
        self.assertEqual(error.diagnostics["dual_x_coordinates"], "scaled_decision")
        np.testing.assert_allclose(error.failed_values, seed, atol=1.e-12, rtol=0)

    def test_fixed_failure_metadata_reports_requested_but_inactive_scale(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import SplineSolveError, solve_spline_line
        fixed = np.full(self.spline.control_count, .1)
        def fake_solver(*args):
            class Solver:
                def __call__(self, **kwargs):
                    n = len(kwargs["x0"])
                    return {"x": kwargs["x0"], "g": kwargs["lbg"], "f": 1.,
                            "lam_x": np.zeros(n), "lam_g": np.zeros(len(kwargs["lbg"]))}
                def stats(self): return {"return_status": "Maximum_Iterations_Exceeded", "iter_count": 1}
            return Solver()
        with patch("casadi.nlpsol", side_effect=fake_solver):
            with self.assertRaises(SplineSolveError) as caught:
                solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                  self.spline, self.settings, fixed_coefficients=fixed,
                                  coefficient_coordinate_scale=.1)
        diag = caught.exception.diagnostics
        self.assertEqual(diag["coefficient_coordinate_scale"], .1)
        self.assertFalse(diag["coefficient_coordinate_scale_applied"])
        self.assertEqual(diag["dual_x_coordinates"], "scaled_active_decision")
        self.assertEqual(diag["dual_x_full_decision_indices"],
                         list(range(fixed.size, fixed.size + caught.exception.dual_x.size)))


if __name__ == "__main__":
    unittest.main()
