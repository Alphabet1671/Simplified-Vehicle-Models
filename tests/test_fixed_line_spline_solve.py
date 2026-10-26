"""Focused regressions for opt-in fixed spline coefficients."""
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
                             np.full((81, 61), -8.), np.full(61, 25.), "fixed-test")


def _casadi():
    try:
        import casadi
    except ImportError:
        return False
    return True


@unittest.skipUnless(_casadi(), "CasADi is required")
class FixedLineSplineSolveTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        track = load_track(_write_circle_track(Path(self.tmp.name), radius=40., nodes=72,
                                               half_width=3.))
        self.track = replace(track, is_closed=False)
        base = corridor_from_track(self.track)
        self.corridor = base
        self.spline = OffsetSpline(self.track, control_spacing=20.)
        self.mesh = np.array([0., self.track.lap_length])
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        self.settings = QssSettings(start_speed=10., maximum_iterations=80, tolerance=1.e-6)

    def tearDown(self):
        self.tmp.cleanup()

    def test_invalid_values_rejected_before_solver_and_never_clipped(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
        valid = np.full(self.spline.control_count, .1)
        invalid = [np.zeros(valid.size - 1), np.full(valid.size, np.nan),
                   np.full(valid.size, 100.), np.full(valid.size, True),
                   [True, *([.1] * (valid.size - 1))],
                   [np.bool_(True), *([.1] * (valid.size - 1))]]
        with patch("casadi.nlpsol") as nlpsol:
            for values in invalid:
                with self.subTest(values=values), self.assertRaises(ValueError):
                    solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                      self.spline, self.settings, fixed_coefficients=values)
            nlpsol.assert_not_called()

    def test_none_keeps_default_solution_path_and_layout(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
        default = solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                    self.spline, self.settings)
        explicit_none = solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                          self.spline, self.settings, fixed_coefficients=None)
        np.testing.assert_array_equal(default.coefficients, explicit_none.coefficients)
        np.testing.assert_array_equal(default.speed, explicit_none.speed)
        self.assertEqual(default.objective_value, explicit_none.objective_value)
        self.assertIsNone(default.fixed_coefficients)
        self.assertTrue(default.line_optimized)

    def test_fixed_nonzero_coefficients_solve_and_are_exact_readonly_provenance(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
        fixed = np.full(self.spline.control_count, .1)
        seed = np.zeros(self.spline.control_count + 3 * self.mesh.size - 1)
        solution = solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                     self.spline, self.settings, seed=seed,
                                     fixed_coefficients=fixed)
        np.testing.assert_array_equal(solution.coefficients, fixed)
        np.testing.assert_array_equal(solution.fixed_coefficients, fixed)
        self.assertFalse(solution.fixed_coefficients.flags.writeable)
        self.assertFalse(solution.line_optimized)
        self.assertEqual(solution.status, "Solve_Succeeded")

    def test_substituted_objective_and_constraint_rows_match_full_physical_point(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh
        fixed = np.full(self.spline.control_count, .1)
        mesh = build_mesh(self.track, 20.)
        built = build_spline_ocp(self.track, self.corridor, _surface(), mesh,
                                 self.spline, self.settings)
        full_x = built["nlp"]["x"]
        center, scale = __import__(
            "simplified_vehicle_models.solvers.qss_spline_ocp", fromlist=["_decision_scaling"],
        )._decision_scaling(built["control_count"], mesh.size)
        active = ca.MX.sym("active", int(full_x.numel()) - fixed.size)
        physical = ca.vertcat(ca.DM(fixed), ca.DM(center[fixed.size:])
                              + ca.DM(scale[fixed.size:]) * active)
        reduced_f = ca.substitute(built["nlp"]["f"], full_x, physical)
        reduced_g = ca.substitute(built["nlp"]["g"], full_x, physical)
        values = np.zeros(int(full_x.numel()))
        values[:fixed.size] = fixed
        nodes = mesh.size
        values[fixed.size:fixed.size + nodes] = 10.
        values[fixed.size + nodes:] = 0.
        scaled = (values[fixed.size:] - center[fixed.size:]) / scale[fixed.size:]
        f_full = ca.Function("fixed_full_f", [full_x], [built["nlp"]["f"]])
        g_full = ca.Function("fixed_full_g", [full_x], [built["nlp"]["g"]])
        f_reduced = ca.Function("fixed_reduced_f", [active], [reduced_f])
        g_reduced = ca.Function("fixed_reduced_g", [active], [reduced_g])
        self.assertAlmostEqual(float(f_full(values)), float(f_reduced(scaled)), places=11)
        np.testing.assert_allclose(np.asarray(g_full(values)), np.asarray(g_reduced(scaled)),
                                   rtol=1.e-12, atol=1.e-12)

    def test_fixed_failure_reports_full_values_and_active_dual_mapping(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import SplineSolveError, solve_spline_line
        fixed = np.full(self.spline.control_count, .1)
        def fake_solver(*args):
            class Solver:
                def __call__(self, **kwargs):
                    self.kwargs = kwargs
                    n = len(kwargs["x0"])
                    return {"x": kwargs["x0"], "g": kwargs["lbg"], "f": 1.,
                            "lam_x": np.zeros(n), "lam_g": np.zeros(len(kwargs["lbg"]))}
                def stats(self): return {"return_status": "Maximum_Iterations_Exceeded", "iter_count": 1}
            return Solver()
        with patch("casadi.nlpsol", side_effect=fake_solver):
            with self.assertRaises(SplineSolveError) as caught:
                solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                  self.spline, self.settings, fixed_coefficients=fixed)
        error = caught.exception
        full = self.spline.control_count + 3 * self.mesh.size - 1
        self.assertEqual(error.failed_values.size, full)
        np.testing.assert_array_equal(error.failed_values[:fixed.size], fixed)
        self.assertEqual(error.dual_x.size, full - fixed.size)
        self.assertEqual(error.diagnostics["dual_x_coordinates"], "scaled_active_decision")
        self.assertEqual(error.diagnostics["dual_x_full_decision_indices"],
                         list(range(fixed.size, full)))


if __name__ == "__main__":
    unittest.main()
