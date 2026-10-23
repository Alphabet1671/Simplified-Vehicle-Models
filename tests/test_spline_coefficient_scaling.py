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

    def test_scaled_actual_nlp_has_only_coefficient_coordinate_change(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp, _decision_scaling
        from simplified_vehicle_models.solvers.qss_line_ocp import build_mesh
        mesh = build_mesh(self.track, 20.)
        built = build_spline_ocp(self.track, self.corridor, _surface(), mesh,
                                 self.spline, self.settings)
        n, controls = mesh.size, built["control_count"]
        center, legacy = _decision_scaling(controls, n)
        physical = np.zeros(int(built["nlp"]["x"].numel()))
        physical[:controls] = .1 + .02 * np.sin(np.arange(controls))
        physical[controls:controls+n] = 10.
        for scale in (1., .1, 2.):
            effective = legacy.copy()
            effective[:controls] = scale
            sx = ca.MX.sym("scale_compare", physical.size)
            px = ca.DM(center) + ca.DM(effective) * sx
            sf = ca.substitute(built["nlp"]["f"], built["nlp"]["x"], px)
            sg = ca.substitute(built["nlp"]["g"], built["nlp"]["x"], px)
            f = ca.Function("scale_f_" + str(scale).replace(".", "_"), [sx], [sf])
            g = ca.Function("scale_g_" + str(scale).replace(".", "_"), [sx], [sg])
            z = (physical - center) / effective
            np.testing.assert_allclose(float(f(z)), float(ca.Function(
                "base_f_" + str(scale).replace(".", "_"), [built["nlp"]["x"]],
                [built["nlp"]["f"]])(physical)), rtol=1.e-12, atol=1.e-12)
            np.testing.assert_allclose(np.asarray(g(z)).ravel(), np.asarray(ca.Function(
                "base_g_" + str(scale).replace(".", "_"), [built["nlp"]["x"]],
                [built["nlp"]["g"]])(physical)).ravel(), rtol=1.e-12, atol=1.e-12)
            np.testing.assert_allclose(center + effective * ((built["x_lower"]-center)/effective),
                                       built["x_lower"])
            np.testing.assert_allclose(center + effective * ((built["x_upper"]-center)/effective),
                                       built["x_upper"])

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
