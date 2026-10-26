"""Actual solve-API regressions for energy-defect coordinate scaling."""
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
                             np.full((81, 61), -8.), np.full(61, 25.), "energy-scale")


@unittest.skipUnless(__import__("importlib").util.find_spec("casadi"), "CasADi required")
class EnergyDefectScalingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        track = load_track(_write_circle_track(Path(self.tmp.name), radius=40., nodes=72,
                                               half_width=3.))
        self.track = replace(track, is_closed=False)
        self.corridor = corridor_from_track(self.track)
        self.spline = OffsetSpline(self.track, control_spacing=20.)
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        self.settings = QssSettings(start_speed=10., maximum_iterations=4, tolerance=1.e-6)
        self.mesh = np.array([0., 10., float(self.track.lap_length)])

    def tearDown(self):
        self.tmp.cleanup()

    def _mock_solver(self, *, status="Solve_Succeeded", corrupt_energy=False, raw_dual=2.):
        import casadi as ca
        capture = []
        def fake_nlpsol(_name, _plugin, nlp, opts):
            gf = ca.Function("energy_capture_g", [nlp["x"]], [nlp["g"]])
            ff = ca.Function("energy_capture_f", [nlp["x"]], [nlp["f"]])
            class Solver:
                def __call__(self, **kw):
                    z = kw["x0"]
                    g = np.asarray(gf(z)).ravel()
                    if corrupt_energy:
                        g[0] = kw["lbg"][0] + 2.e-6
                    capture.append({
                        **{k: np.array(v, copy=True) for k, v in kw.items()},
                        "nlp_g_at_x0": np.asarray(gf(z)).ravel(),
                        "nlp_f_at_x0": float(ff(z)),
                        "nlp_g_expression": str(nlp["g"]),
                        "nlp_f_expression": str(nlp["f"]),
                        "options": dict(opts),
                    })
                    return {"x": z, "f": float(ff(z)), "g": g,
                            "lam_x": np.arange(len(z), dtype=float),
                            "lam_g": np.full(len(g), raw_dual)}
                def stats(self): return {"return_status": status, "iter_count": 1}
            return Solver()
        return capture, patch("casadi.nlpsol", side_effect=fake_nlpsol)

    def test_builder_rows_are_actual_hs_rows_and_cell_bounds(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp
        built = build_spline_ocp(self.track, self.corridor, _surface(), self.mesh,
                                 self.spline, self.settings)
        indices = built["energy_defect_indices"]
        np.testing.assert_array_equal(indices, [0, 66])
        self.assertEqual(built["g_lower"][-1], 0.)  # open-start equality is separate

    def test_legacy_default_and_cell_scaling_capture_actual_solver_nlp(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp, solve_spline_line
        built = build_spline_ocp(self.track, self.corridor, _surface(), self.mesh,
                                 self.spline, self.settings)
        size = int(built["nlp"]["x"].numel())
        seed = np.zeros(size); seed[:built["control_count"]] = .1
        seed[built["control_count"]:built["control_count"]+3] = 10.
        seed[built["control_count"]+3:built["control_count"]+6] = [1., 2., -1.]
        seed[built["control_count"]+6:] = [.5, -.25]
        base_g = ca.Function("canonical_energy_g", [built["nlp"]["x"]], [built["nlp"]["g"]])
        captures = []
        expected_g = np.asarray(base_g(seed)).ravel()
        for kwargs in ({}, {"energy_defect_scaling": "legacy_reference_speed_v1"},
                       {"energy_defect_scaling": "cell_length_v1"}):
            capture, mock = self._mock_solver()
            with mock:
                try:
                    solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                      self.spline, self.settings, seed=seed, **kwargs)
                except Exception as exc:
                    from simplified_vehicle_models.solvers.qss_spline_ocp import SplineSolveError
                    if not isinstance(exc, SplineSolveError):
                        raise
            captures.append(capture[0])
        np.testing.assert_array_equal(captures[0]["lbg"], built["g_lower"])
        np.testing.assert_array_equal(captures[1]["lbg"], captures[0]["lbg"])
        np.testing.assert_array_equal(captures[0]["x0"], captures[1]["x0"])
        np.testing.assert_array_equal(captures[0]["ubg"], built["g_upper"])
        np.testing.assert_array_equal(captures[0]["nlp_g_at_x0"], expected_g)
        np.testing.assert_array_equal(captures[1]["nlp_g_at_x0"], expected_g)
        np.testing.assert_array_equal(captures[0]["nlp_f_at_x0"], captures[1]["nlp_f_at_x0"])
        self.assertEqual(captures[0]["nlp_g_expression"], captures[1]["nlp_g_expression"])
        self.assertEqual(captures[0]["nlp_f_expression"], captures[1]["nlp_f_expression"])
        self.assertEqual(captures[0]["options"], captures[1]["options"])
        np.testing.assert_array_equal(captures[0]["x0"], captures[1]["x0"])
        factors = np.ones(built["g_lower"].size)
        factors[built["energy_defect_indices"]] = 10./np.diff(self.mesh)
        np.testing.assert_allclose(captures[2]["lbg"], factors*built["g_lower"])
        np.testing.assert_allclose(captures[2]["ubg"], factors*built["g_upper"])
        np.testing.assert_allclose(captures[2]["nlp_g_at_x0"], factors*expected_g)
        np.testing.assert_allclose(captures[2]["nlp_f_at_x0"], captures[0]["nlp_f_at_x0"])
        self.assertEqual(captures[2]["x0"].tolist(), captures[0]["x0"].tolist())
        # Every non-HS row, including seam/open-start equality, stays unchanged.
        non_energy = np.ones(factors.size, dtype=bool)
        non_energy[built["energy_defect_indices"]] = False
        np.testing.assert_array_equal(captures[2]["lbg"][non_energy], built["g_lower"][non_energy])
        np.testing.assert_array_equal(captures[2]["ubg"][non_energy], built["g_upper"][non_energy])

    def test_builder_energy_rows_are_only_hs_rows_for_open_and_closed(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp
        for closed in (False, True):
            track = self.track if not closed else replace(self.track, is_closed=True)
            corridor = corridor_from_track(track)
            spline = OffsetSpline(track, control_spacing=20.)
            built = build_spline_ocp(track, corridor, _surface(), self.mesh,
                                     spline, self.settings)
            ids = built["energy_defect_indices"]
            self.assertEqual(ids.size, self.mesh.size - 1)
            mask = built["g_lower"] == built["g_upper"]
            self.assertEqual(np.count_nonzero(mask), self.mesh.size)
            self.assertEqual(ids.size + 1, np.count_nonzero(mask))
            self.assertTrue(np.all(mask[ids]))
            seam_or_start = np.flatnonzero(mask & ~np.isin(np.arange(mask.size), ids))
            self.assertEqual(seam_or_start.size, 1)

    def test_failed_duals_are_recovered_but_solver_warmstart_stays_raw(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import SplineSolveError, solve_spline_line
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings
        self.settings = QssSettings(start_speed=10., maximum_iterations=4,
                                    tolerance=1.e-6, tolerance_homotopy=True)
        capture, mock = self._mock_solver(status="Maximum_Iterations_Exceeded", raw_dual=3.)
        with mock, self.assertRaises(SplineSolveError) as caught:
            solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                              self.spline, self.settings, energy_defect_scaling="cell_length_v1")
        err = caught.exception
        factors = np.ones(err.dual_g.size)
        factors[[0, 66]] = [10./10., 10./(self.track.lap_length-10.)]
        np.testing.assert_allclose(err.dual_g, 3.*factors)
        self.assertEqual(err.diagnostics["dual_g_coordinates"], "original_constraint_residuals")

    def test_failed_result_reports_original_g_duals_and_keeps_lam_x(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import SplineSolveError, solve_spline_line
        capture, mock = self._mock_solver(status="Maximum_Iterations_Exceeded", raw_dual=3.)
        with mock, self.assertRaises(SplineSolveError) as caught:
            solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                              self.spline, self.settings,
                              energy_defect_scaling="cell_length_v1")
        error = caught.exception
        factors = np.ones(error.dual_g.size)
        factors[[0, 66]] = 10./np.diff(self.mesh)
        np.testing.assert_allclose(error.dual_g, 3.*factors)
        np.testing.assert_array_equal(error.dual_x, np.arange(error.dual_x.size))
        np.testing.assert_allclose(error.constraint_values,
                                   capture[0]["nlp_g_at_x0"] / factors)

    def test_homotopy_warmstart_uses_raw_solver_dual(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
        raw_lam_g = []
        seen = []
        def fake_nlpsol(_name, _plugin, nlp, _opts):
            gfun = ca.Function("warm_capture_g", [nlp["x"]], [nlp["g"]])
            ffun = ca.Function("warm_capture_f", [nlp["x"]], [nlp["f"]])
            class Solver:
                def __call__(self, **kw):
                    seen.append(kw)
                    z = kw["x0"]
                    g = np.asarray(gfun(z)).ravel()
                    dual = np.arange(1., g.size + 1.)
                    raw_lam_g.append(dual.copy())
                    return {"x": z, "f": float(ffun(z)), "g": g,
                            "lam_x": np.arange(z.size, dtype=float), "lam_g": dual}
                def stats(self): return {"return_status": "Solve_Succeeded", "iter_count": 1}
            return Solver()
        with patch("simplified_vehicle_models.solvers.qss_spline_ocp._tolerance_schedule",
                   return_value=[1.e-3, 1.e-6]), patch("casadi.nlpsol", side_effect=fake_nlpsol):
            try:
                solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                  self.spline, replace(self.settings, tolerance_homotopy=True),
                                  energy_defect_scaling="cell_length_v1")
            except Exception as exc:
                # This fake's zero seed may fail the strict final gate; warm-start
                # arguments are captured before that gate and are the assertion target.
                from simplified_vehicle_models.solvers.qss_spline_ocp import SplineSolveError
                if not isinstance(exc, SplineSolveError):
                    raise
        self.assertEqual(len(seen), 2)
        np.testing.assert_array_equal(seen[1]["lam_g0"], raw_lam_g[0])

    def test_scaled_residual_cannot_hide_original_tolerance_failure(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            SplineSolveError, build_spline_ocp, solve_spline_line,
        )
        built = build_spline_ocp(self.track, self.corridor, _surface(), self.mesh,
                                 self.spline, self.settings)
        control_count = built["control_count"]
        hsid = int(built["energy_defect_indices"][1])
        long_cell_factor = 10.0 / (self.track.lap_length - 10.0)
        seed = np.r_[np.zeros(control_count), np.full(3, 10.0), np.zeros(5)]
        canonical_g = ca.Function("tol_gate_canonical_g", [built["nlp"]["x"]],
                                  [built["nlp"]["g"]])
        fake_g = np.asarray(canonical_g(seed)).ravel()
        fake_g[hsid] = long_cell_factor * 1.5e-6
        self.assertLess(fake_g[hsid], 1.e-6)
        called = []
        def succeeds_with_bad_physical_hs(_name, _plugin, nlp, _opts):
            gfun = ca.Function("tol_gate_scaled_g", [nlp["x"]], [nlp["g"]])
            ffun = ca.Function("tol_gate_scaled_f", [nlp["x"]], [nlp["f"]])
            class Solver:
                def __call__(self, **kw):
                    called.append(True)
                    return {"x": kw["x0"], "f": float(ffun(kw["x0"])), "g": fake_g,
                            "lam_x": np.zeros(kw["x0"].size), "lam_g": np.zeros(fake_g.size)}
                def stats(self): return {"return_status": "Solve_Succeeded", "iter_count": 1}
            return Solver()
        with patch("casadi.nlpsol", side_effect=succeeds_with_bad_physical_hs):
            with self.assertRaises(SplineSolveError) as caught:
                solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                  self.spline, self.settings, seed=seed,
                                  energy_defect_scaling="cell_length_v1")
        self.assertTrue(called)
        err = caught.exception
        self.assertEqual(err.diagnostics["return_status"], "Solve_Succeeded")
        self.assertAlmostEqual(err.diagnostics["physical_max_constraint_violation"], 1.5e-6)
        self.assertAlmostEqual(err.constraint_values[hsid], fake_g[hsid] / long_cell_factor)

    def test_fixed_coefficient_failure_recovers_original_g_and_duals(self):
        import casadi as ca
        from simplified_vehicle_models.solvers.qss_spline_ocp import (
            SplineSolveError, build_spline_ocp, solve_spline_line,
        )
        built = build_spline_ocp(self.track, self.corridor, _surface(), self.mesh,
                                 self.spline, self.settings)
        control_count = built["control_count"]
        seed = np.r_[np.full(control_count, .05), np.full(3, 10.0),
                     np.array([1., 2., -1.]), np.array([.5, -.25])]
        canonical_g = ca.Function("fixed_canonical_g", [built["nlp"]["x"]], [built["nlp"]["g"]])
        failed_seed = seed.copy()
        failed_seed[:control_count] = 0.0
        expected_g = np.asarray(canonical_g(failed_seed)).ravel()
        capture = []
        raw_dual = 4.0
        def fail_fixed_solver(_name, _plugin, nlp, _opts):
            gfun = ca.Function("fixed_scaled_g", [nlp["x"]], [nlp["g"]])
            ffun = ca.Function("fixed_scaled_f", [nlp["x"]], [nlp["f"]])
            class Solver:
                def __call__(self, **kw):
                    capture.append(kw)
                    z = kw["x0"]
                    return {"x": z, "f": float(ffun(z)), "g": np.asarray(gfun(z)).ravel(),
                            "lam_x": np.zeros(z.size), "lam_g": np.full(kw["lbg"].size, raw_dual)}
                def stats(self): return {"return_status": "Maximum_Iterations_Exceeded", "iter_count": 1}
            return Solver()
        with patch("casadi.nlpsol", side_effect=fail_fixed_solver):
            with self.assertRaises(SplineSolveError) as caught:
                solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                  self.spline, self.settings, seed=seed,
                                  fixed_coefficients=np.zeros(control_count),
                                  energy_defect_scaling="cell_length_v1")
        err = caught.exception
        self.assertEqual(err.diagnostics["layout"]["active_decision_size"],
                         err.diagnostics["layout"]["full_decision_size"] - control_count)
        self.assertEqual(err.failed_values.size, err.diagnostics["layout"]["full_decision_size"])
        np.testing.assert_array_equal(err.failed_values[:control_count], np.zeros(control_count))
        np.testing.assert_allclose(err.constraint_values, expected_g)
        factors = np.ones(expected_g.size)
        factors[built["energy_defect_indices"]] = 10.0 / np.diff(self.mesh)
        np.testing.assert_allclose(err.dual_g, raw_dual * factors)

    def test_nonfinite_cell_scaling_factor_rejected_before_solver_construction(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp, solve_spline_line
        built = build_spline_ocp(self.track, self.corridor, _surface(), self.mesh,
                                 self.spline, self.settings)
        extreme_mesh = np.array([0., 1.e-310, float(self.track.lap_length)])
        seed = np.r_[np.zeros(built["control_count"]), np.full(3, 10.0), np.zeros(5)]
        def return_extreme_mesh(*_args, **_kwargs):
            changed = dict(built)
            changed["mesh"] = extreme_mesh
            return changed
        with patch("simplified_vehicle_models.solvers.qss_spline_ocp.build_spline_ocp",
                   side_effect=return_extreme_mesh), patch("casadi.nlpsol") as solver:
            with np.errstate(over="ignore"), self.assertRaisesRegex(ValueError, "scaling factors"):
                solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                  self.spline, self.settings, seed=seed,
                                  energy_defect_scaling="cell_length_v1")
        solver.assert_not_called()

    def test_invalid_mode_rejected_before_build(self):
        from simplified_vehicle_models.solvers.qss_spline_ocp import solve_spline_line
        class Derived(str): pass
        for bad in (None, 0, True, "bad", Derived("cell_length_v1")):
            with patch("simplified_vehicle_models.solvers.qss_spline_ocp.build_spline_ocp",
                       side_effect=AssertionError("build called")) as build:
                with self.assertRaises(ValueError):
                    solve_spline_line(self.track, self.corridor, _surface(), self.mesh,
                                      self.spline, self.settings, energy_defect_scaling=bad)
                build.assert_not_called()


if __name__ == "__main__":
    unittest.main()
