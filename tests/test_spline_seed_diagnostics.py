"""Focused tests for exact, non-solving spline seed diagnostics."""
from dataclasses import replace
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

try:
    import casadi as ca
except ImportError:
    ca = None

from simplified_vehicle_models.formulations.point_mass_capability import CapabilitySurface
from simplified_vehicle_models.solvers.spline_seed_diagnostics import inspect_spline_seed
from simplified_vehicle_models.tracks.corridor import corridor_from_track
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from simplified_vehicle_models.tracks.profile import load_track
from tests.test_track_width import _write_circle_track


def _surface():
    return CapabilitySurface(np.linspace(.5, 40., 81), np.linspace(-.3, .3, 61),
                            np.full((81, 61), 3.), np.full((81, 61), -8.),
                            np.full(61, 25.), "seed-diagnostic-test")


@unittest.skipUnless(ca, "CasADi required")
class SplineSeedDiagnosticsTest(unittest.TestCase):
    def synthetic(self):
        x = ca.MX.sym("x", 6)
        g = ca.vertcat(x[0], x[1], x[2])
        return {"nlp": {"x": x, "f": ca.dot(x, x), "g": g},
                "x_lower": np.full(6, -np.inf), "x_upper": np.full(6, np.inf),
                "g_lower": np.array([0., -np.inf, -1.]),
                "g_upper": np.array([np.inf, 2., 1.]), "count": 1,
                "control_count": 1, "mesh": np.array([0., 1.]),
                "node_family_indices": np.array([0, 2])}

    def test_exact_seed_families_ties_caps_and_readonly(self):
        built = self.synthetic()
        built["g_lower"] = np.array([-1., -1., -1.])
        built["g_upper"] = np.array([1., 1., 1.])
        seed = np.array([2., 0., 2., 0., 0., 0.]); seed.flags.writeable = False
        before = seed.copy()
        with patch("casadi.nlpsol", side_effect=AssertionError("must not solve")) as solver:
            report = inspect_spline_seed(built, seed, maximum_reported_rows=1)
        solver.assert_not_called()
        np.testing.assert_array_equal(seed, before)
        self.assertEqual(report["maximum_constraint_violation"], 1.)
        self.assertEqual(report["worst_constraint_rows"][0]["index"], 0)
        self.assertTrue(report["constraint_rows_truncated"])
        self.assertEqual(report["constraint_family_maxima"]["node_family"], 1.)
        self.assertEqual(report["unclassified_constraint_maximum"], 0.)
        self.assertFalse(report["accepted"])
        json.dumps(report, allow_nan=False)

    def test_one_sided_inf_and_nonfinite_paths(self):
        built = self.synthetic()
        report = inspect_spline_seed(built, np.zeros(6))
        row = report["worst_constraint_rows"]
        self.assertEqual(report["constraint_row_count"], 3)
        self.assertTrue(report["evaluation_complete"])
        self.assertFalse(report["policy_approved"])
        json.dumps(report, allow_nan=False)
        for invalid in ([False, 0, 0, 0, 0, 0],
                        np.array([True, 0, 0, 0, 0, 0], dtype=object),
                        np.zeros((2, 3)), np.zeros(5), np.array([np.nan, 0, 0, 0, 0, 0])):
            self.assertFalse(inspect_spline_seed(built, invalid)["finite_input"])
        self.assertFalse(inspect_spline_seed(built, np.zeros(6), maximum_reported_rows=False)["evaluation_complete"])
        broken = dict(built); broken["g_lower"] = np.array([np.nan, 0., 0.])
        self.assertFalse(inspect_spline_seed(broken, np.zeros(6))["evaluation_complete"])
        broken = dict(built); broken["bad_indices"] = np.array([99])
        self.assertFalse(inspect_spline_seed(broken, np.zeros(6))["evaluation_complete"])
        nonfinite = dict(built); nonfinite["nlp"] = dict(built["nlp"])
        nonfinite["nlp"]["g"] = ca.vertcat(ca.sqrt(-1), built["nlp"]["g"][1:])
        bad = inspect_spline_seed(nonfinite, np.zeros(6))
        self.assertFalse(bad["evaluation_complete"])
        json.dumps(bad, allow_nan=False)

    def test_actual_tiny_builder_matches_independent_physical_evaluation(self):
        from simplified_vehicle_models.solvers.qss_line_ocp import QssSettings, build_mesh
        from simplified_vehicle_models.solvers.qss_spline_ocp import build_spline_ocp
        with tempfile.TemporaryDirectory() as tmp:
            track = load_track(_write_circle_track(Path(tmp), radius=40., nodes=72, half_width=3.))
            track = replace(track, is_closed=False)
            corridor = corridor_from_track(track)
            spline = OffsetSpline(track, control_spacing=20.)
            mesh = build_mesh(track, 20.)
            built = build_spline_ocp(track, corridor, _surface(), mesh, spline,
                                     QssSettings(start_speed=10.))
            seed = np.zeros(int(built["nlp"]["x"].numel()))
            controls, nodes = built["control_count"], mesh.size
            seed[controls:controls + nodes] = 10.
            seed[controls + nodes:] = 0.
            report = inspect_spline_seed(built, seed)
            np.testing.assert_array_equal(seed[:controls], np.zeros(controls))
            np.testing.assert_array_equal(seed[controls:controls + nodes], np.full(nodes, 10.))
            np.testing.assert_array_equal(seed[controls + nodes:], np.zeros(seed.size - controls - nodes))
            independent_g = np.asarray(ca.Function("seed_test_g", [built["nlp"]["x"]],
                                                     [built["nlp"]["g"]])(seed)).reshape(-1)
            independent_f = float(ca.Function("seed_test_f", [built["nlp"]["x"]],
                                               [built["nlp"]["f"]])(seed))
            expected = np.maximum.reduce((np.maximum(built["g_lower"] - independent_g, 0.),
                                          np.maximum(independent_g - built["g_upper"], 0.)))
            self.assertEqual(report["objective_value"], independent_f)
            self.assertAlmostEqual(report["maximum_constraint_violation"], float(expected.max()))
            seed[0] = built["x_upper"][0] + 1.
            violated = inspect_spline_seed(built, seed)
            self.assertGreater(violated["maximum_bound_violation"], 0.)
            self.assertEqual(seed[0], built["x_upper"][0] + 1.)
            self.assertEqual(seed[0], built["x_upper"][0] + 1.)
            self.assertNotIn("lap_time", report)
            self.assertFalse(report["built_problem_source_identity_verified"])


if __name__ == "__main__":
    unittest.main()
