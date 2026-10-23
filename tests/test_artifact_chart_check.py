"""Focused tests for explicit cached-polynomial artifact/chart binding."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.formulations.signed_capability import SignedTransientCapability
from simplified_vehicle_models.solvers.artifact_chart_check import check_artifact_chart_element
from simplified_vehicle_models.solvers.trajectory_artifact import load_trajectory_artifact
from simplified_vehicle_models.solvers.qss_spline_ocp import (
    _SplineStateEvaluator, TIME_QUADRATURE_NAME, _time_quadrature_metadata,
)
from simplified_vehicle_models.tracks.line_trajectory import ReferenceCurve, OffsetTrajectory
from simplified_vehicle_models.tracks.offset_spline import OffsetSpline
from tools.run.solve_minimum_time_line import _write_trajectory_metadata


def make_solution(control="quadratic_energy_rate_over_metric_v1"):
    s = np.linspace(0., 5., 6)
    north, east = s.copy(), np.zeros_like(s)
    track = SimpleNamespace(path_coordinates=SimpleNamespace(distance=s, x_north=north,
        y_east=east, is_closed=False), lap_length=5., is_closed=False)
    ref = ReferenceCurve(track)
    spline = OffsetSpline(track, 1.)
    trajectory = OffsetTrajectory(ref, spline, np.zeros(spline.control_count))
    mesh = np.array([0., 1., 2., 3., 4., 5.])
    speed = np.full(mesh.size, 10.)
    acceleration = np.zeros(mesh.size)
    midpoint = np.zeros(mesh.size-1)
    evaluator = _SplineStateEvaluator(trajectory, mesh, speed, acceleration, midpoint,
                                      control_representation=control)
    return SimpleNamespace(trajectory=trajectory, midpoint_acceleration=midpoint,
        coefficients=trajectory.coefficients, distance=mesh, speed=speed,
        acceleration=acceleration, state_representation="energy_hs_v1",
        control_representation=control, time_quadrature=TIME_QUADRATURE_NAME,
        time_quadrature_metadata=_time_quadrature_metadata())


def chart():
    return SignedTransientCapability(np.array([0.,1.]), np.array([-1.,1.]),
        np.full((2,2),10.), np.full((2,2),-10.), np.full(2,30.), .5, "test")


class ArtifactChartCheckTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/"small.npz"
        _write_trajectory_metadata(self.path, make_solution())
        self.loaded = load_trajectory_artifact(self.path)

    def tearDown(self):
        self.temp.cleanup()

    def test_real_artifact_binds_cached_extension_not_physical_authority(self):
        before = [a.copy() for a in (self.loaded.state_evaluator.speed,
            self.loaded.state_evaluator.acceleration, self.loaded.trajectory.coefficients)]
        out = check_artifact_chart_element(self.loaded, chart(), 0)
        self.assertTrue(out["chart_constraints_proven"], out)
        self.assertEqual(out["binding_semantics"], "binary64_cached_node_Hermite_Lagrange_real_extension_v1")
        self.assertFalse(out["solver_artifact_qualified"])
        self.assertFalse(out["physical_node_controls_exactly_preserved"])
        self.assertTrue(out["finite_public_eval_not_certified"])
        self.assertFalse(out["accepted"])
        self.assertFalse(out["artifact_file_identity_reverified"])
        self.assertEqual(out["energy_endpoints_exact"][0], {"numerator":100,"denominator":1})
        self.assertEqual(out["energy_rate_sites_exact"], [{"numerator":0,"denominator":1}]*3)
        self.assertEqual(before[0].tolist(), self.loaded.state_evaluator.speed.tolist())
        self.assertEqual(before[1].tolist(), self.loaded.state_evaluator.acceleration.tolist())
        self.assertEqual(before[2].tolist(), self.loaded.trajectory.coefficients.tolist())
        json.dumps(out, allow_nan=False)
        path_result = check_artifact_chart_element(self.path, chart(), 0)
        self.assertTrue(path_result["chart_constraints_proven"], path_result)
        self.assertTrue(path_result["artifact_file_identity_reverified"])

    def test_index_representation_and_cache_tampering_fail_closed(self):
        for idx in (True, -1, 5, 0.0):
            out = check_artifact_chart_element(self.loaded, chart(), idx)
            self.assertFalse(out["chart_constraints_proven"])
            json.dumps(out, allow_nan=False)
        ev = self.loaded.state_evaluator
        ev.metric_nodes.setflags(write=True)
        ev.metric_nodes[0] += .01
        out = check_artifact_chart_element(self.loaded, chart(), 0)
        self.assertFalse(out["chart_constraints_proven"])
        self.assertIn("metric", out["reason"])

    def test_legacy_control_is_never_reinterpreted(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/"legacy.npz"
            _write_trajectory_metadata(path, make_solution("quadratic_acceleration_v1"))
            out = check_artifact_chart_element(path, chart(), 0)
            self.assertFalse(out["chart_constraints_proven"])
            self.assertIn("representation", out["reason"])
            json.dumps(out, allow_nan=False)

    def test_budget_expiry_before_delegate_and_after_delegate(self):
        from simplified_vehicle_models.solvers import artifact_chart_check as module
        with patch.object(module.time, "monotonic", side_effect=[0., 1., 1., 1.]):
            out = check_artifact_chart_element(self.loaded, chart(), 0,
                settings=module.ChartElementSettings(maximum_wall_seconds=.5))
        self.assertFalse(out["chart_constraints_proven"])
        self.assertIn("wall_budget", out["reason"])
        child = {"status": "chart_constraints_proven", "chart_constraints_proven": True,
                 "reason": "all_closed_panels_proven", "panels": []}
        ticks = iter((0., 0., 2.))
        with patch.object(module.time, "monotonic", side_effect=lambda: next(ticks)), patch.object(module, "check_chart_element", return_value=child):
            out = check_artifact_chart_element(self.loaded, chart(), 0,
                settings=module.ChartElementSettings(maximum_wall_seconds=1.))
        self.assertFalse(out["chart_constraints_proven"])
        self.assertEqual(out["reason"], "wall_budget_exhausted")

    def test_source_kernel_degree_mutation_invalidates_wrapper_proof(self):
        from simplified_vehicle_models.solvers import artifact_chart_check as module
        original = module.check_chart_element
        kernel = self.loaded.trajectory.reference._x
        old_degree = kernel.k
        try:
            def mutate(*args, **kwargs):
                result = original(*args, **kwargs)
                kernel.k = old_degree + 1
                return result
            with patch.object(module, "check_chart_element", side_effect=mutate):
                out = check_artifact_chart_element(self.loaded, chart(), 0)
            self.assertFalse(out["chart_constraints_proven"])
            self.assertTrue(out["source_mutation_detected"])
        finally:
            kernel.k = old_degree

    def test_mutation_during_delegate_invalidates_wrapper_proof(self):
        from simplified_vehicle_models.solvers import artifact_chart_check as module
        original = module.check_chart_element
        def mutate(*args, **kwargs):
            result = original(*args, **kwargs)
            self.loaded.trajectory._offset_curve.c[0] += 1e-4
            return result
        with patch.object(module, "check_chart_element", side_effect=mutate):
            out = check_artifact_chart_element(self.loaded, chart(), 0)
        self.assertFalse(out["chart_constraints_proven"])
        self.assertTrue(out["source_mutation_detected"])


if __name__ == "__main__":
    unittest.main()
