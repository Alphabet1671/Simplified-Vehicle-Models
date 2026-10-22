"""Bounded adaptive-line wrapper tests with no CasADi solve."""
from __future__ import annotations

from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.solvers.adaptive_line import solve_adaptive_line


class AdaptiveLineTest(unittest.TestCase):
    def setUp(self):
        self.mesh = np.array([0.0, 1.0, 2.0])
        self.spline = SimpleNamespace(control_distance=np.array([0.0, 1.0, 2.0]))
        self.solution = SimpleNamespace(status="Solve_Succeeded", lap_time=1.25)
        self.solution.trajectory = SimpleNamespace(
            reference=SimpleNamespace(source_station=np.array([.75, 1.25])),
        )
        self.common = (object(), object(), object(), self.mesh, self.spline, object())

    @staticmethod
    def _validation(*, passed, station=None, model=True):
        diagnostic = {"name": "implied_acceleration", "passed": passed,
                      "worst_station": station, "maximum_violation": .1,
                      "unit": "m/s^2", "tolerance": 1.e-6}
        return {"checks": {"dense_feasibility": passed}, "model_feasible": model,
                "diagnostics": [diagnostic]}

    def test_dense_failure_refines_then_true_dense_and_model_returns_solution(self):
        second = SimpleNamespace(status="Solve_Succeeded", lap_time=1.1,
                                 trajectory=self.solution.trajectory)
        seen_meshes, seen_seeds = [], []
        def solve(*args, seed=None, **kwargs):
            seen_meshes.append(np.asarray(args[3])); seen_seeds.append(seed)
            return self.solution if len(seen_meshes) == 1 else second
        reports = [self._validation(passed=False, station=.8),
                   self._validation(passed=True)]
        with patch("simplified_vehicle_models.solvers.adaptive_line.solve_spline_line", solve), \
             patch("simplified_vehicle_models.solvers.adaptive_line.validate_line", side_effect=reports), \
             patch("simplified_vehicle_models.solvers.adaptive_line.project_energy_seed",
                   return_value=(np.array([4.]), {"projection_only": True})):
            solution, report = solve_adaptive_line(*self.common, maximum_rounds=2)
        self.assertIs(solution, second)
        self.assertFalse(report["accepted"])
        self.assertTrue(report["validated_solution_available"])
        self.assertEqual(report["validation_status"], "incomplete")
        self.assertEqual(report["status"], "locally_validated")
        self.assertEqual(len(report["rounds"]), 2)
        self.assertGreater(seen_meshes[1].size, self.mesh.size)
        np.testing.assert_array_equal(seen_seeds[1], [4.])

    def test_refines_all_elements_with_actual_failed_stations(self):
        module = __import__("simplified_vehicle_models.solvers.adaptive_line",
                            fromlist=["_refinement_stations"])
        validation = {"diagnostics": [{"name": "acceleration_max", "passed": False,
                       "worst_station": .2, "refinement_stations": [.2, 1.8]}]}
        additions = module._refinement_stations(self.mesh, self.spline, self.solution, validation)
        self.assertTrue(np.any(np.isclose(additions, .5)))
        self.assertTrue(np.any(np.isclose(additions, 1.5)))

    def test_node_budget_stops_before_projecting_or_second_solve(self):
        with patch("simplified_vehicle_models.solvers.adaptive_line.solve_spline_line",
                   return_value=self.solution) as solve, \
             patch("simplified_vehicle_models.solvers.adaptive_line.validate_line",
                   return_value=self._validation(passed=False, station=.8)), \
             patch("simplified_vehicle_models.solvers.adaptive_line.project_energy_seed") as project:
            solution, report = solve_adaptive_line(*self.common, maximum_rounds=2,
                                                   maximum_nodes=3)
        self.assertIsNone(solution)
        self.assertFalse(report["accepted"])
        self.assertEqual(report["stop"], "maximum_nodes")
        self.assertEqual(solve.call_count, 1)
        project.assert_not_called()

    def test_uncertainty_only_failure_does_not_refine_mesh(self):
        uncertain = {"checks": {"dense_feasibility": False}, "model_feasible": True,
                     "diagnostics": [{"name": "derivative_uncertainty", "passed": False,
                                      "worst_station": 1.0, "maximum_violation": 1.,
                                      "unit": "(m/s)/m", "tolerance": 1.e-6}]}
        with patch("simplified_vehicle_models.solvers.adaptive_line.solve_spline_line",
                   return_value=self.solution) as solve, \
             patch("simplified_vehicle_models.solvers.adaptive_line.validate_line", return_value=uncertain):
            solution, report = solve_adaptive_line(*self.common, maximum_rounds=3)
        self.assertIsNone(solution)
        self.assertEqual(report["stop"], "unresolved_no_refinement_station")
        self.assertEqual(solve.call_count, 1)

    def test_uncertainty_upper_bound_only_failure_does_not_refine_mesh(self):
        uncertain = {"checks": {"dense_feasibility": False}, "model_feasible": True,
                     "diagnostics": [{"name": "implied_acceleration_upper_bound", "passed": False,
                                      "worst_station": 1.0, "maximum_violation": 1.,
                                      "unit": "m/s^2", "tolerance": 1.e-6}]}
        with patch("simplified_vehicle_models.solvers.adaptive_line.solve_spline_line",
                   return_value=self.solution) as solve, \
             patch("simplified_vehicle_models.solvers.adaptive_line.validate_line", return_value=uncertain):
            solution, report = solve_adaptive_line(*self.common, maximum_rounds=3)
        self.assertIsNone(solution)
        self.assertEqual(report["stop"], "unresolved_no_refinement_station")
        self.assertEqual(solve.call_count, 1)

    def test_noninteger_budgets_are_rejected_without_truncation(self):
        for keyword in ("maximum_rounds", "maximum_nodes", "dense_maximum_refinements"):
            with self.subTest(keyword=keyword):
                with self.assertRaisesRegex(ValueError, "integer"):
                    solve_adaptive_line(*self.common, **{keyword: 1.9})

    def test_ordinary_dependency_errors_return_failed_reports(self):
        cases = [
            ("solve_spline_line", KeyError("solver metadata"), "solver_failed"),
            ("validate_line", AttributeError("validator field"), "validation_failed"),
            ("_refinement_stations", IndexError("refinement index"), "refinement_failed"),
        ]
        for target, error, stop in cases:
            with self.subTest(target=target), \
                 patch("simplified_vehicle_models.solvers.adaptive_line.solve_spline_line",
                       return_value=self.solution), \
                 patch("simplified_vehicle_models.solvers.adaptive_line.validate_line",
                       return_value=self._validation(passed=False, station=.8)):
                with patch("simplified_vehicle_models.solvers.adaptive_line." + target,
                           side_effect=error):
                    solution, report = solve_adaptive_line(*self.common, maximum_rounds=2)
            self.assertIsNone(solution)
            self.assertFalse(report["accepted"])
            self.assertEqual(report["stop"], stop)
            self.assertEqual(report["rounds"][-1]["stop"], stop)

    def test_solver_failure_is_never_accepted(self):
        with patch("simplified_vehicle_models.solvers.adaptive_line.solve_spline_line",
                   side_effect=RuntimeError("genuine failure")):
            solution, report = solve_adaptive_line(*self.common)
        self.assertIsNone(solution)
        self.assertFalse(report["accepted"])
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["rounds"][0]["status"], "solver_failed")

    def test_source_knots_near_existing_nodes_do_not_create_tiny_elements(self):
        self.solution.trajectory.reference.source_station = np.array([1.0 + 1.e-12])
        additions = __import__("simplified_vehicle_models.solvers.adaptive_line",
            fromlist=["_refinement_stations"])._refinement_stations(
                self.mesh, self.spline, self.solution,
                self._validation(passed=False, station=.8))
        from simplified_vehicle_models.solvers.line_refinement import merge_refinement_mesh
        merged = merge_refinement_mesh(self.mesh, additions, minimum_spacing=1.e-4)
        self.assertGreaterEqual(np.min(np.diff(merged)), 1.e-4)


if __name__ == "__main__":
    unittest.main()
