import unittest
from dataclasses import fields
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from simplified_vehicle_models.solvers import qss_spline_ocp as solver


class SplineStageTimingTest(unittest.TestCase):
    def test_solution_timing_default_is_independent_and_positional_compatible(self):
        timing_field = next(item for item in fields(solver.SplineSolution) if item.name == "stage_timings")
        first, second = timing_field.default_factory(), timing_field.default_factory()
        self.assertEqual(first, {})
        self.assertIsNot(first, second)
        first["changed"] = 1.0
        self.assertEqual(second, {})

    def test_strict_failure_diagnostics_preserve_stage_timings(self):
        timings = {name: 0.0 for name in (
            "build_seconds", "initialization_seconds", "symbolic_scaling_seconds",
            "solver_construction_seconds", "solver_execution_seconds",
            "postprocess_seconds", "total_seconds",
        )}
        built = {"control_count": 2, "mesh": np.array([0., 1.]),
                 "nlp": {"x": SimpleNamespace(numel=lambda: 5)},
                 "x_lower": np.zeros(5), "x_upper": np.ones(5)}
        with self.assertRaises(solver.SplineSolveError) as caught:
            solver._raise_spline_solver_failure(
                status="Maximum_Iterations_Exceeded", values=np.arange(5.),
                constraints=np.array([0.]), objective=1., constraint_violation=0.,
                bound_violation=0., iter_count=1, total_iterations=1, stats={}, result={},
                built=built, settings=SimpleNamespace(tolerance=1e-6,
                control_representation="quadratic_acceleration_v1"), stage_timings=timings,
            )
        self.assertEqual(caught.exception.diagnostics["stage_timings"], timings)

    def test_successful_solve_returns_all_finite_nonnegative_stages(self):
        names = ("build_seconds", "initialization_seconds", "symbolic_scaling_seconds",
                 "solver_construction_seconds", "solver_execution_seconds", "postprocess_seconds",
                 "total_seconds")
        built = {"count": 1, "control_count": 3, "mesh": np.array([0., 1.]),
                 "nlp": {"x": SimpleNamespace(numel=lambda: 8), "f": object(), "g": object()},
                 "x_lower": np.zeros(8), "x_upper": np.ones(8), "g_lower": np.zeros(1),
                 "g_upper": np.zeros(1), "time_quadrature_panel_count": 1,
                 "time_quadrature_site_count": 2, "reference": object(),
                 "off_site_stations": np.array([]), "off_site_elements": np.array([], dtype=int),
                 "off_site_fractions": np.array([])}
        class Expr:
            def __radd__(self, other): return self
            def __add__(self, other): return self
            def __mul__(self, other): return self
            def __rmul__(self, other): return self
        class MX:
            @staticmethod
            def sym(*args): return Expr()
        class FakeCA:
            @staticmethod
            def DM(value): return value
            @staticmethod
            def substitute(value, *args): return value
            @staticmethod
            def nlpsol(*args):
                return lambda **kwargs: {"x": np.ones(8) * .5, "g": np.zeros(1), "f": 1.,
                                         "lam_x": np.zeros(8), "lam_g": np.zeros(1)}
        class FakeSolver:
            def __call__(self, **kwargs): return {"x": np.ones(8) * .5, "g": np.zeros(1), "f": 1.}
            def stats(self): return {"return_status": "Solve_Succeeded", "iter_count": 1}
        FakeCA.MX = MX
        FakeCA.nlpsol = staticmethod(lambda *args: FakeSolver())
        evaluated = SimpleNamespace(offset=np.zeros(2), heading_error=np.zeros(2), curvature=np.zeros(2))
        trajectory = SimpleNamespace(evaluate=lambda mesh: evaluated)
        evaluator = object()
        with patch.dict("sys.modules", {"casadi": FakeCA}), \
             patch.object(solver, "build_spline_ocp", return_value=built), \
             patch.object(solver, "_decision_scaling", return_value=(np.zeros(8), np.ones(8))), \
             patch.object(solver, "_tolerance_schedule", return_value=(1e-6,)), \
             patch.object(solver, "OffsetTrajectory", return_value=trajectory), \
             patch.object(solver, "_SplineStateEvaluator", return_value=evaluator), \
             patch.object(solver, "physical_time_quadrature", return_value=2.), \
             patch.object(solver, "_time_quadrature_metadata", return_value={"kind": "fixture"}):
            solution = solver.solve_spline_line(None, None, None, np.array([0., 1.]), None,
                                                SimpleNamespace(tolerance=1e-6, maximum_iterations=2,
                                                hessian_approximation="exact",
                                                control_representation="quadratic_acceleration_v1"),
                                                seed=np.full(8, .5))
        self.assertEqual(set(solution.stage_timings), set(names))
        self.assertTrue(all(np.isfinite(value) and value >= 0.0 for value in solution.stage_timings.values()))
        self.assertGreaterEqual(solution.stage_timings["total_seconds"], solution.stage_timings["postprocess_seconds"])


if __name__ == "__main__":
    unittest.main()
