import unittest
from types import SimpleNamespace
import numpy as np

from simplified_vehicle_models.solvers.capability_exchange import (
    ResidualSearchSettings, locate_physical_residuals,
)


class Trajectory:
    def __init__(self):
        self.reference = SimpleNamespace(source_station=np.array([0., 1.]), curvature=np.array([0., 0.]),
                                         evaluate=lambda s: SimpleNamespace(curvature=np.zeros_like(np.asarray(s, dtype=float))))
        self.spline = SimpleNamespace(control_distance=np.array([0., 1.]))
    def evaluate(self, station):
        s = np.asarray(station, dtype=float)
        # Narrow positive Gaussian lies between standard samples.
        offset = 0.34 + 0.1*np.exp(-((s-.437)/.025)**2)
        return SimpleNamespace(offset=offset, curvature=np.zeros_like(s),
                               reference_curvature=np.zeros_like(s), fold_factor=np.ones_like(s))

class Evaluator:
    def evaluate(self, station):
        s = np.asarray(station, dtype=float)
        return {"speed": np.full_like(s, 10.), "acceleration": np.zeros_like(s)}

class Surface:
    representation = "signed_transient_normalized_v1"
    speed_floor = 1.
    curvature = np.array([-1., 1.])
    u = np.array([0., 1.])
    def domain_speed(self, k): return 20.
    def limits(self, v, k): return (-2., 2.)

class ResidualSearchTests(unittest.TestCase):
    def setUp(self):
        self.solution = SimpleNamespace(distance=np.array([0., 1.]), trajectory=Trajectory(), state_evaluator=Evaluator())
        self.corridor = SimpleNamespace(distance=np.array([0., 1.]), minimum=np.array([-1., -1.]),
                                        maximum=np.array([.35, .35]), is_closed=False)

    def test_finds_narrow_between_legacy_sites(self):
        result = locate_physical_residuals(self.solution, self.corridor, Surface())
        self.assertFalse(result["continuous_certificate"])
        peak = next(x for x in result["peaks"] if x["name"] == "corridor_max")
        self.assertAlmostEqual(peak["station"], .437, delta=.01)
        self.assertGreater(peak["violation"], .04)

    def test_evaluation_budget_is_partial_and_retains_samples(self):
        result = locate_physical_residuals(self.solution, self.corridor, Surface(),
                    settings=ResidualSearchSettings(maximum_evaluations=5))
        self.assertFalse(result["completed"])
        self.assertEqual(result["evaluations"], 5)

    def test_settings_reject_boolean_numerics(self):
        with self.assertRaises(ValueError): ResidualSearchSettings(samples_per_partition=True)
        with self.assertRaises(ValueError): ResidualSearchSettings(violation_tolerance=True)

    def test_unsupported_chart_fails_explicitly(self):
        with self.assertRaises(TypeError): locate_physical_residuals(self.solution, self.corridor, object())

    def test_curvature_outside_chart_is_unresolved_and_incomplete(self):
        class BadTrajectory(Trajectory):
            def evaluate(self, station):
                s = np.asarray(station, dtype=float)
                return SimpleNamespace(offset=np.zeros_like(s), curvature=np.full_like(s, 2.),
                                       reference_curvature=np.zeros_like(s), fold_factor=np.ones_like(s))
        self.solution.trajectory = BadTrajectory()
        result = locate_physical_residuals(self.solution, self.corridor, Surface())
        self.assertFalse(result["completed"])
        self.assertTrue(result["unresolved_domains"])
        self.assertEqual(result["stop"], "curvature_domain_unresolved")

    def test_flat_feasible_search_is_bounded_and_complete(self):
        self.corridor.maximum[:] = 1.
        result = locate_physical_residuals(self.solution, self.corridor, Surface())
        self.assertTrue(result["completed"])
        self.assertEqual(result["peaks"], [])

    def test_exchange_runs_independent_validation_and_never_self_accepts(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        calls, callbacks = [], []
        solution = SimpleNamespace(status="Solve_Succeeded", distance=np.array([0., 1.]),
                                   trajectory=self.solution.trajectory, state_evaluator=self.solution.state_evaluator,
                                   coefficients=np.zeros(1), speed=np.ones(2), acceleration=np.zeros(2),
                                   midpoint_acceleration=np.zeros(1))
        def solver(*args, **kwargs):
            calls.append(kwargs.get("constraint_stations")); return solution
        def validator(*args, **kwargs):
            return {"checks": {"geometry_consistency": True, "dense_feasibility": True},
                    "sampling_converged": True, "model_feasible": True}
        _, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]),
            None, None, solver=solver, validator=validator, on_round=callbacks.append)
        self.assertEqual(calls, [[]])
        self.assertEqual(report["status"], "locally_validated")
        self.assertFalse(report["accepted"])
        self.assertEqual(callbacks[-1]["status"], "locally_validated")
        self.assertEqual(len(callbacks), 1)
        self.assertFalse(callbacks[-1]["accepted"])

    def test_exchange_two_rounds_retains_sites_and_seeds_same_layout(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        calls, snapshots = [], []
        solution = SimpleNamespace(status="Solve_Succeeded", distance=np.array([0., 1.]),
                                   trajectory=self.solution.trajectory, state_evaluator=self.solution.state_evaluator,
                                   coefficients=np.array([2.]), speed=np.array([10., 10.]),
                                   acceleration=np.zeros(2), midpoint_acceleration=np.zeros(1))
        def solver(*args, **kwargs):
            calls.append((kwargs["constraint_stations"], kwargs["seed"]))
            center = .437 if len(calls) == 1 else .637
            solution.trajectory.evaluate = lambda s: SimpleNamespace(
                offset=.34+.1*np.exp(-((np.asarray(s, dtype=float)-center)/.025)**2),
                curvature=np.zeros_like(np.asarray(s, dtype=float)), reference_curvature=np.zeros_like(np.asarray(s, dtype=float)),
                fold_factor=np.ones_like(np.asarray(s, dtype=float)))
            return solution
        def validator(*args, **kwargs):
            return {"checks": {"geometry_consistency": False, "dense_feasibility": False},
                    "sampling_converged": False, "model_feasible": None}
        _, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]),
            None, None, solver=solver, validator=validator, maximum_rounds=2,
            maximum_sites=500, on_round=snapshots.append)
        self.assertEqual(report["status"], "maximum_rounds")
        self.assertFalse(report["accepted"])
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0][0], [])
        self.assertTrue(calls[1][0])
        self.assertEqual(calls[1][1].shape, (6,))
        self.assertEqual(len(snapshots), 2)
        self.assertEqual(snapshots[0]["rounds"][-1]["status"], "exchange_sites_added")
        self.assertEqual(snapshots[-1]["status"], "maximum_rounds")
        self.assertEqual(snapshots[-1]["rounds"][-1]["status"], "maximum_rounds")

    def test_soft_wall_overrun_after_validator_blocks_local_solution(self):
        from unittest.mock import patch
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        solution = SimpleNamespace(status="Solve_Succeeded")
        times = iter([0., 0., 2., 2.])
        with patch("simplified_vehicle_models.solvers.capability_exchange.time.monotonic", side_effect=lambda: next(times)):
            result, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]), None, None,
                solver=lambda *a, **k: solution,
                validator=lambda *a, **k: {"checks": {"geometry_consistency": True, "dense_feasibility": True},
                                           "sampling_converged": True, "model_feasible": True},
                maximum_wall_seconds=1.)
        self.assertIsNone(result)
        self.assertFalse(report["accepted"])
        self.assertFalse(report["validated_solution_available"])
        self.assertEqual(report["status"], "wall_budget_exhausted")

    def test_validator_missing_evidence_cannot_be_passed(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        sol = SimpleNamespace(status="Solve_Succeeded")
        _, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0.,1.]), None, None,
            solver=lambda *a, **k: sol, validator=lambda *a, **k: {"checks": {}, "sampling_converged": True, "model_feasible": True},
            maximum_rounds=1)
        self.assertFalse(report["accepted"])
        self.assertEqual(report["status"], "search_failed")

    def test_exchange_failures_never_return_last_solver_iterate(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        snapshots = []
        iterate = SimpleNamespace(status="Not_Succeeded")
        result, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]),
            None, None, solver=lambda *a, **k: iterate, validator=lambda *a, **k: self.fail("validator must not run"),
            on_round=snapshots.append)
        self.assertIsNone(result)
        self.assertEqual(report["status"], "solver_failed")
        self.assertFalse(report["validated_solution_available"])
        self.assertEqual(snapshots[-1]["status"], "solver_failed")

    def test_validator_and_solver_exceptions_return_none_with_finalized_callback(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        solution = SimpleNamespace(status="Solve_Succeeded")
        for solver, validator, expected in (
            (lambda *a, **k: (_ for _ in ()).throw(RuntimeError("solver boom")), lambda *a, **k: {}, "solver_exception"),
            (lambda *a, **k: solution, lambda *a, **k: (_ for _ in ()).throw(RuntimeError("validator boom")), "validator_exception"),
        ):
            snapshots = []
            result, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]), None, None,
                solver=solver, validator=validator, on_round=snapshots.append)
            self.assertIsNone(result)
            self.assertEqual(report["status"], expected)
            self.assertEqual(snapshots[-1]["status"], expected)
            self.assertEqual(snapshots[-1]["rounds"][-1]["status"], expected)

    def test_exchange_unions_independent_validator_violation_station(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        calls = []
        solution = SimpleNamespace(status="Solve_Succeeded", distance=np.array([0., 1.]),
                                   trajectory=self.solution.trajectory, state_evaluator=self.solution.state_evaluator,
                                   coefficients=np.zeros(1), speed=np.ones(2), acceleration=np.zeros(2),
                                   midpoint_acceleration=np.zeros(1))
        def solver(*args, **kwargs):
            calls.append(kwargs["constraint_stations"])
            solution.trajectory.evaluate = lambda s: SimpleNamespace(
                offset=.34+.1*np.exp(-((np.asarray(s, dtype=float)-(.437 if len(calls)==1 else .637))/.025)**2),
                curvature=np.zeros_like(np.asarray(s, dtype=float)), reference_curvature=np.zeros_like(np.asarray(s, dtype=float)),
                fold_factor=np.ones_like(np.asarray(s, dtype=float)))
            return solution
        validation = {"checks": {"geometry_consistency": False, "dense_feasibility": False},
                      "sampling_converged": False, "model_feasible": None,
                      "diagnostics": [{"name": "acceleration_min", "passed": False,
                                       "worst_station": .2424253, "refinement_stations": [.2424253]}]}
        _, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]), None, None,
            solver=solver, validator=lambda *a, **k: validation, maximum_rounds=2)
        self.assertEqual(report["status"], "maximum_rounds")
        self.assertTrue(any(abs(s-.2424253) < 1e-12 for s in calls[1]))

    def test_validator_model_gate_requires_true_live_model(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        solution = SimpleNamespace(status="Solve_Succeeded")
        result, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]), None, None,
            solver=lambda *a, **k: solution,
            validator=lambda *a, **k: {"checks": {"geometry_consistency": True, "dense_feasibility": True},
                                       "sampling_converged": True, "model_feasible": None},
            maximum_rounds=1)
        self.assertIsNone(result)
        self.assertFalse(report["accepted"])
        self.assertFalse(report["validated_solution_available"])

    def test_exchange_rejects_array_sites_and_initial_overflow(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        with self.assertRaises(ValueError):
            solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]), None, None,
                                      constraint_stations=np.array([.2, np.nan]))
        with self.assertRaises(ValueError):
            solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]), None, None,
                                      constraint_stations=np.array([.2, .3]), maximum_sites=1)

    def test_detects_multiple_peaks_across_knot_partition(self):
        self.solution.trajectory.evaluate = lambda s: SimpleNamespace(
            offset=.2+.2*np.exp(-((np.asarray(s, dtype=float)-.3)/.035)**2)+.2*np.exp(-((np.asarray(s, dtype=float)-.7)/.035)**2),
            curvature=np.zeros_like(np.asarray(s, dtype=float)), reference_curvature=np.zeros_like(np.asarray(s, dtype=float)),
            fold_factor=np.ones_like(np.asarray(s, dtype=float)))
        self.corridor.distance = np.array([0., .5, 1.])
        self.corridor.minimum = np.full(3, -1.)
        self.corridor.maximum = np.full(3, .3)
        result = locate_physical_residuals(self.solution, self.corridor, Surface())
        peaks = [p["station"] for p in result["peaks"] if p["name"] == "corridor_max"]
        self.assertTrue(result["completed"])
        self.assertTrue(any(abs(x-.3) < .02 for x in peaks))
        self.assertTrue(any(abs(x-.7) < .02 for x in peaks))
        self.assertEqual(result["partition_count"], 2)

    def test_invalid_speed_does_not_query_clipped_limits(self):
        class OutOfDomainEvaluator:
            def evaluate(self, station):
                s = np.asarray(station, dtype=float)
                return {"speed": np.full_like(s, 30.), "acceleration": np.zeros_like(s)}
        class NoClipSurface(Surface):
            def __init__(self): self.limit_queries = 0
            def limits(self, v, k):
                self.limit_queries += 1
                raise AssertionError("limits called for out-of-domain speed")
        self.solution.state_evaluator = OutOfDomainEvaluator()
        surface = NoClipSurface()
        result = locate_physical_residuals(self.solution, self.corridor, surface)
        self.assertFalse(result["completed"])
        self.assertEqual(result["stop"], "capability_queries_unresolved")
        self.assertEqual(surface.limit_queries, 0)
        self.assertFalse(result["search_exhausted"])
        self.assertFalse(result["fatal_reasons"])
        self.assertTrue(result["physical_domain_unresolved"])
        self.assertTrue(result["progress_stations_available"])
        self.assertTrue(result["constraint_stations"])

    def test_speed_below_floor_is_known_partial_domain_progress(self):
        class BelowFloorEvaluator:
            def evaluate(self, station):
                s = np.asarray(station, dtype=float)
                return {"speed": np.full_like(s, .5), "acceleration": np.zeros_like(s)}
        class NoBelowFloorLimits(Surface):
            def __init__(self): self.queries = []
            def limits(self, v, k):
                self.queries.append((v, k))
                if v < self.speed_floor: raise AssertionError("limits called below speed floor")
                return (-2., 2.)
        self.solution.state_evaluator = BelowFloorEvaluator()
        surface = NoBelowFloorLimits()
        result = locate_physical_residuals(self.solution, self.corridor, surface)
        self.assertFalse(result["completed"])
        self.assertEqual(result["stop"], "capability_queries_unresolved")
        self.assertFalse(result["fatal_reasons"])
        self.assertTrue(result["physical_domain_unresolved"])
        self.assertTrue(result["constraint_stations"])
        self.assertFalse(any(v < surface.speed_floor for v, _ in surface.queries))

    def test_partial_domain_progress_exchanges_then_strict_local_gates_hold(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        class OutOfDomainEvaluator:
            def evaluate(self, station):
                s = np.asarray(station, dtype=float)
                speed = 10. + 20.*np.exp(-((s-.37)/.025)**2)
                return {"speed": speed, "acceleration": np.zeros_like(s)}
        class DomainSurface(Surface):
            def domain_speed(self, k): return 20.
            def limits(self, v, k):
                if v > 20.: raise AssertionError("out-of-domain limits query")
                return (-2., 2.)
        calls, snapshots = [], []
        solution = SimpleNamespace(status="Solve_Succeeded", distance=np.array([0., 1.]),
            trajectory=self.solution.trajectory, state_evaluator=OutOfDomainEvaluator(), coefficients=np.array([2.]),
            speed=np.array([30., 30.]), acceleration=np.zeros(2), midpoint_acceleration=np.zeros(1))
        def solver(*args, **kwargs):
            calls.append((list(kwargs["constraint_stations"]), kwargs["seed"]))
            return solution
        def validator(*args, **kwargs):
            if len(calls) > 1:
                return {"checks": {"geometry_consistency": True, "dense_feasibility": True},
                        "sampling_converged": True, "model_feasible": True}
            return {"checks": {"geometry_consistency": False, "dense_feasibility": False},
                    "sampling_converged": False, "model_feasible": None,
                    "diagnostics": [{"name": "speed", "passed": False, "worst_station": .25}]}
        _, report = solve_constraint_exchange(None, self.corridor, DomainSurface(), np.array([0., 1.]),
            None, None, solver=solver, validator=validator, maximum_rounds=2, on_round=snapshots.append)
        self.assertEqual(len(calls), 2)
        self.assertIn(.25, calls[1][0])
        self.assertEqual(calls[1][1].shape, (6,))
        self.assertFalse(report["accepted"])
        self.assertEqual(report["status"], "locally_validated")
        self.assertTrue(report["validated_solution_available"])
        self.assertFalse(report["accepted"])
        self.assertTrue(snapshots[0]["rounds"][0]["search_progress_only"])
        self.assertFalse(snapshots[0]["rounds"][0]["search_complete"])

    def test_domain_progress_does_not_mask_evaluation_budget(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        class OutOfDomainEvaluator:
            def evaluate(self, station):
                s = np.asarray(station, dtype=float)
                return {"speed": np.full_like(s, 30.), "acceleration": np.zeros_like(s)}
        calls = []
        solution = SimpleNamespace(status="Solve_Succeeded", distance=np.array([0., 1.]),
            trajectory=self.solution.trajectory, state_evaluator=OutOfDomainEvaluator())
        _, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0., 1.]), None, None,
            solver=lambda *a, **k: (calls.append(1) or solution),
            validator=lambda *a, **k: {"checks": {}, "diagnostics": [{"name": "x", "passed": False, "worst_station": .25}]},
            search_settings=ResidualSearchSettings(maximum_evaluations=5), maximum_rounds=2)
        self.assertEqual(len(calls), 1)
        self.assertEqual(report["status"], "search_incomplete")
        self.assertEqual(report["rounds"][0]["search"]["stop"], "evaluation_budget_exhausted")

    def test_domain_progress_does_not_mask_capability_query_failure(self):
        class MixedEvaluator:
            def evaluate(self, station):
                s = np.asarray(station, dtype=float)
                return {"speed": 10. + 20.*np.exp(-((s-.37)/.025)**2), "acceleration": np.zeros_like(s)}
        class BrokenLimits(Surface):
            def limits(self, v, k): raise ValueError("unknown capability query failure")
        self.solution.state_evaluator = MixedEvaluator()
        result = locate_physical_residuals(self.solution, self.corridor, BrokenLimits())
        self.assertFalse(result["completed"])
        self.assertTrue(result["physical_domain_unresolved"])
        self.assertTrue(result["fatal_reasons"])
        self.assertEqual(result["stop"], "capability_queries_unresolved")

    def test_nonfinite_capability_limits_fail_closed(self):
        class BadLimits(Surface):
            def limits(self, v, k): return (float("nan"), 1.)
        result = locate_physical_residuals(self.solution, self.corridor, BadLimits())
        self.assertFalse(result["completed"])
        self.assertEqual(result["stop"], "nonfinite_query_unresolved")

    def test_nonfinite_geometry_fails_closed(self):
        class NonfiniteTrajectory(Trajectory):
            def evaluate(self, station):
                s = np.asarray(station, dtype=float)
                return SimpleNamespace(offset=np.full_like(s, np.nan), curvature=np.zeros_like(s),
                                       reference_curvature=np.zeros_like(s), fold_factor=np.ones_like(s))
        self.solution.trajectory = NonfiniteTrajectory()
        result = locate_physical_residuals(self.solution, self.corridor, Surface())
        self.assertFalse(result["completed"])
        self.assertTrue(any("nonfinite" in x["reason"] for x in result["unresolved_domains"]))

    def test_peak_limit_is_explicit_partial_result(self):
        self.solution.trajectory.evaluate = lambda s: SimpleNamespace(
            offset=np.full_like(np.asarray(s, dtype=float), .5), curvature=np.zeros_like(np.asarray(s, dtype=float)),
            reference_curvature=np.zeros_like(np.asarray(s, dtype=float)), fold_factor=np.ones_like(np.asarray(s, dtype=float)))
        self.corridor.maximum[:] = 0.
        cfg = ResidualSearchSettings(maximum_peaks=1)
        result = locate_physical_residuals(self.solution, self.corridor, Surface(), settings=cfg)
        self.assertFalse(result["completed"])
        self.assertEqual(result["stop"], "peak_budget_exhausted")

if __name__ == "__main__": unittest.main()
