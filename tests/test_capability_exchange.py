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

    def test_validator_missing_evidence_cannot_be_passed(self):
        from simplified_vehicle_models.solvers.capability_exchange import solve_constraint_exchange
        sol = SimpleNamespace(status="Solve_Succeeded")
        _, report = solve_constraint_exchange(None, self.corridor, Surface(), np.array([0.,1.]), None, None,
            solver=lambda *a, **k: sol, validator=lambda *a, **k: {"checks": {}, "sampling_converged": True, "model_feasible": True},
            maximum_rounds=1)
        self.assertFalse(report["accepted"])
        self.assertEqual(report["status"], "search_failed")

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
