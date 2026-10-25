import json
import unittest

from simplified_vehicle_models.solvers.refinement_comparison import (
    Attempt, ComparisonStatus, IndependentValidation, StrategyRecord, Terminal,
    compare_strategies,
)


IDENTITY = {"initial_seed": {"id": "seed-1"}, "target_physics": "physics-v1",
            "chart_reserve": {"chart": "chart-a", "reserve": 0.1}, "parent_wall_cap_s": 30,
            "boundary": "closed", "representation": "basis-v1"}


def records(*, attempts=None, identities=None):
    attempts = attempts or {}
    identities = identities or {}
    return [StrategyRecord(n, identities.get(n, dict(IDENTITY)), tuple(attempts.get(n, ())))
            for n in ("uniform", "localized", "exchange")]


def attempt(terminal=Terminal.UNRESOLVED, validation=None, **kw):
    defaults = dict(mesh_stations=[0, 1], constraint_stations=[0.5], seed_ancestry=["seed-1"],
                    stage="solve", elapsed_s=2.5, terminal=terminal, outcome={"status": terminal.value},
                    independent_validation=validation)
    defaults.update(kw)
    return Attempt(**defaults)


class ComparisonTests(unittest.TestCase):
    def test_identity_mismatch_seed_and_chart_reserve_never_selects_winner(self):
        for field, value in (("initial_seed", {"id": "other"}),
                             ("chart_reserve", {"chart": "other", "reserve": 0.1}),
                             ("target_physics", "other"), ("parent_wall_cap_s", 31),
                             ("boundary", "open"), ("representation", "other")):
            altered = dict(IDENTITY)
            altered[field] = value
            result = compare_strategies(records(identities={"exchange": altered}))
            self.assertEqual(result.status, ComparisonStatus.INCOMPARABLE)
            self.assertIsNone(result.diagnostic_best)
            self.assertIn(field, result.reason)

    def test_no_progress_and_unresolved_validation_cannot_win(self):
        r = compare_strategies(records(attempts={"exchange": [attempt(Terminal.NO_PROGRESS)]}))
        self.assertIsNone(r.diagnostic_best)
        self.assertEqual(r.status, ComparisonStatus.NO_INDEPENDENTLY_FEASIBLE)

    def test_completed_or_local_status_is_not_feasible(self):
        for terminal in (Terminal.SOLVER_COMPLETED, Terminal.LOCALLY_VALIDATED):
            r = compare_strategies(records(attempts={"uniform": [attempt(terminal)]}))
            self.assertIsNone(r.diagnostic_best)

    def test_independent_validation_can_set_diagnostic_only(self):
        iv = IndependentValidation(Terminal.INDEPENDENTLY_FEASIBLE, {"validator": "independent-v1"})
        r = compare_strategies(records(attempts={"localized": [attempt(Terminal.INDEPENDENTLY_FEASIBLE, iv)]}))
        self.assertEqual(r.diagnostic_best, "localized")
        self.assertFalse(r.accepted)
        self.assertFalse(r.physical_feasibility_certified)
        self.assertTrue(r.comparison_is_not_qualification)

    def test_preserves_attempt_order_mesh_sites_seed_and_stage(self):
        a = attempt(seed_ancestry=["s0", "s1"], stage="coarse")
        b = attempt(Terminal.FAILED, stage="refine", elapsed_s=4)
        r = compare_strategies(records(attempts={"uniform": [a, b]}))
        got = r.to_dict()["strategies"]["uniform"]["attempts"]
        self.assertEqual([x["stage"] for x in got], ["coarse", "refine"])
        self.assertEqual(got[0]["mesh_stations"], [0, 1])
        self.assertEqual(got[0]["constraint_stations"], [0.5])
        self.assertEqual(got[0]["seed_ancestry"], ["s0", "s1"])

    def test_malformed_nonfinite_duplicate_and_bool_numeric_rejected(self):
        base = records()
        with self.assertRaises(ValueError): compare_strategies(base[:2])
        with self.assertRaises(ValueError): compare_strategies([base[0], base[1], base[0]])
        with self.assertRaises(ValueError): compare_strategies(records(attempts={"uniform": [attempt(elapsed_s=True)]}))
        with self.assertRaises(ValueError): compare_strategies(records(attempts={"uniform": [attempt(outcome={"x": float("nan")})]}))

    def test_strict_json_serialization(self):
        r = compare_strategies(records(attempts={"exchange": [attempt()]}))
        parsed = json.loads(r.to_json())
        self.assertFalse(parsed["accepted"])
        self.assertTrue(parsed["comparison_is_not_qualification"])
        self.assertNotIn("NaN", r.to_json())


if __name__ == "__main__":
    unittest.main()
