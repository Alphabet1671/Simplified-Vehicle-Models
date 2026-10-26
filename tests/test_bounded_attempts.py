import json
import math
import unittest

from simplified_vehicle_models.solvers.bounded_attempts import (
    AttemptBudget, AttemptTerminal, StageResult, run_bounded_attempt, STAGES,
)


class FakeClock:
    def __init__(self, value=0.): self.value = value
    def __call__(self): return self.value
    def advance(self, amount): self.value += amount


def suite(clock, *, durations=None, outputs=None, override=None):
    durations, outputs = durations or {}, outputs or {}
    stages = {}
    for name in STAGES:
        def call(context, n=name):
            if override and n in override: return override[n](context)
            clock.advance(durations.get(n, .1))
            if n == "validate": return outputs.get(n, StageResult({"proof": "local"}, locally_valid=True))
            return outputs.get(n, f"{n}-value")
        stages[name] = call
    return stages


class BoundedAttemptTests(unittest.TestCase):
    def test_deadline_before_seed(self):
        c=FakeClock(); called=[]; st=suite(c)
        reads=[0]
        def advancing_clock():
            reads[0] += 1
            if reads[0] >= 2: c.value = 1.
            return c()
        st["seed"] = lambda ctx: called.append(True)
        r=run_bounded_attempt(budget=AttemptBudget(1), clock=advancing_clock, stages=st)
        self.assertEqual(r["terminal"], AttemptTerminal.DEADLINE_EXHAUSTED.value); self.assertFalse(called)
        self.assertEqual([x["outcome"] for x in r["stages"]], ["skipped"]*6)

    def test_deadline_after_each_stage_preserves_results(self):
        for idx, name in enumerate(STAGES):
            with self.subTest(name=name):
                c=FakeClock(); durations={n: .1 for n in STAGES}; durations[name]=1.
                r=run_bounded_attempt(budget=AttemptBudget(1), clock=c, stages=suite(c,durations=durations))
                self.assertEqual(r["terminal"], AttemptTerminal.DEADLINE_EXHAUSTED.value)
                self.assertEqual(len(r["attempt_state"]), idx+1)
                self.assertEqual(r["stages"][idx]["outcome"], "completed")
                self.assertTrue(all(x["outcome"] == "skipped" for x in r["stages"][idx+1:]))

    def test_cancellation_between_stages(self):
        c=FakeClock(); checks=[False, True]
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=suite(c), cancellation=lambda: checks.pop(0))
        self.assertEqual(r["terminal"], "cancelled"); self.assertEqual(list(r["attempt_state"]), ["seed"])

    def test_solver_exception_retains_prefix(self):
        c=FakeClock(); st=suite(c); st["solve"]=lambda ctx: (_ for _ in ()).throw(RuntimeError("boom"))
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=st)
        self.assertEqual(r["terminal"], "stage_failed"); self.assertEqual(set(r["attempt_state"]), {"seed","build"})
        self.assertEqual(r["stages"][2]["error_type"], "RuntimeError")
        self.assertEqual(r["stages"][3]["outcome"], "skipped")

    def test_unresolved_validation_even_with_search_result(self):
        c=FakeClock(); st=suite(c, outputs={"validate": StageResult({"unresolved": True}, resolved=False)})
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=st)
        self.assertEqual(r["terminal"], "validation_unresolved"); self.assertIn("search", r["attempt_state"])
        self.assertFalse(r["accepted"])

    def test_valid_export_still_not_accepted(self):
        c=FakeClock(); r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=suite(c))
        self.assertEqual(r["terminal"], "locally_validated"); self.assertFalse(r["accepted"])

    def test_malformed_return_and_nonfinite_clock_fail_closed(self):
        c=FakeClock(); st=suite(c, outputs={"validate": object()})
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=st)
        self.assertEqual(r["terminal"], "stage_failed"); json.dumps(r, allow_nan=False)
        c=FakeClock(math.nan)
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=suite(c))
        self.assertEqual(r["terminal"], "stage_failed"); json.dumps(r, allow_nan=False)
        c=FakeClock(); st=suite(c, durations={"solve": math.inf})
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=st)
        self.assertEqual(r["terminal"], "stage_failed"); json.dumps(r, allow_nan=False)

    def test_exchange_like_no_progress_does_not_prove_acceptance(self):
        c=FakeClock(); st=suite(c, outputs={"search": {"actual_grid": [1,2], "strategy": "same", "progress": False},
                                           "validate": StageResult({"resolved": False}, resolved=False)})
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=st)
        self.assertEqual(r["terminal"], "validation_unresolved"); self.assertFalse(r["accepted"])
        self.assertEqual(r["attempt_state"]["search"]["actual_grid"], [1,2])

    def test_budget_rejects_invalid_scalars(self):
        for x in (True, 0, -1, math.inf, math.nan, "2"):
            with self.subTest(value=x), self.assertRaises(ValueError): AttemptBudget(x)

    def test_cancellation_during_export_wins_and_retains_output(self):
        c=FakeClock(); st=suite(c)
        def export(ctx):
            c.advance(.1)
            cancelled[0] = True
            return {"done": True}
        cancelled=[False]
        st["export"] = export
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=st,
                              cancellation=lambda: cancelled[0])
        self.assertEqual(r["terminal"], "cancelled")
        self.assertEqual(r["attempt_state"]["export"], {"done": True})
        self.assertEqual(r["stages"][-1]["outcome"], "completed")

    def test_malformed_stage_result_flags_fail_closed(self):
        c=FakeClock(); st=suite(c, outputs={"validate": StageResult("x", locally_valid=1)})
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=st)
        self.assertEqual(r["terminal"], "stage_failed")

    def test_nested_nonfinite_stage_data_fails_closed(self):
        c=FakeClock(); st=suite(c, outputs={"solve": {"nested": [float("nan")]}})
        r=run_bounded_attempt(budget=AttemptBudget(3), clock=c, stages=st)
        self.assertEqual(r["terminal"], "stage_failed")
        json.dumps(r, allow_nan=False)

    def test_backward_and_overflowing_finite_elapsed_fail_closed(self):
        class SeqClock:
            def __init__(self, values): self.values=iter(values)
            def __call__(self): return next(self.values)
        cases = (([0., 0., 1., -1., -1.], AttemptBudget(3), "invalid_clock"),
                 ([-1e308, -1e308, -9e307, 1e308], AttemptBudget(1.7e308), "invalid_clock"))
        for values, budget, reason in cases:
            with self.subTest(values=values):
                r=run_bounded_attempt(budget=budget, clock=SeqClock(values), stages=suite(FakeClock()))
                self.assertEqual(r["terminal"], "stage_failed")
                self.assertEqual(r["terminal_reason"], reason)
                if values[0] == 0.:
                    self.assertEqual(r["stages"][0]["outcome"], "completed")
                    self.assertEqual(r["stages"][1]["outcome"], "skipped")
                json.dumps(r, allow_nan=False)


if __name__ == "__main__": unittest.main()
