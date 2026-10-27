# Task 5 — Replay validation cost and coverage

## Scope and method

Implemented run-local timing/count diagnostics in `solve_forward` and propagated them into replay summaries without changing the `ForwardResult` schema. Validation remains per every solver-accepted state and per required dense-output midpoint; defect checks remain per midpoint. No state-dependent domain decision is memoized, no dense probe is removed, and no shared observation optimization was made. The latter was intentionally avoided because validation must independently exercise plant-domain decisions at each state/time.

The forward solver exposes six stage timings: integration, accepted-state validation, dense-state/domain validation, defect checking, record writing, and summary construction. Counts report accepted states, midpoint probes, defects and records; replay totals also include solver evaluations and accepted steps. A validation-budget timeout is a `ForwardIntegrationError` with `last_state_kind=solver_accepted_unvalidated`, read-only last accepted state, elapsed time, and partial timing/counts, so the runner writes its existing failure snapshot and failed summary.

## Red/green unit evidence

Added `tests/test_yaw_replay_validation_cost.py` with coverage regressions that assert exact counts (`accepted_states_validated == len(t)`, `dense_probes_validated == len(t)-1`), and verify invalid accepted-state and dense-midpoint injections still fail. A fake monotonic clock expires after integration and verifies the unvalidated read-only last-state snapshot. Existing watchdog and forward driver/failure tests were run alongside it. Review-correction regressions now also cover runner failure-summary propagation and nonzero forward-result construction timing.

Command (process-level watchdog 180 s):

```text
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- python3 -m unittest tests.test_yaw_replay_validation_cost tests.test_yaw_watchdog_group tests.test_forward_driver tests.test_forward_failure_diagnostics -q
Ran 27 tests in 0.868s — OK (includes review-correction failure-summary and construction-timing regressions).
```

## MIS physical replay and baseline caveat

The requested pre-optimization physical baseline was not captured: implementation edits were already in the worktree before the first eligible replay began. The first run below is therefore the first instrumented replay, **not** a valid before-change baseline. The second replay is a same-code repeat, not a true optimized-code comparison; it cannot establish a speedup or exact pre/post physical equivalence. No baseline retry was attempted. The Task 5 physical before/after cost comparison is consequently inconclusive.

Exact saved replay selection: `runs/yaw_lap_suite/mis-line-40m-retained/plan.npz`, existing final `model-aware` controller, MIS track and `iem24-vehicle-limit-final.npz` map; duration 150 s, BDF, rtol 1e-6, atol 1e-8, max step 0.05 s. Requested cooperative budget 1,400 s and process-group hard deadline 1,500 s. Baseline ran once; optimized comparison used identical inputs. Package/thread provenance: Python 3.13 runtime; NumPy 2.4.4, SciPy 1.17.1; CasADi unavailable in this environment (not required by replay); 8 logical CPUs, arm64; no `*THREAD*` environment overrides recorded.

| Run | Status | Driven time (s) | Lateral RMS (m) | Lateral max (m) | Speed RMS (m/s) | Solver evals | Accepted steps | Total replay wall (s) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Baseline, once | completed | 61.33835327785899 | 0.06344986660680772 | 0.16293166085144378 | 0.31928775538999066 | 66,990 | 20,449 | 172.4674 |
| Optimized-code comparison | completed | 61.33835327785899 | 0.06344986660680772 | 0.16293166085144378 | 0.31928775538999066 | 66,990 | 20,449 | 196.1768 |

The exact driven time and all reported tracking metrics match (difference 0 in displayed values). Optimization did not yield a speedup; the stage instrumentation/observed wall is slightly higher for that comparison. Baseline stage seconds: integration 103.4301, accepted validation 32.9927, dense/domain validation 32.2519, defect 0.4043, record writes 2.3626. The duplicated accepted and midpoint validation costs confirm why the required independent coverage must remain visible; no unproven sharing was attempted.

## Three-run final replay gate

Three serial fresh output directories were run with the same exact final replay inputs and 1,200 s cooperative / 1,500 s hard bounds:

| Fresh run | Completed | Driven time (s) | Lateral RMS (m) | Lateral max (m) | Speed RMS (m/s) | Total replay wall (s) |
|---|---|---:|---:|---:|---:|---:|
| 1 | yes | 61.33835327785899 | 0.063449866851 | 0.162931660851 | 0.319287755 | 172.0019 |
| 2 | yes | 61.33835327785899 | 0.063449866851 | 0.162931660851 | 0.319287755 | 160.9417 |
| 3 | yes | 61.33835327785899 | 0.063449866851 | 0.162931660851 | 0.319287755 | 164.3855 |

Median wall: 164.3855 s; maximum wall: 172.0019 s. All three meet the ≤1,200 s project gate and each has >1,300 s before the hard watchdog. Values are replay total wall from solver diagnostics; the process watchdog also covers setup and summary I/O, and all command invocations completed normally.

## Files and caveats

Task 5 sources: `src/simplified_vehicle_models/solvers/forward_driver.py`, `tools/run/run_yaw_lap_suite.py`, `tests/test_yaw_watchdog_group.py`, and `tests/test_yaw_replay_validation_cost.py`. No legacy OCP edits. Existing unrelated and concurrent Task 4 worktree changes were left untouched and unstaged.
