# W1 Report — Adaptive Coupling Update

Date: 2026-10-12
Branch: `feature/energy-coupling` (HEAD `e866aa4`, worktree `.worktrees/point-mass`)

## Commands

```text
# Unit tests
PYTHONPATH=src python3 -m unittest tests.test_energy_lap -v

# Full test suite
PYTHONPATH=src python3 -m unittest discover -s tests -q

# W1 acceptance test
PYTHONPATH=src python3 /tmp/w1_acceptance_test.py

# Git operations
git add src/simplified_vehicle_models/solvers/forward_backward.py tests/test_energy_lap.py
git commit -m "W1: replace fixed 0.5 relaxation with undamped update + divergence guard"
```

## Core Change

In `src/simplified_vehicle_models/solvers/forward_backward.py::solve_lap_with_energy`, replaced the fixed 0.5 relaxation with an undamped update plus divergence guard.

**Before:**
```python
trace = 0.5 * (trace + nxt)
trace[0] = state0
```

**After:**
```python
# Undamped update: trace = nxt.  The W0 measurements show the
# coupling map is near-idempotent (implied undamped contraction
# λ ≈ 0.0004), so a single pass already moves the profile by only
# ~0.04% of the current error — undamped convergence in ~2-3
# iterations instead of 13.
#
# Divergence guard: if the residual fails to shrink (ratio >= 1),
# the map is non-contractive here and we fall back to the damped
# 0.5 update for this iteration and keep damping until residuals
# shrink again.  This guard never lets the loop oscillate or
# diverge silently; energy_converged stays False if
# maximum_iterations is reached without change <= tolerance.
# The guard exists for coupling maps that are not near-idempotent.
if iteration > 1 and change >= previous_residual:
    trace = 0.5 * (trace + nxt)
else:
    trace = nxt
trace[0] = state0
previous_residual = change
```

The `previous_residual` variable is initialized to `math.inf` before the loop, so on iteration 1 the guard condition `iteration > 1` is false and the undamped update is always used.

## Measured Iteration Counts and Residual Sequences

### SOC 0.9617234998917527 (compiled initial SOC, `initial_state=None`)

| Metric | W0 (damped 0.5) | W1 (undamped + guard) |
|---|---|---|
| Energy iterations | 13 | **3** |
| Residual sequence | 2.838e-02, 1.420e-02, 7.107e-03, 3.557e-03, 1.780e-03, 8.907e-04, 4.457e-04, 2.230e-04, 1.116e-04, 5.585e-05, 2.795e-05, 1.398e-05, 6.996e-06 | [0.02838, 2.515e-05, 1.401e-08] |
| solve_lap seconds | 181.857s | ~36.5s |
| _energy_trace seconds | 0.677s | ~0.001s |
| Total wall | 182.534s | **~37.5s** |

The undamped solver converges in 3 iterations instead of 13. The first residual is the same (~0.0284), the second residual drops to ~2.5e-05 (near machine precision of the tolerance 1e-5), and the third is ~1.4e-08.

### SOC 0.5 (`initial_state=StorageState(0.5)`)

| Metric | W0 (damped 0.5) | W1 (undamped + guard) |
|---|---|---|
| Energy iterations | 13 | **2** |
| Total wall | 894.834s | **~142s** |

The undamped solver converges in 2 iterations instead of 13.

## Acceptance Deltas vs W0 Reference Values

### SOC 0.9617 (`initial_state=None`)

| Metric | W1 result | W0 reference | Delta | Acceptable? |
|---|---|---|---|---|
| lap_time | 69.187442 s | 69.187443 s | 9.3e-07 s | ✓ (≤ 1ms) |
| final SOC | 0.9333203391 | 0.9333203465 | 7.4e-09 | ✓ (≤ 1e-5) |
| consumed energy | 814845.181 J | 814845.010 J | 0.000% | ✓ (≤ 0.1%) |

### SOC 0.5 (`initial_state=StorageState(0.5)`)

| Metric | W1 result | W0 reference | Delta | Acceptable? |
|---|---|---|---|---|
| lap_time | 69.057655 s | 69.057656 s | 1.4e-06 s | ✓ (≤ 1ms) |
| final SOC | 0.4680491962 | 0.4680491991 | 2.9e-09 | ✓ (≤ 1e-5) |
| consumed energy | 823901.271 J | 823901.201 J | 0.000% | ✓ (≤ 0.1%) |

All acceptance criteria met.

## Guard Design Summary

The divergence guard tracks `previous_residual` (initialized to `math.inf`). On each iteration after the first, if `change >= previous_residual` (residual fails to shrink), the guard falls back to the damped 0.5 update for that iteration. The guard re-evaluates each subsequent iteration — if residuals start shrinking again, undamped updates resume.

**Why undamped is safe here:** The W0 measurements show the implied undamped contraction is λ ≈ 0.0004, meaning the coupling map is nearly idempotent. Solving at profile p and re-integrating moves the profile by only ~0.04% of the current error. The first iteration residual is ~0.028, so the second iteration residual is ~0.028 × 0.0004 ≈ 1.1e-05, which is borderline vs tolerance 1e-5 — hence 2-3 iterations total.

**Why the guard exists:** For coupling maps that are not near-idempotent (λ not small), undamped updates could oscillate or diverge. The guard detects non-contractive behavior and falls back to damping, ensuring the loop always terminates.

**Key properties preserved:**
- `energy_converged` remains `False` if `maximum_iterations` is reached without `change <= tolerance`
- `energy_iterations` semantics unchanged (counts coupling iterations)
- The reconciliation loop behavior is unchanged
- `tolerance`, `maximum_iterations`, `state_profile_residual` unchanged in meaning

## Deviations from Instructions

1. **No existing tests needed updating.** No existing tests pinned the 13-iteration count or damped intermediates on the 8-segment track. The `test_outer_nonconvergence_is_reported` test with `maximum_iterations=1` still works correctly because on iteration 1, `previous_residual = math.inf` and `iteration > 1` is false, so the undamped update is used.

2. **`test_bit_identical_with_and_without_diagnostics`** still passes because both with-diagnostics and without-diagnostics use the same code path. The collector only records diagnostics; it doesn't change the update rule.

3. **The `test_default_regen_is_disabled_and_shaft_is_mechanical`** test passes because the update rule change doesn't affect the shaft power computation path.

## Full Test Suite

```
Ran 293 tests in 274.838s
OK
```

All 293 tests pass.

## Tree State

Clean tree, `git diff --check` passes. Single commit `e866aa4` for code+tests. This report is a separate commit.
