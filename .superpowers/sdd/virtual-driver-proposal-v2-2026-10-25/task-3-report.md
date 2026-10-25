# Phase 3 task report: yaw-aware line optimizer

**Date:** 2026-10-25
**Branch:** `feature/yaw-aware-virtual-driver`
**Base Phase 1/2 commits:** `fe9b2f1`, `16796d6`, `9f6366f`

## Delivered

- Added `simplified_vehicle_models.planning.yaw_aware_line` with immutable settings and `YawAwareLinePlan`, preserving the exact offset trajectory, yaw profile, derived `SpeedPlan`, corridor, read-only coefficients, lap time, status, diagnostics, and shape metrics.
- Candidate geometry uses `ReferenceCurve` → `OffsetSpline` → `OffsetTrajectory`, physical-arclength sampling through `station_map`, curvature-rate sampling, dense corridor/fold rejection, yaw-aware profiling, and bounded deterministic Powell searches over centreline and edge-using seeds.
- Candidate outcomes classify geometry/corridor rejection, capability/profile rejection, nonconvergence, and accepted evaluations. If every candidate fails, `YawLineOptimizationError` retains structured rejection diagnostics.
- Added `L50`, peak/mean curvature, apex station/offset, and minimum-speed location metrics.
- Added synthetic closed-circle determinism/containment/projection and yaw-authority tests, plus an open standing-start fixture.
- No `qss_*` OCP source/test files were changed.

## Verification

- `PYTHONPATH=src python3 -m unittest tests.test_yaw_aware_line tests.test_yaw_aware_profile -q` — **11 tests passed**.
- `python3 -m py_compile src/simplified_vehicle_models/planning/yaw_aware_line.py` — passed.
- `git diff --check` — passed.
- Full suite was started as background job `bash-3085` but canceled after exceeding the available turn window; no full-suite result is claimed.

## Repair round 1 (review findings)

- Default corridor construction now uses `SIGNED_POLICY` with the fitted `ReferenceCurve`; footprint/buffer-shrunk signed bounds are returned as the plan corridor, with the turn-side fold guard reapplied and dense candidate fold checks retained.
- Replaced the prior heuristic stand-in with a deterministic box-constrained convex discrete seed QP for epsilon `(0, .25, .5, .75)`: `(1-eps)||K(c)-k_ref||_W^2 + eps/2||W B1 c||^2`, with `K` a finite-difference linearization of exact offset curvature at the reference, and `B1` the cubic offset derivative basis. Bounded SLSQP uses analytic gradient, fixed zero initialization, `ftol=1e-9`, and max 100 iterations; per-epsilon success/iterations/objective are retained.
- Added curvature-quiet complex detection with quiet-run padding/grouping, per-complex bounded Powell optimization with controls outside each complex fixed, and outer reprofile/reoptimization passes (maximum three, 1 ms convergence criterion).
- Plan status distinguishes `single_pass`, `converged`, and `iteration_limit`; failed proposals retain explicit gate reasons. Apex offset is evaluated through the physical station map at peak curvature.
- Expanded tests for asymmetric width pinch/corner complex detection, exact speed-plan projection, explicit rejection, negative-curvature fold guard at affected turn-side stations, and distinct TV/no-TV map labels with time comparison on circle proxies. **Unresolved:** synthetic tests do not demonstrate the required TV-shorter-L50 ordering on 90-degree/hairpin fixtures; the actual profile/optimizer has not produced that behavioral evidence, so do not interpret this phase as passing the proposal's decisive experiment.
- Dense corridor containment remains an empirical check, not a continuous certificate. QSS OCPs remain unchanged.
