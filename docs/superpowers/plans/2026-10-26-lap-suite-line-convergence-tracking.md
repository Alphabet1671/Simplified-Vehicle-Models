# Lap Suite Line Optimization, Convergence, and Tracking Implementation Plan

> **For agentic workers:** Use subagent-driven-development for focused implementation/review. Execute stages in order; do not wait for an additional execution-choice confirmation because the user explicitly requested implementation.

**Goal:** Make the accepted vehicle-limit yaw map usable by the profile solver, then deliver practical line optimization, honest convergence studies, and improved tracking for MIS/FSC lap simulations.

**Architecture:** Separate optional profile seed proposals from mandatory boundary/refinement witnesses. Bound line-search work globally and cache repeated evaluations while validating every accepted profile. Keep optimization stationarity, profile-resolution convergence, spatial-grid convergence and controller containment as independent claims. Add a reproducible guarded lap-suite entrypoint using the accepted cached map.

**Tech Stack:** Existing Python/NumPy/SciPy/unittest; no new dependencies.

**Spec:** User request: resolve large-map profile-budget failure, then implement line optimization, convergence and tracking controller for the lap simulation suite. This plan's acceptance criteria below define the reproducible diagnostic scope; no measured-vehicle or global-optimality claim.

## Global Constraints

- Every run—including tests—must be hard-killed at1800s wall-clock unless user approves longer beforehand. Process-group watchdog is mandatory; tool timeout and cooperative solver budgets alone are insufficient.
- Preserve strict feasible-corner map interpolation and independent all-segment profile validation.
- Do not modify legacy QSS/OCP solvers or their tests.
- Do not hide failure with a speed cap, map coarsening, tolerance broadening or silent legacy fallback.
- Preserve existing unrelated worktree files.
- Baseline accepted map identity must remain usable: no edits to physical map modules unless a demonstrated bug requires them.
- Do not claim completed lap, containment, profile resolution, optimizer stationarity or mesh convergence interchangeably.

## Stage 1: Bounded profile seeding and guarded execution

**Files:** coupled_yaw_profile.py; new solver helper if seed logic exceeds ~100 lines; tests/test_coupled_yaw_profile.py; new tools/run/run_with_deadline.py and tests.

**Interfaces:** existing solve_coupled_profile signature stays compatible. New diagnostics expose initial requested/retained/omitted seed counts and mandatory-overflow causes. Optional seeds are proposals, not certified feasible states. Every actual edge stays strictly queried.

- [ ] Add failing regression using a synthetic map with >=428 lateral knots and >=79 speed knots; default candidate limit256 must not fail merely due to map axis density. Standing exact zero and closed seam anchors survive.
- [ ] Bound initial proposals by q_seed_count / declared candidate budget, preserve domain endpoints and valid warm suggestion; deterministically spread optional speed/ay-knot projections. Reserve candidate room for refinement. Report omissions, do not change interval component midpoint rules.
- [ ] Test narrow disconnected support remains discoverable via interval oracle and unresolved components never silently disappear. If mandatory constraints exceed capacity return typed budget failure.
- [ ] Add a process-group deadline runner, default1800s, reject larger limit without explicit opt-in flag; timeout kills descendants and returns124. Test a hanging child+grandchild using subsecond deadline.
- [ ] Run accepted cached-map fixed-path probes coarse-to-fine; inspect timings/node/edge counts before full0.05m attempt. Every probe has an outer hard deadline, no duplicate unbounded run.
- [ ] Review/commit seed and watchdog changes before line optimization.

## Stage 2: Practical bounded line search

**Files:** planning/yaw_aware_line.py; tests/test_yaw_aware_line.py; runner CLI; focused new tests for budget/cache behavior.

**Interfaces:** extend immutable YawLineSettings with global wall/evaluation budgets. Existing function returns best validated incumbent with explicit stop reason, or typed diagnostics when none exists.

- [ ] Add tests counting repeated coefficient evaluations and strict profile calls; identical coefficient/path/settings evaluations must reuse exact cached result rather than launch duplicate profile solves.
- [ ] Honor configurable seed_blends instead of overwriting it; include centreline and declared deterministic QP seeds.
- [ ] Bound optimizer max function evaluations globally across seeds/passes/windows, and wall budget; pass remaining budget into profile settings. Stop safely with validated incumbent and explicit search uncertainty.
- [ ] Update each accepted corner's base coefficients so subsequent corners build on the accepted line; never overwrite a faster global incumbent with a worse result.
- [ ] Warm-start by physical/reference station interpolation when path sampling lengths differ, not only equal-shape reuse. Treat warm values solely as suggestions and revalidate.
- [ ] Keep dense sampled containment and fold checks; persist all accepted/rejected causes and selected profile diagnostics.
- [ ] Benchmark an actual multi-control line (not2000m constant-offset smoke), demonstrate accepted improvement versus matched fixed geometry within run budget. No claim global optimum.

## Stage 3: Convergence contract and study driver

**Files:** new planning/yaw_convergence.py; tests; lap-suite entrypoint/report.

**Interfaces:** immutable per-resolution records containing geometry spacing, control spacing, profile settings, lap time, speed difference on common reference stations, coefficient/geometry difference, feasibility, budgets and stop reasons. A convergence verdict requires at least two independently valid resolutions and all declared tolerances; failed/budget-limited cases stay inconclusive.

- [ ] Test constant-return optimizer cannot claim spatial convergence from repeated same grid.
- [ ] Test profile convergence does not imply line optimizer or mesh convergence.
- [ ] Implement explicit physical-space grid/control/refinement ladders with matched baselines; reuse map but never silently substitute a coarser final grid.
- [ ] Persist comparisons and uncertainty; bounded studies split into meaningful separately named runs, not restart-to-evade deadlines.
- [ ] Demonstrate MIS/FSC fixed-geometry profile comparison first; line-control convergence only if budget supports actual multi-control optimization. Report inconclusive honestly.

## Stage 4: Tracking controller and lap-suite replay

**Files:** solvers/forward_driver.py or focused new yaw_tracking_driver.py; existing controller tests; runner/lap-suite CLI and diagnostics.

**Interfaces:** preserve default PreviewDriver compatibility unless new mode explicitly selected. Model-aware acceleration-to-control conversion optional, with explicit actuator limits. Controller consumes optimized path/speed plan, not track centreline by accident.

- [ ] Add synthetic tests for straight acceleration demand, drag compensation, braking exclusivity, steering/yaw-rate damping, finite saturation and preview behavior.
- [ ] Replace fixed acceleration/6 throttle mapping in new mode with available drive/brake authority model; never force full throttle merely to improve metric.
- [ ] Diagnose prior near-seam0.198m excursion using reference geometry and recorded errors. Add speed-dependent lookahead/feedforward/damping changes supported by tests, not arbitrary cap reduction.
- [ ] Persist time-weighted speed error, lateral error, steering/pedal saturation, sampled corridor violation and completion. Keep centre/footprint/continuous containment separate.
- [ ] Replay matched MIS/FSC cases with current accepted map and plan, under1800s per run. If tracking fails retain trace and concrete cause; no blanket qualification claim.

## Completion evidence

- Large map initial seed budget failure eliminated with regression and accepted-map probe.
- Bounded nontrivial line search and explicit convergence diagnostics in suite.
- Improved controller tested and at least matched replay comparison; completion/containment measured independently.
- Current focused tests and meaningful physical probes pass, independent code review completed, docs state residual limitations.
