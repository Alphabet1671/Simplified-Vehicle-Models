# No-TV Minimum-Lap-Time Reliability Implementation Plan

> **For agentic workers:** Use `subagent-driven-development` with per-task implementation/review gates. Execute; the user has already approved implementation.

**Goal:** Implement audit steps 1–3: shared optimized/exported/replayed geometry with station mapping, strict independent validation, and measured robustness on four real tracks plus cut-open MIS/FSC, without TV logic.

**Architecture:** A differentiable XY reference curve owns its actual metric with respect to source station. An offset trajectory owns the existing offset spline and coefficients, derives all geometry analytically, and provides reference/path station mapping. The spline OCP, reconstruction and validator consume that same trajectory. Strict result tiers distinguish optimizer success from numerical validation; a reproducible bounded matrix measures convergence without tuning acceptance criteria.

**Tech Stack:** Python, NumPy, lazy SciPy, lazy CasADi/IPOPT, unittest.

**Spec:** [Readiness audit](../../dev/minimum-lap-readiness-2026-10-22.md), priorities 0–1 and recommended steps 1–3, plus the user's instruction “Carry out 1-3 and implement. Use no TV logic first.” This plan supersedes inconsistent older formulas for the new spline path only.

## Global Constraints

- No TV implementation, controller performance claims, merge or push.
- Preserve every source track XML/CSV; reference interpolation is derived in memory and its fit discrepancies must be recorded.
- Objective is physical elapsed time, not centerline proxy distance. If reference XY derivative has norm `h`, do not assume `h=1`.
- Offset positive right; X north, Y east. Periodic closed and free-endpoint open geometry share the same equations.
- Preserve legacy defaults and its regression evidence; spline remains opt-in during validation.
- An unavailable required check is NOT a passed numerical-validation gate. Failed optimizer iterates are not accepted results.
- No post-optimization smoothing/re-fitting of an accepted spline trajectory.
- Fixes/tests/benchmarks must be reproducible and bounded; record all statuses, not only successful attempts.

## File ownership / task dependencies

1. `tracks/line_trajectory.py`, `tests/test_line_trajectory.py`: analytic reference/offset geometry and station maps.
2. `solvers/qss_spline_ocp.py`, `tracks/racing_line.py`, `solvers/closed_loop_line.py`, runner and associated tests: wire exact trajectory through OCP/export/replay; retain midpoint controls and evaluator metadata.
3. `solvers/line_validation.py`, runner and validation tests: independent dense trajectory/control validation, strict statuses and full-gate aggregation.
4. `tools/diagnostics/verify_no_tv_minimum_lap.py`, diagnostic tests, developer report: bounded four-track/cut-open refinement/multiseed matrix, targeted solver improvements after evidence.

### Task 1: Shared reference geometry and offset trajectory

**Files:** Create `src/simplified_vehicle_models/tracks/line_trajectory.py`, `tests/test_line_trajectory.py`.
**Consumes:** `TrackProfile.path_coordinates`, `OffsetSpline`.
**Produces:** `ReferenceCurve(track)`; `.evaluate(s)` returning north/east, first/second/third derivatives, metric `h`, metric derivative, tangent heading, signed curvature and curvature derivative with respect to *source s*. `OffsetTrajectory(reference, spline, coefficients)`; `.evaluate(s)` returning XY, tangent heading, heading error, curvature, heading-error derivative, offset, metric `dell/ds`, fold factor; `.to_path(s)` returning `PathCoordinates` without fitting another curve. `StationMap` with immutable reference/path arrays and `.to_path(s)` / `.to_reference(ell)`.

- [ ] Write failing tests using synthetic straight/circle and a nonconstant-offset/nonconstant-curvature curve. Test scalar/array shapes; finite differences of XY/tangent; periodic seam derivatives; open out-of-domain rejection; inward/outward circle length and sign; station round trip; no mutation of original arrays. Use unittest and tempfile only where fixture files are needed.
- [ ] Run `PYTHONPATH=src python3 -m unittest tests.test_line_trajectory -v`; confirm failure before implementation.
- [ ] Implement lazy SciPy reference interpolation: prefer a quintic interpolating XY curve (`make_interp_spline`, periodic closed; open not-a-knot) with sufficient samples, cubic fallback for short fixtures. Do not fit curvature independently. Reject degenerate tangents/nonfinite data and inconsistent closed positions; source station remains the parameter. Make arrays immutable copies.

Analytic equations (T=unit tangent, N=right normal, k=reference signed geometric curvature):
```python
h = hypot(x_s, y_s)
k = (x_s*y_ss - y_s*x_ss) / h**3
h_s = (x_s*x_ss + y_s*y_ss) / h
k_s = (x_s*y_sss - y_s*x_sss)/h**3 - 3*k*h_s/h
A = h*(1-k*n)
B = n_s
A_s = h_s*(1-k*n) - h*(k_s*n+k*n_s)
B_s = n_ss
metric = sqrt(A*A+B*B)
psi = atan2(B,A)
psi_s = (A*B_s-B*A_s)/(A*A+B*B)
path_curvature = (h*k+psi_s)/metric
north = x - n*y_s/h
east = y + n*x_s/h
```

- [ ] Integrate `metric` with high-order quadrature splitting at both reference/offset knots for StationMap (not chord length); include 0/end exactly and reject uncovered endpoints instead of clamping. Provide an explicit mapping accuracy setting and monotonicity check. Reuse one curve object per solve, not one fit per evaluation.
- [ ] Run tests. Circle approximation tolerance must be based on interpolation resolution, not exact identity of polygon data; no weakening finite-difference checks to hide inconsistency.
- [ ] Commit implementation and test files; report accuracy measurements and fitting-policy tradeoffs. Review before Task 2.

### Task 2: Integrate trajectory into optimization, export and replay

**Files:** Modify `solvers/qss_spline_ocp.py`, `tracks/racing_line.py`, `solvers/closed_loop_line.py`, `tools/run/solve_minimum_time_line.py`; extend associated tests.
**Consumes:** Task 1 geometry/metric/station interfaces.
**Produces:** `SplineSolution` retains an evaluable `trajectory` and midpoint acceleration/control interpolation metadata. Existing callers remain compatible via optional fields. Spline reconstruction directly calls trajectory `.to_path`; no new XY fit or offset linear interpolation. Replay samples carry explicit reference station or mapping.

- [ ] Add failing regression: nonconstant spline coefficients reconstruct the same XY/tangent/curvature as OCP evaluator on a dense grid; nonzero smoothing rejected for spline results; non-unit reference metric produces correct lap time; reference/path stations differ on inward circle yet cap application hits correct reference points.
- [ ] Replace separately sampled `k0/dk0` with Task 1 reference fields and symbolic twins of the analytic formulas. At node/midpoint time rate = `metric/v`; speed derivative = `a*metric/v`. Preserve node/midpoint speed lower bound, heading/rate/fold/capability limits, quarter corridor rows and zero-corridor handling. Use exact zero test on whole source corridor, not only OCP sample points.
- [ ] Return immutable geometry evaluator and midpoint acceleration. Evaluate dense speed by the actual HS cubic using endpoint rates; acceleration by the node/midpoint polynomial. Use these evaluators in later validation.
- [ ] Carry source station explicitly into replay; never interpolate path arclength as if reference station. Reject missing coverage/ambiguous mapping. Keep controller behavior unchanged, but shared station correction may be required. No new TV logic.
- [ ] Runner exports the exact sampled spline, station map and coefficient/control metadata; preserve provenance and reference fit discrepancy report. Exact path arclength is stored; loader chord tolerances must be tested rather than silently switched back to chords.
- [ ] Run `test_line_trajectory`, `test_qss_spline_ocp`, `test_racing_line`, `test_closed_loop_line`, `test_tv_runner`, `test_minimum_time_line_gate`. Commit and review.

### Task 3: Independent dense checks and strict validation tiers

**Files:** Create `solvers/line_validation.py`, `tests/test_line_validation.py`; modify runner and related tests.
**Consumes:** evaluable Task 2 trajectory/state/control and original capability/corridor.
**Produces:** serializable validation report with per-check status, worst location/value, sampling/refinement settings; distinct `solver_succeeded`, `replay_feasible`, `numerically_validated`, `physically_validated`. `accepted` aliases strict numerical validation, never “all available checks.”

- [ ] Add tests: fabricated between-node corridor excursion rejected; midpoint/quarter speed/capability violation rejected; any None/missing mandatory gate prevents numerical validation and CLI success; solver failure remains failure; synthetic dense-valid circle passes numerical checks; physical validation never implied by a model-to-model replay.
- [ ] Independently evaluate geometry via XY derivative finite-difference checks at off-collocation stations. Check corridor/fold/heading/rate and speed/acceleration capability using original numeric surface interpolation, plus dynamics defect of HS dense speed. Include offset/control knots and capability-critical regions; refine dense sample count until maximum residual stabilizes or report unresolved. A finite sample is documented numerical evidence, not continuous certification.
- [ ] Gate the final exported trajectory, not only solution nodes. Require geometry agreement, dense feasibility, mesh/basis convergence, matched baseline and replay for strict numerical validation; missing checks produce `incomplete`, not pass. Do not lower existing 0.5% mesh / 1% replay limits to obtain success.
- [ ] Report signed path length, reference fit discrepancy, station-map accuracy, controller-off mode, all missing evidence. Legacy results lacking new evidence stay usable as candidates but cannot be promoted silently.
- [ ] Run affected validator/runner/solver tests; commit and review.

### Task 4: Measured no-TV robustness and targeted numerical work

**Files:** Create `tools/diagnostics/verify_no_tv_minimum_lap.py`, `tests/test_no_tv_minimum_lap.py`, `docs/dev/no-tv-minimum-lap-reliability.md`; modify spline solver only when measurements identify a tested improvement.
**Consumes:** Tasks 1–3 and immutable `track_view` from boundary diagnostics.
**Produces:** reproducible per-case JSON, input hashes, failure diagnostics and staged convergence evidence for MIS/FSC closed+full-cut-open, skidpad and acceleration.

- [ ] Test experiment construction: same full geometry/width for paired closed/open, explicit rolling vs standing start, exact mesh/basis/seed recording, exclusive output creation, unavailable/failed stages cannot pass.
- [ ] Implement sequential bounded worker processes per case so a hanging solve can be terminated without losing prior artifacts. Record optimizer status/primal and dual residuals/iterations and failure messages. Serialize settings and source hashes before starting; no source writes.
- [ ] First use 5 m mesh / 35 m offset spacing / 200 iteration cap as an audit baseline, no smoothing, no TV. Then use a preregistered refinement sequence (5, 2.5, 1.25 m mesh and 35, 17.5 m control spacing) and deterministic centerline plus feasible-offset seeds on convergent cases. Compare actual metric/path/speed/curvature, not just time; run cut-open pairs before blaming topology.
- [ ] Diagnose production capability active cells and dense violations. Correct proven conditioning/model defects with tests; do not silently replace the production physics with constant or looser capability. Any continuation must finish at the original physical problem and tolerance. Record failed attempts too.
- [ ] Run independent no-TV point-mass replay on the exact trajectory, with matched initial speed/start treatment and same-condition centerline reference; explicit launch time/distance handling. If a start treatment is unsupported, fail the validation tier rather than substitute another condition.
- [ ] Publish achieved vs outstanding matrix and source hashes. Run full focused regression suite; separate full repository/GUI tests with bounded timeouts. Commit and obtain whole-change review. Completion requires robust convergence evidence, not merely harness implementation; if cases remain unresolved, leave the implementation goal active and report precisely what remains.
