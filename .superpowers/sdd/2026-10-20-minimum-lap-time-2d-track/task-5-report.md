# Task 5 implementation report — Hermite–Simpson quasi-steady free-line transcription

## Scope and contract

Implemented Task 5 on the existing `feature/minimum-time-line-2d-track` branch, based on base commit `eaa5466f8840461a89341c4369994ca3e9ba0f9d`. Added the quasi-steady free-line NLP and its focused tests. The implementation consumes the existing `Corridor` and `CapabilitySurface` data contracts without modifying either.

The decision vector is grouped as required: `[n_0..n_N | psi_0..psi_N | v_0..v_N | a_0..a_N | dpsi_0..dpsi_N | a_mid_0.. | dpsi_mid_0..]`, with size `5(N+1)+2N`. States are `(n, psi, v)` and state rates are `(dn/ds, dpsi/ds, dv/ds)`. Right-positive offset follows the corridor convention.

## Implementation

- Added `qss_line_ocp.py` with `QssSettings`, `QssSolution`, mesh/sample helpers, NLP builder, grouped-vector unpacker, initial guess, fixed-mesh solver, and periodic coarse-to-new-mesh seed projection.
- Formulated Hermite–Simpson midpoint states, collocation defects, and Simpson quadrature for `dt/ds`.
- Enforced corridor bounds, heading/speed/control bounds, fold guard, vehicle-curvature range, sampled acceleration envelope, and speed ceiling at start/mid/end points, plus periodic state closure.
- CasADi is imported only within `build_ocp` and `solve_line`; importing this module does not require CasADi.
- `solve_line` validates seed vector size against the supplied mesh. It does not assume a ladder's next mesh size; `project_seed` explicitly projects the full node solution to a requested mesh and returns properly sized midpoint blocks.
- Added six focused tests: all five numerical/layout tests from the brief plus a solver-boundary test rejecting open tracks. The OCP requires periodic closure for its state boundary constraints and rejects nonperiodic corridors as well.

## Tests and exact output

Command:

```text
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_qss_line_ocp -v
```

Output:

```text
[Ipopt startup banner]
test_decision_layout_matches_the_declared_contract (tests.test_qss_line_ocp.TranscriptionTest.test_decision_layout_matches_the_declared_contract) ... ok
test_left_turn_free_line_hugs_the_negative_edge (tests.test_qss_line_ocp.TranscriptionTest.test_left_turn_free_line_hugs_the_negative_edge) ... ok
test_mesh_ends_on_the_lap_length (tests.test_qss_line_ocp.TranscriptionTest.test_mesh_ends_on_the_lap_length) ... ok
test_open_track_is_rejected_at_periodic_solver_boundary (tests.test_qss_line_ocp.TranscriptionTest.test_open_track_is_rejected_at_periodic_solver_boundary) ... ok
test_right_turn_free_line_hugs_the_positive_edge (tests.test_qss_line_ocp.TranscriptionTest.test_right_turn_free_line_hugs_the_positive_edge) ... ok
test_zero_corridor_recovers_the_steady_circle_time (tests.test_qss_line_ocp.TranscriptionTest.test_zero_corridor_recovers_the_steady_circle_time) ... ok

----------------------------------------------------------------------
Ran 6 tests in 3.341s

OK
```

Also ran `git diff --check`, which completed without output/errors. Initial focused runs exposed CasADi interpolant API incompatibilities; these were corrected (grid definition and vector-valued query inputs), then the full focused suite passed. No earlier failure is being counted as a pass.

## Self-review and limitations

- Rechecked grouped variable and bound ordering, the three-component state-rate order, signed-curvature/right-positive geometry equations, and `1 - kappa0*n` fold guard.
- Solver statuses are surfaced as returned by Ipopt; the solver does not silently translate failed statuses into success. The tests require `Solve_Succeeded` and validate the numerical lap-time/offset results.
- The full repository suite was not run, consistent with the brief's warning about prior >120-second runs. The focused 6-test suite is the validation reported here.
- No material solver limitation was observed in the focused synthetic-circle cases; numerical behavior on arbitrary capability surfaces/tracks remains dependent on Ipopt convergence and surface coverage.

## Commit

Committed as recorded below, including only the Task 5 module, tests, and this report.
