# Task 5 implementation report — Hermite–Simpson quasi-steady free-line transcription

## Scope and contract

Implemented Task 5 on the existing `feature/minimum-time-line-2d-track` branch, based on base commit `eaa5466f8840461a89341c4369994ca3e9ba0f9d`. Added the quasi-steady free-line NLP and its focused tests. The implementation consumes the existing `Corridor` and `CapabilitySurface` data contracts without modifying either.

The decision vector is grouped as required: `[n_0..n_N | psi_0..psi_N | v_0..v_N | a_0..a_N | dpsi_0..dpsi_N | a_mid_0.. | dpsi_mid_0..]`, with size `5(N+1)+2N`. States are `(n, psi, v)` and state rates are `(dn/ds, dpsi/ds, dv/ds)`. Right-positive offset follows the corridor convention.

## Implementation

- Added `qss_line_ocp.py` with `QssSettings`, `QssSolution`, mesh/sample helpers, NLP builder, grouped-vector unpacker, initial guess, fixed-mesh solver, and periodic coarse-to-new-mesh seed projection.
- Formulated Hermite–Simpson midpoint states, collocation defects, and Simpson quadrature for `dt/ds`.
- Enforced corridor bounds at mesh nodes, the Hermite–Simpson collocation midpoint, and Hermite-interpolated quarter, midpoint, and three-quarter points within each element. This is sampled corridor enforcement, not an analytic guarantee for every point of the continuous trajectory. Also enforces heading/speed/control bounds, fold guard, vehicle-curvature range, sampled acceleration envelope, speed ceiling at start/mid/end points, and periodic state closure.
- CasADi is imported only within `build_ocp` and `solve_line`; importing this module does not require CasADi.
- `solve_line` validates seed vector size against the supplied mesh. It does not assume a ladder's next mesh size; `project_seed` explicitly projects the full node solution to a requested mesh and returns properly sized midpoint blocks.
- Added fix-round regression coverage for grouped block offsets and order, invalid settings/mesh, and periodic track boundaries; six total tests exercise these plus all brief numerical cases. The OCP requires periodic closure and compatible track/corridor grids.

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
- Solver statuses are returned only for accepted `Solve_Succeeded` results; nonfinite values or constraint violation above tolerance raise `RuntimeError`. Rejection paths are covered by a pure helper regression.
- The full repository suite was not run, consistent with the brief's warning about prior >120-second runs. The fix-round focused 9-test suite is the validation reported here.
- No material solver limitation was observed in the focused synthetic-circle cases; numerical behavior on arbitrary capability surfaces/tracks remains dependent on Ipopt convergence and surface coverage.

## Fix round 1

Addressed the independent audit findings: added corridor bounds on each Hermite–Simpson midpoint offset; centralized validation for closed/periodic inputs, matching track/corridor grids, mesh endpoints, settings (including heading below pi/2), and capability surface axes/envelopes; changed curvature-rate regularization to Simpson quadrature scaled by element length; and reject non-success, nonfinite, or over-tolerance solver outputs. Periodic sampling now accepts an explicit period and does not extrapolate a uniform step beyond a nonuniform grid. Expanded layout checks to assert grouped block positions and state order, and added midpoint-corridor, invalid-input, and solver-result rejection regressions.

Fix-round focused command and exact output:

```text
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_qss_line_ocp -v
[Ipopt startup banner]
test_decision_layout_matches_the_declared_contract (tests.test_qss_line_ocp.TranscriptionTest.test_decision_layout_matches_the_declared_contract) ... ok
test_invalid_settings_and_mesh_are_rejected (tests.test_qss_line_ocp.TranscriptionTest.test_invalid_settings_and_mesh_are_rejected) ... ok
test_left_turn_free_line_hugs_the_negative_edge (tests.test_qss_line_ocp.TranscriptionTest.test_left_turn_free_line_hugs_the_negative_edge) ... ok
test_mesh_ends_on_the_lap_length (tests.test_qss_line_ocp.TranscriptionTest.test_mesh_ends_on_the_lap_length) ... ok
test_midpoint_offset_has_corridor_constraint (tests.test_qss_line_ocp.TranscriptionTest.test_midpoint_offset_has_corridor_constraint) ... ok
test_open_track_is_rejected_at_periodic_solver_boundary (tests.test_qss_line_ocp.TranscriptionTest.test_open_track_is_rejected_at_periodic_solver_boundary) ... ok
test_right_turn_free_line_hugs_the_positive_edge (tests.test_qss_line_ocp.TranscriptionTest.test_right_turn_free_line_hugs_the_positive_edge) ... ok
test_solver_result_rejects_failure_nonfinite_and_violation (tests.test_qss_line_ocp.TranscriptionTest.test_solver_result_rejects_failure_nonfinite_and_violation) ... ok
test_zero_corridor_recovers_the_steady_circle_time (tests.test_qss_line_ocp.TranscriptionTest.test_zero_corridor_recovers_the_steady_circle_time) ... ok

----------------------------------------------------------------------
Ran 9 tests in 3.473s

OK
```

## Fix round 2

Added interior corridor checks at the quarter, midpoint, and three-quarter Hermite interpolation points in every element. These are explicit sampled feasibility constraints, not a mathematical proof of corridor feasibility at every point of the continuous curve; arbitrary violations between sample points remain possible. Corrected track-curvature coverage to use the signed surface interval and included variable-bound violations in the solver acceptance maximum alongside constraint violations. Tests assert the three interior sample pairs per element (in addition to the independent HS midpoint bound), exercise asymmetric curvature coverage, and verify node-bound rejection.

Exact focused rerun:

```text
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_qss_line_ocp -v
[Ipopt startup banner]
test_decision_layout_matches_the_declared_contract (tests.test_qss_line_ocp.TranscriptionTest.test_decision_layout_matches_the_declared_contract) ... ok
test_invalid_settings_and_mesh_are_rejected (tests.test_qss_line_ocp.TranscriptionTest.test_invalid_settings_and_mesh_are_rejected) ... ok
test_left_turn_free_line_hugs_the_negative_edge (tests.test_qss_line_ocp.TranscriptionTest.test_left_turn_free_line_hugs_the_negative_edge) ... ok
test_mesh_ends_on_the_lap_length (tests.test_qss_line_ocp.TranscriptionTest.test_mesh_ends_on_the_lap_length) ... ok
test_midpoint_offset_has_corridor_constraint (tests.test_qss_line_ocp.TranscriptionTest.test_midpoint_offset_has_corridor_constraint) ... ok
test_open_track_is_rejected_at_periodic_solver_boundary (tests.test_qss_line_ocp.TranscriptionTest.test_open_track_is_rejected_at_periodic_solver_boundary) ... ok
test_right_turn_free_line_hugs_the_positive_edge (tests.test_qss_line_ocp.TranscriptionTest.test_right_turn_free_line_hugs_the_positive_edge) ... ok
test_signed_asymmetric_curvature_axis_rejects_outside_track_curvature (tests.test_qss_line_ocp.TranscriptionTest.test_signed_asymmetric_curvature_axis_rejects_outside_track_curvature) ... ok
test_solver_result_rejects_failure_nonfinite_and_violation (tests.test_qss_line_ocp.TranscriptionTest.test_solver_result_rejects_failure_nonfinite_and_violation) ... ok
test_zero_corridor_recovers_the_steady_circle_time (tests.test_qss_line_ocp.TranscriptionTest.test_zero_corridor_recovers_the_steady_circle_time) ... ok

----------------------------------------------------------------------
Ran 10 tests in 4.223s

OK
```

## Fix round 3 and Ipopt warning investigation

Removed the duplicate midpoint corridor equality: the Hermite interpolant at `t=1/2` is the same state as the independent Hermite–Simpson collocation midpoint. Corridor constraints now use the explicit HS midpoint plus Hermite quarter- and three-quarter samples—three distinct interior samples per element. The regression checks exactly three asymmetric corridor-bound rows per element. This duplicate caused avoidable equality constraints in the collapsed zero-width corridor case.

Reran the focused suite after deduplication; no Ipopt overconstraint warning appeared in this run. Exact output:

```text
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_qss_line_ocp -v
[Ipopt startup banner]
test_decision_layout_matches_the_declared_contract (tests.test_qss_line_ocp.TranscriptionTest.test_decision_layout_matches_the_declared_contract) ... ok
test_invalid_settings_and_mesh_are_rejected (tests.test_qss_line_ocp.TranscriptionTest.test_invalid_settings_and_mesh_are_rejected) ... ok
test_left_turn_free_line_hugs_the_negative_edge (tests.test_qss_line_ocp.TranscriptionTest.test_left_turn_free_line_hugs_the_negative_edge) ... ok
test_mesh_ends_on_the_lap_length (tests.test_qss_line_ocp.TranscriptionTest.test_mesh_ends_on_the_lap_length) ... ok
test_midpoint_offset_has_corridor_constraint (tests.test_qss_line_ocp.TranscriptionTest.test_midpoint_offset_has_corridor_constraint) ... ok
test_open_track_is_rejected_at_periodic_solver_boundary (tests.test_qss_line_ocp.TranscriptionTest.test_open_track_is_rejected_at_periodic_solver_boundary) ... ok
test_right_turn_free_line_hugs_the_positive_edge (tests.test_qss_line_ocp.TranscriptionTest.test_right_turn_free_line_hugs_the_positive_edge) ... ok
test_signed_asymmetric_curvature_axis_rejects_outside_track_curvature (tests.test_qss_line_ocp.TranscriptionTest.test_signed_asymmetric_curvature_axis_rejects_outside_track_curvature) ... ok
test_solver_result_rejects_failure_nonfinite_and_violation (tests.test_qss_line_ocp.TranscriptionTest.test_solver_result_rejects_failure_nonfinite_and_violation) ... ok
test_zero_corridor_recovers_the_steady_circle_time (tests.test_qss_line_ocp.TranscriptionTest.test_zero_corridor_recovers_the_steady_circle_time) ... ok

----------------------------------------------------------------------
Ran 10 tests in 3.815s

OK
```

## Commit

Committed as recorded below, including only the Task 5 module, tests, and this report.
