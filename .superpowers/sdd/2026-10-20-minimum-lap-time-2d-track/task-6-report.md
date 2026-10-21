# Task 6: Coarse-to-fine mesh ladder

## Implementation

Added `solve_ladder(track, corridor, surface, settings, spacings=(5.0, 2.0, 1.0), *, seed=None)` to `qss_line_ocp.py`. It builds and solves each requested mesh in order, stops with a descriptive `RuntimeError` if a stage does not succeed, and passes a projected prior solution to the following stage. Crucially, projection is made directly onto the NEXT mesh, not the just-solved mesh. Existing grouped vector layout is preserved; `project_seed` returns all five node blocks followed by zero midpoint acceleration and curvature-rate blocks, matching the `5*nodes + 2*elements` decision layout. An empty spacing sequence is rejected.

Appended ladder convergence, empty-spacing, and mock-based next-mesh projection tests to `tests/test_qss_line_ocp.py`.

## Tests

Focused command:

```text
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_qss_line_ocp -v
```

Exact final output:

```text
******************************************************************************
This program contains Ipopt, a library for large-scale nonlinear optimization.
 Ipopt is released as open source code under the Eclipse Public License (EPL).
         For more information visit https://github.com/coin-or/Ipopt
******************************************************************************

test_empty_spacing_list_is_rejected (tests.test_qss_line_ocp.LadderTest.test_empty_spacing_list_is_rejected) ... ok
test_ladder_converges_and_is_mesh_independent (tests.test_qss_line_ocp.LadderTest.test_ladder_converges_and_is_mesh_independent) ... ok
test_next_stage_receives_next_mesh_projection (tests.test_qss_line_ocp.LadderTest.test_next_stage_receives_next_mesh_projection) ... ok
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
Ran 13 tests in 8.187s

OK
```

The ladder-only test was run first and exposed a `project_seed` interpolation bug (the periodic helper extended values but not source coordinates). Corrected it to interpolate the node values against the matching source nodes; then reran the full focused file successfully.

The full repository test suite was not run; only the focused OCP file is verified for this task.

## Self-review

- Verified next-stage seed exact size and values through a mock-based assertion.
- Verified default API spacings and supplied seed contract remain aligned with `solve_line`.
- No track XML or CSV files were changed.
- Implementation retains grouped node arrays and element midpoint blocks and does not add algebraic state.

## Limitations

- Failed/non-successful stage results raise rather than returning a partial list. This is intentional to avoid treating an unsuccessful intermediate point as a usable warm start.
- Projected midpoint control blocks are initialized to zero, per the existing `project_seed` contract.
- The full test suite was not run.

## Commit

`78895dbe0dc7c205015b5e145699e02c6f0fd75f` — `feat: solve the free-line problem on a coarse-to-fine mesh ladder`.
