# Task 2 — Analytic trajectory integration

## Scope

Integrated the Task 1 `ReferenceCurve`/`OffsetTrajectory` through the opt-in spline OCP, spline reconstruction, replay cap plumbing, and runner export. No TV logic, source XML/CSV edits, acceptance-tier work, merge, or push was performed.

## API changes

* `SplineSolution` now optionally retains `trajectory`, `midpoint_acceleration`, `objective_value`, and `state_evaluator`.
* The spline OCP obtains reference metric/curvature and their source-station derivatives from `ReferenceCurve`, and its symbolic equations use the same analytic offset formulas. Physical elapsed time is returned in `lap_time`; raw NLP objective (including optional open regularisation) is separately retained as `objective_value`.
* Dense state evaluation uses the actual Hermite--Simpson cubic speed and node/midpoint quadratic acceleration. `speed_rate` is the cubic derivative; `rhs_speed_rate` is separately exposed as `a*metric/v` for future independent dynamics-defect checks.
* `PathCoordinates` supports optional `reference_distance`. Exact `OffsetTrajectory.to_path` populates it, and spline reconstruction directly calls it. Smoothing of an attached analytic spline trajectory is rejected.
* `ReplaySample.reference_distance` is optional for compatibility, but analytic trajectory replay refuses ambiguous resampled path stations. The exact exported path grid can carry its explicit map. Caps reject incomplete station coverage rather than clamping endpoints.
* Runner rebuilt-width export samples original cross-section geometry and widths by `reference_distance`, never rebuilt arclength. Geometry/width checks precede output writes. Spline summaries retain fit discrepancy, station-map error, physical path length, objective, and physical time.

## Tests

Focused suite passed:

```text
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest \
  tests.test_line_trajectory tests.test_qss_spline_ocp tests.test_racing_line \
  tests.test_closed_loop_line tests.test_tv_runner tests.test_minimum_time_line_gate -q
Ran 110 tests
OK
```

The runner/gate-only subset also passed 71 tests. Existing synthetic closed-loop replay fixtures were corrected to sample reference-grid speeds using `path.reference_distance`, rather than treating path arclength as reference station. A dedicated real `fsae_2026_skidpad` zero-offset regression exports through `_write_rebuilt_track` then loads the emitted XML, confirming adaptive exact arc-length sampling meets existing loader checks without source mutation.

## Discrepancies and concerns

The coarse circle fixture’s shared XY fit has finite curvature discrepancy (maximum observed about `7.3e-7 1/m`), so its assertion now tests interpolation-accuracy scale rather than exact equality to its stored curvature column. Solver constraints remain on the authoritative fitted geometry.

Adaptive export now uses `OffsetTrajectory.sample_path`, bounded source-station bisection against the existing loader's heading/curvature trapezoid, chord-heading, and chord-length limits. It changes neither the analytic curve nor integrated path distance and keeps explicit reference stations. Endpoint geometry evaluation in the dense evaluator is currently not cached; correctness is retained but future dense validation can optimize this.
