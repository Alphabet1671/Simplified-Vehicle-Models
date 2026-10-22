# Task 5: shared high-order physical-time quadrature

## Implementation

The spline OCP now defines one explicit physical-time representation:
`gauss_legendre_16_energy_hermite_knot_union_v1`.

For every mesh element, the integration panel boundaries are the union of the
mesh boundary, source-reference knots, and offset-spline control knots.
Near-coincident knots are merged using the same scale-aware `1e-11 * max(1,
length)` policy in both the symbolic and numerical paths.  At each GL16 site:

- geometry is evaluated from the analytic offset/reference trajectory;
- state is the existing cubic energy-Hermite reconstruction `W_H`;
- speed is `sqrt(W_H)`; and
- the physical integrand is `G / sqrt(W_H)`.

The CasADi objective is this sum exactly.  `SplineSolution.lap_time` calls the
same rule on the same panel construction.  The optional open-path
regularisation remains a separate Simpson penalty; it is included in
`objective_value`, not in `lap_time`.

`SplineSolution` now carries the named quadrature contract and metadata,
including GL order, nodes/weights, geometry-panel policy, and solved panel/site
counts.  Runner trajectory metadata persists that contract.

No validator, model, chart, tolerance, source XML/CSV, or cache behavior was
changed by this work.

## Evidence

Focused tests, using `PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20`:

- The low-start (`v0 = 0.5 m/s`) constant-acceleration straight test proves
  GL16 physical time against `2L/(v0+vend)` to `1e-11`, symbolic objective
  versus numerical reported physical time to `1e-12`, and GL16 versus GL32 to
  `1e-12`.
- A nonzero-offset, nonunit, variable reference-metric fixture proves CasADi
  objective versus numerical physical time to `2e-13`, and GL16 versus GL32 to
  `2e-12`.
- A near-coincident source/control-knot regression proves common knot merging.
- `test_zero_corridor_open_circle_matches_capability_envelope` passed unchanged
  at its existing 2 ms bound in 68.485 s.  This removes the prior reported
  +7.285889 ms Simpson launch bias without weakening that assertion.
- `tests.test_analytic_line_pipeline` passed: 6 tests in 2.681 s.
- Full `tests.test_qss_spline_ocp` passed: 26 tests in 465.746 s.  It was
  started before the separate validator follow-up agent's edits.
- Final focused quadrature + metadata set passed: 4 tests in 1.951 s.

The high-order knot-union objective is materially more expensive than the old
per-element Simpson expression: the open-circle 0.25 m regression above took
68.485 s.  The solution metadata exposes panel/site counts for future runtime
profiling; no cost claim beyond this measured regression is made.

## Actual signed rolling-10 diagnostic

One bounded actual acceleration-case attempt used the immutable
`runs/no_tv_reliability/signed_lp_fullrange_product_20261022.npz` artifact,
original Murphy callback, open rolling start 10 m/s, 0.15625 m mesh, 17.5 m
basis spacing, `maximum_iterations=1000`, and strict NLP tolerance `1e-10`.
It returned `Solved_To_Acceptable_Level`; the existing strict solver gate
rejected it before dense/model validation.  Thus it is not a successful dense
or live pass, and no two-pass reliability claim is made.

A separate diagnostic identified a central finite-difference artifact at a C1
knot on the exact energy fixture: energy-rate dynamics are exact, while a
central implied-acceleration estimate can be discontinuously inaccurate across
the knot.  This task did not modify the independent validator; any validation
follow-up must be separately reviewed.

## Status

The shared optimizer/exported/replayed physical-time contract and the original
open-circle 2 ms regression are complete.  The actual signed rolling-10
requirement remains open because the one strict attempt did not obtain
`Solve_Succeeded`, and no dense/model acceptance was reached.
