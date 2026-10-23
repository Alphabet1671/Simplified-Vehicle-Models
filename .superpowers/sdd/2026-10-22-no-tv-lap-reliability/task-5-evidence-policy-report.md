# Task 5 evidence-policy correction report

## Scope

This correction changes only the no-TV evidence harness, its focused tests, and
this curated task report. It does not change the numerical solver, model,
tolerances, TV sources, or capability data.

## Corrections

1. `actual_initializer.seed_sha256` and all full profile diagnostics remain in
each attempt for audit. Refinement matching instead uses a stable canonical
identity: requested initializer mode/resolved options, actual profile use,
fallback category, and implementation/runtime provenance. Mesh/basis-derived
vector bytes, dimensional mapping, and elapsed profile diagnostics cannot
create a false refinement mismatch.
2. The six CLI geometry/state budgets remain persisted diagnostics only.
`metric_diagnostic_within_user_budget` records whether a comparison is within
the supplied budgets, but it is explicitly nonqualifying. There is no approved
curated source-hashed geometry/state convergence policy or data, so
`geometry_state_convergence` is `None`; strict edges and strict aggregate
remain incomplete even if all dense, reference, replay, and time flags pass.
The established 0.5% reference-time and 1% replay limits are unchanged.
3. The matrix top manifest now persists the resolved actual `base_commit` and
`git_head`, separate `requested_base_commit` and `baseline_label`, plus
`provenance_initial`, `provenance_afterfinal`, and source snapshots. The legacy
`base_commit` key is retained with its accurate resolved-HEAD meaning.

## Verification and limitation

Focused mocked-artifact tests cover differing seed vector SHA/length/timing
under unchanged policy, and reject requested mode/options/fallback changes;
they also prove 1e99 user budgets cannot qualify an edge. No numerical matrix
was run. Strict full-study acceptance remains **false / incomplete pending an
actual preregistered, approved geometry/state convergence policy and data**.
