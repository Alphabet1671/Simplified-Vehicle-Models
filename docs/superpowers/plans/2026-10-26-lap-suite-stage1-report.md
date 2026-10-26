# Stage 1 seed-bounding report

## Scope

Implemented only bounded initial candidate seeding for the coupled yaw-profile
solver. Strict interpolation, transition-oracle semantics, profile validation,
and physical yaw-map modules were not modified.

## Interface

`CoupledProfileSettings` adds:

- `initial_optional_seed_cap: int = 32`
- `refinement_reserve_per_station: int = 32`

Both are nonnegative integers. Existing `solve_coupled_profile` call signatures
remain unchanged.

Initial seeds are now diagnosed through:

- `initial_seed_counts`: one record per station with mandatory, optional,
  retained, map-speed offered/selected, curvature-lateral offered/selected, and
  reserved-room counts.
- `initial_optional_omitted`: aggregate optional proposal omissions grouped by
  `map_speed_knots`, `curvature_lateral_knots`, and deduplication/capacity.
- `initial_optional_seed_cap` and `refinement_reserve_per_station`.
- typed `budget_exceeded` diagnostics additionally report
  `mandatory_overflow_stations` when mandatory seeds alone exceed station cap,
  plus `initial_budget_cause` when even globally trimmed optional proposals
  cannot satisfy the total-node budget.

## Policy

Mandatory initial values are q-grid endpoints/baseline, valid warm q, and the
standing zero boundary. Map speed knots and lateral-curvature projections are
optional proposals. Each source is deterministically spread into its share of
the optional cap; the remaining capacity is deterministically shared. The
reserved room is not silently used by optional initial seeds.

No initial optional candidate is treated as feasible. Every graph edge retains
its strict transition query and independent profile validation. Refinement keeps
its existing visible candidate-budget behavior; this stage does not evict nodes
or discard an interval-component midpoint.

## Verification

Guarded focused run:

```sh
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 120 -- \
  python3 -m unittest tests.test_coupled_yaw_profile -q
```

Result: 13 tests passed in 2.953 seconds for the Stage 1 seed suite. Follow-up exact-edge rotation regression and profile/validator focused run: 40 tests passed in 4.243 seconds.

The new dense-map regression uses 80 speed knots and 450 lateral knots with the
normal 256 station cap. It verifies bounded optional counts, curvature-proposal
omissions, standing zero mandatory retention, and deterministic repeated seed
diagnostics. The synthetic solve may remain resolution-incomplete; this test
checks that map-grid density itself no longer causes initial candidate-budget
failure.

## Remaining Stage 1 work

A guarded accepted cached-map fixed-path probe is still required before
claiming the large physical map is practically usable. The transition oracle
still scans map axes per cache miss; seed bounding reduces requests but does not
coarsen its strict cell cuts.
