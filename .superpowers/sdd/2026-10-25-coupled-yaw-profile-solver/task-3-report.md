# Task 3 Report: Cell-Aware Yaw Transition Intervals

## Test result

Command run exactly as requested:

```text
PYTHONPATH=src:. python3 -m unittest tests.test_yaw_profile_transition -q
```

Result: **PASS** — 3 tests ran in 0.484 seconds.

```text
----------------------------------------------------------------------
Ran 3 tests in 0.484s

OK
```

## Strict-face numerical-guard rationale

The map remains strict; the numerical guard does not relax its strict faces. Random dense samples avoid face ULP slivers, where floating-point proximity can make randomized sampling misleading. Exact boundary checks are retained to verify strict-face behavior directly.

## Fix round 1

- Intersected the free endpoint range with both map speed-endpoint bounds before deriving midpoint-speed bounds; the fixed endpoint is rejected when outside that domain.
- Backward conversion now uses inward-guarded midpoint endpoints, matching the forward conversion.
- Each partition boundary is independently queried. Cell eligibility is based on the strict oracle at the open-cell midpoint, so a knot-pinned exact face is not rejected merely due to zero-weight adjacent masked corners.
- Root residual ambiguity and polynomial-vs-strict-oracle evidence disagreement now fail deterministically with `interval_oracle_unresolved`, including segment, support cell, and capability identity diagnostics.
- `edge_evidence` validates settings. Evidence disagreements are not papered over; they are explicit unresolved errors.

Focused rerun after fixes: `PYTHONPATH=src:. python3 -m unittest tests.test_yaw_profile_transition -q` — **PASS**, 6 tests. Broader yaw regression command `PYTHONPATH=src:. python3 -m unittest tests.test_yaw_profile_transition tests.test_yaw_profile_validation tests.test_yaw_aware_profile tests.test_yaw_profile_fixtures -q` — **PASS**, 29 tests.
