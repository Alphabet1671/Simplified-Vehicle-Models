# Vehicle-limit yaw-map execution ledger

## Scope and rulings

- Worktree: yaw-aware-virtual-driver; baseline `26d42f4` (existing unrelated working-tree files preserved).
- Ruling: “vehicle limit” means the existing quasi-static LP model, not measured or transient vehicle limits. The finite sampling resolution is always disclosed.
- Ruling: preserve the declared longitudinal acceleration domain [-6, 3] m/s² for this lateral task, but record it explicitly; do not call this unrestricted combined-acceleration coverage.
- Ruling: use the actual UniTire peak table to enclose lateral acceleration (`4*max(Fy_peak)/mass`), not a replacement arbitrary g cap. The table itself clamps outside its load domain; that calibration/modeling limitation must remain visible.
- Ruling: artifacts with incomplete discovery or failed holdout acceptance must be rejected by normal consumers. Budget exhaustion is not physical infeasibility.

## Progress

- Discovery module and 8 synthetic tests added: signed support, disconnected sampled islands, empty slices, unresolved LPs, budgets, and finite tyre-derived bound.
- Direct LP status adapter added; 4 tests show only HiGHS status 2 is physical infeasibility, other solver failures/nonfinite objective are unresolved. Physical map builder now raises on numerical failure instead of silently masking it.
- Versioned NPZ artifact module added: exact axes, report/provenance, input/build identity, array integrity, fail-closed incomplete loading. 2 tests pass including tamper/stale/incomplete cases.
- CLI scaffold created with explicit ay spacing, discovery resolution, boundary/loss tolerances, node/probe/time budgets and declared ax extent. `--help` passes.
- Focused verification: 20 discovery/status/existing capability tests pass; 2 artifact tests pass.
- Workflow implementation delegated; parent review caught invalid loss metric (ay cell width cannot measure speed/ax-induced interpolation loss) and oversize-allocation budget violation. Both must be fixed before physical verification/default switch.

## Remaining

1. Finish/review workflow, especially true holdout-to-supported-domain distance and preallocation budgets.
2. Physical IEM24 build with persisted complete/incomplete evidence; tune resolution using evidence, not tolerance relaxation.
3. Shared runner/probe/preflight build/load integration (no silent ±1 g default).
4. Independent audit and regression suite, documentation, commits.
