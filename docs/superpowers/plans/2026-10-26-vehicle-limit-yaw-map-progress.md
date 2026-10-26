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

## Physical evidence and follow-up

- Direct signed physical discovery: 1,984 probes in 1.8 s, complete on 4 speeds × 3 ax slices, bracket width 0.03125 m/s². Steady |ay| limits 15.53125–15.5625 at 5 m/s, 16.75–16.78125 at15, 18.6875–18.71875 at25,20.5625–20.59375 at33. Under ax=-6 at33,22.5625–22.59375. Evidence in ignored runs/yaw_map_build/domain-smoke.json.
- Model-derived enclosing bound77.4102706856 m/s²; tyre peak load table0–3500N. This bound is a search enclosure, not actual achieved capability.
- Consumer default dispatch now vehicle-limit; explicit limited-grid only for comparisons. Tests updated to explicitly select legacy fixture mode, rather than silent fallback.
- Initial physical audit correctly fails; coarse v/ax cells lose real feasible support even after ay extent widens. Parent review caught acceptance gap using only cell-third samples; workflow now being repaired to audit independent refined direct boundary brackets.
- Added shared physical-cell cache (moments and status) across discovery/map/audit; no repeated LP solve for identical node.
- 35 domain/artifact/cache/status/consumer compatibility tests pass.

## Physical build — internal acceptance WITHDRAWN

- v7 completed after55 adaptive rounds:34×28×246=234,192 nodes, ay extent±25.1875m/s², worst strict boundary loss0.25m/s²; no outer truncations or unresolved points. 139,131 discovery LP-coordinate probes +126,124 independent cell-third discovery probes;2172 boundary acceptance comparisons;430.56s wall.
- Maximum tyre load among sampled feasible discovery ends2284N, within3500N tyre table. The table's clamping behavior remains a model limitation but was not reached at these sampled boundary points.
- Review fixes: numerical map-node failure now saves unresolved artifact evidence; singleton declared axes cannot qualify; transitive source hashes (21 files) retained, compiled runtime/code hash authoritative for cache invalidation.
- Fresh quarter-cell holdout verification delegated; final current-code artifact build running with consumer-compatible defaults.
- Broader yaw suite125 tests passed in663.7s before final review-only fixes; current focused31+ tests pass.

## Fresh holdout failure — binding correction

- Quarter-cell fresh direct audit (126,116LP coordinates/73.79s) found5 completely unsupported v/ax faces and finite loss1.21875m/s². No outer truncation. Therefore v7 internal complete is withdrawn; artifact is NOT accepted.
- Ruling: separate cell-third refinement/training from untouched quarter-cell validation. Adaptive insertions use enclosing CELL MIDPOINTS rather than audit coordinates, so fitted audit knot coincidences cannot manufacture success. Completion must require held-out audit too. Cost if wrong: larger tensor/time, but never relax strict support or claim false vehicle-edge coverage.
- Cancelled old final artifact rebuild; leave existing v7 as failed external-validation evidence only.

## Remaining

1. Finish/review workflow, especially true holdout-to-supported-domain distance and preallocation budgets.
2. Physical IEM24 build with persisted complete/incomplete evidence; tune resolution using evidence, not tolerance relaxation.
3. Shared runner/probe/preflight build/load integration (no silent ±1 g default).
4. Independent audit and regression suite, documentation, commits.
