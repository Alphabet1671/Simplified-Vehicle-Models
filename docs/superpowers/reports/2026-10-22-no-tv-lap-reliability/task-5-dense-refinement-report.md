# Task 5 dense-refinement correction report

## Scope and contract

This report covers the scoped correction in `src/simplified_vehicle_models/solvers/qss_spline_ocp.py` and `tests/test_qss_spline_ocp.py`, against immutable signed chart `runs/no_tv_reliability/signed_lp_fullrange_product_20261022.npz`. There is no TV logic, no source XML/CSV/cache change, no capability-profile change, no validator tolerance change, and no objective change. Results below remain **candidate-only**: no acceptance or actual reliability is claimed.

The test surface and full acceleration experiment use the physical original-model callback (`point_mass_view_of_planar(compile_formulation(iem24_murphy_real_planar_7dof.xml).model)` via `validate_line(..., vehicle=vehicle)`). Every reported solve had `model_feasible: true`, but that does not replace dense feasibility.

## Exact baseline diagnosis

Previously recorded strict-NLP results demonstrate two distinct mechanisms:

| mesh (m) | baseline worst chart acceleration | baseline worst implied acceleration / dynamics | interpretation |
|---:|---|---|---|
| 0.625 | +5.2754e-6 at 10.6960 m | +4.6059e-6 at 0.13613 m | chart interpolation and HS defect |
| 0.3125 | +1.5424e-6 at 32.5532 m | +6.6575e-7 at 3.12448 m | chart interpolation remains |
| 0.15625 | +6.3454e-7 at 20.7485 m | +1.9275e-6 at 75.14378 m | terminal HS defect dominates |

The signed chart has piecewise-linear `u` breakpoints every 0.0625. A quadratic control and Hermite speed can cross those faces between finite physical constraint sites. The resolved terminal behavior is also clear: on the last element the node/mid/control values are not a stationary constant-control continuation. For meshes `.3125/.15625/.078125 m`, respectively: endpoint acceleration is `2.01716/2.01108/1.95377 m/s²`, while the preceding node/midpoint values are approximately `2.05295/2.05369`, `2.04928/2.05083`, and `2.03194/2.04455 m/s²`. The reconstructed quadratic peaks inside the element. The worst dense dynamics station moves with the terminal element (`74.98755`, `75.14378`, `75.22189 m`), rather than converging to a fixed chart kink. This is a free-terminal-control/finite-HS representation defect, not a solver-status artifact.

## Implemented numerical correction

1. Every upper and lower capability acceleration row now has an interior lower bound of `8 * settings.tolerance` instead of zero. This is a *measured row-residual budget* (8e-6 m/s² for the normal 1e-6 contract), leaving capability slack at all nodes, HS midpoints, and existing off-sites. It does not claim continuous protection between sites, does not weaken the independent validator, and does not alter the objective.
2. IPOPT uses `acceptable_iter=0`. The public contract already rejects any status other than `Solve_Succeeded`; disabling the acceptable-level termination ensures IPOPT cannot return a looser status in a nominally strict solve. `_validate_solver_result` still checks primal constraints and bounds against the unchanged `settings.tolerance`.
3. No extra off-grid dynamics equalities or dynamics inequalities were retained. A bounded experimental inequality was rejected: it exhausted the `.3125 m` 200-iteration cap and still left `.15625 m` implied acceleration +1.529e-5 m/s². It would change the fixed-order HS discretization without a general error bound.

## Re-measurement (cap 200, tolerance 1e-6, rolling start 10 m/s)

| mesh (m) | status / iterations | time (s) | max acceleration violation | worst dynamics / implied acceleration | physical callback | dense feasible |
|---:|---|---:|---:|---|---|---|
| .3125 | Solve_Succeeded / 6 | 4.508868148 | -4.6655e-6 at 32.59225 m | +1.7675e-6 / +3.9518e-5 at 74.98755 m | true | **false** |
| .15625 | Solve_Succeeded / 7 | 4.508865548 | -5.7757e-6 at 20.72410 m | +3.9420e-6 / +8.8192e-5 at 75.14378 m | true | **false** |
| .078125 | Solve_Succeeded / 6 | 4.508866018 | -6.3027e-6 at 3.07566 m | +2.2004e-5 / +4.9244e-4 at 75.22189 m | true | **false** |
| .0390625 | Solve_Succeeded / 6 | 4.508865889 | -8.3824e-6 at 3.09641 m | +7.5168e-5 / +1.6825e-3 at 75.26094 m | true | **false** |

The correction measurably fixes the prior dense chart acceleration failures: each remeasured mesh has negative acceleration headroom beyond the unchanged 1e-6 validator tolerance. It does **not** yield two dense-feasible results because terminal reconstructed dynamics fail increasingly on blind uniform refinement. That is exact blocking evidence, not fabricated green acceptance.

## Regression evidence

`test_capability_rows_reserve_strict_interior_nlp_margin` evaluates a candidate exactly on a chart face and proves all stored upper-capability rows use `8*tolerance` slack. The existing varying-quarter chart regression still proves the true physical quarter-site bound is rejected. The existing 20 m constant-acceleration straight regression still passes its true quadratic-control dense bounds and analytic-time comparison. The off-site regression explicitly confirms the additional physical rows are inequalities, not extra dynamics equalities.

Focused full suite command and result:

```bash
PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20 python3 -m unittest tests.test_qss_spline_ocp -v
# Ran 20 tests in 156.339s — OK
```

## Blocking mechanism and next action

The remaining blocker is not chart interpolation or `Solved_To_Acceptable_Level`: all remeasured attempts above returned `Solve_Succeeded`, with NLP primal violation below 5.3e-9. It is the final-element cubic-Hermite speed versus quadratic control defect. Forcing terminal acceleration to `amax` is not generally valid (a path can legitimately be braking into a terminal corner); injecting more exact dynamics defects is unsound because nonlinear `dv/ds=a/v` is not generally cubic-Hermite representable. A future correction must use a bounded, validated adaptive terminal representation/refinement policy with a demonstrable interpolation/error bound and warm-start projection, then obtain two independently dense- and callback-feasible meshes before any reliability claim.
