# Task 5 — signed capability-profile transition maximality

## Design status

**Passed.** The signed-chart transition now enumerates physical-speed affine chart cells at each fixed midpoint curvature. For every cell it analytically enumerates quadratic energy roots, chart-cell boundaries, and the requested endpoint cap. It evaluates feasibility at the physical energy midpoint and selects the greatest endpoint in the connected feasible interval. If the enumerated feasible interval is disconnected, the profile reports `transition-domain-failed` rather than using a seed-dependent fixed point.

The implementation only makes this maximal reachable-endpoint claim for `SignedTransientCapability` chart transitions at/above the demonstrated floor. Generic legacy tables retain the reviewed fixed-point path. Standing-start rest callbacks also retain their bounded existing behavior: no global affine/maximality claim is made for an arbitrary callback below the chart floor.

The existing outer forward pass, backward braking preview, rolling/standing boundary pins, flying closure, and exact trajectory station map are unchanged.

## Actual signed acceleration result

The required exact rolling-10 acceleration diagnostic used:

- path artifact: `runs/no_tv_reliability/sixcase_signed_coarse_20261022/acceleration/mesh-5_basis-35_centerline/solution.npz`
- signed chart: `runs/no_tv_reliability/signed_lp_fullrange_product_20261022.npz`
- OCP comparison value: 4.508858 s (reference only; no OCP tuning or optimality claim)

| Profile step (m) | Status | Profile time (s) | Difference from OCP (s) | First profile speed (m/s) |
|---:|---|---:|---:|---:|
| 1.00 | converged | 4.508887051 | +0.000029051 | 10.331214 |
| 0.50 | converged | 4.508865211 | +0.000007211 | 10.167223 |
| 0.25 | converged | 4.508859698 | +0.000001698 | 10.084026 |

The former 5.34988 s profile slowdown is removed without changing a capability limit: the first segment now accelerates instead of retaining the previous under-driven branch. The refined profile sequence approaches the independent OCP candidate to near discretization agreement. This is a fixed-path sampled capability-table lookup reference only, **not** an empirical validation or a claim of unrestricted/global OCP optimality.

## Tests

`PYTHONPATH=src python3 -m unittest tests.test_capability_profile -v`

Result: **15/15 passed**. Coverage includes analytical variable-affine propulsion, backward signed braking preview, nonmonotone connected-branch selection, physical energy-midpoint bounds, existing rolling/standing/flying behavior, exact station mapping, and the actual signed acceleration artifact regression.

## Scope

Modified only:

- `src/simplified_vehicle_models/solvers/capability_profile.py`
- `tests/test_capability_profile.py`
- this task report

No torque-vectoring, solver/chart generation, source/cache writes, history rewrite, merge, or push was performed.
