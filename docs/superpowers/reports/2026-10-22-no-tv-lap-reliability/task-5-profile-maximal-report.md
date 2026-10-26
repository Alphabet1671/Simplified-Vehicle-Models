# Task 5 — signed capability-profile transition maximality

## Design status

**Passed, after connectivity review.** The signed-chart transition enumerates physical-speed affine chart cells at each fixed midpoint curvature. For every cell it analytically enumerates quadratic energy roots, chart-cell boundaries, and the requested endpoint cap, then evaluates feasibility at the physical energy midpoint.

The selected endpoint must belong to one enumerated feasible component and must be directionally continuable from the actual current state. The local `ds → 0` family is `wj = wi + 2 ds a`, `a ∈ [amin(vi), amax(vi)]` (with the backward energy relation analogously reversed). Therefore this check deliberately does **not** require zero acceleration to be feasible: negative `amax` permits an immediate braking branch and positive `amin` permits an immediate accelerating branch. Conversely, an exact zero of the relevant bound *at the current endpoint* is an anchor barrier: it cannot justify motion toward a remote favorable branch. For a finite transition, interior analytic zeros of the relevant affine signed bound are likewise direction barriers; a candidate that crosses one is refused rather than jumping to a remote favorable branch. Multiple enumerated feasible components or an incompatible local/direction-barrier branch produce `transition-domain-failed`.

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

Result: **20/20 passed** in this artifact-bearing checkout. Coverage includes analytical variable-affine propulsion, backward signed braking preview, nonmonotone connected-branch selection, disconnected-current-prefix refusal, exact-zero forward/backward anchor refusal, negative-`amax` braking continuation, positive-`amin` acceleration continuation, physical energy-midpoint bounds, existing rolling/standing/flying behavior, exact station mapping, and the actual signed acceleration artifact regression. The actual-artifact test is explicitly skipped (while all synthetic regressions remain mandatory) when either ignored `.npz` artifact is unavailable in a clean checkout.

## Scope

Modified only:

- `src/simplified_vehicle_models/solvers/capability_profile.py`
- `tests/test_capability_profile.py`
- this task report

No torque-vectoring, solver/chart generation, source/cache writes, history rewrite, merge, or push was performed.
