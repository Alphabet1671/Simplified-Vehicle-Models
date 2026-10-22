# Task 5 control protection report

## Scope delivered

Implemented only the `qss_spline_ocp` protection work and its focused regressions. The decision-vector layout remains `[spline coefficients | speed nodes | acceleration nodes | midpoint acceleration]`; no time-varying logic, surface/chart construction, source data, cache data, objective, or acceptance tolerance changed.

## Solver protections

1. **Continuous global quadratic-control box.** Each element now constrains the quadratic acceleration's middle Bernstein coefficient
   `b1 = 2*am - 0.5*(a0 + a1)`
   to the same global minimum/maximum acceleration bounds used by the node and midpoint decision variables. Together with existing boxes on `a0` and `a1`, the Bernstein convex-hull property bounds the full continuous quadratic control. Signed charts retain their potentially negative global `amin`; no sign assumption was introduced.

2. **Off-collocation feasibility rows.** `QssSettings.off_collocation_fractions` defaults to `(1/8, 1/4, 3/4, 7/8)` and is validated as a nonempty tuple of distinct finite interior fractions excluding the HS midpoint. At every configured site, the solver uses the analytic `OffsetTrajectory`-twin geometry and the existing cubic Hermite speed / quadratic Lagrange acceleration to enforce: speed floor, heading, heading-rate, curvature-table domain, fold guard, speed domain/ceiling, acceleration capability, optional speed cap, and (for nonzero corridors) corridor containment.

3. **Dynamics unchanged.** The transcription retains exactly one HS speed defect equality per element. New off-grid rows are inequalities only. This deliberately does not claim the fixed cubic Hermite speed exactly satisfies the nonlinear distance-domain ODE between collocation points.

Warm starts and seed layout remain compatible because no decision block changed.

## Regression evidence

- `node=[3, 3, -0.6]`, `am=[3, 3]` produces last-element `b1=4.8`; the new global Bernstein row has upper bound `3.0`, rejecting the historic continuous peak (~3.45) although node/midpoint controls are legal.
- A straight 20 m, start-speed 10 m/s, `amax=3 m/s²` solve passed. On 257 dense stations, reconstructed quadratic acceleration remained within `[-8, 3] m/s²`; lap time agreed with `(sqrt(10² + 2*3*20)-10)/3` within 2 ms.
- A varying legacy capability surface has `amax=3` at nodes/midpoint but a local `amax=2` notch near the actual quarter-site HS speed `10.578125 m/s`; the new 1/4-site physical capability row is negative for the candidate, while its global Bernstein box remains legal.
- Dense speed-defect measurement on the straight launch was independent of the OCP rows: max `|dv/ds-a*metric/v|` was `1.2248e-5` at 8 elements and `1.7017e-6` at 16 elements. This is refinement evidence only; it was not made green by adding exact off-grid dynamics equalities or loosening the validator.

## Tests run

Passed with `PYTHONPATH=src:/tmp/casadi-runtime-2026-10-20`:

- Four focused control-protection/straight-launch tests in `tests.test_qss_spline_ocp`.
- `python3 -m unittest tests.test_line_validation -v` (14 tests).

The open-circle analytic-envelope parity regression was refined from a 0.5 m to a 0.25 m mesh after the stronger physical constraints changed the coarse discrete optimum by `0.003206 s`, above its unchanged 2 ms threshold. The refined mesh passes the unchanged threshold; no objective, solver tolerance, or validator tolerance changed.

## Limitations

A finite set of off-collocation sites improves protection but is not a continuous varying-surface certificate. Dense independent validation remains the acceptance gate, and unresolved Hermite dynamics residual requires mesh refinement.
