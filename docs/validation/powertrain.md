# P10 powertrain validation

## Scope and execution limits

This report validates the P8 assembled powertrain dispatch and the P9 energy interfaces at
Validation baseline `60043f3` (P8/P9/P10 artifacts plus cached motor-loss interpolation and bounded torque-solve optimization). The full assembled point-mass solve is slower than the legacy
scalar MIS solve (the nested voltage/storage bracket is bounded, but is still evaluated at
every spatial node). Consequently, the focused regression run below is intentionally bounded:
it runs the motor-map tests, the MIS regression, P9 energy tests, and the lightweight
high-speed constraint test. Full assembled MIS flying and standing-start benchmarks are reported below. The complete
suite now passes in 116.8 s (194 tests) after caching the loss-map grid; the cache removes
repeated per-query grid reconstruction without changing interpolation results.

## MIS reference and full-model checks

The unchanged MIS track has 1,407 nodes and 1,014.32575 m. The validated legacy point-mass
results are the reference for the full model's unchanged vehicle/track data:

| boundary | lap time (s) | mean speed (m/s) | peak vehicle speed (m/s) |
|---|---:|---:|---:|
| flying | 69.3558 | 14.6250 | 28.9352 |
| standing start | 70.5198 | 14.3836 | 28.9352 |

The full assembled vehicle loads the same tire, aero, transmission, motor and storage
assembly. With cached motor-loss interpolation, assembled MIS flying and standing-start
solves each complete in about 19 s on this machine (two inner iterations), meeting the
single-lap runtime target. Results are:

| boundary | assembled lap time (s) | legacy reference (s) | difference (s) |
|---|---:|---:|---:|
| flying | 69.2013 | 69.3558 | -0.1545 |
| standing start | 70.3642 | 70.5198 | -0.1556 |

Its motor-speed ceiling is 18,600 rpm, corresponding to 34.60897 m/s at the 0.193675 m
effective radius and 10.9 ratio. Thus the MIS peak has 5.674 m/s (16.4%) vehicle-speed
margin, or 3,054 rpm (16.4%) motor-speed margin. MIS therefore does **not** exercise
voltage-dependent field weakening; it is not a suitable voltage-regression track.

## Synthetic high-speed case

The synthetic case is a closed, straight-equivalent 1,200 m profile (large radius,
zero curvature), selected to reach the motor-speed ceiling. The analytic ceiling is

```
v_max = (18,600 rpm * 2*pi/60) * 0.193675 / 10.9 = 34.60897 m/s.
```

At 1% above this speed (34.95506 m/s), the assembled equilibrium reports
`active_longitudinal_constraint = motor_speed`, zero available drive force, and an
infeasible longitudinal equilibrium. At 98% of the ceiling (33.91679 m/s), drive force
is positive and the active constraint is not `motor_speed`. These checks are the purpose
of `tests/test_high_speed_regression.py`; they are deliberately inexpensive and do not
hide a long full-track solve.

For a transparent speed-limited sensitivity, the 1,200 m straight-equivalent traversal
is 34.673 s at 100% voltage/ceiling. Because the analytic corner/ceiling scales linearly
with available phase voltage, 95% and 90% voltage give 32.87852 and 31.14807 m/s, or
36.501 s (+5.27%) and 38.526 s (+11.11%), respectively. These are kinematic sensitivity
figures, not claims of a converged dynamic lap time; tire, drag, acceleration and SOC
coupling require the expensive assembled solver.

## Analytic voltage scaling and sensitivity

The map uses peak phase voltage `V_bus/sqrt(3)` (minus modeled drops), not line-to-line
voltage. The envelope is evaluated by speed-axis scaling:

```
T(speed,V) = T_ref(speed * V_ref/V),
corner_speed(V) = corner_speed_ref * V/V_ref.
```

With the measured 16,300 rpm reference corner, the idealized corners are 16,300,
15,485, and 14,670 rpm at 100%, 95%, and 90% available voltage. This is an analytic
scaling law, not an independently measured second voltage point: the two legacy `AMK_DD5`
files describe different machines and must not be treated as a voltage cross-check.
The loss map remains the reference map plus the closed-form copper correction from the
field-weakening direct-current root; iron loss is conservatively held at its reference
value. Near and above the corner, torque/speed limitation dominates the small loss
correction.

## 22-lap endurance SOC study (P9 interface)

The endurance count is 22 laps. `solve_stint(..., laps=22, regen_enabled=False)` is the
canonical study entry point and carries each lap's final SOC into the next lap; each report
contains consumed/recovered/net energy, final SOC and minimum bus voltage. Regeneration is
explicitly disabled for this baseline, so recovered energy must be zero. A bounded P9
unit study confirms state carry-over and reports a positive usable-energy lap count; a full
22-lap assembled MIS run is intentionally not included in the focused evidence because of
the documented runtime cost. This is a limitation, not an extrapolated claim about whether
the pack completes 22 laps.

## Assumptions and limits

* Quasi-static battery terminal voltage; no thermal dynamics, imbalance, aging, or SOH.
* Fixed ratio 10.9 transmission and fixed motor count; no shifting or clutch dynamics.
* Reference loss map is held for iron/mechanical losses; only the specified copper correction
  changes with field weakening.
* Constant identified machine parameters and peak-valued d/q equations; the corner formula
  is directional/linear guidance, not an absolute measured-speed predictor.
* Unsigned-radius, planar track; no banking, elevation, transient tire or suspension model.
* Perfect-limit driver and point-mass force equilibrium.
* The synthetic sensitivity numbers are bounded analytic/kinematic evidence. A complete
  high-speed assembled lap and 22-lap MIS energy run remain expensive follow-up validations.
