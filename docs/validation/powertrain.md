# P10 powertrain validation

## Scope and execution limits

This report validates the P8 assembled powertrain dispatch and the P9 energy interfaces at
Validation baseline `5c65228` (P8/P9/P10 artifacts plus row-preserving cached motor-loss interpolation and bounded torque-solve optimization). The full assembled point-mass solve is slower than the legacy
scalar MIS solve (the nested voltage/storage bracket is bounded, but is still evaluated at
every spatial node). Consequently, the focused regression run below is intentionally bounded:
it runs the motor-map tests, the MIS regression, P9 energy tests, and the lightweight
high-speed constraint test. Full assembled MIS flying and standing-start benchmarks are reported below. The complete
suite now passes in 243.5 s (196 tests) after caching the loss-map grid; the cache removes
repeated per-query grid reconstruction without changing interpolation results.

## MIS reference and full-model checks

The unchanged MIS track has 1,407 nodes and 1,014.32575 m. The validated legacy point-mass
results are the reference for the full model's unchanged vehicle/track data:

| boundary | lap time (s) | mean speed (m/s) | peak vehicle speed (m/s) |
|---|---:|---:|---:|
| flying | 69.2265 | 14.6523 | 29.5823 |
| standing start | 70.3896 | 14.4102 | 29.5823 |

The full assembled vehicle loads the same tire, aero, transmission, motor and storage
assembly. With cached motor-loss interpolation, assembled MIS flying and standing-start
solves each complete in about 20 s on this machine (two inner iterations), meeting the
single-lap runtime target. Results are:

| boundary | assembled lap time (s) | legacy reference (s) | difference (s) |
|---|---:|---:|---:|
| flying | 69.2013 | 69.2265 | -0.0253 |
| standing start | 70.3642 | 70.3896 | -0.0253 |

Its motor-speed ceiling is 18,600 rpm, corresponding to 34.60897 m/s at the 0.193675 m
effective radius and 10.9 ratio. Thus the MIS peak has 5.027 m/s (14.5%) vehicle-speed
margin, or 2,701 rpm (14.5%) motor-speed margin. MIS therefore does **not** exercise
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

The endurance count is 22 laps. For a practical runtime, the study uses
`solve_stint(..., laps=22, regen_enabled=False, reuse_profile=True)`: the first lap runs the
state-coupled energy solve, then its solved speed/force profile is reused while storage
integration carries SOC across the remaining laps. Each report contains
consumed/recovered/net energy, final SOC and minimum bus voltage. Regeneration is explicitly
disabled for this baseline, so recovered energy must be zero. This is a fixed-profile
approximation: later-lap speed and force profiles are not re-solved as voltage/SOC changes,
so it is not equivalent to the default fully state-coupled stint (`reuse_profile=False`).
The approximation enables the complete 22-lap study without repeating the expensive profile
solve, but results should not be interpreted as a converged voltage-limited lap trajectory.
On the assembled MIS run, the 22-lap result was 613.34 s wall time, final SOC 0.333721,
and 17.934 MJ consumed/net energy; lap time stayed fixed at 69.1973 s by construction.
The report's remaining usable-energy estimate is 0.1117 equivalent average-energy laps
at the final SOC. The remaining runtime is the per-node storage and energy integration cost.

## Assumptions and limits

* Quasi-static battery terminal voltage; no thermal dynamics, imbalance, aging, or SOH.
* Fixed ratio 10.9 transmission and fixed motor count; no shifting or clutch dynamics.
* Reference loss map is held for iron/mechanical losses; only the specified copper correction
  changes with field weakening.
* Constant identified machine parameters and peak-valued d/q equations; the corner formula
  is directional/linear guidance, not an absolute measured-speed predictor.
* Unsigned-radius, planar track; no banking, elevation, transient tire or suspension model.
* Perfect-limit driver and point-mass force equilibrium.
* The synthetic sensitivity numbers are bounded analytic/kinematic evidence. The completed
  22-lap result uses the documented fixed-profile approximation; a fully state-coupled 22-lap
  MIS energy run remains an expensive follow-up validation.
