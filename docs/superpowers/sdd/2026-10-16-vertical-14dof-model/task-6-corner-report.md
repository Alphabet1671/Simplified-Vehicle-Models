# Task 6 corner report: `solve_steady_corner` (steady corner solve, LLTD gate)

Scope: `solve_steady_corner` for the vertical 14-DoF model, plus the tests in
`tests/test_vertical_equilibrium.py::SteadyCornerTests`. Files touched:
`src/simplified_vehicle_models/solvers/vertical_equilibrium.py` and
`tests/test_vertical_equilibrium.py`. Nothing was committed, merged or pushed.
`src/simplified_vehicle_models/formulations/vertical_14dof.py` was **not**
edited (the concurrent longitudinal-closure work landed before this solve was
measured; see the revision hashes below).

Revision measured against (sha256, first 16 hex):

| file | sha256[:16] |
|---|---|
| `formulations/vertical_14dof.py` (evaluator, not mine) | `36ab2b79c715407d` |
| `solvers/vertical_equilibrium.py` (mine, at the measured revision) | `2494b12c4200a167` |
| `tests/test_vertical_equilibrium.py` (measured revision) | `fe3706551169c7b7` |

The evaluator hash was re-read at the end of the run and had not changed since
the first measurement, so every number below is against the settled Form B roll
closure and the settled longitudinal (drag/thrust, anti-dive/squat) closure.

## 1. What was implemented

`solve_steady_corner(vehicle, speed, lateral_acceleration) -> VerticalState`
(m/s, m/s^2, positive = right turn; `ValueError` for non-finite input or
`speed < 3 m/s`). It wraps a private
`_solve_steady_corner_detail(...) -> SteadyCornerSolution`, a frozen dataclass
with `state`, `steering`, `driven_torque`, `wheel_torques`, `tire_states`,
`residual_inf`, `iterations`, `continuation_steps` and the final
`VerticalEvaluation`.

Unknowns (14), in order: `[steer, beta, T, heave, roll, pitch, z_fl, z_fr,
z_rl, z_rr, k_fl, k_fr, k_rl, k_rr]` with the design's scales
`[0.1, 0.05, 20, 0.01, 0.01, 0.005, 0.01x4, 0.05x4]` and central-difference
steps `[1e-6, 1e-6, 1e-4, 1e-6, 1e-7, 1e-7, 1e-6x4, 1e-6x4]`.

Residuals are `evaluate_vertical_dynamics(...).rates` at
`[vx_dot, vy_dot, yaw_acc, heave_acc, roll_acc, pitch_acc, jounce_acc x4,
wheel_acc x4]`, divided by the design's independent scales (`g`, `g`,
`m_total g L / I_yaw`, `g`, `m_s g half_track / I_roll`,
`m_s g (L/2) / I_pitch`, `(m_total g / 4) / m_unsprung_front` x4,
`r_1 (m_total g / 4) / J_wheel` x4).

Path target: `r = ay_target / V`, `v_x = V cos(beta)`, `v_y = V sin(beta)`, so
the evaluator's `ay - r v_x = 0` row gives body `ay = ay_target cos(beta)`.
Wheel speeds are the exact inverse of the evaluator's slip ratio,
`omega_i = (k_i max(|v_l,i|, 1) + v_l,i) / R_i` with `R_i` the loaded radius;
per-corner tire states are `steady_rolling_state(tire, transient_tire, sa_i,
sr_i, N_i)` built from the slips the evaluator re-derives, so the tire states
are consistent by construction.

Solution path: straight seed from `solve_static_equilibrium(vehicle, speed)`,
Newton with backtracking (`r_new < r_old (1 - 1e-4 lambda)` over
`lambda = 1 ... 1/1024`), then continuation in `0.1 g` steps warm-started from
the previous point, a kinematic-steer retry (`L ay / V^2`) on a failed step,
and step halving down to `0.005 g` before raising
`RuntimeError("steady corner failed to converge at ay=... (residual ...)")`.
Trial bounds: `|steer| <= steering_maximum_angle` (0.5236 rad),
`|T| <= drivetrain.peak_motor_torque` (108.727 N*m at the wheel),
`|z| <= 0.0318 m`, all four loads `> 2 N`, `|beta| <= 0.5`,
`|kappa| <= 0.5`. A rejected trial (including a `DomainError` from the
suspension travel or the damper table) counts as a failed line-search step.

Final validation on the converged point (all gates raise `RuntimeError` on
failure): unscaled `rates[[0,1,2,3,4,5,17..20]] < 1e-6`,
`out_of_range_count == 0`, no lifted wheel, `max|tire_state_rates| < 1e-8`,
and body `ay` equal to `ay_target cos(beta)` within `1e-6`.

One small robustness fix was made to the existing `solve_static_equilibrium`
in the same file: its backtracking line search now treats a `DomainError` trial
as a rejected step instead of letting the exception escape. Production values
are unchanged; it is needed for the stiff-tire reference vehicle below.

## 2. Evidence

```
$ PYTHONPATH=src python3 -m unittest tests.test_vertical_equilibrium -q
Ran 15 tests in 18.0s
OK

$ PYTHONPATH=src python3 -m unittest tests.test_vertical_equilibrium.SteadyCornerTests -v
Ran 11 tests in 21.0s
OK
```

`SteadyCornerTests` (11 tests, one shared class fixture, ~20 s):
`test_straight_solve_reduces_to_static_equilibrium`,
`test_production_straight_solve_balance_and_thrust_shift`,
`test_one_g_and_minus_one_g_converge_and_mirror`,
`test_positive_lateral_acceleration_signs_and_load_transfer`,
`test_whole_vehicle_lateral_moment_identity`,
`test_tire_states_are_steady_and_the_slips_recompute`,
`test_direct_jump_matches_continuation`,
`test_input_validation_and_unreachable_steering`,
`test_three_g_target_raises`,
`test_reference_roll_stiffness_matches_source_numbers`,
`test_reference_rigid_tire_roll_gradient_and_lltd`.

Oracles are independent of the solver's own residual rows: the static
equilibrium solver (a separate 7-equation implementation), Newton's laws for
the whole vehicle (`Sigma N = m g + downforce`, `Sigma y_i N_i = -m h_cg ay`,
`Sigma Fx = drag`), mirrored-vehicle symmetry, an independent recomputation of
the contact kinematics and slips through the public
`contact_velocities`/`slip_quantities`/`steady_rolling_state` path, finite
differences of the public corner model for the roll stiffness, and the plan's
D4-D7 reference numbers.

Mutation check (temporary edits, then reverted; `git diff --check` clean
afterwards):

| mutation | caught by |
|---|---|
| `yaw_rate = -ay/V` in the corner build | `RuntimeError: steady corner body lateral acceleration -9.80487 does not match 9.80487` (surfaces as a `setUpClass` error) |
| total-CG `wheel_positions` arms instead of `sprung_wheel_positions` for the tire deflection | `RuntimeError: steady corner tire-state rates exceed 1e-08 (maximum 2.072e-02)` |

Both mutations are the kind a real sign/kinematics slip would produce, and both
make the class fail rather than pass silently.

## 3. Measured production numbers (compliant tire, measured RC map)

**NOT acceptance values.** The Task 6 roll oracle is defined for the rigid-tire
reference case in section 4; these production values are reported only.

At 1 g, positive `ay` (right turn), production vehicle:

| V (m/s) | steer | beta | T (N*m) | roll | roll gradient | front LLTD | loads FL/FR/RL/RR | Newton it | continuation steps | solve time |
|---|---|---|---|---|---|---|---|---|---|---|
| 8 | 13.554 deg | +4.405 deg | 17.83 | -0.6601 deg | 0.6601 deg/g | 0.4966 | 1019/345/1135/452 | 43 | 10 | 1.13 s |
| 12 | 6.078 deg | +1.092 deg | 21.81 | -0.6696 deg | 0.6696 deg/g | 0.4885 | 1034/369/1208/512 | 43 | 10 | 1.10 s |
| 15 | 3.963 deg | +0.227 deg | 28.34 | -0.6705 deg | 0.6705 deg/g | 0.4873 | 1064/400/1264/566 | 42 | 10 | 1.18 s |
| 25 | 1.543 deg | -0.509 deg | 61.86 | -0.6697 deg | 0.6697 deg/g | 0.4860 | 1234/572/1522/822 | 33 | 10 | 0.88 s |

Signs are correct (Form B): positive `ay` gives negative SAE roll, left tires
heavier on both axles, positive steering. `Fy/Fz` at 15 m/s: 0.87 / 1.02 / 0.77
/ 0.91, no lift, `out_of_range_count = 0`, `tire_state_rates = 0.0`.

Production roll gradient: **0.660-0.670 deg/g** (not gated). For scale: the
rigid-tire limit is 0.2897 deg/g closed form, and a naive series estimate of the
spring (2161.9 N*m/deg) with the tire roll stiffness
(`2 k_t (t/2)^2` = 1078.8 front + 1082.8 rear N*m/deg) gives 0.579 deg/g. The
solved production value is 16 % above that series estimate; the residual
difference is a solved nonlinear effect of the compliant corner (the tire
deflection feedback changes the jacking force the body reacts against) and was
not investigated further because it is not an acceptance value and nothing was
tuned. The measured RC map (heights held at the 1 g values above 0.289 deg of
roll) *reduces* the roll a little versus the constant static RC case:
0.6705 vs 0.6773 deg/g at 15 m/s.

## 4. Rigid-tire / static-RC reference vehicle (the Task 6 roll oracle)

Recipe (the brief states the oracle but not the build; this is the derivation
used, no tuning): `dataclasses.replace` of the compiled vehicle with

- `tire.r_2` and `tire.r_3` divided by **300** (stiff tire), and
- `suspension.front.roll_centre_height` / `...rear...` replaced by constant
  splines at the static heights **41.275 mm front / 57.15 mm rear**, keeping
  every other compiled part, in particular the rear ARB and both axles'
  wheel-rate springs.

Why 300 and not the brief's suggestion of 1e4: the loaded-radius inverse blends
a quadratic low-load ramp up to a *fixed* 0.5 mm deflection, and its parameters
are derived from the stiffness at that knot. Dividing `r_2`/`r_3` by more than
about 470 turns the ramp's lower knot positive (`delta0 > 0`) and the load law
stops being a monotone spring (measured: `a = 1000` gives `delta0 = +1.78 mm`,
`k1 < 0`, and the static solve leaves the travel domain). 300 is the largest
well-formed stiffening used here; about 1 % of the baseline tire roll compliance
survives, and the measured gradient at 15 m/s is 0.2916 against the 0.2897
closed form.

| V (m/s) | roll gradient | front LLTD | jounce FL/FR/RL/RR |
|---|---|---|---|
| 12 | 0.2905 deg/g | 0.4810 | +3.56 / -2.64 / +3.96 / -2.24 mm |
| 15 | 0.2916 deg/g | 0.4789 | +4.05 / -2.17 / +4.82 / -1.41 mm |
| 25 | 0.2923 deg/g | 0.4774 | +6.34 / +0.09 / +8.74 / +2.49 mm |

Against the plan's D4-D7 oracles (roll gradient **0.289 deg/g +/- 10 %**,
front LLTD **0.473 +/- 0.02**):

- roll gradient 0.2905-0.2923 deg/g -> inside [0.2601, 0.3179], +0.5 % to
  +1.1 % from the oracle;
- front LLTD 0.4774-0.4810 -> inside [0.453, 0.493], +0.004 to +0.008 from the
  oracle.

Independent closed form derived for the same case (and asserted in the tests,
from finite differences of the public `corner_forces` model):

- spring roll stiffness at the wheels: **1057.44 N*m/deg front, 1104.44
  N*m/deg rear** (total 2161.88) — the source spreadsheet's 1057 / 1104
  (plan reference numbers D4-D7) to 4 significant digits;
- roll-stiffness-weighted roll-centre height: 49.385 mm;
- rigid-tire roll gradient `m_s (h_s - h_rc) g / K_total` = **0.28968 deg/g**
  (the oracle's 0.289);
- rigid-tire front LLTD, from the load-transfer split
  `(K_f phi + h_f m_sf ay + 2 m_uf ay h_u) / (m h_cg ay)` with
  `phi = m_s (h_s-h_rc) g / K_total`: elastic 494.07 N, geometric 67.34 N,
  unsprung 84.10 N, total 1361.86 N -> **0.47399** (the oracle's 0.473).

The reference vehicle reproduces both oracles; nothing was tuned, and the
measured values are consistent with the closed form to +0.7 % (roll) and
+0.006 (LLTD).

## 5. Straight solve (`ay = 0`) versus `solve_static_equilibrium`

The corner solver cannot run below 3 m/s, so the reduction was checked two ways.

1. **No longitudinal force (drag-free glide).** With `aero.drag_area = 0` there
   is no drive force at all: `T = 0` and the evaluator's vertical block *is* the
   static system. Straight corner solve vs `solve_static_equilibrium` at
   5 / 15 / 25 m/s: heave, roll, pitch and all four jounces agree to
   **<= 2.7e-20 m** (the static seed is already the root; 0 Newton iterations).
   This is the 1e-6 reduction oracle in the test suite.
2. **Production vehicle.** With the settled evaluator the drag/thrust
   longitudinal load transfer makes the straight solve genuinely different from
   the static solve, as the parent flagged:

| V (m/s) | T (N*m) | drag*r_1/2 (N*m) | d heave | d roll | d pitch | max d jounce | Sigma N - (m g + downforce) | Sigma Fx - drag |
|---|---|---|---|---|---|---|---|---|
| 5 | 2.3003 | 2.4169 | +9.4e-7 | -2e-18 | +7.3e-5 | 3.1e-5 | 0.0 | 0.0 |
| 12 | 13.1861 | 13.9215 | +5.3e-6 | +3e-18 | +4.2e-4 | 1.8e-4 | 0.0 | 0.0 |
| 15 | 20.5359 | 21.7524 | +8.3e-6 | -1e-16 | +6.6e-4 | 2.8e-4 | 0.0 | 0.0 |
| 25 | 56.1224 | 60.4234 | +2.2e-5 | +4e-16 | +1.8e-3 | 7.8e-4 | 0.0 | 0.0 |

The test therefore asserts the drag-free reduction to 1e-6, and for the
production straight solve the load-sum identity (`Sigma N = m g + downforce`,
exact) and `Sigma Fx = drag` (exact), with the static-solve comparison in a
documented 2e-3 band (measured 6.6e-4 rad pitch, 2.8e-4 m jounce at 15 m/s) plus
the front axle load *decreasing* (thrust line below the sprung CG pushes load
rearward: 1462.2 N vs the static 1506.7 N at 15 m/s, -44.5 N).

`T` is a few percent below `drag r_1 / 2` (4.8 % at 5 m/s, 7.1 % at 25 m/s)
because the wheel equation also carries the tire rolling-resistance moment at
the small driving slip; the test allows 10 %.

## 6. Failure limits (measured, not assumed)

`RuntimeError` (never a partial state) at 3 g at every tested speed:

| V (m/s) | first unreachable target with the 0.1 g continuation + 0.005 g floor | time to raise |
|---|---|---|
| 3 | 2.807 m/s^2 (0.286 g, steering lock) | 2.9 s |
| 5 | 7.710 m/s^2 (0.786 g) | 3.7 s |
| 8 | 14.085 m/s^2 (1.436 g) | 5.9 s |
| 12 | 15.433 m/s^2 (1.574 g) | 7.4 s |
| 15 | 16.402 m/s^2 (1.673 g) | 7.8 s |
| 25 | 20.520 m/s^2 (2.093 g) | 6.2 s |

Maximum feasible lateral acceleration (bisection with the default
continuation, 0.01 g resolution, so the feasible set is bracketed):

| V (m/s) | max feasible ay |
|---|---|
| 12 | 1.570-1.576 g |
| 15 | 1.664-1.670 g |
| 25 | 2.086-2.092 g |

At 3 m/s the limit is the steering lock, not the tires: a 1 g corner needs about
2.9 rad of road-wheel angle against the 0.5236 rad maximum, so
`_solve_steady_corner_detail(vehicle, 3.0, 1 g)` raises in 2.9 s. This is the
"unreachable steering limit" failure path in the tests. Above 8 m/s the limit
is tire-limited and rises with speed as downforce adds load (1.57 g at 12 m/s,
1.67 g at 15, 2.09 g at 25).

## 7. Other gates

- **Mirror symmetry**: `+1 g` and `-1 g` at 12 / 15 / 25 m/s agree with the
  left/right swap to **<= 2.8e-14** in the state and `< 7e-15` in steer and
  torque (test tolerance 1e-8).
- **Whole-vehicle lateral moment identity** `Sigma y_i N_i = -m_total h_cg ay`
  holds to **9e-13 N*m** at 12 / 15 m/s and **2.8e-8 N*m** at 25 m/s with the
  *body-axis* `ay`. It must use the body `ay`: with the path target `V r`
  instead, the residual is `m h ay (1 - cos beta)` = 0.153 N*m at 12 m/s
  (`beta = 1.09 deg`), 0.0066 N*m at 15 m/s. Test tolerance 1e-6 N*m, chosen as
  the scaled roll residual (3e-11) times `m_s g half_track` = 163 N*m/N.
- **Direct jump vs continuation**: both reach the same root at `+1 g`, 15 m/s
  and `-1 g`, 12 m/s with `max|difference| = 7e-14`, and the direct jump now
  converges at `+/-1 g` at 8, 12, 15 and 25 m/s (`|diff| <= 1.2e-11`); the
  design note's failure at `-1 g`, 15 m/s no longer occurs with the settled
  evaluator and the kinematic-seed retry. Test tolerance 1e-8.
- Convergence: scaled `residual_inf` is 1e-14 to 3e-11 at 1 g; continuation
  takes 10 steps of 0.1 g at 12-25 m/s (13 steps at 1.3 g), 42-43 Newton
  iterations including the straight seed.

## 8. Deviations from the design doc

1. **Roll/LLTD sign.** The design note's prototype values (positive roll,
   heavier right tires) came from the pre-fix evaluator and are superseded. The
   settled Form B closure gives negative roll and heavier left tires; nothing in
   the solver needed a workaround.
2. **Reference-vehicle recipe.** Not specified anywhere; section 4 documents
   the derivation (300x stiff tire because 1e4 breaks the low-load ramp,
   constant static RC heights, rear ARB kept).
3. **Singular/ill-conditioned Newton systems.** The design said "if singular or
   `cond > 1e12`, `lstsq`; otherwise raise". The implementation uses `lstsq` in
   both cases (never raises from the linear solve); the subsequent evaluation
   and final validation still gate the result.
4. **`solve_static_equilibrium` line search.** One robustness change in the same
   file: a `DomainError` trial is now a rejected step instead of an exception
   escaping the method. Needed to solve the stiff-tire reference; production
   results are unchanged.
5. **`iterations`** counts total Newton iterations including the straight seed
   solve (`continuation_steps` counts only accepted `ay` steps), so a `1 g`
   continuation reports 42-43 iterations, not 39-40 as in the design note.
6. **Straight-solve comparison.** The design's "agrees with
   `solve_static_equilibrium` to 1e-6" cannot hold at speed with the settled
   evaluator; section 5 documents the resolution (drag-free reduction to 1e-6,
   exact load/force identities, 2e-3 band on the production static comparison).
7. **`continuation_step=None`** is an added private keyword that performs a
   single direct jump from the straight solution, used by
   `test_direct_jump_matches_continuation`. The public signature is unchanged.

## 9. Open issues

- **Tire-state rate gate is weak.** `tire_state_rates` is exactly 0.0 at every
  1 g solution because the tire states are built from the same slips the
  evaluator re-derives. The `< 1e-8` gate therefore mostly proves the build is
  consistent (the mutation check shows it does catch a kinematics error), not
  that the tire is in a physical steady state. A stronger oracle would seed the
  tire states from deliberately perturbed slips and check the relaxation, which
  belongs to Task 7.
- **Production roll gradient 0.670 deg/g vs the 0.579 deg/g naive series
  estimate** (section 3). The gap is not explained; it is a solved nonlinear
  corner effect, not a gate, and nothing was tuned. If the parent wants it
  explained, the natural next step is a roll-moment budget at the solution
  (spring moment vs the RC-jacking and unsprung transfer terms).
- **Low-speed feasible region.** Below about 8 m/s the steering lock, not the
  tire, limits the corner; the LLTD gate should stay at 12-25 m/s as the design
  says. At 8 m/s the fronts are already very unequal (`Fy/Fz` 1.22 / 0.58), and
  the production `LLTD` drifts with speed (0.4966 at 8 m/s, 0.4860 at 25 m/s)
  because the RC map is queried at the solved roll angle and the downforce
  grows with speed.
- **`(1 - cos beta)` in the moment identity** is a trap worth keeping: the
  path target and the body-axis lateral acceleration differ by up to 0.15 N*m
  of apparent moment error at 12 m/s. Task 7's driver observation uses
  `body_accelerations`, which is the correct one.
