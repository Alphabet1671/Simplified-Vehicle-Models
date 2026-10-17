# Task 6 acceptance gates for the vertical 14-DoF model

Scope: adds the four remaining Task 6 acceptance gates (modal, inertance, energy,
planar reduction) as one new test module, plus this report. No source file and no
existing test was modified. Nothing is committed.

- New test module: [`tests/test_vertical_acceptance.py`](../../../../tests/test_vertical_acceptance.py) — 17 tests, all passing.
- Command: `PYTHONPATH=src python3 -m unittest tests.test_vertical_acceptance -q`
- Wall time: 62 s (three `setUpClass` hooks dominate: energy 47 s, planar 25 s, modal 0.1 s).
- Tested source revision (sha256, uncommitted worktree):
  - `src/simplified_vehicle_models/formulations/vertical_14dof.py` `36ab2b79c715407da02d06893dfe2b0dc6d82233b0562e4b053c35c78ff5d9b5` at authoring (mtime 2026-10-16 14:17:05). The evaluator was later edited (comment/import tidy only, per the hardening round); the final reviewer reran this module on revision `96c5c837…` and all 17 tests pass with the same numbers
  - `src/simplified_vehicle_models/solvers/vertical_equilibrium.py` `2494b12c4200a167d9da0e3619e14186c8d7d83aa4445fd611ed4640ddc0cae8`
- `git diff --check` clean; the untracked test file has no trailing whitespace, no tabs, no line over 120 columns.

The gates are independent oracles: the modal and energy gates rebuild `M` and `K`
in closed form and build the potential from the compiled curves, the hop gate
derives the reference set from the quarter-car formula plus the ARB convention,
and the planar gate compares two separately written evaluators at the same forced
state.

## 1. Modal gate (`VerticalModalGates`, 6 tests, 0.1 s)

| Test | Oracle | Result |
|---|---|---|
| `test_speed_zero_static_state_is_stationary` | `generalized_forces == 0` (atol 1e-7), vertical rates 0 (atol 1e-9), `K == K.T` (1e-3) | pass |
| `test_heave_dominated_mode_lies_in_the_plan_band` | heave participation > 0.8 and frequency in 3.4-4.7 Hz | pass, 3.8315 Hz |
| `test_body_fixed_hop_set_matches_the_plan_quarter_car_reference` | `f = (1/2pi)sqrt((k_w+k_t)/m_u)`, rear warp adds `k_arb`; rear ARB pinned to 120 lbf/in, front 0 | pass |
| `test_full_vehicle_undamped_modes_pin_the_parameter_implied_values` | ledger pin 3.83 / 7.30 / 9.90 and 17.18 / 17.71 / 19.47 / 23.97 Hz (1 %) with eigenvector assignment | pass |
| `test_pitch_mode_is_outside_the_plan_ride_frequency_band` | pitch > 4.7 Hz (visible deviation, not a pass) | pass, 7.2989 Hz |
| `test_closed_form_mass_and_stiffness_reproduce_the_linearization` | hand-built `M`, `K` vs evaluator FD, frequencies to 1e-3 | pass, 5.5e-12 |

Full-vehicle undamped modes, with the mass-weighted participation of each
eigenvector (`[heave, roll, pitch, jounce FL, FR, RL, RR]`):

| Frequency [Hz] | Label (by participation) | Participation |
|---|---|---|
| 3.83148 | heave | 0.9328 / 0.0000 / 0.0202 / 0.0053 / 0.0053 / 0.0182 / 0.0182 |
| 7.29886 | pitch | 0.0075 / 0.0000 / 0.9140 / 0.0238 / 0.0238 / 0.0155 / 0.0155 |
| 9.89547 | roll | 0.0000 / 0.9506 / 0.0000 / 0.0127 / 0.0127 / 0.0120 / 0.0120 |
| 17.17610 | hop (rear-dominated bounce) | 0.0359 / 0.0000 / 0.0255 / 0.0353 / 0.0353 / 0.4340 / 0.4340 |
| 17.71280 | hop (front/rear mixed) | 0.0000 / 0.0008 / 0.0000 / 0.2708 / 0.2708 / 0.2288 / 0.2288 |
| 19.46531 | hop (front-dominated) | 0.0072 / 0.0000 / 0.1843 / 0.3472 / 0.3472 / 0.0570 / 0.0570 |
| 23.96987 | hop (warp/roll-coupled) | 0.0000 / 0.3589 / 0.0000 / 0.1560 / 0.1560 / 0.1646 / 0.1646 |

Body-fixed hop subsystem (heave/roll/pitch constrained; `K_hop` is block diagonal
by axle, no front-rear coupling):

| Quantity | Value [Hz] |
|---|---|
| front FL, FR (`(k_w+k_t)/m_u`) | 17.613839, 17.613839 |
| rear symmetric (`(k_w+k_t)/m_u`) | 16.630917 |
| rear warp (`(k_w+k_t+k_arb)/m_u`) | 17.819518 |

The four values lie within 3 % of the derived expectation set to 1e-6 relative,
and the plan's 17.61 / 16.63 references are both inside the set (0 % and 0.005 %).

Closed-form check inputs from the compiled parameters: `k_w = 78807.076 /
78807.076 / 61294.392 / 61294.392 N/m`, `k_t = 80417.876 / 80417.876 /
80655.635 / 80655.635 N/m`, `m_u = 13 kg` (all four), `k_arb,rear = 21015.220 N/m`,
`k_arb,front = 0`, `M` identical to the evaluator block to 0.0, `K` to 8.0e-12
relative (2.6e-6 N absolute), frequencies to 5.5e-12 relative.

**Deviation (gate iv), disclosed not claimed as a pass:** the pitch mode is
7.2989 Hz, outside the plan's 3.4-4.7 Hz band. The ledger ruling attributes this
to the user-reported D5 pitch radius (0.386 m); the test asserts `pitch > 4.7 Hz`
so the deviation is visible and cannot silently become a pass.

## 2. Inertance gate (`VerticalInertanceGates`, 3 tests, 0.1 s)

| Test | Oracle | Result |
|---|---|---|
| `test_inertance_adds_b_over_mr_squared_to_the_jounce_diagonal` | `M(b) - M(0) == diag(0,0,0, b_i/MR_i^2)`, body rows untouched | pass, exact |
| `test_small_inertance_leaves_the_undamped_heave_mode_unchanged` | heave shift < 0.2 % at b = 0.3 / 0.5 kg | pass, -0.0849 % |
| `test_body_fixed_hop_frequencies_drop_by_the_inertance_mass_ratio` | per-mode `sqrt(m_u/(m_u+b/MR^2))` on the mode's own axle, to 1e-9 | pass, exact |

| Inertance (F/R) [kg] | mass factor F/R | heave [Hz] (shift) | pitch shift | roll shift | hop set [Hz] |
|---|---|---|---|---|---|
| 0.3 / 0.5 | 1.010412 / 1.015769 | 3.82822 (-0.0849 %) | -0.1565 % | -0.1030 % | 16.37274, 17.43234, 17.43234, 17.54289 |
| 3 / 5 | 1.099688 / 1.147983 | 3.79873 (-0.8547 %) | -1.6392 % | -1.2028 % | 14.48708, 15.52246, 16.01712, 16.01712 |

**Deviation from the brief's parenthetical example:** the brief pairs "the
undamped heave mode is unchanged to < 0.2 %" with "e.g. 3/5 kg". At 3/5 kg the
measured shift is 0.855 %, i.e. the example is not "small" for that tolerance.
The mode has 4.7 % of its energy in jounce motion, so a 10-15 % hub-mass increase
does move the body mode; the shift is close to linear in `b` (-0.085 % at
0.3/0.5 kg, -0.19 % at 1/1 kg, -0.85 % at 3/5 kg, -1.90 % at 10/10 kg). The gate
therefore uses b = 0.3/0.5 kg for the 0.2 % assertion and *also* asserts that the
3/5 kg shift is between 0.2 % and 5 %, so the example's size is visible in the
test rather than silently adjusted. This is a physical sensitivity, not a defect:
the inerter is a ruled two-terminal hub-to-chassis device whose kinetic term can
only enter through jounce motion.

## 3. Energy gate (`VerticalEnergyGates`, 5 tests, shared integration 47 s)

Setup: dampers and tire vertical damping removed, `b = 0` and `b = 3/5` kg, speed 0
(the static solve is then an exact stationary point; at speed > 0 the static solver
carries no tire longitudinal force and drag leaves a pitch-link residual). The
planar and wheel states and the four steady transient tire states are held fixed,
which the test asserts by requiring identically zero tangential tire forces and
zero planar/wheel state values. RK45, `rtol=1e-10`, `atol=1e-12`.

Energy: `E = 0.5 qdot' M qdot + U(x) - U(x0)` with
`U = -(m_s g h + sum m_ui g z_body,i) + sum m_ui g j_i + sum int F_susp,i dj_i +
sum int N_i d(delta_i)`. `F_susp` is the compiled spring curve plus the ARB
half-rate, `N` the compiled elastic tire load through the low-load ramp, and the
geometry the sprung-CG arms. `dU/dx == -Q` was verified coordinate-by-coordinate
against the evaluator's generalized forces (agreement ~1e-7 relative), so it is an
independent invariant.

| Test | Gate | Measured |
|---|---|---|
| `test_holding_approach_freezes_the_planar_and_wheel_states` | zero slip forces, no lift, zero planar/wheel states | pass, max |Fx,Fy| < 1e-9 N |
| `test_nonlinear_energy_is_conserved_without_inertance` | drift < 1e-6, amp 1e-4, 2 s | **2.315e-8** (5689 points, 34220 evaluations) |
| `test_nonlinear_energy_is_conserved_with_inertance` | drift < 1e-6, b = 3/5 kg, 1 s | **1.226e-8** (2091 points, 12548 evaluations) |
| `test_quadratic_form_is_not_the_invariant_of_the_nonlinear_model` | documents the quadratic form's drift | **1.339e-4** on the same trajectory |
| `test_linearized_system_quadratic_energy_is_conserved` | quadratic form on the linear system, rtol 1e-12 | **2.331e-10** |

The quadratic form `0.5 dx' K dx` drifts linearly with amplitude (1.34e-3 at
1e-3 m, 1.34e-4 at 1e-4 m, 1.34e-5 at 1e-5 m), so reaching 1e-6 with it would
need a ~1e-7 m amplitude, below the integrator's own floor. The gate therefore
uses the nonlinear invariant, which meets 1e-6 with a 40x margin at 1e-4 m, and
the test pins the quadratic drift as evidence for that choice rather than
loosening a tolerance.

## 4. Planar-reduction gate (`VerticalPlanarReductionGates`, 3 tests, 25 s)

Reduced oracle: springs stiffened 1e4x **about their preload**
(`F_new(j) = F(0) + 1e4 (F(j) - F(0))`; scaling the whole curve would scale the
preload and move the equilibrium), the vertical coordinates solved to their forced
steady state on the evaluator's generalized forces with the planar velocities and
wheel speeds prescribed, and a test-only
`lateral_load_transfer_distribution` equal to the model's own elastic split. The
CG-height invariant is asserted (the sprung/unsprung split gives exactly the
planar 0.3000 m), and the chosen lateral accelerations keep the planar model's
wheel-lift saturation inactive (`|front transfer| < 0.5 * front axle`).

Measured relative errors versus `evaluate_planar_dynamics`, 30 m/s, sideslip
0.5 m/s, LLTD fitted per case (floors: accelerations 1e-3, yaw 1e-4, wheel
accelerations 1e-2, forces and loads 1 N):

| Case | fitted LLTD | loads [N] FL FR RL RR | ax | ay | yaw | wheel | loads | forces |
|---|---|---|---|---|---|---|---|---|
| coast (kappa 0) | 0.499265 | 426.12, 840.41, 566.24, 981.74 | 3.0e-8 | 2.6e-4 | 1.4e-3 | 3.8e-10 | 3.8e-4 | 5.0e-4 |
| drive (kappa +0.01) | 0.498093 | 319.04, 724.54, 681.16, 1089.77 | 3.5e-4 | 2.8e-4 | 3.3e-4 | 3.6e-4 | 3.8e-4 | 5.2e-4 |
| brake (kappa -0.01) | 0.500429 | 540.03, 947.17, 460.43, 866.87 | 3.4e-4 | 2.8e-4 | 8.0e-4 | 3.4e-4 | 3.7e-4 | 5.1e-4 |

All at or below 1.4e-3, i.e. the 1e-2 gate passes with a 7x margin. Vertical
acceleration residual of the frozen state: <= 3.8e-12 m/s^2.

**Two documented deviations from the literal plan text:**

1. **The aero is zeroed for this comparison.** With aero present the literal
   1e-2 comparison fails, and it fails for a reason in the *planar* model, not the
   vertical one (measured below). The zero-aero comparison still exercises the
   whole planar force path: slip computation, force rotation, wheel spin, yaw
   moment, and both load-transfer splits.
2. **The tires are not stiffened.** Scaling the loaded-radius coefficients by 1e4
   moves the static deflection (~9.7 mm) down to ~1 um, below `delta0 ~ 0.4 mm`
   of the evaluator's fixed `blend = 0.5 mm` low-load ramp, where the elastic load
   is exactly zero: no equilibrium exists. The ramp knot is a hard-coded default
   argument in the evaluator's `tire_deflection_load` call, so a test cannot
   stiffen the tire without a source change. The physical tire law is kept, and
   the springs carry the stiffening.

### Literal (aero-enabled) reduction: measured failure and its cause

At the same states with the aero enabled (fitted LLTD 0.499196 at 30 m/s):

| Speed | ax | ay | yaw | wheel | loads | forces |
|---|---|---|---|---|---|---|
| 30 m/s | 6.1e-13 | 6.0e-5 | **1.11** | 7.8e-10 | **1.04e-1** | **1.61e-1** |
| 5 m/s, sideslip 0.35 | 8.4e-10 | 3.6e-4 | **2.16e-1** | 4.0e-10 | **1.67e-2** | **2.98e-2** |

The rates that do not depend on the split (`ax`, `ay`, wheel accelerations) match
to 6e-5 or better; the yaw acceleration, loads, and forces exceed 1e-2. The
`test_aero_drag_couple_explains_the_planar_reduction_gap` test pins the cause with
an independent rigid-body oracle:

* Coasting, the tire longitudinal forces vanish and only the aero drag decelerates
  the car. The vertical model applies the drag at the sprung CG, so the rigid-body
  pitch balance about the total CG shifts load rearward by `D*(h_s - h_cg)/L`.
  Measured front-axle load for the vertical model versus
  `static + downforce*balance - D*(h_s-h_cg)/L`:

  | Speed | vertical front [N] | rigid-body [N] | difference |
  |---|---|---|---|
  | 5 m/s | 1292.8909 | 1292.8909 | -0.00000 |
  | 30 m/s | 2215.5641 | 2215.5641 | -0.00000 |

  The planar quasi-static rule instead books `m*a*h_cg/L = D*h_cg/L` to the
  front, i.e. it omits the drag couple. Measured gap versus the predicted
  `D*h_s/L + (9.81 - 9.80665)*m_total*f_front`:

  | Speed | planar front [N] | gap [N] | predicted [N] | relative |
  |---|---|---|---|---|
  | 5 m/s | 1298.2687 | 5.3778 | 5.3778 | -4.5e-14 |
  | 30 m/s | 2394.0209 | 178.4569 | 178.4569 | -1.6e-14 |

* The yaw moment is a near-cancellation of the large front and rear lateral force
  contributions, so a 0.4 % axle-load difference becomes 22 % of the yaw
  acceleration. This amplification, not a 22 % force error, is why the yaw
  acceleration is the most sensitive column.

Conclusion: the vertical model is the more physical of the two here and matches
the rigid-body moment balance to 1e-14 relative; the 1e-2 planar-reduction gate
cannot pass with aero present unless the *planar* quasi-static load rule is given
the drag/force-path couple (a `planar_7dof`/`quasi_static` change, outside this
task's file list). The 1e-2 tolerance was not weakened anywhere.

## 5. Mutation checks

Each mutant is the acceptance module copied to `/tmp/mutants` with one term
flipped (source files untouched); a mutation must make the relevant gate fail.

| Mutant | Failing tests |
|---|---|
| evaluator tire vertical load scaled +2 % (static and stiffness) | `test_speed_zero_static_state_is_stationary`, `test_body_fixed_hop_set_matches_the_plan_quarter_car_reference`, `test_closed_form_mass_and_stiffness_reproduce_the_linearization` (closed form 3.83148 vs mutated 3.85036 Hz) |
| tire vertical damping left at 400 N*s/m in the "undamped" vehicle | `test_nonlinear_energy_is_conserved_without_inertance`, `test_nonlinear_energy_is_conserved_with_inertance` |
| body lateral tire force sign flipped in the evaluator | `test_planar_rates_loads_and_forces_match_the_planar_evaluator` in all three cases and five quantities (`ay`, yaw, wheel, forces, loads) |
| ARB term dropped from the derived rear-warp expectation | `test_body_fixed_hop_set_matches_the_plan_quarter_car_reference` |

## 6. Open issues for the parent

1. **Literal aero-enabled planar reduction fails at 1e-2** (numbers above). Cause
   identified and pinned by oracle: the planar quasi-static load rule books the
   longitudinal transfer from the net acceleration and omits the aero drag couple,
   `D*(h_s - h_cg)/L`. No vertical-model defect was found; the vertical model's
   front-axle load matches the rigid-body value to 1e-14 relative. Recommend a
   ruling: either accept the zero-aero reduced oracle (implemented) or extend the
   planar load rule (out of scope here).
2. **Gravity constant mismatch across models.** `vertical_14dof` and
   `vertical_equilibrium` hard-code `9.80665`, while
   `core.conventions.GRAVITY = 9.81` is what the planar quasi-static load law uses.
   The effect is a constant `m_total*f_front*(9.81-9.80665) = 0.4327 N` on the front
   axle (3.4e-4 relative) and a 1 N offset in the total load. It does not break the
   reduction gate, but it is a real cross-model inconsistency worth a ruling.
3. **The brief's inertance tolerance/example pairing is inconsistent** (0.2 % with
   b = 3/5 kg gives 0.855 %). The gate uses b = 0.3/0.5 kg and exposes the 3/5 kg
   value. No model change recommended.
4. **Tire stiffening x1e4 is not representable in a test** without changing the
   evaluator's hard-coded 0.5 mm ramp knot; the brief's "very stiff tires" clause
   is therefore documented as unimplemented.
5. **Pitch-mode plan-band deviation remains** (7.2989 Hz vs 3.4-4.7 Hz), as ruled;
   test asserts the deviation is outside the band.
6. **Task 6 still has no `solve_steady_corner`** as of this revision; none of the
   gates in this module depend on it, so adding it later should not disturb them.
   If `vertical_14dof.py` changes after this report, rerun
   `PYTHONPATH=src python3 -m unittest tests.test_vertical_acceptance -q` and
   compare the source sha256 above.
