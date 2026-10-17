# Task 6 design: `solve_steady_corner`

Status: design note from a read-only investigation, with a scratch NumPy prototype kept **outside** the repo (`/tmp/sc_proto*.py`, not committed). No source file was edited. Every number below was measured with the evaluator as it stood at the time (before the concurrent mass/roll edit). Re-measure after that edit lands.

## 1. Prototype limitations (read first)

1. **Roll/LLTD sign was wrong in the evaluator at prototype time.** At +1 g (right turn) the prototype gave positive roll (+0.14°) and heavier *right* tires, with front LLTD about 0.49. The brief requires heavier left tires and negative SAE roll. The roll generalized force then had only the roll-centre jacking term (`Fy*h_rc`) and no sprung-mass inertial roll moment `m_s*ay*(h_s - h_rc)` and no unsprung lateral load transfer. The solver reads evaluator rates, so it inherits the fix automatically. **Do not pin any roll angle, roll gradient, or LLTD value from the prototype.**
2. **The rigid-tire/static-RC oracle (0.289 deg/g ±10 %, LLTD 0.473 ±0.02) was never run.** The prototype only ran the production vehicle (compliant tire, measured RC map). Nothing here confirms or refutes that oracle. The brief says the tire-compliant roll and LLTD are reported separately and are not acceptance oracles.
3. Only the vehicle `iem24_murphy_real_14dof_simple.xml` was exercised: rear-only drive, parallel steer, zero toe and jounce-steer, no inertance, tire vertical damping 400 N·s/m (irrelevant at zero rates).
4. Tests of the planar cross-check, failure-path exceptions, and the bounds listed in section 5 were **not** implemented in the prototype. They are design intent, not measured behaviour.
5. Timing: about 0.03 s per Newton iteration (28 evaluator calls). Not profiled beyond that.

## 2. Signature and return

```python
def solve_steady_corner(vehicle, speed: float, lateral_acceleration: float) -> VerticalState
```

- `speed` in m/s, `lateral_acceleration` in m/s². Positive is a right turn (SAE +y); it loads the left wheels and must give negative roll.
- Target definition: `ay_path = V * r`, so yaw rate `r = ay / V`. Body-axis `ay` is `V*r*cos(beta)`.
- Returned state: `v_x = V cos(beta)`, `v_y = V sin(beta)`, `yaw_rate = ay/V`, `v_z`, roll rate, pitch rate and jounce rates 0, heave/roll/pitch and jounces from the solution, wheel speeds from the slip ratios.
- `VerticalState` cannot carry steering, torque, or tire states. Add a private `_solve_steady_corner_detail(...)` returning a frozen `SteadyCornerSolution` (state, steering, driven torque, per-wheel torques, four `TransientTireState`, scaled residual, iterations, continuation steps, final `VerticalEvaluation`). The public function wraps it; Task 7 and the LLTD gate use the detail.
- Validation: raise `ValueError` for non-finite inputs or `speed < 3 m/s`. `ay = 0` reduces to the straight solve.

## 3. Unknowns (14) and residuals (14)

`x = [steer, beta, T, heave, roll, pitch, z_fl, z_fr, z_rl, z_rr, k_fl, k_fr, k_rl, k_rr]`

- `steer`: scalar added by `wheel_steering_angles(steer, 4)` (the Ackermann table here is parallel).
- `T`: one torque applied to each driven wheel. Driven wheels come from the transmission (`driven_wheels`, here `('rl','rr')`). Equal torque is assumed; no differential is declared. `front_torque_fraction` is 0 here.
- `k_i`: slip ratios. Wheel speed is the exact inverse of `slip_quantities(..., speed_floor=1.0)`:
  `omega_i = (k_i * max(|vl_i|, 1) + vl_i) / R_i`, with `R_i = loaded_radius(tire, load_i)`. The evaluator uses the loaded radius and the raw wheel speed, not the effective radius.

Residuals are taken from `evaluate_vertical_dynamics(...).rates` at `v_z = roll_rate = pitch_rate = 0`, jounce rates 0:

| Row | `rates` index | Independent scale |
|---|---|---|
| vx_dot, vy_dot | 0, 1 | g |
| yaw acc | 5 | m_total g L / I_yaw |
| heave acc | 2 | g |
| roll acc | 3 | m_s g half_track / I_roll |
| pitch acc | 4 | m_s g (L/2) / I_pitch |
| jounce acc ×4 | 17:21 | (m_total g / 4) / m_unsprung_front |
| wheel acc ×4 | 9:13 | r_1 (m_total g / 4) / J_wheel |

Unknown scales (Newton runs in `y = x / SCALE`): `[0.1 rad, 0.05 rad, 20 N·m, 0.01 m, 0.01 rad, 0.005 rad, 0.01 m ×4, 0.05 ×4]`.

Tire states: per wheel, compute loads (`tire_deflection_load` with zero rate, damping 0), contact velocities, slips, then `steady_rolling_state(P.tire, vehicle.transient_tire, slip_angle, slip_ratio, load)`. The slips used to seed the states are the same ones the evaluator re-derives, so the states are consistent by construction. After the solve, check `max|tire_state_rates| < 1e-8`.

## 4. Jacobian and Newton

- Central differences, 14×14, 28 evaluations per iteration.
- Physical step sizes: steer 1e-6 rad, beta 1e-6 rad, T 1e-4 N·m, heave 1e-6 m, roll 1e-7 rad, pitch 1e-7 rad, jounce 1e-6 m, kappa 1e-6.
- `np.linalg.solve`; if singular or `cond > 1e12`, `lstsq`; otherwise raise. No SciPy (library rule; `planar_forward.py` uses SciPy only in the forward extra).
- Backtracking line search on the infinity norm of the scaled residual: accept when `r_new < r_old (1 - 1e-4 λ)`, λ from 1 to 1/1024. A trial that raises (`DomainError`), is non-finite, or violates the bounds in section 5 counts as rejected.
- Roll-centre height is queried at `|roll|` (clipped at 5°), so it has a kink at roll = 0. Solve the straight case first; there `Fy ≈ 0` and the kink is harmless.

## 5. Initial guess, continuation, bounds, failure

1. **Straight solve (ay = 0):** seed heave, roll, pitch, jounces from `solve_static_equilibrium(vehicle, speed)`, and `T = steer = beta = k = 0`. Measured: 3–4 Newton iterations to about 1e-11 at 8, 15 and 25 m/s; `T` = 5.9, 20.6, 56.3 N·m per driven wheel, consistent with drag.
2. **Continuation in ay:** steps of 0.1 g from 0 to the signed target, each warm-started from the previous solution. On a failed step, retry from a kinematic seed (`steer = L*ay/V²`). If that fails, halve the step down to 0.005 g, then raise `RuntimeError("steady corner failed to converge at ay=... (residual ...)")`. Never return a partial result. (Step halving and the 0.005 g floor are design intent and were **not** exercised.)
3. **Bounds applied to trial points:** `|z| ≤ 0.0318 m` with no out-of-range counter, every load > 2 N (no lifted wheel), `|steer| ≤ steering_maximum_angle`, `|T| ≤ peak motor torque (about 108.7 N·m at the wheel)`, `|beta| ≤ 0.5 rad`, `|k| ≤ 0.5`. (Also design intent; the prototype did not enforce them.)
4. **Convergence:** scaled residual infinity norm < 1e-10. Then re-evaluate and require: unscaled non-wheel rate rows < 1e-6, `out_of_range_count == 0`, `lifted.any()` false, tire state rates < 1e-8, body `ay` equal to `ay_target*cos(beta)` within 1e-6.

## 6. Measured behaviour (prototype, pre-mass/roll edit)

Straight to 1 g, 10 continuation steps, all converged (residual below 1e-9):

| V (m/s) | steer at +1 g | beta | T (N·m) | Newton iterations |
|---|---|---|---|---|
| 8 | 14.06° (kinematic 13.48°) | +4.4° | 17.4 | 40 |
| 12 | 6.06° | +1.06° | 21.7 | 40 |
| 15 | 3.86° | +0.18° | 28.3 | 39 |
| 25 | 1.38° | −0.58° | 62.1 | 30 |

- −1 g mirrors +1 g exactly (steer and beta change sign, loads swap sides) at 8, 12, 15 and 25 m/s.
- 15 m/s, 1 g: jounces 0.8–2.4 mm, `Fy/Fz` 0.83–0.92 (tire peak about 1.59), no lift, no out-of-range.
- 8 m/s, 1 g is a stress case: parallel steer makes the fronts very unequal (FL `Fy/Fz` 1.41, FR 0.72). Use 12–25 m/s for the LLTD gates.
- Direct jump (no continuation) from the straight solution converged at 1 g in 4–6 iterations in most tested cases but **failed at −1 g, 15 m/s (residual 4e-2)**. Continuation is therefore required.

### High-g limits (tested, not assumed)

Continuation in 0.1 g steps, cutoff = first step that fails to converge:

| V (m/s) | 1.5 g | 1.7 g | 2.0 g | 3.0 g |
|---|---|---|---|---|
| 12 | OK | **fail at the 1.7 g step** (last good 1.6 g, residual 2.9e-3) | fail | fail |
| 15 | OK | OK | **fail at the 1.8 g step** (last good 1.7 g, residual 1.2e-2) | fail |
| 25 | OK | OK | OK (`Fy/Fz` 1.36) | **fail at the 2.2 g step** (last good 2.1 g, residual 2.2e-2) |

- 3 g fails at every tested speed (12, 15, 25 m/s). A direct 3 g jump at 15 m/s also failed (residual 1.08).
- The limit rises with speed because downforce adds tire load (about 1.6 g at 12 m/s, 1.7 g at 15 m/s, 2.1 g at 25 m/s). Treat these as the tire-limited feasible region, not as solver bugs.
- A 3 g target must raise `RuntimeError`, not return a partial state. The cutoff values above are prototype values; they shift with the evaluator edit.

## 7. Test plan (`tests/test_vertical_equilibrium.py`)

1. Straight solve: converges; kappa ≈ 0; `T` balances drag and rolling resistance; heave, roll, pitch and jounces match `solve_static_equilibrium` to 1e-6.
2. ±1 g at 12, 15 and 25 m/s: residual < 1e-6; mirror solutions agree to about 1e-9 with sides swapped.
3. Sign (needs the mass/roll fix): left wheels loaded more, roll < 0, front LLTD in (0.3, 0.7).
4. Rigid-tire/static-RC reference vehicle (stiff tire, constant static RC heights, same ARB): roll gradient 0.289 deg/g ±10 %, front LLTD 0.473 ±0.02. **Not yet run.** Report the compliant-tire roll and LLTD separately; they are not oracles.
5. Continuation and direct jump agree to 1e-8 where the direct jump converges.
6. Failure paths: `speed < 3` raises `ValueError`; a 3 g target raises `RuntimeError`; an unreachable steering angle raises `RuntimeError`.
7. Tire state rates are zero at the solution; `out_of_range_count == 0`.
8. Optional low-ay comparison with `evaluate_planar_dynamics` steer: informational only, not a gate.

## 8. Other blockers

- The spring curve is bump-stop stiff near ±29.8 mm; jounces here stay below 6 mm at 1 g.
- The evaluator uses `speed_floor = 1.0`; keep `speed ≥ 3 m/s` so the floor never applies at the contact points.
- Rolling-resistance moment printed as zero in the straight check at 15 m/s on this tire (`f_rr` coefficients), so `T` balances drag only; do not assert a nonzero rolling-resistance term.
