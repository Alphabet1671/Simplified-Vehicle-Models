# Computational Cost by Model

This page shows where the run time goes in one lap of each model. It does not change any model. Use it to choose what to speed up first.

## Method

- Tool: [`tools/diagnostics/profile_models.py`](../../tools/diagnostics/profile_models.py). It calls the library directly, so it writes no attempt directory and checks no gates.
- Settings: the same settings as the four-way comparison attempts in `docs/validation/iem24-murphy/four_way_latest_model/runs/`:
  - Ten-lap MIS track `models/tracks/2024_michigan_endurance_multilap_closed.xml`, 0.5 m spacing.
  - Point mass: the `solve_lap_with_energy` call of `tools/run/run_iem24_point_mass_track_pair.py`.
  - 7DoF and 14DoF: live planner, 60 m/s cap, 0.175 s pedal travel, BDF, rtol 1e-5, atol 1e-7. Maximum step is 0.02 s for 7DoF and 0.005 s for 14DoF.
- Machine: Apple M2, Python 3.13.13, NumPy 2.4.4, SciPy 1.17.1, BLAS threads set to 1. The three models ran at the same time on separate cores.
- Two runs per model:
  - **Unprofiled** run: gives the wall times.
  - **cProfile** run: gives the shares. cProfile makes this call-heavy code 1.5–2× slower. It slows small functions more than large ones, so read the shares as approximate. The "≈ s" columns apply each profiled share to the unprofiled wall time.
- The results matched the recorded attempts: point mass 59.620 s, 7DoF 64.993 s, 14DoF 65.063 s lap time.

## Summary

| Model | Unprofiled wall time | Main cost | Second cost |
|---|---|---|---|
| Point mass | 95.8 s | Corner-speed bisection, 72 % | Drive-limit bisection inside it |
| 7DoF | 9.99 s plan + 93.8 s solve | Inner load loop with tire calls, about 64 % of plant time | Accepted-state replay, 38 % of the solve |
| Simplified 14DoF | 10.0 s plan + 0.24 s init + 131.8 s solve | Tires and suspension tables, about 50 % of plant time | Accepted-state replay, 33 % of the solve |

Three results hold across the models:

1. **The cost is many small scalar calls, not heavy arithmetic.** Most time goes to Python call overhead and NumPy setup on arrays with 1–4 elements: `np.interp` on one value, `asarray`, `zeros_like`, `clip`, and `min`/`max`/`any` reductions. In the point mass, scalar `np.interp` alone takes about 24 % of the time, with 20.5 million calls.
2. **Nested bisections and root solves multiply the call count.** The point mass runs a bisection inside a bisection. Both forward models solve for bus voltage with `brentq` on every plant evaluation.
3. **The post-integration replay costs about one third of every forward solve.** `solve_forward` evaluates the plant again at every accepted state and at every interval midpoint (about 2 × accepted steps). This is acceptance evidence, not integration.

## Point mass

Unprofiled: 95.8 s for `solve_lap_with_energy`. Setup takes less than 0.1 s.

### Phase breakdown

| Phase | Share | ≈ s | Calls |
|---|---:|---:|---:|
| `maximum_corner_speed` (corner speed at each node) | 72.5 % | 69 | 8,080 |
| `_forward_sweep` (acceleration limits) | 21.0 % | 20 | 12 |
| `_backward_sweep` (braking limits) | 6.0 % | 5.7 | 12 |
| `_build_result` and `_energy_trace` | 0.4 % | 0.4 | 4 |

### Why the corner-speed solve dominates

- `solve_lap` runs 4 times: one initial lap and 3 energy iterations. Each run solves the corner speed again at all 2,020 nodes, so there are 8,080 corner solves.
- [`maximum_corner_speed`](../../src/simplified_vehicle_models/formulations/point_mass.py) bisects speed to a 1e-8 m/s tolerance. That needs about 33 `evaluate_equilibrium` calls per node, 267,310 calls in total.
- Each equilibrium needs the available drive force. [`maximum_drive_force`](../../src/simplified_vehicle_models/powertrain/assembly.py) runs a second bisection over motor torque. It averages 14.5 `operating_point` calls per call and makes 3.5 million calls in total. The drive cache serves 83 % of the requests, but the misses still take 115 s profiled.
- Each `operating_point` call runs `solve_current`, `terminal_voltage`, and `motor_torque_limit`. These do scalar table lookups through `energy_storage._interp` (17.6 million calls) and `voltage_scaled_speed` (7.6 million calls).

### Self time by module (profiled)

| Module | Share |
|---|---:|
| Builtins (mostly scalar `np.interp`, `asarray`, reductions) | 25.5 % |
| `formulations/point_mass.py` | 16.9 % |
| `powertrain/motor_envelope.py` | 15.4 % |
| `powertrain/energy_storage.py` | 11.4 % |
| NumPy `interp` Python wrapper | 8.1 % |
| `powertrain/assembly.py` | 6.7 % |
| NumPy `min`/`max` wrappers | 6.3 % |
| NumPy `full`, `iscomplexobj` | 6.1 % |

The energy trace itself costs 0.1 %. The cost of the energy loop comes from solving the lap again, not from integrating the energy.

## 7DoF (planar)

Unprofiled: 9.99 s for the live speed plan and 93.8 s for `solve_forward`.
Counts: 11,559 accepted steps, 36,401 integrator RHS calls, 259 Jacobians, 59,521 plant evaluations, 0 rejected domain trials. One plant evaluation costs 1.58 ms.

### Phase breakdown

| Phase | Share of solve | ≈ s | Plant evaluations |
|---|---:|---:|---:|
| Integrator RHS calls (`solve_ivp` → `rhs`) | 53.6 % | 50 | 32,775 |
| Finite-difference Jacobian | 5.8 % | 5.4 | 3,626 |
| Accepted-state and midpoint replay | 37.6 % | 35 | 23,120 |
| SciPy BDF internal work | 2.0 % | 1.9 | — |
| Dense-output bounds | 0.5 % | 0.5 | — |

The live speed plan is a point-mass `solve_lap` without the energy loop. It costs 10 s, about 10 % of the run, and the corner-speed bisection described above dominates it.

### Cost inside one plant evaluation

| Part | Share of plant time |
|---|---:|
| Inner load loop: tire calls (`evaluate_unitire`) | 45.1 % |
| Inner load loop: other work (normal loads, loaded radius, force rotation, slip) | 18.7 % |
| Rest of `evaluate_planar_dynamics` (Newton/Broyden algebra, `det`, `solve`) | 14.9 % |
| Powertrain closure (`brentq` bus-voltage solve) | 17.9 % |
| Driver command (`PreviewDriver.command`) | 4.1 % |
| Plant checks, path terms, diagnostics | 3.4 % |

- The load loop calls `_load_step` 10.6 times per plant evaluation on average (630,723 calls). Each call evaluates all four tires once.
- In [`unitire_simple.py`](../../src/simplified_vehicle_models/tires/unitire_simple.py), `_polyval` (3.8 million calls) and `loaded_radius` (1.3 million calls) are the largest self-time items after `evaluate_unitire`.
- The Jacobian is cheap here: 12 states and only 259 Jacobian builds.

### Self time by module (profiled)

| Module | Share |
|---|---:|
| `tires/unitire_simple.py` | 33.3 % |
| Builtins | 14.9 % |
| `formulations/planar_7dof.py` | 11.0 % |
| NumPy reduction wrappers | 6.0 % |
| `formulations/point_mass.py` (planner) | 5.1 % |
| NumPy `zeros_like` and related | 3.8 % |
| `solvers/forward_powertrain.py` | 3.2 % |
| NumPy `linalg` (2×2 `det`/`solve`) | 3.2 % |

## Simplified 14DoF (vertical)

Unprofiled: 10.0 s for the live speed plan, 0.24 s for the equilibrium initial state, and 131.8 s for `solve_forward`.
Counts: 20,131 accepted steps, 73,391 integrator RHS calls, 462 Jacobians, 113,655 plant evaluations, 0 rejected domain trials. One plant evaluation costs 1.16 ms. This is less than one 7DoF evaluation because there is no inner load loop. The run takes longer because the maximum step is 4× smaller and the state has 38 entries.

### Phase breakdown

| Phase | Share of solve | ≈ s | Plant evaluations |
|---|---:|---:|---:|
| Integrator RHS calls | 47.9 % | 63 | 54,911 |
| Finite-difference Jacobian | 16.0 % | 21 | 18,480 |
| Accepted-state and midpoint replay | 32.8 % | 43 | 40,264 |
| SciPy BDF internal work | 1.9 % | 2.5 | — |
| Dense-output bounds | 1.0 % | 1.3 | — |

The Jacobian takes three times the share it takes in the 7DoF. Each build makes 40 plant evaluations: one baseline and one per state.

### Cost inside one plant evaluation

| Part | Share of plant time |
|---|---:|
| Transient tires (`evaluate_transient_unitire`, 4 per evaluation, each calls `evaluate_unitire` twice) | 25.4 % |
| Suspension force network (`independent_corner.forces`) | 24.2 % |
| Powertrain closure (`brentq` bus-voltage solve) | 21.1 % |
| Rest of `evaluate_vertical_dynamics` (mass block, kinematics, sums, `astuple`) | 19.1 % |
| Jounce-steer toe lookup (`kinematics.toe`) | 4.2 % |
| Driver command | 4.8 % |
| Plant checks, state slicing, diagnostics | 4.7 % |

- Suspension: damper tables (`damper_table.damper_force` → `_half_force`) take 32 of the 47 profiled seconds. The monotone-spline lookups (`monotone_spline.__call__` and `_locate`, 1.5 million and 1.9 million calls) are the largest part.
- The tire model runs per corner with scalar inputs: 911,864 `evaluate_unitire` calls, against 630,723 vectorised four-wheel calls in the 7DoF.
- NumPy `clip` runs 4.0 million times and `zeros_like` 6.7 million times, mostly from the tire and spline code.

### Self time by module (profiled)

| Module | Share |
|---|---:|
| Builtins | 17.0 % |
| `tires/unitire_simple.py` | 13.5 % |
| `suspension/monotone_spline.py` | 10.0 % |
| NumPy reduction wrappers | 8.3 % |
| `formulations/vertical_14dof.py` | 4.5 % |
| NumPy `zeros_like` and related | 4.4 % |
| `solvers/forward_powertrain.py` | 4.0 % |
| `suspension/damper_table.py` | 3.4 % |
| `formulations/point_mass.py` (planner) | 3.4 % |
| `powertrain/motor_envelope.py` | 3.2 % |
| NumPy `clip` | 3.1 % |

## Where to look first

This list ranks candidates by measured cost. Each item needs its own check that results stay the same before it is accepted.

| Rank | Model | Target | Measured cost |
|---|---|---|---|
| 1 | Point mass, and the live planner of both forward models | Corner-speed bisection (≈33 equilibria per node, solved again in each energy iteration) and the drive-limit bisection inside it | 72 % of the point mass; most of the 10 s plan |
| 2 | 7DoF, 14DoF | Accepted-state and midpoint replay: decide whether every check must call the full plant, or whether one stored evaluation per accepted state is enough | 33–38 % of each solve |
| 3 | 7DoF | Calls of the inner load loop per evaluation (10.6) multiplied by the cost of the four-tire call | ≈64 % of plant time |
| 4 | 7DoF, 14DoF | `brentq` bus-voltage closure | 18–21 % of plant time |
| 5 | 14DoF | Monotone-spline damper and jounce-steer lookups; per-corner scalar tire calls | ≈30 % of plant time |
| 6 | 14DoF | Finite-difference Jacobian (40 evaluations per build) | 16 % of the solve |
| 7 | All | Scalar NumPy overhead (`np.interp` on one value, `asarray`, `zeros_like`, `clip`, reductions on 4-element arrays) | Spread across all of the above |

## Limits

- One lap per model on one machine. The models ran at the same time, so absolute times can change by some percent on an idle machine.
- cProfile changes the relative cost of small and large functions. The shares are good for ranking, not for exact budgets.
- The tool skips the attempt runner's source hashing, CSV and JSON output, and acceptance gates. The 10.4 s `setup_seconds` in the recorded 7DoF and 14DoF summaries is mostly the live speed plan measured here.
