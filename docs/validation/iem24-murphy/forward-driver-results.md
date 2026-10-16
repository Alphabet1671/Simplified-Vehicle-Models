# Forward-driver prototype — trial record and results

> **Superseded 2026-10-16:** generated with track 1.016 m and 55 % front mass. Values do not apply to the current vehicle.

> **Tools layout migration note.** `tools/` was reorganised into category subpackages (`data`, `tracks`, `model_data`, `run`, `gates`, `reports`, `diagnostics`). Commands and paths in this document use the current locations; any command recorded inside an `attempt_*` directory (for example `command.txt`, `parent.json`) was run with the pre-migration flat paths such as `tools/run_iem24_forward_driver.py` and is preserved verbatim. See [docs/dev/tools-layout.md](../../dev/tools-layout.md) for the old-to-new mapping. Recorded hashes, attempt directories and frozen artifacts are unchanged.

## Executive outcome

A complete transient 7DoF lap was simulated in **230.7 s of wall time, inside the 300 s cap**, where the full-lap OCP path could not converge at any mesh ≤5 m. The lap preserves the four wheel-speed states with the user-requested wheel inertia (`0.2064512 kg*m^2`, 10 kg solid cylinder, 16 in diameter). Physics checks passed at every accepted state. The result is a **conservative driven lap, not a minimum-time optimum and not yet a periodic hot lap**; three acceptance gates remain open and are flagged `usable: false` in the machine-readable summary.

| Quantity | Value | Gate | Status |
|---|---:|---|---|
| Full-lap wall time (BDF, incl. validation) | 230.73 s | ≤300 s | pass |
| Lap time (conservative plan, 8 m/s cap) | 167.99 s | informational | — |
| Max lateral path error | 0.184 m | ≤0.25 m | pass |
| Max accepted spatial step | 0.290 m | ≤0.5 m | pass |
| Max load-closure residual | 1.0e-9 | <1e-6 | pass |
| Max electrical power residual | 7.8e-11 W | <1e-6 | pass |
| Min body/wheel speed | 3.32 m/s / 15.5 rad/s | forward only | pass |
| Full-state periodicity at the seam | not met | see below | **open** |
| Between-step constraint checking | not done | see below | **open** |
| Tolerance/step refinement study | not done | see below | **open** |

Reference run: the BDF 230.7 s trial is retained under `forward_driver_trials/attempt_1792004259402435000/` (summary, trace, run log); it completed but remains `usable: false`. A later live-planner BDF trial, `forward_driver_trials/attempt_1792014409550714000/`, also completed but remains unusable. Neither attempt establishes periodicity, between-step validation, or refinement. Other exploratory attempts are not included in this concise record.

## What was built

- `src/simplified_vehicle_models/solvers/forward_driver.py` — model-independent driver stack: `RateSystem` (nonsingular mass-matrix contract, singular systems rejected), `SpeedPlan`/`build_speed_plan` (closed-track forward-acceleration / backward-braking envelope passes), `PreviewDriver` (course-error steering feedback including body sideslip, speed feedforward + feedback, exclusive throttle/brake), `solve_forward` (stiff integration with deadline, trial-domain rejection, scaled finite-difference Jacobian, accepted-state re-validation, Radau/BDF).
- `src/simplified_vehicle_models/solvers/planar_forward.py` — transient planar adapter: 12 states (`s, n, heading error, v_x, v_y, yaw rate, 4 wheel speeds, SOC, steering`), original NumPy tire/load physics, pedal-controller motor mapping, battery voltage/current closure, hydraulic brake mapping, physical rejections (load non-convergence, nonpositive contact, reverse rolling, SOC bounds, steering bound, path-denominator fold).
- `tools/run/run_iem24_forward_driver.py` — parent-capped runner: ≤300 s enforced, per-attempt output isolation, flushed progress logs, source hashes + environment snapshot, full gate reporting with `usable` locked to `false` until all gates close.
- Tests: `tests/test_forward_driver.py`, `tests/test_planar_forward.py`, `tests/test_forward_driver_runner.py` (28 new tests; 109 fast-module tests pass).

## Trial log (chronological, all preserved)

| Attempt (last 6) | Method | Duration | Outcome |
|---|---|---:|---|
| 806000 | Radau | 2 s | Rejected at s=0: zero-sideslip corner seed made the rear-left contact load nonpositive |
| 590000 | Radau | 2 s | Path error 0.313 m > 0.25 gate: driver regulated heading instead of course |
| 654000 | Radau | 2 s | After course fix: 0.0296 m error — 10x improvement |
| 350000 | Radau | 300 s | Rejected: integrator Newton trial stepped SOC outside [0,1] |
| 997000 | Radau | 6 s | Rejected: default FD Jacobian hit machine-scale noise against the 1e-9 closure tolerance |
| 921000 | Radau | 6 s | Completed with scaled FD Jacobian — baseline for cross-method check |
| 734000 | Radau | 300 s | Timed out at 295.3 s, 927.8/1016.7 m — no physics relaxed, simply over budget |
| 005000 | BDF | 6 s | Cross-check vs Radau: endpoints agree ≤1.4e-5; lateral error identical to 8 digits |
| 435000 | BDF | 300 s | **Completed full lap in 230.7 s; gates as in the table above** |

Three defects were found and fixed by their own failing trial, without relaxing any physical tolerance: the corner seed needs a nonzero body sideslip (`v_y = front_mass_fraction * wheelbase * yaw_rate`); integrator trial states outside the plant domain must be rejected as trials, not aborts; and the Jacobian perturbation must be physically scaled, not machine-scaled.

## Cross-method integration evidence

The 6-second Radau (`921000`) and BDF (`005000`) runs, same physics and tolerances, agree at the final state to:

- position/path states ≤ 4.7e-6; body states ≤ 3.0e-6; wheel speeds ≤ 1.4e-5 rad/s; SOC ≤ 2.3e-10;
- peak lateral error 0.07159722 (Radau) vs 0.07159720 (BDF).

This is method-independence evidence at `rtol=1e-6, max_step=0.05 s`, not a full refinement study.

## Open acceptance gates

1. **Periodicity.** The lap is a flying start from a kinematic seed, so seam state differs from start state: wheel speeds −1.30…−1.40 rad/s, `v_x` −0.267 m/s, heading error 0.036 rad, lateral error −0.068 m. A second lap warm-started from the final state would approach the closed orbit, but a second lap costs another ~230 s, exceeding the 300 s cap for a single run. Closing this gate needs either a warm-lap protocol across runs or a settled/equilibrium initial state.
2. **Between-step checking.** Every RHS evaluation and every accepted state passed the physical checks, but constraints were not verified on the dense interpolant between accepted nodes.
3. **Refinement.** Single tolerance setting plus the Radau/BDF cross-check; no `rtol`/`max_step` refinement sweep yet.
4. **Planner conservatism.** The 8 m/s / 2 m/s² constant-acceleration envelope is a request, not the master-plan live-limit solver. Lap time 167.99 s is therefore a feasibility result only and must not be compared against OCP or measured lap times until the real capability planner is wired in.

## Scope and honesty notes

- The 10DoF and 14DoF plants are not implemented in this repository; the shared interface was exercised against synthetic nonsingular mass-matrix systems (32 states) only. That is a contract test, not physical validation of those models.
- The wheel-inertia question was delegated through a model-specific workflow requesting provider `openai-codex`, model `gpt-6-luna` (user-approved). Its findings are applied: XML corrected to the requested 10 kg solid-cylinder value, and the OCP's quasi-steady wheel reduction is documented as a separate model, not a defect.
- Historical OCP artifacts and `measured_stint.csv` are untouched; archived runs retain the old 12 kg inertia and must not be described as same-parameter comparisons.
- The OCP path itself is unchanged (`planar_ocp.py` untouched by this work); the forward driver is an additive, opt-in workflow.
