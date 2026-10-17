# Task 7 report — vertical forward plant

## Status

Implemented and verified with focused tests only (the parent runs the full suite and
commits). Nothing committed. Files:

| File | Change |
|---|---|
| `src/simplified_vehicle_models/solvers/forward_powertrain.py` | NEW — `evaluate_forward_powertrain`, `ForwardPowertrain` (the powertrain closure moved unchanged out of `planar_forward.py`) |
| `src/simplified_vehicle_models/solvers/planar_forward.py` | `PlanarForwardPlant.powertrain` is now a thin call into the shared closure; `ForwardPowertrain` is re-exported from here |
| `src/simplified_vehicle_models/solvers/vertical_forward.py` | NEW — `VerticalForwardPlant` (38 states), `body_accelerations` |
| `tests/test_vertical_forward.py` | NEW — 26 tests |
| `tests/fixtures/planar_forward_powertrain_states.npz` | NEW — 56 recorded pre-move planar states with their rates and powertrain outputs: the original 50 plus 6 speed-regulation-limited rows (sha256 `aa03d84f4e444cb12e008abd008d0ed2a21e34e3d388e124635e3c254800556c`) |
| `tests/test_powertrain_closure_audit.py` | bracket spy retargeted to `forward_powertrain.brentq` (the function it audits moved); docstring updated. No assertion or value changed |

`planar_7dof.py`, `vertical_14dof.py`, `vertical_equilibrium.py` and every other
planar file were **not** touched.

## What was built

### 1. Shared powertrain closure (`forward_powertrain.py`)

`evaluate_forward_powertrain(vehicle, driven, gearing, gear_efficiency, wheel_speed,
soc, command)` is the pre-move body of `PlanarForwardPlant.powertrain` with only the
attribute lookups replaced by the arguments (`self.vehicle` → `vehicle`,
`self.driven` → `driven`, `self.gearing` → `gearing`, `self.gear_efficiency` →
`gear_efficiency`, `state[10]` → `soc`, `state[6:10]` → `wheel_speed`). Statement
order, expressions, and constants are otherwise byte-for-byte the same, so the
arithmetic order is preserved. `PlanarForwardPlant.powertrain` now returns
`evaluate_forward_powertrain(self.vehicle, self.driven, self.gearing,
self.gear_efficiency, state[6:10], state[10], command)`.

### 2. Vertical 14-DoF forward plant (`vertical_forward.py`)

* `state_size = 38`; `state_names = (s, n, heading_error) + 21 VerticalState names +
  (soc, steering) + (kappa_eff, lateral_deformation, trail_angle) × FL, FR, RL, RR`.
  The tire names are `kappa_eff_fl`, `lateral_deformation_fl`, `trail_angle_fl`, … .
* `periodic_indices` excludes `s` (resets each lap) and `soc` (consumed), exactly as
  `PlanarForwardPlant` does, giving 36 entries covering all 12 tire states.
  `periodic_tolerances` has one entry per index, built directly from the index tuple.
* `initial_state(speed)`: `solve_static_equilibrium(vehicle, speed)` seeds heave,
  roll, pitch and the four jounces; the plan's curvature sets `yaw_rate = speed·κ`,
  `v_y = f_front·L·yaw_rate`, `heading_error = −atan2(v_y, v_x)` and
  `steering = atan(L·κ)` (the planar plant's seed). The seven vertical coordinates
  are then refined (`_refine_vertical`, a private NumPy Newton on the evaluator's
  `generalized_forces` with a scaled central-difference Jacobian, least-squares
  fallback and backtracking) and the four tire states are set with
  `steady_rolling_state(...)` at the converged loads, iterating the
  loads ↔ loaded radii ↔ wheel speeds ↔ tire states ↔ vertical refinement loop to a
  `1e-12` fixed point. No artificial vertical or tire transient.
* `evaluate(time, state, command)`: domain guards, then the shared powertrain, then
  `evaluate_vertical_dynamics`. The 38-vector is assembled from
  `RateSystem(out.forcing, out.mass_matrix)`: with zero inertance `mass_matrix is
  None` and `forcing` already holds the solved rates; with positive inertance the
  21×21 physical block is embedded in a 38×38 identity with
  `np.ix_(MECHANICAL_INDEX, MECHANICAL_INDEX)`. `out.rates` is never re-solved. The
  path rows (`s`, `n`, `heading_error`), `soc`, `steering` and the 12 tire rates fill
  the remaining rows.
* `observe(state)`: planar-style `DriverObservation`; `wheel_speed_margin` is
  `min(kappa_eff) + 1` (the transient tire's slip ratio against the `-1` full-lock
  floor). `powertrain(state, command)` delegates to the shared closure.
* Diagnostics with the `PlanarForwardPlant` names (`maximum_load_residual`,
  `minimum_contact_load`, `maximum_power_residual`, `maximum_load_iterations`,
  `load_method_counts` (`{"direct": …}`, since the vertical load path has no inner
  loop), `minimum_load_jacobian_determinant`, `lifted_wheel_evaluations`,
  `speed_regulated_evaluations`, `maximum_motor_speed_ratio`, `evaluations`,
  `last_time`, `last_distance`) plus `out_of_range_evaluations`, `maximum_jounce`,
  `minimum_jounce`, `maximum_damper_velocity` (largest `|damper_velocity|`).
* Trial domain: a jounce beyond the compiled travel limit ± 10 mm raises
  `TrialStateError`; a `DomainError` from a spring/bump-stop/jounce-steer table is
  converted to `TrialStateError` so the integrator retries instead of failing hard.
  Vertical tire load is nonnegative by construction.
* Road-plane `ax, ay`: `observe` refreshes `self.body_accelerations` from the last
  evaluation's `VerticalEvaluation.body_accelerations`; a module-level
  `body_accelerations(state, rates)` helper mirrors `planar_forward`'s for callers
  that only have a recorded trace.

## Commands and counts

| Command | Result |
|---|---|
| `PYTHONPATH=src python3 -m unittest tests.test_vertical_forward -q` | **26 tests OK in 31–32 s** |
| `PYTHONPATH=src python3 -m unittest tests.test_vertical_forward tests.test_planar_forward tests.test_powertrain_closure_audit -q` | **38 tests OK in 32.3 s** (review fix round) |
| `PYTHONPATH=src python3 -m unittest tests.test_vertical_forward tests.test_planar_forward tests.test_powertrain_closure_audit tests.test_vertical_14dof tests.test_vertical_equilibrium -q` | **94 tests OK in 65.9 s** (pre-fix-round; the five-module set was not rerun after the review fixes) |
| `PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver tests.test_forward_driver_plot tests.test_forward_driver_runner tests.test_wheelspin_lift_and_pedals tests.test_planar_dense_certificate tests.test_iem24_seven_dof_track_pair -q` | **64 tests OK in 1.5 s** |
| `git diff --check` | clean |
| Full suite | intentionally not run (parent's job; ~4.5 min) |

Every focused test's out-of-range counters are 0 at the gates (`out_of_range_evaluations
== 0`, no lifted wheels, minimum contact load > 0). No golden value moved.

### Review fix round (independent Task 7 review: approved with fixes)

All six findings fixed in `vertical_forward.py`, `forward_powertrain.py` (a comment),
`tests/test_vertical_forward.py`, the fixture and this report:

1. The quarter-car reference is now the overshoot fraction `+0.171` (the previous form
   subtracted one, giving −0.829 and vacuously passing asserts); the test pins it with
   `assertAlmostEqual(quarter_car, 0.171)`.
2. The path/steering row assertions now use a non-straight state (`n = 0.35`, heading
   0.04 rad, `v_y = 0.9`, yaw −0.06), a curved plan (κ = −0.02) and a nonzero steering-rate
   command, with each expected value proven nonzero before it is asserted.
3. The fixture gained 6 speed-regulation-limited rows recorded from git HEAD's pre-move
   `planar_forward.py` (the module also proves it reproduces the original 50 rows bitwise);
   new sha256 `aa03d84f…556c`.
4. The `DomainError → TrialStateError` conversion now increments
   `out_of_range_evaluations`, and the trial-domain test checks both the damper-flag path
   (no raise, counter 1) and the spline-exit path (raise, counter 2).
5. The diagnostics test builds a fresh plant, so its counters no longer depend on
   alphabetical test order.
6. This report records the four review disclosures below (observations without
   accelerations, lagged `wheel_speed_margin`, Task 8 hash list, unmeasured memory).

## Measured numbers

### Initial state (zero throttle/brake command, so the wheel torques are exactly zero)

* 15 m/s: the 14 vertical rate rows are ≤ 2.6e-15 (gate 1e-6); the 12 tire-state rates
  are 0 to machine precision. Heave 12.18 mm, pitch 1.5 µm, front jounce 1.473 mm,
  rear jounce 2.025 mm (the aero downforce compression at speed, not a start transient).
  The straight-line steady tire states are all `+0.0`/`−0.0`.
* 30 m/s and a 15 m/s curved plan (κ = 0.02) both satisfy the same gate.
* The longitudinal rate is *not* zero (`a_x = −0.74 m/s²` at 15 m/s): drag decelerates
  the body. `solve_static_equilibrium` has no longitudinal force, so the vertical rows
  are what the refinement zeroes — this is the ledger's Task 7 ruling in action.

### Performance (`VerticalForwardPerformanceReportTests` prints this line)

```
PERFORMANCE REPORT: median evaluate 0.896 ms, p95 1.212 ms; 100 m BDF trial:
2968 solver rate-path calls, 6 Jacobian evaluations, 2734 rhs calls, 1295 accepted
steps, 5560 total plant evaluate calls including 2592 replay calls; projected full lap
1016.7 m -> 56529 calls, 51 s at the median rate
```

* 1 000 timed `evaluate` calls after a warm-up call: median 0.88–0.92 ms, p95 1.14–1.21 ms
  across runs (7DoF reference: about 1.12 ms per call, F7).
* 100 m straight, constant 0.10 throttle, BDF, `maximum_step = 0.005`, `rtol = 1e-6`:
  completes in 5.4 s wall, 1 295 accepted steps, 0 rejected domain trials, 0
  out-of-range evaluations, 0 lifted wheels, min contact load 764.5 N.
* Only 6 Jacobian evaluations for 1 295 steps (BDF reuses the Jacobian): the full
  finite-difference Jacobian is not the bottleneck at this step limit. Each Jacobian
  costs 39 plant calls, so 2968 solver-path calls + 2592 replay calls = 5560 total.
* Projected full lap (1 016.7 m legacy closed track): 56 529 `evaluate` calls → about
  51 s at the median rate, including replay. This is a projection from a 100 m
  straight, not a lap measurement: tire-state and damper work is in the same band, but
  a lap adds steering/load-transfer variation. The 14DoF rung has no 300 s limit (D12).

### Straight 100 m trial

Jounce stays in [+1.65, +3.70] mm (gate ±31.8 mm); the BDF dense-polynomial bounds over
every accepted interval are [+1.647, +3.704] mm, so the interpolant is inside the travel
too. Tire states stay ≤ 1.61e-3 and drift ≤ 1.61e-3 from their initial values;
`out_of_range_evaluations == 0`; SOC 0.8291 → 0.8286.

### Step test — 2 000 N downward step on the front axle

Measurement (nonlinear plant, fully coupled 38-state integration, zero command,
inertance 0.5 kg on both axles so the plant exposes its mass matrix):

| Quantity | Nonlinear plant | 7-coordinate linear oracle | Difference |
|---|---|---|---|
| first-peak front body-point drop | 32.03 mm at 0.0793 s | 32.50 mm at 0.0797 s | −1.46 % (gate ±15 %) |
| first-peak time | 0.0793 s | 0.0797 s | −0.50 % |
| steady drop | 24.97 mm | 25.11 mm | −0.54 % |
| first overshoot | 28.26 % | 29.47 % | −4.10 % relative (gate ±15 %) |

### Tire-state decay

At 20 m/s the front corners are at 844 N (σ_y = 0.1833 m, τ = 9.2 ms) and the rear at
990 N (σ_y = 0.2074 m, τ = 10.4 ms). A +2 mm lateral-deformation perturbation gives an
initial rate of exactly −|Vx|·δ/σ_y (relative 1e-9), and after 4τ the residual is
within 0.1 % of δ·exp(−4); the measured per-corner decay rate matches 1/τ within 2 %.

### Refactor gate

* Fixture: 50 deterministic in-domain planar states (3 hand-built seeds plus RNG seed
  20261016 perturbations), each with its driver command, 12-state rate, wheel torques,
  motor torques, bus power, voltage, current, power residual and `limited` flag, plus 6
  appended rows in the inverter speed-regulation band (wheel speeds 0.90–1.02 × the
  voltage-scaled motor speed limit at 25–47 m/s and 0.35–0.90 SoC) so that `limited=True`
  is pinned. The 6 appended rows cannot reach the battery power cap on this motor/pack:
  the torque-envelope taper binds first in a 60-combination sweep of SoC × speed.
* Post-move `PlanarForwardPlant.evaluate(...).solve()` and `powertrain(...)` reproduce
  every recorded value with `np.array_equal` (bitwise) for all 56 states.
* Provenance of the appended rows: `git show HEAD:src/simplified_vehicle_models/solvers/
  planar_forward.py > /tmp/premove_planar_forward.py`, loaded as a module and used to
  (a) prove the pre-move code reproduces the original 50 rows bitwise and (b) record the 6
  new rows; the first 50 rows are byte-identical to the previous fixture. Generator:
  `/tmp/append_limited_rows.py`.
* How the pre-move provenance was ensured: the generator
  `/tmp/record_planar_powertrain_fixture.py` (kept out of the tree; its exact command is
  in the fixture's `note` field) was run at 15:49:38, and `planar_forward.py` was edited
  at 15:50:18 — i.e. the recording preceded the move. The fixture records the rates and
  all powertrain outputs of the old code path only, so the bitwise gate is meaningful
  whichever file the values came from.
* Self-check of the gate: perturbing the moved `gearing` by 1e-9 relative (mutation M6)
  makes the gate fail, so it is sensitive to sub-ulp-scale arithmetic changes.

### Mutation checks (all restored byte-exact; `shasum -c` verified)

| # | Mutation | Caught by |
|---|---|---|
| M1 | plant embeds `evaluation.rates` instead of `evaluation.forcing` | `VerticalForwardMassMatrixTests` |
| M2 | `initial_state` skips the generalized-force refinement | `VerticalForwardInitialStateTests` |
| M3 | tire-state rate rows sign-flipped | `VerticalForwardTireStateTests` |
| M4 | powertrain gear-efficiency factor inverted | refactor gate + closure audit |
| M5 | `kappa_eff` periodic tolerance one tier too tight | `VerticalForwardLayoutTests` |
| M6 | `gearing` perturbed by 1e-9 relative | `PowertrainRefactorGateTests` |

Review fix round (all six caught with an `AssertionError`, files restored byte-exact):

| # | Mutation | Caught by |
|---|---|---|
| R1 | row 1 (`n` rate) sign flipped | `VerticalForwardMassMatrixTests` |
| R2 | row 2 (heading rate) curvature-sign flipped | `VerticalForwardMassMatrixTests` |
| R3 | steering row sign flipped | `VerticalForwardMassMatrixTests` |
| R4 | powertrain `limited` forced to `False` | `PowertrainRefactorGateTests` |
| R5 | table-domain rejection not counted | `VerticalForwardLayoutTests` |
| R6 | quarter-car reference back to the previous `overshoot − 1` form | `VerticalForwardStepTests` |

## Rulings

Ruling: the step-test oracle is the seven-coordinate linear model built from the
closed-form mass (from the compiled sprung mass/inertias/inertance), the closed-form
stiffness (spring + ARB + tire, the same construction the Task 6 acceptance gates
validate against the evaluator's finite-difference linearization to 5e-7) and a
closed-form damping assembly (tire damping with the tire-stiffness structure, damper
slope/MR² on the jounce diagonal), integrated with the same 2 000 N front-axle step —
because a front-axle step excites heave (3.83 Hz) and pitch (7.30 Hz) together, so the
plan's body-fixed quarter-car reference (4.42 Hz, ζ = 0.49 → 17.1 % overshoot) cannot
describe the first peak; the ledger already ruled that the full-vehicle pitch mode is
7.3 Hz rather than the plan's 3.4–4.7 Hz band. Cost if wrong: the step gate would not
detect a genuine ride/damper regression that keeps the coupled two-mode response intact.
Measured: peak −1.46 %, overshoot −4.10 %, peak time −0.50 %, all inside ±15 %. The
quarter-car reference is computed and reported in the same test (`test_quarter_car_
reference_is_reported_not_gated`) and fails the ±15 % band on the overshoot (+65 %), and
that failure is asserted, not hidden.

Ruling: the step test uses a 0.5 kg hub-to-chassis inerter on both axles (via
`dataclasses.replace`) rather than the zero-inertance baseline, because the only way to
superpose an external generalized force on this implicit plant is to use the
`RateSystem(forcing + offset, mass_matrix)` path, and `mass_matrix` is `None` without
inertance; the oracle carries the same `b/MR²` term. Cost if wrong: the step numbers are
those of a slightly heavier hub (jounce diagonal +3.5 %); the heave/pitch response the
gate measures is unaffected at the 0.5 % level.

Ruling: `DomainError` from a suspension table (spring, bump stop, jounce steer) is
converted to `TrialStateError` in `evaluate`. The plan's trial-domain sentence names only
"travel limits plus 10 mm", but the compiled jounce-steer curve ends exactly at ±31.8 mm,
so a trial in (31.8 mm, 41.8 mm] would otherwise raise a hard `ValueError` out of
`solve_forward` instead of being retried. Cost if wrong: a genuine table-domain failure
inside the travel is reported as a rejected trial (counted) rather than as an exception;
the travel gate still fails such a state.

Ruling: `periodic_tolerances` uses three tiers in the planar runner's spirit — 0.5 mm for
lengths (`n`, `heave`, jounce), 0.005 for angles (`heading_error`, `roll`, `pitch`,
`steering`) and angular rates (`roll_rate`, `pitch_rate`, `yaw_rate`), 0.01 for linear
velocities, wheel speeds and jounce rates — plus the plan's Task 9 tire-state tolerances
(0.002 kappa, 0.2 mm deformation, 0.002 rad trail). Cost if wrong: Task 9's periodicity
gate is looser or tighter than intended for the body and wheel states; the plan fixes the
three magnitudes for the vertical set (0.5 mm, 0.005 rad, 0.01 m/s) and only the grouping
is chosen here.

Ruling: `out_of_range_evaluations` counts every evaluation that left a compiled table
domain — a damper velocity outside its table (`evaluation.out_of_range_count > 0`, the
evaluator's linear-extension flag) **and** a spring, bump-stop, or jounce-steer spline
query outside its domain (the `DomainError` converted to `TrialStateError`). This matches
the plan's Task 9 `table_domain_valid` gate, which names dampers, springs, and RC tables
in one counter. Cost if wrong: a rejected trial whose cause is a table exit is counted
twice if a caller also counts rejected trials as domain failures; the damper and
spline-exit contributions are not distinguished in the total, so a diagnostic that needs
to tell them apart must use the evaluator's own `out_of_range_count` and the exception
message.

Ruling: `observe` reports the road-plane `ax, ay` by refreshing
`self.body_accelerations` from the last evaluation and exposes a module-level
`body_accelerations(state, rates)` mirror of `planar_forward`; `DriverObservation` has no
acceleration fields and `PreviewDriver.command()` consumes none, so nothing else changed.
Cost if wrong: a later driver that expects the accelerations inside the observation needs
a `forward_driver.py` API change (Task 8 territory).

## Deviations from the plan

1. **`tests/test_powertrain_closure_audit.py` patch target.** The closure audit spied on
   `planar_forward.brentq`. The body it audits moved, so the spy follows it to
   `forward_powertrain.brentq`. No assertion, threshold or value changed; the test still
   brackets the same residual function. This is the "exact output regression" exception
   in the plan's global constraint.
2. **`tests/test_vertical_forward.py` also holds the refactor gate** (the plan lists it
   under Task 7 Step 1), even though it tests the planar plant.
3. **Fixture generator not committed.** The 50-state recording script lived in `/tmp`; its
   exact command and seed are recorded in the fixture's `note`/`seed` fields and above.
   Committing a generator under `tests/fixtures/` would have been an extra,
   untested file.
4. **`state_names` was not added to `PlanarForwardPlant`** (Task 8's job), and no runner,
   gate tool or tolerance vector was touched.
5. **Extra tests beyond Step 1's list**: a curved closed-loop `PreviewDriver` trial, the
   trial-domain rejection tests, the periodic-metadata test, and the powertrain
   pass-through test.

### Review-round deviations and disclosures

6. **Road-plane `ax, ay` are exposed on the plant, not in the observation.**
   `DriverObservation` has no acceleration fields and `PreviewDriver.command()` reads none,
   so `observe` refreshes `plant.body_accelerations` from the last evaluation and the
   module-level `body_accelerations(state, rates)` mirrors `planar_forward`. If a later
   task (Task 8's runner or a new driver) wants them inside the observation,
   `forward_driver.py` needs optional `ax`/`ay` fields — an API change outside Task 7's
   scope.
7. **`wheel_speed_margin` uses the lagged `kappa_eff` state.** The planar plant computes
   the margin from the instantaneous slip ratio; the vertical plant reads the integrated
   transient slip state, which lags it. Measured on the compiled vehicle at a 500 N brake
   command the two differ (0.982 vs 0.964), so the brake governor is slightly more
   permissive on the vertical plant. The tire's own relaxation is the physical reason for
   the lag; nothing else consumes the margin.
8. **Task 8's runner source-hash list must gain `forward_powertrain.py`.** Task 7 created
   the module, so the plan's Task 8 hash list (which names `planar_forward.py` only for the
   planar mode) needs `src/simplified_vehicle_models/solvers/forward_powertrain.py` added
   for both models, or the recorded hashes will not cover the shared closure.
9. **Memory is unmeasured.** Plan F17 asks Task 7 to report predicted full-lap wall time,
   memory, and accuracy. This report gives wall time (a 51 s projection) and the accuracy
   evidence above; peak memory was not instrumented, because the only measured trial is a
   100 m BDF run whose state is 38 × ~1.3 k samples plus the 38 × 38 Jacobians.

## Open issues

1. **Projected lap time is a projection.** 51 s at the median rate for 1 016.7 m is
   extrapolated from a straight 100 m trial with 0.10 throttle. A real lap adds steering,
   load transfer, and a different speed profile; Task 9/10 should measure the lap rather
   than trust this. Memory was not measured.
2. **The 6-Jacobian count is specific to this trial.** At `maximum_step = 0.005` and a
   near-steady straight the BDF reuses its Jacobian heavily; a full lap with steering and
   braking will re-form it more often, so the projected call count is a lower bound.
3. **The step gate uses only the heave/pitch response of one operating point** (15 m/s,
   zero command, straight). Roll and the wheel-hop modes are not stepped; ARB and hop
   behaviour stay with the Task 6 gates.
4. **`minimum_contact_load` and the load-closure diagnostics are near-trivial** for this
   plant (`load_method_counts = {"direct": …}`); consumers that read
   `maximum_load_residual` or `minimum_load_jacobian_determinant` get 0 and +inf. Task 8's
   `contact_valid` gate should read `minimum_contact_load ≥ 0`, `out_of_range_evaluations
   == 0` (now covering damper, spring, bump-stop and jounce-steer exits) and the travel
   bound, per the plan. The counter does not separate damper from spline exits; the
   exception message and the evaluator's `out_of_range_count` do.
5. **The transient relaxation constants remain PROPOSED** (Task 4A/5 handoff); the tire
   decay test verifies the plant propagates the planned `sigma_y`, not that `sigma_y` is
   fitted to this tire.
6. **Inertance is 0 in the compiled vehicle**, so the mass-matrix path is exercised only
   through `dataclasses.replace` in tests. A future nonzero-inertance compile needs the
   same task-10/task-9 gates rerun.
