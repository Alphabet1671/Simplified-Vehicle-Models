# Forward Driver Solver

This page describes how the forward driver solver works for the planar 7DoF model and the vertical 14DoF model. It follows ASD-STE100 (Simplified Technical English).

The page describes the code as read on 2026-10-17. It does not describe measured vehicle behavior. A completed lap is not a validated lap and is not a minimum-time lap.

## 1. Purpose

The forward driver moves a complete vehicle model along a fixed track path. It integrates all vehicle states in time. A simulated driver gives the steering, throttle, and brake commands. The integration stops at the end of the lap.

The 7DoF model and the 14DoF model use the same driver and the same integrator. The plants are different.

## 2. Files

| Item | File |
|---|---|
| Driver, speed plan, integrator | [forward_driver.py](../../src/simplified_vehicle_models/solvers/forward_driver.py) |
| Motor and battery closure (shared) | [forward_powertrain.py](../../src/simplified_vehicle_models/solvers/forward_powertrain.py) |
| 7DoF plant (12 states) | [planar_forward.py](../../src/simplified_vehicle_models/solvers/planar_forward.py) |
| 14DoF plant (38 states) | [vertical_forward.py](../../src/simplified_vehicle_models/solvers/vertical_forward.py) |
| 7DoF equations | [planar_7dof.py](../../src/simplified_vehicle_models/formulations/planar_7dof.py) |
| 14DoF equations | [vertical_14dof.py](../../src/simplified_vehicle_models/formulations/vertical_14dof.py) |
| Run script | [run_iem24_forward_driver.py](../../tools/run/run_iem24_forward_driver.py) |

## 3. Terms

| Term | Meaning |
|---|---|
| Plant | The vehicle model. It supplies observations and state rates. |
| Rate system | The equation `M * state_rate = forcing`. `M` is optional. |
| Speed plan | The requested speed, curvature, and feed-forward acceleration at each point of the path. |
| Trial state | A state that the integrator tries. The integrator can reject it. |
| Accepted state | A state that the integrator keeps. |
| Wheel-speed margin | The slip ratio of the most-locked wheel plus one. It is 0 at full wheel lock. |
| Path coordinates | `s` is the distance along the path. `n` is the lateral error. |

## 4. How the solver works

### 4.1 Setup (run script)

1. Compile the vehicle from the XML formulation. For the 14DoF, the driver and the planner use the `planar` part of the compiled vehicle.
2. Load the track. Make a distance grid with the `--spacing` value. Read the path curvature at each grid point.
3. Call `build_speed_plan`. The planner has two modes:
   - **Live mode.** Reduce the vehicle to a point-mass vehicle. Solve a flying lap with the point-mass solver. Limit the speed with `--maximum-speed` and with the optional lateral-acceleration cap.
   - **Conservative mode.** Use constant limits for speed, lateral acceleration, drive acceleration, and brake acceleration. Sweep forward and backward around the closed lap. Stop when the speeds do not change.
4. `SpeedPlan` calculates the feed-forward acceleration from the speed. It smooths this acceleration with a moving average of ±4 m along the path. This prevents a sudden step in brake force at a corner entry.
5. Create the plant (`PlanarForwardPlant` or `VerticalForwardPlant`). Create a `PreviewDriver` with the steering limits and the pedal limits of the vehicle.
6. Call `plant.initial_state(speed)` with the first planned speed:
   - **7DoF.** Set the state for steady motion on the planned curvature. Align the rear contact velocity. Set the wheel speeds from the contact velocities.
   - **14DoF.** Start from the static-equilibrium solution. Use a Newton solve on seven vertical coordinates (heave, roll, pitch, and four jounces) to make the vertical forces zero. Set the 12 tire states to their steady rolling values. Repeat up to 12 times.
7. With `--warm-start`, use the last state of an earlier lap as the initial state. Set `s` to zero. Restore the pedal position. The earlier lap must match the planner, the speed cap, the track, and the source hashes.

### 4.2 Integration (`solve_forward`)

8. Validate the inputs. All limits must be finite and positive. The method must be `Radau` or `BDF`. The initial state must match the plant layout. `wall_seconds` must not be more than `maximum_wall_seconds`.
9. If `pedal_travel_time` is set, add one state: the signed pedal position. The value +1 is full throttle. The value −1 is full brake. Throttle and brake cannot overlap.
10. Evaluate the right-hand side once at the initial state. An invalid initial state fails at this point.
11. Call SciPy `solve_ivp` with these settings:
    - A Jacobian that the solver supplies. It uses finite differences. If a perturbed state is outside the plant domain, it tries smaller perturbations.
    - A terminal event. It fires when the distance reaches the lap length.
    - Dense output.
12. If the plant raises `TrialStateError`, return NaN rates. The integrator then rejects the trial and reduces its step. The solver counts these rejections.
13. At each evaluation, check the wall-clock time. Raise `TimeoutError` when the budget is exceeded.

### 4.3 One right-hand-side evaluation

14. **Observe.** The plant makes a `DriverObservation`. It contains the distance, the lateral error, the heading error, the speed, the yaw rate, the steering angle, the lateral speed, and the wheel-speed margin.
    - The 7DoF plant calculates the slip ratio from the wheel speeds and the contact velocities.
    - The 14DoF plant reads the slip ratio from the transient tire states.
15. **Driver steering.** The driver adds four terms to get the desired steering angle:
    - Feed-forward: `atan(wheelbase × preview curvature)`. The preview distance is speed × 0.15 s.
    - Heading-error feedback, gain 1.5.
    - Lateral-error feedback, gain 1.5.
    - Yaw-rate damping, gain 0.1.

    The driver clips the result to the maximum steering angle. It converts the result to a steering rate with a time constant of 0.08 s. It clips the steering rate to the maximum steering rate.
16. **Driver speed.** Calculate `acceleration = feed-forward + 2 × (planned speed − speed)`.
    - The throttle is `acceleration / 6`, clipped to 0–1.
    - The brake request is `−acceleration / 6`, clipped to 0–1.
    - The brake governor multiplies the brake request by `margin / 0.85`, clipped to 0–1. This lowers the brake force when a wheel is near lock. It is not an ABS controller.
17. **Pedal.** With pedal dynamics on:
    - The pedal target is `throttle − brake force / maximum brake force`.
    - The pedal rate is `(target − position) / 0.02 s`. The rate is limited to ±1 / `pedal_travel_time`.
    - The plant receives the throttle and the brake force from the pedal position, not from the request.
18. **Powertrain.** Both plants call `evaluate_forward_powertrain`:
    1. Check that the command is in the physical limits and that throttle and brake do not overlap.
    2. Calculate the battery discharge power limit and charge power limit at the present state of charge.
    3. Calculate the requested motor drive torque from the throttle. Calculate the regeneration torque from the brake force.
    4. Find the bus voltage with `brentq`. The loop is: voltage → motor torque limit → bus power → current → terminal voltage.
    5. Scale the motor torque if the bus power is outside the battery limits.
    6. Add the hydraulic brake torques to the wheel torques.
    7. Check the voltage closure and the electrical power balance. Raise `RuntimeError` if a check fails.
19. **Plant dynamics.** See Section 5.
20. **Rate system.** The plant returns a `RateSystem`. `RateSystem.solve()` returns the state rates. If a mass matrix exists, it uses a dense linear solve. A singular mass matrix raises `ValueError`.
21. **Pedal rate.** The solver adds the pedal rate to the state rates.

### 4.4 Checks after integration

22. Raise `RuntimeError` if `solve_ivp` fails.
23. With `BDF`, calculate bounds for each state component over each accepted interval. Use the exact dense polynomial. `Radau` has no such bounds.
24. Evaluate every accepted state again. Do not use the trial-rejection path. This proves that no accepted state is outside the plant domain.
25. Evaluate the dense solution at the midpoint of each accepted interval. Compare the interpolant slope with the model slope. Record the largest scaled defect. This is evidence only. It is not a bound.
26. Return a `ForwardResult`. It contains the time, the states, the pedal, `lap_completed`, the wall time, the evaluation count, and the rejected-trial count.

### 4.5 Reporting (run script)

27. Write `trace.csv` and `pedal.csv`.
28. Calculate the lap summary: lap time, maximum lateral error, maximum step in distance, final state of charge, and closure of the periodic states.
29. Compare the closure with the plant tolerances. Set `periodicity_valid` from this comparison.
30. Set `between_step_validation_complete` and `refinement_validated` to `False`. Separate gate tools change them later. A completed lap alone does not make the result usable.

## 5. Differences between the 7DoF plant and the 14DoF plant

| Item | 7DoF (`PlanarForwardPlant`) | 14DoF (`VerticalForwardPlant`) |
|---|---|---|
| States | 12: `s`, `n`, heading error, `v_x`, `v_y`, yaw rate, 4 wheel speeds, SOC, steering | 38: 3 path states, 21 mechanical states, SOC, steering, 12 transient tire states |
| Vertical motion | None. The body acceleration gives the load transfer. | Heave, roll, pitch, and 4 jounces are states. |
| Normal loads | Solved in an inner loop. The loads depend on the acceleration. The acceleration depends on the tire forces. The loop uses damped Newton iteration. A fixed-point loop is the fallback. | Calculated directly from the tire deflection and damping. There is no inner loop. |
| Tire model | Quasi-steady UniTire | Transient UniTire, 3 states per wheel (effective slip, lateral deformation, trail angle) |
| Suspension | Not modeled | Springs, dampers, bump stops, jounce steer, roll-centre jacking, anti-dive, anti-squat, and optional inertance |
| Mass matrix | None | Present when inertance is more than zero. The 21×21 block is embedded in a 38×38 identity matrix. |
| Extra trial-domain checks | A negative contact load (both inside wheels lifted) raises `TrialStateError`. | A jounce outside the travel limit plus 10 mm raises `TrialStateError`. A spring, damper, bump-stop, or jounce-steer table exit also raises `TrialStateError`. |
| Periodic states | 10 states. The list excludes `s` and SOC. | 36 states. The list excludes `s` and SOC. |

### 5.1 Equations in both plants

Both plants use the same path equations. `κ` is the path curvature. `ψ` is the heading error.

- `ṡ = (v_x cos ψ − v_y sin ψ) / (1 − κ n)`
- `ṅ = v_x sin ψ + v_y cos ψ`
- `ψ̇ = yaw rate − κ ṡ`

Both plants reject a state in these cases:

- `v_x` is 0.25 m/s or less.
- A wheel speed is negative.
- The state of charge is outside 0–1.
- The steering angle is more than the physical limit.
- `1 − κ n` is less than 0.5, or the path progress is 0.25 m/s or less.

## 6. Known limits

- The 7DoF `observe` calculates the wheel radii with zero acceleration. It does not use the true load transfer.
- The brake governor is a proportional back-off. It is not tuned to the tire peak.
- The driver follows the speed plan. It does not find the minimum lap time.
- The midpoint check uses one point in each interval. It does not prove the whole interval.
- The `Radau` method gives no dense-output bounds.

## 7. Not covered

This page does not describe the option parsing, the model table, the parent watchdog, or the acceptance gate in the run script. See [Tools layout](tools-layout.md) and the validation reports under `docs/validation/`.
