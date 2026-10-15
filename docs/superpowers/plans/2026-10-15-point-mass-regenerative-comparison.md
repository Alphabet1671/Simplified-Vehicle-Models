# Regenerative Full-Vehicle Comparison Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan inline with review checkpoints. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make point-mass and planar runs honor the placeholder's wheel-specific torque and voltage-dependent motor limits, model regenerative braking in both, and publish a matched full-vehicle trace report.

**Architecture:** Keep the point-mass and planar solvers' state formulations intact, but make their motor drive/regen constraints consistent with the shared motor envelope, axle torque shares, and battery limits. Expose synchronized node/segment traces from both results, then generate signed DC-power and vehicle-dynamics overlays on the same 1 m track mesh.

**Tech Stack:** Python, NumPy, CasADi/IPOPT, Matplotlib, unittest, JSON, compressed NumPy archives.

**Spec:** User request in conversation: enforce the point-mass 20/80 motor torque allocation and loaded-voltage speed limit; enable regenerative braking for both models and plot recovered DC power as negative; report and overlay longitudinal/lateral acceleration, speed, DC power, instantaneous turning radius, and each wheel's normal load on the same placeholder vehicle/track.

## Global Constraints

- Use the placeholder formulation `models/formulations/placeholder_planar_7dof.xml`, assembly `models/vehicles/placeholder_2024_formula.xml`, tire `models/tires/hoosier-43075-r20-simple.xml`, and track `models/tracks/2024_michigan_endurance_closed.xml`.
- Preserve the planar drive wheel shares `[0.1, 0.1, 0.4, 0.4]` (FL, FR, RL, RR), corresponding to 20% front / 80% rear axle torque.
- Use the motor envelope's `regeneration_torque_fraction` and the battery's charge-current/voltage limits; report positive pack DC power for discharge and negative DC power for regeneration.
- Keep the planar optimization's signed DC power/current balance feasible; do not synthesize regen only in plotting code.
- Generate the new regenerative report in `docs/validation/point-mass-planar-regeneration/`; do not overwrite the original no-regen comparison artifacts.
- Use the same 1 m mesh and distance coordinates for all overlays; mark model-specific trace locations/averaging explicitly.

---

### Task 1: Share-Aware Point-Mass Drive Capability

**Files:**
- Modify: `src/simplified_vehicle_models/drivetrain/fixed_split.py`
- Modify: `src/simplified_vehicle_models/formulations/point_mass.py`
- Modify: `src/simplified_vehicle_models/powertrain/assembly.py`
- Modify: `src/simplified_vehicle_models/powertrain/motor_envelope.py`
- Test: `tests/test_point_mass.py`
- Test: `tests/test_powertrain_planar.py`

**Interfaces:**
- Reuse `wheel_shares(0.2)` to obtain `[0.1, 0.1, 0.4, 0.4]` in FL, FR, RL, RR order.
- Add a point-mass drive-limit path that accepts current speed, `StorageState`, and four wheel shares and returns a scalar total drive-force limit plus its active motor/battery/tire constraint.
- Add a voltage-aware motor-speed check that compares voltage-scaled motor speed with `maximum_motor_speed`; use the current SOC and terminal voltage, not only the initial open-circuit voltage.
- Leave legacy point-mass drivetrain behavior unchanged when the vehicle has no `FullDrivetrainRuntime` with a `MotorEnvelopeParameters` motor.

- [ ] **Step 1: Add failing motor-share and voltage-speed tests.** At a fixed wheel speed and terminal voltage, assert that a 20/80 split limits the total motor torque to `min_i(T_i/share_i)` and that reducing terminal voltage lowers the admissible vehicle speed. Include a regression where a wheel speed below the nominal-voltage limit but above the voltage-adjusted limit returns zero drive torque.
- [ ] **Step 2: Run the focused tests and verify failure.** Run `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_point_mass tests.test_powertrain_planar -q`; expect failures showing the current equal-unit aggregate cap and static speed cap.
- [ ] **Step 3: Implement share-aware drive limits.** Compute each motor's voltage-adjusted wheel-torque capability, preserve the 20/80 shares with `min(wheel_limit_i/share_i)`, and apply the battery discharge limit to the resulting total bus power. In `evaluate_equilibrium`, allocate positive longitudinal force by the same shares and mark the state infeasible if any wheel exceeds its remaining combined-slip tire capacity; do not redistribute an overloaded rear share onto the front axle.
- [ ] **Step 4: Couple the point-mass speed bound to the current electrical state.** Pass the SOC/voltage through corner-speed and transition calculations. Return zero positive drive capability once voltage-scaled motor speed exceeds the motor envelope, and ensure speed-cap construction uses the same voltage-aware motor limit.
- [ ] **Step 5: Re-run the focused tests.** Confirm fixed shares, per-wheel tire feasibility, motor torque/power limits, battery scaling, voltage-dependent motor speed, and legacy behavior all pass.

### Task 2: Smooth-Motor Regeneration in Point Mass

**Files:**
- Modify: `src/simplified_vehicle_models/powertrain/assembly.py`
- Modify: `src/simplified_vehicle_models/solvers/forward_backward.py`
- Test: `tests/test_point_mass.py`
- Test: `tests/test_powertrain_planar.py`

**Interfaces:**
- Extend `maximum_regen_force` / `resolve_braking` to support `MotorEnvelopeParameters` in radians per second, with the declared per-motor regenerative torque fraction, fixed wheel shares, voltage-scaled motor speed, motor/inverter/gear efficiencies, and battery charge-power limit.
- Preserve `EnergyReport.bus_power` sign convention: discharge positive, accepted regeneration negative.

- [ ] **Step 1: Add failing smooth-motor regen tests.** Assert regen force is zero above the voltage-adjusted motor speed, respects the 0.5 per-motor regen torque fraction, never exceeds battery charge power/current acceptance, and returns negative bus power when regen is accepted. Assert excess braking demand is assigned to friction braking.
- [ ] **Step 2: Run the focused tests and verify failure.** Run `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_powertrain_planar tests.test_point_mass -q`; expect the existing smooth-motor regen dispatch to fail or exceed one of the new limits.
- [ ] **Step 3: Implement smooth-motor regen dispatch.** Add an explicit `MotorEnvelopeParameters` branch in the regen limit calculation. Convert motor regen torque to wheel torque using the placeholder's gearing/efficiency conventions, cap total regen by per-wheel shares and accepted charge power, and retain friction force for demand beyond regen capability.
- [ ] **Step 4: Enable point-mass regeneration in the comparison runner.** Call `solve_lap_with_energy(..., regen_enabled=True)` and verify the energy integrator records accepted recovery as negative DC bus power without exceeding SOC 1.0 or the battery charge-current limit.
- [ ] **Step 5: Re-run point-mass regen tests.** Confirm torque, power, speed, sign, charge-current, charge-SOC, and friction-remainder behavior.

### Task 3: Regenerative Braking in the Planar OCP

**Ruling:** Represent the optimizer-selected total regenerative wheel torque as one bounded algebraic variable, distributed by the existing fixed wheel shares. This preserves the public three-control interface and seed format while allowing the optimizer to blend regeneration with hydraulic braking. The quasi-steady OCP will not add a regenerative-torque slew constraint; regen may change at mesh nodes.

**Files:**
- Modify: `src/simplified_vehicle_models/solvers/planar_ocp.py`
- Modify: `src/simplified_vehicle_models/solvers/planar_equilibrium.py`
- Test: `tests/test_planar_ocp.py`

**Interfaces:**
- Extend the algebraic vector by one scalar `regen_total_wheel_torque` at index 9, after four loads, four wheel speeds, and pack current. Keep controls `[throttle, friction_brake, steering]` unchanged.
- Per-wheel regen magnitude is `wheel_shares(DRIVE_FRONT_FRACTION) * regen_total_wheel_torque`; it is zero unless friction braking is requested, and each share obeys the motor's declared regeneration torque fraction plus voltage-scaled power/speed bounds.
- Expose the four signed per-wheel regenerative torques on `PlanarOcpResult`; expose signed node DC power in the existing `bus_power` field.
- Keep `pack_current` bounded by `[-charge_current_limit, discharge_current_limit]`; the electrical balance and SOC dynamics integrate negative regen power.

- [ ] **Step 1: Add failing OCP regen tests.** At a fixed braking node, assert per-wheel regen torques have the 10/10/40/40 split, bus power is negative, negative current increases SOC, regen is zero without braking, and charge-current/motor torque limits are not exceeded.
- [ ] **Step 2: Run targeted OCP tests and verify failure.** Run `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_planar_ocp -q`; expect the current 9-value algebraic vector and friction-only electrical equation to fail the new tests.
- [ ] **Step 3: Extend node equations and bounds.** Read `regen_total_wheel_torque` from algebraic index 9, subtract its shared per-wheel torque from wheel balance, and add its signed motor/inverter power to the electrical power residual using the regeneration efficiency direction. Bound the shared regen torque by `brake * regen_reference`, by per-wheel `regeneration_torque_fraction * torque_bound`, and by each wheel's voltage-scaled power bound. Retain the battery current bounds for charge acceptance.
- [ ] **Step 4: Update algebraic initialization and projection.** Increase `ALGEBRAIC_COUNT` from 9 to 10; initialize regen torque to zero in cold/equilibrium seeds, include it in all algebraic arrays and resampling, preserve the existing three-row control arrays, and compute current from signed motor bus power during local projection.
- [ ] **Step 5: Re-run the OCP tests.** Check equilibrium seeds, projected seeds, per-wheel torque allocation, signed pack power, SOC integration, charge limits, and OCP feasibility.

### Task 4: Full-Trace Extraction and Comparison Figures

**Files:**
- Modify: `src/simplified_vehicle_models/solvers/planar_ocp.py`
- Modify: `tools/plot_point_mass_planar_comparison.py`
- Create: `docs/validation/point-mass-planar-regeneration/`
- Test: `tests/test_planar_ocp.py`

**Interfaces:**
- Add per-node `longitudinal_acceleration`, `lateral_acceleration`, and signed `curvature` to `PlanarOcpResult`, computed from the same node functions used to certify the solve.
- Save a common-distance trace archive with point-mass and planar `ax`, `ay`, speed, signed DC power, reference radius, realized radius, and four-wheel `Fz`.
- Define point-mass realized radius as the centerline radius (`1/abs(curvature)`) because point mass has no yaw state; define planar realized radius as ground speed divided by absolute yaw rate, with straight-line values represented as infinity and clipped only for plotting. Plot the shared reference-track radius as a separate overlay for context.

- [ ] **Step 1: Add failing trace-shape/sign tests.** Verify planar acceleration traces and curvature have one value per node, normal loads have shape `(nodes, 4)`, and signed bus power contains negative values only when regeneration is active.
- [ ] **Step 2: Expose planar accelerations from the node dynamics.** Preserve physical `ax` and `ay` from the evaluated planar dynamics in the result instead of reconstructing from distance derivatives without progress-speed correction.
- [ ] **Step 3: Capture point-mass segment traces.** Store its `longitudinal_acceleration`, `lateral_acceleration`, `normal_loads`, energy `bus_power`, speed, and distance. Place segment-valued signals at segment midpoints and node-valued signals at nodes.
- [ ] **Step 4: Capture planar node traces and align segments.** Store the node-valued OCP signals; for comparisons against point-mass segment outputs, average adjacent planar node values at the segment midpoint and record that averaging in the report.
- [ ] **Step 5: Add the runner's regeneration flag and generate figures.** Add `--regen` to the comparison runner; when selected, pass `regen_enabled=True` to point mass and use the OCP's regenerative control. Write Matplotlib figures for `ax`, `ay`, velocity, signed DC power (negative regen), reference/realized turning radius, and four-panel FL/FR/RL/RR `Fz`; save them beside the JSON summary and compressed trace archive.

### Task 5: End-to-End Regen Benchmark, Verification, and Commit

**Files:**
- Create: `docs/validation/point-mass-planar-regeneration/full-report.md`
- Create: `docs/validation/point-mass-planar-regeneration/summary.json`
- Create: `docs/validation/point-mass-planar-regeneration/traces.npz`
- Create: six Matplotlib PNG figures named `ax.png`, `ay.png`, `velocity.png`, `dc-power.png`, `cornering-radius.png`, and `fz-wheels.png` in that directory.
- Modify: `tools/plot_point_mass_planar_comparison.py`

**Interfaces:**
- Runner defaults to the existing placeholder vehicle, Michigan closed track, 1 m mesh, planar cold/equilibrium seed, and regeneration enabled in both formulations.
- Report lap time, convergence/status, iterations, initial/final SOC, consumed/recovered/net kWh, peak positive/negative DC power, trace definitions/units, torque shares, and declared regen/charge limits.

- [ ] **Step 1: Run the matched regenerative cases.** Execute `PYTHONPATH=src:/tmp/casadi_pkgs MPLCONFIGDIR=/tmp/matplotlib-config python3 tools/plot_point_mass_planar_comparison.py --spacing 1 --iterations 500 --regen --output-dir docs/validation/point-mass-planar-regeneration`.
- [ ] **Step 2: Validate signed power and electrical conservation.** Confirm point-mass and planar regen intervals have negative DC power, recovered energy is positive in the report totals, `integral(Pdc dt)` matches the energy/SOC trace within integration tolerance, charge current stays within bounds, and both runs converge before comparing lap metrics.
- [ ] **Step 3: Inspect every figure and report.** Confirm the six overlays use the same track-distance domain, axes/units and radius definitions are stated, Fz wheel ordering is FL/FR/RL/RR, and negative power is visible below zero.
- [ ] **Step 4: Run regression tests and whitespace checks.** Run `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest discover -s tests -q` and `git diff --check`.
- [ ] **Step 5: Stage only implementation and report outputs.** Leave the unrelated `.DS_Store` untracked, review staged status, and commit as `Add regenerative full-vehicle comparison report`.
