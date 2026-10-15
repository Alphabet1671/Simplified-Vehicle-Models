# Shared Forward Driver Prototype Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task. The user requested parent-agent implementation and a separate Luna investigation.

**Goal:** Try a bounded, opt-in transient-wheel virtual-driver simulation with a model-independent mass-system interface.

**Architecture:** A driver uses physical observations, not plant state indices. A plant returns a mass matrix and force vector for stiff forward integration. Implement the planar adapter only; document future 10DoF/14DoF integration requirements and reject singular systems.

**Tech Stack:** Python, NumPy, optional SciPy Radau integration, existing XML/compiler and NumPy vehicle physics.

**Spec:** [compatibility/design](<../../validation/iem24-murphy/forward-driver-design.md>) and [master plan](<../../project-master-plan.md>).

## Global Constraints

- SI units everywhere.
- Wheel order is always `(FL, FR, RL, RR)`.
- Public functions do not mutate caller-owned NumPy arrays.
- All models operate on one prescribed path derived from XY coordinates.
- Preserve all dynamic wheel states and user-requested inertia `0.2064512 kg*m^2`.
- Parent wall limit <=300 s for every actual-lap trial; preserve attempts.
- No telemetry edits, no OCP edits, no automatic commits, no claims of physical 10DoF/14DoF implementation.
- Existing feature branch contains required uncommitted work; keep changes additive and inspect scoped diffs.

---

### Task 1: Pin inertia and shared mass-system contract

**Files:** Create `src/simplified_vehicle_models/solvers/forward_driver.py`, `tests/test_forward_driver.py`; modify `models/vehicles/iem24_murphy_real_planar.xml`, `docs/validation/planar-7dof-parameters.md`.

**Interfaces:** `RateSystem(forcing, mass_matrix=None).solve()` returns the complete state derivative; `DriverObservation(distance, lateral_error, heading_error, speed, yaw_rate, steering)`; `DriverCommand(throttle, brake_force, steering_rate)`; `ForwardPlant` declares `state_size`, `observe(state)`, `evaluate(time,state,command)`.

- [ ] Write failing tests for inertia, nonsingular mass solve, singular rejection and input immutability:

```python
self.assertAlmostEqual(vehicle.wheel_inertia, 0.5 * 10 * (16 * 0.0254 / 2)**2)
system = RateSystem(np.array([2., 6.]), np.diag([2., 3.]))
np.testing.assert_allclose(system.solve(), [1., 2.])
with self.assertRaises(ValueError):
    RateSystem(np.ones(2), np.zeros((2, 2))).solve()
```

- [ ] Run `PYTHONPATH=src python3 -m unittest tests.test_forward_driver -v`; confirm missing-module failure before implementation.
- [ ] Implement frozen dataclasses, shape/finiteness checks, nonsingular numerical solve and protocol. Correct XML inertia and document historical 12 kg runs separately.
- [ ] Re-run tests and inspect scoped diff; do not commit.

### Task 2: Implement driver, conservative planner and generic integration

**Files:** Extend `forward_driver.py` and its tests.

**Interfaces:** `SpeedPlan(distance,speed,curvature)` with interpolation; `build_speed_plan(distance,curvature,maximum_speed,lateral_acceleration,drive_acceleration,brake_acceleration)`; `PreviewDriver(plan,wheelbase,maximum_steering,maximum_steering_rate,maximum_brake_force).command(observation)`; `solve_forward(plant,driver,initial_state,duration,wall_seconds,maximum_step,rtol,atol)`.

- [ ] Add tests for seam-consistent speed planning, early braking, right-turn steering sign, steering-rate bound, exclusive throttle/brake, extra-state retention and deadline/shape rejection.
- [ ] Run the focused tests and confirm failures for absent functions.
- [ ] Implement closed speed-envelope passes, preview feedback and bounded Radau integration. Accept arbitrary state size; each RHS obtains the full rate system from the plant. Stop on lap progress, return accepted time/state arrays and explicit completion only, not OCP convergence.
- [ ] Verify a synthetic mass-matrix plant with extra decaying states:

```python
self.assertEqual(result.state.shape[0], plant.state_size)
self.assertLess(result.state[-1, -1], initial_state[-1])
self.assertTrue(np.all(np.diff(result.time) > 0))
```

- [ ] Run focused tests and inspect scoped diff; do not commit.

### Task 3: Implement actual transient planar adapter and bounded trial runner

**Files:** Create `src/simplified_vehicle_models/solvers/planar_forward.py`, `tests/test_planar_forward.py`, `tools/run_iem24_forward_driver.py`.

**Interfaces:** `PlanarForwardPlant(vehicle,plan)` implements the protocol; layout is `s,n,heading_error,v_x,v_y,yaw_rate,omega_fl,omega_fr,omega_rl,omega_rr,soc,steering`. `initial_state(speed)` initializes rolling wheel speeds and track-relative pose. `evaluate` returns all rates using existing tire/load dynamics and checked pedal/motor/battery allocation. Runner CLI provides output path and a hard parent timeout.

- [ ] Test wheel acceleration response, inertia scaling, tire-force response after coupled integration, rear-only drive, no current-state tire-force dependence on applied torque, electrical closure and invalid local-physics rejection.
- [ ] Run `PYTHONPATH=src python3 -m unittest tests.test_planar_forward -v`; confirm missing module failure.
- [ ] Implement adapter with bounded electrical closure and original brake-pedal mapping. Preserve dynamics feedback and load-solve convergence status. Reject reverse wheel motion in this initial forward-only adapter instead of inventing reverse friction behavior.
- [ ] Implement runner using the actual track; write flushed diagnostics and distinct JSON/CSV output. All setup and validation belong inside a parent-capped child. Label the planned envelope conservative, not an optimum or a live capability map.
- [ ] Run unit tests and short bounded actual-plant trials before a full-lap attempt. Preserve timeout and invalid-physics outcomes. Check accepted-node constraints and resolution; label between-node and periodicity/refinement limitations explicitly.
- [ ] Record timings, progress, failures, completed-lap status and remaining acceptance gates in `docs/validation/iem24-murphy/forward-driver-results.md`. Do not change old usable/converged fields or archived reports.

## Execution record

- Master plan reviewed in full; sections 3.2–3.4, 8, 9 and 14 define the compatibility boundaries. Added section 9.7 for the experimental forward workflow.
- Task 1 implemented and initial tests passed. Wheel inertia corrected to `0.2064512 kg*m^2`; old artifacts retained.
- Task 2 implemented and initial tests passed, including a 32-state synthetic nonsingular mass system. Added lateral speed to driver observations so course feedback includes sideslip.
- Full-lap experiments and all gates recorded in [the results document](<../../validation/iem24-murphy/forward-driver-results.md): the BDF full-lap attempt completed in 230.7 s with a 167.99 s conservative lap; gates and Radau timed out at 295.3 s at 927.8 m. Cross-method evidence, defects fixed, and open periodicity/refinement/between-step gates are in [the results document](<../../validation/iem24-murphy/forward-driver-results.md).
- Initial smoke failure exposed an inconsistent zero-sideslip corner seed. The corrected kinematic seed aligns rear contact velocity, but is explicitly not a periodic dynamic equilibrium.
- Initial full-run failure exposed machine-scale Jacobian perturbations interacting with local closure noise. A physically scaled finite-difference Jacobian completed a six-second transient without relaxing physical residual tolerances.
- SciPy is an optional `forward` extra in the package metadata. Additional runner deadline/retention tests are in `tests/test_forward_driver_runner.py`.

## Review checkpoints

Before any runtime claim, verify exact requested wheel inertia, all four dynamic wheel equations, no state clipping, original prescribed path, complete state propagation and parent deadline. Test selected existing planar/powertrain modules in addition to new tests. Actual 10DoF/14DoF plant and contact validation remains future work; do not substitute interface tests for it.
