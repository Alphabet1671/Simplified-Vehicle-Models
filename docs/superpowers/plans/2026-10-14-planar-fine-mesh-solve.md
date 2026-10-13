# Planar Fine-Mesh Solve Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Attempt a physically valid minimum-time planar hot lap on a curvature-resolving 1–2 m mesh and report verified feasibility rather than a plausible-looking infeasible lap.

**Architecture:** Repair the existing spatial NLP's electrical conservation and spatial SOC equation before interpreting lap times. Build consistent initial states with algebraic projection and integrated curvature on each mesh interval; solve incremental meshes with measured constraint residuals and retain reproducible traces.

**Tech Stack:** Python 3, NumPy, SciPy, optional CasADi 3.8.1/IPOPT in `/tmp/casadi_pkgs`, unittest.

**Spec:** `docs/validation/planar-ocp.md`, `docs/superpowers/plans/2026-10-12-planar-7dof-optimal-control.md` §§Global Constraints, Discussion Gate 5–7.

## Global Constraints

- SAE J670: +x forward, +y right, +z down; positive yaw and track curvature turn right; wheel order FL, FR, RL, RR.
- Fixed 20/80 front/rear torque split, friction-only braking, no traction control or invented tire-force clipping.
- Do not commit private input workbook or unmodified raw source-car sheet values; leave unrelated point-mass goldens unchanged.
- Never call an iteration-limited result converged, and measure equality and inequality feasibility as well as mesh accuracy.

## File Structure

- Modify `src/simplified_vehicle_models/solvers/planar_ocp.py`: correct OCP equations, path mesh, initialization, solve options, and feasibility reporting.
- Modify `tests/test_planar_ocp.py`: regression tests for electrical balance, SOC conversion, speed cap and projected interpolation.
- Modify `docs/validation/planar-ocp.md`: reproducible mesh study, constraint residuals, caveats.
- Create `tools/solve_planar_mesh.py`: restartable mesh experiment and compact result trace.

---

### Task 1: Make the OCP physically closed

**Files:** Modify `src/simplified_vehicle_models/solvers/planar_ocp.py`; test `tests/test_planar_ocp.py`.

**Interfaces:** `_NodeFunctions.algebraic(x,u,a,kappa)` returns four load, four wheel balance, and one pack-power equality; `dynamics(x,u,a,kappa)[5]` is SOC per metre.

- [ ] **Step 1: Write failing regressions.** At zero throttle/brake require zero current, at positive throttle require bus power equal `sum(Tdrive * omega / (motor_efficiency * inverter_efficiency))`, and require `dSOC/ds = -I/(3600*C*progress)`; assert motor speed below voltage-scaled maximum.
- [ ] **Step 2: Run** `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_planar_ocp.SymbolicResidualTests -v` **and see failures** before implementation.
- [ ] **Step 3: Add** `electrical_power = sum1(shares * throttle * drive_reference * wheel_speed)/(motor.motor_efficiency * motor.inverter_efficiency)` and `power_residual = terminal*pack_current - electrical_power`; divide the SOC derivative by `safe_progress`; add `scaled_speed - motor.maximum_motor_speed <= 0` separately for each motor. Update residual storage for nine equality equations and update the tests to pass a consistent pack current.
- [ ] **Step 4: Run** the targeted tests again and ensure all pass; commit this isolated correction before comparing laps.

### Task 2: Project a new-mesh guess and inspect geometry

**Files:** Modify `src/simplified_vehicle_models/solvers/planar_ocp.py`; test `tests/test_planar_ocp.py`.

**Interfaces:** `_interpolate_start(start, distance)` returns state/control/algebraic arrays; `_project_start(...)` returns an algebraically consistent guess using bounded least-squares at each node.

- [ ] **Step 1: Add failing tests** checking interpolation of a synthetic periodic state, electrical-current recomputation, and that a projected warm start reduces the symbolic load/wheel residual relative to linear interpolation.
- [ ] **Step 2: Run** `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_planar_ocp -v` **and verify targeted failures**.
- [ ] **Step 3: Implement** per-node bounded least-squares on four loads and four wheel speeds using `_NodeFunctions.algebraic`, initialized from interpolated values; calculate pack current from positive power root of `I*(OCV-R*I)=electrical_power`, and recompute SOC/time from the spatial derivatives. Reject an infeasible projection with measured residual rather than silently accept it. Measure fine-path curvature reconstruction error and use 1–2 m samples (or curvature-aware interval refinement); averaging curvature alone could conceal corner peaks. Retain exact closed path length.
- [ ] **Step 4: Run** the targeted tests and a ~20 m baseline experiment; record projection residual before/after. Commit the change.

### Task 3: Solve, verify, and report

**Files:** Create `tools/solve_planar_mesh.py`; modify `docs/validation/planar-ocp.md`.

**Interfaces:** the CLI emits status, objective, node count, periodicity, algebraic residual, spatial-dynamics residual, electrical power mismatch, motor-speed margin and grip margins for every completed mesh.

- [ ] **Step 1: Implement a mesh experiment** for ~20, 10, 5, 2 and 1 m with independent checkpoint files; accept interrupted/restarted runs; only continue to a finer rung when the previous solution satisfies all constraints.
- [ ] **Step 2: Test the harness** on a 20 m budget with a small iteration cap and prove it labels the solution unconverged (no false success).
- [ ] **Step 3: Run the finer meshes** within available compute, compare 2 m and 1 m where feasible; independently replay tire, loads, motor/battery and path curvature at every node. Record exact status and wall times.
- [ ] **Step 4: Update the validation report** with the measured result or the exact verified obstruction, run `PYTHONPATH=src:/tmp/casadi_pkgs python3 -m unittest tests.test_planar_ocp -q` and the full default suite, and commit only the tested code and report.
