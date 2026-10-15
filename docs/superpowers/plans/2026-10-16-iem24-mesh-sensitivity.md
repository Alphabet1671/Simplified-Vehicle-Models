# IEM24 7DoF Mesh Sensitivity Study Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the IEM24 7DoF mesh study at 0.05, 0.1, 0.5, 1, 2, and 5 m, preserve every attempt, and deliver a converged usable result only when spacing is at most 0.5 m and end-to-end solve time is at most 300 seconds per lap.

**Architecture:** Ordinary IEM24 runs default to and are capped at 0.5 m; requested coarser sensitivity points remain explicit diagnostic-only opt-ins. The study driver executes each requested spacing with identical physical/model settings, a 300-second per-run cap, separate directories, and records runtime, convergence, and usability. If none of 0.05/0.1/0.5 m converges within budget, diagnose the residual/runtime failure modes and make evidence-backed solver/initialization improvements without relaxing model physics or changing measured data.

**Tech Stack:** Python 3, existing IEM24 7DoF runner, IPOPT/CasADi, JSON/CSV/Markdown outputs.

**Spec:** User requests six mesh samples (0.05, 0.1, 0.5, 1, 2, 5 m) and imposes hard usability limits: “>0.5m step in distance is basically not usable” and “more than 5 mins of solve time per lap is also not usable.”

## Global Constraints

- Usable IEM24 7DoF results require mesh spacing <=0.5 m, solve wall time <=300 seconds per lap, and valid solver convergence.
- Ordinary 7DoF runs default to 0.5 m and reject coarser meshes; sensitivity runs may explicitly opt into 1, 2, and 5 m, but those samples are diagnostic-only and never usable lap predictions.
- Cap every sensitivity subprocess at 300 seconds; record timeouts and actual wall time without treating solver objective values as lap times.
- Use the same IEM24 formulation, track, solver tolerances, motor/tire assumptions, initial conditions, and brake policy across all mesh sizes.
- Do not modify `measured_stint.csv`; it is not an input to the OCP sensitivity sweep.
- Report lap time only as valid when the solver converges; retain residuals and status for every spacing.
- Store each mesh attempt in a distinct directory so retries do not overwrite prior evidence.
- Any solver fix must preserve model physics and constraints; do not manufacture convergence by relaxing tolerances/constraints or editing measured data.

---

### Task 1: Make coarse-mesh use explicit

**Files:**
- Modify: `tools/run_iem24_murphy_planar_7dof.py`
- Test: `tests/test_iem24_mesh_sensitivity.py`

**Interfaces:**
- Ordinary runner keeps `--spacing` default at 0.5 m and rejects values above 0.5 m.
- Keep `--allow-coarse-mesh` as an explicit opt-in for any positive spacing above 0.5 m; every opted-in spacing is diagnostic-only. The default matrix samples 1, 2, and 5 m above the usable cap.

- [x] Add argument-validation tests for accepted 0.5 m, rejected 0.5001 m, and accepted 1/2/5 m only with the opt-in.
- [x] Implement the 0.5 m ordinary cap/default while preserving explicit coarse opt-in.
- [x] Run the focused CLI tests and `py_compile`.

### Task 2: Add a reproducible six-mesh driver

**Files:**
- Modify: `tools/run_iem24_mesh_sensitivity.py`
- Test: `tests/test_iem24_mesh_sensitivity.py`

**Interfaces:**
- Default spacings: `(0.05, 0.1, 0.5, 1.0, 2.0, 5.0)` m. Pass `--allow-coarse-mesh` to the child runner for every requested spacing above 0.5 m.
- Each mesh owns `docs/validation/iem24-murphy/mesh_sensitivity/spacing_<value>/`; before rerunning, archive existing outputs under that mesh's `attempt_<n>/` directory so retries preserve evidence and a timeout cannot accept a stale summary.
- Each solver subprocess has a default 300-second timeout; an explicit timeout may only lower this cap and must be finite and positive. The driver records `runtime_s`, `converged`, `usable`, and status. A row is usable only if spacing <=0.5 m, convergence is true, and runtime_s <=300. For fine-mesh cache reuse, require recorded runtime; if missing, rerun. Matching coarse diagnostic caches may be reused with runtime marked unknown.
- Lap-time deltas use a converged, usable 0.5 m baseline; plots contain only usable rows. Coarser converged results remain table-only diagnostics.
- Final aggregate outputs: `mesh_sensitivity_summary.csv` and `mesh_sensitivity_report.md` under `docs/validation/iem24-murphy/mesh_sensitivity/`.

- [x] Add fixture tests for converged/failed rows, timeout rows, and usability predicates at 0.5 versus >0.5 m and 300 versus >300 seconds.
- [x] Implement sequential subprocess runs with identical iteration/projection/scaling settings and a 300-second default timeout; continue after timeout/failure and record measured wall time.
- [x] Preserve each attempt in a distinct directory and aggregate spacing, node count, runtime, lap time, convergence, usability, iterations, residuals, and status.
- [x] Run focused unit tests, `py_compile`, and `git diff --check`.

### Task 3: Execute the requested study within the hard runtime cap

**Files:**
- Create outputs under: `docs/validation/iem24-murphy/mesh_sensitivity/`

- [x] Include the verified 1 m attempt as diagnostic-only; preserve its nonconvergence and historical >300-second evidence.
- [x] Reuse exact-settings 1/2/5 m diagnostic caches with runtime shown as unknown where it was not measured; execute the missing 0.05/0.1/0.5 m samples sequentially, each capped at 300 seconds in non-overwriting attempt directories. Rerun any coarse point only if no matching cache exists.
- [x] Verify every requested spacing appears in the aggregate table, with runtime, convergence, usability, residuals, and status.
- [x] Mark only converged rows at <=0.5 m and <=300 seconds usable; never report raw objectives from failures/timeouts as lap times.

### Task 4: Diagnose and fix if no usable fine mesh converges

**Files:**
- Inspect/modify: `src/simplified_vehicle_models/solvers/planar_ocp.py`
- Modify as justified: `tools/run_iem24_murphy_planar_7dof.py`, `tools/run_iem24_mesh_sensitivity.py`
- Test: `tests/test_planar_ocp.py`, `tests/test_iem24_mesh_sensitivity.py`, and any focused regression test for the identified cause

- [ ] If none of 0.05/0.1/0.5 m is both converged and <=300 seconds, classify each failure from IPOPT status, iteration count, wall time, dynamics/algebraic residuals, inequality violation, control overlap, and power mismatch.
- [ ] Trace the dominant failure signal into the OCP constraints/initialization and design a deterministic regression test for the identified defect or numerical weakness before changing solver code.
- [ ] Use the Task 3 isolated 0.5 m (projection off, scaling off) 300-second timeout as the baseline trial; run fresh isolated 0.5 m trials for the remaining flag combinations (on/off, off/on, on/on) under the same 8,000-iteration cap, tolerance, physical constraints, and attempt-specific outputs. Compare status, wall time, dynamics/algebraic residuals, inequality violation, and control overlap across all four combinations.
- [ ] If none of those trials converges acceptably, inspect the existing `equilibrium_seed` and coarse-to-fine `warm_start` initialization paths in `PlanarOcpOptions`; test a justified initializer under the same 0.5 m / 300-second / 8,000-iteration limits, without relaxing tire/motor/brake physics or feasibility constraints.
- [ ] Implement only the smallest evidence-backed fix, then run focused regression/OCP tests and verify at least one requested fine mesh (0.5, 0.1, or 0.05 m) converges within 300 seconds without loosening feasibility or solver acceptance criteria; use 0.5 m as the first optimization target because it has the fewest nodes among usable spacings.
- [ ] If solver initialization/scaling settings change, update the sensitivity driver’s command and cache-matching metadata for those settings, then rerun all six meshes under one identical configuration to keep the study comparable; each subprocess remains capped at 300 seconds and outputs remain attempt-preserving. A usable result may be any of 0.5, 0.1, or 0.05 m; report remaining failures honestly.

### Task 5: Verify defaults, tests, and deliver the results

**Files:**
- Verify/modify if needed: `tools/run_iem24_murphy_planar_7dof.py`, `tools/compare_iem24_three_way.py`, sensitivity CSV, and Markdown report

- [ ] Confirm a default runner invocation selects 0.5 m; any spacing above 0.5 m requires explicit diagnostic opt-in.
- [ ] If a usable <=0.5 m 7DoF solution exists, regenerate the three-way measured/point-mass/7DoF overlay from that solution with comparison sampling no coarser than 0.5 m; otherwise do not present a failed/over-budget 7DoF curve as valid.
- [ ] Preserve the lap-start battery-voltage offset and measured telemetry; run the relevant comparison and OCP tests plus `git diff --check`.
- [ ] Present the sensitivity report and aggregate table, clearly distinguishing diagnostic-only coarse rows from usable <=0.5 m results and reporting every timeout honestly.
