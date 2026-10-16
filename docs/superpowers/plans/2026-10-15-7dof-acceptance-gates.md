# 7DoF Forward-Lap Acceptance Gates Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Evaluate and close the four open 7DoF driven-lap acceptance gates without asserting validity on incomplete evidence.

**Architecture:** Extend the existing bounded forward-driver runner and solver, preserving the 300 s parent timeout. Certify contact and between-step physics from validated states; investigate a warm-start protocol for periodicity; compare a second run at tighter integration settings for refinement. No acceptance flag is set merely because an integration returns successfully.

**Tech Stack:** Python 3, NumPy, SciPy BDF, unittest; current compiled IEM24 planar formulation.

**Spec:** `docs/validation/iem24-murphy/forward-driver-results.md` and `tools/run_iem24_forward_driver.py` acceptance_gate() definition.

## Global Constraints

- Runner parent timeout must remain <=300 seconds per attempt.
- A lifted zero-load wheel is physically valid; negative contact load is not.
- A completed transient lap is not a converged OCP optimum.
- Preserve existing measured and point-mass traces and existing uncommitted report artifacts.

---

### Task 1: Contact certification

**Files:** Modify `src/simplified_vehicle_models/solvers/planar_forward.py`, `src/simplified_vehicle_models/solvers/forward_driver.py`, `tools/run_iem24_forward_driver.py`; Test `tests/test_forward_driver_runner.py` and `tests/test_forward_driver.py`.

**Interfaces:** `PlanarForwardPlant.minimum_contact_load` records the minimum of all evaluated wheel loads; the solver reevaluates every accepted state; runner writes `contact_valid` only when completed and all accepted states passed with nonnegative loads.

- [ ] Add a failing test: fake plant returns negative contact evidence or accepted-state evaluation failure; runner cannot mark contact valid.
- [ ] Run `PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver tests.test_forward_driver_runner -q` to see the new test fail.
- [ ] Track minimum contact load in `PlanarForwardPlant.evaluate`; reevaluate all accepted solver states (not every Nth) and set the result flag only after completion; include `contact_valid` and minimum in summary.
- [ ] Run targeted tests and one capped live-plan trial, checking runtime and honest contact status.

### Task 2: Between-step certification

**Files:** Modify `src/simplified_vehicle_models/solvers/forward_driver.py`, `tools/run_iem24_forward_driver.py`; Test `tests/test_forward_driver.py`.

**Interfaces:** Use `solve_ivp(dense_output=True)` and evaluate physical state/command at the midpoint of every accepted interval. Set `ForwardResult.midpoint_sample_validation_complete` only after all sampled midpoints pass; leave `between_step_validation_complete=false` until continuous-interval certification is implemented. A finite sample never proves the whole interval.

- [ ] Add synthetic-plant tests whose endpoints are valid but interpolated midpoints violate the domain.
- [ ] Confirm failure with `PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver -q`.
- [ ] Enable dense output, check each interval midpoint under the same wall deadline, record midpoint-sampling evidence separately, and leave continuous-between-step certification open.
- [ ] Run targeted tests and a new capped live-lap attempt; do not mark success if checks timeout.

### Task 3: Periodic hot lap

**Files:** Modify `tools/run_iem24_forward_driver.py` and `tests/test_forward_driver_runner.py` if initial-state support is required; write a separate attempt summary.

**Interfaces:** Start another full lap from the prior final state while wrapping path distance to zero; carry wheel speeds, SOC and steering, preserving all physical constraints.

- [ ] Add a failing test proving seam initial state maps to the next-lap starting state without resetting physical states.
- [ ] Run targeted test; implement a bounded warm-lap option with explicit source attempt provenance.
- [ ] Run successive attempts within separate 300 s caps; evaluate the ten periodic deltas against the existing tolerances in `tools/run_iem24_forward_driver.py`.
- [ ] Keep `periodicity_valid=false` if any component still exceeds tolerance; report measured residuals.

### Task 4: Integration refinement

**Files:** Modify `tools/run_iem24_forward_driver.py` only if metadata needs supplement; Test `tests/test_forward_driver_runner.py`.

**Interfaces:** Compare independently integrated baseline and tighter-setting traces at common distance samples, with stated speed, acceleration, wheel-speed and lap-time tolerances; set `refinement_validated` only when explicit thresholds are met.

- [ ] Write test that rejects missing, nonfinite, or out-of-tolerance refinement metrics.
- [ ] Run targeted test and confirm failure.
- [ ] Run a separate capped BDF attempt at smaller max step and tighter relative tolerance; save provenance and common-grid comparison.
- [ ] Report actual results; do not call one run a refinement study.
