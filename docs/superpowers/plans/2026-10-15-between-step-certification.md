# 7DoF Between-Step Certification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Determine what can be certified over every accepted 7DoF BDF integration interval and refuse full-lap usability until all physical constraints have defensible interval-wide evidence.

**Architecture:** Compute bounds of each piecewise BDF dense-output polynomial on each accepted interval before its interpolation data are discarded; check explicit plant state-domain and path-progress inequalities using conservative bounds. Keep implicit tire-load, load-convergence, electrical closure and physical trajectory-error checks separate: do not mark full between-step certification from finite sampling or a partial interpolant certificate. Record independent, provenance-linked diagnostics and reproduce two warm-start track variants only after tests pass.

**Tech Stack:** Python 3, NumPy, SciPy 1.17 BDF dense output, unittest, existing bounded IEM24 runner.

**Spec:** `docs/validation/iem24-murphy/seven_dof_track_pair/acceptance_gate_trials.md` and `docs/superpowers/plans/2026-10-15-7dof-acceptance-gates.md`.

## Global Constraints

- Runner parent timeout must remain <=300 seconds per attempt and path spacing <=0.5 m.
- A lifted zero-load wheel is physically valid; negative contact load is not.
- A sampled midpoint does not certify an entire interval.
- A continuous certificate for an interpolant does not establish a bound on the physical ODE solution.
- Preserve prior cold/warm/refined attempts and existing uncommitted report artifacts.
- Do not set `between_step_validation_complete=true` or `usable=true` for a partial state-domain certificate.

---

### Task 1: Bound dense-output state components across intervals

**Files:** Create `src/simplified_vehicle_models/solvers/dense_domain.py`; Modify `src/simplified_vehicle_models/solvers/forward_driver.py`; Test `tests/test_dense_domain.py`, `tests/test_forward_driver.py`.

**Interfaces:** `dense_component_bounds(solution, index)` returns shape `(n_intervals, n_states, 2)` with conservative component minima/maxima from degree-5 BDF interpolation, or refuses unsupported method. `ForwardResult.dense_domain_bounds` is optional and defaults to `None` for generic plants. Test an arch with interior peak and endpoints/midpoint below the limit, an ordinary linear trajectory, and unknown interpolation types that must fail closed. Match reconstructed polynomial at additional interior locations with the public `solution(t)`; handle event-truncated final interval and round-off conservatively.

- [ ] Write tests for an interior peak, ordinary degree-1 segment, and unsupported dense interpolator. Run `PYTHONPATH=src:. python3 -m unittest tests.test_dense_domain -q` and confirm missing-function failure.
- [ ] Implement polynomial extraction and conservative interval bounds; rerun the tests.
- [ ] Integrate optional BDF-only bounds into `solve_forward`, preserving existing accepted-state and midpoint checks and wall deadline. Verify `tests.test_forward_driver`.

### Task 2: Certify explicit planar constraints without promoting partial evidence

**Files:** Modify `src/simplified_vehicle_models/solvers/planar_forward.py`, `src/simplified_vehicle_models/solvers/forward_driver.py`, `tools/run_iem24_forward_driver.py`; Test `tests/test_dense_domain.py`, `tests/test_forward_driver_runner.py`.

**Interfaces:** `PlanarForwardPlant.certify_dense_domain(bounds)` reports finite state bounds, SOC [0,1], longitudinal body speed >0.25, wheel speeds >=0, steering angle within the existing maximum, path denominator >=0.5 and path progress >0.25 using conservative curvature/trigonometric bounds; any unresolved bound fails closed. `ForwardResult.dense_polynomial_domain_certified` is only true if all accepted intervals pass. Runner records diagnostic counters and keeps `between_step_validation_complete=false`.

- [ ] Test explicit violations between nodes (SOC, steering, wheel speed, progress) and unsupported plants; verify they fail before implementation.
- [ ] Implement conservative checks with named assumptions and fail-closed behavior; verify focused tests.
- [ ] Add summary flag and source provenance; test no available result can be marked usable solely by these flags.

### Task 3: Review contact theorem and full certification boundary

**Files:** Create `src/simplified_vehicle_models/solvers/contact_bound.py` only if a mathematically complete parameter-dependent bound can be demonstrated; Test `tests/test_contact_bound.py`; Modify `docs/validation/iem24-murphy/seven_dof_track_pair/acceptance_gate_trials.md`.

**Interfaces:** A state-independent tire/contact bound may be reported separately only if it covers the fitted UniTire force law (including the linear slip branch), axle loads at every admissible speed, and nonconverged load-closure cases. Any unproven assumption keeps contact-theorem flag false; sampled `contact_valid` is unchanged. Document algebraic and physical-trajectory limitations even if explicit dense bounds pass.

- [ ] Derive the bound from implemented formulas and test deliberately unsafe parameters; retain the sampled flag if the proof fails.
- [ ] Run focused tests and document which constraints do and do not admit interval-wide certification.

### Task 4: Live acceptance and honest verdict

**Files:** New attempts under `docs/validation/iem24-murphy/seven_dof_track_pair/*/between_step_acceptance/`; Modify `docs/validation/iem24-murphy/seven_dof_track_pair/acceptance_gate_trials.md`.

**Interfaces:** Two independently capped warm-start laps on the respective original/canonical lines with matching model/source hashes and existing periodic/refinement settings. Re-run independent refinement only if the newly hashed solver/runner makes the previous baseline/refined provenance incompatible; use the same warm source for each pair. Preserve `usable=false` unless the entire physical interval gate is actually proved and every separate gate passes.

- [ ] Run `PYTHONPATH=src python3 -m unittest discover -s tests -q` and `git diff --check`.
- [ ] Run two 300-second-capped live attempts, inspect summaries and event logs; if timeouts occur retain failure evidence rather than assuming pass.
- [ ] If new paired refinement attempts are necessary for source parity, run them and compare on the common distance grid.
- [ ] Update the exploratory acceptance note with measured bounds, provenance, and explicit usable/not-usable verdict.
