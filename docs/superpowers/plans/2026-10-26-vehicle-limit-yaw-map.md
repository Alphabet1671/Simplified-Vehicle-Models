# Vehicle-Limit Yaw Map Implementation Plan

> **For agentic workers:** Use subagent-driven-development for scoped implementation and review.

**Goal:** Replace the arbitrary ±1 g map extent with reproducible model-derived lateral coverage and a quantified interpolation boundary error.

**Architecture:** Direct LP evaluations discover signed feasible intervals over a declared speed/longitudinal-acceleration sampling domain. A model-derived force upper bound encloses the search; sampled scans do not assume monotonic feasibility. The rectangular map retains its strict feasible-corner mask. An independent direct-query audit distinguishes domain truncation from interpolation loss and fails the build when configured tolerances or budgets are not met.

**Tech Stack:** Existing Python, NumPy, SciPy/HiGHS, unittest; no dependency additions.

**Spec:** This document implements the request to build maps to the vehicle limit, replacing ±1 g. “Vehicle limit” means the current quasi-static tyre/aero/powertrain/brake LP model, not measured vehicle capability or transient safety.

## Global constraints

- Leave legacy QSS/OCP files and tests unchanged.
- Do not relax strict map interpolation or mask semantics.
- Never infer a global physical limit from the first infeasible probe.
- Report declared speed and ax coverage independently; this lateral task must not claim unrestricted ax coverage while using [-6, 3] m/s².
- Model calibration limits, numerical resolution, physical feasibility, and build completeness are separate statuses.
- Preserve exact zero acceleration knots. Widening an axis without maintaining resolution is not sufficient.
- LP numerical failures are unresolved, not physical infeasibility.

## Evidence

Coarse direct LP probes of IEM24 (no-TV) support |ay| approximately 15 m/s² at v=5, ax=0; 20 m/s² at v=33, ax=0; and 22 m/s² at v=33, ax=-6. These are integer-spaced observations, not final limits.

## Task 1: Discovery engine and tests

Create `formulations/yaw_map_domain.py` and `tests/test_yaw_map_domain.py`.

- [ ] Define immutable settings: scan spacing, bracket tolerance, probe budget, speed/ax audit subdivision, lateral interpolation-loss tolerance.
- [ ] Derive a conservative finite search bound from the actual tyre envelope: for UniTirePeakEnvelope, each wheel capacity is bounded by max(tabulated lateral peak force); thus |ay| <= 4*max_peak/mass. Record tyre load-table saturation separately. Unsupported tyre classes must raise a specific unsupported-bound error rather than substitute a g cap.
- [ ] Scan each signed ay ray at every requested speed/ax pair and holdout subdivisions. Use direct LP feasibility (both yaw directions contain zero) and cache coordinates. Do not stop after the first infeasible point.
- [ ] Refine feasible/infeasible transitions to the declared bracket width. Preserve separate sampled components; label the no-undetected-islands guarantee as resolution-limited, not continuous proof.
- [ ] Distinguish infeasible LP status from numerical failure; cap/time exhaustion returns incomplete diagnostics.
- [ ] Tests: limits beyond 1 g, asymmetric signs, disconnected support, empty slice, unresolved LP, probe budget, tyre-derived bound and malformed settings.

## Task 2: Rectangular map construction and acceptance audit

Create `formulations/yaw_map_workflow.py` and tests.

- [ ] Build nonuniform signed ay axis including zero and discovered boundary-adjacent feasible/infeasible points. Preserve configured maximum cell width rather than a fixed point count.
- [ ] Build through the existing physical map builder; no hull filling.
- [ ] Holdout direct LP probes assess outer-face truncation and unsupported-interpolation distance from directly feasible boundaries. Boundary discovery and acceptance probes must not be identical sets.
- [ ] Refine responsible speed/ax/ay cells within explicit node/call/time budgets. Emit complete only when audits meet requested tolerance; otherwise save unresolved evidence and do not advertise a qualified map.
- [ ] Report directional moments and zero-yaw feasibility separately. Full yaw-demand feasibility remains the planner's job.

## Task 3: Reproducible build CLI and shared integration

Create `tools/run/build_vehicle_yaw_map.py`, tests, and usage documentation.

- [ ] Persist NPZ map, exact axes, direct boundary brackets, holdout audit, settings, model/input hashes, implementation revision, timings and completion status. Refuse stale cache identity and refuse incomplete-map loading by default.
- [ ] Route runner, fixed-path probe and preflight through the same builder/load workflow. Keep explicit legacy-grid mode for comparisons, labelled limited-domain. No silent ±1 g default remains.
- [ ] Test build/save/load identity, incomplete output/exit code, all consumer paths and explicit limited-domain overrides.

## Task 4: Physical verification and review

- [ ] Build IEM24 no-TV map and compare to the old map on shared points.
- [ ] Show signed limits versus speed at ax=0 and braking/drive slices, enclosing search bounds, numerical uncertainty, axis sizes and runtime.
- [ ] Independently audit MIS/FSC demand coverage. Do not call steady ax=0 rejection proof of general dynamic infeasibility.
- [ ] Review code and evidence before accepting new default. Document any numerical resolution or tyre-load calibration limit instead of hiding it.
