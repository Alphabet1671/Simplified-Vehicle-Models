# IEM24 Verification Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix verified model/report defects and make the remaining 7DoF validation gap explicit.

**Architecture:** Keep powertrain physics, forward-run acceptance, and report/test maintenance separate. Do not promote a transient lap or an old OCP output to a validated result.

**Tech Stack:** Python 3, NumPy, SciPy, unittest.

**Spec:** Findings in `docs/validation/iem24-murphy/verification_2026-10-15/planar_forward/attempt_1792047163837755000/summary.json` and `docs/validation/iem24-murphy/verification_2026-10-15/comparison_summary.json`.

## Global Constraints

- Preserve unrelated local edits and saved runs.
- Regeneration is disabled in the current IEM24 setup; the regen fix must not change its baseline.
- A completed forward trace is not a validated periodic or minimum-time lap.
- MIS 2024 reports must follow `docs/standards/mis-2024-report-standard.md`.

## File Map

- `src/simplified_vehicle_models/solvers/planar_forward.py`: separate drive and regeneration torque bounds.
- `tests/test_planar_forward.py`: brake/overspeed regression tests.
- `src/simplified_vehicle_models/solvers/forward_driver.py`, `tools/run_iem24_forward_driver.py`, forward-driver tests: periodic shooting or explicit nonperiodic rejection.
- `tests/test_planar_7dof.py`, `tests/test_point_mass_seed.py`: align legacy OCP tests with the chosen solver support policy.
- `docs/validation/iem24-murphy/forward_driver_trials/attempt_1792006853120683000/report.md`: repair or archive the obsolete noncompliant report.
- `tests/test_mis_report_standard.py`: verify all active reports conform.

### Task 1: Regenerative braking near motor speed limit

- [x] Add a test in `tests/test_planar_forward.py`: enable regen, request 100 N brake force (below hydraulic threshold), set driven wheels above the voltage-scaled motor speed limit, and require negative rear motor torque and bus power; assert zero hydraulic torque. Also assert the existing regen-disabled test is unchanged.
- [x] Run `PYTHONPATH=src python3 -m unittest tests.test_planar_forward -q`; confirm the new test fails.
- [x] In `planar_forward.py`, apply `speed_regulated_torque_limit(...)` only to positive drive torque. Bound negative torque with the motor's regenerative torque and electrical limits, including charge-current limiting; prevent positive drive above the speed cap.
- [x] Rerun the same test module; require all tests to pass. Commit only this task's files.

### Task 2: Forward-lap validity

- [x] Add a forward-driver regression test using the current closing-state tolerances in the saved attempt: the trial must remain `usable: false` when any periodic delta exceeds its tolerance.
- [x] Run the forward-driver test module; confirm the test fails if the code accepts the nonperiodic trial.
- [x] Keep the current rejection rule; if a periodic lap is needed, implement a separate iterative initial-state shooting loop with a bounded iteration count and require periodicity, contact, resolution, and between-step validation before setting `usable: true`. Never label this controller result a minimum-time optimum. No shooting loop was needed for this verification fix; the current run remains explicitly unusable.
- [x] Rerun a bounded 7DoF forward trial and its tests. Report either a passing periodic state and diagnostics or a still-unusable trace; commit the runner/test changes only after verifying the acceptance gate. The recorded run remains `usable: false`; periodicity and between-step/refinement gates remain open.

### Task 3: Retired OCP tests and contradictory outputs

- [x] Decide whether the deleted `planar_ocp.py` and `point_mass_seed.py` APIs are supported. If supported, restore their implementations and test imports; otherwise move OCP-only tests out of active discovery and mark saved OCP summaries/three-way claims as historical and unverified. Do not simply skip failures without a documented policy. The APIs are retired; OCP-only tests were removed from active discovery, saved results are labeled historical/unverified, and stale comparison entrypoints now import without those APIs and clearly indicate the planar comparison is unavailable.
- [x] Run `PYTHONPATH=src python3 -m unittest discover -s tests -q`; require no import errors. Commit the tests/documentation change independently. Final result: 567 tests passed; the stale comparison entrypoints are covered by an import regression test.

### Task 4: Old report and final check

- [ ] In the old 146.55 s forward-driver report, correct the standard link, required signal rows, and missing `lap_spread.png` only if source data support a compliant regeneration. Otherwise archive it outside `docs/validation/**/report.md` with an explicit obsolete/nonvalidated label; do not invent metrics.
- [ ] Run `PYTHONPATH=src python3 -m unittest tests.test_mis_report_standard -q`, then the full unittest discovery command; require both to pass.
- [ ] Regenerate point-mass and measured comparisons into a fresh directory; check CSV hashes against the existing verified baseline and record the 7DoF validity flags. Commit only the task's intended report/test changes.
