# Forward-Driver Usability Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make newly generated canonical-track planar 7DoF and vertical 14DoF forward-driver attempts eligible for `usable=true` without weakening any existing acceptance threshold.

**Architecture:** Keep `acceptance_gate()` unchanged. Expose `PreviewDriver` steering configuration while retaining the existing 0.25 m lateral-error limit. Add a model-aware post-run certificate: retain the planar load-closure path and add a vertical state-domain-only dense interpolant certificate with provenance and fail-closed semantics. Fresh usability evidence chains and report regeneration remain incomplete because no tuned candidate was established.

**Tech Stack:** Python 3.13, NumPy, SciPy BDF dense output, unittest, existing XML compiler and forward-driver/gate utilities.

**Spec:** No separate product specification exists. The binding operational requirements are the unchanged acceptance gate in `tools/run/run_iem24_forward_driver.py`, the documented 0.25 m lateral-error threshold, per-run wall caps, and the current evidence/report conventions.

## Global Constraints

- Do not weaken, bypass, or special-case any condition in `acceptance_gate()`.
- Retain `path_tracking_valid` as maximum `abs(n) <= 0.25` m.
- Retain the planar 7DoF 300 s per-integration-run cap; do not count independent checking time against that cap.
- Preserve per-attempt isolation, source hashes, source snapshots, and existing historical evidence unchanged.
- A run can be labelled usable only after cold/warm/refinement evidence and a model-appropriate between-step certificate all pass.
- Do not claim physical validation against measured vehicle data; numerical acceptance remains distinct from model calibration.
- Keep all tests deterministic and avoid test cases that need a full canonical lap.

---

## File Structure

| Path | Responsibility |
|---|---|
| `src/simplified_vehicle_models/solvers/forward_driver.py` | Shared driver tuning/configuration and steering computation. |
| `tools/run/run_iem24_forward_driver.py` | Expose/record driver tuning and preserve warm-start compatibility. |
| `tools/gates/certify_iem24_forward_between_step.py` | Dispatch the correct planar or vertical certificate and update only the supplied finest attempt. |
| `src/simplified_vehicle_models/solvers/vertical_dense_certificate.py` | New conservative interval certificate for vertical dense-polynomial states and table/contact domains. |
| `tests/test_forward_driver.py` and `tests/test_forward_driver_runner.py` | Unit/regression coverage for driver settings, recording, and acceptance behavior. |
| `tests/test_vertical_dense_certificate.py` | New certificate unit tests covering passing and failing bounds. |
| `tests/test_forward_between_step_certificate.py` | Model dispatch and fail-closed/provenance tests using synthetic attempt fixtures. |
| `docs/validation/iem24-murphy/four_way_latest_model/` | Fresh attempt evidence and regenerated report only after all gates pass. |

### Task 1: Make shared driver tuning explicit and testable

**Files:**
- Modify: `src/simplified_vehicle_models/solvers/forward_driver.py`
- Modify: `tools/run/run_iem24_forward_driver.py`
- Modify: `tests/test_forward_driver.py`
- Modify: `tests/test_forward_driver_runner.py`

**Interfaces:**
- Consumes: `PreviewDriver.command(DriverObservation) -> DriverCommand` and the runner’s existing CLI arguments.
- Produces: immutable, recorded driver settings used identically by the planar and vertical plants; source-compatible defaults for callers that instantiate `PreviewDriver` directly.

- [x] **Step 1: Write failing tests for the explicit configuration**

Add tests that construct a curved `SpeedPlan`, pass an explicit preview/lateral/heading steering configuration, and assert `command()` remains finite, respects steering limits, and produces a different steering request from the legacy default at a non-zero lateral error. Add a runner parser/summary test asserting each driver setting is written to `summary["settings"]` and a warm start is rejected if a recorded setting differs.

- [x] **Step 2: Run the focused tests and verify failure**

Run:
```bash
PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver tests.test_forward_driver_runner -q
```
Expected: failure because the runner does not accept the new settings and the test cannot select the tuned configuration.

- [x] **Step 3: Implement the minimal shared configuration**

Add CLI options for `preview_seconds`, `heading_gain`, and `lateral_gain`, with the current defaults. Pass them to `PreviewDriver`; include them in `warm_start_matches()` comparison keys. Keep `PreviewDriver` defaults unchanged for library callers. Do not hard-code acceptance-specific behavior or branch on model type.

- [x] **Step 4: Select the configuration with a bounded recorded sweep**

  Attempted the initial candidate, but the run timed out before completing and its recorded planner did not match the wrapper's request. No configuration was selected; see `docs/validation/iem24-murphy/forward-usability-sweep/README.md` and Task 3 report. The bounded sweep objective is blocked on reliable one-lap orchestration.

Create isolated scratch attempts on the canonical track using 7DoF, the existing live plan and 300 s cap. Vary only the three newly exposed values; rank candidates by maximum lateral error, then lap completion and contact/resolution. Choose one configuration that gives margin below 0.25 m for both 7DoF and 14DoF cold laps. Record the exact candidate table and rationale in a validation note; do not commit scratch traces that are not the selected evidence chain.

- [x] **Step 5: Run focused tests and commit**

  Driver tuning interface committed as `385ffb9`; focused tests passed (see commit/report history).

Run:
```bash
PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver tests.test_forward_driver_runner -q
git add src/simplified_vehicle_models/solvers/forward_driver.py tools/run/run_iem24_forward_driver.py tests/test_forward_driver.py tests/test_forward_driver_runner.py
git commit -m "feat: expose forward-driver steering tuning"
```

### Task 2: Add a fail-closed vertical between-step certificate

**Files:**
- Create: `src/simplified_vehicle_models/solvers/vertical_dense_certificate.py`
- Modify: `tools/gates/certify_iem24_forward_between_step.py`
- Create: `tests/test_vertical_dense_certificate.py`
- Create: `tests/test_forward_between_step_certificate.py`

**Interfaces:**
- Consumes: vertical attempt `dense_polynomials.npz`, `trace.csv`, `summary.json`, the compiled `14dof-simple` formulation, and recorded source hashes.
- Produces: `vertical_dense_certificate.certify_vertical_dense_trajectory(vehicle, dense_polynomials, summary) -> dict`, whose `certified` field is true only when every interval bound proves all required domains.

- [x] **Step 1: Write failing certificate tests**

Add synthetic dense-polynomial fixtures covering: a finite in-bound vertical state interval that passes; jounce beyond `jounce_travel_limits_m` that fails; non-finite/missing bounds that fail; and a tire-load/contact interval whose lower bound is negative that fails. Add an entrypoint test showing a 14DoF summary dispatches to the vertical certificate rather than importing the planar-only load-closure code.

- [x] **Step 2: Run new tests and verify failure**

Run:
```bash
PYTHONPATH=src:. python3 -m unittest tests.test_vertical_dense_certificate tests.test_forward_between_step_certificate -q
```
Expected: failure because the module and model-aware dispatch do not exist.

- [x] **Step 3: Implement conservative vertical state-domain certification**

Implement a pure certificate that rejects malformed data and proves, for every dense interval, finite bounds, positive path progress/domain denominator, SOC bounds, steering limits, wheel-speed floor, jounce travel, and the vertical model’s table-domain limits. Replay interval endpoints and midpoint through the vertical plant/evaluator to collect tire-load, damper/suspension/table-domain evidence; any non-finite result, negative load, lifted wheel, out-of-range counter, or state outside a bound fails. Return exact counts, worst margins, failures, scope, model name, and no optimistic inference about the true ODE trajectory.

Update `certify_iem24_forward_between_step.py` to read `summary["model"]`, verify model-specific expected hash coverage, use the existing planar chain unchanged for `planar-7dof`, and use the vertical certificate for `14dof-simple`. Both paths must require a passing refinement/tube artifact and current source hashes before setting `between_step_validation_complete`; unknown models fail with a report and leave usability false.

- [x] **Step 4: Run focused tests and commit**

  Commits `af4b783` and `f8c88ad` contain the vertical state-domain certificate and follow-up fail-closed fixes. Focused suite with warnings as errors passed (118 tests).

Run:
```bash
PYTHONPATH=src:. python3 -m unittest tests.test_vertical_dense_certificate tests.test_forward_between_step_certificate tests.test_forward_driver_runner -q
git add src/simplified_vehicle_models/solvers/vertical_dense_certificate.py tools/gates/certify_iem24_forward_between_step.py tests/test_vertical_dense_certificate.py tests/test_forward_between_step_certificate.py
git commit -m "feat: certify vertical forward intervals"
```

### Task 3: Produce complete canonical 7DoF and 14DoF evidence chains

> **Status: incomplete.** No qualifying tuning candidate or updated usability evidence was produced. The sole partial attempt timed out before a full accepted lap and had planner metadata inconsistent with the wrapper request; see the Task 3 report. Do not treat historical evidence as regenerated.

**Files:**
- Create: `docs/validation/iem24-murphy/four_way_latest_model/usability-fix-results.md`
- Create: new timestamped directories under `docs/validation/iem24-murphy/four_way_latest_model/runs/seven_dof/` and `runs/vertical_14dof/`
- Modify: `docs/validation/iem24-murphy/four_way_latest_model/report.md`
- Modify: generated `comparison_summary.json` and figures only through the established report generator.

**Interfaces:**
- Consumes: tuned runner and certificates from Tasks 1–2.
- Produces: source-hash-compatible cold/warm/refined/finer chains for each model, with post-run gate certificates and `usable=true` only for the finest accepted attempt.

- [ ] **Step 1: Run cold canonical attempts**

Use the chosen settings and existing report arguments. Run 7DoF at `maximum_step=0.02`, `rtol=1e-5`, 300 s; run 14DoF at `maximum_step=0.005`, `rtol=1e-5`, its documented budget. Verify full lap, physics/contact/resolution, and `maximum_lateral_error <= 0.25 m` before proceeding.

- [ ] **Step 2: Run periodic warm starts and refinements**

For each model, warm-start from the matching cold attempt using identical settings. Run the documented finer step/tolerance settings and produce the existing refinement evidence. Check periodicity and refinement with the model-appropriate checker. Use additional predeclared refinement level only if the existing tube protocol requires it.

- [ ] **Step 3: Run the model-aware between-step certificate**

Run the certificate on each finest attempt. Require all source hashes to match its worktree, tube/refinement evidence to pass, and each certificate to set `between_step_validation_complete=true`. Confirm `acceptance_gate()` reports `usable=true` without editing the field manually.

- [ ] **Step 4: Regenerate the report and document evidence**

Invoke the existing four-way report generator against the final attempt directories. Update the report’s configuration/gate narrative and caveats to identify the selected final attempts, their numerical scope, and that no measured-correlation validation is implied. In `usability-fix-results.md`, list every command, attempt, source hash compatibility result, gate outcome, runtime, and the tuning sweep table.

- [ ] **Step 5: Run regression suite and commit evidence**

Run:
```bash
PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver tests.test_forward_driver_runner tests.test_vertical_dense_certificate tests.test_forward_between_step_certificate tests.test_vertical_14dof_evidence -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
git add docs/validation/iem24-murphy/four_way_latest_model
git commit -m "docs: certify current forward model laps"
```
