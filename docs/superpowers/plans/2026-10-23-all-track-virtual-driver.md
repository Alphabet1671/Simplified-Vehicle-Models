# All-Track Virtual Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use subagent-driven-development to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Complete the all-track no-TV roadmap with actual six-case numerical qualification, correct boundary/event semantics and reproducible consumer execution, without fabricated acceptance.

**Architecture:** Extend the existing shared analytic trajectory and strict validator rather than replacing their physics. First remove uniform-site cost using explicit localized physical constraint sites, then implement bounded constraint exchange and diagnose remaining conditioning. Qualify a frozen configuration only after independent dimensional policy review, boundary/event tests and serialized-consumer tests.

**Tech Stack:** Python, NumPy, SciPy/HiGHS, CasADi/IPOPT, unittest, existing XML vehicle graph and track data.

**Spec:** [All-track research roadmap](../../dev/virtual-driver-all-track-research-roadmap.md).

## Global Constraints

- Preserve source XML/CSV and existing caches; keep no-TV physics; retain failed evidence; do not relax validation to obtain a pass; do not reinterpret old artifacts under a new control representation.
- New formulations or conservative margins must be explicit, versioned and compared under matched conditions.
- ALL means all six configurations: mis_closed, mis_cut_open, fsc_closed, fsc_cut_open, skidpad, acceleration.
- A solver success, local candidate, empirical sampled check, numerical optimum and physical predictive validity are distinct claims.
- No merge, push, history rewriting, destructive cleanup or external publication.
- Strict release cannot use diagnostic caller budgets as an approved convergence policy.
- No accepted standing-start event time until launch distance is counted inside the original course distance.
- One implementation writer at a time; independent read-only research may run concurrently.

## File and task map

| Task | Responsibility | Main files / interface |
|---|---|---|
| 1 | Sparse extra physical sites | qss_spline_ocp.py; keyword-only constraint_stations on builder/solver |
| 2 | Bounded residual search and constraint exchange | new capability_exchange.py, tests; consumes Task 1 |
| 3 | Convergence profiling and initialization integration | profile_line_seed.py, solver timing/diagnostics; consumes Tasks 1–2 |
| 4 | Boundary/event semantics | isolated launch/boundary module and tests; simulator integration as established by interface audit |
| 5 | Source-hashed dimensional policy | versioned policy/data and strict harness integration; independently reviewed justification |
| 6 | Replay/consumer/environment integration | trajectory artifact loader, consumer tests, reproducible dependency instructions |
| 7 | Actual six-case qualification and release | harness artifacts, curated results; frozen matched settings and actual gates |

### Task 1: Localized additional physical constraint sites

**Files:** Modify `src/simplified_vehicle_models/solvers/qss_spline_ocp.py`; create `tests/test_qss_constraint_stations.py`.

**Interfaces:** Add keyword-only `constraint_stations=None` to `build_spline_ocp` and `solve_spline_line`. Existing positional arguments and default behavior remain unchanged. Built output exposes actual `off_site_stations`, `off_site_elements`, `off_site_fractions` and per-element indexing.

- [ ] Write failing tests for uneven per-element additions: one site in one element must not create that fraction in every element.
- [ ] Test invalid NaN/outside endpoints, duplicates, near-node/midpoint sites, mesh ordering, unchanged defaults and physical site acceleration semantics in both control modes.
- [ ] Implement a canonical union of legacy off-sites and finite explicit interior stations. Deduplicate coincident sites; map each to exactly one element/fraction; exclude already-enforced nodes/midpoints using a scale-aware tolerance.
- [ ] Replace rectangular `element * fraction_count + local` indexing with mapped per-element slices, using actual fraction for reconstructed energy/control. Reuse existing analytic geometry/corridor/capability/speed-cap constraints unchanged.
- [ ] Forward the solver keyword to construction and retain the actual site metadata on success/failure diagnostics without changing acceptance.
- [ ] Run `PYTHONPATH=src:/tmp/no-tv-casadi-runtime python3 -m unittest tests.test_qss_constraint_stations tests.test_qss_spline_ocp.SplineTranscriptionTest -v`.
- [ ] Commit explicit source/test files, report exact tests and concerns, then independent review.

### Task 2: Independent residual localization and bounded exchange

**Files:** Create `src/simplified_vehicle_models/solvers/capability_exchange.py` and `tests/test_capability_exchange.py`; modify harness only after core review.

**Interfaces:** Search consumes an immutable trajectory/state evaluator, mesh, signed capability and corridor; produces named physical residual maxima and source stations, not acceptance. Exchange consumes a fixed solver callback supporting Task 1 and an independent validator callback.

- [ ] Test a narrow synthetic violation between uniform sites, multiple local maxima, chart-domain boundary, exhausted/uncertain search, and a flat feasible element.
- [ ] Partition candidate elements at reference/offset knots and known interpolation boundaries where available; use bounded multi-bracket search plus independent samples, with explicit empirical/non-certificate labeling.
- [ ] Add only violated physical stations, retain all prior sites, impose round/site/wall limits and record reasons for halt.
- [ ] Test that optimizer/search agreement alone cannot grant acceptance; only independent full validation supplies local feasibility, while all convergence gates remain required.
- [ ] Reproduce the stored skidpad hotspots without hand-coded station constants; compare uniform/local/exchange under equal end-to-end budgets.
- [ ] Commit and independent review before extending actual endurance trials.

### Task 3: Conditioning and initialization study

- [ ] Instrument geometry/build/solver/validation/export time separately without changing the objective or constraints.
- [ ] Use retained failures to distinguish primal, dual, interpolation and initialization problems; compare one factor at a time on matched MIS/FSC closed/cut-open cases.
- [ ] Integrate bounded fixed-energy control projection as an explicit optional seed strategy; test HS, bounds, domain and seam independently, with no clipping/false feasibility claim.
- [ ] Run cold and preregistered perturbed starts; compare exact/limited-memory and coarse-to-fine methods under declared budgets.
- [ ] Review whichever numerical changes are supported by measured conditioning evidence; publish all attempts and denominators.

### Task 4: Boundary and event contract

- [ ] Audit existing driver/time integration interfaces and record whether the product exposes offline planning, closed-loop driver, or both; preserve existing consumers.
- [ ] Implement a short launch phase joined at positive speed, or a justified zero-speed energy alternative, with original course distance shared rather than duplicated.
- [ ] Test constant-acceleration analytic launch, braking, transition-speed sensitivity, exact finish distance and independent time integration.
- [ ] Define cut location, rolling entry, terminal freedom and seam handling for cut-open MIS/FSC; test multiple cut locations.
- [ ] Review physical and event semantics independently before removing unsupported labels.

### Task 5: Dimensional policy before final tuning

- [ ] Define units, RMS and localized-error measures for time, XY, offset, metric, length, speed and curvature; separate feasibility and convergence uncertainty.
- [ ] Justify budgets from consumer accuracy and independent analytic/error studies; preserve policy draft status until independent review accepts the justification.
- [ ] Hash/version the accepted policy and test fail-closed mismatch, missing evidence, diagnostic caller override and source/runtime changes.
- [ ] Register genuinely distinct basis grids and seed/rung coverage before collecting final evidence.

### Task 6: Consumer and reproducible runtime

- [ ] Round-trip real serialized artifacts into an independently instantiated consumer and compare station/units/signs/control/state/time/seam metadata.
- [ ] Implement and test the audited driver interface, actuator constraints and recovery behavior if closed-loop execution is part of the product; do not equate surrogate replay to higher-fidelity execution.
- [ ] Replace temporary dependency assumptions with a documented tested installation; test cancellation, partial output, stale inputs and failed archives.
- [ ] Preserve separation of numerical usability and empirical vehicle validation.

### Task 7: Frozen full qualification

- [ ] Freeze reviewed policy/code/chart/runtime and run all six cases with declared mesh, realized basis, seed, reference and replay coverage.
- [ ] Retain every failure/timeout and require all applicable strict gates; do not average cases into acceptance.
- [ ] Run relevant regression suites and assess known unrelated failures for release impact.
- [ ] Independent broad readiness review; fix findings with scoped review; only mark goal complete when all roadmap release checklist items actually pass.

## Execution notes

This is an adaptive research implementation plan: later numerical details are selected from measured failures, not invented in advance. Each later implementation dispatch must receive a concrete brief with exact interfaces and tests before coding. Unresolved research is not permission to weaken a gate. The direct user request authorizes continuous execution; no execution-choice pause is needed.
