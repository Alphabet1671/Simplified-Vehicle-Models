# Research roadmap: making the no-TV virtual-driver solver usable on all tracks

**Status:** Draft for technical review; not an acceptance policy or implementation-completion claim.  
**Prepared:** 2026-10-22.  
**Evidence baseline:** repository revision `6e1c4eeccdb50ff1dd8e08127c99b7be0aeb4c66`.  
**Scope:** no torque vectoring; four physical courses evaluated through six registered configurations.

## Executive recommendation

The next phase should prioritize **reliable curved-track feasibility and predictable convergence**, rather than more nominal lap-time optimization or additional fixed sampling points. The project already has substantial geometry, transcription, validation, and provenance infrastructure. Its unresolved problem is demonstrating that this infrastructure produces usable trajectories across the full track set within a declared computational budget.

The evidence suggests three distinct workstreams:

1. **Numerical reliability:** resolve between-site capability violations, MIS/FSC convergence failures, and initialization limitations.
2. **Correct problem definition:** settle open/closed boundaries, standing-start distance/time accounting, corridor assumptions, and the meaning of “virtual driver.”
3. **Qualification and deployment:** approve a dimensional convergence policy, execute the complete registered study, and provide reproducible runtime and artifact handling.

These workstreams should run in parallel where independent, but release must depend on all applicable gates. A solver success flag, a passing unit suite, or one conservative centreline run is insufficient.

### Evidence and interpretation

This document synthesizes the existing [results ledger](<no-tv-reliability-results-2026-10-22.md>) and [harness guide](<no-tv-minimum-lap-reliability.md>). It proposes research; it does **not** claim that the proposed methods have been implemented or experimentally validated. Hypotheses below are explicitly distinguished from measured results. It is not a new literature review or an MIS event report.

## 1. Define what “usable for ALL tracks” means

### 1.1 Required configurations

| Physical course | Required configuration | Boundary questions to settle | Present evidence gap |
|---|---|---|---|
| MIS endurance | `mis_closed` | Flying periodic lap; speed, path and applicable control continuity at the seam | No qualified curved-case convergence study; bounded attempts time out or fail |
| MIS endurance | `mis_cut_open` | Exact cut station, rolling entry state, terminal constraints, no unintended periodic coupling | No qualified cut-open study |
| FSC | `fsc_closed` | Flying periodic lap and corridor assumptions | Solver success has not established independently dense-feasible, converged free-line results |
| FSC | `fsc_cut_open` | Same source geometry with explicitly open boundary conditions | Timeouts/failures and missing qualification |
| Skidpad | `skidpad` | Distinguish rolling diagnostic from the intended event’s start, finish and timing definition | Repeated between-site capability violations despite solver/live-check successes |
| Acceleration | `acceleration` | Standing launch and exact timed distance versus rolling-10 diagnostic | Narrow rolling results pass local checks; standing-start event use is unsupported |

**ALL** means every applicable configuration passes; results must not be averaged into an overall pass. Cut-open cases are numerical/boundary-condition tests, not automatically distinct official events. Their initial conditions must not be silently replaced by standing-start assumptions.

### 1.2 Three levels of usability

- **U1 — numerical trajectory usability:** the same trajectory is optimized, exported and replayed; physical checks pass; mesh/basis/seed evidence supports the declared accuracy; runs finish within a declared budget or fail clearly.
- **U2 — event-time usability:** U1 plus correct start, finish, timing, distance and boundary semantics for the intended event. A positive speed-floor result is not a standing-start event result.
- **U3 — virtual-driver integration:** U1/U2 plus the simulator’s required tracking interface and demonstrated execution behavior. If “virtual driver” means a closed-loop steering/throttle/brake controller, path optimization alone does not satisfy this level.

**Recommended decision:** specify whether the immediate deliverable is an offline minimum-time planner or a closed-loop driver. Do not claim U3 from quasi-steady feasibility or matched-profile replay. Physical prediction against measured vehicle data is a further validation claim, not implied by any purely numerical level.

## 2. What is already available—and should be preserved

The documented implementation provides:

- Shared analytic reference/offset geometry, explicit reference-station/path-arclength mapping, and trajectory metadata.
- Energy-state Hermite–Simpson transcription and shared physical-time quadrature.
- An opt-in metric-consistent energy-rate control: physical node/midpoint accelerations remain physical quantities; off-site acceleration is recovered from the interpolated rate and analytic metric.
- Signed capability-domain handling, independent dense checks, and live original-model feasibility checks.
- Fail-closed validation, mesh/basis comparisons, matched references, reproducible bounded attempts, and input/code/runtime provenance.
- Structured failed-solver diagnostics and separate failed-iterate artifacts that cannot qualify as solutions.
- A bounded fixed-energy control-projection utility. Its guarantee is limited to its discrete equations and global bounds—not capability or OCP feasibility.

**Guardrails:** preserve source XML/CSV and existing caches; keep no-TV physics; retain failed evidence; do not relax validation to obtain a pass; do not reinterpret old artifacts under a new control representation. New formulations or conservative margins must be explicit, versioned and compared under matched conditions.

## 3. Measured blockers versus research hypotheses

| Observation in the ledger | What it establishes | What it does not establish |
|---|---|---|
| Rolling acceleration has dense/live passes across two meshes and a genuine four-versus-six-control basis comparison | Some narrow straight-course configurations are numerically viable | Cold-start robustness, all-track convergence, standing-start timing or global optimality |
| Corrected skidpad rate-control run: about 21.67353 s; speed-domain excess about 0.004086 m/s near station 15.17 m | Finite constraint sites miss a local physical restriction | That the capability table or physical model should be relaxed |
| Extra sites reduce the first excess to about 0.0000746 m/s; a 0.001 m/s margin exposes another peak near 129.60 m | Fixing one hotspot does not establish whole-course feasibility | That a fixed global margin is a general solution |
| A 0.05 m/s margin sensitivity satisfies the speed-domain check but leaves acceleration/braking excesses | Domain feasibility and longitudinal-force feasibility are separate issues | Feasibility or optimality of the original zero-margin problem |
| Zero-corridor skidpad still has acceleration/braking violations | Lateral freedom is not the sole cause of the failure | That the free-line problem is infeasible |
| MIS/FSC attempts reach bounded wall/iteration limits | Current methods/settings do not provide the needed results within those budgets | Physical infeasibility or impossibility of convergence |
| FSC fixed-energy control LP satisfies its HS equations and global bounds; its subsequent OCP times out | A seed-construction obstacle can be removed without clipping energy | A capability-feasible initializer or a converged minimum-time trajectory |

Pre-fix new-mode evidence affected by the corrected node/midpoint capability bug must remain excluded. The candidate times above are **not accepted event times**.

## 4. Prioritized research packages

### R1 — Whole-element capability enforcement and targeted refinement

**Priority:** P0. **Primary diagnostic course:** skidpad; then FSC and MIS.

**Question:** How can the optimizer prevent physical violations between its finite sites without imposing an unaffordable uniform mesh?

**Hypotheses to test:** localized extrema in curvature, metric, reconstructed speed, and piecewise capability interpolation produce narrow violations; uniform extra sites spend computation away from those extrema. These are plausible explanations, not a complete causal diagnosis.

**Experiments**

1. For frozen failed candidates, map each violation against source-geometry knots, offset knots, capability-cell boundaries and OCP elements.
2. Evaluate physical residual functions for speed domain, upper/lower acceleration, corridor and geometry validity over each suspect element. Search for local extrema and partition at known nonsmooth boundaries; do not assume one optimizer finds all extrema.
3. Compare three methods under equal wall-time budgets: uniform refinement; localized mesh refinement; and an exchange method that adds violated physical sites and resolves.
4. Where practical, investigate interval or conservative element bounds. State explicitly whether the result is a sampled empirical check or a continuous certificate.
5. Keep the acceptance validator independent. Finding new sites using one scan and accepting solely on that same scan is inadequate.

**Deliverables:** violation maps, runtime/accuracy trade-off curves, a bounded refinement algorithm with termination reasons, and regression cases for missed extrema.

**Exit:** dense/live-feasible skidpad and at least one endurance candidate, followed by transfer to all required cases. Any uncertainty-limited or exhausted check remains incomplete. Failure to meet runtime targets triggers method revision—not weaker tolerances.

### R2 — MIS/FSC convergence, scaling and initialization

**Priority:** P0. **Depends on:** diagnostic support already implemented; R1 can proceed concurrently.

**Question:** Are failures primarily poor seeds, conditioning, nonsmooth interpolation, transcription size, or insufficient runtime budgets?

**Controlled study**

- Record initialization, geometry construction, symbolic graph construction, solver, validation and export time separately. The hard budget must include the intended end-to-end operation.
- Inspect primal violations, scaled dual infeasibility, iteration/restoration histories, variable scales, and active constraint families from retained failed iterates.
- Compare centreline and free-line problems at the same longitudinal discretization and capability assumptions.
- Compare exact versus limited-memory Hessians, justified scaling changes, coarse-to-fine continuation, and source-station-aware meshes. Change one factor at a time.
- Compare current initializers with the bounded fixed-energy projection. Independently check positive dense energy, speed domain, signed acceleration limits, boundary compatibility and seam conditions before describing a seed as physically feasible.
- Test cold starts and preregistered perturbed seeds; separate feasibility restoration from minimum-time optimization.

**Decision rule:** preserve original physics and final constraints through continuation. Intermediate easier problems are allowed only as initialization; their solutions cannot qualify the target problem.

**Exit:** repeatable strict successes and dense/live-feasible candidates for MIS/FSC closed and cut-open cases within an agreed budget. Report success counts over all attempts, not only the best seed. A longer budget is an explicit product decision, not hidden evidence of improved numerical conditioning.

### R3 — Standing-start and open-boundary semantics

**Priority:** P0 for U2; may run in parallel with numerical research.

**Question:** How should the spatial formulation handle zero-speed launch without corrupting event distance and timing?

Investigate a short time-domain launch joined to the spatial phase at positive speed, versus a rigorously derived zero-speed energy formulation. A hybrid design must count launch distance inside the course distance and enforce continuity of position/station, speed/energy and any required states at the join.

**Required experiments:** analytical straight-line launch/braking benchmarks; transition-speed sensitivity; mesh refinement near the join; exact finish-distance checks; and comparison against direct time-domain integration under matched physics. Adding launch time to an already full-length positive-speed solution double-counts distance and is unacceptable.

For cut-open MIS/FSC, define cut station, initial state, terminal freedom, corridor interpolation and seam handling independently of the standing-start task. Test cut-location changes and boundary neighborhoods.

**Exit:** a documented event/boundary contract and converged launch-to-finish accounting. Until then, retain explicit rolling/flying labels and unsupported standing-start status.

### R4 — Capability-chart validity and conservatism

**Priority:** P1, promoted to P0 if R1/R2 implicate chart behavior.

Separate original-model feasibility, interpolated-chart feasibility, and conservative-margin feasibility. Investigate interpolation error at cell boundaries and signed acceleration intervals, continuity relevant to derivatives, curvature coverage, and behavior near the speed floor.

Use independent original-model probes concentrated at active candidate states and chart boundaries. Assess adaptive chart refinement or alternative interpolation only with explicit versions, physical checks and measured cost. Do not alter a chart merely to admit a failing candidate.

**Exit:** supported operating-domain coverage with an auditable error/conservatism budget. Margin sensitivity must show the lap-time/feasibility trade-off; a positive-margin optimum must not be reported as the unrestricted optimum.

### R5 — Dimensional convergence policy and complete qualification matrix

**Priority:** P0 governance work, initiated now; final qualification follows R1–R4.

An approved policy is missing. Draft a versioned policy **before** final tuning, with units, rationale, uncertainty handling, reviewer ownership and source hash. Base thresholds on intended downstream use and independent numerical/error studies—not on values that happen to pass current candidates.

Cover physical time, XY/path position, offset, metric, total path length, speed and curvature. Include localized maxima or quantiles as well as RMS so narrow violations cannot hide in averages. Separately specify feasibility tolerances, convergence tolerances and measurement uncertainty; do not conflate them.

Run all six configurations across registered mesh rungs, genuinely different realized basis grids, and a preregistered seed set. If nominal basis spacings produce identical grids, register an actually distinct comparison rather than relabeling existing diagnostics as preregistered evidence.

Keep vehicle/chart, boundary conditions, margins, representation, initializer policy and executed-code provenance fixed within each qualification edge. Use independent matched-reference and replay evidence; record failures, fallback categories and timeouts in the denominator.

**Exit:** approved policy plus complete qualifying evidence for every required configuration. Runtime-generated caller budgets alone must remain nonqualifying. A numerical local optimum is not a certified global optimum.

### R6 — Replay, driver integration and operational reproducibility

**Priority:** P1 for U1; P0 for U3.

Audit the complete export/reload/consumer path: station convention, units, curvature sign, control representation, time quadrature, periodic closure and boundary conditions. Test actual serialized reloads, not only two functions sharing the same in-memory object. Matched fixed-path replay is useful but is not closed-loop vehicle execution.

If a closed-loop driver is required, define reference signals and outputs, actuator/rate limits, tracking tolerances, initial-state recovery and safe failure behavior. Evaluate whether the quasi-steady model omits dynamics material to that interface, using matched higher-fidelity replay or measurements. Design those tests before promising tracking performance.

Replace dependence on temporary wheel locations with a documented reproducible installation and tested dependency versions. Ship inspectable artifacts and manifests; ignored local run directories are development evidence, not a durable release package. Test cancellation, failed archives, stale input detection and partial output recovery.

**Exit:** a clean-environment reproduction and end-to-end consumer test for every required case, plus closed-loop validation if U3 is claimed.

## 5. Execution sequence and decision gates

| Stage | Work | Gate before proceeding |
|---|---|---|
| A — Freeze and diagnose | Snapshot inputs/environment; define U1/U2/U3; classify failures; draft convergence policy | Reproducible baseline and agreed operating/boundary contracts |
| B — Curved feasibility | R1 on skidpad; R2 on MIS/FSC; targeted R4 chart studies | Whole-course dense/live feasibility without altered acceptance rules |
| C — Robustness | Mesh/basis/seed studies; runtime analysis; boundary/cut tests | Repeatable successes under declared budgets and matched conditions |
| D — Event and integration validity | R3 and applicable R6 work | Correct timing/distance and demonstrated consumer behavior |
| E — Qualification | Freeze policy/code; execute full six-case matrix; independent review | All applicable gates pass; evidence package reproducible by another user |

Do not commit to a completion date before the diagnostic stage establishes the dominant failure mechanisms and hardware/runtime target. Assign an accountable numerical-method owner, model/capability owner, event/interface owner and independent validation reviewer; one person may hold multiple roles, but acceptance review should not merely repeat the optimizer’s own checks.

## 6. Minimum experiment record

Every research run should include:

1. Case, source hashes, cut location, start/terminal conditions, corridor and timing convention.
2. Executed revision, runtime/platform, formulation/chart hashes and representation versions.
3. Actual mesh and basis grids, constraint sites, initializer and fallback policy, explicit margins.
4. Solver tolerances, Hessian/scaling choices, iteration cap and hard end-to-end wall cap.
5. Strict status, objective and physical time separately, primal/dual diagnostics, elapsed-time breakdown.
6. Independent physical residual maxima with stations, uncertainty/sampling status, live-model result.
7. Matched-reference, replay, seed/mesh/basis comparisons, and a machine-readable acceptance reason.
8. Candidate or failed-iterate artifacts clearly distinguished; no failed iterate promoted into qualification.

## 7. Research reading and evaluation agenda

A subsequent literature search should investigate adaptive direct collocation and constraint exchange, mesh-error estimation, nonsmooth/interpolated path constraints, optimal-control NLP scaling and warm starts, and zero-speed spatial/time-domain transitions. For driver integration, add trajectory tracking with actuator constraints and model-mismatch validation.

For each method, record its assumptions, supported smoothness, feasibility guarantees, implementation complexity, runtime evidence and compatibility with this signed capability model. Literature success on another vehicle or track is motivation for an experiment—not proof that it solves this project’s failures. No external paper has been assessed as part of this draft.

## 8. Release checklist and present verdict

An all-track release requires **every** applicable item:

- [ ] All six named configurations have strict solver, independent dense and live-model passes.
- [ ] Approved dimensional policy and qualifying mesh, realized-basis, seed, reference and replay evidence.
- [ ] Correct flying/rolling/standing boundary and event semantics, with no unaccounted launch distance.
- [ ] Predictable end-to-end runtime and explicit non-success behavior.
- [ ] Artifact reload/consumer equivalence; closed-loop validation if that product is claimed.
- [ ] Reproducible environment and durable evidence; source preservation verified.
- [ ] Relevant regression failures resolved or explicitly assessed for release impact; no blanket all-tests-pass claim.
- [ ] Independent readiness review distinguishes numerical usability from empirical predictive validity.

**Present verdict: not ready for all-track use.** The next decisive evidence is a robust, independently dense-feasible curved-course solution followed by transfer to MIS/FSC closed and cut-open cases—not another nominal solver success or a passing helper test.
