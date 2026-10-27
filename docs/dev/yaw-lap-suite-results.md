# Yaw lap-suite implementation evidence

Snapshot: 2026-10-26. This is an engineering implementation ledger, not an MIS simulated-versus-measured report. Legacy OCP is unchanged. No hidden acceleration cap, filled map masks, or relaxed strict support/moment tolerance was used.

## Implemented

- Bounded dense-map initial proposals with protected required and matched cyclic seam seeds.
- Exact physical segment lengths preserved through cyclic graph rotation and independent validation.
- Vectorized layered graph reachability and shortest path with scalar-equivalent deterministic ties.
- Globally budgeted line search, exact coefficient success/failure cache, all seeds before local search, reference-station squared-speed warm suggestions, and best validated objective retained even when Powell is interrupted.
- Shared geometry-only seed linearization and deterministic seed contraction through the unchanged dense geometry gate.
- Persisted, digest/identity-checked plans; replay never repeats line optimization and independently revalidates every profile edge.
- Separate optimizer/profile/spatial-grid/control-basis convergence statuses and saved-study comparison. Unbound historical reports cannot certify convergence.
- Explicit model-aware tracking mode; legacy PreviewDriver defaults remain unchanged.
- Bounded failure summaries and unvalidated solver-state snapshots with last domain rejection cause.
- Cell-local interval polynomials avoid dense-cell cancellation; restricted knot enumeration matches the full-axis cut list in 1,000 randomized cases.

## Runtime and tests

Every physical run/test used a process-group watchdog below 1,800 s. Longest configured run in this snapshot is the final MIS replay: 1,400 s cooperative, 1,500 s hard deadline. No longer run was approved or attempted.

The final expanded scoped regression suite passed **128 tests in29.884 s** under a180 s process deadline. The printed0.25 s watchdog kill is an intentional descendant-kill regression, not a suite timeout. The comparator/convergence pair separately passed14 tests in0.089 s. The final six functional line tests passed in79.582 s under a600 s watchdog (previously157.908 s before shared geometry-only linearization). These are scoped suites, not a whole-repository pass claim.

The matched MIS 2 m fixed profile retained exactly 61.4706029303142 s while runtime improved from approximately 40.02 s (graph 27.89 s) to 14.37 s (graph 1.97 s). Physical timings are workload/concurrency dependent.

## Task 7 fresh profile and spatial-grid closure (2026-10-26)

Fresh run records for this closure are preserved under `runs/yaw_lap_suite/closure-v1/`; this section does not substitute the historical 2→1 m deltas below for convergence evidence. Full commands, per-run statuses/digests, comparisons, and rulings are in `.superpowers/sdd/2026-10-26-yaw-lap-suite-methodology-closure/task-7-report.md` in the agent workspace. The completed map is unchanged. MIS track geometry is reconstructed rather than independently surveyed; the FSC edges are synthetic 3 m edges.

MIS fresh 2 m and 1 m plans are independently valid but both profile-refinement-unproven; the 2→1 m geometry pair is explicitly inconclusive because required matched warm-start policy differs (cold 2m, warm 1m). MIS warm and cold 0.5 m attempts both failed strict validation and have no lap times or plan digests. No MIS 0.25 m study is eligible. FSC 2 m and 1 m plans are independently valid and profile-refinement-converged; 0.5 m warm and cold plans are valid but profile-refinement-unproven. The corrected FSC warm 1→0.5 m comparison exceeds project geometry thresholds (2.385749% time; 9.427667 m/s max speed difference; 0 m offset), but remains inconclusive because the finer profile is unproven. The required one-shot FSC 0.25 m warm suggestion completed strictly valid but refinement-unproven (56.5233479531 s); its 0.5→0.25 comparison is likewise inconclusive and outside project criteria (1.976319% time; 3.698577 m/s speed; 0 m offset).

All r24 follow-up objectives and plan digests exactly match the corresponding r16 case, including FSC 0.5 m and 0.25 m. No selected convergence-supported finest grid, serial repeat, or Task8 physical basis is established by this evidence. Cold/warm comparisons are sensitivity evidence only, not matched spatial refinement. Fresh full comparisons are curated in `docs/dev/yaw-lap-suite-closure/`.

Task7 revealed and corrected comparator implementation errors: plan digests were incorrectly treated as cross-run identity, nested line-spacing fields caused false incompatibility, profile objective recomputation used a different integration rule, and wrapped closed-heading seam was rejected. The comparator now verifies each artifact's plan digest, contract hash and strict validation, uses the exact harmonic segment-time objective, excludes only per-run digests/study coordinates from identity, preserves warm-policy matching, and accepts wrapped-heading seam equivalence. Focused comparator and contract suite: 18 tests passed under watchdog. These corrections do not alter solver code.

## Fixed-line spatial studies

| Track | 2 m plan | 1 m plan | Relative time change | Maximum speed difference |
|---|---:|---:|---:|---:|
| MIS | 61.4706 s | 64.1665 s | 4.386% | 5.828 m/s |
| FSC | 51.9168 s | 54.3389 s | 4.665% | 6.188 m/s |

These are **not spatially converged** results. Profile refinement is unresolved for the 2 m plans and FSC 1 m. MIS 1 m reports profile-refinement convergence, which does not prove spatial-grid convergence. MIS 0.5 m originally failed the strict polynomial consistency guard; local-coordinate construction removed that numerical failure, but the subsequent 300 s frontier-refinement study exhausted its budget without a validated path. Speed- and squared-speed-prolongated warm suggestions did not resolve this within the same bounded budget. No 0.5 m lap time is reported.

Historical physical study summaries created before digest binding remain exploratory evidence only. The production comparator explicitly treats them as inconclusive, rather than retroactively inventing a provenance binding.

## Actual multi-control line searches

| Track | Geometry | Control spacing | Controls | Retained lap | Fixed 2 m reference | Search outcome |
|---|---|---:|---:|---:|---:|---|
| MIS | 2 m | 40 m | 25 | 60.5892 s | 61.4706 s | wall-budget stop, ~602.7 s, 5 profile calls |
| FSC | 2 m | 40 m | 19 | 51.1684 s | 51.9168 s | wall-budget stop, ~615.4 s, 6 profile calls |
| FSC | 2 m | 20 m | 38 | 51.2578 s | 51.9168 s | iteration limit, ~518.8 s, 11 profile calls |

Both tracks demonstrate nonzero multi-control improvements, approximately 1.43% MIS and 1.44% FSC, without arbitrary speed caps. They do **not** demonstrate optimizer stationarity or control-basis convergence. The finer FSC basis is slightly slower, and different bounded search progress prevents interpreting the small time difference as convergence. Spatial resolution is also not converged. A useful validated incumbent is not a converged optimum.

An early physical line run exposed a bug where better valid in-Powell objective samples were lost when budget interruption prevented Powell returning. The implementation now promotes every improved valid evaluation immediately, synchronizes between windows, and has a targeted interruption regression. Reported retained results equal their best accepted candidate records.

## Matched FSC forward-controller comparison

Identical saved 1 m fixed plan, planned time 54.33894 s, unchanged speed targets:

| Metric | Legacy preview | Model-aware, original current-curvature yaw target |
|---|---:|---:|
| Completed driven lap | 58.79268 s | 55.24287 s |
| Speed error, time-weighted RMS | 1.15294 m/s | 0.32720 m/s |
| Lateral error, time-weighted RMS | 0.05418 m | 0.06379 m |
| Maximum absolute lateral error | 0.16037 m | 0.43161 m |
| Sampled centre corridor violation | 0 m | 0 m |
| Maximum realized throttle | 0.92593 | 1.0 |
| Time at realized full throttle | 0% | 15.258% |

The model-aware mode improves speed tracking/lap time but worsens lateral tracking. It is not uniformly superior. Full throttle results from physical drive-force inversion; no forced-throttle metric substitution is used. Both complete accepted/between-step solver validation; neither certifies footprint or between-step corridor containment.

The current implementation subsequently aligns yaw damping with the preview curvature used for feedforward. The FSC table above is intentionally labeled with the earlier controller revision. The **final matched-preview revision also completes**:54.96176 s driven, speedRMS0.29239 m/s, lateralRMS0.07007 m/max0.17543 m, sampled centre-corridor violation0 m. Against the identical saved plan and legacy preview baseline58.79268 s, it is3.83091 s faster with lower speed error, but lateral RMS/max remain slightly worse than baseline0.05418/0.16037 m. Requested full-throttle time21.732%; no requested brake/steering-rate saturation. Footprint and between-step corridor containment remain uncertified.

## MIS forward tracking: matched-preview correction succeeds

The early model-aware controller diverged near station 346.6 m, producing an unvalidated failure state with lateral displacement -12.17 m and a near-zero front wheel speed. This is a real tracking failure, not successful containment. Finite Newton-trial steering overshoot is now retryable without changing the physical bound, and inward steering-stop commands are tested.

A longer-preview gain experiment exhausted its 550 s integration budget. Matched-preview yaw damping advanced to accepted-state validation but exhausted 550 s there. The subsequent 1,400 s cooperative / 1,500 s hard-watchdog replay **completed all solver validation**: driven64.80223 s versus planned64.16651 s, lateralRMS0.06579 m/max0.23174 m, speedRMS0.27060 m/s, sampled centre-corridor violation0 m. Requested full-throttle time23.855%, no requested steering-rate/brake saturation. This is successful fixed-line replay, not footprint/between-step corridor certification. The same final revision also completes FSC as reported above.

## Remaining work

The new virtual driver has completed fixed-line forward laps on MIS and FSC. Bounded line optimization, saved-plan replay, and convergence reporting are implemented. What remains is to demonstrate a reliable **optimized-line planning → tracking → validation** pipeline, not to implement those components again.

### 1. Resolve fine-grid profile connectivity and runtime

- Diagnose the remaining candidate/frontier connectivity bottleneck and reduce interval-refinement cost. MIS 0.5 m still exhausts its 300 s profile budget without a validated path, including with squared-speed warm suggestions.
- Obtain independently validated finer-grid profiles on both MIS and FSC without broadening support/moment tolerances, filling map masks, or introducing an arbitrary acceleration cap.
- Keep the complete process and its children inside the **1,800 s hard deadline per run**. Improve accepted-state and between-step replay-validation efficiency as well: the successful MIS replay required a 1,400 s cooperative allowance and a 1,500 s hard watchdog, leaving little practical runtime margin.

**Done when:** finer-grid planning and complete replay validation finish reproducibly within the hard limit, with useful runtime margin and explicit failure/budget diagnostics.

### 2. Demonstrate convergence, separately for each numerical layer

- **Profile refinement:** resolve the outstanding refinement uncertainty, including the coarse profiles and FSC 1 m case.
- **Spatial grid:** compare genuinely finer geometry grids at fixed line-control resolution, using the same map, track, settings, and common reference-station axis. The current 2 m → 1 m lap-time changes of approximately 4.4–4.7% are not convergence evidence.
- **Line optimizer:** demonstrate stationarity or a documented stopping criterion other than wall/evaluation/iteration exhaustion. The retained MIS/FSC improvements are feasible incumbents, not converged optima.
- **Control basis:** refine line-control spacing while holding geometry resolution fixed and giving each search sufficient, comparable progress. The existing FSC 40 m/20 m studies do not establish basis convergence.
- Generate fresh digest-bound study artifacts; historical unbound reports must remain exploratory/inconclusive.

**Done when:** the independent profile, optimizer, spatial-grid, and control-basis checks pass their declared tolerances, or the suite explicitly reports which claim remains inconclusive. This does not establish global optimality or continuous feasibility.

### 3. Validate tracking of the optimized lines

- Replay the saved optimized MIS and FSC plans through the full forward plant without reoptimizing them or substituting the fixed-line speed plan.
- Verify the physical-path/reference-station mapping, curvature and seam behavior, then collect completed-lap time, time-weighted speed/lateral errors, maximum errors, realized control saturation, and corridor violations.
- Compare fixed and optimized lines with the same final controller revision and consistent numerical settings. Fixed-line completion alone does not show that the optimized plans are trackable or improve driven lap time.

**Done when:** both optimized-line replays complete all solver validation and demonstrate their actual driven-time/tracking trade-off against the matched fixed-line baseline.

### 4. Strengthen controller robustness and containment evidence

- Reduce the remaining lateral-tracking trade-off. On FSC, the final driver improves speed tracking but has lateral RMS/max errors of 0.070/0.175 m versus the legacy preview driver's 0.054/0.160 m.
- Test sensitivity to initial-state/line perturbations, rapid curvature reversals, cyclic seams, and physically justified vehicle/actuator variations; retain diagnostic failures rather than claiming robustness from two nominal laps.
- Add whole-vehicle footprint and between-step corridor checks if stronger containment claims are required. Current zero violations concern the **sampled vehicle centre only**; accepted/between-step solver-domain validation is not a corridor-containment certificate.

**Done when:** the claimed operating range and tracking/containment tolerances have explicit passing evidence. Until then, preserve the footprint and between-step corridor non-certifications.

### 5. Close the measured-data and reproducibility gaps

- Use the [single-replay MIS measured-data report](../validation/iem24-murphy-yaw-replay/report.md) to investigate the 64.802 s simulated versus 71.685 s measured-median lap-time gap: 6.883 s, or 9.60%. The report establishes the discrepancy, not its cause.
- Separate controller/line/grip assumptions from battery initial-condition effects and measurement alignment. Keep reconstructed motor torque request distinct from delivered torque, and acknowledge that the reference track geometry derives from the same measured log rather than an independent survey.
- Embed complete configuration/source/controller provenance in future run records. The compared historical record lacks a configuration-source hash; hashing it now does not recover that missing historical provenance.
- Extend regression coverage and rerun the relevant integration tests after further changes. Existing scoped passes are not a whole-repository test-suite pass claim.

**Done when:** new reports and run records are self-contained and reproducible, major comparison offsets are explained or explicitly unresolved, and every stronger performance claim is tied to the exact tested artifacts.

Throughout this work, keep the legacy OCP unchanged, retain strict map semantics, and enforce the hard runtime limit. Neither further implementation nor successful nominal replay should silently upgrade the present convergence, optimality, or containment claims.
