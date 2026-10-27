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

## Task 13 closure — five prior work items adjudicated

Task 13 closes the evidence ledger without upgrading unresolved claims. Full status cells, artifact identities, manifests, failures, and report links are indexed in [the closure index](yaw-lap-suite-closure/index.md). Historical values and comparisons above remain untouched and keep their original labels.

### 1. Fine-grid profile connectivity and runtime — **inconclusive**

MIS 0.5 m warm and cold closure-v1 attempts both fail strict validation; no plan digest/lap time exists. Earlier polynomial-consistency failure and later local-polynomial frontier budget exhaustion are distinct historical failures and remain preserved. FSC profiles are strict-valid through 0.25 m, but 0.5/0.25 m profile refinement is unproven. Runtime margins are case-specific: Task 5's three instrumented MIS replays passed its 1,200 s cooperative target under 1,500 s hard watchdog (median 164.3855 s, maximum 172.0019 s); no universal runtime-margin gate follows.

### 2. Numerical convergence and stopping — **inconclusive**

FSC 0.5→0.25 m valid-plan comparison misses project thresholds (1.976319% relative time; 3.698577 m/s maximum speed difference) and both profiles are refinement-unproven. FSC 1→0.5 m is likewise inconclusive (2.385749%; 9.427667 m/s; fine profile unproven). MIS 2→1 m is warm-policy incompatible; MIS 1→0.5 m has no strictly valid fine plan. No common validated grid exists, so Task 8 physical basis studies were not applicable and not run. Historical optimizer exits are budget/iteration stops, not convergence. No global optimality claim.

### 3. Optimized-line tracking — **inconclusive identity; exploratory laps completed**

Task 9 completed solver-validated fixed and historical optimized replays on MIS/FSC at BDF `rtol=1e-6`, `atol=1e-8`, max step 0.05 s, 150 s duration, 1,200 s cooperative / 1,500 s process watchdog. However plan source revisions/dirty hashes are absent, so paired identity and the planning→tracking→validation gate remain inconclusive. Sampled center passed; sampled wheel-contact checks failed. The MIS-only max-step 0.025 s replay repeated printed metrics; FSC integration sensitivity was not tested.

### 4. Robustness and containment — **inconclusive / not certified**

Task 10's perturbation and actuator matrix is planned, not physically run; there is no selected controller candidate and no FSC lateral-penalty-removal claim. Task 11 provides a provider-based checker, but no actual provider-run certificate binds to the Task 9 replays. Sampled wheel-contact violations fail; body dimensions are unavailable and whole-body/swept containment remain unknown. Sampled-center status does not imply footprint or between-step containment.

### 5. Measured-data attribution and reproducibility — **inconclusive**

Task 12 reproduces 64.802231718 s simulated versus 71.685012818 s measured median (6.882781099 s; 9.60%), with ten flying laps and standing-start exclusion. All controlled treatments (controller, line, battery, grip/map/replan, alignment) remain not run/inconclusive; historical configuration-source provenance is unknown. The historical report is not rewritten, and no causal contribution is assigned.

Task 13 scoped yaw regressions: **fail** (188 tests; 4 failures and 2 errors in 23.638 s under 180 s); physical-window functional tests: **pass** (2 tests in 3.716 s under 600 s); complete repository suite: **timeout** (exit 124 at 1,800 s, process group killed; run once only). Thus no whole-repository green claim. Exact commands and failure causes are recorded in the ignored Task 13 report and [closure index](yaw-lap-suite-closure/index.md). Legacy OCP, strict map semantics, tolerance values, masks, and controller defaults remain unchanged; no hidden acceleration cap or discarded failure record is accepted as part of this closure.
