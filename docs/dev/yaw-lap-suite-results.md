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

- Extend matched fixed-line controller evidence to independent replay of the optimized lines.
- Improve frontier/refinement efficiency enough to resolve the 0.5 m profiles within the hard runtime cap; do not widen strict tolerances.
- Demonstrate optimizer and control-basis/spatial convergence, rather than only bounded improvement.
- Validate optimized-line replay independently of fixed-line controller evidence.
- Preserve explicit footprint/between-step containment non-certifications.
