# All-track execution evidence — 2026-10-23

**Verdict: not complete; not approved for all-track use.** This is a bounded snapshot of reviewed implementation and actual run evidence, not a release report. A solver's `Solve_Succeeded`, one case's local checks, or successful consumer completion does not satisfy the all-track gates. The roadmap's “ALL” is six configurations; failures and qualifications cannot be averaged. The evidence baseline is revision `47abf8d` (`47abf8dfc5ce089bbd571c1b9cebb0560a8ae2cc`); the documentation commit adding this report is later. No merge or push is made. No merge, push, or broad all-repository green claim is made.

## Implemented and reviewed components

The following scoped changes were implemented and reviewed; these are components, not completion of their roadmap tasks:

- **Task 1 — sparse physical constraint stations:** `21257d7`.
- **Task 2 — bounded capability search/exchange:** `91a7979`.
- **Task 3a — optional bounded profile-line seed and stage timing:** `727190f`.
- **Task 4a — original-model launch phase:** `e54f3e7`.
- **Task 4b — frozen remainder and exact distance/time accounting:** `81f9394`.
- **Task 6a — serialized trajectory loader/reference path:** `e7b38a6`.
- **Task 6b — reproducible environment manifest and pins:** `a59781d`.
- **Task 5a — diagnostic dimensional comparator:** `47abf8d`, independently reviewed as safe for diagnostic use. It is **not** an approved acceptance policy.

Task 1/2 provide localized enforcement and bounded search machinery; neither provides a continuous feasibility certificate or all-case qualification. Task 3a supplies a seed option and instrumentation, not a completed convergence study. Task 4a/4b demonstrate scoped launch and accounting behavior, not a jointly optimized, event-qualified standing start. Task 6a/6b provide loader/environment pieces, not qualified full dynamic tracking. Task 5a remains explicitly diagnostic.

## Actual numerical evidence

### Skidpad

The coarse rolling skidpad run with conservative **0.05 m/s** speed-domain margin passed the recorded local geometry, dense, live-model, and sampling checks. The evidence records nine refinement levels, 427,031 samples, and 67 localized physical sites; the resulting time was **21.7677767458 s**. This is a useful bounded candidate, not strict acceptance, an unrestricted optimum, or proof of convergence.

Mesh/basis refinement rejects a convergence claim:

| Actual run | Time | Result |
|---|---:|---|
| Coarse rolling reference/candidate | 21.7677767458 s | Local geometry/dense/live/sampling checks pass; other strict qualification gates remain unmet. |
| Finer basis (15 controls) | 21.4857334531 s | Strict acceptance false; dense domain check fails. |
| Finer mesh (417 nodes) | 21.7573381411 s | Strict acceptance false; dense acceleration and finite-difference uncertainty fail. |

The refinement runs are diagnostic and unregistered. Their differences and failures mean the candidate is **non-converged**. A successful optimizer status is not a contrary result.

The actual serialized skidpad round-trip (`skidpad_artifact_roundtrip_20261023/result.json`) showed near-machine parity for the tested same analytic trajectory: speed error `5.33e-15`, acceleration `3.96e-13`, north-position error `4.26e-14`, curvature `6.70e-13`; serialized and original physical times were equal to displayed precision (21.767776745763612 vs 21.76777674576362 s). This establishes artifact identity/reload for that test, **not plant tracking**; an east-position error is not reported here.

### MIS and FSC

- **MIS bounded-seed attempt:** `59.6556073674 s`, solver `Solve_Succeeded`, strict `accepted: false`; it does not pass dense capability/corridor gates. The local-mesh/control diagnostic was `59.6493273736 s`, also `accepted: false`, with dense caps and finite-difference checks failing. These are failed qualification attempts, not evidence of infeasibility.
- **FSC registered original-NLP budget:** `50.7776616340 s`, `accepted: false`; dense capability/corridor gates fail. The stricter `1e-7` diagnostic attempt did not converge within its iteration budget and is retained as a failure. No validator tolerance was waived.

No closed/cut-open MIS or FSC set has completed the required strict/dense/live qualification. These individual attempts must not be generalized into a claim that a track is infeasible.

### Acceleration and dynamic consumer

The frozen standing-start component uses an original-model launch plus an exactly balanced remaining-course segment: launch distance **0.1295558445 m**, remainder **75.1704441555 m**, total **75.3 m**, distance balance zero, and combined time **6.6332530562 s**. It is frozen-path accounting only: `joint_line_optimized`, `accepted`, `physically_validated`, and `event_qualified` are all false. The launch-join speed-policy variation is about **14 ms**; it is not a qualified joint free-line or event result.

The full dynamic skidpad run used the explicit endpoint-hold consumer adapter and actual actuator bounds. It completed, traveled **234.688939 m** in **23.58070057 s** against a **21.76777675 s** reference, with maximum lateral error **0.1504348 m**, heading error **0.1101962 rad**, and speed-tracking error **1.99716 m/s**. Midpoints were checked and 144 out-of-domain endpoint queries were held by the **consumer adapter**, not by clipping the stored trajectory. The result explicitly records `accepted: false`. This is not time parity, periodic-state closure, standing-start qualification, or U3 tracking qualification; no physical trajectory states were clipped.

## Reproducibility, provenance, and scope

The actual fresh environment in `runs/no_tv_reliability/runtime_install_20261023/manifest.json` reports Python 3.13.13, NumPy 2.5.4, SciPy 1.17.1, and CasADi 3.8.1, with each module loaded from the run's own fresh virtual environment (not `/tmp`) and exact compatibility true. The reviewed focused verification log records **174 tests passing in 18.451 s**. The full recorded command line and module list are not in that log; reproduce the 174-test result only from the original invocation record, not from an inferred command. This is focused verification, not a claim that the entire repository suite is green. Historical unrelated failures, timeouts, and a Qt stall remain unresolved/relevant to broad readiness.

The recorded provenance review verified all seven source hashes and the tracked cache state unchanged from the prior ledger; no source XML/CSV or cache changes are claimed. These run outputs live in ignored local workspace artifacts: they are development evidence, not versioned release results or a durable evidence package. The snapshot reports their statuses as observed; it does not upgrade failed artifacts or infer absent gates. Earlier debug runs with incorrect result layout are invalid evidence and are excluded.

## Exact remaining roadmap checklist

The roadmap's release checklist remains open item by item:

- [ ] All six named configurations have strict solver, independent dense, and live-model passes.
- [ ] Approved dimensional policy and qualifying mesh, realized-basis, seed, reference, and replay evidence.
- [ ] Correct flying/rolling/standing boundary and event semantics, with no unaccounted launch distance.
- [ ] Predictable end-to-end runtime and explicit non-success behavior.
- [ ] Artifact reload/consumer equivalence; closed-loop validation if that product is claimed.
- [ ] Reproducible environment and durable evidence; source preservation verified.
- [ ] Relevant regression failures resolved or explicitly assessed for release impact; no blanket all-tests-pass claim.
- [ ] Independent readiness review distinguishing numerical usability from empirical predictive validity.

In particular, the independent Task 5a review kept policy approval on **HOLD**: numeric budgets remain assumptions because no approved consumer-accuracy requirement was established, and unsampled continuous maxima remain unproven. The comparator correctly leaves `accepted: false` and maximum-error certification false. No threshold was approved or relaxed here. Task 3 endurance convergence, Task 4 joint standing-start/free-line and boundary semantics, Task 6 actual driver tracking/safe-end behavior, and Task 7 all-six frozen qualification and broad readiness remain unfinished.

## Reproduction and interpretation guardrails

Use the exact commands and inputs recorded in the ignored run manifests/reports; run into a new output directory rather than overwriting retained artifacts. The recorded focused verification command was:

```sh
runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest \\
  tests.test_convergence_policy tests.test_no_tv_runtime tests.test_trajectory_artifact \\
  tests.test_standing_event tests.test_standing_launch tests.test_profile_line_seed \\
  tests.test_spline_stage_timing tests.test_no_tv_minimum_lap tests.test_capability_exchange \\
  tests.test_qss_constraint_stations tests.test_line_validation tests.test_adaptive_line \\
  tests.test_line_refinement tests.test_fixed_path_energy_nlp tests.test_repository_layout -q
```

This is the original invocation with its explicit 15 modules; it uses the fresh-venv interpreter without an inferred `PYTHONPATH`. The result is captured in `.superpowers/sdd/2026-10-23-all-track-virtual-driver/reviewed-feature-verification.log`. Verify source hashes against the recorded run provenance before comparing outputs. Do not substitute debug-layout runs, change the source revision/representation, or reuse outputs as if a different validation policy applied.

No numerical/runtime changes, new thresholds, research claims, or “all finished” declaration are introduced by this evidence summary. The full adaptive plan and its original release gates remain in [the implementation plan](../superpowers/plans/2026-10-23-all-track-virtual-driver.md) and [the research roadmap](virtual-driver-all-track-research-roadmap.md).
