# Task 13 — Evidence ledger and regression closure

Date: 2026-10-26. Scope: docs only plus tests; no solver, planner, controller, map, replay, or physical experiment changes. The required Task 13 brief could not be read from the current checkout path at execution time (the initial supplied absolute path resolved before runtime context changed; it is absent under current worktree). Outcomes supplied in the task request and verified against Task 7/9/10/11/12 reports are reflected here. The Task 8 physical matrix/report is absent because the study was not run.

## Deliverables and gate rulings

- Added `docs/dev/yaw-lap-suite-closure/index.md` with separate status cells for all 15 required dimensions, curated comparison links, failures, report links, known unknowns, and exact identities/settings/budgets where recorded.
- Updated `docs/dev/yaw-lap-suite-results.md`: replaced the five remaining-work sections with the actual gate outcome and preserved old historical tables/values.
- Updated `docs/dev/yaw-lap-suite-workflow.md` with closure-v1 identities, failure retention and containment/provenance caveats.
- Updated `docs/dev/yaw-lap-suite-study-protocol.md` with frozen task outcomes and the parked `//` root-source key limitation.
- Task 7: MIS strict grid evidence inconclusive: 2→1 m warm policy mismatch; 0.5 m warm/cold strict failures; no common validated basis. FSC strict-valid 0.5→0.25 m plans remain profile-refinement-unproven; relative time 1.976319%, max common-reference speed difference 3.698577 m/s, offset 0, exceeding declared project criteria. Task 8 physical basis matrix not applicable/not run.
- Task 9: historical exploratory fixed and optimized laps completed with full solver validation, but plan source identity is unknown and matched comparison status remains inconclusive. Sampled wheel-contact failures: MIS −14.307/−14.192 m; FSC −2.411/−2.039 m. Whole-body dimensions unavailable; swept containment false/unknown and no actual provider-run certificate is present. MIS max-step .025 repeat unchanged at printed precision; FSC repeat absent.
- Task 10 physical robustness not run; no controller candidate selected, no FSC lateral-penalty-removal claim.
- Task 11 checker is provider-based, but implementation/unit tests are not evidence of a real provider run certificate.
- Task 12 discrepancy reproduced: 64.802231718 s simulated vs 71.685012818 s median measured, 6.882781099 s / 9.60%; all treatment arms not-run/inconclusive.
- Parked Important `//` source-path key: accepted literally in provenance mappings; path normalization/portability equivalence is not established. A normalized reconstruction can fail source-hash lookup and produce an inconclusive identity; unsafe normalization could collapse distinct namespaces. No historical manifest was rewritten.

## Evidence identity details

Closure-v1 map source identity: `f8658cafbd35dc60cd195153ca4761db21d9a9d8b4819f1112a4f46ad7322419`. MIS reconstructed track identity: `69bce5525e17c12bf17655760e6aedd5e1a98af7bd2d23b1bb05f8fccafad66f`; FSC synthetic 3 m edge track identity: `9d23af92a8e943aad7a90da5581f73f2733ab3cbed3fbb454f8843bb5fc797eb`. Task 7 used 40 m control spacing, strict tolerances (`interval_q=1e-8`, acceleration/yaw=`1e-7`), 900 s profile allowance and 1200 s outer process watchdog, with r16/r24 confirmations. Fresh run manifests include file/array digests, contract hashes, settings and source/vehicle/track hashes; plan/source hashes for Task 9 historical plans remain unavailable and are explicitly not inferred. Replay identity/settings/budget and validator outputs are detailed in Task 9 report and run comparison report.

## Regression scope and commands

This closure pass runs modified scoped modules under process-group watchdogs (unit tests <=180 s; physical functional tests <=600 s), then the full repository command exactly once under 1800 s. The full-suite outcome is recorded below after execution. The earlier Task reports' focused results remain historical scoped results, not a whole-repository pass claim.

Task 13 documentation diff review found no source-code changes in this task. The pre-existing dirty source diff consists only of a new `ProfileInvalidError` declaration in `yaw_lap_artifact.py` and a clarifying accepted-command comment in `plant_channels.py`; unrelated existing diff/report/plans are not staged by this task. No Task 13 change modifies OCP, strict tolerances, masks, caps, controller defaults, or failure-record handling.

### Test execution record

- Scoped unit regressions, process-group watchdog 180 s: `PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- python3 -m unittest tests.test_yaw_study_contract tests.test_yaw_profile_validation tests.test_yaw_lap_artifact tests.test_compare_yaw_lap_study tests.test_yaw_convergence tests.test_yaw_profile_connectivity tests.test_yaw_profile_transition tests.test_coupled_yaw_profile tests.test_yaw_replay_validation_cost tests.test_yaw_watchdog_group tests.test_yaw_replay_binding tests.test_yaw_replay_metrics tests.test_yaw_lap_suite tests.test_yaw_aware_record_path tests.test_yaw_planning_replay tests.test_yaw_tracking_cases tests.test_yaw_tracking_driver tests.test_yaw_corridor_validation tests.test_yaw_mis_report tests.test_yaw_mis_sensitivity tests.test_mis_report_standard -q` — initial run **FAIL**, exit 1; 188 tests in 23.638 s, 4 failures and 2 errors. Comparator fixtures lacked strict-valid `vehicle_configuration_hashes` attestations; a replay stub lacked `initial_state_overrides`. After fixing fixture attestations and using compatibility-safe `getattr(..., None)` for optional replay `initial_state_overrides` and `plant_variation`, exact rerun **PASS**, exit 0; 188 tests in 20.272 s under the same watchdog. Intentionally malformed and missing contract cases remain as tested. No strict production validation was bypassed.

- Scoped physical-window functional regression, process-group watchdog 600 s: `PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 600 -- python3 -m unittest tests.test_coupled_yaw_physical_windows -q` — **PASS**, exit 0; 2 tests in 3.716 s.
- Full repository command, run exactly once with 1,800 s process-group watchdog from worktree root: `PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 1800 -- python3 -m unittest discover -s tests -q` — **TIMEOUT**, exit 124 after 1,800 s; watchdog killed the process group. Output had partial test activity, which is not a full pass. The first attempted invocation from the repository parent exited 2 immediately because the worktree-only watchdog script was not present there; it never launched tests and is not counted as the required run. The actual complete-repository command above was launched once from the worktree and was not restarted.
- No test was labeled green based on partial full-suite output. Repository-wide status is **not green / timeout**; only the physical-window scoped module passed in this Task 13 invocation. The 188-test scoped suite failed.

## Commit scope

Commit only the new closure index and the three curated documentation updates (index, results ledger, workflow, study protocol). Do not stage ignored `.superpowers` Task 13 report or `runs/` outputs. Preserve all pre-existing dirty/untracked files. Explicit test-scope statement: only listed scoped module tests and the one full repository invocation are reported; no green status may be inferred if the full run times out or fails.
