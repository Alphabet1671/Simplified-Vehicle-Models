# TTC tire correlation GUI implementation handoff

## Local branch and launch

- Branch: `feature/ttc-tire-correlation-pyqt`.
- Isolated working tree: `.worktrees/ttc-tire-gui` beneath the original checkout.
- Implementation head: `7c1ec1d8fff8e2c0690718f3303c9d9c446d57d3`.
- Original planning checkout and unrelated edits were not integrated, reset or staged.
- No merge, push or publication was performed.

From the isolated working tree:

```bash
python3 -m pip install -e '.[tire-gui]'
PYTHONPATH=src:. python3 tools/tire_gui/app.py models/tires/hoosier-43075-r20-simple.xml
```

See [the user guide](../../dev/tire-correlation-gui.md).

## Delivered

Native PyQt6 steady/transient/diagnostics tabs; automatic steady force-then-trail correlation; gated transient scale/trail identification; precise manual controls, bounds and locks; history; asynchronous cancellable fitting/replay; explicit stale-safe candidate preview/application; actual segment/load selection and holdout metrics; private sessions; separate compiler-validated steady/transient XML bundles.

The mixed-tire identity remains explicit: 43075 R20 lateral, 43100 R20 longitudinal/combined proxy (different tire size and rim). Runtime tire physics, existing model XML and XML schema were not changed.

Four TTC runs and the requested MATLAB seed were copied to ignored private storage. Raw samples, source MATs and generated measurement artifacts are not tracked. Ordinary Git exclusions are not a substitute for checking applicable TTC licensing before sharing fitted outputs.

## Verification and limits

- Final tire-GUI suite: **77 tests passed**, including offscreen Qt tests.
- Direct final steady/transient/vertical tire runtime check: **62 tests passed** in 5.866 seconds.
- Timed offscreen app launch passed during GUI implementation.
- Whole-repository discovery timed out at 300 seconds. Its last visible test was `test_open_standing_start.OpenSpeedPlanTests.test_lateral_margin_one_is_the_unmargined_plan`; no full-suite pass is claimed.
- A broader `test_vertical*.py` run executed 209 tests with 16 failures and 13 errors. Inspected failures include missing ignored validation traces, pinned source-hash mismatches and preexisting suspension API assertions. Corresponding runtime/model/test files are unchanged from the branch base. These were not repaired as tire-GUI scope.
- Local TTC compatibility samples/objectives matched the requested MATLAB fitData. A new full two-stage fit ran, but reached iteration limits/bounds and was **not applied** or called a validated fit.
- Run 7 is identified as transient by the reference notes. Its data contains elapsed time, not a calendar acquisition date. The 27 automatically screened local candidate events remained unapproved and failed current speed/stability eligibility. No measured transient fit or identified longitudinal relaxation is claimed.
- Manual visual inspection was unavailable because the configured model could not consume screenshots. Render/launch/widget execution is not visual approval.
- Final whole-branch review found two Important integration issues; both were fixed and passed the one scoped final re-review. No Critical/Important findings remained.
- Nonblocking limitations: only one candidate is extracted per uninterrupted fragment; condition excursions are safely rejected rather than automatically partitioned; trail-lead/joint-trail recovery is not established by the synthetic test battery; no-data Fy/Fx/Mz plots show current curves but omit seed overlays (combined-slip shows seed). Candidate comparison with measured data is available.

## Rulings made during execution

1. **Use an isolated execution branch/worktree rather than the concurrently edited planning checkout.** This protects unrelated changes. Cost if wrong: a different branch/path and an explicit later integration step.
2. **Extend immutable records with provenance/session metadata and use a separate monotonic GUI generation token.** This preserves provenance and rejects stale selections/history results. Cost if wrong: modest API/adapter expansion.
3. **Measure baseline duration to the transition boundary, retaining exclusive baseline-stop indexing.** Twenty 0.01-second pre-event samples represent a 0.2-second baseline despite the last sample being at 0.19 seconds. Cost if wrong: protocol-specific rework.
4. **Permit explicit steady-only loading of duplicate elapsed timestamps without removing rows; keep transient time validation strict.** The upstream steady fit does not use time and relies on original indices. Cost if wrong: extra loader-mode/UI plumbing; replay independently checks continuity.
5. **Correct pressure screening to SI Pa: 82.7 kPa becomes 82700 Pa, and the 3 kPa gate becomes 3000 Pa.** The loader already converts pressure to SI. Cost if wrong: synthetic helper/display conversion rework, not source-data changes.
6. **Defer the minor model-only seed-overlay gap after the final scoped review.** Both original Important issues are fixed; no second final fix wave was added. Cost: missing seed comparison on three no-data plots until follow-up; measured-data candidate comparison remains available.

## Review record

Each of the seven tasks received an implementation and spec/quality review gate. Task 6 required two fix/review rounds for selector/privacy wiring and legacy session holdout semantics. The final whole-branch review covered the complete feature diff; its consolidated fix preserved fit metadata and enabled pre-apply candidate curves. Temporary agent reports/review packages were scratch artifacts; this handoff is the curated record.
