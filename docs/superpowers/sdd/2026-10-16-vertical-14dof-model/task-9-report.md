# Task 9 report — vertical 14DoF-simple lap run, validation gates and report

## Status

Task 9 is complete: the acceptance lap and both paired runs finished inside the chosen 14DoF wall
budget, the seven plan gates are recorded with measured evidence, and the parameter document and master
plan/provenance updates are written. Nothing is committed (the parent commits after review). Files:

| File | Change |
|---|---|
| `tools/gates/check_14dof_simple_gates.py` | NEW — the Task 9 post-run gate tool: evaluates the plan's seven gates from four finished attempts, writes the seven plan-named booleans (`plan_gates`), each gate's measured evidence, the four trace sha256 pins and the source-hash agreements into `gates.json` |
| `tests/test_vertical_14dof_evidence.py` | NEW — 28 tests: the gate table and its `plan_gates` block, the gate↔runner-key mapping, evidence presence/non-nullity, source-hash and reference-provenance agreement, trace-pinned re-derivation of the refinement/periodicity/cross-fidelity numbers, and unit tests of the matched-distance comparison |
| `docs/validation/14dof-simple-parameters.md` | NEW — parameter table with sources, reference numbers, transient tire equations + DOI, the measured gate table, both cross-fidelity sampling grids, the full PROPOSED list, and the disclosures (D1–D17) |
| `docs/validation/iem24-murphy/14dof_simple_trials/attempt_1792190392697144000/` | NEW — cold acceptance attempt: `summary.json`, `parent.json`, `gates.json`, `cross_fidelity_speed.csv` tracked; `trace.csv`, `pedal.csv`, `dense_polynomials.npz`, `run.log`, `source_snapshot/` gitignored |
| `docs/validation/iem24-murphy/14dof_simple_trials/periodicity/attempt_1792190828124776000/` | NEW — warm-started periodicity baseline |
| `docs/validation/iem24-murphy/14dof_simple_trials/refinement/attempt_1792190567221803000/` | NEW — warm-started refinement (`maximum_step` 0.0025 s, `rtol` 1e-6) |
| `docs/project-master-plan.md` | §3.4 names the two rungs and the 38-state plant; §5 records the implemented LUT schema; §15 Milestone 4 splits `14dof-simple` (delivered) from `14dof-complete` (next) |
| `docs/model-data/provenance.md` | NEW section: the Task 9 lap evidence, that no data file changed, and the fitting-tool requirement |

No model, solver, runner or existing evidence file was modified. `tests/`, `tools/gates/` and `docs/`
additions only, plus one added test method in the existing `tests/test_forward_driver_runner.py` (the
periodic-tolerance tier pin; no existing assertion was changed). The untracked plan
`docs/superpowers/plans/2026-10-16-vertical-14dof-model.md` is preserved. No MIS report generator was run
and no `report.md` was created under `14dof_simple_trials/`, so the `docs/validation/**/report.md`
scanners are unaffected.

## Commands and counts

| Command | Result |
|---|---|
| `PYTHONPATH=src:. python3 tools/run/run_iem24_forward_driver.py --model 14dof-simple --track models/tracks/2024_michigan_endurance_closed.xml --planner live --maximum-speed 60.0 --pedal-travel-time 0.175 --maximum-step 0.005 --rtol 1e-05 --method BDF --spacing 0.5 --duration 300.0 --timeout-seconds 1800.0 --output-root docs/validation/iem24-murphy/14dof_simple_trials` (background, `nohup`) | `status=Completed`, `lap_completed=true`, full lap, child wall **138.03 s** ≤ 1800 s |
| the same command with `--maximum-step 0.0025 --rtol 1e-6 --warm-start <cold attempt> --output-root .../refinement` | `Completed`, lap 66.58945439 s, child wall 260.69 s |
| the same command with `--maximum-step 0.005 --rtol 1e-05 --warm-start <cold attempt> --output-root .../periodicity` | `Completed`, lap 66.58945362 s, `periodicity_valid=true`, child wall 146.30 s |
| `PYTHONPATH=src:. python3 tools/gates/check_14dof_simple_gates.py --attempt <cold> --periodicity <warm> --refinement <refined> --planar-reference docs/validation/iem24-murphy/seven_dof_track_pair/original/attempt_1792187768241214000` | 7/7 gates PASS, wrote `gates.json` (incl. `plan_gates`, trace pins, source-hash agreements) + `cross_fidelity_speed.csv` |
| `PYTHONPATH=src:. python3 -m unittest tests.test_vertical_14dof_evidence -q` | **28 tests OK** (1.0 s) |
| `PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver_runner -q` | **40 tests OK** (2 new: the 36-name periodic-tolerance tier pin is one of them) |
| `PYTHONPATH=src:. python3 -m unittest tests.test_vertical_14dof_evidence tests.test_forward_driver_runner tests.test_repository_layout tests.test_mis_report_standard tests.test_mis_regression tests.test_retired_ocp_evidence tests.test_iem24_multilap_track tests.test_iem24_point_mass_track_pair tests.test_iem24_trace_comparison -q` | **123 tests OK** (review-fix round) |
| `/tmp/task9/mutations_review_fix.py` (4 mutations, files restored byte-exact) | all 4 caught |
| `git diff --check` | clean |
| Full suite | intentionally not run (the parent's job) |

The 1800 s guard was **not** needed and no longer 14DoF budget was chosen: the cold lap needed 138.0 s of
child wall time (125.8 s of solver wall against a 1764 s solver budget). The 7DoF 300 s limit was not
touched (`--model planar-7dof` still rejects `--timeout-seconds 301`).

## Measured numbers

### Cold acceptance lap

| Quantity | Value |
|---|---|
| Lap time / distance | **66.56339750 s** / 1016.70064959 m |
| Mean / min / max speed | 15.110 / 8.850 / 22.702 m/s |
| Accepted steps / solver evaluations / plant evaluations | 20 075 / 74 486 / 114 638 |
| Rejected domain trials | 0 |
| Solver wall / setup / child wall | 125.811 s / 10.888 s / 138.025 s |
| Requested / budget / parent | 1800 s / 1764 s / 1800 s (`within_wall_limit` true) |
| Peak child RSS | 1 651 490 816 B (1.65 GB) |
| Minimum tire load / lifted-wheel evaluations | 97.887 N / **0** |
| Accepted jounce (nodes + dense bounds) | −5.8023 to +7.1369 mm of ±31.8 mm |
| Max \|roll\| / \|pitch\| | 1.193° / 0.650° |
| Max damper velocity | 0.09253 m/s of a 0.5 m/s table (81.5 % headroom) |
| `out_of_range_evaluations` | **0** (damper, spring, bump-stop, jounce-steer) |
| Max lateral path error | 0.38302 m (`path_tracking_valid=false`) |
| Max spatial step / dense midpoint samples | 0.1132 m / 20 075 |
| `dense_defect_scaled_max` | 88.78 (diagnostic, not a bound) |
| Final SOC / planned lap time | 0.805380 / 61.113 s |
| Tire states: \|kappa_eff\| / deformation / trail | 0.557 / 39.5 mm / 0.1576 rad |

### Paired runs

| Quantity | Periodicity baseline | Refinement |
|---|---|---|
| Settings | `maximum_step` 0.005 s, `rtol` 1e-5 | `maximum_step` 0.0025 s, `rtol` 1e-6 |
| Lap time | 66.58945362 s | 66.58945439 s |
| Accepted steps / evaluations | 19 949 / 74 425 | 38 744 / 140 635 |
| Solver / child wall | 134.52 s / 146.30 s | 247.79 s / 260.69 s |
| Peak child RSS | 1.65 GB | **3.08 GB** |
| `periodicity_valid` | **true** | true |
| Minimum tire load / out-of-range / travel | 97.9 N / 0 / valid | 97.9 N / 0 / valid |

## Gate results (honest per gate)

The plan's gate table is evaluated by `tools/gates/check_14dof_simple_gates.py` and recorded in
`.../attempt_1792190392697144000/gates.json`. The runner's own `usable` flag is **false** on all three
attempts (path tracking, un-certified between-step validation); the gate table does not override it.

| Plan gate | Verdict | Evidence |
|---|---|---|
| `completed` | **PASS** | 66.56339750 s lap; `lap_completed=true`, `status=Completed`; child wall 138.025 s ≤ 1800 s requested; 0 rejected domain trials |
| `contact_valid` | **PASS** | minimum tire load 97.887 N ≥ 0; 0 lifted-wheel evaluations; `out_of_range_evaluations` 0; travel valid |
| `travel_valid` | **PASS** | accepted jounce −5.8023 to +7.1369 mm inside ±31.8 mm; **0 bump-stop entries** (onset 27.8 mm); peak wheel force 1083.95 N, minimum wheel force 48.52 N; scope = accepted nodes + per-interval dense bounds |
| `table_domain_valid` | **PASS** | `out_of_range_evaluations = 0` over every evaluation including rejected trials; max damper velocity 0.09253 m/s of 0.5; max \|roll\| 1.193° < the 5° RC table limit (clip never bound) |
| `refinement_validated` | **PASS** | Δ lap time **7.65e-07 s ≤ 5 ms**; max \|Δv\| at 1017 matched s **9.36e-05 m/s ≤ 0.05**; worst matched tire state `kappa_eff_rl` ratio **0.0367 ≤ 1**; identical initial states and source hashes; refined `maximum_step` 0.0025 s, `rtol` 1e-6 |
| `periodicity_valid` | **PASS** | warm lap closure ratio **0.01098 ≤ 1** (36/36 periodic states; worst `n` = 5.49e-06 m vs its 5e-4 m tolerance); plant's own tolerance vector, incl. tire states |
| `cross_fidelity_reported` | **PASS** (report item) | 14DoF 66.56339750 s vs corrected 7DoF 66.53234595 s (+0.03105 s); tracked 200-sample grid speed RMSE 0.032862 m/s and max \|Δv\| 0.207638 m/s at s = 326.98 m (fine 2001-sample grid: 0.033866 m/s and 0.257727 m/s at s = 452.43 m); reference `usable=false` recorded |

Mapping used (the runner's names differ from the plan's): `travel_valid` → `jounce_travel_valid`;
`table_domain_valid` → `out_of_range_evaluations == 0`; `completed` → `lap_completed` + `status` + elapsed
≤ requested; `refinement_validated` / `cross_fidelity_reported` → the new gate tool;
`periodicity_valid`, `contact_valid` → same-named summary keys. The mapping is written into
`gates.json.gate_source_mapping` and asserted by the evidence test.

## Cross-fidelity comparison (expected effects per plan)

| Quantity | 14DoF-simple | corrected 7DoF |
|---|---|---|
| Lap time | 66.56339750 s | 66.53234595 s |
| Accepted steps / evaluations | 20 075 / 74 486 | 11 586 / 36 460 |
| Solver wall / child wall / peak RSS | 125.81 s / 138.03 s / 1.65 GB | 89.50 s / 100.11 s / 0.399 GB |
| Minimum contact load | 97.887 N | 131.377 N |
| Max lateral error / `path_tracking_valid` | 0.38302 m / **false** | 0.36569 m / **false** |
| `usable` | false | false |

Speed trace on two explicitly labelled matched-distance grids, both recorded in `gates.json`:

| Grid | Samples | Speed RMSE | max \|Δv\| | max at | max \|Δv\| after s = 508 m |
|---|---|---|---|---|---|
| Tracked `cross_fidelity_speed.csv` | 200 | 0.032862 m/s | **0.207638 m/s** | **s = 326.98 m** | 0.014226 m/s |
| Finer in-memory grid | 2001 | 0.033866 m/s | **0.257727 m/s** | **s = 452.43 m** | 0.015673 m/s |

Mean signed difference −0.0017 m/s; the difference is concentrated in the first half (finer grid: max
0.2577 m/s over s = 254–508 m) and stays below 0.016 m/s after s = 508 m on both grids. Every number is
quoted with its own sampling density — pairing the 0.2076 m/s extreme with s = 452 m, or 0.2577 m/s with
s = 327 m, would mix the two grids. Full trace in
`.../attempt_1792190392697144000/cross_fidelity_speed.csv`; both grids in `gates.json`.

The two laps also ran with different maximum steps (14DoF 0.005 s, 7DoF 0.02 s), so the first-half
difference is partly integration resolution alongside the different line and start.

The corrected 7DoF reference was produced by the **Task 8 runner before its final review fixes**: its
recorded runner sha256 `4a51d3d5…` matches no commit, while its other 13 recorded sources match the working
tree. `gates.json.planar_reference_source_hash_agreement` records the single mismatch and no gate fails on
it (the fixes were behaviour-preserving for the planar path; the pre-change planar fixture regression still
passes). Re-running the reference would create a new attempt directory at about 100 s of child wall time
plus the exporter, and is deliberately not done.

The plan's expected effects — the 0.473 rigid-tire front LLTD and measured roll-centre geometry against
the planar model's fixed 0.42, plus the tire and damper transients the planar quasi-steady tire lacks —
are consistent in sign and size with a small, first-half-weighted difference, but **this comparison cannot
separate them from the different driven line**: both laps are non-periodic driven transients whose lateral
path errors differ (0.383 m vs 0.366 m), so the 0.031 s lap-time difference mixes model structure with a
different line and start. Nothing was tuned.

## Rulings

Ruling: the plan's seven gates are evaluated by a **post-run** tool, not by the runner — `tools/gates/check_14dof_simple_gates.py` — which reads the three 14DoF attempts and the corrected 7DoF reference and writes `gates.json` into the cold attempt. The plan asks for each gate "as a boolean in `summary.json`", but four of its seven names are not runner summary keys (`completed`, `travel_valid`, `table_domain_valid`, `cross_fidelity_reported`), and `refinement_validated` needs a second integration the runner cannot perform, so the runner's summary cannot carry the plan's table. The gate tool therefore writes the seven plan-named booleans as a `plan_gates` block (with `plan_gate_note` explaining the name gap) and the evidence test checks that block against the per-gate `pass` flags; the runner's summary is not edited. Cost if wrong: a consumer that reads only `summary.json` sees the runner's own names (`lap_completed`, `jounce_travel_valid`, `out_of_range_evaluations`, `refinement_validated=false`) and must read `gates.json` for the plan's table; the file is committed, 33 KB, and named in `docs/validation/14dof-simple-parameters.md`.

Ruling: the **refinement oracle pair is the warm baseline vs the refinement**, not the cold lap vs the refinement, following the 7DoF protocol: both warm runs start from the cold lap's endpoint, so their initial states are identical (`np.array_equal` asserted) and only the step/rtol differ. Cost if wrong: comparing the cold lap with the refinement would measure the initial-condition difference instead of the discretization — the cold/warm lap times differ by 26.06 ms, 5.2× the 5 ms criterion, so the choice is load-bearing and is recorded in the gate evidence (`reference_attempt` is the periodicity attempt).

Ruling: `tools/gates/check_iem24_forward_refinement.py` is **not** used for the vertical pair. Its column indices (1..12) and limits are the planar 13-column layout; on a 39-column vertical trace it would compare `v_z`, `heave` and `roll` as if they were `v_y`, `yaw_rate` and `steering`, and would silently pass or fail on the wrong signals. The plan's Task 9 criteria (5 ms, 0.05 m/s, tire-state tolerances) are implemented in the new gate tool, and the planar checker is untouched. Cost if wrong: two refinement checkers exist, one per rung; a future 39-column user must not point the planar one at a vertical trace, which the tool docstring and this report both state.

Ruling: bump-stop **entry** is `|jounce| > 27.8 mm` — the C1 blend start recorded in the corner LUT metadata (`onset_before_limit_m` 0.002 m, 10× rate from 29.8 mm) — and entry is **reported**, not failed; the ±31.8 mm travel limit stays the failure bound. Cost if wrong: a different onset definition (for example 29.8 mm) would report a different entry count; on this lap the count is 0 either way (max jounce 7.14 mm).

Ruling: the roll-centre part of `table_domain_valid` is evidenced by the measured maximum \|roll\| (1.193°) against the 5° table limit, because the evaluator clips the RC query to 5° silently and publishes no counter. Cost if wrong: if a later run exceeds 5° roll the gate's RC evidence changes meaning ("clip active") and needs an explicit out-of-range counter in the evaluator.

Ruling: the paired attempts live in `14dof_simple_trials/refinement/` and `14dof_simple_trials/periodicity/` subdirectories (the 7DoF `closure_acceptance` / `refinement_acceptance` precedent), so the newest `14dof_simple_trials/attempt_*` is unambiguously the acceptance lap. Cost if wrong: a future run written directly under the trials root makes the table stale relative to the newest attempt.

Ruling (review fix 4): the evidence test selects the newest attempt that **carries a `gates.json`**, and when the newest attempt has none it fails with one message naming the gate-tool command (`... --attempt <cold attempt> ...`) instead of raising a raw `FileNotFoundError` from a missing file or silently passing on an older table. Cost if wrong: a reader who adds a lap without re-running the gate tool sees a failing test with the exact command rather than a confusing error or a false pass.

Ruling: the gate tool records the post-run refinement verdict in the **refined attempt's** `summary.json` (`refinement_validated`, `refinement_evidence`, `refinement_reference_attempt`, recomputed `usable`), exactly as the planar checker does for its rung; the cold and periodicity summaries are not touched. Cost if wrong: the refined attempt's `usable` was recomputed with the runner's gate, which still requires `between_step_validation_complete`, so it stays false; a consumer wanting the vertical verdict must read `gates.json`.

Ruling: `cross_fidelity_reported` is a report item, and its `pass` means "the comparison was computed and recorded", not that either lap is valid. The gate evidence records the reference's `usable=false`, `path_tracking_valid=false` and 0.366 m error, and the evidence test asserts them. Cost if wrong: a reader could mistake a passing report item for a validity claim; the test's assertion and the report's wording are the guard.

Ruling: the gate tool's bump-stop peak force is the compiled spring curve (which already contains the bump stop) evaluated at the accepted extreme, and the minimum wheel force from the droop side of the same curve; both are LUT-derived, not measured. Cost if wrong: if the suspension LUT is regenerated, these two numbers follow the new curve.

Ruling (review fix 1): the identity of all four traces is recorded in `gates.json` (`traces`: path, present, sha256, bytes, rows, columns), and the evidence test re-derives the refinement metrics, the periodicity closure and the cross-fidelity speed trace from those traces with the gate tool's own functions, comparing them with the recorded table. A gitignored trace that is absent, or whose digest no longer matches its pin, makes the re-derivation skip with a message naming what to regenerate; a trace that is *present* with a different digest fails `test_present_traces_match_their_pinned_sha256`, because that is tampering or a stale pin rather than a fresh checkout. Cost if wrong: a reviewer's claim about the refinement, periodicity or cross-fidelity numbers can be reproduced or shown stale, at the cost of a hard failure on a trace that was changed after the gate table was written (re-run the gate tool to re-pin).

Ruling (review fix 1, skip-versus-fail): the review asked for a clean skip when a trace is absent *or* its hash differs. Absence is skipped as asked; a differing hash is split — the re-derivation tests skip, and a small provenance test fails — so the mutation the review used to show the gap (perturbing a refined trace) is caught instead of being silently skipped. Cost if wrong: a stale pin fails loudly rather than skipping, which is the intended fail-closed behaviour and is documented in the parameters doc §7.

Ruling (review fix 5): the corrected 7DoF reference's recorded source hashes are checked like the cold attempt's and recorded as `planar_reference_source_hash_agreement`; one of its 14 sources (the runner, `4a51d3d5…`) differs from the working tree because the reference was produced before Task 8's final review fixes. This is a visible provenance field, never a gate failure — the fixes were behaviour-preserving for the planar path (the pre-change planar fixture regression passes unchanged), and re-running the reference is deliberately out of scope. Cost if wrong: the reference's runner hash does not match any commit, so a reader must treat the reference as a pre-fix-round artifact; the parameters doc §4 and D2 say exactly that.

Ruling (review fix 3): every cross-fidelity speed statistic is recorded twice, once on the tracked 200-sample CSV grid and once on a finer 2001-sample grid, each with the distance of its own extreme (`speed_difference_maximum_distance_m` = 326.98 m at 200 samples, `fine_grid_speed_difference_maximum_distance_m` = 452.43 m at 2001), so a document can quote either density without mixing them. Cost if wrong: a reader who quotes one number against the other grid's location is comparing two sampling densities; the parameters doc and this report label both.

Ruling: the large per-attempt artifacts (`trace.csv` 19.7 MB cold / 38.3 MB refined, `dense_polynomials.npz` 36.4 MB / 73.5 MB, `pedal.csv` 1.0 MB / 1.9 MB, `run.log`, `source_snapshot/`) stay **gitignored** per the repository convention, so a fresh checkout carries `summary.json`, `parent.json`, `gates.json` and `cross_fidelity_speed.csv` but must regenerate traces to re-run the trace-based comparisons. Cost if wrong: reproducing `gates.json`'s trace-derived numbers needs the attempts regenerated (the 7DoF run6 report made the same trade-off).

Ruling: the evidence test **fails** (does not skip) when the gate table or an attempt is missing — the one exception is the trace-pinned re-derivation tests, which skip when their gitignored input is absent — because the gate table is part of the Task 9 deliverable. Cost if wrong: a checkout without the evidence fails the suite loudly instead of passing vacuously, which is the intended fail-closed behavior.

Ruling: source-hash agreement is checked against the **working files** for both the cold attempt (23 recorded sources, all matching) and the reference (14 sources, one runner mismatch), and Task 9 changed none of the hashed sources — the new tool, tests and documents are not in either hash list, and the recorded hash of `tools/run/run_iem24_forward_driver.py` (23-source list) still matches. Cost if wrong: any later edit to a hashed solver/model file invalidates the recorded provenance and the evidence test will say so.

## Deviations from the plan

1. **The plan's gate names are not `summary.json` keys, so the table lives in `gates.json`.** The plan's Task 9 table says each gate "is a boolean in `summary.json`". `summary.json` has `contact_valid` and `periodicity_valid` under those names and `refinement_validated` hard-coded to `false`, but no key at all for `completed`, `travel_valid`, `table_domain_valid` or `cross_fidelity_reported`: the runner records `lap_completed`, `jounce_travel_valid` and `out_of_range_evaluations`, which are different names and, for the table-domain gate, a count rather than a boolean. A post-run gate tool (`tools/gates/check_14dof_simple_gates.py`) therefore evaluates the plan's table and writes the seven plan-named booleans as `plan_gates` in `gates.json`, together with each gate's measured evidence. The runner was not changed to add the missing keys (editing `summary.json` after the fact is not a runner behaviour either).
2. **Gate-name mapping.** `travel_valid` is read from the runner's `jounce_travel_valid`, and `table_domain_valid` from `out_of_range_evaluations == 0`; the plan's `completed` is `lap_completed` + `status` + wall budget. The mapping is explicit in three places (tool, test, this report).
3. **Paired-attempt layout.** Refinement and periodicity attempts are in subdirectories of the trials root instead of the same root (see the ruling).
4. **Refinement reference.** The refinement is compared with the warm baseline, not the cold lap (see the ruling).
5. **Two new committed artifacts per acceptance attempt:** `gates.json` (33 KB after the review-fix round) and `cross_fidelity_speed.csv` (9.9 KB, unchanged by it), neither of which matches an existing `.gitignore` rule.
6. **No MIS/validation `report.md`** was generated: Task 9 asks for a parameter/gate document, and a `report.md` under `docs/validation/` would have to satisfy the MIS 2024 report standard. The document is `docs/validation/14dof-simple-parameters.md`.
7. **Two tool self-checks found and fixed bugs before the table was accepted.** (a) The gate tool's trace-derived jounce extreme is cross-checked against the runner's accepted extremes in the evidence test; that check caught a state-index-versus-trace-column mismatch in the bump-stop evidence (the CSV has `time` as column 0, the dense-bound array does not). (b) `gates.json` repeats the value of `refinement_validated` it leaves in the refined attempt's `summary.json`, and the table is written **after** that update (with `runner_key_origin` explaining that the runner always writes `false`); writing the pre-update value would have disagreed with the file on a first run over fresh attempts — it agreed only because an earlier run had already certified them. Re-running over a certified attempt is idempotent, and the committed table was regenerated from a simulated fresh summary to prove first-run consistency. The committed table is the corrected one.
8. **No source change was needed.** Tasks 6–8 code blocked nothing: the runner already provided `jounce_travel_valid`, `out_of_range_evaluations`, the accepted-vs-trial travel scope, the plant periodic tolerances, the vertical model provenance and the requested/elapsed wall and peak-memory fields. No defect fix was made to Tasks 6–8 code.

## Open issues

1. **Every attempt is `usable=false`.** The acceptance lap fails the 0.25 m path-tracking cap (0.383 m) and
   `between_step_validation_complete` is never certified; the corrected 7DoF reference fails the same
   0.25 m cap (0.366 m) and is non-periodic. The observed path error is a property of the unmodified
   preview driver on this faster corrected car, on both rungs. No driver, plan or model parameter was
   tuned. Task 9 claims numerical gate evidence, not an accepted lap.
2. **The refinement evidence is about the warm-start pair.** Both warm runs start from the cold lap's
   endpoint, so the 5 ms / 0.05 m/s / tire-state criteria quantify the step and tolerance sensitivity
   along that trajectory; they do **not** bound the cold lap's own global discretization error. This
   follows the 7DoF protocol, and it is a limit of the protocol, stated here rather than hidden.
3. **Peak memory grows with the step.** The refinement at 0.0025 s used 3.08 GB (cold 1.65 GB); a next
   level (0.00125 s) would need roughly 6 GB and about 8–9 minutes of wall time. The 1800 s budget is not
   the binding constraint; memory is.
4. **Pitch mode 7.2989 Hz is outside the plan's 3.4–4.7 Hz band** (D5 radii 0.386 m / 0.193 m are
   unvalidated; the roll mode 9.8955 Hz is un-gated). No tuning; the Task 6 tests pin the deviation as a
   visible failure of the band.
5. **The transient tire layer is unfitted** (all four coefficients, the contact window, the longitudinal
   law; the paper text was not re-verified). On this lap the transient states reach large values
   (\|kappa_eff\| 0.557 under braking, trail angle 0.158 rad, carcass deformation 39.5 mm), so the braking
   and high-slip phases are where the provisional law is least trustworthy. A transient tire model
   fitting tool is required and is not built.
6. **The wheels never lift on this lap** (0 lifted-wheel evaluations, minimum load 97.9 N), so the tire's
   contact-loss and re-contact path is not exercised by the acceptance evidence; it is covered only by
   unit tests. The travel gate's ±31.8 mm limit is also far away (max 7.14 mm, 22 %), as is the damper
   table (max 0.0925 m/s of 0.5) and the roll-centre 5° clip (max 1.19°).
7. **`contact_valid` is a weak gate** for this plant: the vertical tire load is nonnegative by
   construction, so its content is the table-domain counter, the travel bound and the accepted-state
   replay, and it cannot localize a corner.
8. **A new attempt written directly under `14dof_simple_trials/` would make the gate table stale.** The
   evidence test now selects the newest attempt **carrying a `gates.json`** and fails with the gate-tool
   command when the newest attempt has none (see the ruling); the subdirectory layout for the paired runs
   keeps the current set unambiguous.
9. **The cross-fidelity comparison is between two non-accepted driven transients** with different maximum
   steps (0.005 s vs 0.02 s), so the 0.031 s lap difference and the 0.033 m/s speed RMSE are not a clean,
   step-converged model-to-model comparison (see the deviations and the parameters document §4 and D16).
10. **The 7DoF reference numbers were not re-run**, only read from
    `seven_dof_track_pair/original/attempt_1792187768241214000` (the Task 8 Step 5 attempt, produced by the
    pre-review-fix runner; disclosed as D2).

## Review fix round

Independent Task 9 review: **approved with fixes**, gate table reproduced byte-for-byte, provenance and
parameter spot-checks matching. Six items were addressed, editing only the gate tool, the two tests, the
parameters document, this report and the two tracked attempt artifacts (the latter solely by re-running the
gate tool). No lap was re-run, no new attempt was created, and the runner's `summary.json` inputs were not
edited by hand.

| # | Finding | Fix |
|---|---|---|
| 1 | The refinement and periodicity gates had no independent test: perturbing the refined trace `v_x` or zeroing the periodicity `periodic_state_delta` left the suite passing | `gates.json` now records the sha256 (plus size/rows/columns) of all four traces, and `PinnedTraceEvidenceTests` re-derives Δlap, max \|Δv\|, the worst tire-state ratio (via the gate tool's `matched_metrics`), the 36-state closure (against `plant.periodic_tolerances` and the recorded delta) and the tracked cross-fidelity CSV from the traces, all compared with the recorded table. Absent or re-pinned traces skip with a message; a present trace whose digest differs fails the provenance test. 6 new tests (28 total) |
| 2 | The report attributed a quotation to the plan that the plan does not contain ("may need a small post-processing tool") | The quotation is gone. The real deviation is stated: the plan asks for each gate "as a boolean in `summary.json`", but four of its seven names are not summary keys. The gate tool now writes a `plan_gates` block (the seven plan-named booleans) plus `plan_gate_note` into `gates.json`, the evidence test checks it against the per-gate `pass` flags and asserts which names are absent from `summary.json`, and the runner's summary is untouched |
| 3 | Cross-fidelity numbers mixed two sampling densities (0.207638 m/s is at s = 326.98 m on the tracked 200-sample grid, not 452 m; 0.258 m/s and "≤0.016 m/s after 508 m" came from a finer grid) | `gates.json` now records both grids with each extreme's own distance (200 samples: RMSE 0.032862, max 0.207638 @ 326.98 m, tail 0.014226; 2001 samples: 0.033866, 0.257727 @ 452.43 m, tail 0.015673). The report and parameters doc quote each number with its grid, and the evidence test asserts the two grids exist and are ordered |
| 4 | `newest_attempt()` made a new unannotated attempt a raw `FileNotFoundError` and silently skipped 16 checks | `acceptance_attempt()` selects the newest attempt carrying `gates.json`; no attempt, no annotated attempt, or an unannotated newest attempt each raise a single `AssertionError` naming the gate-tool command and the stale comparison. Documented in the parameters doc §7; covered by the review's own scenario in the mutation checks (M4/M1 paths) |
| 5 | The reference attempt's runner sha256 `4a51d3d5…` matches no commit (Task 8 pre-fix runner), unrecorded | `gates.json.planar_reference_source_hash_agreement` lists all 14 recorded sources with both digests and records the single mismatch; the evidence test asserts exactly that mismatch and that no gate fails on it. Disclosed in the parameters doc §4 and D2 and in this report's cross-fidelity section, with the cost of re-running the reference (about 100 s of child wall time plus the exporter, a new attempt directory) and the statement that it was deliberately not re-run |
| 6 | Missing disclosures: refinement scope, different cross-fidelity integration steps, and the vacuous load clamp / RC-clip proxy; periodic-tolerance vector not pinned by name | Parameters doc gained D15 (refinement bounds the warm pair, not the cold lap's global error), D16 (0.005 s vs 0.02 s), D17 (gitignored traces and the pin/skip policy), an explicit vacuous-clamp statement in D12 (`loads = max(…, 0)` makes a negative load unrepresentable) and the RC-clip proxy in D13. `tests/test_forward_driver_runner.py` gained `test_vertical_periodic_tolerances_follow_the_plan_tiers_by_name`, which classifies all 36 periodic state names into the plan's tiers and pins the vector |

### Mutation checks (review fix round)

Each mutation was applied to a working file, `tests.test_vertical_14dof_evidence` was rerun, and the file was
restored byte-exact (sha256 verified). Both reviewer mutations are now caught, and two further probes cover
the new `plan_gates` and pinned-metric records.

| # | Mutation | Failing test(s) | Restored |
|---|---|---|---|
| M1 | refined trace `v_x[1:] += 0.5` | `test_present_traces_match_their_pinned_sha256` (the re-derivation tests skip with the pinned-hash message) | byte-exact |
| M2 | periodicity `periodic_state_delta` zeroed in `summary.json` | `test_periodicity_closure_is_reproduced_from_the_pinned_trace` | byte-exact |
| M3 | recorded `refinement_validated.evidence.maximum_body_speed_difference_mps` set to 0 | `test_refinement_metrics_are_reproduced_from_the_pinned_traces` | byte-exact |
| M4 | `plan_gates.refinement_validated` flipped to `false` | `test_plan_gates_block_is_the_plan_table_with_its_own_names`, `test_planar_reference_provenance_is_recorded_not_gated` | byte-exact |

The gate tool was re-run once after the changes (same three attempts and reference, no new laps). The
`gates.json` diff against the pre-review table is **additions only** — new top-level keys `plan_gates`,
`plan_gate_note`, `planar_reference_source_hash_agreement`, `traces`, plus the per-source `sources` lists and
the fine-grid/tail cross-fidelity keys — except `gate_tool.sha256`, which must change because the tool
changed. `cross_fidelity_speed.csv` is byte-identical (sha256 `55ece1bd…`) because the tracked grid is
unchanged. Re-running the tool over the now-certified attempts is idempotent (verified by hashing
`gates.json` and the refined summary before and after).

Focused tests after the round: `tests.test_vertical_14dof_evidence` **28 OK**, `tests.test_forward_driver_runner`
**40 OK**, and the nine-module focused set (evidence, runner and the seven `docs/validation` scanners)
**123 tests OK**. `git diff --check` clean; the full suite was not run (the parent's job).

## Git-tracking and size review

`git status --short -uall` in the worktree lists exactly: the two modified documents, the preserved
untracked plan, the task-9 report, `docs/validation/14dof-simple-parameters.md`, the new evidence test, the
new gate tool, and for the three attempts only `summary.json`, `parent.json`, `gates.json` and
`cross_fidelity_speed.csv`. **Nothing over 100 KB would be tracked** (largest: the preserved plan 64 KB,
the gate tool 41 KB, the parameters document 36 KB, the evidence test 36 KB, this report 35 KB,
`gates.json` 33 KB; each attempt summary 9.7–10.1 KB, `cross_fidelity_speed.csv` 9.9 KB, `parent.json`
1.2–1.3 KB). Every heavy artifact is already ignored by the repository's per-attempt rules and stays out of
Git: `dense_polynomials.npz` 36.4 MB (cold) / 73.5 MB (refined), `trace.csv` 19.7 MB / 38.3 MB, `pedal.csv`
1.0 MB / 1.9 MB, `run.log`, `source_snapshot/`.
