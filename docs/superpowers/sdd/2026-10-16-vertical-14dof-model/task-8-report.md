# Task 8 report — generalize the forward runner and the gate tools

## Status

Implemented and verified with focused tests only (the parent runs the full suite and
commits). Nothing committed. Files:

| File | Change |
|---|---|
| `src/simplified_vehicle_models/solvers/forward_driver.py` | `solve_forward` gains `maximum_wall_seconds` (default 300 s, so the 7DoF ceiling is preserved); finite-positive validation extended to it and `wall_seconds <= maximum_wall_seconds` is enforced |
| `src/simplified_vehicle_models/solvers/planar_forward.py` | `PlanarForwardPlant.state_names` (12 names, the recorded CSV header order) and `periodic_tolerances` (the recorded 10-value vector) added; **nothing else touched** |
| `tools/run/run_iem24_forward_driver.py` | `--model {planar-7dof,14dof-simple}` (default `planar-7dof`); a `ModelSpec` table selects the formulation, the plant class, the hashed source list, the component roles and the wall policy; every fixed index replaced by a plant-metadata lookup; planar dense certificate and load-interval contact conditions are planar-only; 14DoF `contact_valid` from the accepted-trajectory replay (with the trial diagnostics kept separate); `model_provenance` for the 14DoF tire layer; non-finite diagnostics mapped to `None` so a failed child still writes strict JSON; requested/elapsed wall time and peak memory recorded |
| `tools/gates/check_iem24_forward_tube_v2.py` | `NAMES`/`WHEELS`/`STEERING`/`PEDAL` display labels stay frozen, but their indices come from `PlanarForwardPlant.state_names` through an explicit alias table |
| `tools/reports/export_iem24_forward_correlation.py` | Trace width from `1 + plant.state_size`; distance/velocity/wheel columns from the plant layout |
| `tools/reports/plot_iem24_forward_driver_comparison.py` | Replay width from `1 + plant.state_size`; yaw-rate rate column from the plant layout |
| `docs/validation/iem24-murphy/seven_dof_track_pair/tube_v2_protocol.freeze.txt` | Append-only `REVISION 3 (Task 8)` note (13 added lines, 0 removed) with the new v2 tool/test/runner hashes and a recomputed statement that the protocol content, v1 hash and certificate output are unchanged |
| `tests/test_forward_driver_runner.py` | 28 new tests (11 → 39): recorded planar baseline regression, 14DoF layout/evidence, accepted-vs-trial travel gate, early-failure summary, provenance, source-hash coverage, component roles, wall budgets and watchdog, tolerance validation |
| `tests/test_forward_tube_v2.py` | 3 new tests (41 → 44): frozen display names, plant-derived indices, event windows through the derived indices |
| `tests/fixtures/forward_driver_runner_planar_baseline.json` | NEW — header and 51 deterministic summary values recorded from the **pre-change** runner (see "Recording the fixture") |

`planar_7dof.py`, `planar_dense_certificate.py`, `load_interval.py`, `dense_domain.py`,
`vertical_forward.py`, `vertical_14dof.py` and every other model file were **not** touched —
in particular the accepted-vs-trial jounce distinction is computed in the runner from
`result.state` and `result.dense_bounds`, so `vertical_forward.py` needed no change.

## Review fixes (2026-10-16, second round)

| # | Finding | Fix |
|---|---|---|
| 1 | 14DoF child failing before its first `plant.evaluate` left `±inf` jounce diagnostics; `json.dumps(allow_nan=False)` raised inside `finally`, so no `summary.json` was written | `finite_or_none` maps non-finite diagnostics to `None` (mirroring the `minimum_load_jacobian_determinant` guard); reproduced with the real plant before/after; new test `test_14dof_early_failure_still_writes_a_strict_summary` |
| 2 | `jounce_travel_valid` compared all-evaluation extremes (which include legal trials up to limits ± 10 mm) | the gate now reads `accepted_jounce_extremes(result, plant)` = accepted nodes + per-interval dense bounds; all-evaluation extremes remain as `jounce_evaluation_*` diagnostics; `jounce_gate_scope` names the evidence; tests for a trial-only excursion (gate passes), an accepted excursion (gate fails), an interval-bound excursion, and the exact-limit boundary |
| 3 | minor test hardening | real `PlanarForwardPlant.periodic_indices` literal assertion (the M5 survivor), exact intersection + differing-axle-limit travel cases, watchdog truly exercised (parent waits 900 s and records it), `wall_seconds ≈ budget − setup_seconds` within a delta (no hidden 5 s assumption), `lap_completed=False ⇒ contact_valid=False`, boundary case, out-of-range periodic index branch, tolerance array returned as a copy (all listed in the mutation table) |
| 4 | model provenance | `summary["model_provenance"]` for `14dof-simple` (hashed steady/transient tire XMLs + sha256, DOI parsed from the hashed XML, the four compiled coefficients, PROPOSED note); the full parameter table stays Task 9's document |
| 5 | report disclosure | this report now separates the accepted and all-evaluation jounce families, documents every fix, and lists the new open issues |
| 6 | freeze record | append-only `REVISION 3 (Task 8)` block in `tube_v2_protocol.freeze.txt` with the new hashes and a recomputed statement that the protocol content, v1 hash and certificate output are unchanged (13 insertions, 0 deletions) |

## What was built

### 1. `solve_forward` wall ceiling

`wall_seconds` remains the enforced budget; `maximum_wall_seconds` is the ceiling it may
not exceed and defaults to 300 s, so the 7DoF acceptance limit is unchanged
(`solve_forward(..., wall_seconds=301.)` still raises). A rung that may run longer must
raise the ceiling explicitly. Both are wall-clock seconds of the caller's process; the
parent-process timeout is still the caller's job.

### 2. Planar plant metadata

`PlanarForwardPlant.state_names` is exactly the CSV header order the runner wrote before
this task (`s, n, heading_error, v_x, v_y, yaw_rate, omega_fl, omega_fr, omega_rl,
omega_rr, soc, steering`) and `periodic_tolerances` is exactly the recorded
`[.001, .001, .01, .01, .001, .01, .01, .01, .01, .001]`. Tests pin the tolerance vector
and the index list `(1, 2, 3, 4, 5, 6, 7, 8, 9, 11)` against the pre-change literals, and
`validated_periodic_tolerances` returns a **copy** so no caller can mutate the class array.

### 3. Model-driven runner

* `MODELS` maps `planar-7dof` → `iem24_murphy_real_planar_7dof.xml` +
  `PlanarForwardPlant` + the planar source list, and `14dof-simple` →
  `iem24_murphy_real_14dof_simple.xml` + `VerticalForwardPlant` + the vertical source
  list. `--model` defaults to `planar-7dof`.
* Both models compile the same shared assembly
  (`models/vehicles/iem24_murphy_real_planar.xml`); the driver and the live planner read
  the shared planar layer through `getattr(vehicle, "planar", vehicle)`.
* Hashed/snapshotted sources are the runner script, the model's solver modules, the
  track XML + CSV, the vehicle XML, the formulation XML, the steady tire XML, and — for
  14DoF — each extra component role's XML plus every `<data path=…>` table it references
  (transient tire XML, suspension XML + `iem24_murphy_real_corners.npz`, both damper XMLs
  + `multimatic_dssv_vc01.npz`). 14 sources for the planar rung, 23 for the vertical one.
* The trace header is `(",".join(("time",) + plant.state_names))` and every column index
  (distance `s`, lateral error `n`, steering, state of charge, the four wheel speeds) is
  `plant.state_names.index(name)`. Wheel speeds are found by name pattern
  (`omega_*` planar, `wheel_speed_*` vertical) because the plan only authorized
  `state_names`/`periodic_tolerances` on the planar plant.
* The periodicity check uses `validated_periodic_tolerances(plant)` — a copy of the
  plant's own vector, length-checked against `periodic_indices`, index-checked against the
  layout, and rejected when non-finite or negative.
* Wall budget: the parent watchdog keeps the requested `--timeout-seconds`; the solver is
  handed `solver_wall_budget(timeout, model) - setup_elapsed`. Margin =
  `max(fraction × timeout, floor)`: `(0, 5 s)` for `planar-7dof` (bit-identical to the old
  `timeout − setup − 5`), `(2 %, 5 s)` for `14dof-simple`. The child summary records
  `requested_wall_seconds`, `solver_wall_budget_seconds`, `solver_elapsed_seconds`,
  `elapsed_wall_seconds`, and the parent record repeats the request, elapsed time and
  budget.
* Planar-only evidence: `certify_planar_dense_domain` and
  `contact_nonnegativity_conditions` run only for `planar-7dof`. For `14dof-simple` the
  summary records `dense_polynomial_domain_certified`/`dense_polynomial_domain` and
  `contact_nonnegativity_conditions_hold`/`contact_nonnegativity_conditions` as `None`
  ("not applicable") so the key set is model-independent. `dense_polynomials.npz` is
  still written for a BDF 14DoF run (generic BDF dense output).
* `contact_valid` for `14dof-simple` = lap completed **and** `minimum_contact_load ≥ 0`
  **and** `out_of_range_evaluations == 0` **and** jounce inside the travel limits.
  `out_of_range_evaluations` counts damper/spring/bump-stop/jounce-steer table exits over
  every evaluation, including rejected trials, and is therefore stricter than an
  accepted-state-only count.
* **Travel gate scope (review fix 2).** The gate reads the *accepted* trajectory —
  `result.state` at every accepted node plus, when the method produced a verifiable
  interpolant, the per-interval dense bounds that cover every midpoint
  (`accepted_jounce_extremes`, and `jounce_gate_scope` records which evidence was used).
  It deliberately does **not** read `plant.minimum_jounce`/`plant.maximum_jounce`: the
  vertical plant lets a Newton trial reach its travel limits ± 10 mm
  (`JOUNCE_TRIAL_MARGIN`) and records an extreme on *every* evaluation, so a legal trial
  excursion would otherwise fail the gate. The travel interval itself is the intersection
  of the front and rear axle limits (`[-0.0318, 0.0318] m` here), so a state inside it is
  inside every corner's own travel. The all-evaluation extremes are still recorded,
  as clearly named diagnostics: `jounce_evaluation_minimum_m` /
  `jounce_evaluation_maximum_m`. New summary keys: `jounce_travel_limits_m`,
  `jounce_gate_scope`, `jounce_accepted_minimum_m`, `jounce_accepted_maximum_m`,
  `jounce_evaluation_minimum_m`, `jounce_evaluation_maximum_m`, `jounce_travel_valid`,
  `out_of_range_evaluations`, and (finally block) `maximum_damper_velocity`.
* **Strict JSON on an aborted child (review fix 1).** `finite_or_none` maps a non-finite
  diagnostic to `None`, mirroring the existing `minimum_load_jacobian_determinant` guard.
  A 14DoF child that fails before its first successful `plant.evaluate` (for example a
  missing `--warm-start`) used to leave `minimum_jounce = +inf` / `maximum_jounce = -inf`
  and `json.dumps(..., allow_nan=False)` raised inside the `finally` block, so no
  `summary.json` was written and the child exited 1. It now writes
  `status = "Rejected"`, the exception text, `None` for both diagnostics, and exits 2 (the
  runner's "rejected" code), exactly like the planar path.
* **Model provenance (review fix 4).** A 14DoF summary carries `model_provenance`: the
  steady and transient tire XML paths with their recorded sha256, the DOI read from the
  hashed transient XML text (`10.3390/machines10070527`, not a repeated literal), the four
  compiled `TransientTireParameters` values, and a note that they are PROPOSED. The full
  parameter table, equations and reduced-model limits stay Task 9's
  `docs/validation/14dof-simple-parameters.md`.
* Peak memory: `peak_memory_bytes` from `resource.getrusage(RUSAGE_SELF).ru_maxrss`
  (bytes on macOS, kibibytes elsewhere, converted), with `peak_memory_source` documenting
  the unit and the fact that it is a process high-water mark. `None` when `resource` is
  unavailable.

### 4. Gate/report tools

They stay **planar-7DoF replay tools** (each compiles the planar formulation), but their
trace width and the columns they read now come from `Plant.state_names` / `state_size`
instead of `13`, `6:10`, `states[:, 3]`, `rates[:, 5]`. The tube v2 certificate keeps its
frozen component names (`heading`, `w_fl`, …) and the derived indices are exactly the old
literals `(6,7,8,9)`, `11`, `12`; verified by loading the pre-Task-8 module from `HEAD` and
comparing `build()` results on the synthetic three-level and four-level (held-out) fixtures
— **byte-identical** (8 758 bytes each), so no certificate is invalidated. The freeze
record's `REVISION 3` note records that verification and the new hashes (item 6 of the
review).

## Recording the fixture (Step 1)

`tests/fixtures/forward_driver_runner_planar_baseline.json` was recorded **before any
source edit**, from `HEAD = cf213a5`, by
`/tmp/record_forward_driver_runner_fixture.py` (kept out of the tree; its sha256 and
commit are in the fixture's `recorded_from` field, alongside the exact command and the
solver wall budget the pre-change runner passed, 294.87 s). The recorder patched a fake
12-state plant with the recorded layout and a stub `solve_forward` that returns a fixed
two-sample completed lap, then ran `child()` end to end (real track, real plan, real
trace/summary writing). It stored the trace header, the 51 deterministic summary values,
and the list of excluded volatile keys. The committed test reuses the identical fake and
stub in-process, so the comparison is a direct pre/post regression of the planar path.

```bash
PYTHONPATH=src:. python3 /tmp/record_forward_driver_runner_fixture.py   # pre-edit, HEAD cf213a5
# -> tests/fixtures/forward_driver_runner_planar_baseline.json
```

The 51 recorded values include `periodic_state_delta`, `periodic_state_tolerances`,
`steering_peak`, `minimum_wheel_speed`, `final_soc`, `distance_completed`,
`maximum_accepted_spatial_step`, `path_tracking_valid`, `contact_valid`,
`dense_polynomial_domain(_certified)`, `contact_nonnegativity_conditions(_hold)` and
`usable` — i.e. every quantity the runner derives from the state vector. All 51 are
reproduced exactly by the post-change code.

## Commands and counts

| Command | Result |
|---|---|
| `PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver_runner tests.test_forward_tube_v2 -q` before implementing | FAIL (2 errors: missing names / missing `state_names`) |
| `PYTHONPATH=src:. python3 -m unittest tests.test_forward_driver_runner tests.test_forward_tube_v2 tests.test_forward_driver tests.test_planar_forward tests.test_vertical_forward tests.test_powertrain_closure_audit tests.test_forward_driver_plot tests.test_default_track tests.test_iem24_seven_dof_track_pair tests.test_planar_dense_certificate tests.test_vertical_14dof tests.test_wheelspin_lift_and_pedals -q` | **220 tests OK in 50.311 s** (after the review fixes; 209 before them) |
| the same 12 modules plus the 7 modules that scan `docs/validation/**` (`test_mis_report_standard`, `test_mis_regression`, `test_repository_layout`, `test_retired_ocp_evidence`, `test_iem24_multilap_track`, `test_iem24_point_mass_track_pair`, `test_iem24_trace_comparison`) -q | **275 tests OK in 81.728 s** (after the review fixes; 264 before them; the new attempt directory does not disturb the validation-directory scans) |
| 7DoF corrected lap (Step 5, background, `nohup`) | `status=Completed`, `lap_completed=true`, `usable=false` (see below), parent wall 100.25 s ≤ 300 s |
| 14DoF smoke run (conservative planner, 5 s) | `status=Duration_Limit`, 39-column trace, no `error.txt`, child wall 10.28 s |
| `PYTHONPATH=src:. python3 tools/reports/export_iem24_forward_correlation.py <new attempt>` | OK, 11 587 finite rows, monotone distance 0 → 1016.70064959 m |
| `PYTHONPATH=src:. MPLCONFIGDIR=/tmp/matplotlib-config python3 tools/reports/plot_iem24_forward_driver_comparison.py --trace … --comparison original_comparison.csv --attempt-dir <new attempt> --output-dir /tmp/task8_plot` | OK, both figures written to `/tmp` only |
| `PYTHONPATH=src:. python3 /tmp/verify_tube_v2_equivalence.py` (freeze claims) | protocol hash = frozen Rev2 ✓, v1 hash = frozen ✓, 3-level and 4-level certificates byte-identical to `HEAD`'s module ✓ |
| `PYTHONPATH=src:. python3 /tmp/task8_mutations.py` (8 mutations) | all 8 caught; both edited source files restored byte-exact (`sha256` verified) |
| `git diff --check` | clean |
| Full suite | intentionally not run (parent's job) |

Focused-test totals: `test_forward_driver_runner` 39 tests (11 → 28 → 39 after the review
fixes), `test_forward_tube_v2` 44 (41 → 44), the 12 modules that import a touched file 220,
those plus the 7 `docs/validation` scanners 275.

### Mutation checks (review fixes; all restored byte-exact)

The reviewer's four mutations were not recorded in a file I could read, so these are the
four (M1–M4) that probe the new Task 8 gates directly, plus the `periodic_indices` shift
(M5) the review named as a survivor and three fixes-specific probes (M6–M8). Each mutation
was applied to the working file, the focused module was rerun, and the file restored
(`sha256` verified unchanged).

| # | Mutation | Caught by |
|---|---|---|
| M1 | `14dof-simple` `wall_margin_fraction` 0.02 → 0 | `test_margins_per_model`, `test_14dof_fake_plant_records_vertical_evidence`, `test_14dof_900_second_parent_watchdog_is_used_and_recorded` |
| M2 | drop `out_of_range_evaluations == 0` from 14DoF `contact_valid` | `test_14dof_contact_valid_rejects_travel_or_table_excursions` |
| M3 | travel gate reads the all-evaluation jounce extremes | `test_14dof_trial_jounce_excursion_is_a_diagnostic_not_a_gate_failure`, `test_14dof_travel_gate_covers_the_dense_interval_bounds`, `test_14dof_contact_valid_rejects_travel_or_table_excursions` |
| M4 | remove the periodic-tolerance length check | `test_periodic_tolerance_vector_must_match_the_plant_layout` |
| M5 | shift `PlanarForwardPlant.periodic_indices` 11 → 10 | `test_periodic_tolerance_vector_must_match_the_plant_layout` (the only catcher; the tolerance vector still zipped to length 10, which is why the literal assertion was added) |
| M6 | write the raw ±inf jounce diagnostics | `test_14dof_early_failure_still_writes_a_strict_summary` |
| M7 | return the plant tolerance array without copying | `test_returned_tolerances_are_a_copy_of_the_plant_vector` |
| M8 | travel boundary `<=` → `<` | `test_14dof_boundary_jounce_at_the_travel_limit_passes` |

## Measured numbers

### Step 5 — corrected 7DoF lap (Task 1 geometry), new reference

Attempt `docs/validation/iem24-murphy/seven_dof_track_pair/original/attempt_1792187768241214000`
(created by the unmodified report command
`--track models/tracks/2024_michigan_endurance_closed.xml --planner live --maximum-speed 60.0
--pedal-travel-time 0.175 --maximum-step 0.02 --rtol 1e-05 --method BDF --spacing 0.5
--duration 300.0 --timeout-seconds 300.0 --output-root "$OUT/original"`; the runner picked a
fresh timestamped directory, the two pre-existing attempt directories are untouched).

| Quantity | Corrected lap (new) | Superseded `original` attempt |
|---|---|---|
| lap time | **66.53234595425141 s** | 73.56297463565521 s |
| distance / track length | 1016.70064959 m | 1016.70064959 m |
| accepted steps / evaluations | 11 586 / 36 460 | 11 964 / 38 165 |
| rejected domain trials | 0 | 0 |
| minimum contact load | 131.377 N | 9.787 N |
| maximum lateral error | 0.36569 m (**> 0.25 m gate**) | 0.23337 m |
| maximum accepted spatial step | 0.42960 m | (not recorded) |
| dense polynomial domain certified | True | key absent (older runner) |
| contact nonnegativity conditions | True | key absent |
| planned lap time | 61.113 s | 68.288 s |
| final SoC | 0.805344 | 0.809487 |
| solver wall / setup / child wall | 89.504 s / 10.191 s / 100.106 s | – / – / 125.369 s |
| parent wall / budget / timeout | 100.247 s / 295 s / 300 s | – |
| peak child RSS | 398 606 336 B (398.6 MB) | not recorded |
| source hashes | 14 (includes `forward_powertrain.py`) | 13 (pre-Task-7 list) |

Gate outcome: `lap_completed=true`, `contact_valid=true`, `resolution_valid=true`,
`integration_completed=true`, `checked_rhs_physics_valid=true`,
`dense_polynomial_domain_certified=true`, `contact_nonnegativity_conditions_hold=true`;
`path_tracking_valid=**false**` (0.366 m vs the 0.25 m cap), `periodicity_valid=false`
(cold start, never a closed lap), `between_step_validation_complete=false` and
`refinement_validated=false` (no refinement run), so `usable=false`. The corrected car is
faster (planned 61.1 s vs 68.3 s, actual 66.5 s vs 73.6 s) and the unmodified driver
therefore tracks the path worse; nothing was tuned. The lap is the new **reference**, not
an accepted lap.

### 14DoF smoke run

Attempt `/tmp/task8_14dof_smoke/attempt_1792187892202894000`
(`--model 14dof-simple --track …closed.xml --planner conservative --maximum-speed 8.0
--maximum-step 0.005 --rtol 1e-05 --method BDF --spacing 0.5 --duration 5.0
--timeout-seconds 180.0`, output root in `/tmp`):

* `status = Duration_Limit` (expected: 5 s of a lap, not a lap), 5.0 s simulated,
  24.675 m, 1 657 accepted steps, 4 656 evaluations, 0 rejected trials, no `error.txt`.
* 39-column trace with the vertical layout; all values finite and time strictly monotone.
* jounce ∈ [−1.0455, +1.0413] mm, heave ∈ [8.896, 9.519] mm, |roll| ≤ 0.264°,
  |pitch| ≤ 0.114°, v_x ∈ [3.933, 6.523] m/s, |n| ≤ 0.0609 m.
* tire states: |kappa_eff| ≤ 5.33e-3, |lateral_deformation| ≤ 4.12 mm,
  |trail_angle| ≤ 0.0232 rad.
* `minimum_contact_load_n = 495.716`, `out_of_range_evaluations = 0`,
  `jounce_travel_valid = true`, `lifted_wheel_evaluations = 0`,
  `maximum_damper_velocity = 0.0227 m/s`; `contact_valid = false` only because the
  duration-limited run did not complete a lap.
* `dense_polynomial_domain_certified = None`, `contact_nonnegativity_conditions_hold = None`
  (planar-only), `dense_polynomials.npz` written.
* wall: requested 180 s, solver budget 175 s, setup 0.634 s, solver 9.511 s, child
  10.276 s, parent 10.432 s; peak child RSS 210 583 552 B (210.6 MB); 23 source hashes.

### Review-fix re-verification (2026-10-16)

* **Early failure with the real plant** (`--model 14dof-simple --warm-start
  /tmp/does-not-exist --duration 1.0 --timeout-seconds 60.0`, `/tmp/t8_repro2`): before the
  fix the child raised `ValueError: Out of range float values are not JSON compliant: -inf`
  inside `finally`, wrote only `error.txt`/`parent.json`/`run.log` and exited 1. After the
  fix it writes `summary.json` with `status = "Rejected"`,
  `error = "FileNotFoundError: … /tmp/does-not-exist/summary.json"`,
  `jounce_evaluation_minimum_m = jounce_evaluation_maximum_m = None`,
  `out_of_range_evaluations = 0`, no `contact_valid` key, no `Infinity` in the text, and
  exits 2.
* **Accepted-vs-trial gate and provenance with the real plant** (a 2.0 s duration-limited
  14DoF smoke run to `/tmp/t8_smoke2`, `--method BDF --maximum-step 0.005`, not a lap):
  `jounce_gate_scope = "accepted states and the per-interval dense bounds, which cover
  every midpoint"`, accepted jounce `[−1.0455071, +1.0413038] mm` vs all-evaluation
  `[−1.0455049, +1.0413037] mm` (the interval bounds dip ~2 nm below the accepted nodes —
  exactly the extra coverage the gate needs), `jounce_travel_valid = true`,
  `contact_valid = false` (duration limit), `out_of_range_evaluations = 0`,
  `model_provenance.transient_tire_doi = 10.3390/machines10070527` with the recorded
  sha256 equal to `source_hashes[...]`, parameters
  `{longitudinal_length_scale: 1.0, lateral_length_scale: 1.0, trail_pole_ratio: 1.0,
  trail_lead_ratio: 0.25}`, 39 trace columns, `solver_elapsed_seconds = 4.56`,
  `peak_memory_bytes = 146 784 256`.

## Rulings

Ruling: both models' hash lists include `forward_powertrain.py` — the ledger's Task 7 note
required it for 14DoF, and the planar rung must hash it too because
`PlanarForwardPlant.powertrain` now delegates to it, so a planar run's recorded provenance
is incomplete without it. This changes recorded hashes only (planar outputs and the header
are unchanged), and every pre-Task-8 planar attempt is already superseded by the Task 1
geometry. Cost if wrong: hash-set equality with historical planar attempts fails (it
already fails on the corrected geometry), and any external tool that compares a recorded
planar hash dict to a literal 13-key set must accept the extra key.

Ruling: the wall margin is `max(fraction × timeout, floor)` with `(0, 5 s)` for
`planar-7dof` and `(2 %, 5 s)` for `14dof-simple`, applied by one function used for the
child budget and the parent record; the parent watchdog keeps the full requested timeout.
A 900 s parent therefore permits an 882 s solver budget (≥ the 870 s the parent asked for),
and the brief's `1770 s for an 1800 s` example (a flat 30 s) is **not** implemented because
the ledger ruled 2 %. Cost if wrong: if 2 % of a very long 14DoF budget is too small for
setup + replay + reporting, the child raises `TimeoutError` and records `Timed_Out` with
diagnostics preserved (visible, not silent).

Ruling: `maximum_wall_seconds` is the validation ceiling, not the enforced budget; the
runner passes `maximum_wall_seconds=args.timeout_seconds` and
`wall_seconds=solver_wall_budget(...) − setup_elapsed`. Cost if wrong: a caller that
expects `maximum_wall_seconds` to truncate the integration would be surprised; the ceiling
only rejects an over-long request up front.

Ruling: "the same summary keys as before" for `planar-7dof` is implemented as
**superset plus unchanged values** (the fixture asserts that every recorded key still
exists with its recorded value, and that the new Task 8 keys exist). Extra keys are
`model`, `requested_wall_seconds`, `solver_wall_budget_seconds`, `solver_elapsed_seconds`,
`elapsed_wall_seconds`, `peak_memory_bytes`, `peak_memory_source`. Cost if wrong: a
consumer that compares the exact key set (none in the repository; `acceptance_gate` reads
by name) would see the additions.

Ruling: for `14dof-simple` the planar-only evidence keys are recorded as `None` rather
than omitted, so both models publish the same summary key set and a reader can tell
"not applicable" from "missing". Cost if wrong: a consumer that treats `None` as a failed
certificate rather than an inapplicable one; no gate reads these keys.

Ruling: `14dof-simple` `contact_valid` still requires `lap_completed`, matching the planar
gate, so a duration-limited 14DoF run reports `contact_valid=false` even when its load,
table-domain and travel evidence is clean (the smoke run shows exactly this). Cost if
wrong: a Task 9 partial run cannot claim contact validity; the underlying evidence is
still recorded in `minimum_contact_load_n`, `out_of_range_evaluations` and
`jounce_travel_valid`.

Ruling: the 14DoF travel check uses the intersection of the front and rear axle jounce
limits, so a state inside it is inside every corner's own travel. Cost if wrong: the check
is conservative but cannot localize which corner left its travel; Task 9's `travel_valid`
gate needs the same information.

Ruling (review fix 2): the travel gate is computed from the **accepted** trajectory —
`result.state` at every accepted node plus the per-interval dense bounds when the method
produced a verifiable interpolant — and never from the plant's all-evaluation
`minimum_jounce`/`maximum_jounce`, because `VerticalForwardPlant` admits a Newton trial up
to its limits ± 10 mm (`JOUNCE_TRIAL_MARGIN`) and records an extreme on every evaluation.
The all-evaluation extremes stay in the summary as
`jounce_evaluation_minimum_m`/`jounce_evaluation_maximum_m`, and `jounce_gate_scope` states
which evidence backed the gate. This needed no `vertical_forward.py` change: the gate input
is already in `ForwardResult`. Cost if wrong: a legal trial excursion no longer fails the
gate (intended), and on a method without a verifiable interpolant (Radau) midpoints are not
covered — the scope string says so and the Task 9 acceptance run uses BDF, whose dense
bounds do cover them.

Ruling (review fix 1): non-finite diagnostics are mapped to `None` by `finite_or_none`
before the summary is serialized with `allow_nan=False`, mirroring the existing
`minimum_load_jacobian_determinant` guard. `inf`/`-inf` are not JSON-compliant, so an
aborted child previously raised inside `finally`, wrote no `summary.json` and exited 1.
Cost if wrong: a reader must treat `None` as "never measured" (documented in the report and
in the key's guard comment); the alternative (writing `Infinity`) is invalid JSON.

Ruling (review fix 4, provenance): `summary["model_provenance"]` records the machine-readable
essentials for `14dof-simple` — the steady and transient tire XML paths with their recorded
sha256, the DOI parsed from the hashed transient XML text, the four compiled
`TransientTireParameters` values, and a PROPOSED note. The full parameter table, equations,
reduced-model limits and paper discussion stay Task 9's
`docs/validation/14dof-simple-parameters.md` (the plan's Task 9 Step 5 owns them). Cost if
wrong: a consumer wanting the complete provenance in the attempt record must follow the
reference to Task 9's document; the essentials and the exact hashed sources are in the
attempt.

Ruling (review fix 3): `validated_periodic_tolerances` returns a **copy** of the plant's
array, so a caller cannot mutate the class-level metadata. Cost if wrong: a caller that
expected identity with the plant attribute must compare values instead.

Ruling (review fix 6): the freeze record is extended by an append-only `REVISION 3 (Task 8)`
note — no existing bytes are rewritten or removed (13 insertions, 0 deletions, verified) —
because the frozen file's own header says v2 code/test hashes are "appended below when
written (append-only)", and the note states that the protocol content, v1 hash and
certificate output are unchanged, each recomputed (`tube_v2_protocol.md` still
`b0b499b2…`, v1 still `57b92964…`, and the pre-Task-8 module loaded from `HEAD` produces
byte-identical certificates on the synthetic three-level and four-level fixtures). Cost if
wrong: the freeze file grows a clearly dated revision block; readers who treat it as
strictly immutable should read `REVISION 3` as a note that changes nothing above it.

Ruling: the 14DoF source list is the plan's list plus `forward_driver.py` (always hashed),
`forward_powertrain.py`, `unitire_simple.py` (the transient layer's steady force target)
and `tires/vertical.py` (the normal-load law) — the plan's list omits those physics
dependencies. `suspension/components.py` is deliberately **not** hashed (consistent with
the planar list, which hashes no compiler/component module). Cost if wrong: the hash list
is stricter than the plan's and must keep matching the working tree; a change in an
unhashed component module is not detected.

Ruling: component roles are resolved from the vehicle assembly XML and their data tables
from each component XML's `<data path=…>` attribute, so both damper XMLs, the damper NPZ,
the suspension XML/NPZ and the transient tire XML are hashed without hard-coded paths (the
shared damper NPZ appears once after de-duplication). Cost if wrong: a component that
references a table through a different attribute name would not be hashed; the current
damper/suspension/tire components all use `path`.

Ruling: `check_iem24_forward_tube_v2.py` keeps its frozen display names (`heading`,
`w_fl`, …) and only derives the indices from `PlanarForwardPlant.state_names` through an
explicit alias table, with a module-level assertion that the aliases cover the whole
planar layout. Cost if wrong: if a future planar layout renames a state without updating
`ALIASES`, the module raises at import instead of silently mislabeling a certificate; the
certificate JSON keys are unchanged, so existing certificates stay comparable.

Ruling: the export/plot tools stay planar-formulation replays (they compile
`iem24_murphy_real_planar_7dof.xml` and construct `PlanarForwardPlant`), and only their
trace width and column indices become plant-derived. A 14DoF figure/export is Task 9's
job, not a silent widening of the 7DoF correlation tools. Cost if wrong: a 14DoF attempt
handed to the exporter is still rejected (`verify_track_provenance` checks the recorded
formulation), which is the intended fail-closed behavior.

Ruling: Step 5 runs the **closed-track** command from `seven_dof_track_pair/report.md`
(`$OUT/original`), not the multilap one, because Task 9's 14DoF lap uses the same closed
track and the cross-fidelity comparison needs a like-for-like reference; the output root is
the command's own root, and the runner's timestamped `attempt_<ns>` directory guarantees no
overwrite. Cost if wrong: the stored reference is a single-lap 1016.7 m run, which is what
Task 9 compares against; the ten-lap multilap reference would have to be run separately.

Ruling: peak memory is `resource.getrusage(RUSAGE_SELF).ru_maxrss` converted to bytes
(macOS reports bytes, Linux kibibytes), recorded with its unit and meaning in
`peak_memory_source`. Cost if wrong: the number is a whole-child high-water mark including
numpy/SciPy/BLAS and the dense-polynomial bounds, not the solver's own allocation.

## Deviations from the plan

1. **Hash list additions.** `forward_driver.py`, `forward_powertrain.py`,
   `unitire_simple.py` and `tires/vertical.py` are hashed for `14dof-simple` beyond the
   plan's list (see the ruling), and `forward_powertrain.py` is also added to the planar
   list.
2. **Wall-margin arithmetic.** The plan/brief example (`1770 s` for an `1800 s` parent) is
   not implemented; the ledger's 2 % (floor 5 s) ruling is, giving 1764 s for 1800 s and
   882 s for 900 s.
3. **Recorded fixture.** The plan says "compare with a recorded fixture" without naming
   one; a new `tests/fixtures/forward_driver_runner_planar_baseline.json` was recorded from
   the pre-change runner with a fake plant, with the recording script, its sha256 and the
   HEAD commit stored inside the fixture. The script lives in `/tmp` (Task 7 precedent).
4. **Planar-only certificate keys are `None` for 14DoF** instead of being omitted, and
   `dense_polynomials.npz` is still written for a 14DoF BDF run.
5. **Wheel-speed lookup by name pattern.** The plan allowed only `state_names` and
   `periodic_tolerances` on `PlanarForwardPlant`; the two plants name the wheel speeds
   differently (`omega_*` vs `wheel_speed_*`), so the runner resolves them by pattern
   rather than extending the plant API.
6. **`tire_xml_path` gained a `role` argument** and now delegates to the new
   `component_xml`; its default behavior (the steady tire XML) is unchanged.
7. **The `from types import SimpleNamespace` import is back in use** in the edited test
   file (the differing-axle-limits travel test uses it); it was removed and restored within
   this task.
8. **Freeze record updated (review fix 6).** `tube_v2_protocol.freeze.txt` now ends with an
   append-only `REVISION 3 (Task 8)` block listing the new v2 tool/test/runner sha256
   (`66249fc4…`, `fb8783db…`, `df8ada30…`) and stating, with recomputed evidence, that the
   protocol content, v1 tool hash and certificate output are unchanged. The pre-existing
   entries keep their pre-migration paths (`tools/check_iem24_forward_tube_v2.py`) and are
   untouched; 13 lines added, 0 removed. The block is an append, not a rewrite of the frozen
   bytes; the verification command is `/tmp/verify_tube_v2_equivalence.py`.
9. **Smoke run parameters.** The 14DoF plumbing smoke test uses the conservative planner
   and `--duration 5.0` (not the Task 9 command), which is enough to exercise compile →
   plan → plant → `solve_forward` → 39-column trace → summary → parent record. The
   review-fix re-verification used a 2.0 s variant (`/tmp/t8_smoke2`); neither is a lap and
   neither writes under `docs/validation/`.
10. **Fake-plant knob.** The test's vertical fake gained a `jounce_initial` class attribute
    (the jounce written into the accepted states the stub solver returns) so the accepted
    and all-evaluation jounce can be varied independently; the fake numbers used by the
    planar baseline fixture are unchanged, so that fixture comparison still holds.

## Open issues

1. **The corrected 7DoF reference lap fails `path_tracking_valid`** (0.3657 m > 0.25 m), so
   it is `usable=false`; it is still the reference Task 9 compares against. The failure is
   a consequence of the corrected vehicle being faster, not of the runner refactor (the
   runner's tracking quantity is computed from the same `n` column as before). Nothing was
   tuned. `periodicity_valid=false` is expected for a cold, non-closed lap.
2. **Between-step and refinement evidence stays false.** The runner still hard-codes
   `between_step_validation_complete=False`/`refinement_validated=False`; those require the
   separate refinement/between-step tools (Task 9 Steps 4).
3. **14DoF `contact_valid`'s load condition is near-trivial** (the vertical tire load is
   nonnegative by construction); its real content is the table-domain counter and the
   travel bound. It also cannot localize a corner.
4. **14DoF full-lap wall time and memory are still unknown** (plan F17). A 5 s slice used
   9.51 s of solver wall and a 210.6 MB child RSS; the 7DoF lap used 89.5 s and 398.6 MB.
   The 900 s → 882 s budget is available for Task 9 but was not exercised with a real
   14DoF lap.
5. **Every 14DoF run's `out_of_range_evaluations` covers solver trials too**, so the
   `== 0` gate is stricter than an accepted-state-only count; the runner does not separate
   damper from spline exits (the evaluator's `out_of_range_count` and the exception message
   do).
6. **`peak_memory_bytes` is a process high-water mark**, not specific to the solver, and
   `None` where `resource` is unavailable.
7. **The planar source list change invalidates hash-comparison with all pre-Task-8 planar
   attempts** (already stale from Task 1). Warm-start matching is unaffected within a
   newly run set.
8. **The travel gate's midpoint coverage depends on the method.** For BDF the accepted
   nodes are supplemented by the per-interval dense bounds; for Radau (or any method
   without a verifiable interpolant) only the accepted nodes are covered and
   `jounce_gate_scope` says "accepted states only". Task 9's acceptance run uses BDF, so the
   gate there covers the midpoints.
9. **The two jounce key families measure different things and must not be compared
   directly**: `jounce_accepted_*` are gate inputs (accepted nodes + interval bounds) while
   `jounce_evaluation_*` are plant diagnostics over every evaluation, including rejected
   trials inside the ±10 mm trial margin. A visible gap is expected on a stiff or
   under-resolved run and is not itself a failure.
10. **`model_provenance` is a summary of the tire layer, not the full parameter record.**
    The parameter table, the PROPOSED list and the transient-tire fitting-tool requirement
    remain Task 9's `docs/validation/14dof-simple-parameters.md`; the attempt records the
    hashed XMLs, the DOI and the compiled coefficients that point back to it.
