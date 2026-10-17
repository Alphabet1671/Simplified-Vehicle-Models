# Task report — four-way MIS 2024 comparison report (measured vs point mass vs 7DoF vs 14DoF)

Plan / request: produce a NEW four-way MIS 2024 report (`docs/validation/iem24-murphy/four_way_latest_model/report.md`)
comparing on-car measured data with point-mass, planar 7DoF and vertical 14DoF-simple
simulations, all run fresh with the current checked-in model files in the worktree
`.worktrees/14dof-simple` (branch `feature/14dof-simple`, HEAD `81034a2`).
Not committed, merged or pushed; no subagents; no full suite.

Date: 2026-10-16. Working directory: `/Users/alphabet1671/Dev/Simplified-Vehicle-Models/.worktrees/14dof-simple`.

## 1. Commands run

Everything is reproducible from the report's own `Reproduce` section; the exact
commands used here were:

```bash
OUT=docs/validation/iem24-murphy/four_way_latest_model

# (a) point mass — energy-coupled minimum-time flying lap, canonical track,
#     library call into the existing runner (writes runs/point_mass/)
PYTHONPATH=src:/tmp/casadi_pkgs python3 -c "
from pathlib import Path
from tools.run.run_iem24_point_mass_track_pair import export_run
from tools.reports.compare_planar_point_mass import FORMULATION, _point_mass_vehicle
from simplified_vehicle_models.compiler.compile import compile_formulation
planar = compile_formulation(str(FORMULATION)).model
vehicle, _ = _point_mass_vehicle(planar)
export_run(vehicle, Path('models/tracks/2024_michigan_endurance_multilap_closed.xml'), Path('$OUT/runs/point_mass'), 0.5)"

# (b) planar 7DoF forward driver (settings verbatim from seven_dof_track_pair/report.md:135, track + output root changed)
PYTHONPATH=src:. python3 -u tools/run/run_iem24_forward_driver.py --model planar-7dof \
  --track models/tracks/2024_michigan_endurance_multilap_closed.xml --planner live --maximum-speed 60.0 \
  --pedal-travel-time 0.175 --maximum-step 0.02 --rtol 1e-05 --method BDF --spacing 0.5 \
  --duration 300.0 --timeout-seconds 300.0 --output-root $OUT/runs/seven_dof

# (c) vertical 14DoF-simple forward driver (Task 9 acceptance settings, canonical track)
PYTHONPATH=src:. python3 -u tools/run/run_iem24_forward_driver.py --model 14dof-simple \
  --track models/tracks/2024_michigan_endurance_multilap_closed.xml --planner live --maximum-speed 60.0 \
  --pedal-travel-time 0.175 --maximum-step 0.005 --rtol 1e-05 --method BDF --spacing 0.5 \
  --duration 300.0 --timeout-seconds 1800.0 --output-root $OUT/runs/vertical_14dof

# (d) channel export (new model-agnostic module), output into each attempt dir
PYTHONPATH=src:. python3 tools/reports/export_forward_attempt_channels.py "$SEVEN"    --output "$SEVEN"
PYTHONPATH=src:. python3 tools/reports/export_forward_attempt_channels.py "$VERTICAL" --output "$VERTICAL"

# (e) report + figures + supporting tables
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/matplotlib-config python3 tools/reports/compare_iem24_four_way.py \
  --point-mass-dir $OUT/runs/point_mass --seven-dof-attempt "$SEVEN" \
  --vertical-14dof-attempt "$VERTICAL" --output-dir $OUT
```

Runs (b) and (c) were launched with `nohup ... &` under a 1800 s / 300 s parent
watchdog and polled at ≤5 min intervals; no tight polling loops.

## 2. Run results

| Model | Attempt / run dir | Track | Lap time (s) | Distance (m) | Mean speed (m/s) | Gates | Accepted steps | Wall (s) | Peak RSS |
|---|---|---|---:|---:|---:|---|---:|---:|---:|
| Point mass | `runs/point_mass` | canonical | **59.620457** | 1009.195015 | 16.927 | `converged` + `energy_converged` = PASS (3 profile, 3 energy iterations) | 2020 nodes | 90.76 (solve) | not recorded by that runner |
| Planar 7DoF | `runs/seven_dof/attempt_1792196348296421000` | canonical | **64.993128** | 1009.195015 | 15.528 | `path_tracking_valid`=FALSE (0.3339 m), `periodicity_valid`=FALSE (509×), `between_step`=FALSE, `refinement`=FALSE ⇒ **`usable`=FALSE** | 11429 | 102.13 | 394.5 MB |
| Vertical 14DoF | `runs/vertical_14dof/attempt_1792196233966416000` | canonical | **65.062853** | 1009.195015 | 15.511 | `path_tracking_valid`=FALSE (0.4202 m), `periodicity_valid`=FALSE, `between_step`=FALSE, `refinement`=FALSE ⇒ **`usable`=FALSE** | 20174 | 145.28 | 1.660 GB |

Measured reference: median of 10 flying laps **71.685013 s** over **1022.40 m**
(14.262 m/s), source `~/Desktop/0615CompEndurance.mat`, SHA-256
`091e4745a5ff6ccf7d48729154033bb555edd028f0e3e2ed2f3f270e14f6e5b7`, 334513192 bytes.

Headline metric (speed RMSE vs the folded measured median): point mass
**3.019 m/s**, planar 7DoF **1.813 m/s**, vertical 14DoF **1.801 m/s**.

**The finding the parent needs:** the 14DoF and the planar 7DoF differ by
**0.070 s** in lap time (0.11 %) and **0.035 m/s** speed RMSE against each other,
while each is about **6.7 s** (and the point mass 12.1 s) faster than the measured
lap. Adding suspension load transfer, four-corner vertical dynamics and the
transient tire layer does not move the answer; the gap to the measurement sits in
the shared grip/powertrain priors, not in vertical fidelity.

## 3. Rulings

- **Ruling: all three models were re-run fresh on the canonical track; no existing
  attempt or report number is reused** — the only existing 14DoF attempt and the
  corrected 7DoF reference attempt are on the *legacy* `2024_michigan_endurance_closed.xml`,
  and the 7DoF reference attempt's recorded `run_iem24_forward_driver.py` hash does
  not match the current runner (disclosed in the Task 9 ledger), so a legacy
  secondary table would have needed fresh runs too and is not "cheap". Cost if
  wrong: ~5 min of extra compute and no legacy-track cross-check, but no reused
  number can be stale.
- **Ruling: palette** — measured stays `#1f4e79` (median) / `#9dc3e6` (band); the
  standard's single simulation colour `#c00000` goes to the **vertical 14DoF**
  (the latest, richest model); planar 7DoF keeps `#ed7d31` (its colour in the
  existing three-way and paired-track reports); point mass gets `#70ad47`
  (green). Cost if wrong: the point mass is red in `point_mass_track_pair/report.md`,
  so cross-report colour identity for that one model is broken; every report's own
  legend stays self-consistent and the departure is stated in the first paragraph.
- **Ruling: per-state channel exports live inside the attempt directories and one
  additive `.gitignore` line (`docs/validation/**/attempt_*/*_channels.csv`) ignores
  them**, while `*_channels_summary.json` stays tracked — mirroring the existing
  convention that `simulated_lap_7dof.csv` is ignored but `simulated_lap_7dof_summary.json`
  is not. Cost if wrong: one extra ignore line for review; without it ~9 MB of
  generated per-state CSV would enter the commit.
- **Ruling: pack terminal voltage for both forward models is the plant's own
  terminal-voltage closure (`ForwardPowertrain.voltage`) at each accepted state and
  command** — the same method for both — instead of the bus-power + SOC
  reconstruction used by `report_iem24_seven_dof_track_pair.py`. Cost if wrong: the
  7DoF voltage row does not reproduce the superseded report's reconstruction method
  (sub-volt difference); the report states the method and does not mix the two.
- **Ruling: point-mass SOC is reconstructed by integrating `I = P/V`** from its
  exported bus power and terminal voltage, and cross-checked against the solver's
  recorded final SOC (0.14 % of the SOC drop, tolerance 2 %). Cost if wrong: the
  point-mass SOC panel is an approximation; it is labelled as reconstructed and the
  check value is in `comparison_summary.json`.
- **Ruling: the metric table keeps absolute pack voltage** (the convention of the
  paired-track report) and quantifies the offset separately in the same bullet by
  re-zeroing each trace at its own lap start (three-way tool convention) and by
  de-meaning. Cost if wrong: the ~23 V voltage RMSE reads as model error to a reader
  who skips the offset sentence.
- **Ruling: the cumulative-time gap uses a new monotone lap-clock fold
  (`fold_lap_clock`) rather than the profile channels' periodic interpolation.**
  A synthetic test caught the periodic folder wrapping the clock to zero across the
  seam (last-bin gap −11.33 s instead of the true −12.08 s). Cost if wrong: none
  known; the six profile channels still use the repository's periodic fold.
- **Ruling: `export_iem24_forward_correlation.py` was not modified**; instead of a
  golden-output file the test suite freezes its source SHA-256 and asserts the new
  module does not import it, plus it verifies the new module's channel columns cover
  the report channels. Cost if wrong: a deliberate edit to the established exporter
  trips one test that must be updated consciously.

## 4. Deviations from the report standard

`mis-2024-report-standard.md` assumes one simulation. The departure is stated in the
report's first paragraph, as the standard requires, and is limited to:

1. three distinguishable simulation colours instead of one (documented above);
2. one metric table with a `Model` column (18 rows: 6 channels × 3 models) instead of
   one row per channel;
3. a fourth figure `time_gap.png` (cumulative time gap + speed vs elapsed time) and a
   model-vs-model speed table.

Sections, required channels, required columns (`Bias (sim − meas)` etc.), the three
required figures, signs, units, the lap table, the filtering statement and all six
mandatory disclosures are unchanged.

## 5. Verification performed

- **Tests** (no `.mat` needed by any of them; the tool's MAT use is hash-only and both
  branches of its provenance sentence are unit-tested):
  - `tests/test_iem24_four_way.py` (new, 31 tests) — PASS.
  - `tests.test_mis_report_standard` (10), `tests.test_repository_layout` (9),
    `tests.test_iem24_three_way` (1), `tests.test_iem24_seven_dof_track_pair` (17)
    and `tests.test_iem24_four_way` (31) together: **68 tests OK** in 0.797 s.
  - `tests.test_forward_driver_runner` (40 tests) — OK.
  - Full suite deliberately not run (parent runs it).
- **Byte-for-byte regeneration:** the tool was run twice into
  `/tmp/fw_a` and `/tmp/fw_b` (`diff -r` → identical) and twice in place in the
  final directory; the SHA-256 of all seven outputs (`report.md`, `comparison.png`,
  `lap_spread.png`, `simulated_lap.png`, `time_gap.png`, `comparison_summary.json`,
  `four_way_channels.csv`) was identical after the second run
  (`BYTE-FOR-BYTE REGENERATION OK`). Only the standard link inside `report.md` is
  path-dependent, by design (it is relative to the report's own directory).
- **`git diff --check`** — clean.
- **`git status --short -uall`** — only `M .gitignore` plus new files; **no existing
  report.md, attempt directory, figure, CSV or JSON under `docs/validation/` was
  modified**, and the untracked plan `docs/superpowers/plans/2026-10-16-vertical-14dof-model.md`
  is preserved untouched.
- **Independent sanity checks** (recomputed from raw traces / the extraction record,
  not through the new tool's helpers):
  1. measured median flying lap time from `telemetry_extraction.json` =
     **71.685013 s** (report 71.685) — agree;
  2. raw `trace.csv` last time / `s` = 7DoF **64.993128 s** over **1009.195015 m**,
     14DoF **65.062853 s** over **1009.195015 m** (report identical); the 14DoF
     exported channels match the raw trace bitwise (`max|Δv| = max|Δyaw| = max|Δs| = max|Δt| = 0`);
  3. point-mass `summary.json` lap time 59.620456 s = its trace clock end (report identical);
  4. an independently written projection + fold + 1 Hz filter gives 14DoF speed
     RMSE **1.802 m/s** / bias **+1.2886 m/s** against the folded measured median,
     versus the report's **1.8012 / +1.2874** (difference from a coarser 0.5 m
     translation search in the check, not a tool error).
- **Point-mass legacy cross-check (context only):** the superseded
  `point_mass_track_pair` run recorded 71.958 s on the *same* ten-lap geometry
  (identical track-CSV hash `2db5a981…`); the current-vehicle optimum is 59.620 s.
  The pre-fix vehicle was not re-run, so this is attributed by inspection of the
  recorded inputs to commit `1e1346a` (track width 1.016 → 1.240 m, front mass
  fraction 55 % → 45 %), not by a controlled experiment. This is disclosed in the
  report's Caveats.

## 6. What a commit would include, with sizes

Ignored (not tracked): `runs/**/source_snapshot/`, `runs/**/trace.csv`, `pedal.csv`,
`run.log`, `*.npz`, `*_channels.csv` — the heavy artifacts (7.6 GB of traces + 1.7 GB
peak RSS runs produced ~57 MB of on-disk trace/pedal/npz/CSV data, all ignored).

Tracked candidates (total ≈ 4.4 MB):

| Bytes | Path |
|---:|---|
| 1314677 | `docs/validation/iem24-murphy/four_way_latest_model/comparison.png` |
| 859808 | `.../simulated_lap.png` |
| 636972 | `.../four_way_channels.csv` |
| 608769 | `.../runs/point_mass/simulated_lap.csv` (precedent: the same file is tracked in `point_mass_track_pair/`) |
| 605095 | `.../lap_spread.png` |
| 183649 | `.../time_gap.png` |
| 83027 | `tools/reports/compare_iem24_four_way.py` |
| 33591 | `.../report.md` |
| 18287 | `tests/test_iem24_four_way.py` |
| 17370 | `tools/reports/export_forward_attempt_channels.py` |
| 11629 | `.../comparison_summary.json` |
| 9757 | `.../runs/vertical_14dof/attempt_1792196233966416000/summary.json` |
| 7110 | `.../runs/seven_dof/attempt_1792196348296421000/summary.json` |
| 1809/1802 | the two `*_channels_summary.json` |
| 1270/1248 | the two attempt `parent.json` |
| 1126/1104 | `runs/{vertical_14dof,seven_dof}/launch.log` (nohup stdout) |
| 1065 | `.../runs/point_mass/summary.json` |
| 1032 | `.gitignore` (one added pattern) |
| 1017 | `.../runs/point_mass/run.log` |

## 7. Open issues

1. **Both forward laps fail gates**: `path_tracking_valid` = FALSE (0.334 m / 0.420 m
   against the 0.25 m gate), `periodicity_valid` = FALSE, between-step and refinement
   validation not established ⇒ `usable` = FALSE for both. No tuning was attempted
   (parent's ruling); the report says so plainly and claims no validation.
2. **The 14DoF-vs-7DoF difference (0.070 s) is smaller than the numerical step
   difference** (0.005 s vs 0.02 s maximum step) and both laps are non-periodic
   transients, so the report cannot claim the 14DoF is the more accurate model — only
   that the two agree closely and both underestimate the measured lap by ~6.7 s.
3. **Grip prior dominates**: the 2/3-of-lab friction prior is unvalidated and the
   leading suspect for all three models beating the measured median. A road-grip
   measurement (or a re-fit) is the highest-value next step.
4. **Transient tire coefficients remain unfitted** (`1.0 / 1.0` m length scales,
   trail pole/lead `1.0 / 0.25`); a fitting tool does not exist in the repository.
   The 14DoF-vs-7DoF comparison therefore cannot separate suspension load transfer
   from tire relaxation.
5. **Pitch inertia radius D5 unvalidated** ⇒ undamped pitch mode 7.30 Hz, outside the
   plan's 3.4–4.7 Hz band; disclosed in the report's Caveats per the ledger ruling.
6. **No legacy-track secondary table** (would need three fresh legacy runs; not
   cheap, and no model failed on the canonical track).
7. `runs/*/launch.log` and `runs/point_mass/run.log` are untracked console captures,
   not gitignored; they are tiny (≈1 KB each) and could be deleted before a commit.
