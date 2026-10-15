# Task 4 report: archive unsupported report and recheck comparisons

## Status

Task 4 completed. The old 146.55 s forward-driver report could not be regenerated into a compliant MIS 2024 report from its attempt data, so it was preserved verbatim apart from a prominent obsolete/nonvalidated archive label and moved outside `docs/validation/**/report.md` to `archive/attempt_1792006853120683000-report-obsolete-nonvalidated.md`.

## Old report disposition

The attempt directory contains its existing trace, comparison inputs, and plots, but the report itself identifies `tools/make_forward_driver_report.py` as “proposed” and lists it as the report generator; that tool is absent. Its content has only four comparison signal rows (speed, longitudinal acceleration, lateral acceleration, yaw rate), omitting the standard's motor torque request and pack terminal voltage rows in the metric table, and embeds `comparison.png` and `simulated_lap.png` but no `lap_spread.png`. Existing attempt data therefore do not demonstrate a complete six-channel metric dataset and the missing required figure cannot be reliably regenerated from the documented generator. No missing values were fabricated. The attempt's other artifacts were left in place.

## Fresh comparison regeneration

Ran `tools/compare_iem24_three_way.py` with copied source inputs into the fresh directory `docs/validation/iem24-murphy/three_way_recheck_2026-10-15/`. This regenerated the three-way CSV, PNG, JSON summary, and Markdown summary from immutable copies of measured, point-mass, and 7DoF inputs. The resulting report labels the 7DoF result as an unconverged illustrative iterate, not a valid lap time.

CSV SHA-256 values match the existing verified baseline files exactly:

| Input | Baseline SHA-256 | Fresh copy SHA-256 | Match |
|---|---|---|---|
| `measured_stint.csv` | `795d37a904302f712bab1fa1809dbd838c4cb9c0db3e60c9215561487d6c3d2b` | same | yes |
| `simulated_lap.csv` | `b5ecd09443688a5bc0fcc5adafbeb6fa749113690f0855638cb9d166d70048a5` | same | yes |
| `simulated_lap_7dof.csv` | `957e9a2e6baacd65c1f479bccf23713ea3a14c1377b7dd1236f0e7241e04bbec` | same | yes |

The point-mass run is marked converged (`77.5612538 s`). The 7DoF run is not converged: `Maximum_Iterations_Exceeded` at 8,000 iterations, `converged=false`; its 35.1854814 s iterate is illustrative only and `7dof_metrics_valid=false`. It is not presented as evidence of a valid lap. These inputs comprise the current saved 1 m / 1018-node run; prior unsupported OCP data remain historical/unverified as separately documented by Task 3.

## Tests

- `PYTHONPATH=src python3 -m unittest tests.test_mis_report_standard -q` — **passed**, 10 tests.
- An earlier concurrent full-suite run reported 564 tests with one failure in the Task 1 regen-speed regression; the isolated test and module passed on rerun.
- Final `PYTHONPATH=src python3 -m unittest discover -s tests -q` — **passed**, 566 tests in 151.969 s, exit code 0.
- Final full discovery emitted a non-failing `ResourceWarning` about an unclosed temporary `diag.jsonl` file.
- `git diff --check` — clean at review.

## Intended Task 4 files

- Archived, labeled old report under `.superpowers/sdd/2026-10-15-iem24-verification-fixes/archive/`.
- Fresh comparison outputs and copied CSV/JSON inputs under `docs/validation/iem24-murphy/three_way_recheck_2026-10-15/`.
- This report.

Existing reports and attempt artifacts were not overwritten. No commit was created by this subtask; the working tree contains unrelated/other-task changes and Task 4 files remain uncommitted pending parent coordination.
