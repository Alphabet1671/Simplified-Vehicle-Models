# Task 3 report: retired OCP tests and contradictory outputs

## Support policy

The planar OCP and point-mass seed APIs are **retired and unsupported** in the current tree. This task does not restore complex solver APIs. OCP-specific checks are moved outside default unittest discovery while tests for still-present planar parameter/geometry functionality remain active. Historical solver runs and artifacts are retained, but the 20 m / 52-node IEM24 result and dependent three-way claims are historical/unverified because the coarse geometry is invalid per `docs/validation/planar-ocp.md`.

## Changes

- Removed active `test_planar_7dof.py` imports and assertions coupled specifically to deleted OCP wheel-share helpers; retained active planar formulation compilation, parameter, load and geometry tests.
- Moved the retired point-mass seed API test from `tests/test_point_mass_seed.py` to `docs/validation/retired-tests/test_point_mass_seed.py`, preserving it outside default test discovery without deleting it.
- Documented the retired/unsupported policy and labeled all solver results/API descriptions in `docs/validation/planar-ocp.md` as historical; clarified the old 43.9 s result is not a current validated result.
- Marked the IEM24 three-way report and JSON metadata historical/unverified, invalidated OCP metric/convergence claim fields, and annotated the saved prior 20 m summary.
- Marked the archived 2026-10-15 recheck as a nonvalidated historical run at report, comparison-summary, and OCP-run-summary level.
- Marked archived placeholder point-mass-vs-planar report and machine-readable summaries historical/unverified.
- Added `tests/test_retired_ocp_evidence.py` to prevent regression in three-way report/summary status disclosure.

## Tests and results

- An earlier concurrent run of `PYTHONPATH=src python3 -m unittest discover -s tests -q` reported 564 tests with one failure in `test_planar_forward.PlanarForwardTests.test_regen_survives_motor_speed_limit_without_hydraulic_braking`; this did not reproduce in focused reruns.
- Final `PYTHONPATH=src python3 -m unittest discover -s tests -q` — **passed**: 566 tests in 151.969 s, exit code 0. A non-failing `ResourceWarning` for a temporary `diag.jsonl` file was emitted.
- Focused reproduction, `PYTHONPATH=src python3 -m unittest tests.test_planar_forward.PlanarForwardTests.test_regen_survives_motor_speed_limit_without_hydraulic_braking -v` — **passed** (1 test, 0.005 s).
- Module reproduction, `PYTHONPATH=src python3 -m unittest tests.test_planar_forward -v` — **passed** (9 tests, 0.212 s). The one full-suite failure was not reproducible in focused reruns; likely order/state-sensitive. No Task 3 code changes were made to this unrelated forward-driver behavior.
- `python3 -m json.tool` validation passed for edited JSON summaries; `git diff --cached --check` passed before the original commit.

## Diff scope

Task-specific files include:

- `tests/test_planar_7dof.py`, `tests/test_retired_ocp_evidence.py`
- moved test: `tests/test_point_mass_seed.py` → `docs/validation/retired-tests/test_point_mass_seed.py`
- `docs/validation/planar-ocp.md`
- `docs/validation/iem24-murphy/comparison_three_way_report.md`
- `docs/validation/iem24-murphy/comparison_three_way_summary.json`
- `docs/validation/iem24-murphy/mesh_sensitivity/prior_20m_run/simulated_lap_7dof_summary.json`
- `docs/validation/iem24-murphy/three_way_recheck_2026-10-15/comparison_three_way_report.md`
- `docs/validation/iem24-murphy/three_way_recheck_2026-10-15/comparison_three_way_summary.json`
- `docs/validation/iem24-murphy/three_way_recheck_2026-10-15/simulated_lap_7dof_summary.json`
- `docs/validation/point-mass-planar-comparison.md`, `docs/validation/point-mass-planar-comparison.json`, `docs/validation/point-mass-planar-plots/summary.json`
- this report.

Unrelated dirty files were preserved and excluded from the task commit.

## Self-review and concerns

- Result data, traces, and plots remain archived. OCP-derived plots and raw CSV still contain the historical OCP trace; report banners/metadata warn users that they are not current or valid evidence.
- The plan document is now prominently labeled as an archived historical record; internal present-tense references are changed at the opening/API-description sections. Remaining detailed solver narrative is within that archival framing.
- Archived seed tests intentionally no longer execute under standard discovery because the tested API is absent; active regressions verify current support/evidence status instead.
- The earlier suite failure was not reproducible; the final full discovery passed all 566 tests.

## Commit

Initial task commit: `caccda8` (`Retire unsupported planar OCP verification claims`). Follow-up review changes are prepared for a scoped amend.
