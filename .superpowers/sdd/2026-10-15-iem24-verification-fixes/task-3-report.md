# Task 3 report: retired OCP tests and contradictory outputs

## Support policy

The planar OCP and point-mass seed APIs are **retired and unsupported** in the current tree. This task does not restore complex solver APIs. OCP-specific checks are moved outside default unittest discovery while tests for still-present planar parameter/geometry functionality remain active. Historical solver runs and artifacts are retained, but the 20 m / 52-node IEM24 result and dependent three-way claims are labeled historical/unverified because the coarse geometry is invalid per `docs/validation/planar-ocp.md`.

## Changes

- Removed active `test_planar_7dof.py` imports and assertions coupled specifically to deleted OCP wheel-share helpers; retained the active planar formulation compilation, parameter, load and geometry tests.
- Moved the retired point-mass seed API test from `tests/test_point_mass_seed.py` to `docs/validation/retired-tests/test_point_mass_seed.py`, preserving it outside default test discovery without deleting it.
- Documented the explicit retired/unsupported policy in `docs/validation/planar-ocp.md`.
- Marked the IEM24 three-way report and JSON metadata historical/unverified, invalidated OCP metric/convergence claim fields, and annotated the saved prior 20 m summary.
- Marked the archived placeholder point-mass-vs-planar report and machine-readable summaries as historical/unverified.
- Added `tests/test_retired_ocp_evidence.py` to prevent regression in three-way report/summary status disclosure.

## Tests and results

Command: `PYTHONPATH=src python3 -m unittest discover -s tests -q`

Result: the command did not finish within the initial 120-second foreground window and continued as background job `bash-97`; initial output contained a `ResourceWarning` for a temporary `diag.jsonl` file. Final exit status and test totals are pending collection.

## Diff scope

Task-specific files only:

- `tests/test_planar_7dof.py`
- `tests/test_retired_ocp_evidence.py`
- `docs/validation/retired-tests/test_point_mass_seed.py` (archived test relocation)
- `docs/validation/planar-ocp.md`
- `docs/validation/iem24-murphy/comparison_three_way_report.md`
- `docs/validation/iem24-murphy/comparison_three_way_summary.json`
- `docs/validation/iem24-murphy/mesh_sensitivity/prior_20m_run/simulated_lap_7dof_summary.json`
- `docs/validation/point-mass-planar-comparison.md`
- `docs/validation/point-mass-planar-comparison.json`
- `docs/validation/point-mass-planar-plots/summary.json`
- `.superpowers/sdd/2026-10-15-iem24-verification-fixes/task-3-report.md`

Unrelated dirty files were preserved and are excluded from the task commit.

## Self-review and concerns

- The result data, traces and plots were retained rather than deleted. The three-way plot and raw CSV still visually/data-wise include the historical OCP trace; the report explicitly warns against treating them as current evidence and links the archived metrics context. The summary's OCP convergence/metrics flags are invalidated.
- The plan description in `docs/validation/planar-ocp.md` contains broader historical narrative; this change specifically establishes current support status and marks dependent claims unverified without rewriting unrelated validation history.
- Archived seed tests intentionally no longer execute under standard discovery because the tested API is absent; the active suite regression instead verifies the unsupported policy and evidence markings.

## Commit

Pending.
