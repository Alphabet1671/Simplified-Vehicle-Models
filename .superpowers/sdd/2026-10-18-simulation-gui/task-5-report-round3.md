# Task 5 report (round 3)

## Tests
Added `StudyTests.test_parent_dimension_refreshes_children`, `test_dimension_from_sweep_reads_members`, and `test_sweep_that_fits_no_slot_is_refused` in `tests/test_sim_gui_core.py`.

## RED
Initial run: 28 tests, 2 failures. The sweep test exposed that one-at-a-time combinations used index 0 for every dimension in the baseline case, causing a baseline sweep point to be recorded. The parent test also initially expected the unchanged child path incorrectly; the new assembly correctly refreshes both children.

## Fix
Updated `tools/sim_gui/study.py` so one-at-a-time expansion represents the baseline with no selected dimension values, preventing baseline sweep points and preserving baseline choices. No other production defect was found.

## GREEN
`MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_core -v`: **Ran 28 tests in 0.120s — OK**.

## Files
- `tests/test_sim_gui_core.py`
- `tools/sim_gui/study.py`
- `.superpowers/sdd/2026-10-18-simulation-gui/task-5-report-round3.md`

## Fix round

RED was reproduced before the fixes: canonical-path and missing-git regressions were not covered, and
factorial expansion materialized oversized products before enforcing the cap. Added tests for absolute
baseline paths, source-backed factorial parent/model cases, missing git, and early cap checking. Fixed
canonical path comparison for slots and tracks, caught `OSError` around git metadata, and calculated case
counts before building combinations. GREEN: `tests.test_sim_gui_core` ran 31 tests, all passed.
