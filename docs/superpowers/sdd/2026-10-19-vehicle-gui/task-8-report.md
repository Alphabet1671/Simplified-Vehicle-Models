# Task 8 report

## Repository-layout regression and resolution

The full-suite comparison against baseline `4af6515` detected three new repository-layout failures: tracked Task 2–7 reports were under ignored `.superpowers/sdd/`, violating the rule that tracked SDD Markdown belongs under `docs/superpowers/sdd/`. The reports were moved with `git mv` to preserve contents and history. The ignored plan/brief and pre-existing untracked plan/spec were left untouched.

Validation:

- `PYTHONPATH=src:. python3 -m unittest tests.test_repository_layout -q`
  - Result: `Ran 9 tests in 0.079s — OK`.
- `MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_xmltext tests.test_vehicle_gui_model tests.test_vehicle_gui_save tests.test_vehicle_gui_table tests.test_vehicle_gui_qt tests.test_vehicle_gui_docs -q`
  - Result: `Ran 59 tests in 1.844s — OK`.
- `git ls-files .superpowers` produced no output; tracked Markdown exists under `docs/superpowers/sdd/`.
- Focused branch GUI test suite (including docs): **Ran 59 tests in 1.844s — OK**.
- GUI smoke test: `VEHICLE_GUI_QUIT_AFTER_MS=300 MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 tools/vehicle_gui/app.py; echo "exit $?"` returned `exit 0` (offscreen event-loop smoke only).
- Model integrity: `git status --short models/` produced no output.
- Full suite, first run before report relocation: **Ran 1701 tests in 480.027s; FAILED (failures=23, errors=16, skipped=4)**. Sorted failure-list comparison against baseline found only three new repository-layout failures described above; the other failure/error names matched baseline.
- After report relocation, rerun full suite: **Ran 1701 tests in 505.728s; FAILED (failures=20, errors=16, skipped=4)**. Sorted failure-list comparison returned **same failures as the baseline** (diff exit 0). The baseline at `4af6515` was **Ran 1619 tests in 480.826s; FAILED (failures=20, errors=16, skipped=4)**; do not characterize it as passing.

## Broad branch review

The independent broad review found one medium robustness issue: `set_model_id()` did not reject an apostrophe, which can break an XML attribute written with single quotes. The reviewer reported no other actionable integration, plan-compliance, guardrail, or documentation findings.

## Apostrophe model-ID fix

Updated `tools/vehicle_gui/xmltext.py:set_model_id()` to reject apostrophes alongside double quotes and other XML-invalid characters. Added regression coverage for rejecting apostrophes and for preserving the existing quote style when updating a single-quoted model ID. Ordinary IDs and existing double-quoted behavior remain unchanged.

Validation:

- `PYTHONPATH=src:. python3 -m unittest tests.test_vehicle_gui_xmltext -q`
  - Result: `Ran 11 tests in 0.026s — OK`.

## Not performed / scope

- Manual desktop interaction (edit `total_mass`, generated-slot read-only check, save and open in simulation GUI) was not performed in this offscreen session. It remains a manual validation limitation.
- Task 8 says to push and open a PR. Per explicit user boundary, no push, publish, merge, or PR was done. This branch remains local.
