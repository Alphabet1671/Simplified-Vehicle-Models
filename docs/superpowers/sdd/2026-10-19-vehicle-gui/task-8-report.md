# Task 8 report

## Repository-layout regression and resolution

The full-suite comparison against baseline `4af6515` detected three new repository-layout failures: tracked Task 2–7 reports were under ignored `.superpowers/sdd/`, violating the rule that tracked SDD Markdown belongs under `docs/superpowers/sdd/`. The reports were moved with `git mv` to preserve contents and history. The ignored plan/brief and pre-existing untracked plan/spec were left untouched.

Validation:

- `PYTHONPATH=src:. python3 -m unittest tests.test_repository_layout -q`
  - Result: `Ran 9 tests in 0.079s — OK`.
- `MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_xmltext tests.test_vehicle_gui_model tests.test_vehicle_gui_save tests.test_vehicle_gui_table tests.test_vehicle_gui_qt tests.test_vehicle_gui_docs -q`
  - Result: `Ran 59 tests in 1.844s — OK`.
- `git ls-files .superpowers` produced no output; tracked Markdown exists under `docs/superpowers/sdd/`.
