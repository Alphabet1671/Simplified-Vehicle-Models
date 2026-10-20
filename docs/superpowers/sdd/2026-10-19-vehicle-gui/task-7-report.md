# Task 7 report

Added documentation and registration for the reviewed vehicle GUI.

## Files

- `docs/dev/vehicle-gui.md`: user guide covering installation, startup, opening and editing vehicles, compiler checks, component swaps, generated slots, save behavior, tests, and limits.
- `docs/dev/tools-layout.md`: added the `tools/vehicle_gui/` category row linked to the guide.
- `.gitignore`: added `/vehicles_out/` for the GUI's default output folder.
- `tests/test_vehicle_gui_docs.py`: added three documentation and registration checks.

## Checks and commands

Initial red test (expected before implementation):

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_docs -v
```

Result: **Ran 3 tests — FAILED** as expected: missing guide, missing tools-layout row, and missing ignore entry.

Focused documentation check:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_docs -v
```

Result: **Ran 3 tests in 0.000s — OK**.

Vehicle GUI regression checks:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_xmltext tests.test_vehicle_gui_model tests.test_vehicle_gui_save tests.test_vehicle_gui_table tests.test_vehicle_gui_qt -v
```

Result: **Ran 54 tests in 1.850s — OK**.

## Self-review

- Manual source cross-checks confirmed each corrected behavior statement: `_revert_slot` in `tools/vehicle_gui/qt/main_window.py` removes parameter edits only; `render_files` in `tools/vehicle_gui/save.py` always includes the assembly; `save_component` in `tools/vehicle_gui/qt/main_window.py` and `tools/vehicle_gui/save.py` uses the save-bar stem and a separate model id; `protected_directory_problem` in `tools/gui_common/files.py` exempts `models/sweeps/`.
- The guide uses short, one-idea sentences for the reviewed behavior and save warnings.
- Documentation tests are content-presence regressions. They assert corrected statements and reject the exact obsolete claims from the brief (old revert wording, conditional assembly output, model-id/file-name conflation, and unconditional models-folder protection). They do not verify source behavior.
- The source cross-check was manual against `tools/vehicle_gui/qt/main_window.py`, `tools/vehicle_gui/save.py`, and `tools/gui_common/files.py`.
- PyQt6, `src/`, `tools/sim_gui/`, `models/`, and `pyproject` were not modified.
- Pre-existing untracked planning/spec documentation was left untouched.
- The ignored task brief was amended with an implementation-corrections ledger and ruling that source behavior wins where the brief conflicts; it was not staged because it is intentionally ignored.

## Commit

- `docs: add the vehicle GUI guide`

## Review fix round 2

Strengthened the documentation tests with exact obsolete wording checks. These checks are content-presence regressions only. They do not provide automated source verification.

Command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_docs -v
```

Result: **Ran 5 tests in 0.000s — OK**.
