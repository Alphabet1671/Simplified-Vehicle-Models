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

- Each corrected behavior statement was checked against current source: `_revert_slot` removes parameter edits only; `render_files` always includes the assembly; component save uses the save-bar stem and a separate model id; `models/sweeps/` is exempt from folder protection.
- The guide uses short, one-idea sentences for the reviewed behavior and save warnings.
- Documentation tests cover the three required discoverability changes and corrected behavior statements without parsing the full document.
- PyQt6, `src/`, `tools/sim_gui/`, `models/`, and `pyproject` were not modified.
- Pre-existing untracked planning/spec documentation was left untouched.
- The ignored task brief was amended with an implementation-corrections ledger and ruling that source behavior wins where the brief conflicts; it was not staged because it is intentionally ignored.

## Commit

- `docs: add the vehicle GUI guide`
