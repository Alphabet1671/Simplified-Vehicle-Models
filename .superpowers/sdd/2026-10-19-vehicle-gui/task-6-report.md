# Task 6 report

Implemented the PyQt6 vehicle GUI main window and application entry point.

## Files
- `tools/vehicle_gui/qt/main_window.py`: slot tree, parameter table, swaps, compiler verdict, save controls, geometry settings, close confirmation.
- `tools/vehicle_gui/app.py`: QApplication entry point and `VEHICLE_GUI_QUIT_AFTER_MS` hook.
- `tests/test_vehicle_gui_qt.py`: 11 offscreen Qt integration tests with all QMessageBox methods stubbed and restored.

## Test command and result
`MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_qt -v`

Result: **Ran 11 tests in 1.280s — OK**.

## Self-review
- PyQt6 imports are confined to Qt modules, app.py, and Qt tests.
- Save operations use the reviewed save layer; compiler checks rerun after edits and swaps.
- Tests stub and restore `QMessageBox.question`, `warning`, and `information`; no modal dialog is left unstubbed.
- No `src/`, `tools/sim_gui/`, `models/`, or `pyproject` files were modified.

## Commit
Committed as `feat: add the vehicle GUI main window and application`.
