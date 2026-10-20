# Task 6 report

Implemented the PyQt6 vehicle GUI main window and application entry point.

## Files
- `tools/vehicle_gui/qt/main_window.py`: slot tree, parameter table, swaps, compiler verdict, save controls, geometry settings, close confirmation.
- `tools/vehicle_gui/app.py`: QApplication entry point and `VEHICLE_GUI_QUIT_AFTER_MS` hook.
- `tests/test_vehicle_gui_qt.py`: readable offscreen Qt integration tests with QMessageBox methods stubbed and restored (11 originally; 13 after review fixes).

## Test command
`MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_qt -v`

Initial implementation result: **Ran 11 tests in 1.280s — OK**.

## Self-review
- PyQt6 imports are confined to Qt modules, app.py, and Qt tests.
- Save operations use the reviewed save layer; compiler checks rerun after edits and swaps.
- Tests stub and restore `QMessageBox.question`, `warning`, and `information`; no modal dialog is left unstubbed.
- No `src/`, `tools/sim_gui/`, `models/`, or `pyproject` files were modified.

## Commits
- `e87bbe8` — `feat: add the vehicle GUI main window and application`
- `75329e3ea45cceaf36595257249ca16ca62b88df` — `test: cover vehicle GUI window persistence and close behavior`
- `2fd0892ba7806ffda831120bba062a3171f7b83e` — `docs: clarify vehicle GUI test report`

## Review fix round 1
Added coverage for QSettings geometry restoration and unsaved-change close rejection, reformatted the Qt tests for maintainability, and disallowed saving the assembly as a component. The report now accurately states that QMessageBox methods are stubbed/restored (not that every method is exercised).

Command:
`MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_vehicle_gui_qt -v`

Result after review fixes: **Ran 13 tests in 1.266s — OK**.

Fix commit: `75329e3ea45cceaf36595257249ca16ca62b88df` — `test: cover vehicle GUI window persistence and close behavior`.

