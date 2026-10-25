# Task 9 report — simulation GUI lifecycle integration

## Scope

Added `tools/maneuver_gui/qt/controller.py`, a simulation-only `ManeuverController` that owns a single shared `ProcessJob`. It snapshots a schema-validated maneuver definition to a temporary JSON file, launches `python -m tools.maneuver_gui.runner`, forwards JSON progress lines, supports cancellation, and restores the tab on completed, cancelled, or failed terminal states.

`MainWindow` now includes a **Maneuvers** tab alongside **Single Run**. Opening or changing a vehicle model seeds the maneuver tab with that selected formulation. Maneuvers have no track prerequisite. MainWindow reserves the existing `_run_mode` while a maneuver is active, so Single Run and Study actions are excluded; Stop delegates to the maneuver controller, and close cancels then waits a bounded period for the controller before allowing the window to close.

No hardware command path, physical backend, in-process simulation thread, or animation exporter integration was added.

## Verification

- `QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_controller -q`: **2 passed**.
- Focused MainWindow tab/no-track/model-sharing and busy lifecycle tests: **2 passed**.
- `QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_*.py' -q`: **97 passed**.
- `PYTHONPATH=src:. python3 -m tools.maneuver_gui.runner --help`: passed.
- `git diff --check`: passed.

The existing complete `test_sim_gui_qt.py` suite was started but exceeded the focused 30-second command window without diagnostic output; this report does not claim a full-suite green result. The focused integration cases above are the Task 9 evidence.
