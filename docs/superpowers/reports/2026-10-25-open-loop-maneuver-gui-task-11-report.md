# Task 11 report — maneuver guide and acceptance evidence

## Deliverables

- Added [`maneuver-gui.md`](../../dev/maneuver-gui.md) with installation, GUI/headless starts, all preset names, SI units and ownership, CSV import, peak-ay calibration, artifact/cancellation semantics, skidpad interpretation, and an explicit no-real-hardware/commissioning boundary.
- Added strict executable examples: `step-steer.maneuver.json` and an irregular three-channel CSV input.
- Added doc smoke tests that schema-validate the JSON, import the CSV via the strict source loader, run runner help, and require every preset plus physical disclaimer wording.
- Linked the maneuver guide from the README and Simulation GUI guide.

## Gate evidence

- Documentation smoke: **3 passed** in **0.079 s**.
- Final maneuver/docs discovery: **100 passed** in **0.956 s**.
- Fake robot contract: **6 passed**.
- Targeted MainWindow maneuver integration: **2 passed** in **0.232 s**.
- Runner `--help` and `git diff --check`: passed.

The full legacy `test_sim_gui_qt.py` discovery was previously started during Task 9 and exceeded a 30-second focused command window without diagnostics; it is not claimed green here. No web server was started, no manual visual inspection is claimed, and no hardware was connected.
