# Task 8 report — maneuver editor, CSV dialog and preview

## Scope

Added the simulation-only Qt maneuver editor package under `tools/maneuver_gui/qt/` and a pure, separately testable draft/preflight model in `tools/sim_gui/qt/maneuver_tab.py`.

- `ManeuverTab` exposes run, stop, and calibration signals; it labels the backend as simulation and permanently disables hardware arming. It supports vehicle configuration, validated definition retrieval, busy state, result/status display, and requested/applied preview input.
- `ManeuverEditor` validates via the maneuver schema and round-trips a typed draft through `definition()` and `load_definition()` without unit conversion.
- `CsvDialog` validates an existing CSV and complete mapping, then returns strict `CsvMapping` and immutable `CsvSource` values via the library loader.
- `PreviewWidget` keeps defensive requested/applied copies, supports `set_trace`, and visibly distinguishes requested from applied traces. It performs no simulation or process execution.
- The pure draft model owns SI-backed presentation conversion, inline schema errors, mode enablement, peak-ay calibration fingerprint/status preflight, CSV hash/mtime/size freshness checks, physical-backend blocking, and requested/applied preview data.

No subprocess orchestration, live hardware control, or physics evaluation is added by this task.

## Verification

- `QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_qt.py' -q`: **6 passed**.
- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_tab.py' -q`: **6 passed**.
- `QT_QPA_PLATFORM=offscreen PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_*.py' -q`: **90 passed**.
- `git diff --check`: passed.

## Fix round 1 — reviewed widget completeness

Bound visible editor fields to the typed draft with preset/mode controls and CSV ownership disabling; connected Tab run requests to calibration/CSV freshness preflight; exposed full strict CSV mapping controls with inline loader failures; and rendered requested/applied SWA, accelerator, and brake traces in three shared-time QPainter panes.

Verification after repair: Qt **8 passed**, draft **6 passed**, CSV **7 passed**, maneuver **92 passed**, and `git diff --check` passed.

## Fix round 2 — mode completeness

Added schema-backed path/radius target controls and additional profile controls. Blank optional numeric fields now create values in the draft. Offscreen regressions exercise every preset, CSV ownership, path/radius target editing, and optional field creation.

Verification after repair: Qt **9 passed**, draft **6 passed**, CSV **7 passed**, maneuver **93 passed**, and `git diff --check` passed.

## Fix round 3 — state and ownership coverage

Added direct state assertions for path/radius target mutual exclusion, locked-SWA and peak-ay target behavior, non-CSV ownership labels, and CSV channel/mode disablement with its ownership label.

Verification after repair: Qt **10 passed**, draft **6 passed**, CSV **7 passed**, maneuver **94 passed**, and `git diff --check` passed.
