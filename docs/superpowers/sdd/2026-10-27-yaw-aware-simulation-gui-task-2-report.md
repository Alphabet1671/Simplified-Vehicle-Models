# Task 2 Report — GUI solver settings and resolution contract

## Review round 2 fixes

- Kept point-mass settings and source-resolution selection serializable, but execution preflight now rejects point-mass before case materialization with `point-mass adaptation unavailable until yaw GUI runner migration`. Direct child-command construction also rejects point-mass and any legacy `--model` command; no full-model forward runner invocation is possible from this GUI path.
- Added strict JSON deserialization checks for setting types, finite numerics, enum values, `StudyDefinition` required field types, choices/dimensions containers, per-dimension keys and value types.
- Restored install/start/load, component replacement, sweep generation, single-run, output, headless invocation and hand-off guidance while updating solver semantics. Point-mass docs state physical execution is currently preflight-blocked; yaw-aware is limited to 7/14DoF with completed yaw map.
- Review-round tests: 45 focused core and Qt settings/MainWindow tests passed under the 180-second watchdog.

## Implemented

- Replaced forbidden `legacy-forward` naming with the public `point-mass` solver kind; serialization rejects old unprefixed/legacy setting names.
- Added separate point-mass and yaw-aware setting fields, strict basic validation, and explicit yaw-map artifact validation for yaw-aware fixed/line/replay workflows.
- Added `solution_model_for(settings, selected_model)`: point-mass accepts a direct point-mass source or adapts selected 7/14DoF to a point-mass solution; yaw-aware accepts selected 7/14DoF and resolves both to planar-7dof, explicitly including 14DoF→7DoF.
- Persisted source/solution model mapping in the case contract. This resolver is declarative; it does not fake physical compilation/adaptation.
- Kept point-mass case arguments on the existing forward-runner mapping for now; its child dispatch and physical point-mass view adaptation remain Task 3 work.
- Updated the settings UI and user documentation; source/workflow compatibility controls GUI run availability. Direct point-mass source remains available only in point-mass workflow.
- Removed provisional skipped test substitutions; retained existing platform-level PyQt6 availability guards only.

## Verification

Passed under the 180-second process watchdog:

```text
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 180 -- python3 -m unittest tests.test_sim_gui_core tests.test_sim_gui_qt.ModelPanelTests tests.test_sim_gui_qt.SettingsTabTests tests.test_sim_gui_qt.StudyTabTests tests.test_sim_gui_qt.TrackTabTests tests.test_sim_gui_qt.SingleRunTabTests tests.test_sim_gui_qt.CaseQueueTests tests.test_sim_gui_qt.MainWindowTests.test_main_window_point_mass_source_workflow_gate tests.test_sim_gui_qt.MainWindowTests.test_save_open_study_round_trip -q
Ran 69 tests ... OK
```

A broader run including the full MainWindow integration class timed out at the requested 180-second watchdog; it was not restarted. The focused core/Qt suites above passed. No physical yaw/replay case was run.

## Remaining Task 3 interface

Task 3 must consume `solution_model_for` / `case.json.resolution_mapping`, adapt full-model sources to a true point-mass view for point-mass runs, and adapt 14DoF to planar 7DoF for yaw-aware runs. It must implement the yaw-aware child runner; Task 2 intentionally retains current point-mass child mapping.
