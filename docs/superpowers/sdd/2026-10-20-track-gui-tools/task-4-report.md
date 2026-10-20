# Task 4 report: Track GUI channel controls and viewer

## Implemented

- Added explicit MAT channel selectors, metadata display (units, samples, time support), coordinate mode, per-axis scales, and unknown-unit opt-ins. Selected values are copied into immutable `ClosedSettings` snapshots. The untouched legacy route retains `channel_selection=None` for synthetic Value-only MAT fixtures; explicit modes/scales/unknown-unit flags require channel discovery and selection.
- Added multi-lap controls and report output for core diagnostics. Overlap curves appear behind the mean geometry. Build controls are disabled while a worker runs; successful output is invalidated when relevant settings change. Errors clear plots and disable save/handoff. Existing worker close behavior is retained and does not claim that `quit()` cancels numerical work.
- Added a viewer tab that accepts existing track files and transfers current design/build geometry. It supports open/closed/unsigned geometry, reversal, equal/editable distance cuts, map-node click cuts, removal, plot cut markers, and closed section export through `track_ops`. Unsigned legacy files show an explicit no-map warning and disable map-dependent section operations.
- Added pixel-radius node click signals and a visible start/direction arrow. Telemetry controls are in a scroll area for ordinary desktop viewport sizes.

## Verification

```text
PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_track_gui_tools_qt tests.test_track_gui_qt tests.test_track_gui_widgets -q
Ran 32 tests — OK

PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen TRACK_GUI_QUIT_AFTER_MS=300 python3 -c 'from tools.track_gui.app import main; raise SystemExit(main([]))'
Exited successfully

git diff --check -- [GUI-owned source and test paths]
Passed
```

New tests cover channel controls and settings snapshots, legacy-vs-explicit selection behavior, result invalidation, viewer reverse/unsigned state, failed-load clearing, core-writer handoff and refusal of conflicting section targets. Existing closed-circle build/save, design save, startup, and worker-close tests pass.

## Limitations and unverified acceptance

- No interactive desktop/manual visual pass was performed. Offscreen smoke verifies startup only.
- Real MAT metadata discovery and the full MIS/FSC real-file GUI workflow were not exercised in this GUI task. See the independent core report for its acceptance results.
- Automated GUI tests do not yet execute a successful section batch write with overwrite confirmations or a real mouse click mapped to a plotted node. Core section tests cover the geometry/save API; GUI confirmation flows remain review targets.
