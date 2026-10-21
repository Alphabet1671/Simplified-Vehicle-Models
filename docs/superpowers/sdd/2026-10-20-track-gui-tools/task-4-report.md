# Task 4 report: Track GUI channel controls and viewer

## Implemented

- MAT channel discovery runs in a `QThread` worker and displays unit, sample-count and time metadata. The telemetry path is canonicalized and recorded with the displayed choices. If the path changes after discovery, the channel selectors are cleared and rediscovery is required; late worker results are discarded by path and generation token. Discovery failures and empty results also clear the selectors and cannot fall back to legacy defaults.
- Explicit coordinate mode, per-axis scale and unknown-unit controls are copied into immutable `ClosedSettings` build snapshots. The untouched legacy route retains `channel_selection=None` for synthetic Value-only MAT fixtures; explicit settings require a valid discovered selection.
- Multi-lap controls feed the core detector/averager. Core diagnostics and overlap curves are shown, with overlaps painted behind the mean. Build controls are frozen while a worker runs; changing a relevant setting afterward invalidates prior geometry, plots and save/handoff actions. Build results are also discarded if their input token changed. Build and discovery cannot run together, and close waits for either worker without claiming that `quit()` cancels numerical work.
- The viewer accepts existing track files and current design/build geometry. It handles open, closed and unsigned legacy tracks; unsigned tracks show an explicit no-map notice and disable map-dependent sectioning. Reversal preserves the closed start seam through core reversal.
- Cuts can be entered as an editable distance table, generated as N equal parts, snapped from a map click within a pixel hit radius, and removed. Both map and curvature plots mark the cuts. Closed section export uses core `save_sections`, has all targets guarded before writes, and requires overwrite plus a separate generated-file confirmation when applicable.
- Added a running-direction arrow and start marker to the QPainter map. Telemetry controls use a scroll area for ordinary desktop viewport sizes.

## Verification

```text
PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_track_gui_tools_qt tests.test_track_gui_qt tests.test_track_gui_widgets -q
Ran 42 tests — OK

PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tests -p 'test_track_gui*.py' -q
Ran 106 tests — OK

PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen TRACK_GUI_QUIT_AFTER_MS=300 python3 -c 'from tools.track_gui.app import main; raise SystemExit(main([]))'
Exited successfully
```

The Task 4 Qt tests include:

- A synthetic MAT file discovered through the actual async worker, including metadata and default VNorth/VEast selection.
- Selector clearing after a path change, failure clearing, and discard of an out-of-date discovery result.
- An initial discovery that becomes stale after a path change cannot fall back to legacy defaults; a subsequent build is refused with a rediscovery message.
- Settings snapshot, invalid-result clearing and build/discovery close lifecycle.
- A real `QTest.mouseClick` on a 400-node plotted circle that adds the nearest-node cut and updates markers on both plots.
- Actual equal-N, table edit and remove button operations with map and curvature markers.
- A real GUI section export that writes four numbered XML/CSV pairs and reads every XML back through `load_track`.
- Refusal when overwrite is not enabled, and generated-file replacement declined or confirmed before a complete rewrite.
- Actual viewer save of a closed track that reads back through the core loader.

## Limitations and unverified acceptance

- No interactive desktop/manual visual pass was performed. Offscreen smoke verifies startup only.
- Real-file MIS/FSC GUI acceptance was not performed in this GUI task; see the independent core real-data validation for its results.
- Viewer file-load coverage is narrow: signed and unsigned tracks are injected in memory with `set_track(...)`, and the only Qt `load_path` test exercises the failing path. Core `load_track_data` tests do perform actual open, closed and unsigned XML file loads and round trips; a successful GUI `load_path` remains a follow-up.
- The GUI-specific models-folder permission confirmation flow is not directly covered by a new GUI test. Core target guards and existing save tests cover the protected-folder API.
