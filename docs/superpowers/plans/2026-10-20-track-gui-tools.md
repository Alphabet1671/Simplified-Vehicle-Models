# Track GUI Tools Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use subagent-driven-development to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Extend the track GUI to import varied telemetry, average laps, view and reverse tracks, and export open sections.

**Architecture:** Qt-free modules own MAT synchronization, lap averaging, and track transforms/export. The current design and closed tabs retain their public attributes; a focused viewer tab provides loading, direction reversal, and section cuts. Background workers build telemetry geometry; existing protected atomic save rules apply to all output.

**Tech Stack:** Python, NumPy, optional SciPy, PyQt6, unittest.

**Spec:** [Track GUI tools specification](2026-10-20-track-gui-tools-spec.md).

## Global Constraints

- Keep Qt imports in qt/, app entrypoints and Qt tests.
- Do not modify src/, tools/sim_gui/, tools/anim/, or pyproject.toml.
- Preserve canonical model files. Keep new top-level tool basenames unique.
- Documentation uses ASD-STE100 style.
- Do not commit MAT files. Do not push or publish.
- Preserve overwrite, generated-file confirmation, models protection, and load_track validation.
- Work in feature/track-gui-tools; local main merge is approved after review and tests.

### Task 1: Selectable synchronized MAT channels

**Files:** Create tools/track_gui/mat_channels.py and tests/test_track_gui_channels.py. Other files are not owned by this task.

**Interfaces:** Produce frozen ChannelInfo(name, units, samples, start_time, end_time), ChannelSelection(x_channel='VNorth', y_channel='VEast', mode='velocity', x_scale=1.0, y_scale=1.0), list_channels(path)->list[ChannelInfo], load_coordinates(path, selection, start_time, end_time)->tuple[np.ndarray,np.ndarray,np.ndarray] (time,north,east). Modes: velocity, position, gps. ChannelInfo may expose extra useful metadata with defaults. Load coordinates uses each channel Time; only missing Time uses xtime.Value legacy fallback. Velocity scales combine units conversion and explicit scale; position supports metres and known length conversions; GPS uses degrees and a local tangent-plane approximation. Reject unknown physical units unless an explicit conversion mechanism is provided and documented. Raise actionable ValueError for malformed fields, nonfinite inputs, duplicate/decreasing clocks, no common interval, fewer than two samples, invalid GPS coordinates and degenerate paths. Use selected-variable loads, not the full MAT content for build.

- [ ] Write synthetic savemat tests for mismatched clocks, m/s and km/h, positions, GPS, bad clocks, missing channels, malformed arrays, finite checks and legacy xtime fallback. Example: x.Time=[0,1,2], x.Value=[1,1,1], y.Time=[0,.5,1,1.5,2], y.Value=zeros(5); velocity result ends at north=2 and east=0.
- [ ] Run `PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_channels -q` and capture initial missing-module failure.
- [ ] Implement interfaces, metadata discovery, conversion, synchronization without extrapolation, integration and validation.
- [ ] Run the module and original tests.test_track_gui_closed; self-review numeric edge cases.
- [ ] Commit implementation and tests; write task report with exact command/output and API decisions.

### Task 2: Reference-free robust multi-lap geometry

**Files:** Create tools/track_gui/lap_average.py, tests/test_track_gui_laps.py; modify tools/track_gui/closed.py. May add an additive geometry-from-arrays helper to tools/tracks/build_closed_track.py without changing CLI defaults.

**Interfaces:** Consume ChannelSelection/load_coordinates. Extend ClosedSettings after existing positional fields with channel selection and multi-lap/detection/clipping settings using defaults that preserve old calls. build_closed(settings)->dict retains geometry keys consumed by summarize/save_closed. New lap_average helpers detect boundaries, align loops and compute a clipped arithmetic mean; expose diagnostics and per-lap overlap arrays in geometry. Store exact settings and diagnostics in saved provenance; do not retain misleading velocity-only source claims for GPS input.

- [ ] Write tests for noisy translated/phase-shifted loops, one deviating sample and one bad lap, incomplete tail, stationary intervals, irregular clocks, clockwise/counterclockwise input and failure with no complete loops. Assert accepted sample mean, not just median; assert finite positive length and correct winding.
- [ ] Run `PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_laps -q` to verify missing behavior fails.
- [ ] Implement return detection with heading/travel/time guards, loop quality filtering, translation/cyclic alignment, robust point clipping, mean and periodic smoothing. Avoid arbitrary rotation of north/east frame. Bound nodes and work. Give informative refusals and diagnostics.
- [ ] Integrate single/multi-lap paths in ClosedSettings/build_closed/save_closed while preserving legacy single-lap tests.
- [ ] Run tests.test_track_gui_laps, tests.test_track_gui_channels, tests.test_track_gui_closed, tests.test_closed_track, tests.test_track_tool_patches; commit and report.

### Task 3: Track reversal and section exports

**Files:** Create tools/track_gui/track_ops.py and tests/test_track_gui_ops.py; modify tools/track_gui/export.py only if shared export support is required.

**Interfaces:** Produce TrackData(distance,x_north,y_east,heading,curvature,is_closed,title,maximum_radius=300.0) with optional XY arrays for unsigned legacy tracks; load_track_data(path)->TrackData; reverse_track(track)->TrackData; split_closed(track,cuts)->list[TrackData]; save_track_data(track,directory,stem,model_id,title,*,allow_models=False,overwrite=False,overwrite_generated=False)->SavedTrack; save_sections(track,cuts,directory,stem,model_id,title,*,allow_models=False,overwrite=False,overwrite_generated=False)->list[SavedTrack]. Add helper from closed geometry and OpenCourse if needed for GUI handoff.

- [ ] Write tests for open and closed reverse, same closed start, double reversal, negative curvature, unsigned legacy reversal, exact/end/wrap cuts, invalid cuts and protected destinations. Run tests.test_track_gui_ops and capture missing-module failure.
- [ ] Implement immutable copied geometry and transformations, using load_track to read all supported formats. For sectioning, preserve line positions with suitable interpolation/resampling and derive consistent heading/curvature; verify strict open-loader tolerances without changing src. Record any reconstruction correction in output provenance.
- [ ] Implement staging/validation for all section outputs before installation. Validate numbered names and all overwrite guards before destination changes; roll back the whole batch if installation fails. Escape metadata safely.
- [ ] Test canonical closed files and open presets through split/reverse/save/load. Assert no partial files when a later section is refused or installation fails.
- [ ] Run tests.test_track_gui_ops, tests.test_track_gui_export, tests.test_track_gui_design, tests.test_tracks; commit and report.

### Task 4: GUI channel controls and viewer

**Files:** Modify tools/track_gui/qt/main_window.py and plots.py; create tools/track_gui/qt/track_viewer.py and optional focused Qt helper modules; add tests/test_track_gui_tools_qt.py.

**Interfaces:** Consume Tasks 1–3 exact public APIs. Keep MainWindow.design_tab and closed_tab and current test-visible attributes. Add viewer_tab. Viewer receives TrackData from files or current design/build, supports reverse and split export. Reusable plot click signal emits nearest node index only within a reasonable pixel hit radius. Display explicit unsigned legacy notice and disable map/section operations that need XY.

- [ ] Write Qt tests for metadata-populated channel selectors, selected settings reaching worker, invalid builds clearing stale results, loading open/closed/unsigned files, reversal, table/N/map cut controls and save refusal/confirmation.
- [ ] Run new tests to establish failure; implement channel/mode/scale selectors, multi-lap controls, overlap and curvature diagnostics, and transfer-to-viewer buttons. Invalidate or clearly distinguish results if settings change. Keep workers safe during closing and prevent stale result use.
- [ ] Implement viewer load/map/curvature/reverse controls and editable cuts; cuts are in metres, sorted/validated, plotted and export numbered open tracks. Show start and running direction. Keep plots and controls usable at normal desktop size.
- [ ] Run all track_gui Qt/widget tests with QT_QPA_PLATFORM=offscreen and app smoke with TRACK_GUI_QUIT_AFTER_MS=300.
- [ ] Commit and report exact tests and manual limitations.

### Task 5: Real-data acceptance and documentation

**Files:** Add tests/test_track_gui_real_data.py (skip clearly when private fixtures absent), tools/track_gui/validate_track_imports.py; update docs/dev/track-gui.md and gui-manual-checks.md; add curated acceptance diagnostics/report under docs/superpowers/sdd/2026-10-20-track-gui-tools/.

**Interfaces:** Consume all completed tasks. Validator accepts explicit MAT paths and output directory; emits JSON diagnostics and overlap figures, validates saved/reversed/sectioned output via load_track. Tests use environment-supplied private paths with documented local defaults, never require private data in CI.

- [ ] Add acceptance tests that use VNorth/VEast for MIS and Latitude_MTI/Longitude_MTI for FSC. Use spatial lap detection; record selected windows and settings. Assert plausible loops, nonzero winding, finite geometry, clipping diagnostics and export/reverse/split round trips.
- [ ] Run validator on both actual files and inspect overlap figures with read_image. Record hashes, commands, lap counts, rejected laps, clipping and geometry diagnostics. Fix actual defects through reviewed implementation amendments, not by weakening assertions.
- [ ] Update user guide and desktop checklist with channel units/axes, known empty FSC channels, multi-lap limits, viewer/reverse/cuts, safety rules and exact tests. Preserve phrases asserted by existing documentation tests.
- [ ] Run all focused track tests and full suite; compare normalized named failures with main baseline. Record new failures separately. Run independent whole-branch review, fix findings, re-review and merge local main only after acceptance.
- [ ] Commit docs/acceptance artifacts; report exact tests without claiming manual desktop validation.
