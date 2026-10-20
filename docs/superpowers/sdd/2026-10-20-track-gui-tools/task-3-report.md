# Task 3 report: Track reversal and section exports

Implemented `TrackData`, `load_track_data`, `reverse_track`, `split_closed`, `save_track_data`, and `save_sections` in `tools/track_gui/track_ops.py`.

Section outputs use a periodic cubic SciPy spline in XY, derive heading/curvature analytically and arc distance numerically, compare sampled output positions to the source polyline, and refuse when measured maximum departure exceeds 0.02 m. Each staged output is validated through the strict existing `load_track`; XML provenance records measured departure and limit. Per-section sample allocation is capped at 100,000 nodes. SciPy is optional for normal loading/reversal and required only for section reconstruction; absent SciPy raises an explicit `ValueError` with installation guidance.

The batch saver checks every numbered destination before writing, stages and validates every pair, and uses a shared backup/rollback transaction for the entire batch. Existing model/generated-file protections are reused. Legacy unsigned reversal retains unsigned values and does not fabricate XY or signed direction.

Additional public converters `from_closed_geometry(geometry, title="Closed track", maximum_radius=300.0)` and `from_open_course(course, title="Open track", maximum_radius=300.0)` support GUI handoff. The closed converter accepts build_closed_track's `north`/`east` keys and appends the repeated seam row. Saves retain open/closed/unsigned subtype semantics and preserve `maximum_radius` in the standard XML parameter. For open tracks, the generated standard parameter is updated in place so the strict loader and simulator use the same cap.

Tests: `PYTHONPATH=src:. python3 -m unittest tests.test_track_gui_ops tests.test_track_gui_export tests.test_track_gui_design tests.test_tracks -q` passed (36 tests). The initial required missing-module baseline was captured before creating tests: `ModuleNotFoundError: No module named 'tests.test_track_gui_ops'`. Canonical section reconstruction was manually exercised for both canonical closed sources (single and multi-lap), and staged strict-loader validation passed.

Public API: `TrackData`, `load_track_data(path)`, `reverse_track(track)`, `split_closed(track, cuts)`, `save_track_data(track, directory, stem, model_id, title, *, allow_models=False, overwrite=False, overwrite_generated=False)`, `save_sections(track, cuts, directory, stem, model_id, title, *, allow_models=False, overwrite=False, overwrite_generated=False)`. XY values may be `None` for unsigned legacy profiles.
