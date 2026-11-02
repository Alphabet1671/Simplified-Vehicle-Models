# Task 2 — multi-lap averaging/build/GUI integration

## Result
Implemented Task 2 in the isolated `fix/start-finish-closure` worktree. This updates multi-lap correction and seam processing; the single-lap `geometry_from_arrays` route remains unchanged. The Task 1 `smooth_seam` helper was consumed without modification.

### Code and diagnostics
- Per-lap endpoint drift correction now uses raw cumulative polyline distance divided by raw total distance, not sample index. Raw endpoint gap magnitudes are collected as `closure_gaps_m`.
- Retains the robust unclipped mean for seam detection/frame and uses it with accepted aligned laps and the keep mask to call the seam helper after existing optional periodic smoothing, before final periodic arc-length resampling.
- Adds `correction_method` with the linear-drift assumption and identifies raw distance basis. Adds `seam_closure` diagnostics and explicit semantics: its fit window is the sampled matched-phase span, bounds are sampled accepted lap observations rather than physical track edges, and final interpolated/resampled geometry is not envelope-certified. Solver-grid conditions are therefore not claimed as physical-track containment.
- Existing `closed.py` carries all lap diagnostics into geometry and XML provenance; the integration regression verifies the exported XML includes diagnostics and correction method.
- The multi-lap checkbox tooltip explains the automatic bounded straight join, advises selecting a window starting on the start/finish straight, clarifies that the corridor is not track edges, and notes fit may skip. The report shows seam status, sampled matched-phase span and peak movement on success or skip reason otherwise, while preserving full diagnostics output.
- `average_laps(...)`, `ClosedSettings`, robust acceptance/clipping, overlaps, and single-lap behavior/signatures remain intact. No controls or dependencies added.

## Tests
- Density regression was run before code: **failed as expected**, with max coordinate difference ~0.332 m between identical geometry sampled using uniform versus warped spatial phase. After the arc-distance correction: **passed** (0.06 m tolerance).
- Synthetic stadium MAT integration first failed because diagnostics were absent. After integration: **passed**; invokes the actual `build_closed`/`smooth_seam` route (no helper mock), verifies successful seam status, finite geometry and curvature, saved XML, load-back closure and diagnostic/provenance fields.
- GUI slot regression checks the tooltip caveat, and renders a skipped seam status/reason in report. It is located in the PyQt-guarded `test_track_gui_qt.py`, so core closed tests remain Qt-free.
- Focused command:
  `QT_QPA_PLATFORM=offscreen MPLCONFIGDIR=/tmp/start-finish-mpl PYTHONPATH=src:. ../../.venv/bin/python -m unittest discover -s tests -p 'test_track_gui*.py' -q`
  **124 tests passed** in ~5.1 s. Baseline Qt font/`propagateSizeHints` notices only. `git diff --check` passed.
- Full-suite validation is incomplete: controller's bounded run timed out at 300 s (exit 124) without a completed summary. Historical baseline full discovery exceeded 30 minutes; no unbounded rerun attempted.

## Limitations / semantics
Real measured data has not been used to claim improved track accuracy. A separate controller diagnostic noted an MIS sample succeeds when its mean/overlaps are jointly phase-rotated to a real straight (span 16.47 m; energy 0.0184883 to 0.0121472), while the current observed seam at the selected window can legitimately skip because its surrounding phase is curved. This change does not relocate the seam. If fit is skipped, existing periodic average remains. Drift and genuine driving-line endpoint displacement cannot be separated from these channels; the correction assumes their net endpoint displacement is linear with distance.

## Commit / repository state
Committed Task 2 integration files as `bfe5bb88dda446aa6d756f07f323deaba10d5025` on `fix/start-finish-closure` in this worktree only. No branch switch, merge, or changes to original checkout.
