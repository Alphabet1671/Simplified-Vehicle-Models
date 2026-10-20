# Animation GUI Final-Review Fix Report

## Status

Implemented both final-review blocker fixes and committed them on the animation GUI branch.

## Changes

- Renamed the animation GUI entry point from `tools/anim_gui/app.py` to `tools/anim_gui/animation_gui_app.py` and settings module from `tools/anim_gui/settings.py` to `tools/anim_gui/animation_gui_settings.py`.
- Updated all animation GUI imports, application launch commands, documentation, task reports, and Qt app test references. No compatibility shims were left behind.
- Added a regression to `tests/test_default_track.py` that checks each new animation top-level tool filename is unique across `tools/*/*.py`, keeping the `tests.test_default_track` glob lookup unambiguous.
- Added format-aware cleanup state in the main window. Failed, cancelled, timed-out, or errored MP4 jobs always remove only that job's `<stem>.partial.mp4`, even when the final MP4 existed before the run. PNG cleanup remains suppressed for pre-existing output directories so existing frame-only output is preserved.
- Added Qt regressions for failed and cancelled exports with a pre-existing MP4 target and partial file. Existing PNG preservation coverage remains in place.

## Tests

Exact focused command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_settings tests.test_anim_gui_inspect tests.test_anim_gui_progress tests.test_anim_gui_records tests.test_anim_gui_qt tests.test_anim_gui_docs tests.test_default_track -v
```

Exact result:

```text
Ran 61 tests in 8.743s

OK
```

The run included the new basename uniqueness test, both pre-existing-MP4 cleanup tests, and the pre-existing-PNG preservation test. The Qt suite exercised the application launch hook, real PNG/MP4 paths, cancellation, preview, and safety checks.

## Diff-check

Exact command:

```text
git diff --check
```

Exact output: no output; exit code 0.

## Self-review

- Verified that no `tools/anim_gui/app.py` or `tools/anim_gui/settings.py` files remain.
- Verified launch documentation points to `animation_gui_app.py` and imports point to `animation_gui_settings`.
- Cleanup branches on the active export format rather than only on whether the target pre-existed. MP4 cleanup delegates to the existing format-specific partial remover and does not touch the confirmed final output or unrelated files.
- PNG cleanup still runs only for newly created outputs; pre-existing PNG directories are preserved on failure.
- No changes were made under `src/`, `tools/anim/`, or `tools/sim_gui/`.

## Concerns

- Manual desktop validation remains outstanding; tests use Qt's offscreen platform.
- The broader repository suite retains the baseline failures documented in the animation GUI ledger and was not used as a green-status claim.
