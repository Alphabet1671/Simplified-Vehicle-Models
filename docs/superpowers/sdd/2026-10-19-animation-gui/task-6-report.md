# Task 6 Report: Animation GUI Main Window and Application

## Status
Implemented the animation exporter GUI main window and application entry point.

## Files
- `tools/anim_gui/qt/__init__.py`
- `tools/anim_gui/qt/main_window.py`
- `tools/anim_gui/app.py`
- `tests/test_anim_gui_qt.py`

## Implementation
- Added `MainWindow` with idle/export/preview modes, one shared `ProcessJob`, six-hour export timeout, and 120-second preview timeout.
- Added record/study loading, exporter-owned dry-run plan inspection, format/size/range controls, progress/log display, cancellation, still-frame preview, geometry persistence through `QSettings("Simplified-Vehicle-Models", "anim-gui")`, and `ANIM_GUI_QUIT_AFTER_MS` support.
- Corrected `ProcessJob` invocation to pass the command argument tail because `ProcessJob` supplies the current Python interpreter.
- Added two-step overwrite confirmation for every existing output, including empty directories.
- Added canonical `models/` protection requiring explicit confirmation before writing there; cleanup remains limited to exporter partial outputs.
- Qt imports remain confined to Qt modules/application/tests. No changes were made under `src/`, `tools/sim_gui/`, `tools/anim/`, or `pyproject.toml`.

## Focused verification
```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_qt -q
Ran 14 tests in 7.798s
OK
```

The suite launches real child processes for PNG export, MP4 export (when ffmpeg is available), preview, cancellation, and exporter-error handling. No test hung; all completed within the command timeout. Validation was performed with Qt's offscreen platform; no manual desktop-window validation was performed in this run.

## Diff check
`git diff --check` passed.
