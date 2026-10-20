# Task 2 Report: Animation GUI Export Settings

## Status
Implemented and committed the Qt-free animation GUI export settings foundation.

## Files
- `tools/anim_gui/__init__.py`
- `tools/anim_gui/animation_gui_settings.py`
- `tests/test_anim_gui_settings.py`

## Implementation
- Added frozen, slotted `ExportSettings` with the exact exporter defaults and all required scene, timing, view, and output options.
- Added `check()` validation for format, record existence, numeric ranges, even dimensions, CRF, encoder preset, time range, body dimensions, and MP4 suffix.
- Added `arguments()` generation matching `tools/anim/export_animation.py`, including PNG output, dry-run, progress, STL, body geometry, transforms, and view flags.
- Added safe default output naming beneath `videos/` using `tools.gui_common.files.check_name`.
- Added PNG preview settings helper with an even 640x360 default window.
- No Qt imports, rendering changes, exporter changes, or changes under `src/`, `tools/sim_gui/`, `tools/anim/`, or `pyproject.toml`.

## Tests
Focused command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_settings -v
Ran 6 tests in 0.029s
OK
```

The animation CLI regression suite was also run. Its three render-related tests failed while invoking the existing `tests.test_run_record_runner.real_child("14dof-simple", "--record")` setup (status 2), before exercising this settings module. The focused settings suite passes completely.

## Self-review
`git diff --check` passed. The implementation follows the task brief verbatim and uses repository-provided helpers only.

## Commit
`feat: add the export settings for the animation GUI`

## Fix Round 1 Review Findings
Addressed all review findings:
- `check()` now rejects non-finite `transform_x` and `transform_z`.
- `check()` now requires integer (non-boolean) width, height, and CRF values, preventing float values from diverging from argparse behavior.
- `default_output()` now rejects formats outside `FORMATS`.
- `default_output()` now sanitizes both the parent directory name and record stem, including common names containing spaces.
- Added focused regression cases covering all of the above.

Focused verification:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_settings -v
Ran 6 tests in 0.031s
OK
```
