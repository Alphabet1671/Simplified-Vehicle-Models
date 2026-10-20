# Task 3 Report: Animation GUI Dry-Run Inspection

## Status
Implemented the Qt-free dry-run inspection core and committed it.

## Files
- `tools/anim_gui/inspect_plan.py`
- `tests/test_anim_gui_inspect.py`

## Implementation
- Added frozen, slotted `PlanSummary` with exporter plan metadata, visibility, warnings, model, and wheel-data state.
- Added `encoder_problem()` using the animation encoder's `find_ffmpeg()` and `EncoderUnavailable` interfaces.
- Added `namespace()` that delegates argument parsing to `export_animation.build_parser()` using `ExportSettings.arguments(dry_run=True)`.
- Added `inspect()` that validates settings, constructs the exporter `Plan`, and returns the exact exporter `describe()` text without rendering or writing output; exporter errors become user-facing `ValueError`s.
- Added `record_span()` using the repository run-record loader with clear invalid-record errors.
- No changes to `src/`, `tools/sim_gui/`, `tools/anim/`, or `pyproject.toml`.

## Tests
Focused command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_inspect -v
Ran 6 tests in 0.033s
OK
```

A full-suite attempt was started but remained running for over four minutes with only an existing `ResourceWarning` visible; it was stopped to avoid indefinite waiting. No exit status was produced. The focused suite passes completely.

## Self-review
`git diff --check` passed before commit. The implementation follows the brief's exact interfaces and delegates dry-run planning and description to the existing exporter in-process, without importing rendering or starting ffmpeg.

## Commit
`feat: add the dry-run inspection for the animation GUI`
