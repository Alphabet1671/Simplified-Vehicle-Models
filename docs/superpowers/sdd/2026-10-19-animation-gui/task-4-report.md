# Task 4 Report: Animation GUI Progress Parser

## Status
Implemented and committed the Qt-free animation progress parser for the GUI.

## Files
- `tools/anim_gui/progress.py`
- `tests/test_anim_gui_progress.py`

## Implementation
- Added frozen, slotted `Progress` with `done`, `total`, `frames_per_second`, and `eta_seconds` fields.
- Added `fraction` computation with a zero-total guard.
- Added regex parsing for the exporter's `frame done/total rate frames/s eta seconds s` progress line, including `nan` rate/ETA values.
- Added `parse_error()` for exporter lines prefixed with `error:`.
- Added `describe_result()` messages for successful, cancelled, timed-out, failed-to-start, known exit-code, and unknown exit-code results.
- Added `format_eta()` with human-readable minute/second formatting and an `estimating…` result for non-finite values.
- No Qt imports and no changes to `src/`, `tools/sim_gui/`, `tools/anim/`, or `pyproject.toml`.

## Tests
Focused command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_progress -v
Ran 5 tests in 0.012s
OK
```

The brief's required baseline run was also performed before implementation and failed as expected because `tools.anim_gui.progress` did not yet exist (`ImportError`).

## Diff-check

```text
git diff --check
```

Passed with no whitespace errors.

## Commit
Committed as:

```text
feat: add the progress parser for the animation GUI
```

## Concerns
- The parser intentionally follows the exact exporter format and brief-provided regular expressions; it does not accept signed values or alternate progress labels.
- `eta_seconds` and `frames_per_second` use `math.nan` when the exporter reports `nan`, as specified.
- An unrelated pre-existing untracked `docs/superpowers/sdd/2026-10-19-animation-gui/task-2-report.md` remains outside this task commit.
