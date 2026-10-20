# Task 5 Report: Animation GUI Record Finder

## Status
Implemented and committed the Qt-free animation GUI record finder.

## Files
- `tools/anim_gui/records.py`
- `tests/test_anim_gui_records.py`

## Implementation
- Added `RECORD_NAME = "run_record.npz"`.
- Added frozen, slotted `RecordChoice` with `label` and `path` fields.
- Added `study_records()` to validate and read `sim-study-v1` studies, include only existing records inside the study directory, and reject unreadable or invalid study files with `ValueError`.
- Added `find_records()` to locate records recursively, order nearest records first, and cap results at the requested limit.
- Added `remove_partials()` to remove video `.partial.mp4` files and delete PNG frame folders only when every entry is a frame PNG.
- No Qt imports and no changes to `src/`, `tools/sim_gui/`, `tools/anim/`, or `pyproject.toml`.

## Tests
Focused command:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_records -v
```

Output:

```text
test_bad_study_files ... ok
test_find_records ... ok
test_remove_partials_png_only_when_the_folder_is_all_frames ... ok
test_remove_partials_video ... ok
test_study_lists_only_cases_with_a_record ... ok

----------------------------------------------------------------------
Ran 5 tests in 0.005s

OK
```

The required pre-implementation run failed as expected with an `ImportError` because `tools.anim_gui.records` did not yet exist.

## Diff-check

```text
git diff --check
```

Passed with no whitespace errors.

## Commit
Committed as:

```text
feat: add the record finder for the animation GUI
```

## Concerns
- `find_records()` follows the brief's nearest-first ordering by directory depth, then path name; it does not use filesystem modification times.
- `remove_partials()` intentionally leaves mixed-content frame folders untouched to avoid deleting unrelated user files.
- An unrelated pre-existing untracked `docs/superpowers/sdd/2026-10-19-animation-gui/task-2-report.md` remains outside this task commit.

## Fix Round 1

Addressed reviewer safety and validation findings:

- `remove_partials()` now rejects symlink and non-regular-file frame entries, does not recurse through symlinked output folders, and safely ignores a directory-shaped `.partial.mp4` path.
- `study_records()` now raises `ValueError` for non-object top-level JSON, non-object cases, and non-string `run_record` values.
- `find_records()` now validates `limit` as a non-boolean, non-negative integer; `limit=0` returns no choices.
- Added regression coverage for all of the above.

Focused rerun:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_anim_gui_records -v
Ran 5 tests in 0.006s
OK
```

Diff-check passed again with `git diff --check`.

Ordering ruling: nearest-first ordering remains as specified by the normative reference implementation and exact test; changing it would risk mismatch with the required interface behavior.
