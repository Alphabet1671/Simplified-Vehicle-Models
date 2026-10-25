# Task 2 report — maneuver waveforms and CSV playback

## Delivered

Implemented Qt-free `SteeringWaveform` with validated step, ramp, and integral-cycle sine profiles; analytic sampling and breakpoints; immutable `CsvMapping` and `CsvSource`; strict CSV parsing with explicit column/unit/interpolation settings; canonical SI `RobotCommand` output; hold/linear interpolation; strict bounds without extrapolation; source-byte retention and SHA-256.

CSV validation includes UTF-8 BOM, selected comma/semicolon delimiter and decimal convention, header and trailing blank lines, mapped fields, numeric finite values, strictly increasing nonnegative times, explicit rebase, bounded steering and pedals, pedal exclusivity, linear accelerator/brake crossings, 1,000,000 data-row limit, and 100 MiB file limit. Three-column data requires an explicit positive sample interval.

## Tests

TDD was followed: target tests were created and verified to fail on missing modules before implementation. Passing commands:

- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_csv.py' -v` — 4 tests passed.
- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_waveforms.py' -v` — 3 tests passed.
- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_schema.py' -v` — 7 Task 1 regression tests passed.

CSV tests verify exact original `source_bytes` equality and SHA-256; mocked oversized `stat` prevents `read_bytes`, and a mocked row ceiling exercises the row-limit rejection. Additional coverage exercises malformed mappings/headers and out-of-range steering/pedal values. The loader checks `Path.stat().st_size` before reading, then checks the actual read length again to handle file-growth races.

## Follow-up hardening evidence

After reviewer feedback, added pre-read file-size protection and retained the post-read race-safe check. The expanded CSV suite now has 6 tests (including pre-read cap, row cap, source bytes/hash, mapping/header and input envelope cases). Re-ran CSV (6 passed), waveform (3 passed), and Task 1 schema regression (7 passed). `git diff --check` passed after removing trailing whitespace.

## Notes

No Task 1 source files, planning documents, or dependencies were changed. Sine release is continuous by requiring integral cycles; the release field does not add a post-sine tail. The steering safety envelope is a fixed +/- pi rad input bound pending a later explicit envelope parameter/interface.
