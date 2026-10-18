# Task 7 report

## Built

- Added `tools/sim_gui/run_study.py` with timestamped UTC study directories, case preparation and validation, invalid-case persistence, parallel child execution, timeout kill handling, locked `StudyIndex` updates, callbacks, and CLI argument handling.
- Added integration coverage for two parallel 0.3 s planar cases, CLI invocation, callback status transitions, and run-record outputs.
- Added a unit-style invalid-case test using a patched validator; invalid cases are persisted and never start.

## Study directory ruling

The plan specifies the output layout as `runs/<study-name>_<YYYYmmdd-HHMMSS>/`, but does not provide a separate literal “Study folder” decision row. This implementation uses `<output_root>/<definition.name>_<UTC yyyymmddThhmmssZ>` to provide collision-resistant UTC names while preserving the planned layout.

## TDD evidence

RED:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_run -v
...
ModuleNotFoundError: No module named 'tools.sim_gui.run_study'
(exit code 1)
```

GREEN:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_core tests.test_sim_gui_run -v
Ran 38 tests in 44.391s
OK
```

## Concerns

- The integration test runs the two cases once through the Python API and invokes the CLI as a second run, so it takes roughly 45 seconds on this machine.
- Invalid cases produce CLI exit status 0 when all non-invalid cases complete; this follows the brief’s “every valid case” wording.

## Review fix round 1

- Study directories now use local `%Y%m%d-%H%M%S` names, with `_2`, `_3`, etc. collision suffixes.
- Worker failures after `running` are recorded as `failed` with the exception text, without stopping other workers.
- CLI exit status is 0 only when at least one valid case exists and every valid case completes. All-invalid studies return 1.
- Added docstrings and CLI validation for positive `--parallel`.

Review validation:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_core tests.test_sim_gui_run -v
Ran 41 tests in 44.918s
OK
```
