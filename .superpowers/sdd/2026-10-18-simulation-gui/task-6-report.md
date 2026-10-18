# Task 6 report

## Built

- Added `tools/sim_gui/execute.py` with case preparation, formulation validation, attempt creation, child command construction, progress parsing, and final status/parent metadata.
- Added `tools/sim_gui/run_case.py` child entry point. It loads the case manifest, patches the selected runner model to use the case variant formulation and vehicle, and invokes the existing runner child.
- Added unit coverage for progress parsing, command construction, and status mapping.
- Added a real 0.3 s planar Hoosier integration test. It verifies the child return code, loads the generated run record with source verification, and checks the Hoosier source manifest entry.

## TDD evidence

RED:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_core -v
...
ModuleNotFoundError: No module named 'tools.sim_gui.execute'
(exit code 1)
```

GREEN:

```text
MPLCONFIGDIR=/tmp/mplcfg PYTHONPATH=src:. python3 -m unittest tests.test_sim_gui_core tests.test_sim_gui_run -v
Ran 35 tests in 11.224s
OK
```

## Files

- `tools/sim_gui/execute.py`
- `tools/sim_gui/run_case.py`
- `tests/test_sim_gui_core.py`
- `tests/test_sim_gui_run.py`

## Concerns

- `finish_case` receives the command and timing from the parent orchestration; this task does not itself manage process cancellation or timeout watches.
- The child intentionally reuses the existing runner implementation and does not modify hash-pinned runner files.
