# Task 7 report — headless maneuver runner

## Scope and implementation

Implemented the headless simulation-only execution boundary in `tools/maneuver_gui/runner.py`, with the artifact API from the committed `df2ef15` implementation (`create_attempt_directory`, `write_artifacts`). Definition validation happens in the parent before a child is launched; invalid definitions produce a rejection summary and do not invoke physics. The accepted backend is explicitly `simulation`; there are no physical-command interfaces in this runner.

Added `run_child`, a testable subprocess utility that creates a new process session, reads merged child output line-by-line, decodes JSON event lines for a progress callback, and forwards non-JSON diagnostics to stderr. Cancellation callbacks/event-like objects and bounded timeouts are supported. Termination signals the whole process group with SIGTERM, escalates to SIGKILL after the configured grace period, and reaps the child.

The CLI remains available as `python -m tools.maneuver_gui.runner --definition … --output-root …`; `--help` is covered by a subprocess test.

## Verification

Command: `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_runner -v`

Result: **4 tests passed**. Coverage includes artifact writing and manifest integrity, invalid-definition rejection without child physics, progress JSON delivery and cancellation of a child process group, unique attempt directories, and CLI help.

No expensive plant integration was run. The runner's child plant path therefore remains outside this test's claims.
