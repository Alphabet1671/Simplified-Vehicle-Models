# Task 7 report — headless maneuver runner

## Scope and implementation

Implemented the headless simulation-only execution boundary in `tools/maneuver_gui/runner.py`, with the artifact API from the committed `df2ef15` implementation (`create_attempt_directory`, `write_artifacts`). Definition validation happens in the parent before a child is launched; invalid definitions produce a rejection summary and do not invoke physics. The accepted backend is explicitly `simulation`; there are no physical-command interfaces in this runner.

Added `run_child`, a testable subprocess utility that creates a new process session, reads merged child output line-by-line, decodes JSON event lines for a progress callback, and forwards non-JSON diagnostics to stderr. Cancellation callbacks/event-like objects and bounded timeouts are supported. Termination signals the whole process group with SIGTERM, escalates to SIGKILL after the configured grace period, and reaps the child.

The CLI remains available as `python -m tools.maneuver_gui.runner --definition … --output-root …`; `--help` is covered by a subprocess test.

## Fix round 1 — safety review findings

The reviewed safety gaps are addressed:

- Artifact writers write the full set in a unique sibling staging directory and publish it with a same-filesystem directory rename. Attempt/run names are exclusively reserved; duplicate attempts do not overwrite. Output roots resolving inside `models/` are rejected, including attempts reached through symlink resolution.
- The loader verifies canonical hashes for definition, calibration and model, compares the manifest's definition/model/calibration against `maneuver.json`, checks the summary outcome against the manifest, and verifies hashes for summary, events, commands and NPZ evidence.
- The parent runner accepts optional cancellation and timeout parameters. Child output is isolated in a staging directory and only published after a successful child exit and validated manifest; cancellation/failure removes staging and writes only a failure summary. Formulations must be existing XML files resolving under this repository's `models/` root.
- Process-group cleanup always sends TERM and then KILL after grace even if the process leader has exited; a TERM-ignoring grandchild regression confirms cleanup returns without a hang and the descendant is gone. Physical command backends remain rejected; execution is simulation-only.

## Verification

- `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_artifacts -v`: **5 passed**.
- `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_runner -v`: **5 passed** (including TERM-ignoring descendant, consistency, and unsafe formulation regressions).
- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_*.py' -q`: **74 passed**.
- `git diff --check`: passed.

No expensive plant integration was run; the runner's full child-plant execution path remains outside these test claims.
