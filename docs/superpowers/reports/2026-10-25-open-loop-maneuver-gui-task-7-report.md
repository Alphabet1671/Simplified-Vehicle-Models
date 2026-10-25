# Task 7 report — headless maneuver runner

## Scope and implementation

Implemented the headless simulation-only execution boundary in `tools/maneuver_gui/runner.py`, with the artifact API from the committed `df2ef15` implementation (`create_attempt_directory`, `write_artifacts`). Definition validation happens in the parent before a child is launched; invalid definitions produce a rejection summary and do not invoke physics. The accepted backend is explicitly `simulation`; there are no physical-command interfaces in this runner.

Added `run_child`, a testable subprocess utility that creates a new process session, reads merged child output line-by-line, decodes JSON event lines for a progress callback, and forwards non-JSON diagnostics to stderr. Cancellation callbacks/event-like objects and bounded timeouts are supported. Termination signals the whole process group with SIGTERM, escalates to SIGKILL after the configured grace period, and reaps the child.

The CLI remains available as `python -m tools.maneuver_gui.runner --definition … --output-root …`; `--help` is covered by a subprocess test.

## Fix round 1 — safety review findings

The reviewed safety gaps are addressed:

- Artifact writers write the full set in a unique sibling staging directory and publish it with a same-filesystem directory rename. Attempt/run names are exclusively reserved; duplicate attempts do not overwrite. Output roots resolving inside `models/` are rejected, including attempts reached through symlink resolution.
- The loader verifies canonical hashes for definition, calibration and model, compares the manifest's definition/model/calibration against `maneuver.json`, checks the summary outcome against the manifest, and verifies hashes for summary, events, commands and NPZ evidence.
- The parent runner accepts optional cancellation and timeout parameters. Child output is isolated in a unique result directory beside the attempt; the parent validates it then atomically renames the entire directory to `attempt/result` as one operation. Cancellation/failure removes staging and writes only a failure summary; a finalizer also cleans staging on exceptions/timeouts. Formulations must be existing XML files resolving under this repository's `models/` root.
- Process-group cleanup always sends TERM and then KILL after grace even if the process leader has exited; a TERM-ignoring grandchild regression confirms cleanup returns without a hang and the descendant is gone. Physical command backends remain rejected; execution is simulation-only.

## Verification

- `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_artifacts -v`: **5 passed**.
- `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_artifacts tests.test_maneuver_runner -v`: **14 passed**, including atomic success publication, unsuccessful-stage cleanup, TERM-ignoring descendant cleanup, and post-publish rollback regressions.
- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_*.py' -q`: **78 passed**.
- `git diff --check`: passed.

## Fix round 3 — post-publication rollback

- The runner performs normal fallible preparation before renaming validated stage evidence into `attempt/result`; it tracks the published result and removes it if a later exceptional path is reached before writing a rejection summary.
- The fault-injection regression forces a post-publication failure and verifies that `summary.json` is failed while neither `result` nor a result-stage directory remains.

No expensive plant integration was run; the runner's full child-plant execution path remains outside these test claims.
