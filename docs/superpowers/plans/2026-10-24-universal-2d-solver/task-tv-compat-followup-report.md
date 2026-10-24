# TV runner compatibility follow-up report

## Scope and base

- Implementation base: `8e2c774fd52cd79ecd419898618831bcc8914ee9` (`Untrack ignored SDD scratch pointer`). Implementation commit: `2c7880967153f2064385ec9597e6ff8ab55bf3ab` (`Preserve TV runner namespace wall cap compatibility`).
- Requested runtime: `PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python`.
- Verified runtime: Python 3.13.13, NumPy 2.5.4, SciPy 1.17.1, CasADi 3.8.1.
- Limited to the demonstrated public Namespace compatibility default and existing TV-runner test seam. No TV physics/model/cache/validator/tolerance changes, no shared worker architecture changes, and no U3 claim.

## Changes

- `tools/run/solve_minimum_time_line.py`: `_run_public_shared_spline` reads missing `wall_cap_s` with parser-compatible `900.0`; explicit values are retained in the request. Worker request remains fixed `public_spline_setup_v1` and continues through the shared supervised ladder.
- `tests/test_tv_runner.py`: historical synchronous numerical fixtures and their validation-guard checks now invoke the explicit `_run_compat_pipeline(args)` seam. Standard public ideal+spline tests stay on `run(args)` and verify the fixed shared-public route and forwarded 900-second/default and explicit wall caps; a distinct compatibility dispatch test verifies the fixed supervisor route. Existing-output collision coverage invokes `_prepare_public_spline_request_in_worker` in a temporary control root, asserts typed `worker_exception` / `FileExistsError` is returned before shared request construction, and confirms prior output bytes are unchanged.
- No arbitrary injectable worker/callable route was introduced; production CLI remains supervised.

## RED/GREEN evidence

Pinned runtime prefix for every command below:

```text
PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python
```

- Initial focused RED: `-m unittest tests.test_tv_runner -q` ran 49 tests and failed with 8 failures + 14 errors. Both standard shared-spline tests visibly reproduced missing `args.wall_cap_s`; remaining failures/errors were consistent with in-process mocks not crossing the supervisor boundary.
- Focused GREEN before implementation commit: `-m unittest tests.test_tv_runner -q` — 51 tests, OK.
- Covering GREEN before implementation commit: `-m unittest tests.test_minimum_time_line_gate tests.test_minimum_time_run tests.test_tv_runner -q` — 108 tests, OK.
- Post-implementation-commit focused verification, command `-m unittest tests.test_tv_runner tests.test_minimum_time_line_gate tests.test_minimum_time_run -q` — 108 tests, OK in 16.471 seconds.
- Post-implementation-commit seven-module covering command:

```text
-m unittest tests.test_run_manifest tests.test_input_preflight tests.test_no_tv_minimum_lap tests.test_minimum_time_run tests.test_trajectory_artifact tests.test_minimum_time_line_gate tests.test_tv_runner -q
```

  Result: **177 tests passed in 28.108 seconds** on the final post-path-correction run: 126 tests across the first six modules plus 51 TV-runner tests. An earlier identical run passed 177 tests in 28.502 seconds.
- An earlier mistaken invocation used inferred module names `tests.test_pipeline_migration` and `tests.test_no_tv_supervision` and returned two import errors after 59 tests. This was a mistaken invocation; it changed no files and is superseded by the seven-module command above.
- The full 2,357-test suite was not run, as instructed.

## Report-path correction

The initial implementation commit mistakenly tracked a report copy at repository root. That root file is retained byte-for-byte on disk and untracked from Git. This curated copy is the report artifact; the ignored task workspace copy is `.superpowers/sdd/2026-10-24-universal-2d-solver/task-tv-compat-followup-report.md`.

## Boundaries

- Numerical legacy fixtures exercise an explicit compatibility pipeline seam; they do not claim this is the production public route.
- Public shared route is tested without running an NLP. The actual child existing-output guard is tested without NLP through its fixed preparation entry.
- No statement is made about broader branch-suite disposition, solver qualification, U2 event support, or U3 closed-loop execution.
