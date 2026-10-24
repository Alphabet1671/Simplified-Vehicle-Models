# TV runner compatibility follow-up report

## Scope and base

- Base: `8e2c774fd52cd79ecd419898618831bcc8914ee9` (`Untrack ignored SDD scratch pointer`).
- Requested runtime: `PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python`.
- Verified runtime: Python 3.13.13, NumPy 2.5.4, SciPy 1.17.1, CasADi 3.8.1.
- Limited to the demonstrated public Namespace compatibility default and the existing TV-runner test seam. No TV physics/model/cache/validator/tolerance changes, no shared worker architecture changes, and no U3 claim.

## Changes

- `tools/run/solve_minimum_time_line.py`: `_run_public_shared_spline` reads missing `wall_cap_s` with parser-compatible `900.0`; explicit values are retained in the request. Worker request remains fixed `public_spline_setup_v1` and continues through the shared supervised ladder.
- `tests/test_tv_runner.py`: historical synchronous numerical fixtures and their validation-guard checks now invoke the explicit `_run_compat_pipeline(args)` seam. Standard public ideal+spline tests stay on `run(args)` and verify the fixed shared-public route and forwarded 900-second/default and explicit wall caps; a distinct compatibility dispatch test verifies the fixed supervisor route. Existing-output collision coverage invokes `_prepare_public_spline_request_in_worker` in a temporary control root, asserts typed `worker_exception` / `FileExistsError` is returned before shared request construction, and confirms prior output bytes are unchanged.
- No arbitrary injectable worker/callable route was introduced; production CLI remains supervised.

## RED then GREEN evidence

- Initial focused RED: `PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_tv_runner -q` ran 49 tests and failed with 8 failures + 14 errors. Both standard shared-spline tests visibly reproduced missing `args.wall_cap_s`; the remaining failures/errors were consistent with in-process mocks not crossing the supervisor boundary.
- Focused final: `PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_tv_runner -q` — 51 tests, OK.
- Covering tests: `PYTHONPATH=src runs/no_tv_reliability/runtime_install_20261023/venv/bin/python -m unittest tests.test_minimum_time_line_gate tests.test_minimum_time_run tests.test_tv_runner -q` — 108 tests, OK. This is 57 tests from the two covering modules + 51 TV-runner tests; repository 126 preceding coverage count was not reconstructed because referenced migration/supervision module names were unavailable in this checkout.
- `git diff --check` passed.
- A mistaken attempted test invocation named absent modules `tests.test_pipeline_migration` and `tests.test_no_tv_supervision`; unittest reported 2 import errors after 59 tests. The corrected extant-module combined command above passed. No files were changed by the failed invocation.
- The full 2,357-test suite was not run, as explicitly instructed.

## Remaining scope limits

- The test seam exercises numerical legacy pipeline behavior explicitly; it does not assert those in-process numerical fixtures are the production public route.
- Public shared route is tested without running an NLP. Actual child existing-output guard is tested without NLP through its fixed preparation entry.
- No statement is made about broader branch-suite disposition, solver qualification, U2 event support, or U3 closed-loop execution.
