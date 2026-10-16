# Task 5: Structured Progress Reporting

## RED/GREEN

- RED: Added callback-order tests and ran `PYTHONPATH=src python3 -m unittest tests.test_energy_lap.EnergyLapTests.test_progress_callback_order_and_none_silence -v`; collection failed because `SolverProgress` did not yet exist.
- GREEN: Implemented the frozen progress record, callback emission, stint forwarding, and CLI output. The focused progress tests passed (`2 tests, OK`).
- Full energy-lap suite: `PYTHONPATH=src python3 -m unittest tests.test_energy_lap -v` — 12 tests, OK.
- Full suite: `PYTHONPATH=src python3 -m unittest discover -s tests -v` — 212 tests, OK.
- `git diff --check` passed.

## Files

- `src/simplified_vehicle_models/results/traces.py`: added frozen, slotted `SolverProgress`.
- `src/simplified_vehicle_models/solvers/forward_backward.py`: added optional callbacks to energy lap and stint solvers; emits lap start, energy iteration, lap completion, and stint completion events with monotone elapsed time and lap/iteration metadata.
- `tests/test_energy_lap.py`: added callback ordering, stint metadata, and no-callback coverage.
- `examples/run_mis_point_mass.py`: added `--progress` concise energy iteration/lap completion output.

## Self-review

Callbacks are optional, return values are ignored, and reporting is outside numerical kernels. Existing solver defaults and numerical paths remain unchanged when no callback is supplied. Reused stint profiles also report lap boundaries and final stint completion.

## Concerns

Callback exceptions are intentionally not swallowed; callers can use them to stop a solve. The CLI progress stream is human-readable output before the JSON report when `--energy --progress` is selected.
