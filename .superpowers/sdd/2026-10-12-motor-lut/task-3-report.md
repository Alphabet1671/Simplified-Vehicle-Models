# Task 3 Report

## Status
Implemented LUT-first forward and inverse motor APIs with exact analytical fallback. Regen inversion scans every negative-load piecewise-linear segment and preserves the full non-monotone envelope; efficiency is derived from interpolated bus power and analytically computed shaft power.

## Commit
- `7069521 feat: query motor LUT forward and inverse`

## Interfaces
- `TorqueLimits`, `TorqueSolveResult`
- `evaluate_motor(parameters, motor_speed_rpm, shaft_torque, bus_voltage)`
- `motor_torque_limits(parameters, motor_speed_rpm, bus_voltage)`
- `torque_for_bus_power(parameters, motor_speed_rpm, bus_voltage, target_bus_power, direction)`
- `reset_motor_lut_counters()` and `motor_lut_counters()` for benchmark observability

## Exact tests
- `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_powertrain -v` — 27 tests, OK.
- `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest discover -s tests -q` — 231 tests, OK.
- `git diff --check` — passed before commit.

Added deterministic node, seeded-random, inverse, and fallback tests to `tests/test_powertrain.py`. The random forward sample used 100 samples across the voltage/speed domain and both torque directions. Absolute bus-power error distribution versus `evaluate_shaft_power_exact`: minimum 0.00030 W, median 0.445 W, 95th percentile 9.738 W, maximum 328.157 W. The test gate is a 95th-percentile error below 120 W. Exact-node samples passed at three voltage/speed corners/interior nodes. Out-of-domain voltage fallback returned an identical `ShaftPowerResult` and incremented the fallback counter.

## Lookup timings
100,000-query Python microbenchmark on the canonical AMK map:

| operation | microseconds/query |
|---|---:|
| exact analytical evaluator | 16.27
| LUT forward evaluator | 10.61
| in-domain drive inverse | 70.53

Inverse targets were generated from in-domain half-drive-load points. Timing is informational, not a numerical test gate. The benchmark and timings are recorded in `docs/validation/powertrain.md`.

## Fallback behavior
Queries outside persisted voltage/speed or normalized-load domains call `evaluate_shaft_power_exact` without clamping or extrapolation and set `used_fallback` on limit/inverse results. Out-of-range inverse targets use a bracketed analytical/sample fallback and report residual and feasibility. Counters distinguish forward, inverse, and fallback paths.

## Self-review
- Drive inversion uses `searchsorted` only on monotone positive-load rows.
- Regen inversion does not assume monotonicity; it evaluates every segment, retains all feasible crossings/endpoints, and selects the most-negative satisfying torque.
- Production LUT bus/loss interpolation is scalar bilinear/trilinear arithmetic; efficiency is derived rather than interpolated.
- Existing `evaluate_shaft_power` remains the exact compatibility alias so strict assembled fixed-point convergence remains unchanged; new LUT callers use `evaluate_motor`.

## Concerns
- The inverse API uses signed bus-power targets (drive positive, regen negative); callers should not pass unsigned charge magnitudes.
- LUT interpolation can have larger worst-case error near sharp non-monotone regen features (328 W in the 100-sample exploratory distribution), though the reviewed 95th-percentile gate passes.
