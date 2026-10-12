# Task 4 Report: Propagate Inner Controls and Reuse Coupled-Lap Hints

## Status

Implemented and committed as `perf: reuse coupled lap solve hints`.

## RED/GREEN

- Added mock-based propagation coverage for inner lap tolerance, maximum iterations, root tolerance, and `DriveCachePolicy` across all SOC and reconciliation solves.
- Added regen-enabled cold-versus-warm equivalence coverage for convergence, speed, lap time, final SOC, consumed energy, and recovered energy.
- Initial strict equivalence assertions exposed expected floating-point differences from corner-root hinting; assertions were set to the configured numerical tolerances (`1e-8` profile/SOC-scale values and `1e-4` energy units).
- GREEN: the required focused suite completed successfully: 23 tests, 0 failures.

## Commands / Results

```text
PYTHONPATH=src python3 -m unittest tests.test_energy_lap tests.test_lap_solver tests.test_mis_regression -v
Ran 23 tests in 22.088s
OK
```

Also verified `git diff --check` successfully.

## Changes

- `src/simplified_vehicle_models/solvers/forward_backward.py`
  - Added explicit `lap_tolerance`, `lap_maximum_iterations`, and `root_tolerance` controls to `solve_lap_with_energy` with existing defaults.
  - Forwarded inner profile controls and the exact cache policy to every SOC and reconciliation `solve_lap` call.
  - Carried prior coupled speed/corner hints between SOC passes through the existing validated hint interfaces; `initial_speed` remains validation-only in `solve_lap`.
  - Extended `solve_stint` with named inner controls and propagated coupled hints between solved stint laps.
- `tests/test_energy_lap.py`
  - Added mock propagation assertions.
  - Added regen-enabled cold/warm numerical equivalence test.

## Self-review

- Energy-loop controls remain separate from inner profile controls.
- No transition/profile warm-start acceleration was reintroduced; the solver still initializes from conservative caps.
- Corner hints continue to use the bracket-preserving `maximum_corner_speed` interface.
- Existing default behavior and drive-cache lifecycle remain unchanged.
- Focused energy, lap, and MIS regression tests pass.

## Concerns

- Warm and cold paths can differ by normal root-solver floating-point noise; tests use configured numerical tolerances rather than exact bitwise equality.
- Hints are generated internally only across a fixed vehicle/track/mode/shape context; `solve_lap` continues to validate externally supplied hint shapes and values.
