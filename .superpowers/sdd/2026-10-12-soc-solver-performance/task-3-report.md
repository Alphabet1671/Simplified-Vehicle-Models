# Task 3 Report: Solver-Local Drive Cache

## Status
Implemented; round 1 review fixes applied. Exact caching is the default; quantization is opt-in through `DriveCachePolicy`.

## RED/GREEN

RED (before implementation):

```text
ImportError: cannot import name 'DriveCachePolicy' from simplified_vehicle_models.formulations.point_mass
```

GREEN (after implementation):

```text
Ran 31 tests in 62.645s
OK

Ran 6 tests in 11.547s
OK

Ran 1 test in 37.219s
OK
```

## Implementation

- Added immutable `DriveCachePolicy(speed_resolution=0.0, soc_resolution=0.0)` with finite/nonnegative validation.
- Replaced the module-global assembled-drive cache with a cache created inside each `solve_lap` call.
- Propagated the cache through corner-speed, forward/backward sweep, root, and final-equilibrium evaluations.
- Exact mode uses `(speed, soc)` float keys and reports zero displacement.
- Positive resolutions use `round(speed / resolution)` and `round(soc / resolution)` keys; the first actual coordinates in each bucket are retained.
- Added cache hit/entry counts and maximum speed/SOC bucket displacement to `LapResult` diagnostics.
- Extended `solve_lap_with_energy` to accept/pass the policy.

## Exact versus quantized measurements

MIS assembled flying lap, tolerance `1e-6`:

```text
exact:     69.20128624364081 s, converged=True, iterations=2,
           hits=188408, entries=26672, max speed displacement=0.0,
           max SOC displacement=0.0
quantized: 69.20126421237728 s, converged=True, iterations=2,
           hits=208073, entries=7007, max speed displacement=0.0009901704703878522,
           max SOC displacement=0.0
lap delta: 0.00002203126353 s
```

The assembled MIS reference remains `69.20128624364081 s`; the focused test confirms the quantized policy stays below the `0.01 s` criterion, remains converged, and does not exceed the motor speed limit.

Focused energy-track policy comparison (the full MIS energy solve exceeded the practical validation timeout):

```text
exact:     lap=4.066804801284017 s, final SOC=0.9915835125434573,
           converged=True, energy_converged=True
quantized: lap=4.066837845135636 s, final SOC=0.991583664729591,
           converged=True, energy_converged=True
lap delta=0.0000330438516 s, final-SOC delta=0.0000001521861
```

Both are within the requested `0.01 s` and `1e-4` thresholds.

## Commands/output

```bash
PYTHONPATH=src python3 -m unittest tests.test_point_mass tests.test_mis_regression -v
# Ran 31 tests ... OK

PYTHONPATH=src python3 -m unittest tests.test_energy_lap -v
# Ran 6 tests ... OK

PYTHONPATH=src python3 -m unittest tests.test_mis_regression.AssembledMisRegressionTests.test_quantized_cache_stays_within_mis_policy -v
# Ran 1 test ... OK
```

## Files

- `src/simplified_vehicle_models/formulations/point_mass.py`
- `src/simplified_vehicle_models/solvers/forward_backward.py`
- `src/simplified_vehicle_models/results/traces.py`
- `src/simplified_vehicle_models/__init__.py`
- `tests/test_point_mass.py`
- `tests/test_mis_regression.py`

## Self-review

- Confirmed no module-global drive cache remains.
- Confirmed exact default preserves assembled MIS behavior and Task 1 directional-limit tests.
- Confirmed cache entries are scoped to one solve and quantized displacement is measurable.
- Confirmed legacy lumped drivetrain behavior remains unchanged.
- Added lifecycle, immutability/validation, approximation, and MIS policy tests.

## Round 1 review fixes

- Fixed the build-result equilibrium evaluations to receive the same solver-local `DriveCache` used by corner, sweep, and root evaluations. Added a regression test that verifies every final equilibrium call receives that exact cache object.
- The prior 37.219 s figure combined setup and two solve paths. Individual MIS `solve_lap` wall times are now measured separately below; each is under 30 s.
- Added bounded assembled-MIS energy evidence with `maximum_iterations=1`. Neither run is claimed converged; both report the status and profile residual explicitly.

## Round 1 commands/output

Focused suites:

```bash
PYTHONPATH=src python3 -m unittest tests.test_point_mass tests.test_mis_regression -v
# Ran 33 tests in 14.879s
# OK
```

Cache propagation test:

```bash
PYTHONPATH=src python3 -m unittest tests.test_point_mass.PointMassTests.test_final_equilibrium_evaluations_use_solver_cache -v
# test_final_equilibrium_evaluations_use_solver_cache ... ok
# Ran 1 test in 0.018s
# OK
```

Separate assembled MIS `solve_lap` wall-time measurements:

```bash
# exact:     7.837250 s, lap=69.201286243641, converged=True, iterations=2, under_30=True
# quantized: 3.762589 s, lap=69.201264212377, converged=True, iterations=2, under_30=True
```

Bounded assembled MIS energy comparison (`maximum_iterations=1`):

```bash
# exact:     19.373918 s, lap=69.199296148966, lap_converged=True,
#            energy_converged=False, energy_iterations=1,
#            final_soc=0.971249884143, profile_residual=1.437866797950e-02
# quantized: 10.277722 s, lap=69.199298753958, lap_converged=True,
#            energy_converged=False, energy_iterations=1,
#            final_soc=0.971249872600, profile_residual=1.437859280210e-02
# lap delta=0.000002604992 s; final-SOC delta=0.000000011543
```

## Concerns

- The bounded assembled MIS energy comparison intentionally does not establish convergence: both energy loops stopped at one iteration and reported `energy_converged=False`, with approximately `1.44e-2` SOC profile residual.
- Full converged MIS `solve_lap_with_energy` exact-vs-quantized benchmarking belongs to Task 6.
- Speed and SOC displacement are reported as separate diagnostics because their units differ.
