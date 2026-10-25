# Task 5 Implementation Report

## Rejected implementation replaced

Replaced the previous Euler-based maneuver simulation core with segmented stiff integration through `scipy.integrate.solve_ivp(method='Radau')`. `SimulationSettings` now exposes the required `tick_s`, `rtol`, `atol`, and `max_wall_s` fields (plus an optional cancellation callback). The immutable `SimulationResult` carries time, state, state rates, requested/applied commands, observations, channels, events, execution outcome, and termination reason; numeric arrays are defensive copied and read-only, and containers are tuples (channel/event mappings are frozen).

## Behavior and outcomes

- Controller updates occur at exact `k * tick_s` times strictly before duration, independently of solver RHS evaluation count/tolerances.
- Integration is segmented at the sorted union of controller ticks, source/program breakpoints, and final time. Source-only knots refresh source commands without ticking the controller. Boundary rows are right-continuous: rates, observations, outputs, and applied commands are recomputed for the newly active command.
- Radau accepted states are recorded with matching accepted rates and commands; forward speed zero is a terminal root event.
- Cancellation and wall budget, nonfinite states, model-domain limits, generic model errors, duration, zero-speed completion, and early CSV source end are classified. CSV playback stops at the last source timestamp and does not extrapolate beyond it.
- Simulation completion is a modeled outcome, not a hardware stop trajectory.

## Tests added

`tests/test_maneuver_simulation.py` now directly covers sampled tick sequence and tolerance/RHS-count independence, CSV source knot behavior without extra controller ticks, CSV end without extrapolation, nonintegral duration ticks, wall/cancellation outcomes, model-domain classification, immutable result containers/arrays, and zero-speed root termination.

## Verification

- `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_maneuver_simulation.py' -q` — PASS, 7 tests.
- `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_maneuver_*.py' -q` — PASS, 47 tests.
- `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_forward_driver*.py' -q` — PASS, 57 tests.
- `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_maneuver_planar.py' -q` — PASS, 11 tests.
- `git diff --check` — PASS.

## Commit

`38ad61b feat: execute maneuvers with sampled controls and exact breakpoints`

## Fix round 3 — initial speed already within the stop band

- At each segment start, if longitudinal speed is strictly above the plant's actual minimum but already at or below the computed `speed_floor`, the simulator now records accepted rate/evidence for the actual state, adds a `zero_speed` event with threshold and state speed, and immediately returns `execution_outcome='stopped'` / `termination_reason='zero_speed'` without invoking `solve_ivp`.
- Speed at/below the plant's actual minimum is not accepted through this shortcut; normal model-domain behavior remains authoritative.
- Regression test uses `SpeedPlant.minimum_speed = 0.1` and initial `vx = 0.1005` (floor ~0.101), asserting stopped/zero_speed, event threshold/state speed, finite accepted rate and accepted output evidence.
- Verification: `test_maneuver_simulation.py` PASS (9 tests); all `test_maneuver_*.py` PASS (49); all `test_planar*.py` PASS (77); all `test_forward*.py` PASS (130); `git diff --check` PASS.
