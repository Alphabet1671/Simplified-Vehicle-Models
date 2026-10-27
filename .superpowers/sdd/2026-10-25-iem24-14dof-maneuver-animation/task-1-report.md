# Task 1 report — vertical maneuver adapter

## Scope
Implemented only the time-domain maneuver adapter over the existing `VerticalForwardPlant`, plus focused adapter tests. No runner, export, or hardware behavior was changed. Existing unrelated worktree changes were left untouched.

## Key decisions
- Construct an open, zero-curvature `SpeedPlan` with the configured reference length; create a fresh plan/base plant for each `initial_state` speed.
- Preserve the vertical plant's state layout and evaluator, translating initial `x/y/heading` directly to `s/n/heading_error`. Evaluation delegates to `VerticalForwardPlant.evaluate` and retains its state/domain errors.
- Convert canonical commands to the existing `DriverCommand`: calibrated accelerator force is normalized by the full calibration endpoint; brake force comes directly from the supplied calibration map; steering uses the 0.08 s servo and calibrated rate limit. `RobotCommand` itself rejects pedal overlap.
- Expose world pose, command and acceleration values, speed, vertical body channels, and four-corner wheel/jounce/tire-state channels from accepted outputs.
- Require explicit `ActuatorCalibration` and validate its accelerator endpoint against the compiled vehicle's maximum drive motor torque.

## Changed files
- `src/simplified_vehicle_models/solvers/maneuver_vertical.py` (new)
- `tests/test_maneuver_vertical.py` (new)
- `.superpowers/sdd/2026-10-25-iem24-14dof-maneuver-animation/task-1-report.md` (this report)

## Tests / outputs
- `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_vertical -v` — PASS, 3 tests.
- `PYTHONPATH=src:. python3 -m unittest tests.test_maneuver_planar tests.test_vertical_forward -q` — PASS, 38 tests.

## Commit
Pending.

## Remaining concerns
- No full repository suite was run; validation was focused on the new adapter and its planar/vertical neighbors.
- Adapter instances are intended for sequential simulation; the wrapped base plant retains its existing mutable diagnostic/evaluation state.
