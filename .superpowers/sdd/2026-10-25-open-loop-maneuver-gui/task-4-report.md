# Task 4 Fix Round 1 Report

## Reviewer findings addressed

- Preserved `steering_angles()` physical FL/FR channels: positive SWA/right turn has FR inner; negative SWA/left turn has FL inner. Added explicit bilateral assertions.
- Removed cached accepted-rate observation dependency. `observe(time_s, state, command)` computes dynamics from the supplied state and command; its default command is zero pedal/zero SWA. RHS trial evaluations cannot poison the observation. Added deterministic trial-state regression.
- Preserved accelerator force map semantics and made the physical contract explicit: the calibrated endpoint must equal the vehicle's maximum drive motor torque, which is the scale consumed by the existing normalized forward throttle closure. Incompatible absolute N maps are rejected by the plant rather than renormalized to an arbitrary scale.
- Validated pedal axes remain entirely in [0,1], span the range, and start at zero force.
- Added accepted output channels for longitudinal acceleration, lateral acceleration, yaw acceleration, and rotating-frame ay.

## Verification

- Task 4 test file: 10 passed.
- `test_planar_forward.py`: 9 passed.
- `test_controls.py`: 33 passed.
- All `test_maneuver*.py`: 39 passed.
- `git diff --check`: passed.

## Fix round 2 — independent planar closure comparison

Added the Task 4-required zero/small-angle comparison in `tests/test_maneuver_planar.py`. For zero and 0.01 rad SWA, the test builds an equivalent `PlanarState`, derives road-wheel angles directly from calibration geometry via `steering_angles`, maps the zero pedal travels to physical forces, constructs the matching `DriverCommand`, and obtains wheel torques through `evaluate_forward_powertrain`. Those independently computed values feed `evaluate_planar_dynamics`; its body acceleration and wheel-rate components are compared to `ManeuverPlanarPlant.evaluate`. The reference does not use plant internals or `accepted_outputs`.

The comparison is for zero pedal input. Its physical pedal maps yield zero force, matching zero throttle/brake at the forward closure; nonzero pedal-map/command equivalence is not asserted here. Cartesian world-pose rates and SWA-servo rate have no path-planar counterpart and remain excluded.

## Verification

- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_planar.py' -v` — PASS, 11 tests.
- `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_planar_forward.py' -v` — PASS, 9 tests.
- `git diff --check` — PASS.

No production code was changed, and no existing forward-run semantics were changed.
