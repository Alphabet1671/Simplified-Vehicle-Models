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

## Notes

Zero/small-angle comparison against existing planar evaluation was not added in this fix round; existing tests verify Cartesian heading pose rates and body acceleration identity. No existing forward-run semantics were changed.
