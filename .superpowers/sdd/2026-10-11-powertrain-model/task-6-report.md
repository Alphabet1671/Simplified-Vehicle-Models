# Task P6 report — fixed-ratio transmission

## Status

Implemented the independent drivetrain transmission component. Powertrain and drivetrain remain separate; no vehicle or assembly wiring was added.

## Implementation

- Added `FixedGearParameters` and XML compiler in `drivetrain/transmission.py`.
- Added wheel-speed to motor-speed conversion using the fixed ratio.
- Added motor-torque to per-wheel torque conversion with directional efficiency:
  - motoring uses multiplication by efficiency;
  - regeneration uses division by efficiency.
- Added front/rear torque allocation and equal left/right side split.
- Added validation for positive ratio, efficiency in `(0, 1]`, front fraction in `[0, 1]`, and all four driven-wheel positions.
- Added `transmission/fixed-gear` registry integration and `torque-transmission` capability.
- Extended XML loading to read driven-wheel configuration.
- Added the Murphy 2024 fixed-gear XML model and focused unittest coverage.

## Tests

- Focused: `PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_transmission.py'` — 5 tests passed.
- Full required discovery: `PYTHONPATH=src python3 -m unittest discover -s tests` — 166 tests passed.

## Concerns

None.
