# Task 1 Report — Strict maneuver definition and canonical command

## Summary
Implemented a Qt-free maneuver contract package under `src/simplified_vehicle_models/controls/maneuvers/` with frozen `RobotCommand` and `ManeuverDefinition` value types; strict versioned JSON parsing; SI/unit conversion; capability-specific errors; and atomic JSON persistence. Added focused unittest coverage in `tests/test_maneuver_schema.py`.

## Contract details
- Canonical command is `(swa_rad, accelerator, brake)`; values must be finite numeric inputs, booleans are rejected, pedals are normalized to `[0, 1]`, and simultaneous accelerator/brake is rejected.
- Maneuver definitions require the full fourteen-key top-level shape and `schema: maneuver-v1`. Object keys are checked recursively, including CSV channel mappings; nested data is defensively copied at the boundary.
- Seven presets are accepted. Steering/longitudinal mode validation enforces supported choices and ownership; CSV playback requires CSV ownership of steering and longitudinal channels, prohibiting overlays across all three canonical channels.
- Timing fields, direction, and whole-number cycles are validated. Simulation can omit calibration; physical actuation and peak-ay steering require calibration and raise `CapabilityError` rather than malformed-definition errors.
- Conversion supports deg/rad, km/h/m/s, g/m/s², percent/fraction, and ms/s. JSON save uses a same-directory temporary file and atomic replacement; loading rejects malformed JSON.

## Tests and review
Command: `PYTHONPATH=src:. python3 -m unittest discover -s tests -p 'test_maneuver_schema.py' -v`
Outcome: **PASS**, 7 tests.

`git diff --check` passed. The focused tests cover the seven preset round trips, command and unit validation, unknown nested keys, timing/mode validation, calibration gates, CSV overlay rejection, defensive copies, and JSON persistence/error handling.

## Scope notes
The implementation is the contract layer only; it does not add GUI, Qt, hardware protocol, controller, or plant behavior. Planning documents already present in the worktree were left untouched.
