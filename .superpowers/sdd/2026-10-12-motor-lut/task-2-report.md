# Task 2 Report

## Status
Implemented deterministic motor LUT generation and XML-declared loading. The production compiler now requires and validates the declared operating-map NPZ; the analytical evaluator is retained as `evaluate_shaft_power_exact` and the existing `evaluate_shaft_power` name remains an alias.

## Commit(s)
- `f361ffb feat: compile motor operating LUT`

## Files and artifact
- `tools/generate_motor_lut.py` — deterministic generator and SHA-256 reporting.
- `src/simplified_vehicle_models/powertrain/power_plant.py` — `MotorOperatingMap`, digest/shape/value/axis/monotonic validation, immutable arrays, analytical construction path, exact evaluator name.
- `models/power-plants/amk_dd5_100c.xml` — canonical `operating-map` declaration.
- `models/data/amk_dd5_100c_operating_map.npz` — 1,011,135 bytes; SHA-256 `15dc8ca95cab3194003bc138af172336321c9e9a10281987694a96e1afd5f20c`.
- `models/data/provenance.md` — artifact metadata and generation record.
- `tests/test_powertrain.py` — canonical-map-aware temporary motor fixture.

Artifact arrays: voltage `(9,)`, speed `(129,)`, signed load `(65,)`, drive/regen torque `(9,129)`, bus/loss power `(9,129,65)`.

## Exact commands/results
1. `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 tools/generate_motor_lut.py models/power-plants/amk_dd5_100c.xml models/data/amk_dd5_100c_operating_map.npz`
   - Success; reported SHA-256 `15dc8ca95cab3194003bc138af172336321c9e9a10281987694a96e1afd5f20c`.
2. Same generator command with output `/tmp/amk_repeat.npz`
   - Success; reported the identical SHA-256.
3. `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest tests.test_powertrain tests.test_export_data -v`
   - **51 tests, OK**.
4. `PYTHONPATH=src /Users/alphabet1671/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/bin/python3 -m unittest discover -s tests -v`
   - **218 tests, OK**.
5. `git diff --check`
   - Passed.

## Reproducibility
Regeneration in a separate output produced the same SHA-256 digest and stable array shapes. The generator uses fixed `float64` axes, stable NPZ key order, deterministic bracket iterations, and compressed NumPy output.

## Self-review
- Digest verification occurs before NumPy parsing.
- Loaded arrays are copied to contiguous `float64` storage and marked read-only.
- Validation covers required keys, dimensions/shapes, finite values, strictly increasing axes, normalized load bounds, nonnegative envelopes/loss, directional bus-power monotonicity, and digest mismatch.
- Canonical XML and artifact are committed together.
- Exact analytical behavior remains available without LUT via the private generator-only source construction path.

## Concerns
- The source analytical loss model is not strictly monotone in signed load near zero for every node. Generation applies cumulative monotonic repair; the recorded maximum repair is **1563.3094661356702 W**, materially above the brief's `1e-9` floating-noise threshold. This is documented in provenance and should be reviewed before Task 3 treats the repaired table as a physics-accurate inverse map.
- Task 3's LUT-first forward/inverse APIs are intentionally not implemented here.
