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
- Task 3's LUT-first forward/inverse APIs are intentionally not implemented here.

## Round 1/5 fix report

Resolved all open findings. The generator now uses the supplied XML path, hashes every consumed XML/CSV source and prints those digests, and fails on analytical monotonicity violations above `1e-9 W`; no cumulative repair is performed. Drive envelopes are selected from the largest analytical monotone envelope. Negative normalized load preserves analytical bus-power sign and carries a `regen_feasible` mask: only net-negative productive regen is invertible, while parasitic positive-draw points remain explicitly infeasible. Load zero remains the physical zero-torque anchor. Loader validation now checks zero-anchor uniqueness/tolerance, directional signs, feasible regen signs, shapes, finite values, axes, losses, monotonicity, and file digest before parsing.

The canonical artifact digest is `9cd1cd29818677e86bc60c21e36b985485fe87fffeeaae4f8b1b49cce8c17602`; regeneration to `/tmp/repeat.npz` produced the identical digest.

Exact commands and output:

```text
PYTHONPATH=src python3 tools/generate_motor_lut.py models/power-plants/amk_dd5_100c.xml models/data/amk_dd5_100c_operating_map.npz
SHA-256: 9cd1cd29818677e86bc60c21e36b985485fe87fffeeaae4f8b1b49cce8c17602
maximum monotonic repair: 0

PYTHONPATH=src python3 tools/generate_motor_lut.py models/power-plants/amk_dd5_100c.xml /tmp/repeat.npz
SHA-256: 9cd1cd29818677e86bc60c21e36b985485fe87fffeeaae4f8b1b49cce8c17602

PYTHONPATH=src python3 -m unittest tests.test_powertrain tests.test_export_data -v
Ran 55 tests in 0.528s; OK

git diff --check
passed
```
