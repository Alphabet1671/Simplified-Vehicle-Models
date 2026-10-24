# Task 4k completion report

## Scope

Implemented the opt-in `physical_units_chain_v1` transcription and explicit physical initial-guess handling in the two authorized files only: `src/simplified_vehicle_models/solvers/standing_straight_ocp.py` and `tests/test_standing_straight_ocp.py`. The default remains `legacy_normalized_fd_v1`; its normalized equality, default finite-difference behavior, bounds, and ftol scaling are retained. The physical mode uses an equality in metres, analytic objective/equality/kinematic chain derivatives, and finite-difference cap derivatives through the globally budgeted/cached callback. Derivative callback probes are included in `limit_queries` and counted in the derivative subset. Rest-speed cap differentiation uses a forward probe at nonzero future jet and does not probe negative speeds.

Explicit initial guesses are copied and validated as finite real 1-D arrays of length `2N`, with duration bounds and nonnegative accelerations checked before callback/solver use. Reports include seed source, canonical float64 SHA-256, and the rollout finish-distance residual when a seed is supplied. No model, physical tolerance, event qualification, acceptance gate, public model, or global-feasibility claim was changed.

## Verification

- Pinned runtime `python -m py_compile src/simplified_vehicle_models/solvers/standing_straight_ocp.py`: passed.
- Pinned runtime combined focused and adjacent tests (`test_standing_straight_ocp`, `test_standing_launch`, `test_standing_event`, `test_live_standing_profile`, `test_open_standing_start`): 61 tests passed in 18.702 s.
- Review follow-up tests verify omitted versus explicit legacy mode callback/objective/equality/inequality values, bounds/options, default seed, and absent Jacobians; physical equality is compared in metres against legacy normalized equality, with matching objective, inequalities, bounds, and a provided seed.
- Review follow-up tests verify provided-seed hash/residual and copy isolation in both modes (including optimizer mutation and bypass of an otherwise-invalid default seed), plus rejection of invalid mode and malformed seed types/shapes/bounds before callback or optimizer entry.
- Constant-cap analytic optimum is exercised for N=4 and N=8 in both transcription modes; all reported acceptance/qualification flags remain false as required for this restricted solver.

## Limitations

The physical mode still uses finite differences for the injected acceleration-cap function, so its derivative is approximate. Feasibility checks remain finite live/held-out samples and are not continuous certificates. The solver remains fixed-straight only, without joint line optimization, event/U3 qualification, or a global-optimum proof. The reported 75.3 m example includes the staging 0.3 m and is not the official timed 75.0 m segment.

## Status

DONE_WITH_CONCERNS: implementation and scoped regression tests pass; no broader validation or qualification claim is made.
