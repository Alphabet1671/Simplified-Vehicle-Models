# Phase 1 task report: yaw capability map

## Delivered

- Added `formulations/yaw_capability.py` with immutable `YawCapabilityGrid` and `YawCapabilityMap` data structures, read-only defensive array copies, finite/order/shape/bounds checks, feasible-cell mask, positive explicit yaw inertia, and source provenance.
- Added physical no-TV and bounded physical-TV builder variants, plus an ideal-TV upper-bound builder. The builders solve deterministic per-cell linear programs with SciPy `linprog(method="highs")`; SciPy remains optional and a clear runtime error is raised when it is unavailable.
- Added focused unit tests for grid ordering and speed bounds, defensive read-only fields, provenance and shape/bound validation, TV limit validation, and building all three source variants with ideal-vs-physical bound checks.
- No existing `qss_*` OCP file was modified.

## Round-1 review repair

The round-2 review required force-source separation and refusal to assume a legacy drivetrain layout. The LP now represents per-wheel motor drive, regenerative braking, and hydraulic braking as distinct variables, and explicitly rejects anything other than `FullDrivetrainRuntime` with a transmission driven-wheel set. Undriven wheel motor force is fixed to zero; each driven motor gets an equal conservative share of the aggregate torque/power limit. Regen is fixed to zero until a charge-power cap is modeled, so regen-only TV cannot accidentally unlock hydraulic braking. Hydraulic braking observes brake bias and equal left/right forces unless `brake_vectoring_allowed`; that flag independently unlocks bounded hydraulic-force vectoring. `regen_vectoring_allowed` does not authorize hydraulic vectoring. Added tests verify regen-only does not change braking yaw capability and legacy drivetrain input is rejected.

The LP's remaining conservative scope is explicit: regenerative force is omitted entirely (zero constrained), and the motor envelope uses equal per-driven-wheel shares of the aggregate limit, not a detailed motor-by-motor electrical dispatch. Brake hardware pressure/thermal limits are not independently modeled.

## Test results

Initial run: `PYTHONPATH=src python3 -m unittest tests.test_yaw_capability -v` — 4 tests passed. After review repairs: the same command — 7 tests passed.

## Constraints and known limitations

The point-mass vehicle type does not carry yaw inertia, so builders require an explicit finite positive `yaw_inertia`; it participates in provenance. The implementation uses equal per-driven-wheel shares of aggregate motor torque/power capability and zero regenerative force pending an explicit charge-power limit. Hydraulic brake hardware pressure/thermal limits are not modeled. The output remains an approximate physical capability table, not a controller-calibrated map. No FMU work was done.
