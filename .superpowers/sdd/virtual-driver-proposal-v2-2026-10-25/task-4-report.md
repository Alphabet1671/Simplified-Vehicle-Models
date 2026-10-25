# Phase 4a: no-TV closed-loop validation

Implemented a focused `solvers.yaw_aware_driver` public helper. `drive_yaw_aware_plan` accepts a `YawAwareLinePlan`, forward plant, `PreviewDriver` using the plan's exact `SpeedPlan`, and initial state; it calls the shared `solve_forward` integrator and checks every accepted output state against the plan corridor. The reference station is obtained from the trajectory's exact `StationMap`, while signed lateral error is added to the plan's signed offset before corridor bounds are checked. The immutable `YawAwareDriveResult` contains the forward result, maximum violation, worst physical path station, containment flag, predicted and driven durations, and a driven-minus-predicted time property.

Tests use a low-cost fake `ForwardPlant` to validate completed closed-path tracking and include a distinct geometry-only check for both contained and deliberately outside offsets. No real vehicle solve is required. The module documents that accepted-state containment is a sampled gate, not a between-step/footprint certificate.

Validation: `PYTHONPATH=src python3 -m unittest tests.test_yaw_aware_driver -v` — 2 tests passed.

Scope: no `qss_*` OCP files, FMU integration, or per-wheel torque allocation were changed. Existing unrelated working-tree changes were excluded from the commit.
