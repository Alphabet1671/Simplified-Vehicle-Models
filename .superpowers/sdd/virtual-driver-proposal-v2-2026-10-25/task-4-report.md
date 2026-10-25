# Phase 4a: no-TV closed-loop validation

Implemented a focused `solvers.yaw_aware_driver` public helper. `drive_yaw_aware_plan` accepts a `YawAwareLinePlan`, forward plant, `PreviewDriver` using the plan's exact `SpeedPlan`, and initial state; it calls the shared `solve_forward` integrator and checks accepted output states against the plan corridor. The reference station is obtained from the trajectory's exact `StationMap`; signed lateral error is added to the plan's signed offset before corridor bounds are checked. Immutable `YawAwareDriveResult` contains the forward result, maximum sampled violation, worst physical path station, completion flag, sampled center-point containment, predicted duration, and optional driven lap duration/time delta.

The result deliberately distinguishes the implemented sampled evidence from full Phase-4 acceptance. `completed` is tied to the forward finish event; `driven_time` and the predicted-versus-driven comparison are `None` for incomplete runs. Containment is a sampled center-point test only. It does not claim footprint or between-step containment, periodicity, refinement, or overall Phase-4 acceptance.

Tests use a low-cost fake `ForwardPlant` for completed closed-path tracking, a duration-limited incomplete run, and geometry-only checks for contained and deliberately outside offsets. No real vehicle solve is required. The fixture uses the deterministic required seed family selected by the current Phase-3 settings.

Validation: `PYTHONPATH=src python3 -m unittest tests.test_yaw_aware_driver -v` — 3 tests passed.

Scope: no `qss_*` OCP files, FMU integration, or per-wheel torque allocation were changed. Existing unrelated working-tree changes were excluded from the commit.
