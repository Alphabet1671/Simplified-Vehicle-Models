# Task 1b report — hydraulic TV authority separation

## Change
- Added `TvLimits.max_hydraulic_force_difference`, validated finite and nonnegative, defaulting to 0.0 N.
- Hydraulic [8:12] left/right LP difference constraints now use only that dedicated wheel-force-difference limit, and only when `brake_vectoring_allowed`. The brake-bias axle-total constraint remains active.
- Motor drive vectoring continues to use only `max_delta_torque / effective_radius`.
- Updated proposal §4 to document the separate hydraulic limit and units.
- Preserved all `qss_*` files and left unrelated untracked planning work untouched.

## Focused verification
- Added a direct LP-constraint test: it captures the LP passed to the optimizer for brake vectoring enabled with 100 N·m motor authority and checks hydraulic difference inequalities are bounded at exactly 0 N for the default limit, and at 500 N for an explicit 500 N limit. This avoids relying on aggregate yaw-map results.
- Command: `PYTHONPATH=src python3 -m unittest tests.test_yaw_capability -q`
- Result: **7 tests passed**.
