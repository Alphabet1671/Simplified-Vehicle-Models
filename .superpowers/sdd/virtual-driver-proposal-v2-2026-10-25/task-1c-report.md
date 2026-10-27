# Task 1c report — hydraulic TV brake-bias preservation

## Review correction
- Preserved the front/rear axle-total brake-bias equality for every physical braking cell, including when hydraulic brake vectoring is enabled. Hydraulic vectoring only permits bounded left/right force differences per axle under `max_hydraulic_force_difference`.
- The same equality is retained for ideal-TV cells as the safest interpretation; no ideal-map omission is documented or permitted by the LP builder.
- Added direct LP regression assertions that the equality row is present in `A_eq` with `brake_vectoring_allowed=True`, alongside the existing force-limit check.
- Retained the hydraulic separation check: with a zero hydraulic limit inequalities are bounded at 0 N; with an explicit 500 N limit they are bounded at 500 N, independent of 100 N·m motor authority.
- Preserved all `qss_*` files and left unrelated uncommitted line/driver work untouched.

## Focused verification
- Command: `PYTHONPATH=src python3 -m unittest tests.test_yaw_capability -q`
- Result: **7 tests passed**.
