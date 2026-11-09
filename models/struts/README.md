# struts

Physical suspension strut bundles consumed by additive suspension assemblies.

Every value in a strut file is a physical value of the real spring/damper unit, not a wheel rate. `spring_rate` is in N/m of unit travel, `preload` is in N, and the damper table is in unit velocity.

`motion_ratio` is **the travel of one wheel divided by the end-to-end travel of the unit**, in the mode that the unit resists:

| Strut | Wheel motion | Unit travel |
|---|---|---|
| corner | that wheel moves `z` | `z / MR` |
| heave | both wheels of the axle move `z` (pure heave) | `z / MR` |
| roll | one wheel moves `+z`, the other `-z` (pure roll) | `z / MR` |

A single-wheel bump of `z` is half heave and half roll, so each axle unit then moves `z / (2 MR)`. Two wheels share one heave or roll unit, so the rate at each wheel in that mode is `k / (2 MR^2)`. A roll unit gives the axle a roll stiffness of `k * half_track^2 / MR^2`. The definition is in `src/simplified_vehicle_models/suspension/layouts/parallel.py`; `tests/test_suspension_parallel.py` checks it against the active Murphy struts.

## Files

| File | Contents |
|---|---|
| `iem24_corner_zero_strut.xml` | Fully explicit disabled independent-corner physical strut. |
| `iem24_heave_front_strut.xml` | Murphy front axle heave spring, preload and heave damper. |
| `iem24_heave_rear_strut.xml` | Murphy rear axle heave spring, preload and heave damper. |
| `iem24_roll_front_strut.xml` | Murphy front axle roll spring and roll damper. |
| `iem24_roll_rear_strut.xml` | Murphy rear axle roll spring and roll damper. |
