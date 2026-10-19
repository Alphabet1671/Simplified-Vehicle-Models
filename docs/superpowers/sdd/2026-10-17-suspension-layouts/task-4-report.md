# Task 4 report: stateless modal layouts

Plan: [2026-10-17-suspension-layouts.md](../../plans/2026-10-17-suspension-layouts.md), Task 4.

## What was added

| Item | File |
|---|---|
| Modal force network, mode matrices, orthonormality check | `src/simplified_vehicle_models/suspension/layouts/modal.py` |
| Axle roll/heave layout | `src/simplified_vehicle_models/suspension/layouts/axle_roll_heave.py` |
| Four-mode layout (heave, roll, pitch, warp) | `src/simplified_vehicle_models/suspension/layouts/four_mode.py` |
| Component registration | `src/simplified_vehicle_models/suspension/modal_components.py`, imported by `compiler/registry.py` |
| `spring-model/mode-rate`, `damper-model/table-mode`, `suspension-assembly/axle-roll-heave` and `/four-mode` | registered in `modal_components.py` |
| Example XML | `models/suspensions/iem24_axle_roll_heave.xml`, `iem24_four_mode.xml`, `models/springs/iem24_mode_*.xml`, `iem24_fourmode_*.xml`, `models/dampers/multimatic_dssv_vc01_mode.xml` |
| Tests | `tests/test_suspension_layouts.py` (29) |

`F_corner = T^T F_mode(T z, T zdot)` with the orthonormal `T` of the plan. The per-axle layout uses the 2x2 version on each axle, with `1/sqrt(2)` rows. Zero jounce carries the sprung static loads as a preload taken from the kinematics, so the mode springs act about the design ride height.

## Plan checks (a) to (f)

All six are tests: equal rates reproduce independent springs (a); a pure mode produces force in that mode only (b); a single wheel displacement gives `T^T K T`, plus a virtual-work gradient check (c); zero jounce carries the static load (d); a roll rate of `k_corner + k_arb` reproduces the independent-corner springs and ARB (e); a non-orthonormal `T`, a negative rate (named by mode), a non-positive heave rate and a non-finite rate all fail at compile time (f).

Compile errors for the nested roles come from Task 2: a missing child role and a child of the wrong kind (a damper in a spring role) are tested on the shipped XML.

## Short runs (plan step 5)

Constant-throttle 60 m trial at 20 m/s, BDF, `rtol` 1e-6, `maximum_step` 0.005 s:

| Layout | Lap completed | Out of range | Lifted | Min contact load | Jounce range |
|---|---|---|---|---|---|
| independent-corner | yes | 0 | 0 | 786.4 N | 1.92 to 3.70 mm |
| axle-roll-heave | yes | 0 | 0 | 786.4 N | 1.92 to 3.70 mm |
| four-mode | yes | 0 | 0 | 786.4 N | 2.16 to 3.25 mm |

The axle layout reproduces the independent-corner trial to the printed digits, as the equivalent-parameter design intends. The four-mode example uses different PROPOSED rates and so differs. The equilibrium solve runs as part of `initial_state` in the same trial.

## Decisions to flag

* **Mode dampers use the wheel-referred mode velocity with no motion ratio** (plan step 3 default, Q3). A damper table is therefore read at `T zdot`, not at damper-shaft velocity as in the independent-corner layout. The user was asked and did not answer. The reported `damper_velocity` of a modal network is the largest absolute mode velocity, copied to all four corners, so the run gates' damper-table headroom check stays conservative.
* **The mode springs are linear.** There is no bump or droop stop in a modal layout. Modal decoupling is exact only for linear elements, so a nonlinear mode element would be a modelling choice. The travel limits stay in the kinematics and the run gates still check them. Plan assumption A8.
* **PROPOSED values:** every `iem24_fourmode_*` rate (mean corner rate, corner rate plus half the rear ARB, corner rate, half the corner rate) and the mode damper clicks 5 and 7. The `iem24_mode_*` axle rates are not proposed: they are the selected IEM24 corner rates, with the rear ARB folded into the rear roll rate.

## Not done here

* The run result's trace has no per-element columns today, so the plan's "variable element list in the run result" needed no change. The element dictionary of `NetworkForces` is available to any future reader.
* The runner's hashed source list does not yet include the modal layout files, because editing the runner would invalidate the regenerated evidence. This is done together with the final regeneration after Task 6.
