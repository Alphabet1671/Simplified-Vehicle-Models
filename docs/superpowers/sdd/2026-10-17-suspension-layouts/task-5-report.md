# Task 5 report: layout-aware setup sweep and four-way report

Plan: [2026-10-17-suspension-layouts.md](../../plans/2026-10-17-suspension-layouts.md), Task 5.

## Changes

* `tools/run/sweep_vertical_setup.py` sweeps the independent-corner layout (unchanged behaviour) and the two modal layouts.
  * Modal override names: `rate_<mode>` (N/m, wheel-referred; `heave`, `roll`, `pitch`, `warp`, or `heave_front`, `roll_front`, `heave_rear`, `roll_rear`), `<mode>.click_<valve>`, bare `click_<valve>` for all four mode dampers, and `static_toe_front/rear`. A modal layout has no `arb_wheel_rate_*` or `inertance_*`, and asking for them raises the usual unknown-name error.
  * Linearization: the corner stiffness is the finite-difference Jacobian of the evaluator, as before. The corner damping matrix is `T^T diag(slope) T` for a modal layout and `diag(slope / MR^2)` for independent corners. The per-axle corner damping ratio is the secant damper force at 25 mm/s through the same matrix.
  * The hashed source list gains `layouts/modal.py` and `modal_components.py`.
* `tools/reports/compare_iem24_four_way.py` builds the suspension row through `suspension_description(...)`. The independent-corner text is byte-identical to before ("dampers (5/7 clicks)"), so the stored report does not change.
* `tests/test_vertical_setup_sweep_modal.py` (9 tests).

## Checks

* The linearized stiffness of the axle example equals the independent-corner baseline (rtol 2e-4).
* The damping block is `T^T diag(slope) T`, built by hand from the compiled dampers.
* A roll-rate override moves the roll mode and leaves heave and pitch frequencies equal to 1e-6 Hz. A heave-rate override moves heave and leaves roll alone.
* The 33 existing sweep tests and the runner tests still pass unchanged.

## Findings worth knowing

* The roll mode saturates when the roll rate rises, because the tires are in series with the roll spring. The undamped roll mode goes from 9.90 Hz at the proposed rate to 10.41 Hz at 12 times that rate, so a roll-rate sweep has a weak effect on roll frequency. A modal roll rate can be tuned to roll stiffness, not to roll frequency alone.
* On the axle example the front ride frequency matches the independent baseline (4.283 Hz). The rear does not (3.74 Hz against 3.59 Hz): a modal corner rate is the diagonal of `T^T K T`, which includes the rear ARB. The axle example reproduces the roll behaviour exactly (see Task 4) and does not reproduce the independent-corner "rear corner rate".
* The per-axle ride frequency and corner damping ratio of a modal layout use the diagonal of the stiffness and damping matrices. When the modes couple the corners (four-mode with unequal rates), these are approximations. The seven-coordinate modal frequencies are exact for the linearized model.
* The damper convention differs: a mode damper has no motion ratio. Heave of the axle example is 2 % above the independent baseline for that reason (4.072 Hz against 3.989 Hz, `|lambda|` convention).

## Not done

* No lap or acceptance evidence was regenerated. The sweep is linear analysis only.
* The step-response and frequency-response studies were run on the modal layouts only through the tests above. No committed sweep output was produced for them.
