# Task 11 — Footprint checks and certification limits

Implemented sampled tire-contact and oriented body-envelope checks in `yaw_corridor_validation.py`, reusing Task 6's branch-aware projection helper. Checks use actual XY/body heading, asymmetric local corridor limits, and sample polygon edges as well as corners. Missing explicit front/rear extents or half-width yields `unknown` whole-body status; wheel-contact evidence remains separate when wheelbase/track width are available.

Forward results expose empirical dense-output midpoint times/states separately from polynomial component bounds. Replay evaluates accepted and midpoint states and publishes `footprint_validation`; existing containment certification flags remain false. Edge provenance and conditionality are recorded. No legacy OCP or strict map semantics changed.

`check_swept_footprint` separates empirical samples from caller-supplied rigorous interval lower bounds. It only considers interval certification when all supplied lower bounds are positive and sampled body status passes. Missing bounds, unavailable dimensions, or exhausted budget prevent certification.

## Verification

- Focused tests: 7 passed under a 175 s process watchdog (0.013 s runtime).
- Python compilation of changed modules passed.
- No physical replay performed.

## Limitations / incomplete requirements

- Sound conservative interval lower bounds are not computed by this implementation; they must be supplied by callers, otherwise result is `unknown`.
- Body-edge crossing uses finite edge samples, not exact segment/curve intersection; narrow notches may be missed.
- Required quarter-step adversarial cubic excursion and analytic bound-arithmetic tests were not implemented. Consequently no sampled result is promoted to between-step certification.
