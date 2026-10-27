# Task 11 — Footprint checks and certification limits

Implemented sampled tire-contact and oriented body-envelope checks in `yaw_corridor_validation.py`, reusing Task 6's branch-aware projection helper. Checks use actual XY/body heading, asymmetric local corridor limits, and sample polygon edges as well as corners. Missing explicit front/rear extents or half-width yields `unknown` whole-body status; wheel-contact evidence remains separate when wheelbase/track width are available.

Forward results expose empirical dense-output midpoint times/states separately from polynomial component bounds. Replay evaluates accepted and midpoint states and publishes `footprint_validation`; existing containment certification flags remain false. Edge provenance and conditionality are recorded. No legacy OCP or strict map semantics changed.

## Correction evidence (2026-10-26)

Task 11 had been reported partial. Finite body-edge samples are now empiric only: an observed negative clearance remains `fail`, while nonnegative sampled edges return `unknown` because continuous edge containment cannot be established. `check_swept_footprint` no longer certifies from bare numeric `interval_bounds`: without a binding, sound geometry/motion interval provider, these are reported as unverified metadata and status is `unknown`. Thus finer/dense sampling never sets `between_step_containment_certified=True`; adaptive interval certification remains unavailable and is explicitly disclosed rather than misrepresented.

Added tests for the quarter-step cubic excursion (endpoints and midpoint zero, quarter-step excursion 3.75 m), an inward notch crossed between sparse edge samples, and the analytic straight-corridor clearance-loss arithmetic. Existing explicit-dimensions, separate wheel/body outputs, yaw corner, pinch/asymmetric corridor tests remain.

## Verification

- Focused tests: 10 passed under a process-level 175 s watchdog (0.013 s runtime).
- Full unit suite and replay checker were not run in this correction pass.
- No OCP or map semantics changed.

## Remaining limitation

A sound adaptive interval bound for arbitrary vehicle motion and boundary geometry is not implemented. The checker correctly returns `unknown` absent such proof. No footprint-containment certification is claimed.
