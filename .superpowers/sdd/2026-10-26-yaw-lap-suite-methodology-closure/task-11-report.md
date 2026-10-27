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

## Completion update (2026-10-26)

Implemented `IntervalClearanceBoundProvider`, an explicit interface whose implementations bind conservative whole-subinterval clearance lower bounds to nonempty geometry and vehicle-motion model identities. `check_swept_footprint` now adaptively bisects any interval that does not receive a proven positive lower bound; absent/invalid providers, exhausted budgets, and nonpositive/unknown terminal bounds remain `unknown`. Bare numeric `interval_bounds` remain untrusted and cannot certify. The result keeps empirical sample fields separate from rigorous interval bounds and records provider/model identities and subdivision metadata. Missing body dimensions now still return the common top-level status, clearance, worst time/station, model/edge identity, and representation fields (unknown values are `None`).

Added a provider-backed analytic straight-corridor enclosure test and a constructed cubic quarter excursion tested through the production subdivision path; the latter forces subdivision and cannot pass. Kept the Task 6 branch-aware projection, empirical finite-edge unknown behavior, and no-false-certification policy.

## Verification

- Focused command: `PYTHONPATH=src python3 -m unittest tests.test_yaw_corridor_validation -q` — 12 tests passed (0.018 s), with the run governed by a 175 s Python process wait watchdog.
- No Task 7 comparator/runner artifacts were changed.

## Remaining limitation

The provider contract is intentionally narrow: mathematical soundness of a concrete provider implementation remains its implementer's responsibility. No provider for arbitrary real-track geometry or arbitrary trajectories is included; without a sound provider, swept containment remains `unknown`. No generalized footprint-containment certification is claimed.

## Re-review correction (2026-10-26)

Swept certification now requires at least one sampled interval; a one-sample trajectory remains `unknown`, records zero certified intervals, and cannot pass despite a positive provider implementation. Explicit front extent, rear extent, and half-width are also mandatory for whole-body certification: a positive interval-provider bound cannot override missing dimensions or the sampled whole-body `unknown` result. Provider identity and both model-identity strings remain reported. Results explicitly label these model identities as provider assertions and state that corridor/model compatibility has not been independently verified by the checker; a string alone is not compatibility evidence.

Added regressions for a one-sample positive provider, missing body dimensions with a positive provider, and the legitimate two-sample positive-provider path (including its provider/model provenance). Focused command `PYTHONPATH=src python3 -c "import subprocess,sys; p=subprocess.run([sys.executable,'-m','unittest','tests.test_yaw_corridor_validation','-q'],timeout=175); raise SystemExit(p.returncode)"` — 14 tests passed under the 175-second process watchdog.
