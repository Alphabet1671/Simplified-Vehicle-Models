# Stage 2 bounded line optimizer report

Implemented global profile-evaluation and wall-clock controls, exact coefficient
caching, and reference-station warm interpolation in `yaw_aware_line.py`.

`YawLineSettings` appends `maximum_profile_evaluations=24`,
`maximum_wall_seconds=900.`, and `minimum_improvement_seconds=1e-4`, without
changing existing positional fields. Caller-provided finite `[0,1]` seed blends
are retained.

Cache misses that reach a coupled profile call consume the global evaluation
budget; geometry rejections do not. Cached success and failure evaluations are
reused and reported as cache hits. Wall time is checked before evaluation, and
the remaining line budget bounds the passed coupled-profile budget. A search
that exhausts a global budget returns its strictly validated incumbent with
`budget_exceeded`, or raises a typed optimization error with the explicit cause
if no incumbent exists.

Warm suggestions are projected through each trajectory's reference station map;
closed seams are explicitly duplicated. They remain suggestions only, subject to
the coupled solver's strict validation.

Focused settings test, guarded at 120 seconds:

```sh
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 120 -- \
  python3 -m unittest tests.test_yaw_line_budget -q
```

Result: 8 focused settings, cache-equivalence, geometry-only, and behavioral tests passed in 2.267 seconds. Broader guarded functional run:

```sh
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 600 -- \
  python3 -m unittest tests.test_yaw_aware_line -q
```

Result: 6 tests passed in 80.417 seconds after QP geometry-only/shared-linearization optimization (previous guarded run was 157.908 seconds; this focused synthetic suite is not a physical performance claim).

## Focused behavioral coverage and deferred checks

`tests/test_yaw_line_budget.py` now has nine guarded tests covering settings and
seed-blend validation, a profile-evaluation cap retaining a validated incumbent,
exact duplicate-coefficient cache reuse, cached profile failure reuse, the
wall-time stop before QP construction, cached QP-linearization equivalence,
geometry-only trajectory equivalence, and seed geometry contraction. Its latest
guarded run passed in 5.428 seconds.

QP seed controls are box-bounded but a continuous spline can still overshoot a
corridor between controls. A geometry-rejected seed is therefore contracted
deterministically toward the centreline at scales `1, .5, .25, .125, .0625, 0`.
Every candidate remains subject to the unchanged dense fold/corridor gate; this
is not a corridor or feasibility relaxation. The accepted seed scale is retained
in candidate diagnostics.

Two desirable white-box spy cases remain deferred: direct assertion that every
later corner optimizer starts from the selected incumbent, and direct capture of
a warm suggestion across deliberately mismatched path sample counts. The
implementation uses `base = best_coeff` and reference-station interpolation for
those behaviors.
