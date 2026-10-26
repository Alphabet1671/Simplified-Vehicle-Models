# Final reliability status documentation report

## Scope

Curated developer-status documentation only:

- modified `docs/dev/no-tv-minimum-lap-reliability.md`;
- created `docs/dev/no-tv-reliability-results-2026-10-22.md`.

No solver, tests, capability archive, source XML/CSV, cache, torque-vectoring
logic, run artifact, history rewrite, merge, or push was changed. Local
`.superpowers/` and `runs/` artifacts remain ignored; the developer documents
name them only as reproducibility references and preserve the source-hash values
in the curated text.

## Content delivered

The updated developer guide now documents the reviewed opt-in metric-consistent
energy-rate control, conservative speed-domain margin, explicit runtime and
provenance behavior, and legacy-default handling. It states that CLI
geometry/state budgets are diagnostic only and cannot yield strict acceptance
without an approved source-hashed policy.

The dated result ledger distinguishes narrow achieved rolling-acceleration
checks from unqualified curved cases. It records the requested runtime-38
artifacts, including timeouts and dense failures; marks pre-fix new-mode
qualification INVALID; gives the 37-test frozen and focused/integration test
outcomes; and explicitly says neither global study completion nor an
all-repository-green status is claimed.

## Actual outstanding work

The full six-case registered matrix, dense/live-feasible curved refinements,
independent mesh/basis/seed/reference evidence, approved convergence policy,
and standing-start treatment remain incomplete. The `0.05 m/s` margin
sensitivity is documented as a strict NLP success followed by dense
acceleration-bound failures; it is neither feasible nor an unrestricted
original-problem optimum.

## Verification

```bash
PYTHONPATH=src:/tmp/no-tv-casadi-runtime python3 -m unittest \
  tests.test_repository_layout tests.test_track_gui_docs \
  tests.test_vehicle_gui_docs tests.test_anim_gui_docs -v
```

Passed: **21 tests in 0.734 s**.

The documentation change is intended for commit as explicit paths only:

```bash
git add docs/dev/no-tv-minimum-lap-reliability.md \
        docs/dev/no-tv-reliability-results-2026-10-22.md \
        docs/superpowers/reports/2026-10-22-no-tv-lap-reliability/task-final-status-docs-report.md
git commit -m "docs: record no-TV reliability status"
```
