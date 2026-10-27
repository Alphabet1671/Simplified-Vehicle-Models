# Forward integration failure diagnostics

## Scope

Added a typed unsuccessful-IVP error without changing vehicle physics,
integration acceptance, or OCP behavior.

## Interface

`ForwardIntegrationError` subclasses `RuntimeError` for compatibility. When
`solve_ivp` returns `success=False`, `solve_forward` raises it with:

- readonly `last_state`, copied from the final solver-accepted state;
- `last_time` from the final accepted solver time;
- bounded scalar diagnostics: solver message, failure kind, accepted step count,
  RHS evaluations, rejected domain trials, duration, wall budget, method, and
  elapsed wall time.

If an unsuccessful result has no accepted history, the error uses the initial
state and time zero. Failure reporting does not re-evaluate the plant and does
not produce a `ForwardResult`, so no unvalidated state is represented as a
successful simulation.

Timeout behavior remains the existing `TimeoutError` path in this minimal
change.

## Verification

```sh
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 120 -- \
  python3 -m unittest tests.test_forward_failure_diagnostics -q
```

Result: 3 tests passed in 1.648 seconds. Tests mock unsuccessful SciPy IVP
results and verify RuntimeError compatibility, readonly final-state snapshots,
empty-history initial fallback, and bounded diagnostic fields.
