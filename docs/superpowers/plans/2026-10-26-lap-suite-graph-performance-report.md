# Stage 1b graph-DP performance report

## Scope

Optimized the existing packed layered graph dynamic program only. Public graph
APIs, edge construction, graph feasibility cuts, and graph-optimality semantics
are unchanged.

## Implementation

`_solve` now precomputes CSR edge sources once per solve. Reachability and
co-reachability propagate by layer through NumPy indexed assignments. The
minimum-time pass operates layer-by-layer with `numpy.minimum.at` on existing
edge costs. A second indexed minimum reduction chooses the smallest packed
source ID among exactly equal target minima without a per-target Python loop.

Equal-cost predecessor behavior remains deterministic and matches the former
scalar update rule: after reducing each target's minimum cost, the solver
selects the smallest packed source node ID among sources with that exact cost.
Traceback and final terminal tie selection remain unchanged.

## Verification

Guarded focused run:

```sh
PYTHONPATH=src:. python3 tools/run/run_with_deadline.py --seconds 120 -- \
  python3 -m unittest tests.test_layered_q_graph -q
```

Result: 9 tests passed in 0.045 seconds.

The new seeded randomized regression constructs 50 small layered graphs with
empty/disconnected edge patterns and exact equal-cost ties. It compares the
vectorized solver to an in-test scalar reference for reachable nodes,
co-reachable nodes, disconnection status, and exact traceback node IDs. Existing
open/closed exhaustive minimum and budget tests also pass.

## Limits

No timing threshold is asserted in unit tests. The intended physical benchmark
remains a guarded accepted-map profile probe; this change does not make any
claim about map feasibility, continuous optimality, or line-optimization
convergence.
