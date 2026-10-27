# Task 4 report — packed graph reachability and traceback

## Implementation

Added `LayeredQGraph` as immutable packed NumPy arrays (int64 offsets/targets; float64 q/costs) in `src/simplified_vehicle_models/solvers/layered_q_graph.py`. Constructor checks CSR/layer offsets, index ranges and finite positive exact edge costs. Open/closed solve functions enforce optional global node/edge budgets before solver work and report `budget_exceeded`, rather than pruning candidates.

Open solve computes forward reachability and backward co-reachability, retains their safe intersection, and applies topological shortest-path DP using supplied edge costs. Predecessor ties resolve by ascending source node ID; terminal ties by terminal ID. Traceback follows stored predecessor edges. Results include all required fields: `status`, `node_ids`, `time_seconds`, `reachable_nodes`, `co_reachable_nodes`, `failed_layer`. Disconnection reports `disconnected` for refinement.

Closed solve enumerates explicitly supplied `(start, matching_terminal)` seam pairs and solves only each matching pair, selecting the least-time valid traceback. Diagnostics limit the optimality claim to supplied edges and enumerated anchors.

## Focused verification

Command:

```sh
PYTHONPATH=src python3 -m unittest tests.test_layered_q_graph -v
```

Output: 5 tests ran, all passed (`OK`). Coverage includes a locally-fast dead end with slower connected route, odd-cycle anchor mismatch, exhaustive tiny open and closed minimum comparisons, disconnected bands/positive-direction edges, and pre-solve node budget behavior.

## Fix round 1

Added constructor validation that every stored edge advances exactly one layer, rejecting same-layer, backward, and skipped-layer targets. Added `pack_layered_q_graph`, an edge-list factory that receives per-layer node counts and edge count before CSR allocation. If either declared count exceeds `max_nodes` / `max_edges`, it returns `budget_exceeded` with counts before allocating packed arrays. Focused command rerun:

```sh
PYTHONPATH=src python3 -m unittest tests.test_layered_q_graph -v
```

Output: 7 tests ran, all passed (`OK`), including invalid edge layer progression and factory preallocation budget behavior.

## Concerns / limits

The graph layer is synthetic and performs no physical transition construction, map access, or QSS/OCP work, as required. Exactness is only over the graph's provided directed edges and positive costs. `q` is retained as packed candidate data but does not independently establish physical feasibility. The caller must supply correctly matching seam-pair IDs and already-built graph topology. Closed traversal currently relies on the supplied graph representing the duplicated seam traversal topology; it does not create copies itself.
