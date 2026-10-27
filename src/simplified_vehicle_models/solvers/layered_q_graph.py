"""Packed layered graph reachability and minimum-time traceback.

The solver is graph-optimal for the supplied edges and exact edge costs; it makes
no claim about physical feasibility beyond those encoded graph edges.
"""
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class LayeredQGraph:
    node_offsets: np.ndarray
    q: np.ndarray
    edge_offsets: np.ndarray
    targets: np.ndarray
    costs: np.ndarray

    def __post_init__(self) -> None:
        no = np.asarray(self.node_offsets, dtype=np.int64)
        q = np.asarray(self.q, dtype=np.float64)
        eo = np.asarray(self.edge_offsets, dtype=np.int64)
        tg = np.asarray(self.targets, dtype=np.int64)
        co = np.asarray(self.costs, dtype=np.float64)
        if no.ndim != 1 or len(no) < 2 or no[0] != 0 or no[-1] != len(q) or np.any(np.diff(no) < 0):
            raise ValueError("invalid node_offsets")
        if eo.shape != (len(q) + 1,) or eo[0] != 0 or eo[-1] != len(tg) or np.any(np.diff(eo) < 0):
            raise ValueError("invalid edge_offsets")
        if co.shape != tg.shape or np.any(~np.isfinite(co)) or np.any(co <= 0):
            raise ValueError("edge costs must be finite and positive")
        if np.any(~np.isfinite(q)) or np.any(tg < 0) or np.any(tg >= len(q)):
            raise ValueError("invalid packed graph values")
        # Every edge must advance exactly one layer; this is required by the
        # forward/backward passes and makes the DP topological by packed ID.
        if len(tg):
            sources = np.repeat(np.arange(len(q), dtype=np.int64), np.diff(eo))
            source_layers = np.searchsorted(no, sources, side="right") - 1
            target_layers = np.searchsorted(no, tg, side="right") - 1
            if np.any(target_layers != source_layers + 1):
                raise ValueError("every edge must connect adjacent forward layers")
        for name, value in (("node_offsets", no), ("q", q), ("edge_offsets", eo), ("targets", tg), ("costs", co)):
            value.setflags(write=False)
            object.__setattr__(self, name, value)

    @property
    def n_layers(self) -> int:
        return len(self.node_offsets) - 1

    @property
    def n_nodes(self) -> int:
        return len(self.q)


def pack_layered_q_graph(node_counts: np.ndarray, q_values: np.ndarray, edge_sources: np.ndarray,
                         targets: np.ndarray, costs: np.ndarray, *, max_nodes: int, max_edges: int) -> LayeredQGraph | dict:
    """Pack edge-list inputs after checking global budgets, before CSR allocation.

    Returns ``{"status": "budget_exceeded"}`` when the declared node/edge
    counts exceed limits. Input edge-list arrays are caller-owned; no packed
    graph arrays are allocated on that path.
    """
    counts = np.asarray(node_counts)
    if counts.ndim != 1 or counts.size < 2 or np.any(counts < 0):
        raise ValueError("node_counts must contain nonnegative per-layer counts")
    nnodes = int(np.sum(counts, dtype=np.int64))
    nedge = len(targets)
    if nnodes > max_nodes or nedge > max_edges:
        return {"status": "budget_exceeded", "node_count": nnodes, "edge_count": nedge}
    q = np.asarray(q_values, dtype=np.float64)
    src = np.asarray(edge_sources, dtype=np.int64)
    tgt = np.asarray(targets, dtype=np.int64)
    cost = np.asarray(costs, dtype=np.float64)
    if q.shape != (nnodes,) or src.shape != tgt.shape or cost.shape != tgt.shape:
        raise ValueError("inconsistent packed graph input lengths")
    offsets = np.concatenate(([0], np.cumsum(counts, dtype=np.int64)))
    if np.any(src < 0) or np.any(src >= nnodes):
        raise ValueError("invalid edge source")
    order = np.argsort(src, kind="stable")
    sorted_src = src[order]
    edge_offsets = np.empty(nnodes + 1, dtype=np.int64)
    edge_offsets[0] = 0
    np.cumsum(np.bincount(sorted_src, minlength=nnodes), out=edge_offsets[1:])
    return LayeredQGraph(offsets, q, edge_offsets, tgt[order], cost[order])


def _disconnect_cut(graph: LayeredQGraph, reach: np.ndarray,
                    coreach: np.ndarray) -> tuple[int, int] | None:
    reachable_layers = [k for k in range(graph.n_layers)
                        if np.any(reach[graph.node_offsets[k]:graph.node_offsets[k + 1]])]
    coreachable_layers = [k for k in range(graph.n_layers)
                         if np.any(coreach[graph.node_offsets[k]:graph.node_offsets[k + 1]])]
    if not reachable_layers or not coreachable_layers:
        return None
    left, right = max(reachable_layers), min(coreachable_layers)
    return (left, right) if left < right else None


def _result(status: str, path: list[int], time: float, reach: np.ndarray, coreach: np.ndarray, failed: int | None, **extra: Any) -> dict:
    return dict(status=status, node_ids=np.asarray(path, dtype=np.int64), time_seconds=float(time),
                reachable_nodes=np.asarray(reach, dtype=np.int64), co_reachable_nodes=np.asarray(coreach, dtype=np.int64),
                failed_layer=failed, **extra)


def _budgets(graph: LayeredQGraph, max_nodes: int | None, max_edges: int | None) -> bool:
    return (max_nodes is None or graph.n_nodes <= max_nodes) and (max_edges is None or len(graph.targets) <= max_edges)


def _solve(graph: LayeredQGraph, start: int, terminals: set[int], max_nodes: int | None, max_edges: int | None) -> dict:
    if not _budgets(graph, max_nodes, max_edges):
        return _result("budget_exceeded", [], np.inf, [], [], None)
    if not 0 <= start < graph.n_nodes or not terminals or any(t < 0 or t >= graph.n_nodes for t in terminals):
        raise ValueError("invalid start or terminal node")
    if np.searchsorted(graph.node_offsets, start, side="right") - 1 != 0:
        raise ValueError("start_node must be in first layer")
    if any(np.searchsorted(graph.node_offsets, t, side="right") - 1 != graph.n_layers - 1 for t in terminals):
        raise ValueError("terminal nodes must be in last layer")
    # Sources are invariant for this solve.  Layer-wise vector reductions keep
    # exactly the scalar graph semantics while avoiding a Python visit per edge.
    sources = np.repeat(np.arange(graph.n_nodes, dtype=np.int64), np.diff(graph.edge_offsets))
    reach = np.zeros(graph.n_nodes, bool); reach[start] = True
    for layer in range(graph.n_layers - 1):
        first, last = graph.node_offsets[layer:layer+2]
        edge_first, edge_last = graph.edge_offsets[first], graph.edge_offsets[last]
        selected = reach[sources[edge_first:edge_last]]
        if np.any(selected):
            reach[graph.targets[edge_first:edge_last][selected]] = True
    coreach = np.zeros(graph.n_nodes, bool); coreach[list(terminals)] = True
    for layer in range(graph.n_layers - 2, -1, -1):
        first, last = graph.node_offsets[layer:layer+2]
        edge_first, edge_last = graph.edge_offsets[first], graph.edge_offsets[last]
        target_live = coreach[graph.targets[edge_first:edge_last]]
        if np.any(target_live):
            coreach[sources[edge_first:edge_last][target_live]] = True
    live = reach & coreach
    if not np.any(live):
        reach_nodes = np.flatnonzero(reach)
        coreach_nodes = np.flatnonzero(coreach)
        return _result("disconnected", [], np.inf, reach_nodes, coreach_nodes, 0,
                       disconnect_cut=_disconnect_cut(graph, reach, coreach))
    dist = np.full(graph.n_nodes, np.inf); pred = np.full(graph.n_nodes, -1, dtype=np.int64); dist[start] = 0.
    for layer in range(graph.n_layers - 1):
        first, last = graph.node_offsets[layer:layer+2]
        edge_first, edge_last = graph.edge_offsets[first], graph.edge_offsets[last]
        src = sources[edge_first:edge_last]
        target = graph.targets[edge_first:edge_last]
        keep = live[src] & live[target] & np.isfinite(dist[src])
        if not np.any(keep):
            continue
        src, target = src[keep], target[keep]
        candidate = dist[src] + graph.costs[edge_first:edge_last][keep]
        # First reduce costs, then explicitly pick smallest source ID among
        # equal minima, matching scalar `cand == dist[v] and u < pred[v]`.
        next_first, next_last = graph.node_offsets[layer+1:layer+3]
        width = next_last-next_first
        local_target = target-next_first
        minimum = np.full(width, np.inf)
        np.minimum.at(minimum, local_target, candidate)
        winners = np.full(width, np.iinfo(np.int64).max, dtype=np.int64)
        winning = candidate == minimum[local_target]
        np.minimum.at(winners, local_target[winning], src[winning])
        active = np.isfinite(minimum)
        nodes = next_first + np.flatnonzero(active)
        dist[nodes] = minimum[active]
        pred[nodes] = winners[active]
    best = min(terminals, key=lambda t: (dist[t], t))
    if not np.isfinite(dist[best]):
        reach_nodes = np.flatnonzero(reach)
        coreach_nodes = np.flatnonzero(coreach)
        return _result("disconnected", [], np.inf, reach_nodes, coreach_nodes, 0,
                       disconnect_cut=_disconnect_cut(graph, reach, coreach))
    path = [best]
    while path[-1] != start:
        p = int(pred[path[-1]])
        if p < 0:
            reach_nodes = np.flatnonzero(reach)
            coreach_nodes = np.flatnonzero(coreach)
            return _result("disconnected", [], np.inf, reach_nodes, coreach_nodes, 0,
                           disconnect_cut=_disconnect_cut(graph, reach, coreach))
        path.append(p)
    path.reverse()
    return _result("ok", path, dist[best], np.flatnonzero(reach), np.flatnonzero(coreach), None,
                   optimality="graph-optimal for supplied edges and exact costs")


def solve_open_graph(graph: LayeredQGraph, start_node: int, terminal_nodes: np.ndarray, *, max_nodes: int | None = None, max_edges: int | None = None) -> dict:
    return _solve(graph, int(start_node), set(map(int, np.asarray(terminal_nodes).ravel())), max_nodes, max_edges)


def solve_closed_graph(graph: LayeredQGraph, seam_pairs: np.ndarray, *, max_nodes: int | None = None, max_edges: int | None = None) -> dict:
    """Find minimum matching-anchor cycle; seam_pairs columns are start, terminal."""
    if not _budgets(graph, max_nodes, max_edges):
        return _result("budget_exceeded", [], np.inf, [], [], None)
    pairs = np.asarray(seam_pairs, dtype=np.int64)
    if pairs.ndim != 2 or pairs.shape[1] != 2: raise ValueError("seam_pairs must have shape (n, 2)")
    answers = [_solve(graph, int(a), {int(b)}, max_nodes, max_edges) for a, b in pairs]
    valid = [x for x in answers if x["status"] == "ok"]
    if not valid:
        if any(x["status"] == "budget_exceeded" for x in answers): return _result("budget_exceeded", [], np.inf, [], [], None)
        reach = np.unique(np.concatenate([x["reachable_nodes"] for x in answers])) if answers else np.array([], dtype=np.int64)
        coreach = np.unique(np.concatenate([x["co_reachable_nodes"] for x in answers])) if answers else np.array([], dtype=np.int64)
        reach_mask = np.zeros(graph.n_nodes, dtype=bool); reach_mask[reach] = True
        coreach_mask = np.zeros(graph.n_nodes, dtype=bool); coreach_mask[coreach] = True
        return _result("disconnected", [], np.inf, reach, coreach, 0,
                       disconnect_cut=_disconnect_cut(graph, reach_mask, coreach_mask),
                       optimality="no matching anchored cycle in supplied graph")
    best = min(valid, key=lambda x: (x["time_seconds"], tuple(x["node_ids"])))
    best["optimality"] = "graph-optimal over enumerated matching anchors and supplied edges"
    return best
