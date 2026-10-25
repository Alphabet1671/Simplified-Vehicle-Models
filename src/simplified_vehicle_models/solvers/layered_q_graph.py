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
        for name, value in (("node_offsets", no), ("q", q), ("edge_offsets", eo), ("targets", tg), ("costs", co)):
            value.setflags(write=False)
            object.__setattr__(self, name, value)

    @property
    def n_layers(self) -> int:
        return len(self.node_offsets) - 1

    @property
    def n_nodes(self) -> int:
        return len(self.q)


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
    reach = np.zeros(graph.n_nodes, bool); reach[start] = True
    for u in range(graph.n_nodes):
        if reach[u]: reach[graph.targets[graph.edge_offsets[u]:graph.edge_offsets[u+1]]] = True
    coreach = np.zeros(graph.n_nodes, bool); coreach[list(terminals)] = True
    for u in range(graph.n_nodes - 1, -1, -1):
        a, b = graph.edge_offsets[u:u+2]
        coreach[u] |= bool(np.any(coreach[graph.targets[a:b]]))
    live = reach & coreach
    if not np.any(live):
        return _result("disconnected", [], np.inf, np.flatnonzero(reach), np.flatnonzero(coreach), 0)
    dist = np.full(graph.n_nodes, np.inf); pred = np.full(graph.n_nodes, -1, dtype=np.int64); dist[start] = 0.
    for u in range(graph.n_nodes):
        if not live[u] or not np.isfinite(dist[u]): continue
        a, b = graph.edge_offsets[u:u+2]
        for ei in range(a, b):
            v = int(graph.targets[ei])
            if not live[v]: continue
            cand = dist[u] + graph.costs[ei]
            if cand < dist[v] or (cand == dist[v] and (pred[v] < 0 or u < pred[v])):
                dist[v] = cand; pred[v] = u
    best = min(terminals, key=lambda t: (dist[t], t))
    if not np.isfinite(dist[best]):
        return _result("disconnected", [], np.inf, np.flatnonzero(reach), np.flatnonzero(coreach), 0)
    path = [best]
    while path[-1] != start:
        p = int(pred[path[-1]])
        if p < 0: return _result("disconnected", [], np.inf, np.flatnonzero(reach), np.flatnonzero(coreach), 0)
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
        return _result("disconnected", [], np.inf, [], [], 0, optimality="no matching anchored cycle in supplied graph")
    best = min(valid, key=lambda x: (x["time_seconds"], tuple(x["node_ids"])))
    best["optimality"] = "graph-optimal over enumerated matching anchors and supplied edges"
    return best
