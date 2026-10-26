import itertools
import unittest

import numpy as np

from simplified_vehicle_models.solvers.layered_q_graph import (LayeredQGraph, pack_layered_q_graph, solve_closed_graph, solve_open_graph)


def graph(layers, edges):
    offsets = np.cumsum([0] + [len(x) for x in layers], dtype=np.int64)
    q = np.concatenate([np.asarray(x, float) for x in layers])
    rows = [[] for _ in q]
    for u, v, c in edges: rows[u].append((v, c))
    eo = [0]; targets = []; costs = []
    for row in rows:
        for v, c in row: targets.append(v); costs.append(c)
        eo.append(len(targets))
    return LayeredQGraph(offsets, q, eo, targets, costs)


def scalar_open(graph_obj, start, terminals):
    reach=np.zeros(graph_obj.n_nodes,bool); reach[start]=True
    for u in range(graph_obj.n_nodes):
        if reach[u]: reach[graph_obj.targets[graph_obj.edge_offsets[u]:graph_obj.edge_offsets[u+1]]]=True
    coreach=np.zeros(graph_obj.n_nodes,bool); coreach[list(terminals)]=True
    for u in range(graph_obj.n_nodes-1,-1,-1):
        a,b=graph_obj.edge_offsets[u:u+2]; coreach[u]|=bool(np.any(coreach[graph_obj.targets[a:b]]))
    live=reach&coreach; dist=np.full(graph_obj.n_nodes,np.inf); pred=np.full(graph_obj.n_nodes,-1,dtype=np.int64);dist[start]=0.
    for u in range(graph_obj.n_nodes):
        if not live[u] or not np.isfinite(dist[u]): continue
        for edge in range(graph_obj.edge_offsets[u],graph_obj.edge_offsets[u+1]):
            v=int(graph_obj.targets[edge]); candidate=dist[u]+graph_obj.costs[edge]
            if live[v] and (candidate<dist[v] or (candidate==dist[v] and (pred[v]<0 or u<pred[v]))):dist[v]=candidate;pred[v]=u
    best=min(terminals,key=lambda node:(dist[node],node))
    if not np.isfinite(dist[best]): return None,reach,coreach
    path=[best]
    while path[-1]!=start:path.append(int(pred[path[-1]]))
    return path[::-1],reach,coreach


class PackedGraphTests(unittest.TestCase):
    def test_vectorized_dp_matches_scalar_random_graphs_and_ties(self):
        rng=np.random.default_rng(26)
        for _ in range(50):
            counts=np.array([1, *[rng.integers(0,5) for _ in range(2)], 1])
            offsets=np.r_[0,np.cumsum(counts)]; edges=[]
            for layer in range(3):
                for source in range(offsets[layer],offsets[layer+1]):
                    for target in range(offsets[layer+1],offsets[layer+2]):
                        if rng.random()<.55: edges.append((source,target,float(rng.integers(1,4))))
            g=graph([list(range(offsets[i],offsets[i+1])) for i in range(4)],edges)
            terminal=list(range(offsets[-2],offsets[-1]))
            expected,reach,coreach=scalar_open(g,0,terminal)
            actual=solve_open_graph(g,0,np.array(terminal))
            np.testing.assert_array_equal(actual["reachable_nodes"],np.flatnonzero(reach))
            np.testing.assert_array_equal(actual["co_reachable_nodes"],np.flatnonzero(coreach))
            if expected is None:self.assertEqual(actual["status"],"disconnected")
            else:self.assertEqual(actual["node_ids"].tolist(),expected)

    def test_dead_end_local_fast_route_and_traceback(self):
        g = graph([[0], [1, 2], [3]], [(0, 1, 1), (0, 2, 2), (2, 3, 1)])
        r = solve_open_graph(g, 0, np.array([3]))
        self.assertEqual(r['status'], 'ok')
        self.assertEqual(r['node_ids'].tolist(), [0, 2, 3])
        self.assertEqual(r['time_seconds'], 3)
        self.assertEqual(r['reachable_nodes'].tolist(), [0, 1, 2, 3])
        self.assertEqual(r['co_reachable_nodes'].tolist(), [0, 2, 3])

    def test_odd_cycle_local_domains_do_not_imply_matching_cycle(self):
        # Three seam anchors have edges only to the next anchor; each layer has
        # local values, but no start copy reaches its own terminal copy.
        g = graph([[0, 1, 2], [3, 4, 5]], [(0, 4, 1), (1, 5, 1), (2, 3, 1)])
        r = solve_closed_graph(g, np.array([[0, 3], [1, 4], [2, 5]]))
        self.assertEqual(r['status'], 'disconnected')

    def test_open_and_closed_minima_match_exhaustive(self):
        g = graph([[0], [1, 2], [3, 4]], [(0,1,3),(0,2,1),(1,3,1),(1,4,5),(2,3,4),(2,4,2)])
        costs = {(u,v):c for u,v,c in [(0,1,3),(0,2,1),(1,3,1),(1,4,5),(2,3,4),(2,4,2)]}
        paths = [[0,b,c] for b,c in itertools.product([1,2],[3,4]) if (0,b) in costs and (b,c) in costs]
        expected = min(sum(costs[a,b] for a,b in zip(p,p[1:])) for p in paths)
        self.assertEqual(solve_open_graph(g, 0, np.array([3,4]))['time_seconds'], expected)
        cg = graph([[0,1], [2,3], [4,5]], [(0,2,2),(0,3,1),(1,2,1),(1,3,2),(2,4,2),(2,5,5),(3,4,4),(3,5,1)])
        pair_cost = {(u,v):c for u,v,c in [(0,2,2),(0,3,1),(1,2,1),(1,3,2),(2,4,2),(2,5,5),(3,4,4),(3,5,1)]}
        anchors = [(0,4),(1,5)]
        expected_closed = min(pair_cost[a,b]+pair_cost[b,c] for a,c in anchors for b in [2,3] if (a,b) in pair_cost and (b,c) in pair_cost)
        self.assertEqual(solve_closed_graph(cg, np.array(anchors))['time_seconds'], expected_closed)

    def test_disconnected_graph_reports_actual_reachability_cut(self):
        # Four layers. Start reaches layers 0 and 1; terminals co-reach layers 2
        # and 3. No edge crosses 1 -> 2.
        packed = pack_layered_q_graph(
            np.array([1, 1, 1, 1]), np.array([0., 1., 2., 3.]),
            np.array([0, 2]), np.array([1, 3]), np.array([1., 1.]),
            max_nodes=4, max_edges=2,
        )
        result = solve_open_graph(packed, 0, np.array([3]))
        self.assertEqual(result["status"], "disconnected")
        self.assertEqual(result["disconnect_cut"], (1, 2))

    def test_disconnected_bands_and_positive_acceleration_edges_are_preserved(self):
        g = graph([[0,1], [2,3], [4,5]], [(0,2,1),(1,3,1),(2,4,1),(3,5,1)])
        r = solve_open_graph(g, 0, np.array([5]))
        self.assertEqual(r['status'], 'disconnected')
        self.assertEqual(solve_open_graph(g, 1, np.array([5]))['status'], 'ok')

    def test_rejects_non_adjacent_or_backward_edges(self):
        offsets = np.array([0, 1, 2, 3])
        # Source 0 to node 2 skips a layer.
        with self.assertRaisesRegex(ValueError, "adjacent forward layers"):
            LayeredQGraph(offsets, np.array([0., 1., 2.]), np.array([0, 1, 1, 1]),
                          np.array([2]), np.array([1.]))
        # Source 1 to node 0 goes backward.
        with self.assertRaisesRegex(ValueError, "adjacent forward layers"):
            LayeredQGraph(offsets, np.array([0., 1., 2.]), np.array([0, 0, 1, 1]),
                          np.array([0]), np.array([1.]))

    def test_factory_checks_budgets_before_packing(self):
        result = pack_layered_q_graph(np.array([1, 1]), np.array([0., 1.]),
                                      np.array([0]), np.array([1]), np.array([1.]),
                                      max_nodes=1, max_edges=1)
        self.assertEqual(result["status"], "budget_exceeded")
        self.assertNotIsInstance(result, LayeredQGraph)
        packed = pack_layered_q_graph(np.array([1, 1]), np.array([0., 1.]),
                                      np.array([0]), np.array([1]), np.array([1.]),
                                      max_nodes=2, max_edges=1)
        self.assertIsInstance(packed, LayeredQGraph)

    def test_budget_is_status_not_pruning(self):
        g = graph([[0], [1]], [(0,1,1)])
        r = solve_open_graph(g, 0, np.array([1]), max_nodes=1)
        self.assertEqual(r['status'], 'budget_exceeded')
        self.assertEqual(r['node_ids'].size, 0)


if __name__ == '__main__': unittest.main()
