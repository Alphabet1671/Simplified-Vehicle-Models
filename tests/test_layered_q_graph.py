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


class PackedGraphTests(unittest.TestCase):
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
