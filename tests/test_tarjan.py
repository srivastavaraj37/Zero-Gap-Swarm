import random

import networkx as nx

from zg.ap_shield import articulation_points


def _adj(G, n):
    return [list(G.neighbors(i)) for i in range(n)]


def test_chain_and_cycle():
    chain = nx.path_graph(6)
    assert articulation_points(_adj(chain, 6)) == {1, 2, 3, 4}
    ring = nx.cycle_graph(6)
    assert articulation_points(_adj(ring, 6)) == set()


def test_matches_networkx_on_random_graphs():
    rng = random.Random(0)
    for _ in range(200):
        n = rng.randint(2, 30)
        G = nx.gnp_random_graph(n, rng.uniform(0.05, 0.3), seed=rng.randint(0, 10**6))
        assert articulation_points(_adj(G, n)) == set(nx.articulation_points(G))
