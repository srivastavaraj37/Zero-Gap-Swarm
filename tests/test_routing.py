import numpy as np

from zg.comm import CommModel, link_prob

C = dict(max_range=100.0, p_full_range=70.0, p_at_max_range=0.7, hop_latency_s=0.02,
         hop_jitter_s=0.01, max_retries=3)


def test_link_prob_curve():
    assert link_prob(50) == 1.0
    assert abs(link_prob(85) - 0.85) < 1e-9
    assert abs(link_prob(100) - 0.7) < 1e-9
    assert link_prob(100.1) == 0.0


def test_two_uav_range():
    """Link exists at 99 m, drops beyond 100 m."""
    cm = CommModel(C, np.random.default_rng(0))
    for d, linked in ((60, True), (99, True), (101, False)):
        pos = np.array([[0, 0, 0], [d, 0, 0]], float)
        P, _ = cm.build(pos, np.array([True, True]))
        assert (P[0, 1] > 0) == linked


def test_prefers_reliable_two_hop_over_marginal_link():
    cm = CommModel(C, np.random.default_rng(0))
    # 0 -- 2 direct is 98 m (p=0.72); via node 1 two 49 m hops (p=1)
    pos = np.array([[0, 0, 0], [49, 0, 0], [98, 0, 0]], float)
    P, _ = cm.build(pos, np.ones(3, bool))
    _, pred = cm.routes_to_gcs(P)
    assert cm.path(pred, 2) == [2, 1, 0]


def test_disconnected_and_delivery():
    cm = CommModel(C, np.random.default_rng(1))
    pos = np.array([[0, 0, 0], [60, 0, 0], [300, 0, 0]], float)
    P, _ = cm.build(pos, np.ones(3, bool))
    _, pred = cm.routes_to_gcs(P)
    assert cm.path(pred, 2) is None
    k, lat, tx = cm.forward(P, cm.path(pred, 1))
    assert k == 1 and 0.02 <= lat < 0.05 and tx == 1


def test_jamming_cuts_links():
    cm = CommModel(C, np.random.default_rng(0))
    cm.jam_regions.append((60, 0, 10))
    P, _ = cm.build(np.array([[0, 0, 0], [60, 0, 0]], float), np.ones(2, bool))
    assert P[0, 1] == 0
