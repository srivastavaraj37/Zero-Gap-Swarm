import numpy as np

from zg.config import load_config
from zg.safety import SeparationMonitor, clamp_geofence, geofence_ok, hard_filter
from zg.sim import Sim


def test_battery_never_negative_and_all_land():
    sim = Sim(load_config(), seed=5)
    s = sim.run()
    assert s["battery_depleted"] == 0
    assert s["min_battery_pct"] >= 0
    assert s["all_landed_by_45min"]
    assert s["geofence_violations"] == 0 and s["altitude_violations"] == 0


def test_hard_filter_stops_head_on_collision():
    pos = np.array([[0.0, 0, 50], [30.0, 0, 50]])
    vel = np.array([[5.0, 0, 0], [-5.0, 0, 0]])
    active = np.array([True, True])
    prio = np.array([1.0, 2.0])
    p = pos.copy()
    v = vel.copy()
    for _ in range(20):
        v, _ = hard_filter(p, vel.copy(), prio, active, 21.0, 0.5, 5.0)
        p = p + v * 0.5
    assert np.linalg.norm(p[0] - p[1]) >= 20.0


def test_separation_monitor_counts_events():
    m = SeparationMonitor(20.0)
    pos = np.array([[0.0, 0, 30], [15.0, 0, 30], [200.0, 0, 30]])
    m.update(0.0, pos, np.array([True] * 3), [0, 1, 2])
    m.update(0.5, pos, np.array([True] * 3), [0, 1, 2])
    assert m.events == 1 and m.pair_ticks == 2 and abs(m.min_dist - 15.0) < 1e-9


def test_geofence_clamp():
    mis = load_config()["mission"]
    q = clamp_geofence(np.array([-100.0, 100.0, 120.0]), mis)
    assert geofence_ok(q, mis) and q[2] <= 100.0
