import numpy as np

from zg.baton import Kinematics, handover_ok, intercept
from zg.config import load_config
from zg.sim import Sim


def test_time_home_and_ttmr():
    k = Kinematics(load_config())
    pad = np.array([-73.0, 500.0])
    near = k.time_home(np.array([0.0, 500.0, 51.0]), pad)
    far = k.time_home(np.array([900.0, 500.0, 51.0]), pad)
    assert far > near > 0
    # ttmr = battery - home - margin
    assert abs(k.ttmr(1000.0, np.array([0.0, 500.0, 51.0]), pad) - (1000 - near - 60)) < 1e-9


def test_intercept_converges_on_moving_slot():
    sim = Sim(load_config(), seed=0)
    sl = sim.bb.slots[sim.bb.n_fixed - 1]          # a surveyor slot (moving)
    t = 400.0
    pad = sim.uavs[0].pad
    t_arr, tgt = intercept(sim.bb, sl, t, t, lambda g: sim.kin.time_out(pad, g, sl.alt))
    # self-consistent: arrival time matches travel time to where the slot is then
    assert abs(t_arr - (t + sim.kin.time_out(pad, sim.bb.pos(sl, t_arr), sl.alt))) < 1.0


def test_make_before_break_check():
    # GCS(0) - old(1) - surveyor(2); new(3) also links GCS and surveyor
    P = np.zeros((4, 4))
    for a, b in ((0, 1), (1, 2), (0, 3), (3, 2)):
        P[a, b] = P[b, a] = 1.0
    assert handover_ok(P, [0, 1, 2, 3], [1, 2], old_node=1, new_node=3)
    P[3, 2] = P[2, 3] = 0.0                        # new cannot reach the surveyor yet
    assert not handover_ok(P, [0, 1, 2, 3], [1, 2], old_node=1, new_node=3)


def test_baton_relieves_before_battery_runs_out():
    sim = Sim(load_config(), seed=3)
    sim.run(1500.0)
    recs = sim.planner.hlog.records
    assert any(r["kind"] == "baton_mbb" and r["gap_s"] == 0.0 for r in recs)
