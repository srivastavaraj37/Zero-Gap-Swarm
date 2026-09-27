"""Re-run a ZERO-GAP mission and dump per-UAV trajectories for the Gazebo replay.

Run with the system numpy (PYTHONNOUSERSITE=1) if the user site has numpy 2.x.
"""
import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from zg.config import load_config  # noqa: E402
from zg.sim import Sim  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--uavs", default="10,9,31,33")
    ap.add_argument("--t0", type=float, default=795.0)
    ap.add_argument("--t1", type=float, default=880.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    uids = [int(u) for u in a.uavs.split(",")]
    sim = Sim(load_config(), seed=a.seed, log_packets=False)
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "uid", "x", "y", "z", "state", "slot_kind", "slot_index"])
        while sim.t < a.t1:
            sim.step()
            if sim.t < a.t0:
                continue
            for u in uids:
                v = sim.uavs[u]
                sl = sim.planner.slots[v.slot] if v.slot is not None else None
                w.writerow([f"{sim.t:.1f}", u, f"{v.pos[0]:.2f}", f"{v.pos[1]:.2f}", f"{v.pos[2]:.2f}", v.state,
                            sl.kind if sl else "", sl.index if sl else ""])
    print("wrote", a.out)


if __name__ == "__main__":
    main()
