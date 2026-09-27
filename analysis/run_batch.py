"""Batch runner: seeds x {baseline, zerogap} x {no faults, faults}, plus a fleet-size sweep.

    python3 analysis/run_batch.py --seeds 20 --workers 10
    python3 analysis/run_batch.py --sweep 24,28,32,36,38,40 --sweep-seeds 3
"""
import argparse
import os
import sys
import time
from multiprocessing import Pool

import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from zg.config import load_config  # noqa: E402
from zg.metrics import proxy_score  # noqa: E402
from zg.sim import Sim  # noqa: E402


def one(job):
    mode, faults, seed, fleet = job
    t0 = time.time()
    sim = Sim(load_config(), seed=seed, mode=mode, faults=faults, fleet=fleet, log_packets=False)
    s = sim.run()
    s["proxy_score"] = proxy_score(s)
    s["wall_time_s"] = time.time() - t0
    return s


def run(jobs, workers):
    out = []
    with Pool(workers) as p:
        for i, s in enumerate(p.imap_unordered(one, jobs)):
            out.append(s)
            print(f"[{i + 1}/{len(jobs)}] {s['mode']:8s} faults={s['faults']!s:5s} seed={s['seed']:3d} "
                  f"N={s['fleet_size']} on_time={s['pois_on_time']} conn={s['connectivity_availability_pct']:.1f}% "
                  f"score={s['proxy_score']:.1f} ({s['wall_time_s']:.0f}s)", flush=True)
    return pd.DataFrame(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--workers", type=int, default=max(1, os.cpu_count() - 2))
    ap.add_argument("--fleet", type=int, default=None)
    ap.add_argument("--sweep", default=None, help="comma separated fleet sizes")
    ap.add_argument("--sweep-seeds", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(ROOT, "results"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    if a.sweep:
        Ns = [int(n) for n in a.sweep.split(",")]
        jobs = [(m, False, s, n) for n in Ns for m in ("zerogap", "baseline") for s in range(1, a.sweep_seeds + 1)]
        df = run(jobs, a.workers)
        df.sort_values(["mode", "fleet_size", "seed"]).to_csv(os.path.join(a.out, "fleet_sweep.csv"), index=False)
        print("wrote", os.path.join(a.out, "fleet_sweep.csv"))
        return
    jobs = [(m, f, s, a.fleet) for m in ("baseline", "zerogap") for f in (False, True)
            for s in range(1, a.seeds + 1)]
    df = run(jobs, a.workers)
    df.sort_values(["mode", "faults", "seed"]).to_csv(os.path.join(a.out, "batch_runs.csv"), index=False)
    print("wrote", os.path.join(a.out, "batch_runs.csv"))


if __name__ == "__main__":
    main()
