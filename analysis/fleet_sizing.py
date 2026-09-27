"""Analytical fleet sizing for the ZERO-GAP comb topology.

    python3 analysis/fleet_sizing.py [--write results/fleet_sizing.md]
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from zg.backbone import Backbone, Formation  # noqa: E402
from zg.baton import Kinematics  # noqa: E402
from zg.config import load_config  # noqa: E402
from zg.sim import column_end_time  # noqa: E402


def sizing(cfg):
    m, s = cfg["mission"], cfg["swarm"]
    cx, cy = m["center"]["xy"]
    a = m["area"]
    R = m["comm"]["planning_range"]
    T = m["uav"]["max_flight_time_s"]
    swap = m["uav"]["swap_time_s"]
    kin = Kinematics(cfg)
    out = {}

    # 1. minimum chain to the farthest point of the area
    corners = [(a["xmax"], a["ymin"]), (a["xmax"], a["ymax"]), (a["xmin"], a["ymin"]), (a["xmin"], a["ymax"])]
    d_far = max(math.hypot(x - cx, y - cy) for x, y in corners)
    hops = math.ceil(d_far / R)
    out["d_far_m"] = d_far
    out["chain_hops"] = hops
    out["chain_relays"] = hops - 1
    out["min_sep_ok"] = d_far / hops >= m["safety"]["min_separation"]

    # 2. surveyor column: lanes spaced < 2r so footprints overlap
    sv = s["survey"]
    band_h = (a["ymax"] - a["ymin"]) / sv["bands"]
    out["lane_spacing_m"] = sv["lane_spacing"]
    out["surveyors"] = math.ceil(band_h / (2 * m["poi"]["sensing_radius"]))
    out["surveyors_cfg"] = sv["surveyors"]

    # 3. slot activity from the real schedule
    F = Formation(cfg, column_end_time(cfg))
    bb = Backbone(cfg, F)
    active = (F.t_grid >= F.t_hold) & (F.t_grid <= F.t_col_end)
    rows = []
    for sl in bb.slots:
        frac = bb.need[sl.sid][active].mean()
        # typical on-station point: mean over the active window
        ts = F.t_grid[active][::20]
        P = np.array([bb.pos(sl, t) for t in ts])
        wt = bb.need[sl.sid][active][::20]
        p = P[wt].mean(axis=0) if wt.any() else P.mean(axis=0)
        pads = [np.array([x, y]) for x in s["pads"]["x"] for y in s["pads"]["y"]]
        pad = pads[len(pads) // 2]
        t_out = kin.time_out(pad, p, sl.alt)
        t_home = kin.time_home(p, pad)
        on_station = T - t_out - t_home - kin.margin - s["baton"]["handover_buffer_s"]
        cycle = T - kin.margin + swap + s["launch_interval_s"]
        uavs = frac * cycle / on_station
        rows.append(dict(slot=f"{sl.kind[0].upper()}{sl.index}", active_frac=frac, t_out=t_out,
                         t_home=t_home, on_station=on_station, uavs=uavs))
    out["rows"] = rows
    out["peak_slots"] = int(bb.need[:, active].sum(axis=0).max())
    out["mean_slots"] = float(bb.need[:, active].sum(axis=0).mean())
    out["rotation_uavs"] = sum(r["uavs"] for r in rows)
    out["shadows"] = s["ap_shield"]["max_shadows"]
    out["fault_reserve"] = 2
    out["N_recommended"] = math.ceil(out["rotation_uavs"] + out["shadows"] + out["fault_reserve"])
    return out


def report(o):
    L = []
    L.append("# Fleet sizing (by hand)\n")
    L.append(f"* The farthest points from the center are the far corners, {o['d_far_m']:.1f} m away.")
    L.append(f"* A plain relay chain with hops of at most 90 m needs {o['chain_hops']} hops, so "
             f"{o['chain_relays']} relays (each hop is {o['d_far_m'] / o['chain_hops']:.1f} m, "
             f"more than the 20 m separation: {o['min_sep_ok']}).")
    L.append(f"* Surveyor lanes are {o['lane_spacing_m']:.1f} m apart (less than 2 x 40 m footprint), "
             f"so {o['surveyors']} lanes cover one band: {o['surveyors_cfg']} surveyors.")
    L.append(f"* The spine has fixed relay stations and only the ones behind the column are used. "
             f"At most {o['peak_slots']} slots are needed at once, {o['mean_slots']:.1f} on average.")
    L.append("* Each slot needs active_frac x cycle / on_station UAVs, where "
             "on_station = 1200 - t_out - t_home - 60 margin - 60 handover overlap and cycle = battery used + swap.\n")
    L.append("| slot | active frac | t_out s | t_home s | on-station s/sortie | UAVs |")
    L.append("|---|---|---|---|---|---|")
    for r in o["rows"]:
        L.append(f"| {r['slot']} | {r['active_frac']:.2f} | {r['t_out']:.0f} | {r['t_home']:.0f} | "
                 f"{r['on_station']:.0f} | {r['uavs']:.2f} |")
    L.append(f"\n* UAVs needed for rotation (sum): {o['rotation_uavs']:.1f}")
    L.append(f"* Plus AP-Shield shadows: {o['shadows']}, plus spares for faults: {o['fault_reserve']}")
    L.append(f"* Recommended N = {o['N_recommended']}")
    L.append("\nThis estimate assumes a steady state. In the real 45-min mission all UAVs start charged "
             "and the last flights are short, so the simulated sweep over N in results/ decides the final N.")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", default=None)
    args = ap.parse_args()
    o = sizing(load_config())
    txt = report(o)
    print(txt)
    if args.write:
        os.makedirs(os.path.dirname(args.write), exist_ok=True)
        with open(args.write, "w") as f:
            f.write(txt + "\n")


if __name__ == "__main__":
    main()
