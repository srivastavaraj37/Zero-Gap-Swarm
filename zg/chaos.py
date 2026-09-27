"""Seeded fault injector: UAV kills, regional link jamming, extra packet loss."""
import numpy as np


class Chaos:
    def __init__(self, chaos_cfg, mission_cfg, rng, enabled):
        self.enabled = enabled
        self.events = []            # (t, kind, params) sorted by t
        self.log = []
        if not enabled:
            return
        c = chaos_cfg
        a = mission_cfg["area"]
        for _ in range(int(c["kills"]["count"])):
            self.events.append((rng.uniform(c["kills"]["t_min"], c["kills"]["t_max"]), "kill",
                                {"pick": rng.random()}))
        j = c["jamming"]
        for _ in range(int(j["count"])):
            t0 = rng.uniform(j["t_min"], j["t_max"])
            ctr = (rng.uniform(a["xmin"], a["xmax"]), rng.uniform(a["ymin"], a["ymax"]), j["radius"])
            self.events.append((t0, "jam_on", {"region": ctr}))
            self.events.append((t0 + j["duration"], "jam_off", {"region": ctr}))
        e = c["extra_loss"]
        for _ in range(int(e["count"])):
            t0 = rng.uniform(e["t_min"], e["t_max"])
            self.events.append((t0, "loss_on", {"factor": e["factor"]}))
            self.events.append((t0 + e["duration"], "loss_off", {}))
        self.events.sort(key=lambda ev: ev[0])

    def step(self, t, sim):
        while self.events and self.events[0][0] <= t:
            t_ev, kind, prm = self.events.pop(0)
            if kind == "kill":
                # kill an on-station UAV (the kind of failure that hurts)
                cands = [u for u in sim.uavs if u.alive and u.state == "ON_STATION"]
                if not cands:
                    continue
                u = cands[int(prm["pick"] * len(cands)) % len(cands)]
                sim.kill(u, t)
                self.log.append(dict(t=t, kind="kill", uid=u.uid, slot=u.slot))
            elif kind == "jam_on":
                sim.comm.jam_regions.append(prm["region"])
                self.log.append(dict(t=t, kind="jam_on", region=prm["region"]))
            elif kind == "jam_off":
                if prm["region"] in sim.comm.jam_regions:
                    sim.comm.jam_regions.remove(prm["region"])
                self.log.append(dict(t=t, kind="jam_off"))
            elif kind == "loss_on":
                sim.comm.loss_factor = prm["factor"]
                self.log.append(dict(t=t, kind="loss_on", factor=prm["factor"]))
            elif kind == "loss_off":
                sim.comm.loss_factor = 1.0
                self.log.append(dict(t=t, kind="loss_off"))
