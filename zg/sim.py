"""ZERO-GAP headless simulator (runs ~100-300x faster than real time).

    python3 -m zg.sim --seed 1 --mode zerogap [--faults] [--fleet 26] [--out results/runs/x]
"""
import argparse
import json
import os
import time
from dataclasses import dataclass, field

import numpy as np

from .ap_shield import ap_losses, resilience
from .backbone import Backbone, Formation
from .baton import Kinematics
from .chaos import Chaos
from .comm import GCS, CommModel
from .config import load_config
from .metrics import Metrics, proxy_score
from .planner import AIRBORNE, Planner
from .safety import (SeparationMonitor, clamp_geofence, clamp_speed, geofence_ok,
                     hard_filter, separation_velocity)

PRIO = {"ON_STATION": 3, "HANDOVER_WAIT": 3, "APPROACH": 2}


@dataclass
class UAV:
    uid: int
    pad: np.ndarray
    batt: float
    pos: np.ndarray = None
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    state: str = "GROUND"
    slot: int = None
    alive: bool = True
    t_dead: float = np.inf
    t_state: float = 0.0
    t_ready: float = 0.0
    t_swap_end: float = 0.0
    leg: int = 0
    vacate_xy: np.ndarray = None
    merge_wait: int = None
    t_blocked: float = None          # old occupant that must clear the slot first
    sorties: int = 0
    flight_s: float = 0.0

    def __setattr__(self, k, v):
        if k == "state" and getattr(self, "state", None) != v:
            object.__setattr__(self, "t_state", getattr(self, "_now", 0.0))
            object.__setattr__(self, "t_blocked", None)
        object.__setattr__(self, k, v)


def column_end_time(cfg):
    """Latest time the surveyor column may still be sweeping so that every UAV
    can fly home and land before the 45-min deadline (with margin)."""
    kin = Kinematics(cfg)
    m, s = cfg["mission"], cfg["swarm"]
    sv = s["survey"]
    pads = [np.array([x, y]) for x in s["pads"]["x"] for y in s["pads"]["y"]]
    far_pad = max(pads, key=lambda p: -p[0])        # pad nearest the area is not the worst;
    worst = 0.0
    k, sp = sv["surveyors"], sv["lane_spacing"]
    H = m["area"]["ymax"] - m["area"]["ymin"]
    for b in range(sv["bands"]):
        yc = m["area"]["ymin"] + (b + 0.5) * H / sv["bands"]
        for i in range(k):
            p = np.array([sv["x_min"], yc + (i - (k - 1) / 2) * sp, s["altitudes"]["surveyor"]])
            worst = max(worst, max(kin.time_home(p, pd) for pd in (pads[0], pads[-1], far_pad)))
    # the k surveyors leave through one gate: allow launch-interval spacing + margin
    return m["mission_duration_s"] - m["end_margin_s"] - worst - k * s["launch_interval_s"] - 30.0


class Sim:
    def __init__(self, cfg, seed=0, mode=None, faults=False, fleet=None, log_packets=True):
        self.cfg = cfg
        self.m, self.s = cfg["mission"], cfg["swarm"]
        self.mode = mode or self.s["mode"]
        self.faults = faults
        self.seed = seed
        self.dt = float(self.s["dt"])
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.step_i = 0
        N = int(fleet or self.s["fleet_size"])
        pads = [np.array([x, y], float) for x in self.s["pads"]["x"] for y in self.s["pads"]["y"]]
        # nearest-to-area pads first
        pads.sort(key=lambda p: (-p[0], abs(p[1] - 500.0)))
        assert N <= len(pads), f"fleet {N} > {len(pads)} pads"
        full = self.m["uav"]["max_flight_time_s"]
        self.uavs = [UAV(i, pads[i], full, pos=np.array([pads[i][0], pads[i][1], 0.0])) for i in range(N)]
        self.kin = Kinematics(cfg)
        t_col_end = column_end_time(cfg)
        self.formation = Formation(cfg, t_col_end)
        self.bb = Backbone(cfg, self.formation)
        self.planner = Planner(cfg, self.bb, self.mode)
        self.comm = CommModel(self.m["comm"], np.random.default_rng(seed + 1000))
        self.chaos = Chaos(cfg["chaos"], self.m, np.random.default_rng(seed + 2000), faults)
        self.sep = SeparationMonitor(self.m["safety"]["min_separation"])
        self.metrics = Metrics()
        self.log_packets = log_packets
        self.deadline_s = self.m["comm"]["report_deadline_s"]
        gcs = np.array([*self.m["center"]["xy"], self.m["center"]["antenna_height"]], float)
        self.node_pos = np.zeros((N + 1, 3))
        self.node_pos[0] = gcs
        self.last_landing = float("nan")
        self.last_losses = {}
        self.connected = set()
        self.P = np.zeros((N + 1, N + 1))
        self.pred = None
        self.reports = []
        self.kills = []
        self.block_log = []
        # POIs: random positions AND random spawn times (seeded)
        prng = np.random.default_rng(seed + 3000)
        pc = self.m["poi"]
        a = self.m["area"]
        ts = np.sort(prng.uniform(pc["spawn_t_min"], pc["spawn_t_max"], pc["count"]))
        self.pois = [dict(id=i, x=float(prng.uniform(a["xmin"], a["xmax"])),
                          y=float(prng.uniform(a["ymin"], a["ymax"])), t_spawn=float(ts[i]),
                          t_detect=None, t_report=None, by=None) for i in range(pc["count"])]
        self.poi_xy = np.array([[p["x"], p["y"]] for p in self.pois])
        self.coverage = np.full((50, 50), -1e9)    # last-seen time, 20 m cells
        self._tel_every = max(1, int(round(self.m["comm"]["telemetry_period_s"] / self.dt)))

    # ------------------------------------------------------------------ utils
    @staticmethod
    def node(u):
        return u.uid + 1

    def airborne(self, u):
        return u.alive and u.state in AIRBORNE

    def kill(self, u, t):
        u.alive = False
        u.t_dead = t
        u.state = "DEAD"
        u.vel = np.zeros(3)
        sl = self.planner.slots[u.slot] if u.slot is not None and u.slot < len(self.planner.slots) else None
        self.kills.append(dict(t=t, uid=u.uid, sid=sl.sid if sl is not None and sl.kind != "shadow" else None,
                               kind=sl.kind if sl is not None else None, recovery_s=None))

    # ------------------------------------------------------------- controller
    def _track(self, u, target, vff=None, kp=0.5):
        v = kp * (target - u.pos)
        if vff is not None:
            v = v + vff
        v[2] = np.clip(0.8 * (target[2] - u.pos[2]), -self.m["uav"]["max_vertical_speed"],
                       self.m["uav"]["max_vertical_speed"])
        return v

    def _vertical_clear(self, u, z_to):
        """True if nothing is inside the vertical band this UAV must cross (20 s max wait)."""
        if u.t_blocked is not None and self.t - u.t_blocked > 20.0:
            return True
        if self._critical(u):
            return True
        ok = self._band_free(u, z_to)
        if ok:
            u.t_blocked = None
        elif u.t_blocked is None:
            u.t_blocked = self.t
        return ok

    def _critical(self, u):
        """Battery barely covers the remaining trip: gets right of way."""
        need = self.kin.time_home(u.pos, u.pad) / self.kin.home_factor
        return u.state in ("VACATE", "TRANSIT_HOME", "LANDING") and u.batt < need + 30.0

    def _band_free(self, u, z_to):
        lo, hi = min(u.pos[2], z_to) - 21.0, max(u.pos[2], z_to) + 21.0
        for o in self.uavs:
            if o is u or not self.airborne(o) or o.pos[2] < 1.0:
                continue
            if np.hypot(*(o.pos[:2] - u.pos[:2])) >= 21.0 or not (lo <= o.pos[2] <= hi):
                continue
            if o.uid < u.uid or not self._moving_vertically(o):
                self.block_log.append((self.t, u.uid, u.state, o.uid, o.state))
                return False
        return True

    def _moving_vertically(self, o):
        return o.state in ("TAKEOFF", "LANDING", "VACATE") or (
            o.state == "APPROACH" and abs(o.pos[2] - self.planner.slots[o.slot].alt) > 1.0)

    def _desired(self, u, t):
        A = self.s["altitudes"]
        vmax = self.m["uav"]["max_speed"]
        st = u.state
        pl = self.planner
        if st == "TAKEOFF":
            if u.pos[2] >= A["outbound"] - 0.5:
                u.state = "TRANSIT_OUT"
            if not self._vertical_clear(u, A["outbound"]):
                return np.zeros(3)
            return self._track(u, np.array([u.pad[0], u.pad[1], A["outbound"]]))
        if st == "TRANSIT_OUT":
            rv = pl.rendezvous(u, t)
            # predictive pursuit of the (moving) rendezvous point
            base = pl.bb.pos(pl.slots[u.slot], t, pl.slots)
            tau = 0.0
            for _ in range(3):
                rv_f = pl.bb.pos(pl.slots[u.slot], t + tau, pl.slots) + (rv - base)
                tau = self.kin.path_len(u.pos, rv_f) / vmax
            aim = rv_f
            lane = self.s["transit_lane_y"]
            if pl.slots[u.slot].kind == "relay" and u.pos[0] < rv_f[0] - 10.0 and rv_f[0] > 40.0:
                aim = np.array([rv_f[0], lane, rv_f[2]])      # fly the lane, not the spine
            g = self.kin.gate_for(u.pos, aim)
            if g is not None:
                return self._track(u, np.array([g[0] + 3.0, g[1], A["outbound"]]), kp=1.0)
            if aim is not rv_f:
                return self._track(u, np.array([aim[0], aim[1], A["outbound"]]), kp=1.0)
            if np.linalg.norm(rv[:2] - u.pos[:2]) < 8.0:
                u.state = "APPROACH"
            return self._track(u, np.array([rv_f[0], rv_f[1], A["outbound"]]), kp=1.0)
        if st == "APPROACH":
            rv = pl.rendezvous(u, t)
            far = np.linalg.norm(rv[:2] - u.pos[:2]) > 15.0
            if far and np.linalg.norm(rv[:2] - u.pos[:2]) > 60.0:
                # long re-task leg: use the outbound layer like any departing UAV
                tgt = np.array([rv[0], rv[1], A["outbound"]])
                g = self.kin.gate_for(u.pos, rv)
                if g is not None:
                    tgt[:2] = g
                v = self._track(u, tgt, kp=1.0)
                if abs(u.pos[2] - A["outbound"]) > 1.0 and not self._vertical_clear(u, A["outbound"]):
                    v[:] = 0.0
                elif abs(u.pos[2] - A["outbound"]) > 1.0:
                    v[:2] = 0.0                               # change layer first
                return v
            v = self._track(u, rv, pl.slot_vel(u, t))
            if far or not self._vertical_clear(u, rv[2]):
                v[2] = 0.0                                    # hold altitude while far / blocked
            if np.linalg.norm(rv[:2] - u.pos[:2]) < 4.0 and abs(rv[2] - u.pos[2]) < 1.0:
                pl.arrived(u, t, self)
            return v
        if st == "HANDOVER_WAIT":
            return self._track(u, pl.rendezvous(u, t), pl.slot_vel(u, t))
        if st == "ON_STATION":
            sl = pl.slots[u.slot]
            tgt = pl.bb.pos(sl, t, pl.slots)
            if u.merge_wait is not None:
                o = self.uavs[u.merge_wait]
                if o.alive and o.state == "VACATE" and np.linalg.norm(o.pos - tgt) < 25.0:
                    tgt = tgt + sl.rv_offset                  # hold beside the slot until it is clear
                else:
                    u.merge_wait = None
            return self._track(u, tgt, pl.bb.vel(sl, t, pl.slots))
        if st == "VACATE":
            if u.pos[2] >= A["inbound"] - 0.5:
                u.state = "TRANSIT_HOME"
            v = self._track(u, np.array([u.vacate_xy[0], u.vacate_xy[1], A["inbound"]]))
            if np.linalg.norm(u.vacate_xy - u.pos[:2]) > 5.0 or not self._vertical_clear(u, A["inbound"]):
                v[2] = 0.0                                    # side-step first, then climb when clear
            return v
        if st == "TRANSIT_HOME":
            g = self.kin.gate_for(u.pos, u.pad)
            if g is not None:
                return self._track(u, np.array([g[0] - 3.0, g[1], A["inbound"]]), kp=1.0)
            tgt = np.array([u.pad[0], u.pad[1], A["inbound"]])
            if np.linalg.norm(tgt[:2] - u.pos[:2]) < 1.5:
                u.state = "LANDING"
            return self._track(u, tgt, kp=1.0)
        if st == "LANDING":
            v = self._track(u, np.array([u.pad[0], u.pad[1], 0.0]))
            v[2] = -self.m["uav"]["max_vertical_speed"] if self._vertical_clear(u, 0.0) else 0.0
            return v
        return np.zeros(3)

    # ------------------------------------------------------------------ tick
    def step(self):
        t = self.t
        dt = self.dt
        for u in self.uavs:
            object.__setattr__(u, "_now", t)
        self.chaos.step(t, self)

        # comm graph + routing ------------------------------------------------
        active = np.zeros(len(self.uavs) + 1, bool)
        active[0] = True
        for u in self.uavs:
            self.node_pos[self.node(u)] = u.pos
            active[self.node(u)] = self.airborne(u)
        self.P, _ = self.comm.build(self.node_pos, active)
        dist, self.pred = self.comm.routes_to_gcs(self.P)
        self.connected = {u.uid for u in self.uavs if active[self.node(u)] and np.isfinite(dist[self.node(u)])}

        surv_nodes = [self.node(self.uavs[sl.occupant]) for sl in self.planner.slots
                      if sl.kind == "surveyor" and sl.occupant is not None]
        air_nodes = [self.node(u) for u in self.uavs if active[self.node(u)]]
        self.last_losses, conn_surv, _, _, _ = ap_losses(self.P, air_nodes, surv_nodes)

        # planner ---------------------------------------------------------------
        self.planner.step(t, self, self.P, self.connected)

        # sensing + reporting ------------------------------------------------------
        self._sense_and_report(t, active)

        # motion -------------------------------------------------------------------
        self._move(t)

        # metrics ---------------------------------------------------------------
        self._metrics(t, surv_nodes, conn_surv, air_nodes)
        self.t += dt
        self.step_i += 1

    def _sense_and_report(self, t, active):
        r = self.m["poi"]["sensing_radius"]
        sens = [u for u in self.uavs if self.airborne(u) and u.pos[2] <= 40.0]
        for u in sens:
            gx = np.clip(((u.pos[0] + np.array([-40, 0, 40])) / 20).astype(int), 0, 49)
            gy = np.clip(((u.pos[1] + np.array([-40, 0, 40])) / 20).astype(int), 0, 49)
            self.coverage[gx.min():gx.max() + 1, gy.min():gy.max() + 1] = t
        if sens:
            S = np.array([u.pos[:2] for u in sens])
            for p in self.pois:
                if p["t_detect"] is not None or p["t_spawn"] > t:
                    continue
                d = np.hypot(S[:, 0] - p["x"], S[:, 1] - p["y"])
                j = int(np.argmin(d))
                if d[j] <= r:
                    p["t_detect"], p["by"] = t, sens[j].uid
                    self.reports.append(dict(poi=p["id"], t0=t, holder=self.node(sens[j]),
                                             status="in_flight", hops=0, tx=0, lat=0.0))
        # telemetry: every airborne UAV -> GCS once per period (no store-and-forward)
        for u in self.uavs:
            if not active[self.node(u)] or (self.step_i + u.uid) % self._tel_every:
                continue
            path = self.comm.path(self.pred, self.node(u))
            if path is None:
                self.metrics.packet("telemetry", u.uid, t, "dropped", np.nan, 0, 0)
                continue
            k, lat, tx = self.comm.forward(self.P, path)
            ok = k == len(path) - 1
            self.metrics.packet("telemetry", u.uid, t, "delivered" if ok else "dropped",
                                lat if ok else np.nan, len(path) - 1, tx)
        # POI reports: store-and-forward with per-hop ARQ, retried every tick
        for rp in self.reports:
            if rp["status"] != "in_flight":
                continue
            h = self.uavs[rp["holder"] - 1] if rp["holder"] != GCS else None
            if h is not None and not h.alive:
                rp["status"] = "dropped"
                self.metrics.packet("report", h.uid, rp["t0"], "dropped", np.nan, rp["hops"], rp["tx"])
                continue
            if h is not None and h.state in ("SWAP", "GROUND"):     # carried home physically
                self._deliver(rp, t)
                continue
            if h is not None and not active[rp["holder"]]:
                continue
            path = self.comm.path(self.pred, rp["holder"])
            if path is None:
                continue                                             # hold (store)
            k, lat, tx = self.comm.forward(self.P, path)
            rp["hops"] += k
            rp["tx"] += tx
            rp["lat"] = lat
            if k == len(path) - 1:
                self._deliver(rp, t + lat)
            else:
                rp["holder"] = path[k]

    def _deliver(self, rp, t_arr):
        rp["status"] = "delivered"
        p = self.pois[rp["poi"]]
        p["t_report"] = t_arr
        self.metrics.packet("report", p["by"], rp["t0"], "delivered", t_arr - rp["t0"], rp["hops"], rp["tx"])

    def _move(self, t):
        dt = self.dt
        U = self.uavs
        n = len(U)
        vdes = np.zeros((n, 3))
        prio = np.zeros(n)
        act = np.zeros(n, bool)
        for u in U:
            if self.airborne(u) or u.state == "TAKEOFF":
                vdes[u.uid] = self._desired(u, t)
                act[u.uid] = self.airborne(u) and u.pos[2] > 1.0
                prio[u.uid] = PRIO.get(u.state, 1) + 0.001 * u.uid   # uid breaks ties
                if u.slot is not None and self.planner.slots[u.slot].kind == "shadow":
                    prio[u.uid] = 1.5 + 0.001 * u.uid                # shadows make way
                if self._critical(u):
                    prio[u.uid] = 5.0
            elif u.state == "SWAP" and t >= u.t_swap_end:
                u.batt = self.m["uav"]["max_flight_time_s"]
                u.state = "GROUND"
                u.t_ready = t
        pos = np.array([u.pos for u in U])
        vdes = clamp_speed(vdes, self.m["uav"]["max_speed"], self.m["uav"]["max_vertical_speed"])
        sc = self.s["safety"]
        v = separation_velocity(pos, vdes, prio, act, self.m["safety"]["min_separation"],
                                sc["guard_radius"], sc["lookahead_s"], self.m["uav"]["max_speed"])
        v = clamp_speed(v, self.m["uav"]["max_speed"], self.m["uav"]["max_vertical_speed"])
        amax = self.m["uav"]["max_accel"] * dt
        vnew = np.zeros((n, 3))
        for u in U:
            if not (self.airborne(u)):
                continue
            dv = v[u.uid] - u.vel
            nd = np.linalg.norm(dv)
            if nd > amax:
                dv *= amax / nd
            vnew[u.uid] = u.vel + dv
        vnew, nb = hard_filter(pos, vnew, prio, act, self.m["safety"]["min_separation"] + 1.0, dt,
                               self.m["uav"]["max_speed"])
        self.metrics.emergency_brakes += nb
        for u in U:
            if not (self.airborne(u)):
                continue
            u.vel = clamp_speed(vnew[u.uid], self.m["uav"]["max_speed"], self.m["uav"]["max_vertical_speed"])
            newp = u.pos + u.vel * dt
            if not geofence_ok(newp, self.m):
                self.metrics.geofence_interventions += 1
            if newp[2] > self.m["geofence"]["max_altitude"]:
                self.metrics.geofence_interventions += 1
            newp = clamp_geofence(newp, self.m)
            u.pos = newp
            u.batt -= dt
            u.flight_s += dt
            if not geofence_ok(u.pos, self.m):
                self.metrics.geofence_viol += 1
            if u.pos[2] > self.m["geofence"]["max_altitude"] + 1e-9:
                self.metrics.alt_viol += 1
            if u.state == "LANDING" and u.pos[2] <= 0.05:
                u.pos[2] = 0.0
                u.vel = np.zeros(3)
                u.state = "SWAP"
                u.sorties += 1
                u.t_swap_end = t + self.m["uav"]["swap_time_s"]
                self.last_landing = t
                if u.slot is not None:
                    u.slot = None
            if u.batt <= 0 and u.alive and self.airborne(u):
                self.metrics.battery_depleted += 1
                self.kill(u, t)
        for u in U:
            if self.airborne(u):
                self.metrics.min_batt = min(self.metrics.min_batt, u.batt)

    def _metrics(self, t, surv_nodes, conn_surv, air_nodes):
        M = self.metrics
        pos = np.array([u.pos for u in self.uavs])
        air = np.array([self.airborne(u) and u.pos[2] > 1.0 for u in self.uavs])
        self.sep.update(t, pos, air, [u.uid for u in self.uavs])
        F = self.formation
        active_phase = F.t_hold <= t <= F.t_col_end
        # fault recovery: slot refilled by a connected UAV AND all surveyors connected
        surv_slots = [sl for sl in self.planner.slots if sl.kind == "surveyor"]
        all_ok = all(sl.occupant is not None and sl.occupant in self.connected for sl in surv_slots)
        for k in self.kills:
            if k["recovery_s"] is not None:
                continue
            slot_ok = True
            if k["sid"] is not None:
                sl = self.planner.slots[k["sid"]]
                slot_ok = (not self.bb.needed(sl, t)) or (sl.occupant is not None and sl.occupant in self.connected)
            if slot_ok and (all_ok or not active_phase):
                k["recovery_s"] = t - k["t"]
                M.recoveries.append(k)
        if not active_phase or self.step_i % 2:
            return
        dt_s = 2 * self.dt
        M.samples += 1
        n_ok = sum(1 for sl in surv_slots if sl.occupant is not None and sl.occupant in self.connected
                   and self.uavs[sl.occupant].state == "ON_STATION")
        M.surv_slot_ok += n_ok
        M.surv_slot_total += len(surv_slots)
        M.downtime_s += (len(surv_slots) - n_ok) * dt_s
        M.all_conn_ok += int(n_ok == len(surv_slots))
        sft, ret = resilience(self.last_losses, len(conn_surv), len(air_nodes))
        M.sft_ok += int(sft)
        M.retention_sum += ret
        if self.step_i % 2 == 0:
            M.timeline.append(dict(t=t, airborne=int(air.sum()), surv_connected=n_ok,
                                   all_connected=int(n_ok == len(surv_slots)), sft=int(sft),
                                   retention=ret, aps=len(self.last_losses),
                                   shadows=len(self.planner.slots) - self.bb.n_fixed,
                                   ready=sum(u.state == "GROUND" and u.slot is None for u in self.uavs),
                                   min_batt=min([u.batt for u in self.uavs if self.airborne(u)] or [np.nan]),
                                   column_x=float(F.column(t)[0])))

    # ------------------------------------------------------------------- run
    def run(self, t_end=None):
        t_end = t_end if t_end is not None else self.m["mission_duration_s"]
        while self.t < t_end - 1e-9:
            self.step()
        self.planner.hlog.finalize(self.t)
        for k in self.kills:
            if k["recovery_s"] is None:
                self.metrics.recoveries.append(k)
        return self.metrics.summary(self)

    def save(self, out_dir, summary):
        import pandas as pd
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2, default=float)
        pd.DataFrame(self.metrics.pkt_rows, columns=["kind", "src_uid", "t_created", "status",
                                                     "latency_s", "hops", "tx_attempts"]
                     ).to_csv(os.path.join(out_dir, "packets.csv"), index=False)
        pd.DataFrame(self.planner.hlog.records).to_csv(os.path.join(out_dir, "handovers.csv"), index=False)
        pd.DataFrame(self.pois).to_csv(os.path.join(out_dir, "pois.csv"), index=False)
        pd.DataFrame(self.metrics.timeline).to_csv(os.path.join(out_dir, "timeline.csv"), index=False)
        pd.DataFrame(self.planner.events, columns=["t", "event", "uid", "slot"]
                     ).to_csv(os.path.join(out_dir, "planner_events.csv"), index=False)
        pd.DataFrame(self.planner.shield_log).to_csv(os.path.join(out_dir, "shadows.csv"), index=False)
        pd.DataFrame(self.sep.log, columns=["t", "uid_a", "uid_b", "dist_m"]
                     ).to_csv(os.path.join(out_dir, "separation_violations.csv"), index=False)
        pd.DataFrame(self.chaos.log).to_csv(os.path.join(out_dir, "chaos.csv"), index=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--mode", choices=["zerogap", "baseline"], default=None)
    ap.add_argument("--faults", action="store_true")
    ap.add_argument("--fleet", type=int, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--t-end", type=float, default=None)
    a = ap.parse_args()
    cfg = load_config()
    t0 = time.time()
    sim = Sim(cfg, seed=a.seed, mode=a.mode, faults=a.faults, fleet=a.fleet)
    s = sim.run(a.t_end)
    s["proxy_score"] = proxy_score(s)
    s["wall_time_s"] = time.time() - t0
    for k, v in s.items():
        print(f"{k:34s} {v:.3f}" if isinstance(v, float) else f"{k:34s} {v}")
    if a.out:
        sim.save(a.out, s)
        print("saved ->", a.out)


if __name__ == "__main__":
    main()
