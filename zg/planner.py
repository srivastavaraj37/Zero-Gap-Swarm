"""Center-side mission planner: slot filling, Relay Baton, recall, AP Shield.

Runs at the GCS each tick with the latest telemetry (Stage-1 assumption: the
planner sees every UAV state; UAVs that lose the link keep flying the shared
deterministic schedule autonomously, so a disconnection never freezes the plan).
"""
import numpy as np

from .ap_shield import place_shadow
from .backbone import Slot
from .baton import HandoverLog, Kinematics, handover_ok, intercept
from .safety import geofence_ok

AIRBORNE = {"TAKEOFF", "TRANSIT_OUT", "APPROACH", "HANDOVER_WAIT", "ON_STATION",
            "VACATE", "TRANSIT_HOME", "LANDING"}


class Planner:
    def __init__(self, cfg, backbone, mode):
        self.cfg = cfg
        self.m, self.s = cfg["mission"], cfg["swarm"]
        self.bb = backbone
        self.kin = Kinematics(cfg)
        self.zerogap = mode == "zerogap"
        bt = self.s["baton"]
        self.baton = self.zerogap and bt["enabled"]
        self.shield = self.zerogap and self.s["ap_shield"]["enabled"]
        self.promote = self.zerogap and bt["promote_shadows"]
        self.slots = backbone.slots
        self.hlog = HandoverLog()
        self.launch_queue = []
        self.last_launch = -1e9
        self.duration = self.m["mission_duration_s"]
        self.deadline = self.duration - self.m["end_margin_s"]
        self.t_col_end = backbone.F.t_col_end
        self.next_shield = 0.0
        self.min_service = bt["min_service_s"]
        self.shield_log = []
        self.events = []

    # ----------------------------------------------------------- helpers
    def rendezvous(self, u, t):
        sl = self.slots[u.slot]
        p = self.bb.pos(sl, t, self.slots)
        if sl.occupant is not None and sl.occupant != u.uid:
            p = p + sl.rv_offset
        return p

    def slot_vel(self, u, t):
        return self.bb.vel(self.slots[u.slot], t, self.slots)

    def ready_ground(self, sim, t):
        return [u for u in sim.uavs if u.alive and u.state == "GROUND" and u.slot is None]

    def _eta_ground(self, u, sl, t):
        wait = max(0.0, self.last_launch + self.s["launch_interval_s"] * (1 + len(self.launch_queue)) - t)
        t_arr, tgt = intercept(self.bb, sl, t, t + wait,
                               lambda g: self.kin.time_out(u.pad, g, sl.alt))
        return t_arr

    def _eta_air(self, u, sl, t):
        t_arr, _ = intercept(self.bb, sl, t, t, lambda g: self.kin.time_to(u.pos, g, sl.alt))
        return t_arr

    def _worth(self, sl, t_arr, pad):
        """45-min rule + last-launch cutoff: can the UAV serve >= 60 s and still be
        landed before the deadline?"""
        home = self.kin.time_home(self.bb.pos(sl, t_arr, self.slots), pad)
        if t_arr + 60.0 + home > self.deadline:
            return False
        if sl.kind != "shadow":
            nn = self.bb.next_needed_t(sl, t_arr)
            if nn == np.inf or self.bb.needed_until(sl, max(t_arr, nn)) - t_arr < 30.0:
                return False
        return True

    def pick(self, sim, sl, t, allow_air):
        """Choose the UAV that can be on station earliest.

        Candidates: ready UAVs on the ground, plus (ZERO-GAP) airborne shadows and
        UAVs flying home that still have enough battery for a useful stint."""
        best, best_t = None, np.inf
        if allow_air:
            for u in sim.uavs:
                if not u.alive:
                    continue
                shadow = (u.state == "ON_STATION" and u.slot is not None
                          and self.slots[u.slot].kind == "shadow")
                homing = u.state in ("VACATE", "TRANSIT_HOME") and u.slot is None
                if not (shadow or homing):
                    continue
                t_arr = self._eta_air(u, sl, t)
                home_from_slot = self.kin.time_home(self.bb.pos(sl, t_arr, self.slots), u.pad)
                service = u.batt - (t_arr - t) - home_from_slot - self.kin.margin
                if service < self.min_service:
                    continue
                if t_arr < best_t and self._worth(sl, t_arr, u.pad):
                    best, best_t = u, t_arr
        ground = self.ready_ground(sim, t)
        if ground:
            u = min(ground, key=lambda g: (g.t_ready, g.uid))
            t_arr = self._eta_ground(u, sl, t)
            if t_arr < best_t and self._worth(sl, t_arr, u.pad):
                best, best_t = u, t_arr
        return best, best_t

    def assign(self, sim, u, sl, t, why):
        if u.slot is not None and u.slot != sl.sid:          # promoted shadow
            old = self.slots[u.slot]
            if old.occupant == u.uid:
                old.occupant = None
            self.events.append((t, "promote", u.uid, sl.sid))
        elif u.state in ("VACATE", "TRANSIT_HOME"):
            self.events.append((t, "retask", u.uid, sl.sid))
        sl.incoming = u.uid
        u.slot = sl.sid
        if u.state == "GROUND":
            u.state = "QUEUED"
            self.launch_queue.append(u.uid)
        else:
            u.state = "APPROACH"
        self.events.append((t, why, u.uid, sl.sid))

    def release(self, sim, u, t):
        sl = self.slots[u.slot] if u.slot is not None else None
        if sl is not None:
            if sl.occupant == u.uid:
                sl.occupant = None
            if sl.incoming == u.uid:
                sl.incoming = None
        u.slot = None
        was = u.state
        if was == "TAKEOFF" and np.linalg.norm(u.pos[:2] - u.pad) < 5.0:
            u.state = "LANDING"                           # never left the pad area
            return
        u.state = "VACATE"
        u.vacate_xy = u.pos[:2].copy()
        if sl is not None and was in ("ON_STATION", "HANDOVER_WAIT") and u.pos[0] >= 0:
            off = sl.vacate_offset
            if sl.kind == "surveyor":                     # step behind the moving column
                vx = self.bb.vel(sl, t)[0]
                off = np.array([-30.0 if vx >= 0 else 30.0, 0.0])
            u.vacate_xy = np.clip(u.pos[:2] + off, [0.0, 0.0], [1000.0, 1000.0])

    def arrived(self, u, t, sim):
        """UAV finished its approach at the rendezvous point."""
        sl = self.slots[u.slot]
        if sl.occupant is None:
            sl.occupant = u.uid
            if sl.incoming == u.uid:
                sl.incoming = None
            u.state = "ON_STATION"
        else:
            u.state = "HANDOVER_WAIT"

    # ------------------------------------------------------------- main
    def step(self, t, sim, P, connected):
        bb, kin = self.bb, self.kin
        detect = self.s["baton"]["failure_detect_s"]
        node = sim.node

        # 1. failure detection (heartbeat timeout) --------------------------
        for sl in self.slots:
            for attr in ("occupant", "incoming"):
                uid = getattr(sl, attr)
                if uid is None:
                    continue
                u = sim.uavs[uid]
                if not u.alive and t - u.t_dead >= detect:
                    setattr(sl, attr, None)
                    if attr == "occupant" and sl.kind != "shadow" and bb.needed(sl, t) \
                            and sl.sid not in self.hlog.open_gaps:
                        self.hlog.event(t, sl.sid, "failure", uid, None, gap_start=u.t_dead)
        # drop empty shadow slots
        keep = self.slots[:bb.n_fixed]
        for sl in self.slots[bb.n_fixed:]:
            if sl.occupant is not None or sl.incoming is not None:
                keep.append(sl)
        self._reindex(sim, keep)

        # 2. make-before-break handovers ------------------------------------
        active_nodes = [node(u) for u in sim.uavs if u.alive and u.state in AIRBORNE]
        role_nodes = [node(sim.uavs[sl.occupant]) for sl in self.slots
                      if sl.occupant is not None and sl.kind != "shadow"]
        for sl in self.slots:
            if sl.incoming is None:
                continue
            new = sim.uavs[sl.incoming]
            if sl.occupant is None:
                if new.state == "HANDOVER_WAIT":          # old one already gone
                    sl.occupant, sl.incoming = new.uid, None
                    new.state = "ON_STATION"
                continue
            old = sim.uavs[sl.occupant]
            if new.state != "HANDOVER_WAIT":
                continue
            if handover_ok(P, active_nodes, role_nodes, node(old), node(new)):
                self.release(sim, old, t)
                sl.occupant, sl.incoming = new.uid, None
                new.state = "ON_STATION"
                new.merge_wait = old.uid
                self.hlog.event(t, sl.sid, "baton_mbb", old.uid, new.uid)

        # 3a. UAVs still on their way: battery / 45-min checks ---------------
        for u in sim.uavs:
            if u.alive and u.state in ("TAKEOFF", "TRANSIT_OUT", "APPROACH", "HANDOVER_WAIT"):
                th = kin.time_home(u.pos, u.pad)
                if u.batt - th - kin.margin <= 0 or t + th >= self.deadline:
                    self.release(sim, u, t)
            elif u.alive and u.state == "QUEUED" and t + 120.0 >= self.deadline:
                sl = self.slots[u.slot]
                if sl.incoming == u.uid:
                    sl.incoming = None
                self._unqueue(u)

        # 3. occupants: must-return, 45-min recall, relief, idle release ----
        early = []
        for sl in list(self.slots):
            if sl.occupant is None:
                continue
            o = sim.uavs[sl.occupant]
            if not o.alive or o.state != "ON_STATION":
                continue
            th = kin.time_home(o.pos, o.pad)
            tt = o.batt - th - kin.margin
            recall = t + th >= self.deadline
            if tt <= 0 or recall:
                needed = sl.kind != "shadow" and bb.needed(sl, t)
                self.release(sim, o, t)
                if needed and sl.sid not in self.hlog.open_gaps:
                    kind = "recall" if recall else ("forced" if self.baton else "return_low")
                    self.hlog.event(t, sl.sid, kind, o.uid, None, gap_start=t)
                continue
            if sl.kind == "shadow":
                continue
            # idle release: not needed now and cannot be useful before it must leave anyway
            nn = bb.next_needed_t(sl, t)
            if not bb.needed(sl, t) and (nn > t + tt or nn > t + 2 * th + self.m["uav"]["swap_time_s"] + 60):
                if sl.incoming is not None:
                    inc = sim.uavs[sl.incoming]
                    sl.incoming = None
                    self.release(sim, inc, t) if inc.state not in ("QUEUED",) else self._unqueue(inc)
                self.release(sim, o, t)
                continue
            if self.baton and sl.incoming is None:
                t_leave = t + tt
                if bb.next_needed_t(sl, t_leave) <= min(t_leave + 60.0, self.t_col_end):
                    cand, t_arr = self.pick(sim, sl, t, allow_air=self.promote)
                    if cand is not None and t_arr + self.s["baton"]["handover_buffer_s"] >= t_leave:
                        self.assign(sim, cand, sl, t, "relief")
                    elif cand is not None:
                        early.append((tt, sl))
        # opportunistic early relief: when idle UAVs exceed the reserve, relieve
        # the lowest-battery occupants now so returns are staggered (swap queue)
        if self.baton:
            n_ready = len(self.ready_ground(sim, t))
            spare = n_ready - self._pending_needs(sim, t) - self.s["ap_shield"]["ground_reserve"]
            for tt, sl in sorted(early, key=lambda e: e[0]):
                if spare <= 0 or tt > 420.0 or sl.incoming is not None:
                    break
                cand, _ = self.pick(sim, sl, t, allow_air=False)
                if cand is None:
                    break
                self.assign(sim, cand, sl, t, "relief_early")
                spare -= 1

        # 4. vacant / soon-needed slots (priority: spine near GCS first) ------
        order = sorted(self.slots[:bb.n_fixed], key=lambda s: self._prio(s, t))
        for sl in order:
            if sl.occupant is not None or sl.incoming is not None:
                continue
            nn = bb.next_needed_t(sl, t)
            if nn == np.inf or nn > self.t_col_end:
                continue
            if self.zerogap:
                cand, t_arr = self.pick(sim, sl, t, allow_air=self.promote)
                if cand is None:
                    continue
                if nn <= t or t_arr + self.s["baton"]["lookahead_s"] >= nn:
                    self.assign(sim, cand, sl, t, "fill" if nn > t else "fill_now")
            elif nn <= t:                                   # baseline: purely reactive
                cand, _ = self.pick(sim, sl, t, allow_air=False)
                if cand is not None:
                    self.assign(sim, cand, sl, t, "fill_now")

        # 5. AP Shield --------------------------------------------------------
        if self.shield and t >= self.next_shield:
            self.next_shield = t + self.s["ap_shield"]["period_s"]
            self._shield(t, sim, P)

        # 6. center-side launch queue (one launch per interval) --------------
        if self.launch_queue and t - self.last_launch >= self.s["launch_interval_s"]:
            u = sim.uavs[self.launch_queue.pop(0)]
            if u.alive and u.state == "QUEUED":
                u.state = "TAKEOFF"
                u.leg = 0
                self.last_launch = t

        # 7. service-gap bookkeeping -----------------------------------------
        self.hlog.close_gaps(t, self.slots, connected, bb.needed)

    # ------------------------------------------------------------ internals
    def _unqueue(self, u):
        if u.uid in self.launch_queue:
            self.launch_queue.remove(u.uid)
        u.state = "GROUND"
        u.slot = None

    def _pending_needs(self, sim, t):
        """Ground UAVs that vacant slots will need within the next few minutes."""
        n = 0
        for sl in self.slots[:self.bb.n_fixed]:
            if sl.occupant is None and sl.incoming is None and self.bb.next_needed_t(sl, t) <= t + 300:
                n += 1
        return n

    def _prio(self, sl, t):
        if sl.kind == "relay":
            return (0, sl.index)
        p = self.bb.pos(sl, t)
        return (1, abs(p[1] - self.bb.spine_y))

    def _reindex(self, sim, keep):
        if len(keep) == len(self.slots):
            return
        remap = {}
        for new_id, sl in enumerate(keep):
            remap[sl.sid] = new_id
        for sl in keep:
            sl.sid = remap[sl.sid]
            if sl.anchor is not None:
                sl.anchor = remap.get(sl.anchor)
        for u in sim.uavs:
            if u.slot is not None:
                u.slot = remap.get(u.slot)
        # shadows whose anchor vanished are released
        self.slots[:] = keep
        for sl in list(self.slots[self.bb.n_fixed:]):
            if sl.anchor is None:
                for uid in (sl.occupant, sl.incoming):
                    if uid is not None and sim.uavs[uid].alive:
                        u = sim.uavs[uid]
                        if u.state == "QUEUED":
                            self._unqueue(u)
                        else:
                            self.release(sim, u, t=0.0)
        self.hlog.open_gaps = {remap[k]: v for k, v in self.hlog.open_gaps.items() if k in remap}

    def _shield(self, t, sim, P):
        bb = self.bb
        losses = sim.last_losses
        node_to_uid = {sim.node(u): u.uid for u in sim.uavs}
        # anchors that are currently shielded
        shields = self.slots[bb.n_fixed:]
        for sh in list(shields):
            anc = self.slots[sh.anchor]
            if not bb.needed(anc, t) or anc.occupant is None:
                for uid in (sh.occupant, sh.incoming):
                    if uid is not None:
                        u = sim.uavs[uid]
                        self._unqueue(u) if u.state == "QUEUED" else self.release(sim, u, t)
                sh.occupant = sh.incoming = None
        # rank unshielded critical APs
        shielded = {self.slots[sh.anchor].occupant for sh in self.slots[bb.n_fixed:]
                    if sh.anchor is not None and (sh.occupant is not None or sh.incoming is not None)}
        # re-place shadows whose AP is still critical after they are on station
        for sh in self.slots[bb.n_fixed:]:
            if sh.occupant is None or sh.anchor is None:
                continue
            anc = self.slots[sh.anchor]
            if anc.occupant is None:
                continue
            n_anc = sim.node(sim.uavs[anc.occupant])
            if n_anc in losses and sim.uavs[sh.occupant].state == "ON_STATION" \
                    and t - sim.uavs[sh.occupant].t_state > 10.0:
                q = self._placement(sim, t, n_anc, anc, P, exclude_uid=sh.occupant)
                if q is not None:
                    sh.offset = q - bb.pos(anc, t, self.slots)
        n_sh = len([s for s in self.slots[bb.n_fixed:] if s.occupant is not None or s.incoming is not None])
        ranked = sorted(losses.items(), key=lambda kv: -kv[1])
        for n_ap, lost in ranked:
            if n_sh >= self.s["ap_shield"]["max_shadows"]:
                break
            uid = node_to_uid.get(n_ap)
            if uid is None or uid in shielded:
                continue
            u_ap = sim.uavs[uid]
            if u_ap.slot is None or u_ap.state != "ON_STATION":
                continue
            anc = self.slots[u_ap.slot]
            if anc.kind == "shadow" or anc.occupant != uid:
                continue
            ready = self.ready_ground(sim, t)
            if len(ready) - self._pending_needs(sim, t) <= self.s["ap_shield"]["ground_reserve"]:
                break
            q = self._placement(sim, t, n_ap, anc, P)
            if q is None:
                continue
            sh = Slot(len(self.slots), "shadow", len(self.slots) - bb.n_fixed,
                      q[2], (0, 0, 0))
            sh.anchor = anc.sid
            sh.offset = q - bb.pos(anc, t, self.slots)
            self.slots.append(sh)
            cand = min(ready, key=lambda g: (g.t_ready, g.uid))
            t_arr = self._eta_ground(cand, sh, t)
            if not self._worth(sh, t_arr, cand.pad):
                self.slots.pop()
                continue
            self.assign(sim, cand, sh, t, "shadow")
            self.shield_log.append(dict(t=t, ap_uid=uid, lost=lost, shadow_uid=cand.uid,
                                        pos=q.round(1).tolist()))
            shielded.add(uid)
            n_sh += 1

    def _placement(self, sim, t, n_ap, anc, P, exclude_uid=None):
        bb = self.bb
        adj_nodes = [sim.node(u) for u in sim.uavs if u.alive and u.state in AIRBORNE
                     and u.uid != exclude_uid]
        from .ap_shield import adjacency
        adj, nodes, loc = adjacency(P, [0] + adj_nodes)
        if n_ap not in loc:
            return None
        pos = sim.node_pos
        avoid = [pos[n] for n in adj_nodes if n != n_ap]
        for sl in self.slots[:bb.n_fixed]:
            p = bb.pos(sl, t)
            avoid.append(p)
            avoid.append(p + sl.rv_offset)
        for sh in self.slots[bb.n_fixed:]:
            if sh.anchor is not None and (sh.occupant is None or sh.occupant != exclude_uid):
                avoid.append(bb.pos(sh, t, self.slots))
        moving = anc.kind == "surveyor"
        alts = [self.s["altitudes"]["relay"]]
        q, _ = place_shadow(n_ap, P, pos, adj, nodes, loc, alts,
                            self.m["comm"]["planning_range"], avoid,
                            self.s["ap_shield"]["clearance_m"],
                            lambda p: geofence_ok(p, self.m), bb.spine_y, moving)
        return q
