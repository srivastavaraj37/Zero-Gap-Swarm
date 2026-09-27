"""Relay Baton: must-return timing, slot intercept and the make-before-break handover check."""
import numpy as np

from .ap_shield import reachable


class Kinematics:
    """Time estimates shared by the Baton protocol and the 45-min recall."""

    def __init__(self, cfg):
        m, s = cfg["mission"], cfg["swarm"]
        self.v = m["uav"]["max_speed"]
        self.vz = m["uav"]["max_vertical_speed"]
        self.alt_out = s["altitudes"]["outbound"]
        self.alt_in = s["altitudes"]["inbound"]
        self.gate_x = float(s["gates"]["x"])
        self.gate_y = tuple(s["gates"]["y_range"])
        self.margin = m["uav"]["return_margin_s"]
        self.home_factor = m["uav"]["home_time_factor"]

    def gate_for(self, a_xy, b_xy):
        """Corridor crossing point for a->b, or None if the straight line is legal."""
        a, b = np.asarray(a_xy, float)[:2], np.asarray(b_xy, float)[:2]
        gx = self.gate_x
        if (a[0] > gx) == (b[0] > gx):
            return None
        yc = a[1] + (b[1] - a[1]) * (gx - a[0]) / (b[0] - a[0])
        if self.gate_y[0] <= yc <= self.gate_y[1]:
            return None
        return np.array([gx, np.clip(yc, *self.gate_y)])

    def path_len(self, a_xy, b_xy):
        a, b = np.asarray(a_xy, float)[:2], np.asarray(b_xy, float)[:2]
        g = self.gate_for(a, b)
        if g is None:
            return np.linalg.norm(a - b)
        return np.linalg.norm(a - g) + np.linalg.norm(g - b)

    def time_home(self, pos, pad):
        climb = max(self.alt_in - pos[2], 0.0) / self.vz
        horiz = self.path_len(pos, pad) / self.v
        return self.home_factor * (climb + horiz + self.alt_in / self.vz)

    def time_out(self, pad, target, alt):
        """Pad -> takeoff -> outbound transit -> descend to `alt` at target."""
        return (self.alt_out / self.vz + self.path_len(pad, target) / self.v
                + abs(self.alt_out - alt) / self.vz)

    def time_to(self, pos, target, alt):
        """Airborne UAV -> target (used when promoting a shadow)."""
        return np.linalg.norm(np.asarray(pos)[:2] - target[:2]) / self.v + abs(pos[2] - alt) / self.vz + 5.0

    def ttmr(self, batt, pos, pad):
        return batt - self.time_home(pos, pad) - self.margin


def intercept(bb, slot, t, t_ready, travel_fn, iters=60):
    """Arrival time at a (possibly moving) slot: fixed point of
    t_arr = t_ready + travel(target(t_arr)), iterated to convergence."""
    t_arr = t_ready
    for _ in range(iters):
        tgt = bb.pos(slot, t_arr) + (slot.rv_offset if slot.occupant is not None else 0)
        nxt = t_ready + travel_fn(tgt)
        if abs(nxt - t_arr) < 0.5:
            return nxt, tgt
        t_arr = nxt
    return t_arr, tgt


def handover_ok(P, active_nodes, role_nodes, old_node, new_node):
    """Make-before-break check on the live graph: with `old_node` removed, the new
    node and every role node that is currently connected must still reach the GCS."""
    from .ap_shield import adjacency
    adj, nodes, loc = adjacency(P, [0] + [n for n in active_nodes if n != 0])
    if new_node not in loc or old_node not in loc:
        return False
    before = reachable(adj, 0)
    if loc[new_node] not in before:
        return False
    after = reachable(adj, 0, removed=loc[old_node])
    for n in role_nodes:
        if n == old_node or n not in loc:
            continue
        if loc[n] in before and loc[n] not in after:
            return False
    return loc[new_node] in after


class HandoverLog:
    """Every role transfer and the slot's service gap (time the slot had no
    GCS-connected occupant while it was needed)."""

    def __init__(self):
        self.records = []
        self.open_gaps = {}          # sid -> record

    def event(self, t, sid, kind, old, new, gap_start=None):
        rec = dict(t=t, slot=sid, kind=kind, old=old, new=new, gap_s=0.0,
                   t_gap_start=gap_start, closed=gap_start is None)
        self.records.append(rec)
        if gap_start is not None:
            self.open_gaps[sid] = rec
        return rec

    def close_gaps(self, t, slots, connected_uids, needed_fn):
        for sid, rec in list(self.open_gaps.items()):
            sl = slots[sid]
            if sl.occupant is not None and sl.occupant in connected_uids:
                rec["gap_s"] = t - rec["t_gap_start"]
                rec["closed"] = True
                rec["new"] = sl.occupant
                del self.open_gaps[sid]
            elif not needed_fn(sl, t):
                rec["gap_s"] = t - rec["t_gap_start"]
                rec["closed"] = True
                rec["note"] = "slot no longer needed"
                del self.open_gaps[sid]

    def finalize(self, t):
        for sid, rec in self.open_gaps.items():
            rec["gap_s"] = t - rec["t_gap_start"]
            rec["note"] = "never refilled"
        self.open_gaps = {}
