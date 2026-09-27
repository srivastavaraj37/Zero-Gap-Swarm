"""Relay backbone + surveyor formation (slots) and the deterministic sweep schedule.

Topology (the "comb"):
  GCS -- R0 -- R1 -- ... -- Rj            fixed relay stations on the spine y = 500 (alt relay)
                             \\
                              S0 - S1 - ... - S(k-1)    surveyor column (alt surveyor), lanes
                                                      spaced < 2 x sensing radius, relays for itself

The column sweeps the area band by band (lawnmower with lane spacing < 2r).
Because the schedule is a deterministic function of time, every UAV - and the
Baton protocol - can predict where any slot will be at any future time, which
lets relief UAVs intercept moving slots and lets disconnected UAVs keep
executing the plan autonomously.
"""
import numpy as np


class Formation:
    def __init__(self, cfg, t_col_end):
        m, s = cfg["mission"], cfg["swarm"]
        sv = s["survey"]
        self.k = int(sv["surveyors"])
        self.nb = int(sv["bands"])
        self.spacing = float(sv["lane_spacing"])
        self.v = float(sv["sweep_speed"])
        self.x_min, self.x_max = float(sv["x_min"]), float(sv["x_max"])
        self.alt = float(s["altitudes"]["surveyor"])
        a = m["area"]
        H = a["ymax"] - a["ymin"]
        self.band_c = [a["ymin"] + (b + 0.5) * H / self.nb for b in range(self.nb)]
        self.t_hold = float(sv["start_hold_s"])
        self.t_col_end = float(t_col_end)
        self.dt = float(s["dt"])
        self.duration = float(m["mission_duration_s"])
        self._build()

    # ------------------------------------------------------------ schedule
    def _build(self):
        """Waypoints (t, X, Yc). At each end of the area the column shifts to the
        band with the oldest coverage (coverage-age rule) and sweeps back."""
        spine_y = 500.0
        b = int(np.argmin([abs(c - spine_y) - (0.1 if c > spine_y else 0) for c in self.band_c]))
        last = {i: -1e9 for i in range(self.nb)}
        last[b] = self.t_hold
        wps = [(0.0, self.x_min, self.band_c[b]), (self.t_hold, self.x_min, self.band_c[b])]
        t, X, Yc = self.t_hold, self.x_min, self.band_c[b]
        L = self.x_max - self.x_min
        while True:
            R = self.t_col_end - t
            other = [i for i in range(self.nb) if i != b] or [b]
            nb_ = min(other, key=lambda i: (last[i], abs(self.band_c[i] - Yc)))
            shift = abs(self.band_c[nb_] - Yc) / self.v
            at_min = X <= self.x_min + 1e-6
            if at_min:
                if R >= shift + 2 * L / self.v:        # full leg out
                    if shift > 0:
                        t += shift; Yc = self.band_c[nb_]; b = nb_
                        wps.append((t, X, Yc))
                    t += L / self.v; X = self.x_max
                    wps.append((t, X, Yc)); last[b] = t
                    continue
                # partial out-and-back that ends exactly at t_col_end
                if R > shift + 20.0:
                    if shift > 0:
                        t += shift; Yc = self.band_c[nb_]; b = nb_
                        wps.append((t, X, Yc))
                    R = self.t_col_end - t
                    Xf = self.x_min + min(L, R * self.v / 2)
                    wps.append((t + R / 2, Xf, Yc))
                wps.append((self.t_col_end, self.x_min, Yc))
                break
            else:
                # at x_max: always sweep back. The x_min rule guarantees R >= L/v here.
                back = L / self.v
                if R >= shift + back:
                    if shift > 0:
                        t += shift; Yc = self.band_c[nb_]; b = nb_
                        wps.append((t, X, Yc))
                    t += back
                else:                                  # no time to change band: sweep back slower
                    t = self.t_col_end
                X = self.x_min
                wps.append((t, X, Yc)); last[b] = t
                if t >= self.t_col_end - 1e-6:
                    break
        # hold at x_min after the sweep ends (slots are released then anyway)
        wps.append((self.duration + 600.0, self.x_min, wps[-1][2]))
        self.wps = np.array(wps)
        self.t_grid = np.arange(0.0, self.duration + 600.0 + self.dt, self.dt)
        self.X = np.interp(self.t_grid, self.wps[:, 0], self.wps[:, 1])
        self.Yc = np.interp(self.t_grid, self.wps[:, 0], self.wps[:, 2])

    def column(self, t):
        """(X, Yc, vX, vY) of the column centre at time t."""
        w = self.wps
        X = np.interp(t, w[:, 0], w[:, 1])
        Y = np.interp(t, w[:, 0], w[:, 2])
        i = int(np.clip(np.searchsorted(w[:, 0], t, side="right") - 1, 0, len(w) - 2))
        dtw = w[i + 1, 0] - w[i, 0]
        if dtw <= 1e-9:
            return X, Y, 0.0, 0.0
        return X, Y, (w[i + 1, 1] - w[i, 1]) / dtw, (w[i + 1, 2] - w[i, 2]) / dtw

    def lane_y(self, i, Yc):
        return Yc + (i - (self.k - 1) / 2.0) * self.spacing

    def idx(self, t):
        return int(np.clip(round(t / self.dt), 0, len(self.t_grid) - 1))


class Slot:
    """A role position that must be occupied: relay station, surveyor lane or shadow."""

    def __init__(self, sid, kind, index, alt, rv_offset, vacate_offset=(0.0, 0.0)):
        self.sid = sid
        self.kind = kind            # 'relay' | 'surveyor' | 'shadow'
        self.index = index
        self.alt = alt
        self.rv_offset = np.asarray(rv_offset, float)
        self.vacate_offset = np.asarray(vacate_offset, float)   # where the old occupant steps before climbing
        self.occupant = None        # uid of UAV serving the slot
        self.incoming = None        # uid of UAV dispatched to (re)fill it
        # shadow-only
        self.anchor = None          # sid of the slot whose articulation point it shields
        self.offset = np.zeros(3)
        self.shield_node = None


class Backbone:
    """All slots + time-indexed 'needed' tables for look-ahead dispatch."""

    def __init__(self, cfg, formation):
        s = cfg["swarm"]
        self.cfg = cfg
        self.F = formation
        bb = s["backbone"]
        self.spine_y = float(bb["y"])
        self.lead = float(bb["lead_m"])
        self.station_x = np.linspace(bb["x_first"], bb["x_last"], int(bb["count"]))
        self.relay_alt = float(s["altitudes"]["relay"])
        off = float(s["rendezvous_offset_m"])
        self.slots = []
        for j, x in enumerate(self.station_x):
            self.slots.append(Slot(len(self.slots), "relay", j, self.relay_alt, (off, 0, 0), (0.0, 40.0)))
        for i in range(formation.k):
            self.slots.append(Slot(len(self.slots), "surveyor", i, formation.alt, (0, off, 0), (0.0, -30.0)))
        self.n_fixed = len(self.slots)
        self._tables()

    def _tables(self):
        F = self.F
        tg = F.t_grid
        active = tg <= F.t_col_end
        need = []
        for sl in self.slots:
            if sl.kind == "relay":
                need.append(active & (self.station_x[sl.index] <= F.X + self.lead))
            else:
                need.append(active.copy())
        self.need = np.array(need)
        n = len(tg)
        idx = np.where(self.need, np.arange(n)[None, :], n)
        self.next_need = np.minimum.accumulate(idx[:, ::-1], axis=1)[:, ::-1]

    # ------------------------------------------------------------ queries
    def pos(self, sl, t, slots=None):
        if sl.kind == "relay":
            return np.array([self.station_x[sl.index], self.spine_y, sl.alt])
        if sl.kind == "surveyor":
            X, Yc, _, _ = self.F.column(t)
            return np.array([X, self.F.lane_y(sl.index, Yc), sl.alt])
        anchor = (slots or self.slots)[sl.anchor]
        return self.pos(anchor, t, slots) + sl.offset

    def vel(self, sl, t, slots=None):
        if sl.kind == "relay":
            return np.zeros(3)
        if sl.kind == "surveyor":
            _, _, vx, vy = self.F.column(t)
            return np.array([vx, vy, 0.0])
        return self.vel((slots or self.slots)[sl.anchor], t, slots)

    def needed(self, sl, t):
        if sl.kind == "shadow":
            return True
        return bool(self.need[sl.sid, self.F.idx(t)])

    def next_needed_t(self, sl, t):
        """Earliest time >= t the slot is needed (inf if never again)."""
        if sl.kind == "shadow":
            return t
        k = self.next_need[sl.sid, self.F.idx(t)]
        return float("inf") if k >= len(self.F.t_grid) else float(self.F.t_grid[k])

    def needed_until(self, sl, t):
        """End of the current 'needed' interval containing t (t if not needed)."""
        if sl.kind == "shadow":
            return float("inf")
        k = self.F.idx(t)
        row = self.need[sl.sid]
        if not row[k]:
            return t
        off = np.argmin(row[k:])
        if row[k + off]:
            return float(self.F.t_grid[-1])
        return float(self.F.t_grid[k + off])
