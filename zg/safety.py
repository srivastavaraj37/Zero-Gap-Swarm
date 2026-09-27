"""Safety layer: velocity-level separation (repulsion + priority yielding), speed /
altitude / geofence clamps, and violation accounting.

Primary separation comes from altitude layering (surveyor 30 / relay 52 /
outbound 75 / inbound 97 m: >= 22 m apart) - this layer handles the residual
cases (climbs/descents, handovers, converging transit traffic)."""
import numpy as np


def clamp_speed(v, vmax, vzmax):
    v = v.copy()
    v[..., 2] = np.clip(v[..., 2], -vzmax, vzmax)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    scale = np.where(n > vmax, vmax / np.maximum(n, 1e-9), 1.0)
    return v * scale


def separation_velocity(pos, vdes, prio, active, dmin, guard, lookahead, vmax):
    """Repulsion for pairs that are closing inside `guard` (or already near dmin).

    The lower-priority UAV of a pair does most of the yielding."""
    idx = np.flatnonzero(active)
    v = vdes.copy()
    if len(idx) < 2:
        return v
    p = pos[idx]
    q = p + vdes[idx] * lookahead
    d_now = np.linalg.norm(p[:, None] - p[None], axis=-1)
    d_pred = np.linalg.norm(q[:, None] - q[None], axis=-1)
    trig = ((d_pred < guard) & (d_pred < d_now - 1e-3)) | (d_now < dmin + 2.0)
    np.fill_diagonal(trig, False)
    ii, jj = np.nonzero(np.triu(trig, 1))
    for a, b in zip(ii, jj):
        ua, ub = idx[a], idx[b]
        dvec = p[a] - p[b]
        n = np.linalg.norm(dvec)
        if n < 1e-6:
            dvec, n = np.array([1.0, 0.0, 0.0]), 1.0
        u = dvec / n
        d = min(d_now[a, b], d_pred[a, b])
        mag = vmax * np.clip(2.0 * (guard - d) / (guard - dmin), 0.3, 2.0)
        pa, pb = prio[ua], prio[ub]
        wa = 1.0 if pa < pb else 0.1
        wb = 1.0 if pb < pa else 0.1
        # stacked vertically: also slide sideways so nobody gets pinned
        lat = np.array([u[0], u[1], 0.0])
        if np.hypot(dvec[0], dvec[1]) < 8.0:
            lat = np.array([1.0, 0.0, 0.0]) if ua < ub else np.array([-1.0, 0.0, 0.0])
        for (w, uid, sgn) in ((wa, ua, 1.0), (wb, ub, -1.0)):
            closing = np.dot(v[uid], -sgn * u)
            if closing > 0 and w >= 1.0:
                v[uid] += sgn * u * closing
            v[uid] += sgn * (u + 0.5 * lat) * mag * w
    return v


def hard_filter(pos, vel, prio, active, d_safe, dt, vmax, passes=4):
    """Last-resort projection: cancel any closing speed that would bring a pair
    under d_safe within one tick. Returns (vel, number of pairs braked)."""
    idx = np.flatnonzero(active)
    v = vel.copy()
    braked = 0
    for _ in range(passes):
        q = pos[idx] + v[idx] * dt
        d_next = np.linalg.norm(q[:, None] - q[None], axis=-1)
        np.fill_diagonal(d_next, np.inf)
        ii, jj = np.nonzero(np.triu(d_next < d_safe, 1))
        if len(ii) == 0:
            break
        for a, b in zip(ii, jj):
            ua, ub = idx[a], idx[b]
            dvec = pos[ua] - pos[ub]
            n = np.linalg.norm(dvec)
            u = dvec / n if n > 1e-6 else np.array([0.0, 0.0, 1.0])
            closing = -np.dot(v[ua] - v[ub], u)
            allowed = max(0.0, n - d_safe) / dt
            excess = closing - allowed
            if excess <= 0:
                continue
            braked += 1
            lo, hi = (ua, ub) if prio[ua] < prio[ub] else (ub, ua)
            sgn = 1.0 if lo == ua else -1.0
            v[lo] += sgn * u * excess * 0.8
            v[hi] -= sgn * u * excess * 0.2
    n = np.linalg.norm(v, axis=1, keepdims=True)
    v = np.where(n > vmax, v * vmax / np.maximum(n, 1e-9), v)
    return v, braked


def geofence_ok(p, mission):
    a, c = mission["area"], mission["geofence"]["corridor"]
    in_area = a["xmin"] <= p[0] <= a["xmax"] and a["ymin"] <= p[1] <= a["ymax"]
    in_cor = c["xmin"] <= p[0] <= c["xmax"] and c["ymin"] <= p[1] <= c["ymax"]
    return in_area or in_cor


def clamp_geofence(p, mission):
    """Project a position back into area U corridor, altitude into [0, max]."""
    a, c = mission["area"], mission["geofence"]["corridor"]
    q = p.copy()
    q[2] = np.clip(q[2], 0.0, mission["geofence"]["max_altitude"])
    if geofence_ok(q, mission):
        return q
    qa = np.array([np.clip(q[0], a["xmin"], a["xmax"]), np.clip(q[1], a["ymin"], a["ymax"]), q[2]])
    qc = np.array([np.clip(q[0], c["xmin"], c["xmax"]), np.clip(q[1], c["ymin"], c["ymax"]), q[2]])
    return qa if np.linalg.norm(qa - q) <= np.linalg.norm(qc - q) else qc


class SeparationMonitor:
    """Counts separation violations (a pair's distance first dropping below dmin
    is one event) and tracks the global minimum inter-UAV distance."""

    def __init__(self, dmin):
        self.dmin = dmin
        self.events = 0
        self.pair_ticks = 0
        self.min_dist = np.inf
        self.min_dist_t = None
        self._in = set()
        self.log = []

    def update(self, t, pos, airborne, uids):
        idx = np.flatnonzero(airborne)
        if len(idx) < 2:
            self._in = set()
            return
        p = pos[idx]
        D = np.linalg.norm(p[:, None] - p[None], axis=-1)
        np.fill_diagonal(D, np.inf)
        m = D.min()
        if m < self.min_dist:
            self.min_dist, self.min_dist_t = float(m), t
        ii, jj = np.nonzero(np.triu(D < self.dmin, 1))
        now = {(uids[idx[a]], uids[idx[b]]) for a, b in zip(ii, jj)}
        self.pair_ticks += len(now)
        for pr in now - self._in:
            self.events += 1
            a, b = pr
            self.log.append((t, a, b, float(np.linalg.norm(pos[a] - pos[b]))))
        self._in = now
