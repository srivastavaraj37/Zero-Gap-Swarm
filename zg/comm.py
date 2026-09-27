"""Communication model: link quality, connectivity graph, -log(p) routing, packet delivery.

Node 0 is always the ground control station (GCS); UAV uid u is node u + 1.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

GCS = 0


def link_prob(d, p_full_range=70.0, max_range=100.0, p_at_max=0.7):
    """Per-transmission success probability vs 3-D distance (vectorised)."""
    d = np.asarray(d, dtype=float)
    p = 1.0 - (1.0 - p_at_max) * (d - p_full_range) / (max_range - p_full_range)
    p = np.clip(p, p_at_max, 1.0)
    return np.where(d <= max_range, p, 0.0)


class CommModel:
    def __init__(self, comm_cfg, rng):
        self.c = comm_cfg
        self.rng = rng
        self.loss_factor = 1.0          # chaos: global extra loss
        self.jam_regions = []           # chaos: list of (x, y, radius)

    # ------------------------------------------------------------------ graph
    def build(self, pos, active):
        """pos: (M,3) node positions (row 0 = GCS antenna); active: (M,) bool.

        Returns the link-probability matrix P (0 = no link) for this tick."""
        diff = pos[:, None, :] - pos[None, :, :]
        D = np.sqrt((diff ** 2).sum(-1))
        P = link_prob(D, self.c["p_full_range"], self.c["max_range"], self.c["p_at_max_range"])
        P = P * self.loss_factor
        np.fill_diagonal(P, 0.0)
        mask = active[:, None] & active[None, :]
        if self.jam_regions:
            jammed = np.zeros(len(pos), bool)
            for (jx, jy, jr) in self.jam_regions:
                jammed |= np.hypot(pos[:, 0] - jx, pos[:, 1] - jy) <= jr
            mask &= ~jammed[:, None] & ~jammed[None, :]
        P = np.where(mask, P, 0.0)
        return P, D

    @staticmethod
    def routes_to_gcs(P):
        """Shortest paths to the GCS with cost -log(p). Returns (dist, pred).

        pred[n] is the next hop from n toward the GCS (-9999 if unreachable)."""
        W = np.where(P > 0, -np.log(np.maximum(P, 1e-12)) + 1e-6, 0.0)
        dist, pred = dijkstra(csr_matrix(W), directed=False, indices=GCS,
                              return_predecessors=True)
        return dist, pred

    @staticmethod
    def path(pred, src):
        """Hop list src -> ... -> GCS, or None if src is disconnected."""
        if src == GCS:
            return [GCS]
        if pred[src] < 0:
            return None
        out = [src]
        n = src
        while n != GCS:
            n = pred[n]
            if n < 0:
                return None
            out.append(n)
        return out

    # ---------------------------------------------------------------- packets
    def forward(self, P, path):
        """Hop-by-hop ARQ along `path`. Returns (reached_index, latency_s, tx_attempts).

        reached_index == len(path)-1 means delivered; otherwise the packet is
        stuck (held) at path[reached_index]."""
        tries = 1 + int(self.c["max_retries"])
        lat = 0.0
        attempts = 0
        for h in range(len(path) - 1):
            p = P[path[h], path[h + 1]]
            ok = self.rng.random(tries) < p
            k = int(np.argmax(ok)) if ok.any() else tries
            n_try = min(k + 1, tries)
            attempts += n_try
            lat += n_try * self.c["hop_latency_s"] + self.rng.random(n_try).sum() * self.c["hop_jitter_s"]
            if k >= tries:
                return h, lat, attempts
        return len(path) - 1, lat, attempts


@dataclass
class Packet:
    pid: int
    kind: str            # 'telemetry' | 'report'
    src_uid: int
    t_created: float
    holder: int          # current node holding the packet
    poi: int = -1
    t_delivered: float = float("nan")
    status: str = "in_flight"   # in_flight | delivered | dropped
    hops: int = 0
    tx: int = 0
    extra: dict = field(default_factory=dict)
