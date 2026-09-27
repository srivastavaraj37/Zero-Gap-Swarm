"""AP Shield: Tarjan articulation points, per-AP surveyor loss and shadow placement."""
import numpy as np

from .comm import GCS


def articulation_points(adj):
    """Iterative Tarjan (lowlink) articulation points of an undirected graph.

    adj: list of neighbour lists. Returns a set of vertex ids."""
    n = len(adj)
    disc = [-1] * n
    low = [0] * n
    parent = [-1] * n
    aps = set()
    timer = 0
    for root in range(n):
        if disc[root] != -1:
            continue
        disc[root] = low[root] = timer
        timer += 1
        root_children = 0
        stack = [(root, iter(adj[root]))]
        while stack:
            v, it = stack[-1]
            advanced = False
            for w in it:
                if disc[w] == -1:
                    parent[w] = v
                    disc[w] = low[w] = timer
                    timer += 1
                    if v == root:
                        root_children += 1
                    stack.append((w, iter(adj[w])))
                    advanced = True
                    break
                elif w != parent[v]:
                    low[v] = min(low[v], disc[w])
            if advanced:
                continue
            stack.pop()
            if stack:
                u = stack[-1][0]
                low[u] = min(low[u], low[v])
                if u != root and low[v] >= disc[u]:
                    aps.add(u)
        if root_children > 1:
            aps.add(root)
    return aps


def adjacency(P, nodes):
    """Neighbour lists restricted to `nodes` (graph node ids), plus local->global map."""
    nodes = list(nodes)
    loc = {g: i for i, g in enumerate(nodes)}
    sub = P[np.ix_(nodes, nodes)] > 0
    adj = [list(np.flatnonzero(sub[i])) for i in range(len(nodes))]
    return adj, nodes, loc


def reachable(adj, start, removed=-1):
    seen = {start}
    st = [start]
    while st:
        v = st.pop()
        for w in adj[v]:
            if w != removed and w not in seen:
                seen.add(w)
                st.append(w)
    return seen


def ap_losses(P, airborne_nodes, surveyor_nodes):
    """For every AP of the component holding the GCS: number of connected
    surveyors that the loss of that single node would cut off.

    Returns (losses dict node->count, connected surveyor set, adj, nodes, loc)."""
    adj, nodes, loc = adjacency(P, [GCS] + list(airborne_nodes))
    comp = reachable(adj, 0)
    surv_loc = {loc[s] for s in surveyor_nodes if s in loc and loc[s] in comp}
    aps = articulation_points(adj)
    losses = {}
    for a in aps:
        if a not in comp or a == 0:
            continue
        r = reachable(adj, 0, removed=a)
        lost = len([s for s in surv_loc if s != a and s not in r])
        if lost:
            losses[nodes[a]] = lost
    connected = {nodes[s] for s in surv_loc}
    return losses, connected, adj, nodes, loc


def resilience(losses, n_connected_surv, n_airborne):
    """Returns (sft, retention). sft is True if no single UAV loss would cut off a surveyor.
    retention is the expected share of surveyors still connected after one random UAV is lost."""
    if n_connected_surv == 0 or n_airborne == 0:
        return False, 0.0
    sft = len(losses) == 0
    exp_lost = sum(losses.values()) / n_airborne
    return sft, 1.0 - exp_lost / n_connected_surv


def place_shadow(ap_node, P, pos, adj, nodes, loc, alt_choices, planning_range,
                 avoid_pts, clearance, geofence_fn, spine_y=None, moving=False):
    """Search positions around the AP that link (<= planning range) the GCS-side
    component with every component the AP would cut off. Returns (point, score)
    or (None, 0)."""
    a = loc[ap_node]
    r_gcs = reachable(adj, 0, removed=a)
    cut = []
    seen = set(r_gcs) | {a}
    for w in adj[a]:
        if w not in seen:
            comp = reachable(adj, w, removed=a)
            seen |= comp
            cut.append(comp)
    if not cut:
        return None, 0.0
    gcs_pts = np.array([pos[nodes[i]] for i in r_gcs])
    cut_pts = [np.array([pos[nodes[i]] for i in c]) for c in cut]
    center = pos[ap_node]
    best, best_score = None, 0.0
    avoid = np.array(avoid_pts) if len(avoid_pts) else np.zeros((0, 3))
    for r in (25.0, 32.0, 40.0, 48.0):
        for ang in np.deg2rad(np.arange(0, 360, 15)):
            for z in alt_choices:
                q = np.array([center[0] + r * np.cos(ang), center[1] + r * np.sin(ang), z])
                if not geofence_fn(q):
                    continue
                if moving and spine_y is not None and abs(q[1] - spine_y) < clearance:
                    continue                  # would drag through the relay spine
                if len(avoid) and np.min(np.linalg.norm(avoid - q, axis=1)) < clearance:
                    continue
                dg = np.min(np.linalg.norm(gcs_pts - q, axis=1))
                if dg > planning_range:
                    continue
                dc = [np.min(np.linalg.norm(cp - q, axis=1)) for cp in cut_pts]
                if max(dc) > planning_range:
                    continue
                score = planning_range - max([dg] + dc)   # worst-link slack
                if score > best_score:
                    best, best_score = q, score
    return best, best_score
