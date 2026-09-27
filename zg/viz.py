"""Live / recorded animation of a run.

    python3 -m zg.viz                          # live window
    python3 -m zg.viz --save demo.mp4          # MP4 via ffmpeg (imageio-ffmpeg binary)
    python3 -m zg.viz --snapshots 300,1400,2200 --outdir docs/figures
"""
import argparse
import os

import matplotlib
import numpy as np

from .config import load_config
from .sim import Sim

ROLE_COL = {"surveyor": "#2a78d6", "relay": "#eb6834", "shadow": "#1baf7a", "transit": "#8a8983"}
INK2 = "#52514e"


def role_of(sim, u):
    if u.state in ("ON_STATION", "HANDOVER_WAIT") and u.slot is not None:
        return sim.planner.slots[u.slot].kind
    return "transit"


class Viewer:
    def __init__(self, sim, plt):
        self.sim, self.plt = sim, plt
        self.fig = plt.figure(figsize=(12.8, 7.2), facecolor="#fcfcfb")
        gs = self.fig.add_gridspec(1, 2, width_ratios=[3.2, 1], wspace=0.08)
        self.ax = self.fig.add_subplot(gs[0])
        self.axb = self.fig.add_subplot(gs[1])

    def draw(self):
        sim, ax, axb = self.sim, self.ax, self.axb
        ax.clear()
        axb.clear()
        t = sim.t
        age = np.clip((t - sim.coverage.T) / 600.0, 0, 1)          # 0 fresh .. 1 old / never
        ax.imshow(age, origin="lower", extent=(0, 1000, 0, 1000), cmap="Blues_r",
                  vmin=-0.6, vmax=1.0, alpha=0.35, zorder=0)
        ax.add_patch(matplotlib.patches.Rectangle((0, 0), 1000, 1000, fill=False, ec=INK2, lw=1))
        c = sim.m["geofence"]["corridor"]
        ax.add_patch(matplotlib.patches.Rectangle((c["xmin"], c["ymin"]), c["xmax"] - c["xmin"],
                                                  c["ymax"] - c["ymin"], fill=False, ec=INK2, lw=0.8, ls=":"))
        gx, gy = sim.m["center"]["xy"]
        ax.plot(gx, gy, marker="s", ms=10, color="#0b0b0b", zorder=5)
        ax.text(gx, gy - 45, "GCS", ha="center", fontsize=8, color=INK2)
        # links, darker = better
        P = sim.P
        idx = [sim.node(u) for u in sim.uavs if sim.airborne(u)] + [0]
        pos = sim.node_pos
        for i in idx:
            for j in idx:
                if j <= i or P[i, j] <= 0:
                    continue
                p = P[i, j]
                col = "#e34948" if p < 0.8 else "#0b0b0b"
                ax.plot([pos[i, 0], pos[j, 0]], [pos[i, 1], pos[j, 1]], color=col,
                        lw=0.6 + 1.2 * (p - 0.7) / 0.3, alpha=0.25 + 0.5 * (p - 0.7) / 0.3, zorder=1)
        # POIs
        for p in sim.pois:
            if p["t_spawn"] > t:
                continue
            if p["t_detect"] is None:
                ax.plot(p["x"], p["y"], marker="*", ms=12, mfc="none", mec="#e34948", zorder=3)
            else:
                ok = p["t_report"] is not None and p["t_report"] - p["t_detect"] <= 10
                ax.plot(p["x"], p["y"], marker="*", ms=12, color="#008300" if ok else "#eda100", zorder=3)
        # UAVs
        for u in sim.uavs:
            if not sim.airborne(u):
                continue
            r = role_of(sim, u)
            ax.plot(u.pos[0], u.pos[1], "o", ms=7, color=ROLE_COL[r], mec="#fcfcfb", mew=1.2, zorder=4)
            if r == "surveyor":
                ax.add_patch(matplotlib.patches.Circle(u.pos[:2], sim.m["poi"]["sensing_radius"],
                                                       fill=False, ec=ROLE_COL[r], lw=0.6, alpha=0.6))
        ax.set_xlim(-160, 1010)
        ax.set_ylim(-10, 1010)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)
        found = sum(p["t_detect"] is not None for p in sim.pois)
        ok = sum(p["t_report"] is not None and p["t_report"] - p["t_detect"] <= 10
                 for p in sim.pois if p["t_detect"] is not None)
        n_surv = sum(1 for sl in sim.planner.slots if sl.kind == "surveyor" and sl.occupant in sim.connected)
        mm, ss = divmod(int(t), 60)
        ax.set_title(f"t = {mm:02d}:{ss:02d}   mode = {sim.mode}{'  + faults' if sim.faults else ''}\n"
                     f"airborne {sum(sim.airborne(u) for u in sim.uavs)}/{len(sim.uavs)}   surveyors connected "
                     f"{n_surv}/7   POIs found {found}/10, on time {ok}   (blue shading = recently searched)",
                     loc="left", fontsize=9.5)
        handles = [matplotlib.lines.Line2D([], [], marker="o", ls="", color=v, label=k)
                   for k, v in ROLE_COL.items()]
        handles += [matplotlib.lines.Line2D([], [], color="#0b0b0b", label="link p >= 0.8"),
                    matplotlib.lines.Line2D([], [], color="#e34948", label="link p < 0.8"),
                    matplotlib.lines.Line2D([], [], marker="*", ls="", color="#008300", label="POI reported"),
                    matplotlib.lines.Line2D([], [], marker="*", ls="", mfc="none", mec="#e34948",
                                            label="POI not yet found")]
        ax.legend(handles=handles, loc="lower left", fontsize=7, frameon=True, framealpha=0.85, ncol=2)
        # battery bars
        b = np.array([u.batt / sim.m["uav"]["max_flight_time_s"] * 100 if u.alive else 0 for u in sim.uavs])
        cols = [ROLE_COL[role_of(sim, u)] if sim.airborne(u) else "#d3d2cc" for u in sim.uavs]
        axb.barh(np.arange(len(b)), b, color=cols, height=0.7)
        axb.set_xlim(0, 100)
        axb.set_ylim(-1, len(b))
        axb.invert_yaxis()
        axb.set_yticks(np.arange(len(b)))
        axb.set_yticklabels([f"u{u.uid} {u.state[:5].lower()}" for u in sim.uavs], fontsize=5.5)
        axb.set_xlabel("battery %", fontsize=8)
        axb.tick_params(axis="x", labelsize=7)
        axb.set_title("fleet (grey = on ground)", fontsize=9, loc="left")
        for s in ("top", "right"):
            axb.spines[s].set_visible(False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--mode", default=None)
    ap.add_argument("--faults", action="store_true")
    ap.add_argument("--save", default=None, help="write MP4 here instead of a live window")
    ap.add_argument("--every", type=int, default=10, help="sim steps per frame")
    ap.add_argument("--fps", type=int, default=24)
    ap.add_argument("--t-end", type=float, default=None)
    ap.add_argument("--snapshots", default=None, help="comma separated times (s) to save as PNG")
    ap.add_argument("--outdir", default="docs/figures")
    a = ap.parse_args()
    if a.save or a.snapshots:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    sim = Sim(load_config(), seed=a.seed, mode=a.mode, faults=a.faults)
    t_end = a.t_end or sim.m["mission_duration_s"]
    view = Viewer(sim, plt)

    if a.snapshots:
        os.makedirs(a.outdir, exist_ok=True)
        for ts in sorted(float(x) for x in a.snapshots.split(",")):
            while sim.t < ts:
                sim.step()
            view.draw()
            path = os.path.join(a.outdir, f"snapshot_t{int(ts):04d}.png")
            view.fig.savefig(path, dpi=110)
            print("saved", path)
        return

    n_frames = int(t_end / sim.dt / a.every)

    def update(_):
        for _ in range(a.every):
            if sim.t < t_end:
                sim.step()
        view.draw()
        return []

    anim = animation.FuncAnimation(view.fig, update, frames=n_frames, interval=1000 / a.fps, blit=False)
    if a.save:
        import imageio_ffmpeg
        plt.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
        writer = animation.FFMpegWriter(fps=a.fps, bitrate=2400)
        anim.save(a.save, writer=writer, dpi=100)
        print("saved", a.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
