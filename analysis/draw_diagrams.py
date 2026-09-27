"""Draws docs/figures/architecture.png and docs/figures/flowchart.png (no mermaid CLI needed).

    python3 analysis/draw_diagrams.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "figures")
INK, INK2, BG = "#0b0b0b", "#52514e", "#fcfcfb"
FILL = {"cfg": "#f1efe8", "core": "#dcebfb", "novel": "#fde3d6", "io": "#e3f4ec"}


def box(ax, x, y, w, h, title, sub="", kind="core"):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.01,rounding_size=0.015",
                                fc=FILL[kind], ec=INK2, lw=1))
    ax.text(x + w / 2, y + h * (0.62 if sub else 0.5), title, ha="center", va="center",
            fontsize=10, color=INK, weight="bold")
    if sub:
        ax.text(x + w / 2, y + h * 0.3, sub, ha="center", va="center", fontsize=7.5, color=INK2)
    return (x, y, w, h)


def arrow(ax, a, b, text=""):
    (x1, y1), (x2, y2) = a, b
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle="-|>", color=INK2, lw=1, shrinkA=2, shrinkB=2))
    if text:
        ax.text((x1 + x2) / 2, (y1 + y2) / 2, text, fontsize=7, color=INK2, ha="center",
                va="bottom", backgroundcolor=BG)


def architecture():
    fig, ax = plt.subplots(figsize=(13, 7), facecolor=BG)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    box(ax, 0.02, 0.78, 0.16, 0.12, "config/", "mission / swarm / chaos .yaml", "cfg")
    box(ax, 0.02, 0.50, 0.16, 0.12, "chaos.py", "kills, jamming, extra loss", "cfg")
    box(ax, 0.26, 0.60, 0.20, 0.22, "sim.py", "tick loop (dt 0.5 s)\nUAV state machine\nsensing, packets, battery")
    box(ax, 0.55, 0.80, 0.19, 0.12, "comm.py", "p(d), -log p Dijkstra, ARQ")
    box(ax, 0.55, 0.60, 0.19, 0.14, "planner.py", "slot filling, recall,\nre-tasking, launch queue")
    box(ax, 0.80, 0.80, 0.18, 0.12, "ap_shield.py", "NOVELTY B: Tarjan APs\n+ shadow placement", "novel")
    box(ax, 0.80, 0.60, 0.18, 0.12, "baton.py", "NOVELTY A: ttmr, intercept,\nmake-before-break", "novel")
    box(ax, 0.55, 0.40, 0.19, 0.12, "backbone.py", "comb: spine stations +\nsurveyor column schedule")
    box(ax, 0.26, 0.32, 0.20, 0.14, "safety.py", "altitude layers, repulsion,\nvertical gating, hard filter")
    box(ax, 0.02, 0.10, 0.18, 0.12, "metrics.py", "per-run metrics + proxy score", "io")
    box(ax, 0.26, 0.10, 0.20, 0.12, "viz.py", "live animation / MP4 / PNG", "io")
    box(ax, 0.52, 0.10, 0.22, 0.12, "ros2_ws/zg_bringup", "swarm_sim + gcs_monitor nodes", "io")
    box(ax, 0.80, 0.10, 0.18, 0.12, "analysis/", "fleet sizing, batch, plots", "io")
    arrow(ax, (0.18, 0.84), (0.26, 0.76), "params")
    arrow(ax, (0.18, 0.56), (0.26, 0.66), "events")
    arrow(ax, (0.46, 0.78), (0.55, 0.85), "positions")
    arrow(ax, (0.74, 0.86), (0.80, 0.86), "graph")
    arrow(ax, (0.46, 0.68), (0.55, 0.67), "state, graph")
    arrow(ax, (0.74, 0.66), (0.80, 0.66))
    arrow(ax, (0.80, 0.82), (0.74, 0.70), "shadows")
    arrow(ax, (0.645, 0.60), (0.645, 0.52), "slot pos(t)")
    arrow(ax, (0.36, 0.60), (0.36, 0.46), "desired v")
    arrow(ax, (0.30, 0.60), (0.11, 0.22), "")
    arrow(ax, (0.265, 0.62), (0.265, 0.22), "")
    arrow(ax, (0.44, 0.60), (0.60, 0.22), "")
    ax.text(0.02, 0.97, "ZERO-GAP software architecture", fontsize=14, color=INK, weight="bold")
    ax.text(0.02, 0.935, "orange = novel algorithms, blue = core simulation, green = outputs / wrappers",
            fontsize=9, color=INK2)
    os.makedirs(OUT, exist_ok=True)
    fig.savefig(os.path.join(OUT, "architecture.png"), dpi=140, bbox_inches="tight")
    fig.savefig(os.path.join(ROOT, "docs", "architecture.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


def flowchart():
    steps = [
        ("Chaos events", "UAV kill / jamming / extra loss", "cfg"),
        ("Comm graph", "links <= 100 m, p(d) 1.0 -> 0.7", "core"),
        ("Routing", "Dijkstra on -log p from GCS", "core"),
        ("Tarjan APs", "loss per articulation point", "novel"),
        ("Planner", "failures, Baton handovers, recall,\nrelief + fill, shadows, launches", "novel"),
        ("POI + packets", "detect r = 40 m, store-and-forward,\nper-hop ARQ, 10 s deadline", "core"),
        ("Controllers", "per-state desired velocity", "core"),
        ("Safety", "repulsion, vertical gating,\nhard 21 m filter, clamps", "core"),
        ("Integrate", "5 m/s, battery, land + swap", "core"),
        ("Metrics", "connectivity, SFT, handover gaps", "io"),
    ]
    fig, ax = plt.subplots(figsize=(7, 11), facecolor=BG)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    h, gap = 0.072, 0.02
    y = 0.93
    prev = None
    for title, sub, kind in steps:
        box(ax, 0.2, y - h, 0.6, h, title, sub, kind)
        if prev is not None:
            arrow(ax, (0.5, prev), (0.5, y))
        prev = y - h
        y -= h + gap
    arrow(ax, (0.8, prev + h / 2), (0.92, prev + h / 2))
    ax.plot([0.92, 0.92], [prev + h / 2, 0.93 - h / 2], color=INK2, lw=1)
    arrow(ax, (0.92, 0.93 - h / 2), (0.8, 0.93 - h / 2))
    ax.text(0.94, 0.5, "next tick (dt = 0.5 s)", rotation=90, va="center", fontsize=8, color=INK2)
    ax.text(0.2, 0.965, "One simulation tick", fontsize=13, weight="bold", color=INK)
    fig.savefig(os.path.join(OUT, "flowchart.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    architecture()
    flowchart()
    print("wrote", OUT)
