"""Plots + results/summary.md from batch_runs.csv, fleet_sweep.csv and one demo run.

    python3 analysis/plot_results.py
"""
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RES = os.path.join(ROOT, "results")
PLOTS = os.path.join(RES, "plots")
sys.path.insert(0, ROOT)

COL = {"zerogap": "#2a78d6", "baseline": "#eb6834"}     # reference palette slots 1-2
LABEL = {"zerogap": "ZERO-GAP", "baseline": "Baseline"}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"

plt.rcParams.update({
    "font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2, "xtick.color": INK2,
    "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
})

METRICS = [
    ("pois_on_time_pct", "POIs reported <= 10 s (%)"),
    ("connectivity_availability_pct", "Surveyor connectivity (%)"),
    ("handovers_zero_gap_pct", "Zero-gap handovers (%)"),
    ("single_failure_retention_pct", "Single-failure retention (%)"),
    ("pdr_pct", "Packet delivery ratio (%)"),
    ("proxy_score", "Proxy score (/95)"),
]


def comparison(df):
    fig, axes = plt.subplots(2, 3, figsize=(12, 6.5))
    conds = [(False, "No faults"), (True, "With faults")]
    w = 0.36
    for ax, (key, title) in zip(axes.flat, METRICS):
        for j, mode in enumerate(("baseline", "zerogap")):
            means, sds = [], []
            for f, _ in conds:
                v = df[(df["mode"] == mode) & (df["faults"] == f)][key]
                means.append(v.mean())
                sds.append(v.std())
            x = np.arange(len(conds)) + (j - 0.5) * (w + 0.02)
            ax.bar(x, means, w, yerr=sds, color=COL[mode], label=LABEL[mode],
                   error_kw=dict(ecolor=INK2, lw=1, capsize=3))
            for xi, m in zip(x, means):
                ax.text(xi, m / 2, f"{m:.0f}", ha="center", va="center", color="white", fontsize=9)
        ax.set_xticks(np.arange(len(conds)))
        ax.set_xticklabels([c[1] for c in conds])
        ax.set_title(title, color=INK, fontsize=10, loc="left")
        ax.grid(axis="x", visible=False)
    axes[0, 0].legend(frameon=False, loc="lower left")
    n = df.groupby(["mode", "faults"]).size().min()
    fig.suptitle(f"ZERO-GAP vs baseline - mean +/- sd over {n} seeds per condition "
                 f"(N = {int(df['fleet_size'].iloc[0])} UAVs)", color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, "comparison.png"), dpi=140)
    plt.close(fig)


def delays(df):
    fig, ax = plt.subplots(figsize=(7, 3.6))
    data, labels, cols = [], [], []
    for mode in ("baseline", "zerogap"):
        for f, fl in ((False, "no faults"), (True, "faults")):
            v = df[(df["mode"] == mode) & (df["faults"] == f)]["report_delay_max_s"].dropna()
            data.append(v.values)
            labels.append(f"{LABEL[mode]}\n{fl}")
            cols.append(COL[mode])
    bp = ax.boxplot(data, labels=labels, patch_artist=True, widths=0.5, medianprops=dict(color=INK))
    for b, c in zip(bp["boxes"], cols):
        b.set_facecolor(c)
        b.set_alpha(0.85)
    ax.axhline(10, color=INK2, lw=1, ls="--")
    ax.text(4.45, 10, "10 s limit", va="bottom", ha="right", color=INK2, fontsize=9)
    ax.set_yscale("symlog", linthresh=1)
    ax.set_ylabel("worst POI report delay per run (s)")
    ax.set_title("Detection-to-center report delay", loc="left", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, "report_delay.png"), dpi=140)
    plt.close(fig)


def sweep(sw):
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))
    for ax, (key, title) in zip(axes, [("connectivity_availability_pct", "Surveyor connectivity (%)"),
                                       ("handovers_zero_gap_pct", "Zero-gap handovers (%)"),
                                       ("proxy_score", "Proxy score (/95)")]):
        for mode in ("baseline", "zerogap"):
            g = sw[sw["mode"] == mode].groupby("fleet_size")[key]
            m, s = g.mean(), g.std().fillna(0)
            x, y, e = m.index.to_numpy(), m.to_numpy(), s.to_numpy()
            ax.fill_between(x, y - e, y + e, color=COL[mode], alpha=0.15, lw=0)
            ax.plot(x, y, color=COL[mode], lw=2, marker="o", ms=5, label=LABEL[mode])
            ax.text(x[-1] + 0.3, y[-1], LABEL[mode], color=INK2, fontsize=9, va="center")
        ax.set_title(title, loc="left", color=INK)
        ax.set_xlabel("fleet size N")
        ax.set_xlim(x[0] - 1, x[-1] + 5)
    axes[0].legend(frameon=False, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, "fleet_sweep.png"), dpi=140)
    plt.close(fig)


def timeline(tl, handovers, title):
    fig, axes = plt.subplots(3, 1, figsize=(10, 6.5), sharex=True)
    t = (tl["t"] / 60).to_numpy()
    tl = {k: tl[k].to_numpy() for k in tl.columns}
    axes[0].plot(t, tl["surv_connected"], color=COL["zerogap"], lw=1.5)
    axes[0].set_ylabel("surveyors\nconnected")
    axes[0].set_ylim(-0.3, 7.5)
    axes[1].plot(t, tl["airborne"], color=COL["zerogap"], lw=1.5, label="airborne")
    axes[1].plot(t, tl["ready"], color=INK2, lw=1.2, label="ready on ground")
    axes[1].legend(frameon=False, loc="upper right", ncol=2)
    axes[1].set_ylabel("UAVs")
    axes[2].plot(t, tl["column_x"], color=COL["zerogap"], lw=1.5)
    axes[2].set_ylabel("column x (m)")
    axes[2].set_xlabel("mission time (min)")
    hb = handovers[handovers["kind"] == "baton_mbb"]
    for ax in axes:
        for tt in hb["t"] / 60:
            ax.axvline(tt, color=GRID, lw=0.8, zorder=0)
    axes[0].set_title(title + "  (grey ticks = make-before-break handovers)", loc="left", color=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, "timeline.png"), dpi=140)
    plt.close(fig)


def fmt(v):
    return f"{v.mean():.1f} +/- {v.std():.1f}"


def summary(df, sw, sizing_md):
    rows = [
        ("POIs found (of 10)", "pois_found"), ("POIs on time <= 10 s (%)", "pois_on_time_pct"),
        ("mean report delay (s)", "report_delay_mean_s"), ("max report delay (s)", "report_delay_max_s"),
        ("mission completion time (s)", "mission_completion_s"),
        ("PDR all packets (%)", "pdr_pct"), ("report PDR (%)", "report_pdr_pct"),
        ("latency mean (ms)", "latency_mean_ms"), ("latency p95 (ms)", "latency_p95_ms"),
        ("surveyor connectivity availability (%)", "connectivity_availability_pct"),
        ("all 7 surveyors connected (% time)", "all_surveyors_connected_pct"),
        ("comm downtime (surveyor-s)", "comm_downtime_surveyor_s"),
        ("relay/role handovers", "handovers"), ("make-before-break handovers", "handovers_mbb"),
        ("zero-gap handovers (%)", "handovers_zero_gap_pct"), ("mean handover gap (s)", "handover_gap_mean_s"),
        ("failures detected", "failures"), ("recovery time mean (s)", "recovery_mean_s"),
        ("single-failure tolerance (% time, strict)", "single_failure_tolerance_pct"),
        ("single-failure retention (%)", "single_failure_retention_pct"),
        ("shadows deployed", "shadows_deployed"),
        ("min inter-UAV distance (m)", "min_inter_uav_dist_m"),
        ("separation violations (<20 m events)", "separation_violations"),
        ("emergency brake interventions", "emergency_brakes"),
        ("min battery (%)", "min_battery_pct"), ("battery depletions", "battery_depleted"),
        ("geofence violations", "geofence_violations"), ("altitude violations", "altitude_violations"),
        ("UAVs lost", "uavs_lost"), ("proxy score (/95)", "proxy_score"),
    ]
    conds = [("baseline", False), ("zerogap", False), ("baseline", True), ("zerogap", True)]
    n = df.groupby(["mode", "faults"]).size().min()
    L = ["# ZERO-GAP results\n",
         f"All numbers below come from `analysis/run_batch.py`: {n} seeds x {{baseline, zerogap}} x "
         f"{{no faults, faults}}, N = {int(df['fleet_size'].iloc[0])} UAVs, 45-min missions. "
         "Each value is the mean +/- sd over seeds. One row per run is in `results/batch_runs.csv`.\n",
         "| metric | baseline | ZERO-GAP | baseline + faults | ZERO-GAP + faults |",
         "|---|---|---|---|---|"]
    for name, key in rows:
        cells = [fmt(df[(df["mode"] == m) & (df["faults"] == f)][key].astype(float)) for m, f in conds]
        L.append(f"| {name} | " + " | ".join(cells) + " |")
    land = [f"{df[(df['mode'] == m) & (df['faults'] == f)]['all_landed_by_45min'].mean() * 100:.0f}%"
            for m, f in conds]
    L.append("| all UAVs landed by 45:00 (% of runs) | " + " | ".join(land) + " |")
    L.append("\nThe proxy score applies the official weights (mission 25, comm 25, relay/role 20, "
             "fault recovery 15, safety 10) to our own metrics. It is not the official scorer.\n")
    L.append("![comparison](plots/comparison.png)\n\n![report delay](plots/report_delay.png)\n")
    if sw is not None:
        L.append("## Fleet size sweep\n")
        L.append(f"{sw.groupby(['mode', 'fleet_size']).size().min()} seeds per point, no faults.\n")
        L.append("| N | baseline connectivity % | ZERO-GAP connectivity % | baseline score | ZERO-GAP score | ZERO-GAP sep. violations |")
        L.append("|---|---|---|---|---|---|")
        for N, g in sw.groupby("fleet_size"):
            b, z = g[g["mode"] == "baseline"], g[g["mode"] == "zerogap"]
            L.append(f"| {N} | {b['connectivity_availability_pct'].mean():.1f} | "
                     f"{z['connectivity_availability_pct'].mean():.1f} | {b['proxy_score'].mean():.1f} | "
                     f"{z['proxy_score'].mean():.1f} | {z['separation_violations'].mean():.1f} |")
        L.append("\n![fleet sweep](plots/fleet_sweep.png)\n")
    if sizing_md:
        L.append(sizing_md.replace("# Fleet sizing (by hand)", "## Fleet sizing (by hand)"))
    L.append("\n## Single run timeline (seed 1, ZERO-GAP, no faults)\n\n![timeline](plots/timeline.png)\n")
    with open(os.path.join(RES, "summary.md"), "w") as f:
        f.write("\n".join(L) + "\n")


def main():
    os.makedirs(PLOTS, exist_ok=True)
    df = pd.read_csv(os.path.join(RES, "batch_runs.csv"))
    comparison(df)
    delays(df)
    sw = None
    if os.path.exists(os.path.join(RES, "fleet_sweep.csv")):
        sw = pd.read_csv(os.path.join(RES, "fleet_sweep.csv"))
        sweep(sw)
    demo = os.path.join(RES, "demo_run")
    if os.path.exists(os.path.join(demo, "timeline.csv")):
        timeline(pd.read_csv(os.path.join(demo, "timeline.csv")),
                 pd.read_csv(os.path.join(demo, "handovers.csv")), "Seed 1, ZERO-GAP")
    sizing = os.path.join(RES, "fleet_sizing.md")
    summary(df, sw, open(sizing).read() if os.path.exists(sizing) else "")
    print("wrote", os.path.join(RES, "summary.md"), "and", PLOTS)


if __name__ == "__main__":
    main()
