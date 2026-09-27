"""Run metrics. Every number is computed from the simulated run itself."""
import numpy as np


class Metrics:
    def __init__(self):
        self.pkt_rows = []          # (kind, src_uid, t_created, status, latency, hops, tx)
        self.samples = 0
        self.surv_slot_ok = 0       # surveyor-slot-samples served by a connected occupant
        self.surv_slot_total = 0
        self.all_conn_ok = 0        # samples where every occupied surveyor slot was connected
        self.sft_ok = 0
        self.retention_sum = 0.0
        self.downtime_s = 0.0       # surveyor-seconds without a connected occupant
        self.min_batt = np.inf
        self.geofence_viol = 0
        self.alt_viol = 0
        self.geofence_interventions = 0
        self.battery_depleted = 0
        self.emergency_brakes = 0
        self.timeline = []          # per-second dict for plots / ROS replay
        self.recoveries = []

    def packet(self, kind, src, t0, status, lat, hops, tx):
        self.pkt_rows.append((kind, src, t0, status, lat, hops, tx))

    def summary(self, sim):
        pk = self.pkt_rows
        tel = [r for r in pk if r[0] == "telemetry"]
        rep = [r for r in pk if r[0] == "report"]
        delivered = [r for r in pk if r[3] == "delivered"]
        lats = np.array([r[4] for r in delivered]) if delivered else np.array([np.nan])
        pois = sim.pois
        found = [p for p in pois if p["t_detect"] is not None]
        on_time = [p for p in found if p["t_report"] is not None and p["t_report"] - p["t_detect"] <= sim.deadline_s]
        delays = np.array([p["t_report"] - p["t_detect"] for p in found if p["t_report"] is not None])
        reported = [p for p in found if p["t_report"] is not None]
        t_complete = max(p["t_report"] for p in reported) if len(reported) == len(pois) else float("nan")
        ho = [r for r in sim.planner.hlog.records if r["kind"] in ("baton_mbb", "forced", "return_low", "recall")]
        fails = [r for r in sim.planner.hlog.records if r["kind"] == "failure"]
        all_land = all((not u.alive) or u.state in ("GROUND", "SWAP") for u in sim.uavs)
        n = max(self.samples, 1)
        rec = [r["recovery_s"] for r in self.recoveries if r["recovery_s"] is not None]
        return {
            "mode": sim.mode, "faults": sim.faults, "seed": sim.seed, "fleet_size": len(sim.uavs),
            "pois_total": len(pois),
            "pois_found": len(found),
            "pois_reported": len(reported),
            "pois_on_time": len(on_time),
            "pois_on_time_pct": 100.0 * len(on_time) / len(pois),
            "report_delay_mean_s": float(np.mean(delays)) if len(delays) else float("nan"),
            "report_delay_max_s": float(np.max(delays)) if len(delays) else float("nan"),
            "mission_completion_s": t_complete,
            "all_landed_by_45min": bool(all_land),
            "last_landing_s": sim.last_landing,
            "uavs_lost": sum(1 for u in sim.uavs if not u.alive),
            "packets_sent": len(pk),
            "pdr_pct": 100.0 * len(delivered) / max(len(pk), 1),
            "telemetry_pdr_pct": 100.0 * sum(r[3] == "delivered" for r in tel) / max(len(tel), 1),
            "report_pdr_pct": 100.0 * sum(r[3] == "delivered" for r in rep) / max(len(rep), 1),
            "latency_mean_ms": 1000 * float(np.nanmean(lats)),
            "latency_p95_ms": 1000 * float(np.nanpercentile(lats, 95)),
            "connectivity_availability_pct": 100.0 * self.surv_slot_ok / max(self.surv_slot_total, 1),
            "all_surveyors_connected_pct": 100.0 * self.all_conn_ok / n,
            "comm_downtime_surveyor_s": self.downtime_s,
            "handovers": len(ho),
            "handovers_mbb": sum(r["kind"] == "baton_mbb" for r in ho),
            "handover_gap_mean_s": float(np.mean([r["gap_s"] for r in ho])) if ho else 0.0,
            "handover_gap_max_s": float(np.max([r["gap_s"] for r in ho])) if ho else 0.0,
            "handovers_zero_gap_pct": 100.0 * sum(r["gap_s"] <= 0.5 for r in ho) / max(len(ho), 1),
            "failures": len(fails),
            "recovery_mean_s": float(np.mean(rec)) if rec else float("nan"),
            "recovery_max_s": float(np.max(rec)) if rec else float("nan"),
            "single_failure_tolerance_pct": 100.0 * self.sft_ok / n,
            "single_failure_retention_pct": 100.0 * self.retention_sum / n,
            "shadows_deployed": len(sim.planner.shield_log),
            "min_inter_uav_dist_m": sim.sep.min_dist,
            "separation_violations": sim.sep.events,
            "separation_violation_pair_s": sim.sep.pair_ticks * sim.dt,
            "emergency_brakes": self.emergency_brakes,
            "min_battery_pct": 100.0 * self.min_batt / sim.m["uav"]["max_flight_time_s"],
            "battery_depleted": self.battery_depleted,
            "geofence_violations": self.geofence_viol,
            "altitude_violations": self.alt_viol,
            "geofence_interventions": self.geofence_interventions,
        }


def proxy_score(s):
    """Our proxy of the official weighting (NOT the official scorer):
    Mission 25, Comm resilience 25, Relay & role mgmt 20, Fault recovery 15,
    Safety 10 (Innovation 5% is judged, not simulated -> left out, total 95)."""
    mission = 0.5 * s["pois_on_time"] / s["pois_total"] + 0.5 * s["pois_found"] / s["pois_total"]
    if not s["all_landed_by_45min"]:
        mission *= 0.5
    comm = 0.5 * s["connectivity_availability_pct"] / 100 + 0.25 * s["pdr_pct"] / 100 \
        + 0.25 * s["single_failure_retention_pct"] / 100
    relay = s["handovers_zero_gap_pct"] / 100 if s["handovers"] else 1.0
    if s["failures"]:
        r = s["recovery_mean_s"]
        fault = 0.0 if np.isnan(r) else float(np.clip(1 - r / 120.0, 0, 1))
    else:
        fault = 1.0
    safety = 1.0 if (s["separation_violations"] == 0 and s["battery_depleted"] == 0
                     and s["geofence_violations"] == 0 and s["altitude_violations"] == 0) else \
        max(0.0, 1.0 - 0.1 * s["separation_violations"] - 0.5 * s["battery_depleted"])
    return 25 * mission + 25 * comm + 20 * relay + 15 * fault + 10 * safety
