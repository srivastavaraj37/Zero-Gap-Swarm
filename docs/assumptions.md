# Assumptions and known limitations

## Assumptions (not given by the organisers)

| Item | Value | Why |
|---|---|---|
| Battery swap / recharge time | 120 s (`mission.yaml: uav.swap_time_s`) | Charge time not specified; a hot swap is realistic for small multirotors. |
| Camera footprint radius | 40 m at 28 m altitude | Sensing radius not specified. Lanes are 71.4 m apart (< 2 x 40 m) so footprints overlap. |
| POI spawn window | uniform in [30 s, 1800 s] | "Random times during the mission". A POI spawning after the last sweep pass cannot be found by any strategy, so spawns stop 30 min before the end (the sweep continues until about 39 min). |
| POI positions | uniform over the 1000 x 1000 m area, seeded | |
| Detection | instant, any airborne UAV at <= 40 m altitude within 40 m horizontally | No classifier model in Stage 1. |
| GCS antenna height | 10 m | Needed for 3-D link distances to the center. |
| Vertical speed / acceleration | 3 m/s, 4 m/s^2 | Within the 5 m/s speed limit (3-D norm). |
| Link model | p = 1.0 up to 70 m, linear to 0.7 at 100 m, none beyond; 20 ms + U(0,10 ms) per hop attempt; 3 link-layer retries | As specified; the ARQ retry count is our choice. |
| Telemetry | every airborne UAV -> GCS once per second, no store-and-forward | Used for PDR / latency. |
| POI reports | store-and-forward: held by the current node if there is no route, retried every tick | On time only if delivered <= 10 s after detection. |
| Planner knowledge | centralised planner at the GCS sees all UAV states; failures are detected after a 2 s heartbeat timeout | Stage-1 simplification. UAVs that lose the link keep flying the shared deterministic schedule, so a disconnection never freezes the plan. |
| Return trip estimate | 1.2 x (climb + path / 5 m/s + descent) + 60 s margin | Traffic around the corridor makes real trips slower than the straight-line estimate. |
| Takeoff / landing | 42 pads on a 24 m grid inside the corridor box x in [-150, 0], y in [420, 580] | The "operational center" is a point; pads spread around it. |
| Geofence | area U corridor box, altitude <= 100 m | The 75 m gap to the center must be flyable. |
| Altitude layers | surveyor 28, relay/shadow 51, outbound 74, inbound 97 m | 23 m apart, so layer separation alone keeps the 20 m minimum. |
| Fleet size | N = 38 default | Analytic steady-state minimum is 28 (`analysis/fleet_sizing.py`); the simulated sweep shows connectivity keeps improving up to about 38, because the 45-min horizon starts everyone charged and the first wave of batteries runs out together. |
| Run-to-run variation | without faults the flight dynamics are deterministic | Seeds change POI times/positions and packet loss; faults change kill times, targets and jamming regions. Connectivity numbers without faults are therefore the same for every seed. |

## Baseline definition

The baseline uses the same simulator, topology, sweep schedule and safety layer, with:

* no Relay Baton: an occupant stays until its own must-return time, then leaves (break-before-make);
* relays and replacements are placed reactively: a slot is only filled once it is needed and empty;
* no AP Shield (no shadows) and no re-tasking of airborne UAVs.

## Known limitations (honest list)

* **Handover gaps are not always zero.** About 21-25 % of role handovers in ZERO-GAP still have a
  gap. They cluster when several surveyor batteries expire together and no relief can be on
  station in time (fleet supply), or when the relief has to chase the column at 1.5 m/s closing
  speed. See `handover_gap_*` in `results/summary.md`.
* **Not every POI is found.** ZERO-GAP finds 9.05 / 10 on average (no faults) and reports every
  found POI on time; the misses are POIs that spawn behind the column late in the mission and are
  not revisited before the final recall (one full area pass takes about 13.6 min).
* **Connectivity is not 100 %.** Remaining downtime comes from the same supply crunches plus the
  first minutes of deployment.
* **Strict single-failure tolerance is low.** In a comb (tree) topology almost every node is an
  articulation point; with 3 shadows we can only protect the worst ones. We also report the graded
  "single-failure retention" (expected share of surveyors that stay connected after a random single
  UAV loss), which the shield does raise.
* **Separation.** At the default N = 38 the no-fault sweep had 0 violations, but some other fleet
  sizes had a persistent close pair (for example N = 34, seed 1: one pair at ~17.5 m for about a minute near the
  corridor). The hard safety filter prevents closing but can be out-voted when three or more UAVs
  crowd one spot. Every violation is counted in the results, never hidden.
* **Emergency braking.** The last-resort separation filter overrides the 4 m/s^2 acceleration
  limit. The number of interventions is reported (`emergency_brakes`).
* **Kinematics are point-mass**; no wind, no attitude dynamics. PX4 SITL was not attempted.
* **Proxy score** uses the official weights on our own metrics; it is not the official scorer.
