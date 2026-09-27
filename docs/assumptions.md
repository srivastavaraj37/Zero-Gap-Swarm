# Assumptions and known limitations

## Assumptions (not given by the organisers)

| Item | Value | Why |
|---|---|---|
| Battery swap / recharge time | 120 s (`mission.yaml: uav.swap_time_s`) | The charge time is not given. A quick battery swap is realistic for small multirotors. |
| Camera footprint radius | 40 m at 28 m altitude | The sensing radius is not given. Lanes are 71.4 m apart (< 2 x 40 m), so the footprints overlap. |
| POI spawn window | uniform in [30 s, 1800 s] | The rules say "random times during the mission". A POI that appears after the last sweep pass cannot be found by any method, so POIs stop appearing 30 min before the end. The sweep goes on until about 39 min. |
| POI positions | uniform over the 1000 x 1000 m area, seeded | |
| Detection | instant, any airborne UAV at <= 40 m altitude within 40 m horizontally | We do not model an image classifier in Stage 1. |
| GCS antenna height | 10 m | Needed for 3-D link distances to the center. |
| Vertical speed / acceleration | 3 m/s, 4 m/s^2 | Stays inside the 5 m/s speed limit (3-D). |
| Link model | p = 1.0 up to 70 m, linear to 0.7 at 100 m, no link beyond; 20 ms + U(0,10 ms) per hop attempt; 3 link-layer retries | As given in the task. The number of retries is our choice. |
| Telemetry | every airborne UAV sends to the GCS once per second, no store-and-forward | Used for PDR and latency. |
| POI reports | store-and-forward: if there is no route, the current node keeps the report and retries every tick | A report is on time only if it arrives <= 10 s after detection. |
| Planner knowledge | one planner at the GCS sees all UAV states; a failure is noticed after a 2 s heartbeat timeout | A Stage 1 simplification. UAVs that lose the link keep flying the shared fixed schedule, so losing the link never stops the plan. |
| Return trip estimate | 1.2 x (climb + path / 5 m/s + descent) + 60 s margin | Traffic near the corridor makes real trips slower than a straight line. |
| Takeoff / landing | 42 pads on a 24 m grid inside the corridor box x in [-150, 0], y in [420, 580] | The "operational center" is a single point, so we spread the pads around it. |
| Geofence | area U corridor box, altitude <= 100 m | UAVs must be able to fly the 75 m gap to the center. |
| Altitude layers | surveyor 28, relay/shadow 51, outbound 74, inbound 97 m | Layers are 23 m apart, so UAVs on different layers are always more than 20 m apart. |
| Fleet size | N = 38 default | The paper estimate is 28 (`analysis/fleet_sizing.py`). In the simulated sweep, connectivity keeps improving up to about 38, because all UAVs start charged and the first batteries run out at about the same time. |
| Run-to-run variation | without faults the flights are the same every run | Seeds change POI times and places and packet loss. With faults they also change which UAV is killed and when, and where jamming happens. That is why the no-fault connectivity numbers are the same for every seed. |

## Baseline

The baseline uses the same simulator, layout, sweep schedule and safety layer, but:

* there is no Relay Baton: a UAV stays until its own must-return time and then leaves, even if
  no replacement is there yet;
* a slot is only filled once it is needed and already empty;
* there is no AP Shield (no shadows) and no re-tasking of UAVs that are already flying.

## Known limitations

* Handover gaps are not always zero. About 21-25 % of handovers in ZERO-GAP still have a gap.
  This happens when several surveyor batteries run out together and no relief can arrive in
  time, or when a relief has to catch up with the moving column at only 1.5 m/s faster than it.
  See `handover_gap_*` in `results/summary.md`.
* Not every POI is found. ZERO-GAP finds 9.05 of 10 on average (no faults) and reports every
  POI it finds on time. The missed ones appear behind the column late in the mission and are
  not visited again before the final recall (one full pass over the area takes about 13.6 min).
* Connectivity is not 100 %. The rest of the downtime comes from the same battery crunches and
  from the first minutes while the swarm is still taking off.
* Strict single-failure tolerance is low. In a comb (a tree) almost every UAV is an articulation
  point, and 3 shadows can only protect the worst ones. So we also report "single-failure
  retention": the expected share of surveyors that stay connected after one random UAV is lost.
  The shield does raise this number.
* Separation: at the default N = 38 the no-fault sweep had 0 violations. At some other fleet
  sizes a pair of UAVs stayed too close for a while (for example N = 34, seed 1: one pair at
  about 17.5 m for about a minute near the corridor). The hard safety filter stops UAVs from
  closing in, but it can lose when three or more UAVs crowd one spot. Every violation is counted
  in the results.
* Emergency braking: the last-resort separation filter can brake harder than the 4 m/s^2
  acceleration limit. We report how often it steps in (`emergency_brakes`).
* The simulator treats each UAV as a point mass, with no wind and no attitude dynamics. The
  Gazebo demo only replays 4 vehicles on PX4 SITL; the full planner has not run on PX4 yet.
* The proxy score applies the official weights to our own metrics. It is not the official scorer.
