# ZERO-GAP Swarm

**Team:** _<team name>_
**Authors:** Saianshi Mohapatra (lead), Esha Agrawal, Raj Srivastava

Stage-1 simulation entry for IIT Bombay Techfest PUSHPAK Grand Challenge 1,
"UAV-X: Resilient BVLOS Swarm Challenge". A UAV swarm searches a 1 km x 1 km area for
10 randomly appearing POIs and must get every report to the ground station within 10 s,
over 100 m radio links, with 20-min batteries, inside a 45-min mission.

ZERO-GAP keeps the swarm connected with a **comb-shaped relay backbone** and two ideas:

* **Relay Baton protocol (make-before-break).** Every UAV tracks
  `time_to_must_return = battery - time_home - 60 s`. A charged relief is dispatched
  early enough to be beside the slot before that hits zero. The old UAV leaves only
  after the live comm graph confirms that removing it keeps every role node connected.
  Reliefs intercept moving slots using the shared, deterministic sweep schedule.
  UAVs flying home with battery to spare are re-tasked instead of landing.
* **Articulation-Point Shield.** Tarjan's algorithm runs on the live graph every tick.
  The articulation points that would cut off the most surveyors get a SHADOW relay
  that adds a second path.

![snapshot](docs/figures/snapshot_t0600.png)

Demo video (45-min mission, 22 s): [`results/demo_zerogap.mp4`](results/demo_zerogap.mp4).
More frames: `docs/figures/snapshot_t1420.png` (relief in progress), `docs/figures/snapshot_t2150.png`.

## Results (20 seeds per condition, N = 38, all numbers from actual runs)

| metric (mean over seeds) | baseline | ZERO-GAP | baseline + faults | ZERO-GAP + faults |
|---|---|---|---|---|
| POIs reported on time (<= 10 s) | 65.5 % | **90.5 %** | 62.0 % | **87.0 %** |
| POIs found (of 10) | 8.85 | 9.05 | 8.85 | 8.85 |
| worst report delay per run | 259 s | **0.5 s** | 256 s | **7.2 s** |
| surveyor connectivity availability | 50.7 % | **94.8 %** | 48.3 % | **90.4 %** |
| zero-gap handovers | 0 % | **78.8 %** | 0 % | **75.0 %** |
| mean handover gap | 194 s | **16 s** | 193 s | **24 s** |
| recovery time after a UAV kill | - | - | 117 s | **70 s** |
| single-failure retention | 37 % | **78 %** | 36 % | **76 %** |
| packet delivery ratio | 86.0 % | **97.8 %** | 81.3 % | **95.8 %** |
| separation violations (<20 m) per run | 1.0 | **0.0** | 0.85 | 0.6 |
| battery depletions | 0 | 0 | 0 | 0 |
| all UAVs landed by 45:00 | 100 % | 100 % | 100 % | 100 % |

Full table, fleet sweep and plots: [`results/summary.md`](results/summary.md).
Assumptions and known limitations: [`docs/assumptions.md`](docs/assumptions.md).

## Gazebo demo (PX4 SITL)

Branch `gazebo-demo`. Replays two baton passes from the seed-1 run (relay stations 2 and 3,
sim t = 795-880 s) on 4 stock PX4 iris vehicles in Gazebo classic, scaled 1:4 in space and
4x in time, over MAVLink (pymavlink, `SET_POSITION_TARGET_LOCAL_NED` position + velocity at 10 Hz, OFFBOARD).

```bash
scripts/run_gazebo_demo.sh            # launches 4 PX4 SITL iris (UDP 14541-14544), replays, records
# -> results/gazebo_demo.mp4 (55 s, Gazebo window only)
```

* `gazebo_demo/launch_sitl.sh`: same as PX4's `sitl_multiple_run.sh -m iris -n 4`, with our world
  (`zg_spine.world`: station poles + fixed camera).
* `gazebo_demo/extract_traj.py`: re-runs seed 1 and dumps the 4 UAV tracks (`traj_seed1.csv`).
  It runs with `PYTHONNOUSERSITE=1` because `zg` needs numpy < 2.
* `gazebo_demo/px4_replay.py`: arm, OFFBOARD, climb to separate layers, move to start, replay, land.
* Needs `~/PX4-Autopilot` v1.14 built for gazebo-classic, and pymavlink.
* Setpoints carry position plus trajectory velocity feed-forward; moving vehicles track within
  about 3 m, relays on station within 0.1 m. The camera sits low and looks up so the black iris
  airframes show against the sky.

## Install

Tested on Ubuntu 22.04 + ROS 2 Humble, Python 3.10.

```bash
git clone <repo> zero_gap_swarm && cd zero_gap_swarm
scripts/install.sh          # pip --user deps (numpy<1.25 for ROS/scipy ABI), builds ros2_ws
python3 -m pytest -q tests  # 15 tests: Tarjan, routing, baton timing, battery, separation
```

No sudo is needed. The MP4 writer uses the ffmpeg binary shipped with `imageio-ffmpeg`.

## Run

```bash
# one headless 45-min mission (~40 s), prints every metric
python3 -m zg.sim --seed 1                       # ZERO-GAP
python3 -m zg.sim --seed 1 --mode baseline       # baseline
python3 -m zg.sim --seed 1 --faults --out results/my_run   # with chaos, save CSV logs

# live animation
scripts/run_demo.sh 1 zerogap            # add --faults as 3rd arg for chaos

# record the demo video
scripts/record_video.sh results/demo_zerogap.mp4 1

# ROS 2 wrapper: publishes zg/uav_states, zg/comm_graph, zg/poi_reports, zg/markers
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch zg_bringup demo.launch.py seed:=1 mode:=zerogap faults:=false speedup:=10.0
```

## Reproduce every number

```bash
scripts/run_batch.sh      # fleet sizing, 80-run batch, fleet sweep, demo run, plots, summary.md
```

This takes about 15 min on 12 cores. Outputs:

* `results/batch_runs.csv`: one row per run (20 seeds x {baseline, zerogap} x {no faults, faults})
* `results/fleet_sweep.csv`: N = 24 ... 40
* `results/summary.md`: tables and plots (`results/plots/*.png`)
* `results/fleet_sizing.md`: analytical fleet sizing
* `results/demo_run/`: per-packet, handover, POI, shadow and timeline logs of one run

## Fleet sizing

`python3 analysis/fleet_sizing.py`:

* The far corners are 1185.6 m from the center. A pure chain at <= 90 m per hop needs 14 hops,
  i.e. **13 relays**.
* Surveyors: 7 lanes of 71.4 m (< 2 x 40 m footprint) cover a 500 m band. The column sweeps
  two bands.
* Comb peak: 13 spine stations + 7 surveyors = 20 slots (13.6 on average).
* Duty-cycle rotation (battery 1200 s minus trips minus 60 s margin minus 60 s handover
  overlap, plus 120 s swap) gives 22.5 UAVs. Adding 3 shadows and 2 fault reserves gives an
  **analytic N = 28**.
* The simulated sweep shows connectivity still climbing to about 95 % at N = 38. Everyone
  starts charged, so the first battery wave runs out together, and the steady-state model
  misses this. We use **N = 38** as the default.

## Layout

```
config/          mission.yaml (hard limits), swarm.yaml (planner), chaos.yaml (faults)
zg/              comm, backbone, planner, baton, ap_shield, safety, chaos, metrics, sim, viz
analysis/        fleet_sizing.py, run_batch.py, plot_results.py, draw_diagrams.py
ros2_ws/src/zg_bringup/   swarm_sim + gcs_monitor nodes, demo.launch.py
scripts/         install / run_demo / run_batch / record_video
tests/           pytest suite
results/         batch CSVs, summary.md, plots, demo video
docs/            architecture.md (mermaid), architecture.png, assumptions.md, figures/
```

## Stage 2 plan

* Move the controllers onto PX4 SITL + Gazebo (offboard setpoints from the planner), starting
  with 3-4 vehicles on one spine segment.
* Decentralise the Baton check: each relay verifies make-before-break from its 2-hop
  neighbourhood, so the GCS is no longer a single point of decision.
* Conveyor rotation: slide relays inward along the spine by battery level to cut transit time.
* Adaptive column reach: shorten a leg when the predicted relay supply cannot cover the full
  spine, instead of flying out disconnected.
* Replace the distance-only link model with a measured RSSI / packet-loss model.
