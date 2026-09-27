# Architecture

## Module map

```mermaid
flowchart LR
    subgraph config
        M[mission.yaml<br/>hard limits]
        S[swarm.yaml<br/>planner params]
        C[chaos.yaml<br/>faults]
    end
    subgraph zg
        BB[backbone.py<br/>sweep schedule + slots]
        PL[planner.py<br/>dispatch / recall / shadows]
        BT[baton.py<br/>ttmr, intercept, MBB check]
        AP[ap_shield.py<br/>Tarjan + shadow placement]
        CM[comm.py<br/>links, -log p routing, ARQ]
        SF[safety.py<br/>repulsion + hard filter]
        CH[chaos.py<br/>kills, jamming, loss]
        SIM[sim.py<br/>tick loop, UAV state machine]
        MT[metrics.py]
        VZ[viz.py]
    end
    M & S & C --> SIM
    SIM --> CM --> AP
    SIM --> PL
    PL --> BB & BT & AP
    SIM --> SF
    CH --> SIM
    SIM --> MT --> R[(results/*.csv, summary.md)]
    SIM --> VZ --> V[(MP4 / PNG)]
    SIM --> ROS[ros2_ws/zg_bringup<br/>swarm_sim + gcs_monitor nodes]
```

## One simulation tick (dt = 0.5 s)

```mermaid
flowchart TD
    A[chaos events] --> B[build comm graph<br/>links <= 100 m, p(d), jamming]
    B --> C[Dijkstra on -log p from GCS<br/>connected set]
    C --> D[Tarjan articulation points<br/>loss per AP]
    D --> E[planner.step]
    E --> E1[failure detection 2 s]
    E --> E2[Baton: make-before-break handovers]
    E --> E3[must-return / 45-min recall / relief dispatch]
    E --> E4[fill + pre-position slots, re-task homing UAVs]
    E --> E5[AP Shield: place shadows]
    E --> E6[launch queue, 1 per 8 s]
    E --> F[POI detection + report packets<br/>store-and-forward, per-hop ARQ]
    F --> G[UAV controllers -> desired velocity]
    G --> H[safety: repulsion, vertical-band gating,<br/>hard separation filter, clamps]
    H --> I[integrate, battery, landing / swap]
    I --> J[metrics sample]
```

## UAV state machine

```mermaid
stateDiagram-v2
    [*] --> GROUND
    GROUND --> QUEUED: assigned to a slot
    QUEUED --> TAKEOFF: launch slot free
    TAKEOFF --> TRANSIT_OUT: at 74 m
    TRANSIT_OUT --> APPROACH: over rendezvous
    APPROACH --> ON_STATION: slot empty
    APPROACH --> HANDOVER_WAIT: slot occupied
    HANDOVER_WAIT --> ON_STATION: graph check OK (old released)
    ON_STATION --> VACATE: relieved / must return / recall / idle
    VACATE --> TRANSIT_HOME: at 97 m
    VACATE --> APPROACH: re-tasked (battery left)
    TRANSIT_HOME --> APPROACH: re-tasked
    TRANSIT_HOME --> LANDING: over pad
    LANDING --> SWAP: touchdown
    SWAP --> GROUND: 120 s swap done
```

## Topology ("comb")

```
 GCS --R0--R1--R2-- ... --Rj            relay stations on y = 500, alt 51 m, ~77 m apart
                          |              (only the stations behind the column are occupied)
                          S0             surveyor column, alt 28 m, lanes 71.4 m apart,
                          S1             sweeps x = 35..965 in two 500 m bands
                          ...
                          S6
 shadows (alt 51 m) sit beside the worst articulation points to add a second path
```

Altitude layers: surveyor 28 m, relay/shadow 51 m, outbound transit 74 m, inbound transit 97 m
(23 m apart, so layer separation alone keeps the 20 m minimum).
