"""Replay a ZERO-GAP baton pass on 4 PX4 SITL iris vehicles over MAVLink (pymavlink).

Vehicle i (1..4) listens on UDP 14540+i and is spawned at Gazebo ENU (0, 3i).
Scene: relay stations 2 and 3 of the seed-1 demo run, scaled 1:8, time 3x.

    python3 gazebo_demo/px4_replay.py            # extracts trajectories first if needed
"""
import argparse
import csv
import os
import subprocess
import sys
import time
from collections import defaultdict

from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
M = mavutil.mavlink

UIDS = [10, 9, 31, 33]          # old relay st2, old relay st3, relief st3, relief st2
SCALE = 8.0                     # 1:8 in space
TSCALE = 3.0                    # sim seconds per wall second
CX, CY, EAST0, NORTH0 = 231.25, 500.0, 20.0, 20.0   # relay station 3 -> Gazebo (20, 20), away from the origin
POS_VEL_YAW = 0b0000100111000000        # use x,y,z + vx,vy,vz (feed-forward) + yaw
OFFBOARD = 6                             # PX4 custom main mode


def load_traj(path):
    tr = defaultdict(list)
    with open(path) as f:
        for r in csv.DictReader(f):
            tr[int(r["uid"])].append((float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"]), r["state"]))
    return tr


def to_enu(x, y, z):
    return (x - CX) / SCALE + EAST0, (y - CY) / SCALE + NORTH0, z / SCALE


def vel_enu(rows, t, h=0.5):
    """Trajectory velocity in the scaled scene, in wall-clock m/s."""
    a = to_enu(*interp(rows, t - h))
    b = to_enu(*interp(rows, t + h))
    return tuple((q - p) / (2 * h) * TSCALE for p, q in zip(a, b))


def interp(rows, t):
    if t <= rows[0][0]:
        return rows[0][1:4]
    for a, b in zip(rows, rows[1:]):
        if a[0] <= t <= b[0]:
            w = (t - a[0]) / (b[0] - a[0])
            return tuple(a[k] + w * (b[k] - a[k]) for k in (1, 2, 3))
    return rows[-1][1:4]


class Vehicle:
    def __init__(self, i):
        self.i = i
        self.m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{14540 + i}", source_system=240 + i)
        self.m.wait_heartbeat(timeout=30)
        self.sys = self.m.target_system
        self.home_n = 3.0 * i                     # spawn offset (Gazebo y = north)
        self.pos = (0.0, self.home_n, 0.0)

    def setpoint(self, e, n, u, ve=0.0, vn=0.0, vu=0.0):
        """ENU world point (+ ENU velocity feed-forward) -> this vehicle's local NED setpoint."""
        self.m.mav.set_position_target_local_ned_send(
            0, self.sys, 1, M.MAV_FRAME_LOCAL_NED, POS_VEL_YAW,
            n - self.home_n, e, -u, vn, ve, -vu, 0, 0, 0, 0.0, 0)

    def cmd(self, command, *p):
        p = list(p) + [0] * (7 - len(p))
        self.m.mav.command_long_send(self.sys, 1, command, 0, *p)

    def drain(self):
        while True:
            x = self.m.recv_match(blocking=False)
            if x is None:
                return
            if x.get_type() == "LOCAL_POSITION_NED":
                self.pos = (x.y, x.x + self.home_n, -x.z)       # back to world ENU


def stream(vs, targets, seconds, rate=10.0):
    for _ in range(int(seconds * rate)):
        for v in vs:
            v.setpoint(*targets[v.i])
            v.drain()
        time.sleep(1.0 / rate)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj", default=os.path.join(HERE, "traj_seed1.csv"))
    ap.add_argument("--hold", type=float, default=3.0, help="seconds to hover before the replay")
    a = ap.parse_args()
    if not os.path.exists(a.traj):
        # zg needs numpy<2 (system scipy); run the extractor with the system site-packages
        env = dict(os.environ, PYTHONNOUSERSITE="1")
        subprocess.run([sys.executable, os.path.join(HERE, "extract_traj.py"), "--out", a.traj],
                       check=True, env=env, cwd=ROOT)
    tr = load_traj(a.traj)
    t0 = tr[UIDS[0]][0][0]
    t1 = tr[UIDS[0]][-1][0]
    start = {i + 1: to_enu(*interp(tr[u], t0)) for i, u in enumerate(UIDS)}

    vs = [Vehicle(i) for i in range(1, 5)]
    print("connected:", [v.sys for v in vs], flush=True)
    ground = {v.i: (0.0, v.home_n, 0.0) for v in vs}

    # offboard needs a setpoint stream before the mode switch
    layer = {1: 6.0, 2: 9.0, 3: 12.0, 4: 15.0}
    climb = {v.i: (0.0, v.home_n, layer[v.i]) for v in vs}
    stream(vs, climb, 1.0)
    for v in vs:
        v.cmd(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, OFFBOARD)
        v.cmd(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    print("armed, offboard, climbing to layers", flush=True)
    stream(vs, climb, 7.0)

    # move to replay start on separate layers, then settle to the replay altitude
    stage = {i: (start[i][0], start[i][1], layer[i] + 6.0) for i in start}
    stream(vs, stage, 9.0)
    stream(vs, start, 4.0)
    open("/tmp/zg_replay_start", "w").close()          # recorder starts here
    stream(vs, start, a.hold)

    print(f"replay sim t={t0:.0f}..{t1:.0f} s at {TSCALE:.0f}x", flush=True)
    w0 = time.time()
    next_log = 0.0
    while True:
        ts = t0 + (time.time() - w0) * TSCALE
        if ts > t1:
            break
        errs = []
        for i, u in enumerate(UIDS):
            tgt = to_enu(*interp(tr[u], ts))
            vs[i].setpoint(*tgt, *vel_enu(tr[u], ts))
            vs[i].drain()
            errs.append(sum((a - b) ** 2 for a, b in zip(tgt, vs[i].pos)) ** 0.5)
        if ts >= next_log:
            print(f"  sim t={ts:5.1f}  tracking error m: " + " ".join(f"{e:4.1f}" for e in errs), flush=True)
            next_log = ts + 10.0
        time.sleep(0.1)
    end = {i + 1: to_enu(*interp(tr[u], t1)) for i, u in enumerate(UIDS)}
    stream(vs, end, 2.0)
    for v in vs:
        v.cmd(M.MAV_CMD_NAV_LAND)
    print("landing", flush=True)
    time.sleep(8)
    _ = ground


if __name__ == "__main__":
    main()
