"""Runs the ZERO-GAP simulator and publishes UAV states, the comm graph and POI reports."""
import json
import os
import sys

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

ROLE_RGB = {"surveyor": (0.16, 0.47, 0.84), "relay": (0.92, 0.41, 0.20),
            "shadow": (0.11, 0.69, 0.48), "transit": (0.54, 0.54, 0.51)}


class SwarmSim(Node):
    def __init__(self):
        super().__init__("zg_swarm_sim")
        self.declare_parameter("zg_root", os.environ.get("ZG_ROOT", ""))
        self.declare_parameter("seed", 1)
        self.declare_parameter("mode", "zerogap")
        self.declare_parameter("faults", False)
        self.declare_parameter("speedup", 10.0)       # sim seconds per wall second
        root = self.get_parameter("zg_root").value
        if root:
            sys.path.insert(0, root)
        from zg.config import load_config
        from zg.sim import Sim
        from zg.viz import role_of
        self.role_of = role_of
        self.sim = Sim(load_config(), seed=self.get_parameter("seed").value,
                       mode=self.get_parameter("mode").value, faults=self.get_parameter("faults").value)
        self.pub_state = self.create_publisher(String, "zg/uav_states", 10)
        self.pub_graph = self.create_publisher(String, "zg/comm_graph", 10)
        self.pub_poi = self.create_publisher(String, "zg/poi_reports", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "zg/markers", 10)
        self.reported = set()
        speed = float(self.get_parameter("speedup").value)
        self.timer = self.create_timer(self.sim.dt / speed, self.tick)
        self.get_logger().info(f"ZERO-GAP sim started: N={len(self.sim.uavs)} mode={self.sim.mode} x{speed}")

    def tick(self):
        s = self.sim
        if s.t >= s.m["mission_duration_s"]:
            self.get_logger().info("mission finished")
            self.timer.cancel()
            return
        s.step()
        states = [dict(uid=u.uid, state=u.state, role=self.role_of(s, u), alive=u.alive,
                       pos=[round(float(v), 2) for v in u.pos], batt_s=round(float(u.batt), 1),
                       connected=u.uid in s.connected) for u in s.uavs]
        self.pub_state.publish(String(data=json.dumps(dict(t=s.t, uavs=states))))
        n = len(s.uavs) + 1
        edges = [(i, j, round(float(s.P[i, j]), 3)) for i in range(n) for j in range(i + 1, n) if s.P[i, j] > 0]
        self.pub_graph.publish(String(data=json.dumps(dict(t=s.t, edges=edges,
                                                           articulation_points=list(map(int, s.last_losses))))))
        for p in s.pois:
            if p["t_report"] is not None and p["id"] not in self.reported:
                self.reported.add(p["id"])
                self.pub_poi.publish(String(data=json.dumps(dict(
                    poi=p["id"], x=p["x"], y=p["y"], t_detect=p["t_detect"], t_report=p["t_report"],
                    delay_s=p["t_report"] - p["t_detect"]))))
        self.pub_mk.publish(self.markers(states, edges))

    def markers(self, states, edges):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for st in states:
            m = Marker()
            m.header.frame_id, m.header.stamp = "map", stamp
            m.ns, m.id, m.type = "uav", st["uid"], Marker.SPHERE
            m.action = Marker.ADD if st["state"] not in ("GROUND", "SWAP", "DEAD", "QUEUED") else Marker.DELETE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = st["pos"]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 12.0
            m.color.r, m.color.g, m.color.b = ROLE_RGB[st["role"]]
            m.color.a = 1.0
            arr.markers.append(m)
        ln = Marker()
        ln.header.frame_id, ln.header.stamp = "map", stamp
        ln.ns, ln.id, ln.type, ln.action = "links", 0, Marker.LINE_LIST, Marker.ADD
        ln.scale.x = 1.5
        ln.color.r = ln.color.g = ln.color.b = 0.2
        ln.color.a = 0.7
        pos = self.sim.node_pos
        for i, j, _ in edges:
            ln.points.append(Point(x=float(pos[i, 0]), y=float(pos[i, 1]), z=float(pos[i, 2])))
            ln.points.append(Point(x=float(pos[j, 0]), y=float(pos[j, 1]), z=float(pos[j, 2])))
        arr.markers.append(ln)
        return arr


def main():
    rclpy.init()
    node = SwarmSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
