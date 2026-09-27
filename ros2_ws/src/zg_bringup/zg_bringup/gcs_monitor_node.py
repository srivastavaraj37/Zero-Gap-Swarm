"""GCS-side monitor: logs POI reports and connectivity once per simulated minute."""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GcsMonitor(Node):
    def __init__(self):
        super().__init__("zg_gcs_monitor")
        self.create_subscription(String, "zg/poi_reports", self.on_poi, 10)
        self.create_subscription(String, "zg/uav_states", self.on_state, 10)
        self.last_min = -1

    def on_poi(self, msg):
        r = json.loads(msg.data)
        ok = "ON TIME" if r["delay_s"] <= 10 else "LATE"
        self.get_logger().info(f"POI {r['poi']} at ({r['x']:.0f},{r['y']:.0f}) reported in {r['delay_s']:.2f} s [{ok}]")

    def on_state(self, msg):
        d = json.loads(msg.data)
        minute = int(d["t"] // 60)
        if minute == self.last_min:
            return
        self.last_min = minute
        air = [u for u in d["uavs"] if u["state"] not in ("GROUND", "SWAP", "DEAD", "QUEUED")]
        conn = sum(u["connected"] for u in air)
        self.get_logger().info(f"t={minute:02d} min  airborne={len(air)}  connected={conn}")


def main():
    rclpy.init()
    node = GcsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
