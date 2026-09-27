"""ros2 launch zg_bringup demo.launch.py seed:=1 mode:=zerogap faults:=false speedup:=10.0"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# repo root: <root>/ros2_ws/src/zg_bringup/launch/demo.launch.py (symlink-install keeps this path)
ROOT = os.environ.get("ZG_ROOT") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."))


def generate_launch_description():
    args = [DeclareLaunchArgument("seed", default_value="1"),
            DeclareLaunchArgument("mode", default_value="zerogap"),
            DeclareLaunchArgument("faults", default_value="false"),
            DeclareLaunchArgument("speedup", default_value="10.0")]
    sim = Node(package="zg_bringup", executable="swarm_sim", name="zg_swarm_sim", output="screen",
               parameters=[{"zg_root": ROOT,
                            "seed": LaunchConfiguration("seed"),
                            "mode": LaunchConfiguration("mode"),
                            "faults": LaunchConfiguration("faults"),
                            "speedup": LaunchConfiguration("speedup")}])
    mon = Node(package="zg_bringup", executable="gcs_monitor", name="zg_gcs_monitor", output="screen")
    return LaunchDescription(args + [sim, mon])
