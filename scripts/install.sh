#!/usr/bin/env bash
# Installs the Python deps into the user site and builds the ROS 2 workspace.
# numpy stays below 1.25 so it works with Ubuntu 22.04's scipy/matplotlib and ROS 2 Humble.
# Usage: scripts/install.sh [--gazebo]    (--gazebo also installs pymavlink for the PX4 demo)
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m pip install --user -r requirements.txt
if [ "${1:-}" = "--gazebo" ]; then
  python3 -m pip install --user -r requirements-gazebo.txt
fi
python3 -c "import numpy, networkx, scipy, matplotlib, yaml, pandas; print('python deps OK, numpy', numpy.__version__)"
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  (cd ros2_ws && colcon build --symlink-install)
  echo "ROS 2 workspace built: source ros2_ws/install/setup.bash"
else
  echo "ROS 2 Humble not found. The core simulator still works without it."
fi
