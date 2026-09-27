#!/usr/bin/env bash
# Install Python deps (user site). numpy is pinned <1.25 to stay ABI-compatible
# with Ubuntu 22.04's system scipy/matplotlib and ROS 2 Humble.
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m pip install --user -r requirements.txt
python3 -c "import numpy, networkx, scipy, matplotlib, yaml, pandas; print('python deps OK, numpy', numpy.__version__)"
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  (cd ros2_ws && colcon build --symlink-install)
  echo "ROS 2 workspace built: source ros2_ws/install/setup.bash"
else
  echo "ROS 2 Humble not found - core simulator still works without it."
fi
