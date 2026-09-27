#!/usr/bin/env bash
# PX4 SITL (Gazebo classic) replay of a ZERO-GAP baton pass on 4 iris vehicles.
# Usage: scripts/run_gazebo_demo.sh [--no-launch] [out.mp4]
# Needs ~/PX4-Autopilot (v1.14, built for gazebo-classic) and pymavlink; X11 display :1 by default.
set -euo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=${DISPLAY:-:1}
LAUNCH=1
[ "${1:-}" = "--no-launch" ] && { LAUNCH=0; shift; }
OUT=${1:-results/gazebo_demo.mp4}
if [ $LAUNCH = 1 ]; then
  gazebo_demo/launch_sitl.sh 4 > /tmp/zg_launch.log 2>&1 < /dev/null
  sleep 25                                  # EKF / GPS lock
fi
FF=$(python3 -c "import imageio_ffmpeg as f; print(f.get_ffmpeg_exe())")
WID=$(xwininfo -root -tree | grep '"Gazebo"' | head -1 | awk '{print $1}')
# capture only the Gazebo window (not the screen), crop to the 3D view
"$FF" -y -loglevel error -f x11grab -draw_mouse 0 -framerate 20 -window_id "$WID" -i "$DISPLAY" -t 55 \
  -vf "crop=1630:880:270:75,scale=1280:-2" -c:v libx264 -pix_fmt yuv420p -crf 23 "$OUT" &
REC=$!
python3 gazebo_demo/px4_replay.py
wait $REC
echo "saved $OUT"
