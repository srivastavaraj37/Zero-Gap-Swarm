#!/usr/bin/env bash
# Same as PX4's sitl_multiple_run.sh (iris, spawn at (0, 3N), MAVLink offboard on UDP 14540+N),
# but loads gazebo_demo/zg_spine.world (sky, grass, station markers, fixed camera).
set -e
N=${1:-4}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
HERE="$(cd "$(dirname "$0")" && pwd)"
build_path=$PX4_DIR/build/px4_sitl_default
export PX4_SIM_MODEL=gazebo-classic_iris
pkill -x px4 || true; pkill -x gzclient || true; pkill -x gzserver || true
for _ in $(seq 1 20); do pgrep -x gzserver >/dev/null || pgrep -x px4 >/dev/null || break; sleep 0.5; done
pkill -9 -x px4 || true; pkill -9 -x gzserver || true; pkill -9 -x gzclient || true
sleep 1
source $PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_DIR $build_path >/dev/null
gzserver "$HERE/zg_spine.world" --verbose > /tmp/zg_gzserver.log 2>&1 &
sleep 5
for n in $(seq 1 $N); do
  wd=$build_path/rootfs/$((n - 1)); mkdir -p $wd
  (cd $wd && $build_path/bin/px4 -i $n -d "$build_path/etc" > out.log 2> err.log &)
  python3 $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jinja_gen.py \
    $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja \
    $PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic \
    --mavlink_tcp_port $((4560 + n)) --mavlink_udp_port $((14560 + n)) --mavlink_id $((1 + n)) \
    --gst_udp_port $((5600 + n)) --video_uri $((5600 + n)) --mavlink_cam_udp_port $((14530 + n)) \
    --output-file /tmp/iris_$n.sdf
  gz model --spawn-file=/tmp/iris_$n.sdf --model-name=iris_$n -x 0.0 -y $((3 * n)) -z 0.83 > /dev/null
done
HOME="$HERE/gzhome" gzclient > /tmp/zg_gzclient.log 2>&1 < /dev/null &   # own gui.ini: 1920x1080
echo "started $N iris vehicles"
