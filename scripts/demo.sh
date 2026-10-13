#!/usr/bin/env bash
# Bài 04 from start to finish in one command (for the demo video):
#   Phần 1  TurtleBot 4 in Gazebo, /scan /odom /cmd_vel check          sim.launch.py slam:=true
#   Phần 2  SLAM Toolbox mapping, map saved to <dir>/map.yaml + map.pgm
#   Phần 3  SLAM stopped, sim restarted on the saved map with AMCL      localization:=true nav2:=true
#   Phần 4  Nav2: three goals in different parts of the maze; pose vs Gazebo again
#
#   ros2 run turtlebot4_bai04 demo.sh [--teleop] [--rviz-goals] [-o DIR]    (DIR: ~/tb4_demo)
#
# By default explore_route.py drives the robot for mapping and send_goals.py sends the goals.
# --teleop: drive with teleop_twist_keyboard instead, Ctrl-C in it when the map is complete.
# --rviz-goals: wait for three goals set with "Nav2 Goal" in RViz instead.
# Ctrl-C anywhere else stops everything.
set -u
HERE=$(dirname "$0")
OUT=$HOME/tb4_demo
TELEOP=false
RVIZ_GOALS=false
while [ $# -gt 0 ]; do
  case $1 in
    --teleop) TELEOP=true ;;
    --rviz-goals) RVIZ_GOALS=true ;;
    -o) OUT=$2; shift ;;
    *) sed -n '2,14p' "$0"; exit 1 ;;
  esac
  shift
done
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1} IGN_IP=${IGN_IP:-127.0.0.1}
mkdir -p "$OUT"

say() { printf '\n\033[1;36m=== %s ===\033[0m\n' "$*"; }
run() { printf '\033[1;33m$ %s\033[0m\n' "$*"; "$@"; }
fail() { printf '\033[1;31m%s\033[0m\n' "$*"; stop; exit 1; }

launch() {  # log name, sim.launch.py arguments
  local log=$OUT/$1; shift
  printf '\033[1;33m$ ros2 launch turtlebot4_bai04 sim.launch.py %s\033[0m   (log: %s)\n' "$*" "$log"
  ros2 launch turtlebot4_bai04 sim.launch.py "$@" > "$log" 2>&1 < /dev/null &
  disown  # stopped by stop_sim.sh; no "Killed" job notice
}

stop() { "$HERE/stop_sim.sh" > /dev/null; sleep 2; }

wait_topic() {  # topic, timeout [s]
  local t=0
  until ros2 topic info --no-daemon "$1" 2>/dev/null | grep -q 'Publisher count: [1-9]'; do
    sleep 2; t=$((t + 2)); [ $t -lt "$2" ] || fail "no publisher on $1 after $2 s (see $OUT/*.log)"
  done
}

wait_active() {  # lifecycle node, timeout [s]
  local t=0
  until ros2 lifecycle get --no-daemon "$1" 2>/dev/null | grep -q '^active'; do
    sleep 2; t=$((t + 2)); [ $t -lt "$2" ] || fail "$1 not active after $2 s (see $OUT/*.log)"
  done
}

trap 'echo; stop; exit 130' INT TERM
"$HERE/stop_sim.sh" > /dev/null   # leftovers of an earlier run
T0=$(date +%s)

say "Phần 1 – TurtleBot 4 trong Gazebo: robot, /scan, /odom, /cmd_vel, RViz2"
launch phan1_2.log slam:=true
wait_topic /scan 180
sleep 5
run ros2 run turtlebot4_bai04 check_topics.py

say "Phần 2 – Xây dựng bản đồ bằng SLAM Toolbox"
wait_topic /map 120
if $TELEOP; then
  echo "Lái robot đi hết mê cung (i tiến, j/l quay, k dừng); xong bấm Ctrl-C trong teleop để lưu bản đồ."
  run ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
  run ros2 run turtlebot4_bai04 explore_route.py
fi
rm -f "$OUT/map.yaml" "$OUT/map.pgm"
run ros2 run nav2_map_server map_saver_cli -f "$OUT/map" --ros-args -p use_sim_time:=true > /dev/null 2>&1
[ -f "$OUT/map.yaml" ] && [ -f "$OUT/map.pgm" ] || fail "map not saved"
ls -l "$OUT/map.yaml" "$OUT/map.pgm"
echo "Bản đồ hoàn chỉnh trên RViz2."
sleep 10
echo "Dừng SLAM."
stop

say "Phần 3 – Localization: khởi động lại robot, load bản đồ vừa tạo, AMCL"
launch phan3_4.log localization:=true nav2:=true map:="$OUT/map.yaml"
wait_active /amcl 180
sleep 5
echo "Vị trí robot trên bản đồ (AMCL) so với vị trí thực trong Gazebo:"
run ros2 run turtlebot4_bai04 compare_pose.py 5

say "Phần 4 – Navigation với Nav2"
wait_active /bt_navigator 180
if $RVIZ_GOALS; then
  echo "Đặt 3 goal ở 3 khu vực khác nhau bằng nút Nav2 Goal trên RViz2."
  run ros2 run turtlebot4_bai04 watch_goals.py 3
else
  run ros2 run turtlebot4_bai04 send_goals.py
fi
echo "Localization sau khi di chuyển:"
run ros2 run turtlebot4_bai04 compare_pose.py 5

say "Xong sau $(( ($(date +%s) - T0) / 60 )) phút – bản đồ: $OUT/map.yaml, log: $OUT"
read -r -p "Bấm Enter để tắt mô phỏng. " _
stop
