# Sends waypoints through /robot/next_waypoint (Task 3 node). For each one it waits
# until the node reports a NEW goal id reaching a terminal state, then records the
# node's localized pose, Nav2's own log line, and Gazebo ground truth.
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/lastmile_ws/config/fastdds_udp_only.xml
OUT=${OUT:-$HOME/lastmile_logs/waypoint_test.log}; : > "$OUT"
WPS=${WPS:-"8.0,0.0 8.8,5.0 15.5,-3.0 0.0,0.0"}
st() { timeout 4 ros2 topic echo /robot/nav_status --once --field data 2>/dev/null | head -1; }
gid() { echo "$1" | grep -oP '"id": \K[0-9]+' | head -1; }
for wp in $WPS; do
  x=${wp%,*}; y=${wp#*,}; t0=$(date +%s)
  prev=$(gid "$(st)"); prev=${prev:-0}
  ros2 topic pub --once -w 1 /robot/next_waypoint geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: $x, y: $y}, orientation: {w: 1.0}}}" >/dev/null 2>&1
  while true; do
    sleep 3; s=$(st); id=$(gid "$s"); state=$(echo "$s" | grep -oP '"state": "\K[a-z_]+')
    if [ -n "$id" ] && [ "$id" -gt "$prev" ]; then
      case "$state" in succeeded|aborted|canceled|rejected|server_unavailable) break;; esac
    fi
    [ $(( $(date +%s) - t0 )) -gt 300 ] && { state=timeout; break; }
  done
  truth=$(timeout 6 gz model -m turtlebot3_waffle -p 2>/dev/null | head -1)
  echo "goal=($x,$y) id=$id result=$state wall_s=$(( $(date +%s) - t0 )) node_pose=$(echo "$s" | grep -oP '"pose": \{[^}]*\}' | sed 's/"pose": //') recoveries=$(echo "$s" | grep -oP '"recoveries": \K[0-9]+') nav_time_s=$(echo "$s" | grep -oP '"nav_time_s": \K[0-9.]+') gz_truth=[$truth]" | tee -a "$OUT"
done
echo DONE >> "$OUT"
