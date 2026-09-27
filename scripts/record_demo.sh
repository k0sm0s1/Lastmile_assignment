#!/usr/bin/env bash
# Records the demo video: fresh bring-up, a chase camera behind the robot,
# the multi-waypoint run through /robot/next_waypoint, overlayed telemetry.
# Output: ${OUT:-$HOME/lastmile_logs/demo.mp4}
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT=${OUT:-$HOME/lastmile_logs/demo.mp4}
source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/config/fastdds_udp_only.xml"
pkill -f waypoint_test.sh; pkill -f "lib/lastmile_dashboard/record_demo"; bash "$WS/scripts/run_demo.sh" > /dev/null
ros2 run gazebo_ros spawn_entity.py -entity demo_camera \
  -file "$(ros2 pkg prefix lastmile_description)/share/lastmile_description/models/demo_camera/model.sdf" \
  -x -1.6 -y 0 -z 1.25 > /dev/null 2>&1
STOP=$(mktemp -u); RAW="${OUT%.*}.avi"; rm -f "$OUT" "$RAW"
ros2 run lastmile_dashboard record_demo --ros-args -p use_sim_time:=true -p out:="$RAW" -p duration:=600.0 -p stop_file:="$STOP" &
REC=$!
sleep 12
WPS=${WPS:-"8.0,0.0 8.8,5.0 15.5,-3.0"} bash "$WS/scripts/waypoint_test.sh"
sleep 4; touch "$STOP"; wait $REC; rm -f "$STOP"
python3 - "$RAW" "$OUT" <<'PY'
import sys, cv2
src, dst = sys.argv[1], sys.argv[2]
cap = cv2.VideoCapture(src)
fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
ok, f = cap.read(); h, w = f.shape[:2]
vw = cv2.VideoWriter(dst, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)); n = 0
while ok:
    vw.write(f); n += 1; ok, f = cap.read()
vw.release(); print(f"converted {n} frames -> {dst}")
PY
rm -f "$RAW"
echo "video: $OUT"
