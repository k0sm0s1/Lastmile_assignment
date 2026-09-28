#!/usr/bin/env bash
# One-command bring-up: Gazebo world + TurtleBot3 at (0,0,0) -> map_server/AMCL/Nav2
# -> robot_state_manager (Task 3) -> web dashboard (http://localhost:8080).
# Usage: bash scripts/run_demo.sh          (headless Gazebo)
#        GUI=1 bash scripts/run_demo.sh    (with the Gazebo client window)
#        OBSTACLES=1 bash scripts/run_demo.sh  (3 unmapped boxes + live lidar obstacle avoidance)
#        DELIVERY=1 bash scripts/run_demo.sh   (two-stop delivery page on http://localhost:8081; combinable)
#        bash scripts/run_demo.sh stop     (kill everything this script starts)

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="$HOME/lastmile_logs"; mkdir -p "$LOG"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
# Fast DDS over UDP only - the shared-memory transport misbehaved under WSL2.
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS/config/fastdds_udp_only.xml"
export TURTLEBOT3_MODEL=waffle

stop_all() {
  pkill -INT -f "ros2 launch (lastmile_description|lastmile_navigation|robot_state_manager|lastmile_dashboard)" 2>/dev/null
  sleep 4
  pkill -9 -f "gzserver|gzclient|nav2_amcl/amcl|nav2_map_server|bt_navigator|controller_server|planner_server|behavior_server|lifecycle_manager|robot_state_manager_node|robot_state_publisher|dashboard_node|obstacle_mapper|delivery_node" 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1
}
if [ "${1:-}" = "stop" ]; then stop_all; echo "stopped"; exit 0; fi

stop_all; ros2 daemon start >/dev/null 2>&1
GUI_ARG=false; [ "${GUI:-0}" = "1" ] && GUI_ARG=true
OBS_ARG=false; [ "${OBSTACLES:-0}" = "1" ] && OBS_ARG=true

[ "$OBS_ARG" = true ] && echo "      obstacle mode: 3 unmapped boxes + obstacle_mapper"
echo "[1/4] Gazebo + scanned world + TurtleBot3 at (0,0,0)   (log: $LOG/sim.log)"
setsid nohup ros2 launch lastmile_description spawn_world.launch.py gui:=$GUI_ARG obstacles:=$OBS_ARG > "$LOG/sim.log" 2>&1 &
# /odom only appears once the robot has actually spawned (with the GUI on, the
# scanned mesh can take well over 30 s to load)
if ! python3 "$WS/scripts/ros_wait.py" topic /odom 150; then
  echo "ERROR: the robot did not spawn (no /odom after 150 s). See $LOG/sim.log"; exit 1
fi
echo "      robot spawned"
sleep 5

echo "[2/4] map_server + AMCL + Nav2                          (log: $LOG/nav2.log)"
nav2_up() { python3 "$WS/scripts/ros_wait.py" active "${1:-3}" map_server nav_map_server amcl planner_server controller_server bt_navigator; }
for attempt in 1 2; do
  setsid nohup ros2 launch lastmile_navigation bringup_nav2.launch.py obstacles:=$OBS_ARG > "$LOG/nav2.log" 2>&1 &
  nav2_up 90 && break
  # under heavy load a lifecycle service reply can get lost and bring-up stalls; restart it once
  echo "      Nav2 bring-up stalled, restarting it (attempt $attempt)"
  pkill -INT -f "bringup_nav2.launch.py"; sleep 4
  pkill -9 -f "nav2_amcl/amcl|nav2_map_server|bt_navigator|controller_server|planner_server|behavior_server|lifecycle_manager|obstacle_mapper"; sleep 2
done
nav2_up || { echo "ERROR: Nav2 did not come up. See $LOG/nav2.log"; exit 1; }

echo "[3/4] robot_state_manager (Task 3)                      (log: $LOG/rsm.log)"
setsid nohup ros2 launch robot_state_manager robot_state_manager.launch.py > "$LOG/rsm.log" 2>&1 &
echo "[4/4] dashboard                                          (log: $LOG/dash.log)"
setsid nohup ros2 launch lastmile_dashboard dashboard.launch.py > "$LOG/dash.log" 2>&1 &
if [ "${DELIVERY:-0}" = "1" ]; then
  echo "[+]   delivery page                                  (log: $LOG/delivery.log)"
  setsid nohup ros2 launch lastmile_dashboard delivery.launch.py > "$LOG/delivery.log" 2>&1 &
fi
sleep 6

python3 "$WS/scripts/ros_wait.py" states map_server nav_map_server amcl controller_server planner_server behavior_server bt_navigator
echo
echo "Ready. Dashboard: http://localhost:8080"
[ "${DELIVERY:-0}" = "1" ] && echo "       Delivery:  http://localhost:8081"
echo "Send a waypoint:  ros2 topic pub --once /robot/next_waypoint geometry_msgs/msg/PoseStamped \\"
echo "                    '{header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}'"
echo "Stop everything:  bash scripts/run_demo.sh stop"
