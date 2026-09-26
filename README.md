# Real-World Indoor Navigation & State Monitoring

A real point-cloud scan of a building corridor → a collidable Gazebo world →
a 2D map with AMCL localization → Nav2 point-to-point navigation, driven and
monitored by a custom ROS 2 node (`robot_state_manager`), plus a live browser
dashboard.

**Stack:** ROS 2 Humble · Gazebo Classic 11 · Nav2 · TurtleBot3 Waffle ·
Ubuntu 22.04 on WSL2 (Windows 11). Python for every custom piece.

![Gazebo chase camera: TurtleBot3 in the reconstructed corridor](docs/demo_frame.jpg)

**Demo video:** [docs/demo.mp4](docs/demo.mp4) — fresh start, three waypoints sent through `/robot/next_waypoint`, rendered by a Gazebo chase camera with the Task 3 telemetry overlaid (recorded headless by `scripts/record_demo.sh`).

---

## Results (measured, not assumed)

Everything below was run end-to-end in this setup; numbers are copied from the
logs (`scripts/waypoint_test.sh` records the Task 3 node's localized pose and
Gazebo's ground-truth model pose at every arrival).

| Check | Result |
|---|---|
| Robot spawns at the assignment's origin (0, 0, 0) inside the scanned mesh | ✅ ground truth `(0.002, 0.000)` after settling |
| TF chain `map → odom → base_footprint → base_scan` | ✅ all present; AMCL republishes `map → odom` at ~5 Hz even when stationary |
| AMCL accuracy vs Gazebo ground truth | ✅ 5–16 cm at goal arrivals (table below) |
| `/robot/current_xyz` rate | ✅ **10.0–10.1 Hz** (`ros2 topic hz`) |
| `/robot/next_waypoint` → `NavigateToPose` → robot drives and arrives | ✅ multi-waypoint run below |
| `/cmd_vel` threshold monitor | ✅ logs every excursion (e.g. `linear speed 0.220 m/s exceeds threshold 0.20 m/s`, `angular speed 1.000 rad/s exceeds threshold 0.80 rad/s (turning fast)`) |
| Dashboard: send goals, live pose/map/plan/lidar, cmd_vel, nav status, health, history | ✅ `http://localhost:8080` |

Multi-waypoint run through `/robot/next_waypoint` (cold start, TurtleBot3 at origin):

| # | Goal (map) | Result | Wall time | Localized pose at arrival | Gazebo ground truth | AMCL error |
|---|---|---|---|---|---|---|
| 1 | (8.0, 0.0) E corridor | ✅ succeeded, 0 recoveries | 72 s | (7.77, −0.04) | (7.82, −0.02) | 0.05 m |
| 2 | (8.8, 5.0) north arm | ✅ succeeded, 0 recoveries | 67 s | (8.84, 4.74) | (8.83, 4.85) | 0.12 m |
| 3 | (15.5, −3.0) south room | ✅ succeeded, 1 recovery | 145 s | (15.40, −2.84) | (15.56, −2.85) | 0.16 m |

(Goal tolerance is 0.25 m; "wall time" includes Gazebo running at ~0.8× real time. This is the run in the demo video.)

---

## What the provided data actually is

`map_ros_cloud.ply` is a **raw point cloud, not a mesh**: binary PLY,
2,338,923 points with xyz + rgb + normals and `element face 0`. It is
already in metres (≈ 28 m × 16.5 m × 4.85 m), close to gravity-aligned
(RANSAC floor normal within ~1.3° of +z, ~2 cm offset), and it is a walked
SLAM scan of a **corridor intersection**: a long E–W corridor with a north
arm, a south room and a west wing, open at the ends.

Two properties of the scan drove most of the engineering:

1. **The floor is not one plane.** After SLAM the floor height drifts by
   about ±10 cm across the site. Anything that measures height above a
   single global floor plane misclassifies floor as obstacle in places.
2. **The scan origin is full of ghost points.** (0, 0) is where the scanner
   started; people walking by and sensor noise left a haze of sparse points
   there at all heights. A "any point in the height band ⇒ occupied" rule turns
   that haze into a solid block.

(1) and (2) together are why an earlier version of this pipeline concluded
that "(0, 0, 0) is inside a wall" and moved the spawn. **It isn't**: (0, 0)
is open corridor with 0.86 m clearance to the nearest wall (checked on the
occupancy grid *and* by ray-casting the Gazebo mesh). The robot now spawns
exactly where the assignment asks.

---

## Task 1 — Environment integration (Gazebo)

`scripts/preprocess_cloud.py` → `scripts/cloud_to_mesh.py` →
`scripts/make_collision_mesh.py` → `src/lastmile_description/models/scanned_environment/`

* **Preprocess:** voxel downsample (3 cm), statistical outlier removal,
  RANSAC floor fit; the rotation + offset is saved to
  `data/floor_transform.json` so every later stage uses the same frame.
* **Mesh:** Poisson reconstruction with density-based trimming (Poisson
  otherwise invents surfaces across the scan's holes), decimated to 150 k
  triangles → `environment.obj`. **Scale is 1:1** — the cloud is already in
  metres and the SDF uses `<scale>1 1 1</scale>`.
* **Collision:** the model is `<static>` with
  * visual = the full reconstructed mesh,
  * collision = `environment_collision.obj` (walls/obstacles only: near-
    horizontal triangles below 0.25 m and everything below 0.12 m removed)
    **+ a flat ground plane at z = 0**.

  Why split: with the bumpy Poisson floor as the drive surface the robot
  first fell through gaps into free fall (`/odom` z in the millions), and after
  that was patched it came to rest on floor bumps with its wheels unloaded and
  would not move under `/cmd_vel`. A flat analytic ground gives stable wheel
  contact; walls still collide and are still what the lidar sees.
* **Spawn:** `spawn_world.launch.py` spawns the TurtleBot3 Waffle at
  **(0, 0)**, dropped from 10 cm so it settles on the floor, and starts
  `robot_state_publisher` (see Task 2 for why that line matters).

## Task 2 — Map generation & localization

**Pre-generated 2D map** (`.pgm` + `.yaml`) from the cloud —
`scripts/cloud_to_occupancy_grid.py` (numpy/scipy only, reads the raw PLY):

1. apply the floor transform;
2. **local floor height** per 0.25 m tile (10th percentile of low points,
   holes filled from neighbours, median-smoothed) — heights are measured
   above the *local* floor;
3. a 5 cm cell is **occupied only with real vertical structure**: ≥ 40 points
   spanning ≥ 8 distinct 5 cm height bins within 0.12–1.60 m (or ≥ 150
   points). Walls have hundreds of points over the full height; ghost haze
   doesn't;
4. **free = cells where floor was actually observed**, minus obstacles,
   connected to the spawn. Never-observed space stays *unknown*, and the
   planner runs with `allow_unknown: false`, so it can't route through
   unscanned space (the v1 map flood-filled "free" out to the bounding box).

Result: 529 × 367 cells @ 5 cm, 32,639 free / 4,872 occupied / rest unknown.
The script regenerates the committed map byte-for-byte.

**Localization:** AMCL (likelihood field) on that map, seeded at (0, 0, 0)
via `set_initial_pose`. `map → odom` from AMCL + `odom → base_footprint`
from the Gazebo diff-drive plugin + `base_footprint → base_link → base_scan`
from `robot_state_publisher`.

### The debugging that got localization working (in order)

| Symptom | Root cause | Fix |
|---|---|---|
| AMCL: *Message Filter dropping message*, `map` frame never appears, `global_costmap` stuck | **No `robot_state_publisher`**: the launch spawned the robot but never published its URDF transforms, so `base_scan` didn't exist and AMCL couldn't place a single scan | `spawn_world.launch.py` now starts TurtleBot3's `robot_state_publisher` with sim time |
| Planner failed from the spawn (start in lethal cost) | v1 map: ghost haze near (0, 0) + floor drift = a fake wall at the origin | map v2 (above) |
| Scan has 0.12–0.16 m returns around the robot; costmap marks the robot's own footprint | lidar hitting the Waffle's own chassis/camera mount (mesh ray-cast shows nothing within 0.6 m) | `obstacle_min_range: 0.3` (costmaps) and `laser_min_range: 0.3` (AMCL) |
| Robot doesn't move although `/cmd_vel` = 0.208 m/s | resting on the noisy Poisson floor | collision mesh without floor + ground plane |
| Global plan always starts at (0, 0) while the robot is at x = 4 → controller "Resulting plan has 0 poses" | the **global** costmap froze: its footprint and TF buffer stopped updating (t = 73 s) | global costmap = static + inflation |
| After a cold start every goal instantly "succeeds" with the robot still at (0, 0) (`tf_help: Transform data too old ... Transform time: 38.6s` forever) | same freeze in the **local** costmap: the laser obstacle layer's tf2 MessageFilter deadlocks the costmap's TF buffer during startup (timing-dependent; frequent with Gazebo below real time under WSL2). With no usable map→odom the Humble controller compares the robot against an all-zero goal and reports "Reached the goal!" | local costmap = static layer + inflation too (walls come from the scan's map). Verified: 3 cold starts with an immediate goal, 0 TF errors, robot drives. Trade-off: obstacles that are not in the scan are not avoided — acceptable for a static scanned world. The Task 3 node also flags any SUCCEEDED that isn't within 0.5 m of the goal. |
| Stop-and-go on the way down the corridor ("recoveries" with no obstacle) | bt_navigator's default 20 ms `default_server_timeout`: under load the planner/controller take longer to *acknowledge* a request, the BT counts it as a failure and runs a recovery | `default_server_timeout: 500`, `controller_frequency: 10` (20 Hz was routinely missed) — recoveries on the E corridor leg went from 5 to 0 |
| Intermittent DDS delivery issues under WSL2 | Fast DDS shared-memory transport | `config/fastdds_udp_only.xml` (UDP only), exported by `scripts/run_demo.sh` |

## Task 3 — `robot_state_manager` (custom node, rclpy)

`src/robot_state_manager/robot_state_manager/robot_state_manager_node.py`

| Assignment item | Implementation |
|---|---|
| Position publisher | Publishes `[x, y, z]` as `std_msgs/Float32MultiArray` on **`/robot/current_xyz` at 10 Hz**. Source = localized pose (TF `map → base_footprint`, i.e. AMCL ∘ odometry); falls back to `/odom` until TF is available (`pose_source:=odom` forces odometry). The timer runs on the **wall clock**: Gazebo runs at RTF ≈ 0.8 with this mesh, and a sim-time timer gave only 7.6 Hz real time. |
| Goal receiver | Subscribes **`/robot/next_waypoint`** (`geometry_msgs/PoseStamped`; empty frame ⇒ `map`, zero quaternion ⇒ identity). |
| Nav2 interfacing | `rclpy.action.ActionClient` for **`nav2_msgs/action/NavigateToPose`** (the assignment's "MapsToPose" doesn't exist in Nav2; this is the pose-goal action). A new waypoint preempts the old one; feedback (distance remaining, recoveries, nav time) and the result are tracked. **Post-check:** on SUCCEEDED the node compares its localized pose with the goal and flags the result *unverified* if it's > 0.5 m away (guards against a Humble controller quirk that can report "Reached the goal!" when the goal fails to transform). |
| Command monitor | Subscribes `/cmd_vel`; echoes it (1 Hz, `echo_cmd_vel`) and logs a WARN **whenever** `|v| > linear_velocity_threshold` or `|ω| > angular_velocity_threshold` (defaults 0.20 m/s, 0.80 rad/s — just under the TurtleBot3's 0.22 / 1.0 limits so the demo actually shows them; set any value at launch). |
| Extra | `/robot/nav_status` (JSON: state machine, goal, distance remaining, recoveries, final error, latest cmd_vel + threshold flags) and `/robot/cancel` (`std_msgs/Empty`). Both used by the dashboard. |

```bash
ros2 launch robot_state_manager robot_state_manager.launch.py \
    linear_velocity_threshold:=0.15 angular_velocity_threshold:=0.5
```

## Dashboard (extra)

`src/lastmile_dashboard` — one rclpy node + stdlib HTTP server (no
rosbridge, nothing to pip-install). Open **http://localhost:8080** (WSL2
forwards it to Windows).

* map (from `/map`) with the robot, Nav2 global plan, live lidar points (projected
  through TF) and the goal;
* **click to send a goal, click-drag to set heading**, numeric entry, presets,
  cancel, and a "set pose" mode that re-seeds AMCL via `/initialpose`;
* live `/robot/current_xyz` (+ measured rate), yaw, nav state, distance
  remaining, recoveries, final error;
* `/cmd_vel` bars against the thresholds + the node's threshold alerts;
* health: AMCL localized (age of `map → odom`, covariance), lifecycle state
  of every Nav2 server, Task 3 node publishing;
* waypoint history with results.

## Running it

```bash
# WSL2 Ubuntu 22.04 with ROS 2 Humble, Gazebo Classic, Nav2, TurtleBot3 (see "Setup")
cd ~/lastmile_ws && colcon build --symlink-install
bash scripts/run_demo.sh            # headless;  GUI=1 bash scripts/run_demo.sh  for Gazebo + RViz-free GUI
# then open http://localhost:8080
```

`run_demo.sh` starts, in order: Gazebo world + robot (+ `robot_state_publisher`),
map_server/AMCL/Nav2, `robot_state_manager`, dashboard — all with the UDP-only
DDS profile — and waits for Nav2 to report active. Logs go to `~/lastmile_logs/`.

The three assignment pieces can also be started by hand (each terminal:
`source install/setup.bash` and `export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/config/fastdds_udp_only.xml`):

```bash
ros2 launch lastmile_description spawn_world.launch.py gui:=true   # Task 1
ros2 launch lastmile_navigation bringup_nav2.launch.py             # Task 2
ros2 launch robot_state_manager robot_state_manager.launch.py      # Task 3
ros2 launch lastmile_dashboard dashboard.launch.py                 # dashboard
```

Try it from the CLI:

```bash
ros2 topic hz /robot/current_xyz
ros2 topic pub --once /robot/next_waypoint geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0}, orientation: {w: 1.0}}}'
bash scripts/waypoint_test.sh      # the multi-waypoint check used for the results table
bash scripts/record_demo.sh        # fresh run + chase-camera video -> ~/lastmile_logs/demo.mp4
```

Good goals in this map: `(8, 0)` east corridor · `(8.8, 5)` north arm ·
`(15.5, -3)` south room · `(-2.8, 5)` west wing · `(0, 0)` home.

## Regenerating the assets from the scan

```bash
pip install -r requirements.txt      # numpy, scipy, pillow, open3d (open3d only for the first two)
cp /path/to/map_ros_cloud.ply data/
python3 scripts/preprocess_cloud.py          # -> data/cleaned_cloud.ply, data/floor_transform.json
python3 scripts/cloud_to_mesh.py             # -> environment.obj
python3 scripts/make_collision_mesh.py       # -> environment_collision.obj
python3 scripts/cloud_to_occupancy_grid.py data/map_ros_cloud.ply   # -> lastmile_map.pgm/.yaml
```

## Repo layout

```
config/fastdds_udp_only.xml          DDS profile used under WSL2
docs/                                demo video + frame
scripts/                             offline pipeline + run_demo.sh, waypoint_test.sh, record_demo.sh
src/lastmile_description/            Task 1: world, scanned model, spawn launch
src/lastmile_navigation/             Task 2: map, Nav2/AMCL params, bringup launch
src/robot_state_manager/             Task 3: the custom node
src/lastmile_dashboard/              web dashboard
src/lastmile_bot_description/        optional: a custom delivery-bot model (not used in the results)
src/lastmile_bot_animator/           optional: its cosmetic idle animation
```

## Known limitations

* Both costmaps take obstacles from the static map (see the debugging table): the robot avoids everything in the scan but would not see an object added to the world later.
* The world is a static scan: doors, people, furniture are whatever the scan
  captured. The ground plane replaces the scanned floor for physics only.
* Gazebo runs below real time (RTF ≈ 0.8) with the 127 k-triangle collision
  mesh on this laptop; navigation times in sim seconds are shorter than wall time.
* The long, feature-poor E–W corridor is ambiguous along its length. With a
  correct initial pose AMCL tracks within ~20 cm, but if it is re-seeded at the
  wrong place (e.g. Nav2 restarted while the robot is elsewhere) it can lock
  on a few metres off — re-seed with the dashboard's "set pose" mode.
* `lastmile_bot` (the custom robot) is kept as an optional extra and was not part of
  the verified runs; TurtleBot3 is the validated robot.
