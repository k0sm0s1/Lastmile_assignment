#!/usr/bin/env python3
"""
Obstacle-avoidance check (run while `OBSTACLES=1 bash scripts/run_demo.sh` is up).

Sends one waypoint through /robot/next_waypoint, the same way as Task 3, then
logs Gazebo's ground-truth robot pose until the goal finishes and reports:
  - result + final error
  - closest approach to each obstacle box (surface clearance, robot radius removed)
  - how far the path swerved sideways at each box
  - what obstacle_mapper detected
Usage: python3 scripts/obstacle_test.py [x y]   (default goal 12.5 0.0)
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

BOXES = {"obstacle_1": (4.5, 0.35), "obstacle_2": (6.5, -0.35), "obstacle_3": (10.5, 0.35)}
HALF = 0.20       # box half-size
ROBOT_R = 0.22    # TurtleBot3 Waffle footprint radius used by Nav2


def box_clearance(px, py, bx, by):
    dx = max(abs(px - bx) - HALF, 0.0)
    dy = max(abs(py - by) - HALF, 0.0)
    return math.hypot(dx, dy) - ROBOT_R


class T(Node):
    def __init__(self):
        super().__init__("obstacle_test")
        self.traj, self.status, self.obst = [], {}, {}
        self.create_subscription(ModelStates, "/gazebo/model_states", self.on_ms, 10)
        self.create_subscription(String, "/robot/nav_status", lambda m: setattr(self, "status", json.loads(m.data)), 10)
        self.create_subscription(String, "/obstacle_mapper/obstacles", lambda m: setattr(self, "obst", json.loads(m.data)), 10)
        self.pub = self.create_publisher(PoseStamped, "/robot/next_waypoint", 10)

    def on_ms(self, m):
        if "turtlebot3_waffle" in m.name:
            p = m.pose[m.name.index("turtlebot3_waffle")].position
            if not self.traj or math.hypot(p.x - self.traj[-1][0], p.y - self.traj[-1][1]) > 0.02:
                self.traj.append((p.x, p.y))


def main():
    gx, gy = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) > 2 else (12.5, 0.0)
    rclpy.init()
    n = T()
    t0 = time.time()
    while (n.pub.get_subscription_count() == 0 or not n.status) and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    prev = (n.status.get("goal") or {}).get("id", 0)
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x, msg.pose.position.y, msg.pose.orientation.w = gx, gy, 1.0
    n.pub.publish(msg)
    print(f"goal ({gx}, {gy}) sent", flush=True)
    t0 = time.time()
    while time.time() - t0 < 400:
        rclpy.spin_once(n, timeout_sec=0.1)
        s = n.status
        if (s.get("goal") or {}).get("id", 0) > prev and s.get("state") in ("succeeded", "aborted", "canceled", "rejected"):
            break
    s = n.status
    print(f"result={s.get('state')} wall_s={time.time()-t0:.0f} final_error={s.get('goal_error_m')} verified={s.get('verified')} recoveries={s.get('recoveries')}")
    print(f"trajectory points: {len(n.traj)}")
    for name, (bx, by) in BOXES.items():
        near = [p for p in n.traj if abs(p[0] - bx) < 0.6]
        if not near:
            print(f"  {name} at ({bx},{by}): robot never reached this x")
            continue
        c = min(box_clearance(px, py, bx, by) for px, py in n.traj)
        y_at = sum(p[1] for p in near) / len(near)
        side = "below (-y)" if y_at < by else "above (+y)"
        print(f"  {name} at ({bx:+.2f},{by:+.2f}): min surface clearance {c:+.2f} m, passed {side} at mean y={y_at:+.2f}")
    print(f"obstacle_mapper clusters: {n.obst.get('clusters')}")
    with open("/tmp/obstacle_traj.json", "w") as f:
        json.dump(n.traj, f)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
