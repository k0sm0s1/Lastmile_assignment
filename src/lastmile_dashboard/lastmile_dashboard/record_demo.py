#!/usr/bin/env python3
"""
Demo recorder: renders a real Gazebo chase-camera view of the robot and writes
an MP4 with a live overlay (Task 3 telemetry, Nav2 state, cmd_vel vs
thresholds, mini-map with plan). Headless - no screen capture needed.

Needs: world plugin libgazebo_ros_state.so (namespace /gazebo) and the
demo_camera model spawned (launch/record_demo.launch.py does both parts it can).

  ros2 run lastmile_dashboard record_demo --ros-args -p out:=/path/demo.mp4 -p duration:=300
"""
import json
import os
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


def quat_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Recorder(Node):
    def __init__(self):
        super().__init__("demo_recorder")
        self.declare_parameter("out", "demo.mp4")
        self.declare_parameter("duration", 300.0)
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("stop_file", "")  # recording ends when this file appears
        self.declare_parameter("back", 1.6)     # m behind the robot
        self.declare_parameter("height", 1.25)  # m above floor
        self.declare_parameter("pitch", 0.42)   # rad down
        self.out = self.get_parameter("out").value
        self.stop_file = self.get_parameter("stop_file").value
        self.duration = float(self.get_parameter("duration").value)
        self.fps = float(self.get_parameter("fps").value)
        self.back, self.height, self.pitch = (float(self.get_parameter(n).value) for n in ("back", "height", "pitch"))
        self.lock = threading.Lock()
        self.frame = None
        self.odom = None
        self.cam_yaw = None
        self.xyz = None
        self.status = {}
        self.plan = []
        self.map_img = None
        self.map_info = None
        self.create_subscription(Image, "/demo_cam/chase/image_raw", self.on_img, 5)
        self.create_subscription(Odometry, "/odom", self.on_odom, QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Float32MultiArray, "/robot/current_xyz", lambda m: setattr(self, "xyz", list(m.data)), 10)
        self.create_subscription(String, "/robot/nav_status", self.on_status, 10)
        self.create_subscription(Path, "/plan", lambda m: setattr(self, "plan", [(p.pose.position.x, p.pose.position.y) for p in m.poses]), 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.create_timer(0.05, self.move_camera)

    def on_status(self, m):
        try:
            self.status = json.loads(m.data)
        except ValueError:
            pass

    def on_map(self, m):
        w, h = m.info.width, m.info.height
        a = np.array(m.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w, 3), (40, 36, 32), np.uint8)
        img[a == 0] = (225, 225, 225)
        img[a > 50] = (20, 20, 20)
        self.map_img = np.flipud(img).copy()
        self.map_info = (m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, w, h)

    def on_img(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.frame = img.copy()

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)

    def move_camera(self):
        if self.odom is None or not self.cli.service_is_ready():
            return
        x, y, yaw = self.odom
        if self.cam_yaw is None:
            self.cam_yaw = yaw
        d = math.atan2(math.sin(yaw - self.cam_yaw), math.cos(yaw - self.cam_yaw))
        self.cam_yaw += 0.12 * d  # smooth the heading so spins don't whip the view
        req = SetEntityState.Request()
        req.state.name = "demo_camera"
        req.state.reference_frame = "world"
        req.state.pose.position.x = x - self.back * math.cos(self.cam_yaw)
        req.state.pose.position.y = y - self.back * math.sin(self.cam_yaw)
        req.state.pose.position.z = self.height
        qx, qy, qz, qw = quat_from_rpy(0.0, self.pitch, self.cam_yaw)
        req.state.pose.orientation.x, req.state.pose.orientation.y = qx, qy
        req.state.pose.orientation.z, req.state.pose.orientation.w = qz, qw
        self.cli.call_async(req)

    # ---------------- overlay ----------------
    def compose(self, frame, t):
        H, W = frame.shape[:2]
        out = frame.copy()
        s = self.status or {}
        nav = s.get("state", "idle")
        goal = s.get("goal") or {}
        cmd = s.get("cmd_vel") or {}
        th = s.get("thresholds") or {"linear": 0.2, "angular": 0.8}
        # top bar
        cv2.rectangle(out, (0, 0), (W, 44), (24, 20, 17), -1)
        cv2.putText(out, "Lastmile - scanned corridor | TurtleBot3 + AMCL + Nav2 + robot_state_manager", (12, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (235, 235, 235), 1, cv2.LINE_AA)
        col = {"navigating": (255, 170, 60), "succeeded": (120, 210, 60), "aborted": (80, 80, 255)}.get(nav, (200, 200, 200))
        cv2.putText(out, nav.upper(), (W - 190, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
        # telemetry panel
        panel = [
            f"/robot/current_xyz  {self.xyz[0]:6.2f} {self.xyz[1]:6.2f} {self.xyz[2]:5.2f}" if self.xyz else "/robot/current_xyz  --",
            f"goal (/robot/next_waypoint)  ({goal.get('x', 0):.2f}, {goal.get('y', 0):.2f})" if goal else "goal  --",
            f"distance remaining  {s.get('distance_remaining'):.2f} m" if s.get("distance_remaining") is not None else "distance remaining  --",
            f"cmd_vel  v={cmd.get('v', 0):+.3f} m/s  w={cmd.get('w', 0):+.3f} rad/s",
        ]
        if s.get("goal_error_m") is not None and nav in ("succeeded", "aborted"):
            panel.append(f"final error {s['goal_error_m']:.2f} m  {'VERIFIED' if s.get('verified') else 'UNVERIFIED'}")
        y0 = H - 18 - 26 * len(panel)
        ov = out.copy()
        cv2.rectangle(ov, (0, y0 - 14), (520, H), (24, 20, 17), -1)
        out = cv2.addWeighted(ov, 0.75, out, 0.25, 0)
        for i, line in enumerate(panel):
            c = (235, 235, 235)
            if line.startswith("cmd_vel") and (cmd.get("lin_exceeded") or cmd.get("ang_exceeded")):
                c = (60, 190, 255)
            cv2.putText(out, line, (12, y0 + 12 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 1, cv2.LINE_AA)
        if cmd.get("lin_exceeded") or cmd.get("ang_exceeded"):
            what = "linear" if cmd.get("lin_exceeded") else "angular"
            lim = th["linear"] if what == "linear" else th["angular"]
            cv2.putText(out, f"THRESHOLD: {what} > {lim}", (12, y0 - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 190, 255), 2, cv2.LINE_AA)
        # mini-map
        if self.map_img is not None:
            res, ox, oy, mw, mh = self.map_info
            scale = 330.0 / mw
            mm = cv2.resize(self.map_img, (int(mw * scale), int(mh * scale)), interpolation=cv2.INTER_AREA)
            to_px = lambda X, Y: (int((X - ox) / res * scale), int((mh - (Y - oy) / res) * scale))  # noqa: E731
            if len(self.plan) > 1 and nav == "navigating":
                cv2.polylines(mm, [np.array([to_px(*p) for p in self.plan], np.int32)], False, (255, 160, 60), 2)
            if goal:
                cv2.circle(mm, to_px(goal["x"], goal["y"]), 5, (60, 190, 255), -1)
            if self.xyz:
                cv2.circle(mm, to_px(self.xyz[0], self.xyz[1]), 5, (120, 210, 60), -1)
            mh2, mw2 = mm.shape[:2]
            x1, y1 = W - mw2 - 10, 54
            cv2.rectangle(out, (x1 - 3, y1 - 3), (x1 + mw2 + 2, y1 + mh2 + 2), (24, 20, 17), -1)
            out[y1:y1 + mh2, x1:x1 + mw2] = mm
        cv2.putText(out, f"t={t:5.1f}s", (W - 110, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return out


def main():
    rclpy.init()
    node = Recorder()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.time()
    while node.frame is None and time.time() - t0 < 60:
        time.sleep(0.2)
    if node.frame is None:
        node.get_logger().error("no camera frames on /demo_cam/chase/image_raw")
        return
    H, W = node.frame.shape[:2]
    vw = cv2.VideoWriter(node.out, cv2.VideoWriter_fourcc(*"mp4v"), node.fps, (W, H))
    node.get_logger().info(f"recording {W}x{H} @ {node.fps} fps -> {node.out}")
    t0 = time.time()
    n = 0
    while time.time() - t0 < node.duration and rclpy.ok():
        with node.lock:
            f = node.frame
        vw.write(node.compose(f, time.time() - t0))
        n += 1
        nxt = t0 + n / node.fps
        time.sleep(max(0.0, nxt - time.time()))
        if node.stop_file and os.path.exists(node.stop_file):
            break
    vw.release()
    node.get_logger().info(f"wrote {n} frames to {node.out}")
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
