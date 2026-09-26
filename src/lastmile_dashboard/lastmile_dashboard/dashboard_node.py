#!/usr/bin/env python3
"""
lastmile_dashboard: a live browser dashboard for the robot.

One rclpy node + a stdlib HTTP server (no rosbridge, no pip installs):
  GET  /                -> the single-page dashboard (web/index.html)
  GET  /map.png         -> the /map occupancy grid rendered as PNG
  GET  /api/map         -> map metadata (size, resolution, origin)
  GET  /api/stream      -> Server-Sent Events, ~10 Hz state snapshots
  POST /api/goal        -> {"x","y","yaw"}  publishes /robot/next_waypoint
  POST /api/cancel      -> publishes /robot/cancel (robot_state_manager cancels the goal)
  POST /api/initialpose -> {"x","y","yaw"}  publishes /initialpose (re-seed AMCL)

Everything the page shows comes from the live ROS graph:
  /robot/current_xyz  (Task 3 node, 10 Hz)      /robot/nav_status (Task 3 node, JSON)
  /cmd_vel            (Nav2 controller output)  /plan             (Nav2 global plan)
  /scan + TF          (lidar points in map)     /amcl_pose        (covariance)
  TF map->odom age    (is AMCL localizing?)     lifecycle states of the Nav2 servers
"""
import json
import math
import os
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.clock import Clock, ClockType

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float32MultiArray, String
from tf2_ros import Buffer, TransformListener, TransformException

NAV2_NODES = ["map_server", "amcl", "controller_server", "planner_server", "behavior_server", "bt_navigator"]
TERMINAL = {"succeeded", "aborted", "canceled", "rejected", "server_unavailable"}


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def png_gray(width, height, rows):
    """Minimal 8-bit grayscale PNG encoder (stdlib only). rows: list of bytes, top row first."""
    raw = b"".join(b"\x00" + r for r in rows)

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


class Dashboard(Node):
    def __init__(self):
        super().__init__("lastmile_dashboard")
        self.declare_parameter("port", 8080)
        self.declare_parameter("host", "0.0.0.0")
        self.port = int(self.get_parameter("port").value)
        self.host = str(self.get_parameter("host").value)

        self.lock = threading.Lock()
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.map_meta = None
        self.map_png = None
        self.xyz = None
        self.xyz_times = []
        self.status = {}
        self.cmd = {"v": 0.0, "w": 0.0}
        self.plan = []
        self.plan_id = 0
        self.scan_pts = []
        self.amcl_cov = None
        self.lifecycle = {n: "unknown" for n in NAV2_NODES}
        self.history = []          # [{id,x,y,yaw,sent,result,finished}]
        self.alerts = []           # recent threshold alerts
        self._last_state = None
        self._last_alert = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(Float32MultiArray, "/robot/current_xyz", self.on_xyz, 10)
        self.create_subscription(String, "/robot/nav_status", self.on_status, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, sensor)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, sensor)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, latched)

        self.goal_pub = self.create_publisher(PoseStamped, "/robot/next_waypoint", 10)
        self.cancel_pub = self.create_publisher(Empty, "/robot/cancel", 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.lc_clients = {n: self.create_client(GetState, f"/{n}/get_state") for n in NAV2_NODES}
        self.create_timer(2.0, self.poll_lifecycle, clock=self.wall)

    # ---------------- ROS callbacks ----------------
    def on_map(self, m):
        w, h = m.info.width, m.info.height
        data = m.data
        lut = bytes([(46 if v == -1 else 0) if v < 0 else (226 if v < 25 else (8 if v > 65 else 120)) for v in range(-128, 128)])
        rows = []
        for r in range(h - 1, -1, -1):  # PNG top row = max y
            row = data[r * w:(r + 1) * w]
            rows.append(bytes(lut[(v + 128) & 0xFF] for v in row))
        png = png_gray(w, h, rows)
        with self.lock:
            self.map_png = png
            self.map_meta = {"width": w, "height": h, "resolution": m.info.resolution,
                             "origin": [m.info.origin.position.x, m.info.origin.position.y]}
        self.get_logger().info(f"map received {w}x{h} @ {m.info.resolution} m")

    def on_xyz(self, m):
        now = time.monotonic()
        with self.lock:
            self.xyz = list(m.data)
            self.xyz_times.append(now)
            self.xyz_times = [t for t in self.xyz_times if now - t < 2.0]

    def on_status(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.status = s
            st, goal = s.get("state"), s.get("goal") or {}
            gid = goal.get("id")
            # history: one row per goal id the Task 3 node has handled (from the
            # dashboard, the CLI, or anywhere else that publishes /robot/next_waypoint)
            if gid is not None and not any(h["id"] == gid for h in self.history):
                for h in self.history:
                    if h.get("result") is None:
                        h["result"] = "preempted"
                self.history.append({"id": gid, "x": round(goal["x"], 2), "y": round(goal["y"], 2),
                                     "yaw": round(goal.get("yaw", 0.0), 2),
                                     "sent": time.strftime("%H:%M:%S"), "result": None})
                self.history = self.history[-50:]
            if st in TERMINAL and gid is not None and self._last_state != (gid, st):
                for h in self.history:
                    if h["id"] == gid and h.get("result") is None:
                        h["result"] = st
                        h["finished"] = time.strftime("%H:%M:%S")
                        h["nav_time_s"] = s.get("nav_time_s")
                        h["recoveries"] = s.get("recoveries")
                        h["error_m"] = s.get("goal_error_m")
                        h["verified"] = s.get("verified")
            self._last_state = (gid, st)
            cv = s.get("cmd_vel") or {}
            if (cv.get("lin_exceeded") or cv.get("ang_exceeded")) and time.monotonic() - self._last_alert > 0.5:
                self._last_alert = time.monotonic()
                kind = "linear" if cv.get("lin_exceeded") else "angular"
                self.alerts.append({"t": time.strftime("%H:%M:%S"), "kind": kind,
                                    "v": cv.get("v"), "w": cv.get("w")})
                self.alerts = self.alerts[-30:]

    def on_cmd(self, m):
        with self.lock:
            self.cmd = {"v": round(m.linear.x, 3), "w": round(m.angular.z, 3)}

    def on_plan(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        step = max(1, len(pts) // 150)
        pts = pts[::step] + (pts[-1:] if pts else [])
        with self.lock:
            self.plan = [[round(x, 3), round(y, 3)] for x, y in pts]
            self.plan_id += 1

    def on_scan(self, m):
        try:
            t = self.tf_buffer.lookup_transform("map", m.header.frame_id, Time())
        except TransformException:
            return
        tx, ty = t.transform.translation.x, t.transform.translation.y
        yaw = quat_to_yaw(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        for i in range(0, len(m.ranges), 3):
            r = m.ranges[i]
            if not (m.range_min < r < m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            pts.append([round(tx + c * lx - s * ly, 3), round(ty + s * lx + c * ly, 3)])
        with self.lock:
            self.scan_pts = pts

    def on_amcl(self, m):
        cv = m.pose.covariance
        with self.lock:
            self.amcl_cov = {"xx": cv[0], "yy": cv[7], "yawyaw": cv[35]}

    def poll_lifecycle(self):
        for n, cli in self.lc_clients.items():
            if not cli.service_is_ready():
                self.lifecycle[n] = "offline"
                continue
            fut = cli.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=n: self.lifecycle.__setitem__(n, f.result().current_state.label if f.result() else "error"))

    # ---------------- helpers ----------------
    def map_odom_age(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "odom", Time())
        except TransformException:
            return None
        now = self.get_clock().now()
        return round((now - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9, 2)

    def snapshot(self, plan_known):
        age = self.map_odom_age()
        with self.lock:
            n = len(self.xyz_times)
            hz = (n - 1) / (self.xyz_times[-1] - self.xyz_times[0]) if n > 2 else 0.0
            s = dict(self.status)
            snap = {
                "xyz": self.xyz, "xyz_hz": round(hz, 2),
                "pose": s.get("pose"), "nav": {k: s.get(k) for k in ("state", "goal", "distance_remaining", "recoveries", "nav_time_s",
                                                "last_result", "goal_error_m", "verified")},
                "cmd": self.cmd, "cmd_flags": s.get("cmd_vel"), "thresholds": s.get("thresholds"),
                "exceed_count": s.get("exceed_count"),
                "scan": self.scan_pts, "plan_id": self.plan_id,
                "health": {
                    # AMCL future-dates map->odom by transform_tolerance, so a small negative age is normal
                    "map_odom_age": age, "localized": age is not None and age < 3.0,
                    "amcl_cov": self.amcl_cov, "lifecycle": dict(self.lifecycle),
                    "nav2_active": all(v == "active" for v in self.lifecycle.values()),
                    "task3_node": bool(s) and hz > 5.0,
                },
                "history": self.history[-12:], "alerts": self.alerts[-10:],
            }
            if plan_known != self.plan_id:
                snap["plan"] = self.plan
        return snap

    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        q = yaw_to_quat(float(yaw))
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.goal_pub.publish(msg)
        self.get_logger().info(f"dashboard goal -> ({x:.2f}, {y:.2f}, yaw {yaw:.2f})")

    def send_initialpose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        q = yaw_to_quat(float(yaw))
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.07
        msg.pose.covariance = cov
        self.init_pub.publish(msg)


def make_handler(node: Dashboard, web_dir: str):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(os.path.join(web_dir, "index.html"), "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            elif self.path.startswith("/map.png"):
                with node.lock:
                    png = node.map_png
                self._send(200, png, "image/png") if png else self._send(503, b"no map yet", "text/plain")
            elif self.path == "/api/map":
                with node.lock:
                    meta = node.map_meta
                self._send(200, json.dumps(meta).encode(), "application/json")
            elif self.path == "/api/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                known = -1
                try:
                    while rclpy.ok():
                        snap = node.snapshot(known)
                        if "plan" in snap:
                            known = snap["plan_id"]
                        self.wfile.write(b"data: " + json.dumps(snap).encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._send(400, b'{"ok":false,"error":"bad json"}', "application/json")
            try:
                if self.path == "/api/goal":
                    node.send_goal(body["x"], body["y"], body.get("yaw", 0.0))
                elif self.path == "/api/cancel":
                    node.cancel_pub.publish(Empty())
                elif self.path == "/api/initialpose":
                    node.send_initialpose(body["x"], body["y"], body.get("yaw", 0.0))
                else:
                    return self._send(404, b'{"ok":false}', "application/json")
            except (KeyError, TypeError, ValueError) as e:
                return self._send(400, json.dumps({"ok": False, "error": str(e)}).encode(), "application/json")
            self._send(200, b'{"ok":true}', "application/json")

    return H


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    web_dir = os.path.join(get_package_share_directory("lastmile_dashboard"), "web")
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    srv = ThreadingHTTPServer((node.host, node.port), make_handler(node, web_dir))
    srv.daemon_threads = True
    node.get_logger().info(f"dashboard at http://localhost:{node.port}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
