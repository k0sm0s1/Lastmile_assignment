#!/usr/bin/env python3
"""
delivery_node: a two-stop delivery mission on top of the existing stack, with
its own customer-facing web page (http://localhost:8081).

Mission (runs here, server-side, so a page refresh never loses progress):

  ready ──start──▶ to_pickup ──arrived──▶ at_pickup ──[Loaded]──▶ to_dropoff
        ──arrived──▶ at_dropoff ──[Unloaded]──▶ returning ──arrived──▶ delivered

Each leg is sent through /robot/next_waypoint, i.e. through the Task 3 node
(robot_state_manager), and a leg only counts as arrived when that node reports
the goal *succeeded and verified* (localized pose within 0.5 m of the stop).
A failed leg moves the mission to "problem" with Retry / Cancel.

Stops are either named places (known-good points in the scanned building) or a
tap on the map; a tapped point is snapped to the nearest reachable cell with
enough clearance for the robot, so the robot is never sent into a wall.

The page shows a *clean* floor plan derived from lastmile_map.pgm (floorplan.py:
gaps sealed, specks removed, walls straightened, traced to a vector outline).
It is for display and stop snapping only; Nav2 still uses the real map.

HTTP
  GET  /                 the page (web/delivery.html)
  GET  /api/map          clean vector floor plan (world coords) + places + map metadata
  GET  /api/stream       Server-Sent Events, ~5 Hz mission + robot snapshot
  POST /api/snap         {x,y}           -> nearest reachable point
  POST /api/mission      {pickup, dropoff} (each {name,x,y})
  POST /api/loaded       confirm loading at the pickup
  POST /api/unloaded     confirm unloading at the drop-off
  POST /api/retry        re-send the current leg after a problem
  POST /api/cancel       stop and return to the dock
"""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node

from scipy import ndimage as ndi

try:
    from lastmile_dashboard import floorplan as fpm
except ImportError:  # running from the source tree
    import floorplan as fpm
from std_msgs.msg import Empty, Float32MultiArray, String

DOCK = {"name": "Dock", "x": 0.0, "y": 0.0}
# Named places: ten real spots spread over the scanned building. Each one is on
# observed floor, >= 0.4 m from the nearest wall, and connected to the dock on
# the bounded planning map (lastmile_nav_map.pgm); build_floorplan() re-checks
# them at start-up and logs any that aren't. Ordered west -> east.
PLACES = [
    {"id": "library", "name": "Library", "detail": "West wing, north end", "x": -2.3, "y": 8.8},
    {"id": "meeting", "name": "Meeting Room", "detail": "West wing", "x": -2.4, "y": 5.0},
    {"id": "reception", "name": "Reception", "detail": "West wing, by the dock", "x": -2.4, "y": 2.0},
    {"id": "kitchen", "name": "Kitchen", "detail": "Main corridor", "x": 2.5, "y": 0.0},
    {"id": "mailroom", "name": "Mailroom", "detail": "Corridor junction", "x": 8.0, "y": 0.0},
    {"id": "print", "name": "Print Room", "detail": "North wing", "x": 8.9, "y": 2.6},
    {"id": "office", "name": "Office 2B", "detail": "North wing, far end", "x": 8.8, "y": 5.0},
    {"id": "hall", "name": "East Hall", "detail": "End of corridor", "x": 13.8, "y": 0.0},
    {"id": "lab", "name": "Lab", "detail": "South room", "x": 15.5, "y": -3.0},
    {"id": "server", "name": "Server Room", "detail": "South room, far end", "x": 15.5, "y": -4.3},
]
TERMINAL = {"succeeded", "aborted", "canceled", "rejected", "server_unavailable"}
LEG_OF = {"to_pickup": "pickup", "to_dropoff": "dropoff", "returning": "dock"}
ARRIVED_STATE = {"to_pickup": "at_pickup", "to_dropoff": "at_dropoff", "returning": "delivered"}


class Delivery(Node):
    def __init__(self):
        super().__init__("delivery_node")
        self.declare_parameter("port", 8081)
        self.declare_parameter("min_clearance", 0.35)   # m, for snapping tapped stops
        self.port = int(self.get_parameter("port").value)
        self.min_clear = float(self.get_parameter("min_clearance").value)
        self.lock = threading.RLock()

        self.map_meta = None
        self.reachable = None     # bool grid: free cells with enough clearance, connected to the dock
        self.xyz = None
        self.status = {}
        self.plan = []
        self.plan_id = 0
        self.obstacles = []
        self.speed = None         # EMA of progress along the route (m per wall-second)
        self._last_prog = None
        self.mission = None
        self.history = []
        self.status_t = 0.0

        self.floor = None
        self.floorplan = []
        self.build_floorplan()
        self.create_subscription(Float32MultiArray, "/robot/current_xyz", self.on_xyz, 10)
        self.create_subscription(String, "/robot/nav_status", self.on_status, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(String, "/obstacle_mapper/obstacles", self.on_obstacles, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/robot/next_waypoint", 10)
        self.cancel_pub = self.create_publisher(Empty, "/robot/cancel", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # ------------------------------------------------------------------ ROS
    def build_floorplan(self):
        """Clean vector floor plan + reachable-stop mask from lastmile_map.pgm (display only)."""
        share = get_package_share_directory("lastmile_navigation")
        meta = {}
        for line in open(os.path.join(share, "maps", "lastmile_map.yaml")):
            if ":" in line and not line.strip().startswith("#"):
                k, v = line.split(":", 1)
                meta[k.strip()] = v.strip()
        res = float(meta["resolution"])
        ox, oy = [float(t) for t in meta["origin"].strip("[]").split(",")[:2]]
        pgm = fpm.read_pgm(os.path.join(share, "maps", meta["image"]))
        H, W = pgm.shape
        dock_rc = (H - 1 - int((DOCK["y"] - oy) / res), int((DOCK["x"] - ox) / res))
        floor = fpm.regularize(fpm.clean_floor(pgm, dock_rc, 9, 3), dock_rc)       # image rows (top = max y)
        polys = fpm.outline(floor)
        world = [[[round(ox + (c + 0.5) * res, 3), round(oy + (H - 1 - r + 0.5) * res, 3)] for c, r in p] for p in polys]
        clear = ndi.distance_transform_edt(floor) * res >= self.min_clear
        lab, _ = ndi.label(clear)
        reach = (lab == lab[dock_rc]) if lab[dock_rc] else clear
        nav_clear = None
        nav_pgm = os.path.join(share, "maps", "lastmile_nav_map.pgm")
        if os.path.exists(nav_pgm):   # clearance to the planner's walls, for the un-stick nudge
            nav_clear = np.flipud(ndi.distance_transform_edt(fpm.read_pgm(nav_pgm) != 0) * res)
        with self.lock:
            self.nav_clear = nav_clear
            self.floor = np.flipud(floor)          # grid rows (row 0 = min y), like /map
            self.reachable = np.flipud(reach)
            self.floorplan = world
            self.map_meta = {"width": W, "height": H, "resolution": res, "origin": [ox, oy]}
        self.get_logger().info(f"floor plan: {len(world)} outline(s), {sum(len(p) for p in world)} vertices; "
                               f"{int(reach.sum())} reachable stop cells")
        for p in PLACES:
            r, c = H - 1 - int((p["y"] - oy) / res), int((p["x"] - ox) / res)
            if not (0 <= r < H and 0 <= c < W and reach[r, c]):
                self.get_logger().warn(f"place {p['name']} ({p['x']}, {p['y']}) is not on reachable floor")

    def on_xyz(self, m):
        with self.lock:
            self.xyz = list(m.data)

    def on_plan(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        step = max(1, len(pts) // 120)
        with self.lock:
            self.plan = [[round(x, 2), round(y, 2)] for x, y in pts[::step]] + ([list(map(lambda v: round(v, 2), pts[-1]))] if pts else [])
            self.plan_id += 1

    def on_obstacles(self, m):
        try:
            clusters = json.loads(m.data).get("clusters", [])
        except ValueError:
            return
        with self.lock:
            mm, keep = self.map_meta, []
            for c in clusters:  # show only obstacles inside the building (not scan clutter outside it)
                r = int((c["y"] - mm["origin"][1]) / mm["resolution"])
                col = int((c["x"] - mm["origin"][0]) / mm["resolution"])
                if 0 <= r < self.floor.shape[0] and 0 <= col < self.floor.shape[1] and self.floor[r, col]:
                    keep.append(c)
            self.obstacles = keep

    def on_status(self, m):
        try:
            s = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.status = s
            self.status_t = time.monotonic()
            ms = self.mission
            if not ms or ms["state"] not in LEG_OF:
                return
            g = s.get("goal") or {}
            gid = g.get("id")
            leg = ms["leg"]
            # bind the leg to the first new goal id at our target
            if leg.get("goal_id") is None and gid is not None and gid > leg["prev_id"] \
                    and math.hypot(g.get("x", 1e9) - leg["x"], g.get("y", 1e9) - leg["y"]) < 0.05:
                leg["goal_id"] = gid
            if leg.get("goal_id") != gid:
                return
            dr = s.get("distance_remaining")
            if dr is not None:
                now = time.monotonic()
                if self._last_prog and dr < self._last_prog[1]:
                    v = (self._last_prog[1] - dr) / max(now - self._last_prog[0], 1e-3)
                    self.speed = v if self.speed is None else 0.8 * self.speed + 0.2 * v
                self._last_prog = (now, dr)
            st = s.get("state")
            if st in TERMINAL:
                if st == "succeeded" and s.get("verified") is not False:
                    self._arrive()
                elif st == "aborted" and ms.get("auto_tries", 0) < 2:
                    # Nav2 gives up when the robot has drifted into a wall's inscribed
                    # zone (NavFn can't start a path there, and its backup refuses to
                    # move "into collision"). Nudge it out and resend the leg, twice at
                    # most, before asking the person.
                    ms["auto_tries"] = ms.get("auto_tries", 0) + 1
                    leg["goal_id"] = -1   # ignore further updates for this goal
                    threading.Thread(target=self._unstick_and_resend, args=(ms["id"], ms["state"]), daemon=True).start()
                else:
                    ms["problem"] = {"state": st, "error_m": s.get("goal_error_m")}
                    ms["resume"] = ms["state"]
                    self._set_state("problem")

    # ------------------------------------------------------------------ mission
    def _set_state(self, st):
        ms = self.mission
        ms["state"] = st
        ms["timeline"].append({"state": st, "t": time.time()})
        self.get_logger().info(f"delivery #{ms['id']}: {st}")

    def _send_leg(self, which):
        ms = self.mission
        tgt = ms[which] if which != "dock" else DOCK
        prev = (self.status.get("goal") or {}).get("id") or 0
        ms["leg"] = {"which": which, "x": tgt["x"], "y": tgt["y"], "prev_id": prev, "goal_id": None, "sent": time.time()}
        self._last_prog = None
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y = float(tgt["x"]), float(tgt["y"])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def _nudge(self):
        """Drive 15-20 cm straight forward or backward, whichever gets further from the walls."""
        with self.lock:
            xyz, yaw, clear, mm = self.xyz, (self.status.get("pose") or {}).get("yaw"), self.nav_clear, self.map_meta
        if not xyz or yaw is None or clear is None:
            return
        def clearance(x, y):
            r = int((y - mm["origin"][1]) / mm["resolution"])
            c = int((x - mm["origin"][0]) / mm["resolution"])
            return float(clear[r, c]) if 0 <= r < clear.shape[0] and 0 <= c < clear.shape[1] else 0.0
        here = clearance(xyz[0], xyz[1])
        best = max((clearance(xyz[0] + sgn * 0.2 * math.cos(yaw), xyz[1] + sgn * 0.2 * math.sin(yaw)), sgn) for sgn in (-1.0, 1.0))
        if best[0] <= here + 0.02:
            return
        sgn = best[1]
        self.get_logger().info(f"un-stick: {'forward' if sgn > 0 else 'backward'} "
                               f"(wall clearance {here:.2f} -> {best[0]:.2f} m)")
        t0, x0, y0 = time.monotonic(), xyz[0], xyz[1]
        tw = Twist()
        tw.linear.x = 0.08 * sgn
        while time.monotonic() - t0 < 6.0:
            with self.lock:
                p = self.xyz
            if p and math.hypot(p[0] - x0, p[1] - y0) >= 0.15:
                break
            self.cmd_pub.publish(tw)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())

    def _unstick_and_resend(self, mid, state):
        time.sleep(1.0)            # let Nav2 finish aborting before we drive
        try:
            self._nudge()
        except Exception as e:     # never let the helper kill the mission
            self.get_logger().warn(f"un-stick failed: {e}")
        time.sleep(1.0)
        with self.lock:
            ms = self.mission
            if ms and ms["id"] == mid and ms["state"] == state:
                self.get_logger().info(f"delivery #{mid}: resending the {LEG_OF[state]} leg (try {ms['auto_tries'] + 1})")
                self._send_leg(LEG_OF[state])

    def _arrive(self):
        ms = self.mission
        nxt = ARRIVED_STATE[ms["state"]]
        ms["leg"]["arrived"] = time.time()
        ms["auto_tries"] = 0
        self._set_state(nxt)
        if nxt == "delivered":
            t = ms["timeline"]
            ms["duration_s"] = round(t[-1]["t"] - t[0]["t"])
            self.history.insert(0, {"id": ms["id"], "pickup": ms["pickup"]["name"], "dropoff": ms["dropoff"]["name"],
                                    "duration_s": ms["duration_s"], "finished": time.strftime("%H:%M")})
            self.history = self.history[:5]

    def start(self, pickup, dropoff):
        with self.lock:
            if self.mission and self.mission["state"] not in ("delivered", "canceled"):
                raise ValueError("a delivery is already running")
            mid = (self.history[0]["id"] + 1) if self.history else (self.mission["id"] + 1 if self.mission else 1)
            self.mission = {"id": mid, "pickup": pickup, "dropoff": dropoff, "dock": DOCK,
                            "state": "ready", "timeline": [], "leg": {}}
            self._set_state("to_pickup")
            self._send_leg("pickup")

    def confirm(self, what):
        with self.lock:
            ms = self.mission
            if what == "loaded" and ms and ms["state"] == "at_pickup":
                self._set_state("to_dropoff")
                self._send_leg("dropoff")
            elif what == "unloaded" and ms and ms["state"] == "at_dropoff":
                self._set_state("returning")
                self._send_leg("dock")
            else:
                raise ValueError(f"'{what}' isn't available right now")

    def retry(self):
        with self.lock:
            ms = self.mission
            if not ms or ms["state"] != "problem":
                raise ValueError("nothing to retry")
            st = ms.pop("resume")
            ms.pop("problem", None)
            ms["auto_tries"] = 2   # the person asked: one nudge + resend, no more automatic tries
            self._set_state(st)
            threading.Thread(target=self._unstick_and_resend, args=(ms["id"], st), daemon=True).start()

    def cancel(self):
        with self.lock:
            ms = self.mission
            if not ms or ms["state"] in ("delivered", "canceled"):
                raise ValueError("no delivery to cancel")
            # a new goal preempts the current one in Nav2, so no separate cancel is needed
            self._set_state("returning")
            ms["canceled"] = True
            self._send_leg("dock")

    def snap(self, x, y):
        with self.lock:
            if self.reachable is None:
                raise ValueError("map not loaded yet")
            m = self.map_meta
            res, ox, oy = m["resolution"], m["origin"][0], m["origin"][1]
            r, c = int((y - oy) / res), int((x - ox) / res)
            H, W = self.reachable.shape
            rr, cc = np.nonzero(self.reachable[max(0, r - 40):min(H, r + 41), max(0, c - 40):min(W, c + 41)])
            if not len(rr):
                raise ValueError("that spot isn't reachable - tap inside a corridor")
            rr, cc = rr + max(0, r - 40), cc + max(0, c - 40)
            i = int(np.argmin((rr - r) ** 2 + (cc - c) ** 2))
            return {"x": round(ox + (cc[i] + 0.5) * res, 2), "y": round(oy + (rr[i] + 0.5) * res, 2),
                    "moved_m": round(math.hypot(cc[i] - c, rr[i] - r) * res, 2)}

    # ------------------------------------------------------------------ snapshot
    def snapshot(self, plan_known):
        with self.lock:
            s = self.status or {}
            ms = json.loads(json.dumps(self.mission)) if self.mission else None
            if ms and ms["state"] in LEG_OF and ms["leg"].get("goal_id") == (s.get("goal") or {}).get("id"):
                dr = s.get("distance_remaining")
                ms["distance_m"] = dr
                ms["eta_s"] = round(dr / self.speed) if (dr and self.speed and self.speed > 0.02) else None
            snap = {"robot": {"xyz": self.xyz, "yaw": (s.get("pose") or {}).get("yaw"),
                              "online": time.monotonic() - self.status_t < 3.0,
                              "moving": abs((s.get("cmd_vel") or {}).get("v", 0.0)) > 0.02},
                    "mission": ms, "history": self.history, "obstacles": self.obstacles,
                    "plan_id": self.plan_id, "server_time": time.time()}
            if plan_known != self.plan_id:
                snap["plan"] = self.plan
            return snap


def handler(node, web_dir):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(os.path.join(web_dir, "delivery.html"), "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/api/map":
                self._send(200, json.dumps({"meta": node.map_meta, "floorplan": node.floorplan,
                                            "places": PLACES, "dock": DOCK}).encode())
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
                        time.sleep(0.2)
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self._send(404, b'{"ok":false}')

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
                out = {"ok": True}
                if self.path == "/api/snap":
                    out.update(node.snap(float(body["x"]), float(body["y"])))
                elif self.path == "/api/mission":
                    pk, dp = body["pickup"], body["dropoff"]
                    if math.hypot(pk["x"] - dp["x"], pk["y"] - dp["y"]) < 0.5:
                        raise ValueError("pickup and drop-off are the same place")
                    node.start({"name": str(pk.get("name") or "Pickup"), "x": float(pk["x"]), "y": float(pk["y"])},
                               {"name": str(dp.get("name") or "Drop-off"), "x": float(dp["x"]), "y": float(dp["y"])})
                elif self.path == "/api/loaded":
                    node.confirm("loaded")
                elif self.path == "/api/unloaded":
                    node.confirm("unloaded")
                elif self.path == "/api/retry":
                    node.retry()
                elif self.path == "/api/cancel":
                    node.cancel()
                else:
                    return self._send(404, b'{"ok":false,"error":"unknown"}')
                self._send(200, json.dumps(out).encode())
            except (KeyError, TypeError, ValueError) as e:
                self._send(400, json.dumps({"ok": False, "error": str(e)}).encode())

    return H


def main():
    rclpy.init()
    node = Delivery()
    web_dir = os.path.join(get_package_share_directory("lastmile_dashboard"), "web")
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", node.port), handler(node, web_dir))
    srv.daemon_threads = True
    node.get_logger().info(f"delivery page at http://localhost:{node.port}/")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
