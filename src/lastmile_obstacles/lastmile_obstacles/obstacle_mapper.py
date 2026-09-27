#!/usr/bin/env python3
"""
obstacle_mapper: live lidar obstacle detection for Nav2, without the laser
obstacle layer.

Why this node exists
--------------------
Nav2's usual way to avoid things that are not on the map is the costmap
ObstacleLayer, which feeds /scan through a tf2 MessageFilter. In this project
(WSL2, Gazebo below real time) that layer froze the costmaps at start-up, so it
was removed and the costmaps use the static map only. This node puts live
obstacle avoidance back in a way that can't freeze:

  /scan + TF(map <- base_scan) + /map
        |
        v
  evidence grid (same cells as /map)
    - each laser hit that is NOT a known wall   -> +HIT evidence
    - each cell the beam passes through          -> -MISS evidence (clears)
    - cell is an obstacle while evidence >= THRESH
        |
        v
  /obstacle_map  (nav_msgs/OccupancyGrid: the /map walls + detections = 100)
        |
        v
  both costmaps read it through a *second StaticLayer* (use_maximum: true),
  so walls from /map stay, obstacles are added on top, and inflation keeps the
  robot's distance. A StaticLayer just copies a grid; there is no message
  filter to deadlock.

The robot is never told where the obstacles are: they are absent from
lastmile_map.pgm and only exist in Gazebo. Everything comes from the lidar.

Topics
  in : /map (latched), /scan, TF
  out: /obstacle_map (latched, republished at publish_rate_hz)
       /obstacle_mapper/obstacles (std_msgs/String JSON: detected clusters)
"""
import json
import math

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy import ndimage as ndi

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ObstacleMapper(Node):
    def __init__(self):
        super().__init__("obstacle_mapper")
        p = self.declare_parameter
        p("wall_margin", 0.20)        # m: hits this close to a mapped wall are the wall (AMCL error ~0.15 m)
        p("min_range", 0.30)          # m: ignore self-hits on the robot body
        p("max_range", 3.4)           # m: beyond this the lidar is unreliable / hits nothing
        p("hit_gain", 3)              # evidence added per hit
        p("miss_loss", 1)             # evidence removed per beam pass-through
        p("threshold", 6)             # evidence needed to call a cell an obstacle
        p("max_evidence", 15)
        p("beam_step", 3)             # use every Nth beam
        p("scan_rate_hz", 5.0)        # max scans processed per second
        p("publish_rate_hz", 2.0)
        p("inflate_cells", 1)         # grow detections by this many cells (laser sees only the front face)
        p("report_min_cells", 20)     # only clusters this big are logged/reported (all cells still go to the costmap)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.wall_margin = float(g("wall_margin"))
        self.min_range, self.max_range = float(g("min_range")), float(g("max_range"))
        self.hit_gain, self.miss_loss = int(g("hit_gain")), int(g("miss_loss"))
        self.threshold, self.max_ev = int(g("threshold")), int(g("max_evidence"))
        self.beam_step = int(g("beam_step"))
        self.scan_period = 1.0 / float(g("scan_rate_hz"))
        self.inflate = int(g("inflate_cells"))
        self.report_min = int(g("report_min_cells"))

        self.info = None          # map metadata
        self.known = None         # bool grid: mapped wall (+margin) or unscanned -> never an obstacle
        self.evidence = None      # int16 grid
        self.last_scan_t = 0.0
        self.clusters = []
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)

        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        self.create_subscription(LaserScan, "/scan", self.on_scan, sensor)
        self.pub = self.create_publisher(OccupancyGrid, "/obstacle_map", latched)
        self.info_pub = self.create_publisher(String, "/obstacle_mapper/obstacles", 10)
        self.create_timer(1.0 / float(g("publish_rate_hz")), self.publish, clock=self.wall)
        self.get_logger().info("obstacle_mapper up: /scan + /map -> /obstacle_map (waiting for /map)")

    # ------------------------------------------------------------------ map
    def on_map(self, m):
        w, h = m.info.width, m.info.height
        grid = np.array(m.data, dtype=np.int16).reshape(h, w)
        walls = grid > 50
        margin = max(1, int(round(self.wall_margin / m.info.resolution)))
        walls = ndi.binary_dilation(walls, iterations=margin)
        unknown = grid < 0
        self.known = walls | unknown
        self.base = np.array(m.data, dtype=np.int8)  # published underneath the detections
        self.info = m.info
        self.evidence = np.zeros((h, w), np.int16)
        self.get_logger().info(f"/map {w}x{h} received; wall margin {margin} cells")
        self.publish()  # publish an empty obstacle map right away so the costmaps don't wait

    # ------------------------------------------------------------------ scan
    def on_scan(self, s):
        if self.info is None:
            return
        now = self.wall.now().nanoseconds * 1e-9
        if now - self.last_scan_t < self.scan_period:
            return
        self.last_scan_t = now
        try:
            t = self.tf.lookup_transform("map", s.header.frame_id, Time())
        except TransformException:
            return
        sx, sy = t.transform.translation.x, t.transform.translation.y
        yaw = yaw_of(t.transform.rotation)

        ranges = np.asarray(s.ranges, dtype=np.float32)[::self.beam_step]
        angles = (s.angle_min + np.arange(len(s.ranges)) * s.angle_increment)[::self.beam_step] + yaw
        finite = np.isfinite(ranges) & (ranges > self.min_range)
        hit = finite & (ranges < min(self.max_range, s.range_max - 0.05))
        # beams with no return clear up to max_range
        no_return = np.isinf(ranges) | (finite & ~hit)
        clear_len = np.where(hit, ranges - 0.08, np.where(no_return, self.max_range, 0.0))
        clear_len = np.clip(clear_len, 0.0, self.max_range)

        res, ox, oy = self.info.resolution, self.info.origin.position.x, self.info.origin.position.y
        H, W = self.evidence.shape

        # --- misses: sample each beam every half cell up to just before the hit
        steps = np.arange(self.min_range, self.max_range, res * 0.5)
        L = clear_len[:, None]
        mask = steps[None, :] < L
        px = sx + steps[None, :] * np.cos(angles)[:, None]
        py = sy + steps[None, :] * np.sin(angles)[:, None]
        cx = ((px[mask] - ox) / res).astype(np.int32)
        cy = ((py[mask] - oy) / res).astype(np.int32)
        ok = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
        miss_cells = np.unique(cy[ok] * W + cx[ok])
        ev = self.evidence.reshape(-1)
        ev[miss_cells] = np.maximum(ev[miss_cells] - self.miss_loss, 0)

        # --- hits that are not known walls
        hx = sx + ranges[hit] * np.cos(angles[hit])
        hy = sy + ranges[hit] * np.sin(angles[hit])
        cx = ((hx - ox) / res).astype(np.int32)
        cy = ((hy - oy) / res).astype(np.int32)
        ok = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
        cx, cy = cx[ok], cy[ok]
        new = ~self.known[cy, cx]
        hit_cells = np.unique(cy[new] * W + cx[new])
        ev[hit_cells] = np.minimum(ev[hit_cells] + self.hit_gain, self.max_ev)

    # ------------------------------------------------------------------ publish
    def publish(self):
        if self.info is None:
            return
        obst = self.evidence >= self.threshold
        if self.inflate > 0 and obst.any():
            obst = ndi.binary_dilation(obst, iterations=self.inflate) & ~self.known
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.info
        # Walls + unknown from /map, detections on top. Carrying the walls too
        # keeps the layer correct whether the costmap merges it by maximum or
        # by overwrite (rolling-window static layers overwrite in some versions).
        data = self.base.copy()
        data[obst.reshape(-1)] = 100
        msg.data = data.tolist()
        self.pub.publish(msg)

        # cluster report (for logs + dashboard)
        labels, n = ndi.label(obst)
        clusters = []
        if n:
            res, ox, oy = self.info.resolution, self.info.origin.position.x, self.info.origin.position.y
            for i, sl in enumerate(ndi.find_objects(labels), start=1):
                rr, cc = np.nonzero(labels[sl] == i)
                if len(rr) < self.report_min:
                    continue
                y = oy + (rr.mean() + sl[0].start + 0.5) * res
                x = ox + (cc.mean() + sl[1].start + 0.5) * res
                clusters.append({"x": round(float(x), 2), "y": round(float(y), 2), "cells": int(len(rr))})
        for c in clusters:
            if not any(math.hypot(c["x"] - o["x"], c["y"] - o["y"]) < 0.5 for o in self.clusters):
                self.get_logger().info(f"new obstacle detected at ({c['x']:.2f}, {c['y']:.2f}) - {c['cells']} cells")
        self.clusters = clusters
        self.info_pub.publish(String(data=json.dumps({"count": len(clusters), "clusters": clusters})))


def main():
    rclpy.init()
    node = ObstacleMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
