#!/usr/bin/env python3
"""
robot_state_manager: the custom telemetry/command node for Task 3.

Responsibilities (per the assignment):
  1. Position publisher - take the robot's localization data and publish the
     raw [x, y, z] on /robot/current_xyz at 10 Hz.
  2. Goal receiver - listen for a target pose on /robot/next_waypoint.
  3. Nav2 interfacing - on receiving a waypoint, drive there with the Nav2
     NavigateToPose action client.
  4. Command monitor - echo /cmd_vel and log whenever linear or angular speed
     exceeds user-defined thresholds.

Extra (used by the web dashboard, harmless otherwise):
  - /robot/cancel (std_msgs/Empty): cancel the active goal.
  - /robot/nav_status (std_msgs/String, JSON): navigation state machine
    (idle / navigating / succeeded / aborted / canceled / rejected /
    server_unavailable), active goal, distance remaining, recoveries, and the
    latest cmd_vel with threshold flags. Published at 5 Hz and on every change.

Design notes
------------
* "MapsToPose" in the assignment text -> Nav2's `nav2_msgs/action/NavigateToPose`.
  There is no MapsToPose action; NavigateToPose is the action that takes a
  target pose and drives the robot there.
* Position source. By default the node publishes the *localized* pose, i.e.
  the map -> base_footprint transform (AMCL's map->odom composed with the
  diff-drive odom->base_footprint). That is what "localization data" means and
  it is in the same frame as the waypoints. If TF is not available yet it falls
  back to raw /odom so the topic never goes silent. `pose_source:=odom` forces
  the odometry-only behaviour.
* The 10 Hz publish timer runs on the steady (wall) clock. With use_sim_time
  a sim-time timer ticks at 10 Hz of *simulated* time, which is only ~7.5 Hz of
  real time when Gazebo runs below real-time factor 1.0 (as it does with this
  150k-triangle scanned mesh). Consumers of /robot/current_xyz expect 10 Hz
  real time, so wall clock is the right clock for this timer.
* Message types: /robot/current_xyz is std_msgs/Float32MultiArray [x, y, z]
  (the assignment's "simple array" option); /robot/next_waypoint is
  geometry_msgs/PoseStamped, which is exactly NavigateToPose's goal type.
  A blank frame_id defaults to "map"; identity orientation is fine if you only
  care about position.
"""
import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty, Float32MultiArray, String
from tf2_ros import Buffer, TransformListener, TransformException

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_ABORTED: "aborted",
    GoalStatus.STATUS_CANCELED: "canceled",
}


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotStateManager(Node):
    def __init__(self):
        super().__init__("robot_state_manager")

        # ---- parameters ----
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("current_xyz_topic", "/robot/current_xyz")
        self.declare_parameter("waypoint_topic", "/robot/next_waypoint")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/robot/nav_status")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("linear_velocity_threshold", 0.20)   # m/s
        self.declare_parameter("angular_velocity_threshold", 0.80)  # rad/s
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("pose_source", "tf")                 # "tf" (map frame) or "odom"
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("echo_cmd_vel", True)
        self.declare_parameter("echo_period_sec", 1.0)
        self.declare_parameter("success_check_distance", 0.5)       # m

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        odom_topic = gp("odom_topic")
        xyz_topic = gp("current_xyz_topic")
        waypoint_topic = gp("waypoint_topic")
        cmd_vel_topic = gp("cmd_vel_topic")
        status_topic = gp("status_topic")
        publish_rate = float(gp("publish_rate_hz"))
        self._lin_thresh = float(gp("linear_velocity_threshold"))
        self._ang_thresh = float(gp("angular_velocity_threshold"))
        action_name = gp("navigate_to_pose_action")
        self._pose_source = str(gp("pose_source")).lower()
        self._global_frame = gp("global_frame")
        self._base_frame = gp("base_frame")
        self._echo = bool(gp("echo_cmd_vel"))
        self._echo_period = float(gp("echo_period_sec"))
        self._success_check = float(gp("success_check_distance"))

        # ---- state ----
        self._odom_xyz = None
        self._xyz = None
        self._xyz_frame = None
        self._yaw = 0.0
        self._goal_handle = None
        self._goal_seq = 0
        self._nav = {"state": "idle", "goal": None, "distance_remaining": None,
                     "recoveries": 0, "nav_time_s": None, "last_result": None,
                     "goal_error_m": None, "verified": None}
        self._cmd = {"v": 0.0, "w": 0.0, "lin_exceeded": False, "ang_exceeded": False}
        self._exceed_count = {"linear": 0, "angular": 0}
        self._wall = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_echo = self._wall.now()

        # ---- TF (localized pose) ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # ---- 1. position publisher ----
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, sensor_qos)
        self._xyz_pub = self.create_publisher(Float32MultiArray, xyz_topic, 10)
        self._xyz_timer = self.create_timer(1.0 / publish_rate, self._publish_xyz, clock=self._wall)

        # ---- 2 & 3. goal receiver + Nav2 action client ----
        self._waypoint_sub = self.create_subscription(PoseStamped, waypoint_topic, self._on_waypoint, 10)
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._cancel_sub = self.create_subscription(Empty, "/robot/cancel", self._on_cancel, 10)

        # ---- 4. command monitor ----
        self._cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, sensor_qos)

        # ---- status for the dashboard ----
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._status_timer = self.create_timer(0.2, self._publish_status, clock=self._wall)

        self.get_logger().info(
            f"robot_state_manager up. pose<-{'TF ' + self._global_frame + '->' + self._base_frame if self._pose_source == 'tf' else odom_topic} "
            f"xyz->{xyz_topic}@{publish_rate:g}Hz(wall)  waypoint<-{waypoint_topic}  cmd_vel<-{cmd_vel_topic}  "
            f"thresholds: |v|>{self._lin_thresh} m/s, |w|>{self._ang_thresh} rad/s  action: {action_name}")

    # ------------------------------------------------------------------ 1
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._odom_xyz = (p.x, p.y, p.z)
        self._odom_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def _lookup_tf_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(self._global_frame, self._base_frame, Time())
        except TransformException:
            return None
        tr = t.transform.translation
        return (tr.x, tr.y, tr.z), yaw_from_quat(t.transform.rotation)

    def _publish_xyz(self):
        pose = self._lookup_tf_pose() if self._pose_source == "tf" else None
        if pose is not None:
            (self._xyz, self._yaw), self._xyz_frame = pose, self._global_frame
        elif self._odom_xyz is not None:
            self._xyz, self._yaw, self._xyz_frame = self._odom_xyz, self._odom_yaw, "odom"
        else:
            return  # nothing received yet; don't publish a fake (0,0,0)
        msg = Float32MultiArray()
        msg.data = [float(v) for v in self._xyz]
        self._xyz_pub.publish(msg)

    # ------------------------------------------------------------------ 2 & 3
    def _on_waypoint(self, msg: PoseStamped):
        p = msg.pose.position
        frame = msg.header.frame_id or self._global_frame
        self.get_logger().info(f"New waypoint received: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) frame={frame}")

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server not available - is bt_navigator active? Dropping waypoint.")
            self._set_nav(state="server_unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.frame_id = frame
        # Stamp 0 = "latest available transform" for Nav2 - the safest choice.
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        if goal.pose.pose.orientation.x == goal.pose.pose.orientation.y == \
                goal.pose.pose.orientation.z == goal.pose.pose.orientation.w == 0.0:
            goal.pose.pose.orientation.w = 1.0  # all-zero quaternion is invalid

        self._goal_seq += 1
        seq = self._goal_seq
        self._set_nav(state="sending", goal={"x": p.x, "y": p.y, "z": p.z,
                                             "yaw": yaw_from_quat(goal.pose.pose.orientation),
                                             "frame": frame, "id": seq},
                      distance_remaining=None, recoveries=0, nav_time_s=None,
                      goal_error_m=None, verified=None)
        fut = self._nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(lambda f, s=seq: self._on_goal_response(f, s))

    def _on_cancel(self, _msg):
        if self._goal_handle is None:
            self.get_logger().info("Cancel requested but no active goal.")
            return
        self.get_logger().info("Cancel requested - cancelling active NavigateToPose goal.")
        self._goal_handle.cancel_goal_async()

    def _on_goal_response(self, future, seq):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("NavigateToPose goal was rejected by Nav2.")
            if seq == self._goal_seq:
                self._set_nav(state="rejected", last_result="rejected")
            return
        self.get_logger().info("NavigateToPose goal accepted; navigating.")
        if seq == self._goal_seq:
            self._goal_handle = handle
            self._set_nav(state="navigating")
        handle.get_result_async().add_done_callback(lambda f, s=seq: self._on_result(f, s))

    def _on_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self._nav["distance_remaining"] = round(float(fb.distance_remaining), 3)
        self._nav["recoveries"] = int(fb.number_of_recoveries)
        self._nav["nav_time_s"] = round(fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9, 1)

    def _on_result(self, future, seq):
        status = future.result().status
        name = STATUS_NAMES.get(status, f"status_{status}")
        log = self.get_logger().info if status == GoalStatus.STATUS_SUCCEEDED else self.get_logger().warn
        log(f"NavigateToPose goal #{seq} finished: {name.upper()}")
        if seq != self._goal_seq:  # a newer goal preempted this one; its result is not interesting
            return
        self._goal_handle = None
        # Post-check: don't take Nav2's word for "succeeded". Compare the localized
        # pose with the goal. (Humble's controller_server can report "Reached the
        # goal!" if the goal pose fails to transform into the odom frame, because it
        # then compares against an all-zero pose - seen once right after startup.)
        err, verified = None, None
        pose = self._lookup_tf_pose()
        g = self._nav.get("goal") or {}
        if pose is not None and g:
            (x, y, _), _ = pose
            err = round(math.hypot(x - g["x"], y - g["y"]), 3)
            verified = err <= self._success_check
            if status == GoalStatus.STATUS_SUCCEEDED and not verified:
                self.get_logger().error(
                    f"Nav2 reported SUCCEEDED for goal #{seq} but the robot is {err:.2f} m from it "
                    f"(> {self._success_check:.2f} m) - treating the result as unverified.")
        self._set_nav(state=name, last_result=name, goal_error_m=err, verified=verified)

    # ------------------------------------------------------------------ 4
    def _on_cmd_vel(self, msg: Twist):
        v = math.hypot(msg.linear.x, msg.linear.y)
        w = abs(msg.angular.z)
        lin_x, ang_x = v > self._lin_thresh, w > self._ang_thresh
        self._cmd = {"v": round(msg.linear.x, 3), "w": round(msg.angular.z, 3),
                     "lin_exceeded": lin_x, "ang_exceeded": ang_x}
        now = self._wall.now()
        if self._echo and (now - self._last_echo) >= Duration(seconds=self._echo_period):
            self._last_echo = now
            self.get_logger().info(f"[cmd_vel] linear.x={msg.linear.x:+.3f} m/s  angular.z={msg.angular.z:+.3f} rad/s")
        if lin_x:
            self._exceed_count["linear"] += 1
            self.get_logger().warn(
                f"[cmd_vel] linear speed {v:.3f} m/s exceeds threshold {self._lin_thresh:.2f} m/s",
                throttle_duration_sec=0.5)
        if ang_x:
            self._exceed_count["angular"] += 1
            self.get_logger().warn(
                f"[cmd_vel] angular speed {w:.3f} rad/s exceeds threshold {self._ang_thresh:.2f} rad/s (turning fast)",
                throttle_duration_sec=0.5)

    # ------------------------------------------------------------------ status
    def _set_nav(self, **kw):
        self._nav.update(kw)
        self._publish_status()

    def _publish_status(self):
        s = dict(self._nav)
        s["pose"] = None if self._xyz is None else {
            "x": round(self._xyz[0], 3), "y": round(self._xyz[1], 3), "z": round(self._xyz[2], 3),
            "yaw": round(self._yaw, 3), "frame": self._xyz_frame}
        s["cmd_vel"] = self._cmd
        s["thresholds"] = {"linear": self._lin_thresh, "angular": self._ang_thresh}
        s["exceed_count"] = self._exceed_count
        self._status_pub.publish(String(data=json.dumps(s)))


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
