#!/usr/bin/env python3
"""
lastmile_bot idle animator: purely cosmetic, publishes a slowly-varying
sinusoidal JointTrajectory for the neck pan and both shoulder joints, so the
character looks alive while standing/driving around. Consumed by the
`libgazebo_ros_joint_pose_trajectory` plugin declared in
lastmile_bot_description/urdf/lastmile_bot.urdf.xacro on topic
`lastmile_bot/joint_trajectory` - that plugin directly sets joint positions
from each trajectory point (no physics/torque involved), which is the right
tool for a costume animation and the wrong tool for anything that needs to
actually push against the world.

Does not touch navigation in any way - the diff-drive base and lidar are
independent of this node.
"""
import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class IdleAnimator(Node):
    def __init__(self):
        super().__init__("lastmile_bot_idle_animator")

        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("neck_amplitude_rad", 0.5)
        self.declare_parameter("neck_period_s", 6.0)
        self.declare_parameter("arm_amplitude_rad", 0.35)
        self.declare_parameter("arm_period_s", 2.5)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self._neck_amp = float(self.get_parameter("neck_amplitude_rad").value)
        self._neck_period = float(self.get_parameter("neck_period_s").value)
        self._arm_amp = float(self.get_parameter("arm_amplitude_rad").value)
        self._arm_period = float(self.get_parameter("arm_period_s").value)

        self._pub = self.create_publisher(JointTrajectory, "lastmile_bot/joint_trajectory", 10)
        self._start_time = self.get_clock().now()
        self._period = 1.0 / rate
        self._timer = self.create_timer(self._period, self._tick)

        self.get_logger().info(
            f"lastmile_bot idle animator running: neck +/-{self._neck_amp:.2f}rad "
            f"every {self._neck_period:.1f}s, arms +/-{self._arm_amp:.2f}rad "
            f"every {self._arm_period:.1f}s (out of phase for a walking-ish sway)"
        )

    def _tick(self):
        t = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        neck_angle = self._neck_amp * math.sin(2 * math.pi * t / self._neck_period)
        # arms swing opposite each other, like an idle counter-sway
        left_arm_angle = self._arm_amp * math.sin(2 * math.pi * t / self._arm_period)
        right_arm_angle = -left_arm_angle

        msg = JointTrajectory()
        msg.joint_names = ["neck_joint", "left_shoulder_joint", "right_shoulder_joint"]
        point = JointTrajectoryPoint()
        point.positions = [neck_angle, left_arm_angle, right_arm_angle]
        # target reached by the next tick - keeps the plugin continuously interpolating
        point.time_from_start = Duration(sec=0, nanosec=int(self._period * 1e9))
        msg.points = [point]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IdleAnimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
