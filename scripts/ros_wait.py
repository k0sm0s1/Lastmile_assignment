#!/usr/bin/env python3
"""
Small readiness checks for run_demo.sh that don't depend on the ros2 CLI daemon
(which can get stuck under WSL2).

  python3 scripts/ros_wait.py topic /odom 150          # wait until a topic has a publisher
  python3 scripts/ros_wait.py active 90 bt_navigator amcl  # wait until lifecycle nodes are 'active'
  python3 scripts/ros_wait.py states map_server amcl ...   # print each node's lifecycle state
Exit code 0 = ready, 1 = timed out.
"""
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState


def state_of(node, name, timeout=3.0):
    cli = node.create_client(GetState, f"/{name}/get_state")
    if not cli.wait_for_service(timeout_sec=timeout):
        node.destroy_client(cli)
        return "offline"
    fut = cli.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    node.destroy_client(cli)
    return fut.result().current_state.label if fut.done() and fut.result() else "no reply"


def main():
    mode = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("ros_wait")
    ok = False
    try:
        if mode == "topic":
            topic, limit = sys.argv[2], float(sys.argv[3])
            t0 = time.time()
            while time.time() - t0 < limit and not ok:
                rclpy.spin_once(node, timeout_sec=0.5)
                ok = any(n == topic for n, _ in node.get_topic_names_and_types()) and node.count_publishers(topic) > 0
        elif mode == "active":
            limit, names = float(sys.argv[2]), sys.argv[3:]
            t0 = time.time()
            while time.time() - t0 < limit and not ok:
                ok = all(state_of(node, n) == "active" for n in names)
                if not ok:
                    time.sleep(1.0)
        elif mode == "states":
            for n in sys.argv[2:]:
                print(f"  {n:<18} {state_of(node, n)}")
            ok = True
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
