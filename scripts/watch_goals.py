#!/usr/bin/env python3
"""Phần 4 with RViz: report every goal set with the "Nav2 Goal" tool (any NavigateToPose
goal) as it finishes, with the robot's pose on the map, until N goals have succeeded.

    ros2 run turtlebot4_bai04 watch_goals.py [N]      # default 3
"""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
DONE = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
        GoalStatus.STATUS_CANCELED: "CANCELED"}


class WatchGoals(Node):
    def __init__(self):
        super().__init__("watch_goals")
        self.started, self.finished, self.succeeded = {}, set(), 0
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self._status, 10)

    def robot_xy(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform.translation
            return f"({t.x:+.2f}, {t.y:+.2f})"
        except Exception:
            return "(unknown)"

    def _status(self, msg):
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid)
            if s.status in ACTIVE and gid not in self.started:
                self.started[gid] = time.monotonic()
                print(f"goal {len(self.started)}: started from {self.robot_xy()}", flush=True)
            elif s.status in DONE and gid in self.started and gid not in self.finished:
                self.finished.add(gid)
                self.succeeded += s.status == GoalStatus.STATUS_SUCCEEDED
                k = list(self.started).index(gid) + 1
                print(f"goal {k}: {DONE[s.status]} after {time.monotonic() - self.started[gid]:.0f} s,"
                      f" robot at {self.robot_xy()}", flush=True)


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rclpy.init()
    node = WatchGoals()
    print(f"waiting for {n} goals (RViz: Nav2 Goal, click and drag on the map)", flush=True)
    while rclpy.ok() and node.succeeded < n:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"RESULT: {node.succeeded} goals reached", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
