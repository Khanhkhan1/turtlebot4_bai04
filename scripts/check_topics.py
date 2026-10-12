#!/usr/bin/env python3
"""Phần 1 check: /scan and /odom are published, and the robot moves on /cmd_vel.

Listens for a few seconds (rates, scan size), then drives forward briefly and reports
the distance travelled according to /odom.

    ros2 run turtlebot4_bai04 check_topics.py
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class TopicCheck(Node):
    def __init__(self):
        super().__init__("check_topics")
        self.scans, self.odoms = [], []
        self.last_scan, self.last_odom = None, None
        self.create_subscription(LaserScan, "scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self._odom, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def _scan(self, msg):
        self.scans.append(time.monotonic())
        self.last_scan = msg

    def _odom(self, msg):
        self.odoms.append(time.monotonic())
        self.last_odom = msg

    def spin_for(self, seconds, while_doing=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if while_doing:
                while_doing()
            rclpy.spin_once(self, timeout_sec=0.05)


def rate(stamps):
    return (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 2 else 0.0


def main():
    rclpy.init()
    node = TopicCheck()
    node.spin_for(5.0)

    ok = True
    if node.last_scan:
        s = node.last_scan
        valid = [r for r in s.ranges if math.isfinite(r)]
        print(f"/scan  OK  {rate(node.scans):5.1f} Hz  frame={s.header.frame_id}  "
              f"{len(s.ranges)} rays  {len(valid)} hits  range {s.range_min:.2f}-{s.range_max:.1f} m")
    else:
        print("/scan  NO DATA")
        ok = False
    if node.last_odom:
        p = node.last_odom.pose.pose.position
        print(f"/odom  OK  {rate(node.odoms):5.1f} Hz  frame={node.last_odom.header.frame_id}"
              f"->{node.last_odom.child_frame_id}  pose ({p.x:.2f}, {p.y:.2f})")
    else:
        print("/odom  NO DATA")
        ok = False

    if node.last_odom:
        start = node.last_odom.pose.pose.position
        cmd = Twist()
        cmd.linear.x = 0.2
        node.spin_for(3.0, lambda: node.cmd_pub.publish(cmd))
        node.cmd_pub.publish(Twist())
        node.spin_for(1.0)
        end = node.last_odom.pose.pose.position
        moved = math.hypot(end.x - start.x, end.y - start.y)
        print(f"/cmd_vel  {'OK' if moved > 0.1 else 'NO MOTION'}  0.2 m/s for 3 s -> moved {moved:.2f} m (odom)")
        ok = ok and moved > 0.1

    print("RESULT:", "OK" if ok else "FAILED")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
