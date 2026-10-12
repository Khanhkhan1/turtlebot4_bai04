#!/usr/bin/env python3
"""Phần 3 check: is the robot where AMCL thinks it is?

Compares the localized pose (TF map->base_link: AMCL's map->odom plus odometry, i.e. what
Nav2 uses) with the robot's true pose in Gazebo (/sim_ground_truth_pose, world frame).
The robot is spawned where mapping started, so the map frame and the Gazebo world
coincide. (/amcl_pose itself is only republished every 0.25 m / 0.2 rad of motion.)

    ros2 run turtlebot4_bai04 compare_pose.py [seconds]
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class ComparePose(Node):
    def __init__(self):
        super().__init__("compare_pose")
        self.truth = None
        self.create_subscription(Odometry, "sim_ground_truth_pose",
                                 lambda m: setattr(self, "truth", m), qos_profile_sensor_data)
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def localized(self):
        try:
            return self.tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
        except Exception:
            return None


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = ComparePose()
    end = time.monotonic() + seconds
    last = 0.0
    worst = 0.0
    seen = False
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        loc = node.localized()
        if loc is None or node.truth is None or time.monotonic() - last < 1.0:
            continue
        last = time.monotonic()
        t = node.truth.pose.pose
        ax, ay, ayaw = loc.translation.x, loc.translation.y, yaw_of(loc.rotation)
        dyaw = math.degrees(math.remainder(ayaw - yaw_of(t.orientation), 2 * math.pi))
        err = math.hypot(ax - t.position.x, ay - t.position.y)
        worst = max(worst, err)
        seen = True
        print(f"AMCL ({ax:+6.2f}, {ay:+6.2f}, {math.degrees(ayaw):+6.1f} deg)"
              f"   Gazebo ({t.position.x:+6.2f}, {t.position.y:+6.2f}, {math.degrees(yaw_of(t.orientation)):+6.1f} deg)"
              f"   error {err * 100:5.1f} cm, {dyaw:+5.1f} deg", flush=True)
    if not seen:
        print("no map->base_link transform (is localization running? set a 2D Pose Estimate in RViz)")
    else:
        print(f"max position error {worst * 100:.1f} cm")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
