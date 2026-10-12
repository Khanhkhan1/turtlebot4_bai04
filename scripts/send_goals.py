#!/usr/bin/env python3
"""Phần 4 check: send navigation goals to Nav2 one after another (the same
NavigateToPose action RViz's "Nav2 Goal" tool uses) and report how each one ended.

    ros2 run turtlebot4_bai04 send_goals.py                 # default goals in the maze
    ros2 run turtlebot4_bai04 send_goals.py 4,-7.5,0 -7,7,90   # x,y,yaw_deg ...
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Three different areas of the maze (map frame = Gazebo world frame).
DEFAULT_GOALS = [(6.0, -2.5, 0.0), (4.0, -7.5, 180.0), (-7.0, 7.0, 90.0)]


def pose(nav, x, y, yaw_deg):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
    p.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
    return p


def main():
    goals = [tuple(float(v) for v in a.split(",")) for a in sys.argv[1:]] or DEFAULT_GOALS
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    results = []
    for k, (x, y, yaw) in enumerate(goals, 1):
        print(f"goal {k}: ({x:+.2f}, {y:+.2f}, {yaw:.0f} deg)", flush=True)
        t0 = time.monotonic()
        nav.goToPose(pose(nav, x, y, yaw))
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and int(time.monotonic() - t0) % 10 == 0:
                print(f"   distance remaining {fb.distance_remaining:5.2f} m, recoveries {fb.number_of_recoveries}",
                      flush=True)
            time.sleep(1.0)
        result = nav.getResult()
        name = {TaskResult.SUCCEEDED: "SUCCEEDED", TaskResult.CANCELED: "CANCELED",
                TaskResult.FAILED: "FAILED"}.get(result, str(result))
        results.append(name)
        print(f"goal {k}: {name} after {time.monotonic() - t0:.0f} s", flush=True)

    print("RESULT:", ", ".join(results))
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
