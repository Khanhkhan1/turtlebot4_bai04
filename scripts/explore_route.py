#!/usr/bin/env python3
"""Phần 2 without a keyboard: drive the TurtleBot 4 around the maze through /cmd_vel (in
place of the teleop operator) so SLAM Toolbox can map it. Coverage waypoints on the free
space of the world file, A* between them, pure pursuit on the SLAM pose (TF map->base_link),
lidar safety stop. The route comes from the world's walls, the map only from SLAM.

    ros2 run turtlebot4_bai04 explore_route.py [max_minutes] [world.sdf]
"""
import heapq
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

RES = 0.1          # planning grid [m]
HALF = 10.0        # maze spans [-10, 10]
CLEARANCE = 0.42   # robot radius (0.17) + margin, for planning
SPACING = 3.0      # coverage waypoint spacing [m]


def wall_boxes(sdf):
    root = ET.parse(sdf).getroot()

    def pose(el):
        p = el.find("pose")
        return [float(v) for v in p.text.split()] if p is not None and p.text else [0.0] * 6

    boxes = []
    for model in root.iter("model"):
        if model.get("name") == "ground_plane":
            continue
        mp = pose(model)
        for link in model.findall("link"):
            col = link.find("collision")
            if col is None or col.find("geometry/box/size") is None:
                continue
            lp, cp = pose(link), pose(col)
            sz = [float(v) for v in col.find("geometry/box/size").text.split()]
            c, s = math.cos(mp[5]), math.sin(mp[5])
            lx, ly, lyaw = mp[0] + c * lp[0] - s * lp[1], mp[1] + s * lp[0] + c * lp[1], mp[5] + lp[5]
            c2, s2 = math.cos(lyaw), math.sin(lyaw)
            boxes.append((lx + c2 * cp[0] - s2 * cp[1], ly + s2 * cp[0] + c2 * cp[1], lyaw + cp[5], sz[0], sz[1]))
    return boxes


def occupancy(boxes):
    n = int(2 * HALF / RES)
    xs = (np.arange(n) + 0.5) * RES - HALF
    X, Y = np.meshgrid(xs, xs, indexing="ij")
    blocked = np.zeros((n, n), bool)
    for (bx, by, yaw, sx, sy) in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = X - bx, Y - by
        u, v = c * dx + s * dy, -s * dx + c * dy
        blocked |= (np.abs(u) <= sx / 2 + CLEARANCE) & (np.abs(v) <= sy / 2 + CLEARANCE)
    edge = (np.abs(X) > HALF - CLEARANCE) | (np.abs(Y) > HALF - CLEARANCE)
    return blocked | edge


def to_cell(x, y):
    return int((x + HALF) / RES), int((y + HALF) / RES)


def to_xy(i, j):
    return (i + 0.5) * RES - HALF, (j + 0.5) * RES - HALF


def astar(blocked, start, goal):
    n = blocked.shape[0]
    openq = [(0.0, start)]
    came, g = {start: None}, {start: 0.0}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == goal:
            path = []
            while cur:
                path.append(cur)
                cur = came[cur]
            return path[::-1]
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == dj == 0:
                    continue
                nb = (cur[0] + di, cur[1] + dj)
                if not (0 <= nb[0] < n and 0 <= nb[1] < n) or blocked[nb]:
                    continue
                cost = g[cur] + math.hypot(di, dj)
                if cost < g.get(nb, 1e18):
                    g[nb], came[nb] = cost, cur
                    heapq.heappush(openq, (cost + math.hypot(nb[0] - goal[0], nb[1] - goal[1]), nb))
    return None


def reachable(blocked, start):
    seen = np.zeros_like(blocked)
    stack = [start]
    seen[start] = True
    while stack:
        i, j = stack.pop()
        for nb in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= nb[0] < blocked.shape[0] and 0 <= nb[1] < blocked.shape[1] and not blocked[nb] and not seen[nb]:
                seen[nb] = True
                stack.append(nb)
    return seen


def coverage_waypoints(blocked, start):
    reach = reachable(blocked, start)
    pts = []
    for x in np.arange(-HALF + 1.5, HALF - 1.0, SPACING):
        for y in np.arange(-HALF + 1.5, HALF - 1.0, SPACING):
            i, j = to_cell(x, y)
            # nearest reachable cell within 1.2 m
            best = None
            for di in range(-12, 13):
                for dj in range(-12, 13):
                    a, b = i + di, j + dj
                    if 0 <= a < reach.shape[0] and 0 <= b < reach.shape[1] and reach[a, b]:
                        d = di * di + dj * dj
                        if best is None or d < best[0]:
                            best = (d, (a, b))
            if best:
                pts.append(best[1])
    # nearest-neighbour tour from the start
    tour, cur, left = [], start, list(dict.fromkeys(pts))
    while left:
        nxt = min(left, key=lambda p: (p[0] - cur[0]) ** 2 + (p[1] - cur[1]) ** 2)
        left.remove(nxt)
        tour.append(nxt)
        cur = nxt
    return tour


class Driver(Node):
    def __init__(self):
        super().__init__("explore_route")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan = None
        self.create_subscription(LaserScan, "scan", lambda m: setattr(self, "scan", m), qos_profile_sensor_data)
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def front_clear(self, cone=0.5):
        if self.scan is None:
            return 10.0
        s = self.scan
        best = 10.0
        for k, r in enumerate(s.ranges):
            a = s.angle_min + k * s.angle_increment
            # rplidar frame is rotated +90 deg about z on the TurtleBot 4 Lite
            a = math.remainder(a + math.pi / 2, 2 * math.pi)
            if abs(a) < cone and math.isfinite(r):
                best = min(best, r)
        return best

    def follow(self, path_xy, deadline):
        idx = 0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is None:
                continue
            x, y, yaw = p
            # advance along the path to a lookahead point 0.6 m away
            while idx < len(path_xy) - 1 and math.hypot(path_xy[idx][0] - x, path_xy[idx][1] - y) < 0.6:
                idx += 1
            tx, ty = path_xy[idx]
            dist = math.hypot(tx - x, ty - y)
            if idx == len(path_xy) - 1 and dist < 0.25:
                self.pub.publish(Twist())
                return True
            err = math.remainder(math.atan2(ty - y, tx - x) - yaw, 2 * math.pi)
            cmd = Twist()
            if abs(err) > 0.7:
                cmd.angular.z = 0.9 * math.copysign(1, err)
            else:
                cmd.linear.x = 0.3 if self.front_clear() > 0.35 else 0.0
                cmd.angular.z = max(-1.0, min(1.0, 1.8 * err))
            self.pub.publish(cmd)
        self.pub.publish(Twist())
        return False


def main():
    minutes = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    sdf = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        get_package_share_directory("turtlebot4_bai04"), "worlds", "maze.sdf")
    blocked = occupancy(wall_boxes(sdf))
    rclpy.init()
    node = Driver()
    t0 = time.monotonic()
    while node.pose() is None and time.monotonic() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.pose() is None:
        print("no map->base_link transform (is SLAM Toolbox running? slam:=true)")
        sys.exit(1)
    x, y, _ = node.pose()
    start = to_cell(x, y)
    tour = coverage_waypoints(blocked, start)
    print(f"{len(tour)} coverage waypoints", flush=True)
    deadline = time.monotonic() + minutes * 60
    cur = start
    for k, goal in enumerate(tour):
        if time.monotonic() > deadline:
            break
        p = node.pose()
        cur = to_cell(p[0], p[1]) if p else cur
        if blocked[cur]:  # inflated cell (robot hugging a wall): plan from nearest free cell
            free = np.argwhere(~blocked)
            cur = tuple(free[np.argmin(((free - np.array(cur)) ** 2).sum(1))])
        path = astar(blocked, cur, goal)
        if not path:
            print(f"  [{k+1}/{len(tour)}] no path to {to_xy(*goal)}", flush=True)
            continue
        ok = node.follow([to_xy(*c) for c in path[::3]] + [to_xy(*goal)], deadline)
        gx, gy = to_xy(*goal)
        print(f"  [{k+1}/{len(tour)}] waypoint ({gx:+.1f},{gy:+.1f}) {'reached' if ok else 'timeout'}"
              f"  t={time.monotonic()-t0:5.0f}s", flush=True)
    node.pub.publish(Twist())
    print("done", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
