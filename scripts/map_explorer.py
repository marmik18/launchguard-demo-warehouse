#!/usr/bin/env python3
"""
Scripted warehouse exploration for SLAM map generation.
Drives a pre-planned route with scan-based wall detection
so the robot stops before hitting obstacles.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

LINEAR = 0.20
TURN = 0.5
WALL_DIST = 0.35

# (type, arg1, arg2)
# ("drive", speed, seconds)  -- drive forward, stop if wall detected
# ("turn", angular_vel, seconds)  -- turn in place
MOVES = [
    # From spawn (0,-2) facing +x, sweep right side
    ("drive", LINEAR, 20),
    ("turn", TURN, 3.14),      # face +y
    ("drive", LINEAR, 25),
    ("turn", -TURN, 3.14),     # face +x again
    ("drive", LINEAR, 15),
    ("turn", TURN, 3.14),      # face +y
    ("drive", LINEAR, 20),
    ("turn", TURN, 3.14),      # face -x
    ("drive", LINEAR, 35),
    ("turn", TURN, 3.14),      # face -y
    ("drive", LINEAR, 25),
    ("turn", TURN, 3.14),      # face +x
    ("drive", LINEAR, 35),
    ("turn", -TURN, 3.14),     # face -y
    ("drive", LINEAR, 25),
    ("turn", -TURN, 3.14),     # face -x
    ("drive", LINEAR, 30),
    ("turn", -TURN, 3.14),     # face +y
    ("drive", LINEAR, 30),
    # Final spin to capture surroundings
    ("turn", TURN, 12.56),
]


class MapExplorer(Node):
    def __init__(self):
        super().__init__('map_explorer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_dist = float('inf')
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
        )

    def _on_scan(self, msg):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return
        samples_per_deg = n / 360.0
        front_start = int(n - 20 * samples_per_deg)
        front_end = int(20 * samples_per_deg)
        front_vals = []
        for i in list(range(front_start, n)) + list(range(0, front_end)):
            r = ranges[i]
            if not math.isinf(r) and r > 0.01:
                front_vals.append(r)
        self.front_dist = min(front_vals) if front_vals else float('inf')

    def run(self):
        # Wait for scan data
        print("Waiting for scan data...", flush=True)
        timeout = time.time() + 30
        while self.front_dist == float('inf') and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        print(f"Scan ready. Front distance: {self.front_dist:.2f}m", flush=True)

        for i, move in enumerate(MOVES):
            mtype = move[0]
            print(f"Move {i+1}/{len(MOVES)}: {move}", flush=True)

            msg = Twist()
            if mtype == "turn":
                msg.angular.z = move[1]
                end = time.time() + move[2]
                while time.time() < end:
                    self.pub.publish(msg)
                    rclpy.spin_once(self, timeout_sec=0.05)
            elif mtype == "drive":
                msg.linear.x = move[1]
                end = time.time() + move[2]
                while time.time() < end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    if self.front_dist < WALL_DIST:
                        print(f"  Wall detected at {self.front_dist:.2f}m, stopping early", flush=True)
                        break
                    self.pub.publish(msg)

            # Brief stop between moves
            stop = Twist()
            self.pub.publish(stop)
            time.sleep(0.5)

        print("Exploration complete.", flush=True)


def main():
    rclpy.init()
    node = MapExplorer()
    try:
        node.run()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
