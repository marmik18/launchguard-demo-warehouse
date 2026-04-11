#!/usr/bin/env python3
"""
Automated warehouse exploration for SLAM map generation.
Drives the robot through the warehouse using a reactive wall-following
strategy with /scan data. Runs for a fixed duration then exits.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

EXPLORE_DURATION = 180  # seconds
LINEAR_SPEED = 0.20
TURN_SPEED = 0.5
OBSTACLE_THRESHOLD = 0.5  # meters -- start turning when obstacle is this close
WALL_FOLLOW_DIST = 0.8    # meters -- ideal distance from right wall


class MapExplorer(Node):
    def __init__(self):
        super().__init__('map_explorer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan = None
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.1, self._tick)
        self.start_time = time.time()
        self.get_logger().info(
            f'Map explorer started. Will explore for {EXPLORE_DURATION}s.'
        )

    def _on_scan(self, msg):
        self.scan = msg

    def _get_sector_min(self, ranges, start_idx, end_idx, total):
        """Get minimum range in a sector of the scan."""
        sector = []
        for i in range(start_idx % total, end_idx % total):
            r = ranges[i % total]
            if not math.isinf(r) and r > 0.01:
                sector.append(r)
        return min(sector) if sector else float('inf')

    def _tick(self):
        elapsed = time.time() - self.start_time
        if elapsed > EXPLORE_DURATION:
            self.get_logger().info('Exploration complete.')
            msg = Twist()
            self.pub.publish(msg)
            raise SystemExit(0)

        if self.scan is None:
            return

        ranges = self.scan.ranges
        n = len(ranges)
        if n == 0:
            return

        # TB3 LiDAR: 0 degrees = front, counterclockwise
        # Front: roughly -30 to +30 degrees
        # Right: roughly -90 to -30 degrees (idx ~270 to ~330 in 360-sample scan)
        # Left: roughly +30 to +90 degrees (idx ~30 to ~90)
        samples_per_deg = n / 360.0

        front_start = int(n - 30 * samples_per_deg)
        front_end = int(30 * samples_per_deg)

        front_min = min(
            self._get_sector_min(ranges, front_start, n, n),
            self._get_sector_min(ranges, 0, front_end, n),
        )

        right_start = int(n - 90 * samples_per_deg)
        right_end = int(n - 30 * samples_per_deg)
        right_min = self._get_sector_min(ranges, right_start, right_end, n)

        left_start = int(30 * samples_per_deg)
        left_end = int(90 * samples_per_deg)
        left_min = self._get_sector_min(ranges, left_start, left_end, n)

        msg = Twist()

        if front_min < OBSTACLE_THRESHOLD:
            # Wall ahead: turn toward the more open side
            if left_min > right_min:
                msg.angular.z = TURN_SPEED
            else:
                msg.angular.z = -TURN_SPEED
            msg.linear.x = 0.05
        elif right_min < WALL_FOLLOW_DIST:
            # Right wall-following: steer slightly left to maintain distance
            msg.linear.x = LINEAR_SPEED
            msg.angular.z = 0.15
        elif right_min > WALL_FOLLOW_DIST * 1.5:
            # Lost the right wall: turn right to find it
            msg.linear.x = LINEAR_SPEED * 0.8
            msg.angular.z = -0.3
        else:
            # Good distance from right wall: drive straight
            msg.linear.x = LINEAR_SPEED
            msg.angular.z = 0.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MapExplorer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
