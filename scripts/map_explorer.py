#!/usr/bin/env python3
"""
Scripted warehouse exploration for SLAM map generation.
Drives a pre-planned route through the warehouse based on known
world-file coordinates to build a complete map.
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR = 0.22
TURN = 0.5

# (linear_x, angular_z, duration_seconds)
# Route: start at (0,-2), sweep the warehouse in a lawnmower pattern
MOVES = [
    # Drive forward (+x) toward right shelves
    (LINEAR, 0.0, 25),
    # Turn left 90 (~face +y)
    (0.0, TURN, 3.14),
    # Drive up along right side
    (LINEAR, 0.0, 30),
    # Turn left 90 (~face -x)
    (0.0, TURN, 3.14),
    # Drive across to left side
    (LINEAR, 0.0, 40),
    # Turn left 90 (~face -y)
    (0.0, TURN, 3.14),
    # Drive down along left side
    (LINEAR, 0.0, 35),
    # Turn left 90 (~face +x)
    (0.0, TURN, 3.14),
    # Drive back toward center
    (LINEAR, 0.0, 20),
    # Turn right 90 (~face -y)
    (0.0, -TURN, 3.14),
    # Drive down through center
    (LINEAR, 0.0, 35),
    # Turn right 90 (~face -x)
    (0.0, -TURN, 3.14),
    # Drive to left side
    (LINEAR, 0.0, 25),
    # Turn right 90 (~face +y)
    (0.0, -TURN, 3.14),
    # Drive up again
    (LINEAR, 0.0, 30),
    # Turn right 90 (~face +x)
    (0.0, -TURN, 3.14),
    # Drive back to center-right
    (LINEAR, 0.0, 30),
    # Spin in place to capture surroundings
    (0.0, TURN, 12.56),
]


class MapExplorer(Node):
    def __init__(self):
        super().__init__('map_explorer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Map explorer started.')

    def run(self):
        for i, (lin, ang, dur) in enumerate(MOVES):
            self.get_logger().info(
                f'Move {i+1}/{len(MOVES)}: lin={lin}, ang={ang}, dur={dur:.1f}s'
            )
            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
            end = time.time() + dur
            while time.time() < end:
                self.pub.publish(msg)
                time.sleep(0.1)

        stop = Twist()
        self.pub.publish(stop)
        self.get_logger().info('Exploration complete.')


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
