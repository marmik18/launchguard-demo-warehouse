#!/usr/bin/env python3
"""
Nav2-based warehouse exploration for SLAM map generation.
Uses BasicNavigator to send goals to known open-space coordinates
from the warehouse world file. SLAM Toolbox builds the map as
the robot navigates.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Known open-space coordinates from the warehouse .world file (world frame).
# Spread across the warehouse to maximize map coverage.
GOALS = [
    (2.0, -2.0, 0.0),
    (2.0, 2.0, 1.57),
    (-1.0, 4.0, 3.14),
    (-4.0, 2.0, -1.57),
    (-4.0, -2.0, 0.0),
    (-4.0, -5.0, -1.57),
    (-1.0, -6.0, 0.0),
    (2.0, -5.0, 1.57),
    (2.0, 0.0, 1.57),
    (0.0, 6.0, 3.14),
    (-3.0, 6.0, -1.57),
    (-3.0, 0.0, 0.0),
    (0.0, -2.0, 0.0),
]


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("Waiting for Nav2 bt_navigator to become active...", flush=True)
    navigator._waitForNodeToActivate('bt_navigator')
    print("Nav2 is active. Starting exploration.", flush=True)

    # Brief pause for SLAM to initialize
    time.sleep(5)

    for i, (x, y, yaw) in enumerate(GOALS):
        print(f"Goal {i+1}/{len(GOALS)}: ({x}, {y}, yaw={yaw:.2f})", flush=True)
        goal = make_pose(navigator, x, y, yaw)
        navigator.goToPose(goal)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                eta = feedback.estimated_time_remaining.sec
                dist = feedback.distance_remaining
                print(f"  ETA: {eta}s, dist: {dist:.2f}m", flush=True)
            time.sleep(2.0)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"  Reached goal {i+1}.", flush=True)
        elif result == TaskResult.CANCELED:
            print(f"  Goal {i+1} canceled.", flush=True)
        else:
            print(f"  Goal {i+1} failed. Skipping.", flush=True)

        time.sleep(1.0)

    print("Exploration complete.", flush=True)
    navigator.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
