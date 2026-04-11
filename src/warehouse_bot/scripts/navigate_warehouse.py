#!/usr/bin/env python3
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Provisional world-frame coordinates (update to map-frame after map generation)
INITIAL_POSE = (0.0, 0.0, 0.0)

WAYPOINTS = [
    (3.5, 0.0, 0.0),
    (3.5, -3.0, -1.57),
    (0.0, -3.0, 3.14),
    (0.0, 0.0, 1.57),
]


def yaw_to_quaternion(yaw):
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    status_pub = navigator.create_publisher(String, '/warehouse_nav/status', 10)

    def publish_status(text):
        msg = String()
        msg.data = text
        status_pub.publish(msg)
        navigator.get_logger().info(f'Status: {text}')

    try:
        ix, iy, iyaw = INITIAL_POSE
        initial = make_pose(navigator, ix, iy, iyaw)
        navigator.setInitialPose(initial)
        navigator.get_logger().info('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()
        navigator.get_logger().info('Nav2 is active. Starting navigation.')

        for i, (wx, wy, wyaw) in enumerate(WAYPOINTS):
            publish_status(f'navigating_to_waypoint_{i}')
            goal = make_pose(navigator, wx, wy, wyaw)
            navigator.goToPose(goal)

            while not navigator.isTaskComplete():
                time.sleep(0.5)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                publish_status(f'reached_waypoint_{i}')
            else:
                publish_status('navigation_failed')
                navigator.get_logger().error(
                    f'Navigation to waypoint {i} failed with result: {result}'
                )
                sys.exit(1)

        publish_status('reached_goal')
        navigator.get_logger().info('All waypoints reached successfully.')

    finally:
        navigator.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
