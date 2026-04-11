import math
import os
import time
import unittest

import launch
import launch_testing.actions
import launch_testing.asserts
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

MIN_SAFE_DISTANCE = 0.19
TIMEOUT = 240
MIN_SAMPLES = 50


def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('warehouse_bot'),
                'launch', 'warehouse_nav.launch.py',
            )
        )
    )
    return (
        launch.LaunchDescription([
            sim_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestSafeDistance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_safe_distance')

    def tearDown(self):
        self.node.destroy_node()

    def test_minimum_clearance(self, proc_output):
        min_ranges = []
        final_status = [None]

        def _on_scan(msg):
            valid = [r for r in msg.ranges if not math.isinf(r) and r > 0.0]
            if valid:
                min_ranges.append(min(valid))

        def _on_status(msg):
            if msg.data in ('reached_goal', 'navigation_failed'):
                final_status[0] = msg.data

        scan_sub = self.node.create_subscription(
            LaserScan, '/scan', _on_scan, qos_profile_sensor_data
        )
        status_sub = self.node.create_subscription(
            String, '/warehouse_nav/status', _on_status, 10
        )
        try:
            end = time.time() + TIMEOUT
            while time.time() < end:
                if final_status[0] is not None:
                    break
                rclpy.spin_once(self.node, timeout_sec=1)

            self.assertEqual(
                final_status[0], 'reached_goal',
                f'Navigation did not complete successfully. '
                f'Final status: {final_status[0] or "timeout"}',
            )
            self.assertGreaterEqual(
                len(min_ranges), MIN_SAMPLES,
                f'Insufficient scan data received: {len(min_ranges)} samples',
            )
            observed_min = min(min_ranges)
            self.assertGreater(
                observed_min, MIN_SAFE_DISTANCE,
                f'Robot came too close to an obstacle: '
                f'min distance = {observed_min:.3f}m '
                f'(threshold: {MIN_SAFE_DISTANCE}m)',
            )
        finally:
            self.node.destroy_subscription(scan_sub)
            self.node.destroy_subscription(status_sub)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -6, -9, -15]
        )
