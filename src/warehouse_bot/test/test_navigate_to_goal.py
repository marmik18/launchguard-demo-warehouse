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
from std_msgs.msg import String

TIMEOUT = 180


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


class TestNavigateToGoal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_navigate_to_goal')

    def tearDown(self):
        self.node.destroy_node()

    def test_robot_reaches_goal(self, proc_output):
        statuses = []

        def _on_status(msg):
            statuses.append(msg.data)

        sub = self.node.create_subscription(
            String, '/warehouse_nav/status', _on_status, 10
        )
        try:
            end = time.time() + TIMEOUT
            while time.time() < end:
                if 'reached_goal' in statuses:
                    break
                rclpy.spin_once(self.node, timeout_sec=1)

            self.assertNotIn(
                'navigation_failed', statuses,
                'Navigation reported failure',
            )
            self.assertIn(
                'reached_goal', statuses,
                f'Robot did not reach final goal within {TIMEOUT}s. '
                f'Last status: {statuses[-1] if statuses else "none"}',
            )
        finally:
            self.node.destroy_subscription(sub)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
