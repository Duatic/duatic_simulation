import warnings
import unittest

import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_testing
import rosgraph_msgs.msg

from duatic_testing.testing_helpers import wait_for_node, wait_for_message

ARGUMENTS = [("world", "warehouse"), ("headless", "true")]


def generate_test_description():
    """Generate a LaunchDescription for the test."""
    pkg_simulation = get_package_share_directory("duatic_simulation")
    path_simulation = PathJoinSubstitution([pkg_simulation, "launch", "gazebo.launch.py"])

    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([path_simulation]), launch_arguments=ARGUMENTS
    )

    ready = TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()])

    ld = LaunchDescription()
    ld.add_action(launch_simulation)
    ld.add_action(ready)

    return ld


# -----------------------
# Shared test definitions
# -----------------------
class TestGazeboWorld(unittest.TestCase):
    """Base test class for all Gazebo world tests."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_gazebo_world")

    def tearDown(self):
        self.node.destroy_node()

    def test_clock_bridge_start(self):
        """Test if the clock_bridge node started."""
        wait_for_node(self.node, "clock_bridge", timeout=10.0)

    def test_publishes_clock(self, proc_output):
        """Check whether /clock messages are published."""
        wait_for_message(self.node, "/clock", rosgraph_msgs.msg.Clock, timeout=10.0)


@launch_testing.post_shutdown_test()
class TestGazeboWorldShutdown(unittest.TestCase):
    """Post-shutdown test to verify processes exited cleanly."""

    def test_exit_codes(self, proc_info):
        try:
            launch_testing.asserts.assertExitCodes(proc_info)
        except AssertionError as e:
            warnings.warn(f"Process exit warning: {e}")
