"""Integration tests for the keyboard_controller."""


import ament_index_python.packages as ament
import launch
import launch.launch_description_sources as description_sources
import launch_testing
import rclpy
import unittest


NODE = "test_keyboard_controller"
PACKAGE = "ezrassor_keyboard_controller"
KEYBOARD_CONTROLLER_LAUNCH_FILE_FORMAT = "{0}/launch/keyboard_controller.py"


def generate_test_description():
    """Create a test description with the keyboard_controller launch file."""
    keyboard_controller_launch_file = (
        KEYBOARD_CONTROLLER_LAUNCH_FILE_FORMAT.format(
            ament.get_package_share_directory(PACKAGE),
        )
    )

    return launch.LaunchDescription(
        [
            launch.actions.IncludeLaunchDescription(
                description_sources.PythonLaunchDescriptionSource(
                    keyboard_controller_launch_file,
                ),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class KeyboardControllerIntegrationTests(unittest.TestCase):
    """A suite of integration tests for the keyboard_controller."""

    @classmethod
    def setUpClass(*arguments):
        """Initialize ROS before testing begins.

        This method name is required by unittest.
        """
        rclpy.init()

    def setUp(self):
        """Initialize testing infrastructure before each test.

        This method name is required by unittest.
        """
        self._node = rclpy.create_node(NODE)

    def tearDown(self):
        """Destroy testing infrastructure after each test.

        This method name is required by unittest.
        """
        self._node.destroy_node()

    @classmethod
    def tearDownClass(*arguments):
        """Shut down ROS after testing is complete.

        This method name is required by unittest.
        """
        rclpy.shutdown()

    def test_sample_test(self):
        """Will replace this test."""
        self.assertEqual(1, 1)
