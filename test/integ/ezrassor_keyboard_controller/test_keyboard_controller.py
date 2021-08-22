"""Integration tests for the keyboard_controller."""
import ament_index_python.packages as ament
import concurrent.futures
import ezrassor_keyboard_controller as keyboard
import ezrassor_keyboard_interfaces.msg
import geometry_msgs.msg
import launch
import launch.launch_description_sources as description_sources
import launch_testing
import rclpy
import std_msgs.msg
import time
import unittest


NODE = "test_keyboard_controller"
PACKAGE = "ezrassor_keyboard_controller"
KEYBOARD_CONTROLLER_LAUNCH_FILE_FORMAT = "{0}/launch/keyboard_controller.py"
WHEEL_ACTIONS_TOPIC = "wheel_actions"
FRONT_ARM_ACTIONS_TOPIC = "front_arm_actions"
BACK_ARM_ACTIONS_TOPIC = "back_arm_actions"
FRONT_DRUM_ACTIONS_TOPIC = "front_drum_actions"
BACK_DRUM_ACTIONS_TOPIC = "back_drum_actions"
ROUTINE_ACTIONS_TOPIC = "routine_actions"
KEY_STATES_TOPIC = "key_states"
QUEUE_SIZE = 20
TIMEOUT = 2.0


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

        # Save the output of each keyboard_controller topic to a list.
        self._wheel_actions = []
        self._node.create_subscription(
            geometry_msgs.msg.Twist,
            WHEEL_ACTIONS_TOPIC,
            lambda message: self._wheel_actions.append(
                (message.linear.x, message.angular.z),
            ),
            QUEUE_SIZE,
        )
        self._front_arm_actions = []
        self._node.create_subscription(
            std_msgs.msg.Float32,
            FRONT_ARM_ACTIONS_TOPIC,
            lambda message: self._front_arm_actions.append(message.data),
            QUEUE_SIZE,
        )
        self._back_arm_actions = []
        self._node.create_subscription(
            std_msgs.msg.Float32,
            BACK_ARM_ACTIONS_TOPIC,
            lambda message: self._back_arm_actions.append(message.data),
            QUEUE_SIZE,
        )
        self._front_drum_actions = []
        self._node.create_subscription(
            std_msgs.msg.Float32,
            FRONT_DRUM_ACTIONS_TOPIC,
            lambda message: self._front_drum_actions.append(message.data),
            QUEUE_SIZE,
        )
        self._back_drum_actions = []
        self._node.create_subscription(
            std_msgs.msg.Float32,
            BACK_DRUM_ACTIONS_TOPIC,
            lambda message: self._back_drum_actions.append(message.data),
            QUEUE_SIZE,
        )
        self._routine_actions = []
        self._node.create_subscription(
            std_msgs.msg.Int8,
            ROUTINE_ACTIONS_TOPIC,
            lambda message: self._routine_actions.append(message.data),
            QUEUE_SIZE,
        )

        self._key_states_publisher = self._node.create_publisher(
            ezrassor_keyboard_interfaces.msg.KeyState,
            KEY_STATES_TOPIC,
            QUEUE_SIZE,
        )

        # Sleep for some time to give ROS a moment to warm up.
        time.sleep(TIMEOUT)

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

    def _press(self, key):
        """Press a key."""
        self._publish(key, True)

    def _release(self, key):
        """Release a key."""
        self._publish(key, False)

    def _publish(self, key, pressed):
        """Publish a new key state to the key states topic."""
        message = ezrassor_keyboard_interfaces.msg.KeyState()
        message.key = key.value
        message.pressed = pressed
        self._key_states_publisher.publish(message)

    def _spin_for_subscribers(self):
        """Spin the node for a brief period of time.

        This is necessary to give the testing subscribers enough spins to
        collect all required outputs.
        """
        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
            future = executor.submit(time.sleep, TIMEOUT)
            rclpy.spin_until_future_complete(
                self._node,
                future,
                timeout_sec=TIMEOUT,
            )

    def _verify_wheel_actions(self, expected_actions):
        """Verify the received wheel actions are as expected."""
        self.assertEqual(self._wheel_actions, expected_actions)

    def _verify_front_arm_actions(self, expected_actions):
        """Verify the received front arm actions are as expected."""
        self.assertEqual(self._front_arm_actions, expected_actions)

    def _verify_back_arm_actions(self, expected_actions):
        """Verify the received back arm actions are as expected."""
        self.assertEqual(self._back_arm_actions, expected_actions)

    def _verify_front_drum_actions(self, expected_actions):
        """Verify the received front drum actions are as expected."""
        self.assertEqual(self._front_drum_actions, expected_actions)

    def _verify_back_drum_actions(self, expected_actions):
        """Verify the received back drum actions are as expected."""
        self.assertEqual(self._back_drum_actions, expected_actions)

    def _verify_routine_actions(self, expected_actions):
        """Verify the received routine actions are as expected."""
        self.assertEqual(self._routine_actions, expected_actions)

    def test_keyboard_controller_produces_accurate_wheel_actions(self):
        """Should produce wheel actions when given wheel-related messages."""
        self._press(keyboard.Key.MOVE_FORWARD)
        self._release(keyboard.Key.MOVE_FORWARD)
        self._press(keyboard.Key.TURN_LEFT)
        self._release(keyboard.Key.TURN_LEFT)
        self._press(keyboard.Key.MOVE_BACKWARD)
        self._release(keyboard.Key.MOVE_BACKWARD)
        self._press(keyboard.Key.TURN_RIGHT)
        self._release(keyboard.Key.TURN_RIGHT)

        self._spin_for_subscribers()

        self._verify_wheel_actions(
            [
                (1.0, 0.0),
                (0.0, 0.0),
                (0.0, 1.0),
                (0.0, 0.0),
                (-1.0, 0.0),
                (0.0, 0.0),
                (0.0, -1.0),
                (0.0, 0.0),
            ],
        )
        self._verify_front_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_front_drum_actions(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self._verify_back_drum_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_routine_actions([])

    def test_keyboard_controller_produces_accurate_arm_actions(self):
        """Should produce arm actions when given arm-related messages."""
        self._press(keyboard.Key.RAISE_FRONT_ARM)
        self._release(keyboard.Key.RAISE_FRONT_ARM)
        self._press(keyboard.Key.LOWER_FRONT_ARM)
        self._release(keyboard.Key.LOWER_FRONT_ARM)
        self._press(keyboard.Key.RAISE_BACK_ARM)
        self._release(keyboard.Key.RAISE_BACK_ARM)
        self._press(keyboard.Key.LOWER_BACK_ARM)
        self._release(keyboard.Key.LOWER_BACK_ARM)

        self._spin_for_subscribers()

        self._verify_wheel_actions(
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ],
        )
        self._verify_front_arm_actions(
            [1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self._verify_back_arm_actions([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0])
        self._verify_front_drum_actions(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self._verify_back_drum_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_routine_actions([])

    def test_keyboard_controller_produces_accurate_drum_actions(self):
        """Should produce drum actions when given drum-related messages."""
        self._press(keyboard.Key.DIG_FRONT_DRUM)
        self._release(keyboard.Key.DIG_FRONT_DRUM)
        self._press(keyboard.Key.DUMP_FRONT_DRUM)
        self._release(keyboard.Key.DUMP_FRONT_DRUM)
        self._press(keyboard.Key.DIG_BACK_DRUM)
        self._release(keyboard.Key.DIG_BACK_DRUM)
        self._press(keyboard.Key.DUMP_BACK_DRUM)
        self._release(keyboard.Key.DUMP_BACK_DRUM)

        self._spin_for_subscribers()

        self._verify_wheel_actions(
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ],
        )
        self._verify_front_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_back_arm_actions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._verify_front_drum_actions(
            [1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self._verify_back_drum_actions(
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0]
        )
        self._verify_routine_actions([])

    def test_keyboard_controller_produces_accurate_routine_actions(self):
        """Should produce routine actions when given routine-related messages."""
        self._press(keyboard.Key.AUTO_DRIVE_ROUTINE)
        self._release(keyboard.Key.AUTO_DRIVE_ROUTINE)
        self._press(keyboard.Key.AUTO_DIG_ROUTINE)
        self._release(keyboard.Key.AUTO_DIG_ROUTINE)
        self._press(keyboard.Key.AUTO_DUMP_ROUTINE)
        self._release(keyboard.Key.AUTO_DUMP_ROUTINE)
        self._press(keyboard.Key.AUTO_DOCK_ROUTINE)
        self._release(keyboard.Key.AUTO_DOCK_ROUTINE)
        self._press(keyboard.Key.FULL_AUTONOMY_ROUTINE)
        self._release(keyboard.Key.FULL_AUTONOMY_ROUTINE)
        self._press(keyboard.Key.STOP_ROUTINE)
        self._release(keyboard.Key.STOP_ROUTINE)

        self._spin_for_subscribers()

        self._verify_wheel_actions(
            [
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ],
        )
        self._verify_front_arm_actions(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self._verify_back_arm_actions(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self._verify_front_drum_actions(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self._verify_back_drum_actions(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self._verify_routine_actions(
            [
                0b000001,
                0b000010,
                0b000100,
                0b001000,
                0b010000,
                0b100000,
            ],
        )
