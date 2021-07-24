"""Execute a ROS node using the ezrassor_keyboard_controller module.

This node listens for KeyState messages and translates them into commands for
the EZRASSOR.
"""
import ezrassor_keyboard_controller as keyboard
import ezrassor_keyboard_interfaces.msg
import functools
import geometry_msgs.msg
import rclpy
import std_msgs.msg


NODE = "keyboard_controller"
WHEEL_ACTIONS_TOPIC = "wheel_actions"
FRONT_ARM_ACTIONS_TOPIC = "front_arm_actions"
BACK_ARM_ACTIONS_TOPIC = "back_arm_actions"
FRONT_DRUM_ACTIONS_TOPIC = "front_drum_actions"
BACK_DRUM_ACTIONS_TOPIC = "back_drum_actions"
ROUTINE_ACTIONS_TOPIC = "routine_actions"
KEY_STATES_TOPIC = "key_states"
QUEUE_SIZE = 10


def main(passed_args=None):
    """Main entry point for the ROS node."""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        # Create publishers for each type of action.
        wheel_actions_publisher = node.create_publisher(
            geometry_msgs.msg.Twist,
            WHEEL_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        front_arm_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            FRONT_ARM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        back_arm_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            BACK_ARM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        back_front_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            FRONT_DRUM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        back_drum_actions_publisher = node.create_publisher(
            std_msgs.msg.Float32,
            BACK_DRUM_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )
        routine_actions_publisher = node.create_publisher(
            std_msgs.msg.Int8,
            ROUTINE_ACTIONS_TOPIC,
            QUEUE_SIZE,
        )

        def process_message(board, message):
            """Callback to create and publish a command from a message."""

            # Update the board based on the KeyState message.
            if message.pressed:
                board.press(message.key)
            else:
                board.release(message.key)

            # Create a command from the existing board.
            command = keyboard.create_command(board)

            if command.wheel_action is not None:
                wheel_action = geometry_msgs.msg.Twist()
                wheel_action.linear.x = command.wheel_action.linear_x
                wheel_action.angular.z = command.wheel_action.angular_z
                wheel_actions_publisher.publish(wheel_action)

            if command.front_arm_action is not None:
                front_arm_action = std_msgs.msg.Float32()
                front_arm_action.data = command.front_arm_action.value
                front_arm_actions_publisher.publish(front_arm_action)

            if command.back_arm_action is not None:
                back_arm_action = std_msgs.msg.Float32()
                back_arm_action.data = command.back_arm_action.value
                back_arm_actions_publisher.publish(back_arm_action)

            if command.front_drum_action is not None:
                front_drum_action = std_msgs.msg.Float32()
                front_drum_action.data = command.front_drum_action.value
                back_front_actions_publisher.publish(front_drum_action)

            if command.back_drum_action is not None:
                back_drum_action = std_msgs.msg.Float32()
                back_drum_action.data = command.back_drum_action.value
                back_drum_actions_publisher.publish(back_drum_action)

            if command.routine_action is not None:
                routine_action = std_msgs.msg.Int8()
                routine_action.data = command.routine_action.value
                routine_actions_publisher.publish(routine_action)

        # Process messages from the KeyStates topic.
        node.create_subscription(
            ezrassor_keyboard_interfaces.msg.KeyState,
            KEY_STATES_TOPIC,
            functools.partial(process_message, keyboard.Board()),
            QUEUE_SIZE,
        )

        # Spin!
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
