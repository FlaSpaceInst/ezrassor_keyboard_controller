"""Launch a keyboard_controller node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create a keyboard_controller node with a launch description."""
    keyboard_controller_node = Node(
        package="ezrassor_keyboard_controller",
        executable="keyboard_controller",
        output={"both": "screen"},
    )

    return LaunchDescription([keyboard_controller_node])
