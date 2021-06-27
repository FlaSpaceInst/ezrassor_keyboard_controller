""""""
import rclpy


NODE = "keyboard_controller"


def main(passed_args=None):
    """"""
    try:
        rclpy.init(args=passed_args)
        node = rclpy.create_node(NODE)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
