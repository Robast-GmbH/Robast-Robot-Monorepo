from . import rest_interface
from . import ros_controller

import rclpy
import threading


def main(args=None):
    rclpy.init()
    ros_node = ros_controller.ros_controller("http://127.0.0.1:3001")
    web_interface = rest_interface.RestInterface(ros_node, 0.5)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,))
    spin_thread.start()
    web_interface.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
