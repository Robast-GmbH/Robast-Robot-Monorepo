from . import rest_interface
from . import ros_controller
from . import ff_controller
import rclpy
import threading


def main(args=None):
    rclpy.init()
    ros_node = ros_controller.ros_controller("")
    free_fleet= ff_controller.free_fleet_controller()
    web_interface = rest_interface.RestInterface(ros_node, free_fleet, 0.5)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,))
    spin_thread.start()
    web_interface.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
