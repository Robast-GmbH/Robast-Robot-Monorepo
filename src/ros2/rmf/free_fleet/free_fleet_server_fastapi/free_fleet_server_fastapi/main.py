from . import rest_interface
from . import free_fleet_controller
import rclpy
import threading


def main(args=None):
    rclpy.init()
    controller = free_fleet_controller.free_fleet_controller()
    web_interface = rest_interface.RestInterface(controller)
    web_interface.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()