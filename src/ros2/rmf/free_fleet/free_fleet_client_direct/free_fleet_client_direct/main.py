import rclpy
from  . import client_direct as ffc


def main(args=None):

    rclpy.init(args=args)
    ff_client = ffc.free_fleet_client_direct()
    rclpy.spin(ff_client)
    ff_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()