#!/usr/bin/env python3

import rclpy
from feuerplan_publisher.module_to_import import MinimalSubscriber   
        
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
