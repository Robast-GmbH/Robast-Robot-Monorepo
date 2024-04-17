#!/usr/bin/env python3

import rclpy
import torch
import numpy as np
import cv2 as cv

from rclpy.node import Node
from LightGlue.lightglue import LightGlue, SuperPoint, DISK, SIFT, ALIKED, DoGHardNet, viz2d
from LightGlue.lightglue.utils import load_image, rbd


from nav_msgs.msg import OccupancyGrid


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid
            'map',
            self.subscriber_callback,
            10)
        self.subscription  

    def subscriber_callback(self, msg):
        image1 = msg.data
        print(image1)
        

        
        


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
