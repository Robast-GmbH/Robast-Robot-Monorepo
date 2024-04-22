#!/usr/bin/env python3

import rclpy
#import torch
import numpy as np
import cv2 as cv

from rclpy.node import Node
from lightglue.lightglue import LightGlue
from lightglue.superpoint import SuperPoint
#from LightGlue.lightglue.utils import load_image, rbd


from nav_msgs.msg import OccupancyGrid

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'map',
            self.callback_map,
            10)
        self.subscription_map

        self.subscription_feuerplan = self.create_subscription(
            OccupancyGrid,
            'global_costmap/feuerplan_image_topic',
            self.callback_feuerplan,
            10)
        self.subscription_feuerplan

    def callback_map(self, msg):
        self.map_msg = msg.data

    def callback_feuerplan(self,msg):
        self.feuerplan_msg = msg.data

    def create_transformation(self):

        #image1 = self.map_msg
        image2 = self.feuerplan_msg
        print(image2)
        #device = "cpu"
        #extractor = SuperPoint(max_num_keypoints = 2048).eval().to(device)
        #matcher = LightGlue(features = "superpoint").eval().to(device)

        """ feats1 = extractor.extract(image1.to(device))
        feats2 = extractor.extract(image2.to(device))
        matches1 = matcher({"image1":feats1, "image2":feats2})
        feats1, feats2, matches1 = [
            rbd(x) for x in [feats1, feats2, matches1]
        ]
        kpts1, kpts2, matches1 = feats1["keypoints"], feats2["keypoints"], matches1["matches"]
        m_kpts1, mkpts1 = kpts1[matches1[...,0]], kpts2[matches1[...,1]] """

    
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
