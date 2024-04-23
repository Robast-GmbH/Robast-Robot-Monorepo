import rclpy
import torch
import numpy as np
import cv2 as cv

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from lightglue.lightglue import LightGlue
from lightglue.superpoint import SuperPoint


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_feuerplan = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/feuerplan_image_topic',
            self.listener_callback_feuerplan,
            qos_profile = self.get_qos_profile())
        self.subscription_feuerplan  # prevent unused variable warning

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback_map,
            10)
        self.subscription_map  # prevent unused variable warning

        self.map_msg = self.feuerplan_msg = None
    
    def get_qos_profile(self):
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos_profile    

    def listener_callback_map(self, msg):
        self.map_msg = msg.data
        self.get_logger().info('I heard map ')
        self.create_transform()
    
    def listener_callback_feuerplan(self, msg):
        self.feuerplan_msg = msg.data
        self.get_logger().info('I heard feuerplan ')
        self.create_transform()

    def create_transform(self):

        if self.map_msg and self.feuerplan_msg is not None:
            image1 = torch.from_numpy(np.array(self.map_msg))
            image2 = torch.from_numpy(np.array(self.feuerplan_msg))

            device = "cpu" 
            print(image1.type())
            extractor = SuperPoint(max_num_keypoints = 2048).eval().to(device)
            matcher = LightGlue(features = "superpoint").eval().to(device)

            #feats1 = extractor.extract(image1)
            #feats2 = extractor.extract(image2)

            print(image1.shape)

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
