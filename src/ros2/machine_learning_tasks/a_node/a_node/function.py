import rclpy
import torch
import numpy as np
import cv2 as cv

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from lightglue.lightglue import LightGlue
from lightglue.utils import numpy_image_to_torch, rbd
from lightglue.superpoint import SuperPoint


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_feuerplan = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/feuerplan_image_topic',
            self.listener_callback_feuerplan,
            qos_profile = self.get_qos_profile())
        self.subscription_feuerplan  

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback_map,
            10)
        self.subscription_map  

        self.map_msg = self.feuerplan_msg = None
    
    def get_qos_profile(self):
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos_profile    

    def listener_callback_map(self, msg):
        self.map_msg = msg
        self.get_logger().info('I heard map ')
        self.create_transform()
    
    def listener_callback_feuerplan(self, msg):
        self.feuerplan_msg = msg
        self.get_logger().info('I heard feuerplan ')
        self.create_transform()

    def ros_msg_to_image(self, ros_msg):
        image = np.zeros((ros_msg.info.height,ros_msg.info.width), dtype=np.uint8)
        data = ros_msg.data
        index = 0
        for y in range(ros_msg.info.height):
            for x in range(ros_msg.info.width):
                image[y,x] = data[index]

                if data[index] == 0 or data[index] == -1:
                    image[y,x] = 255
                else:
                    image[y,x] = data[index]
                index += 1
        return image

    def create_transform(self):
        if self.map_msg and self.feuerplan_msg is not None:
            
            image0 = self.ros_msg_to_image(self.map_msg)
            image1 = self.ros_msg_to_image(self.feuerplan_msg)

            device = "cpu" 
            extractor = SuperPoint(max_num_keypoints = 2048).eval().to(device)
            matcher = LightGlue(features = "superpoint").eval().to(device)

            feats1 = extractor.extract(numpy_image_to_torch(image0))
            feats2 = extractor.extract(numpy_image_to_torch(image1))

            matches12 = matcher({'image0': feats1, 'image1': feats2})
            feats1, feats2, matches12 = [rbd(x) for x in [feats1, feats2, matches12]]
            matches = matches12['matches']
            points1 = feats1['keypoints'][matches[..., 0]]
            points2 = feats2['keypoints'][matches[..., 1]]

            transformation_matrix, _ = cv.estimateAffinePartial2D(points1.numpy(), points2.numpy())

            
            map_origin = np.array([self.map_msg.info.origin.position.x,self.map_msg.info.origin.position.y,self.map_msg.info.origin.position.z])

            feuerplan_origin = cv.warpAffine(map_origin.reshape((1,3)),transformation_matrix,(1,3))

            print(feuerplan_origin)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
