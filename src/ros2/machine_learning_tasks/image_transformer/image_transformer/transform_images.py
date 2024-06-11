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
        self.get_logger().info('Map recieved')
        self.transform_point_on_image()
    
    def listener_callback_feuerplan(self, msg):
        self.feuerplan_msg = msg
        self.get_logger().info('Feuerplan recieved')
        self.transform_point_on_image()

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
    
    def get_three_best_matches(self, confidence_values, matches, keypoints1, keypoints2):
        _, top_indices = torch.topk(confidence_values, k=3)
        top_matches = matches[top_indices]
        points1, points2 = keypoints1[top_matches[..., 0]], keypoints2[top_matches[..., 1]]
        return points1, points2
    
    def create_transform_matrix(self, image1, image2):
            # Initialize feature extractor and matcher
            device = "cpu" 
            extractor = SuperPoint(max_num_keypoints = 2048).eval().to(device)
            matcher = LightGlue(features = "superpoint").eval().to(device)
            # Extract features from images
            self.get_logger().info(f'Extracting features...')
            feats1 = extractor.extract(numpy_image_to_torch(image1))
            feats2 = extractor.extract(numpy_image_to_torch(image2))
            # Match features between images
            self.get_logger().info(f'Obtaining matches...')
            matches12 = matcher({'image0': feats1, 'image1': feats2})
            # Refine features and matches
            feats1, feats2, matches12 = [rbd(x) for x in [feats1, feats2, matches12]]
            # Get the three best matches according to confidence values
            points1, points2 = self.get_three_best_matches(matches12['scores'], matches12['matches'], feats1["keypoints"], feats2["keypoints"])
            # Compute the transformation matrix
            transformation_matrix = cv.getAffineTransform(np.round(points1.numpy()), np.round(points2.numpy()))
            return transformation_matrix
    
    def transform_point_on_image(self):
        if self.map_msg and self.feuerplan_msg:
            image1 = self.ros_msg_to_image(self.map_msg)
            image2 = self.ros_msg_to_image(self.feuerplan_msg)
            transformation_matrix = self.create_transform_matrix(image1, image2)
            self.get_logger().info(f'Created transformation matrix')
    
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()