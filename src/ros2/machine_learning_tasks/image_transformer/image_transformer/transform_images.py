import rclpy
import torch
import numpy as np
import cv2 as cv

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from lightglue.lightglue import LightGlue
from lightglue.utils import numpy_image_to_torch, rbd
from lightglue.superpoint import SuperPoint


class FeuerplanPublisher(Node):

    def __init__(self):
        super().__init__('feuerplan_publisher')

        self.declare_parameter('feuerplan_path', '')

        self.feuerplan_path = self.get_parameter('feuerplan_path').get_parameter_value().string_value 

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        map_qos.history = rclpy.qos.HistoryPolicy.KEEP_LAST

        self.publisher_ = self.create_publisher(OccupancyGrid, 'global_costmap/feuerplan_image_topic', qos_profile=map_qos)
        self.subscription_map = self.create_subscription(OccupancyGrid, 'map', self.listener_callback_map, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription_map  
        self.is_message_published = False
        self.map_msg = None

    def listener_callback_map(self, msg):
        self.map_msg = msg
        if not self.is_message_published:
            self.get_logger().info('Map recieved')
            self.broadcast_frame_and_message()

    def preprocess_image(self, feuerplan_image) -> np.ndarray:
        processed_image = cv.GaussianBlur(feuerplan_image, (3, 3), 3, 3)
        processed_image = cv.Canny(processed_image, 50, 150, 3)
        dilation_size = 1
        dilation_element = cv.getStructuringElement(cv.MORPH_RECT, (2 * dilation_size + 1, 2 * dilation_size + 1))
        processed_image = cv.dilate(processed_image, dilation_element)
        feuerplan_image &= processed_image
        _, preprocessed_feuerplan = cv.threshold(feuerplan_image, 150, 127, cv.THRESH_BINARY)
        return preprocessed_feuerplan

    def ros_msg_to_image(self, ros_msg) -> np.ndarray:
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
    
    def get_best_match(self, confidence_values, matches, keypoints1, keypoints2) -> tuple[float,float]:
        max_value = max(confidence_values.tolist())
        top_matches = matches[confidence_values.tolist().index(max_value)]
        points1, points2 = keypoints1[top_matches[..., 0]], keypoints2[top_matches[..., 1]]
        return points1, points2

    def pixel_coord_to_world_coord(self, pixel, resolution, origin) -> tuple[float,float]:
        world_x = pixel[0] * resolution + origin[0]
        world_y = pixel[1] * resolution + origin[1]
        return (world_x, world_y)
    
    def find_matching_keypoints(self, image1, image2) -> tuple[float,float]:
            # Initialize feature extractor and matcher
            device = "cpu" 
            extractor = SuperPoint(max_num_keypoints = 2048).eval().to(device)
            matcher = LightGlue(features = "superpoint").eval().to(device)
            # Extract features from images
            self.get_logger().info(f'Extracting features...')
            feats1 = extractor.extract(numpy_image_to_torch(image1))
            feats2 = extractor.extract(numpy_image_to_torch(image2))
            # Match features between images
            self.get_logger().info(f'Obtaining best match...')
            matches12 = matcher({'image0': feats1, 'image1': feats2})
            # Refine features and matches
            feats1, feats2, matches12 = [rbd(x) for x in [feats1, feats2, matches12]]
            # Get the three best matches according to confidence values
            points1, points2 = self.get_best_match(matches12['scores'], matches12['matches'], feats1["keypoints"], feats2["keypoints"])
            return points1, points2
    
    def transform_feuerplan_origin(self, map_msg, feuerplan_image) -> tuple[float,float]:
        map_image = self.ros_msg_to_image(map_msg)
        points1, points2 = self.find_matching_keypoints(map_image, feuerplan_image)
        world1x, world1y = self.pixel_coord_to_world_coord(points1.numpy(), map_msg.info.resolution,(map_msg.info.origin.position.x, map_msg.info.origin.position.y))
        world2x, world2y = self.pixel_coord_to_world_coord(points2.numpy(), map_msg.info.resolution,(0,0))            
        translation_x = world1x - world2x
        translation_y = world1y - world2y
        return translation_x, translation_y

    def publish_occupancy_grid(self, feuerplan_image, translation_x, translation_y) -> None:
        data = feuerplan_image.flatten().tolist()

        ros_image_msg = OccupancyGrid()
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = 'feuerplan'
        ros_image_msg.info.map_load_time.sec = 0
        ros_image_msg.info.map_load_time.nanosec = 0
        ros_image_msg.info.resolution = 0.05
        ros_image_msg.info.width = feuerplan_image.shape[1]
        ros_image_msg.info.height = feuerplan_image.shape[0]
        ros_image_msg.info.origin.position.x = translation_x
        ros_image_msg.info.origin.position.y = translation_y
        ros_image_msg.info.origin.position.z = 0.0
        ros_image_msg.info.origin.orientation.x = 1.0
        ros_image_msg.info.origin.orientation.y = 0.0
        ros_image_msg.info.origin.orientation.z = 0.0
        ros_image_msg.info.origin.orientation.w = 0.0
        ros_image_msg.data = data

        self.get_logger().info('Feuerplan published')
        self.publisher_.publish(ros_image_msg)
        self.is_message_published = True

    def is_valid_data(self, data):
        if not isinstance(data, (list, tuple)):
            self.get_logger().info(f'list')
        
        for item in data:
            if not isinstance(item, int):
                self.get_logger().info(f'int')
            elif not (-128 <= item <= 127):
                self.get_logger().info(f'range')
        
        self.get_logger().info(f'true')


    def broadcast_frame_and_message(self)  -> None:
        feuerplan_image = cv.imread(self.feuerplan_path, cv.IMREAD_GRAYSCALE)
        preprocessed_feuerplan = self.preprocess_image(feuerplan_image)
        translation_x, translation_y = self.transform_feuerplan_origin(self.map_msg, preprocessed_feuerplan)
        self.get_logger().info(f'{translation_x, translation_y}')
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'feuerplan'

        transform.transform.translation.x = translation_x
        transform.transform.translation.y = translation_y
        transform.transform.translation.z = self.map_msg.info.origin.position.z

        transform.transform.rotation.x = self.map_msg.info.origin.orientation.x
        transform.transform.rotation.y = self.map_msg.info.origin.orientation.y
        transform.transform.rotation.z = self.map_msg.info.origin.orientation.z
        transform.transform.rotation.w = self.map_msg.info.origin.orientation.w

        self.tf_broadcaster.sendTransform(transform)
        self.publish_occupancy_grid(preprocessed_feuerplan, translation_x, translation_y)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = FeuerplanPublisher()

    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()