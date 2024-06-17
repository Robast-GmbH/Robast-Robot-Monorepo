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

        self.__feuerplan_path = self.get_parameter('feuerplan_path').get_parameter_value().string_value 

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        map_qos.history = rclpy.qos.HistoryPolicy.KEEP_LAST

        self.__publisher = self.create_publisher(OccupancyGrid, 'global_costmap/feuerplan_image_topic', qos_profile=map_qos)
        self.__map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.__listener_callback_map, 10)
        self.__tf_broadcaster = TransformBroadcaster(self)

        self.__is_message_published = False
        self.__map_msg = None

    def __listener_callback_map(self, msg:OccupancyGrid) -> None:
        self.__map_msg = msg
        if not self.__is_message_published:
            self.get_logger().info('Map recieved')
            self.__broadcast_frame_and_message()
            self.destroy_subscription(self.__map_subscriber)

    def __preprocess_image(self, feuerplan_image:np.ndarray) -> np.ndarray:
        processed_image = cv.GaussianBlur(feuerplan_image, (3, 3), 3, 3)
        processed_image = cv.Canny(processed_image, 50, 150, 3)
        dilation_size = 1
        dilation_element = cv.getStructuringElement(cv.MORPH_RECT, (2 * dilation_size + 1, 2 * dilation_size + 1))
        processed_image = cv.dilate(processed_image, dilation_element)
        feuerplan_image &= processed_image
        _, preprocessed_feuerplan = cv.threshold(feuerplan_image, 150, 127, cv.THRESH_BINARY)
        return preprocessed_feuerplan

    def __ros_msg_to_image(self, ros_msg:OccupancyGrid) -> np.ndarray:
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
    
    def __get_best_match(self, confidence_values:torch.tensor, matches:torch.tensor, keypoints1:torch.tensor, keypoints2:torch.tensor) -> tuple[float,float]:
        max_value = max(confidence_values.tolist())
        top_matches = matches[confidence_values.tolist().index(max_value)]
        points1, points2 = keypoints1[top_matches[..., 0]], keypoints2[top_matches[..., 1]]
        return points1, points2

    def __pixel_coord_to_world_coord(self, pixel, resolution, origin) -> tuple[float,float]:
        world_x = pixel[0] * resolution + origin[0]
        world_y = pixel[1] * resolution + origin[1]
        return (world_x, world_y)
    
    def __find_matching_keypoints(self, image1:np.ndarray, image2:np.ndarray) -> tuple[float,float]:
            # Initialize feature extractor and matcher
            device = "cpu" 
            extractor = SuperPoint(max_num_keypoints = 2048).eval().to(device)
            matcher = LightGlue(features = "superpoint").eval().to(device)
            # Extract features from images
            self.get_logger().info('Extracting features...')
            feats1 = extractor.extract(numpy_image_to_torch(image1))
            feats2 = extractor.extract(numpy_image_to_torch(image2))
            # Match features between images
            self.get_logger().info('Obtaining best match...')
            matches12 = matcher({'image0': feats1, 'image1': feats2})
            # Refine features and matches
            feats1, feats2, matches12 = [rbd(x) for x in [feats1, feats2, matches12]]
            # Get the three best matches according to confidence values
            points1, points2 = self.__get_best_match(matches12['scores'], matches12['matches'], feats1["keypoints"], feats2["keypoints"])
            return points1, points2
    
    def __transform_feuerplan_origin(self, map_msg:OccupancyGrid, feuerplan_image:np.ndarray) -> tuple[float,float]:
        map_image = self.__ros_msg_to_image(map_msg)
        points1, points2 = self.__find_matching_keypoints(map_image, feuerplan_image)
        world1x, world1y = self.__pixel_coord_to_world_coord(points1.numpy(), map_msg.info.resolution,(map_msg.info.origin.position.x, map_msg.info.origin.position.y))
        world2x, world2y = self.__pixel_coord_to_world_coord(points2.numpy(), map_msg.info.resolution,(0,0))            
        translation_x = world1x - world2x
        translation_y = world1y - world2y
        return translation_x, translation_y

    def __publish_occupancy_grid(self, feuerplan_image:np.ndarray, translation_x:float, translation_y:float) -> None:
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
        self.__publisher.publish(ros_image_msg)
        self.__is_message_published = True

    def __create_feuerplan_frame(self, translation_x:float, translation_y:float) -> None:
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'feuerplan'

        transform.transform.translation.x = translation_x
        transform.transform.translation.y = translation_y
        transform.transform.translation.z = self.__map_msg.info.origin.position.z

        transform.transform.rotation.x = self.__map_msg.info.origin.orientation.x
        transform.transform.rotation.y = self.__map_msg.info.origin.orientation.y
        transform.transform.rotation.z = self.__map_msg.info.origin.orientation.z
        transform.transform.rotation.w = self.__map_msg.info.origin.orientation.w
        self.__tf_broadcaster.sendTransform(transform)

    def __broadcast_frame_and_message(self) -> None:
        feuerplan_image = cv.imread(self.__feuerplan_path, cv.IMREAD_GRAYSCALE)
        preprocessed_feuerplan = self.__preprocess_image(feuerplan_image)
        translation_x, translation_y = self.__transform_feuerplan_origin(self.__map_msg, preprocessed_feuerplan)
        self.__create_feuerplan_frame(translation_x,translation_y)
        self.__publish_occupancy_grid(preprocessed_feuerplan, translation_x, translation_y)

def main(args=None):
    rclpy.init(args=args)

    feuerplan_publisher = FeuerplanPublisher()

    rclpy.spin(feuerplan_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()