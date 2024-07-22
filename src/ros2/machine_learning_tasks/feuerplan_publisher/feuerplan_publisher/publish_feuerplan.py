import rclpy
import torch
import numpy as np
import cv2 as cv

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from lightglue import viz2d
from lightglue.lightglue import LightGlue
from lightglue.utils import numpy_image_to_torch, rbd
from lightglue.superpoint import SuperPoint


class FeuerplanPublisher(Node):

    def __init__(self):
        super().__init__('feuerplan_publisher')

        self.declare_parameter('feuerplan_path', '')
        self.declare_parameter('confidence_threshold', 0.7)

        self.__feuerplan_path = self.get_parameter('feuerplan_path').get_parameter_value().string_value 
        self.__confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        map_qos.history = rclpy.qos.HistoryPolicy.KEEP_LAST

        self.__publisher = self.create_publisher(OccupancyGrid, 'global_costmap/feuerplan_image_topic', qos_profile=map_qos)
        self.get_logger().info('Waiting for map...')
        self.__map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.__listener_callback_map, 10)
        self.__tf_broadcaster = TransformBroadcaster(self)
        self.__map_msg = None

    def __listener_callback_map(self, msg:OccupancyGrid) -> None:
        self.__map_msg = msg 
        self.get_logger().info('Map recieved')
        self.__broadcast_frame_and_message()

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
    
    def __get_three_best_matches(self, confidence_values:torch.tensor, matches:torch.tensor, keypoints1:torch.tensor, keypoints2:torch.tensor) -> tuple[float,float]:
        top_values, top_indices = torch.topk(confidence_values, k=10)
        top_matches = matches[top_indices]
        points1, points2 = keypoints1[top_matches[..., 0]], keypoints2[top_matches[..., 1]]
        return points1, points2, matches, top_values
    

    def __pixel_coord_to_world_coord(self, pixel, resolution, origin) -> tuple[float,float]:
        world_x = pixel[0] * resolution + origin[0]
        world_y = pixel[1] * resolution + origin[1]
        return (world_x, world_y)
    
    def __calculate_rotation_matrix(self, keypoints1, keypoints2):
            # Calculate the centroids of the keypoints
            centroid1 = np.mean(keypoints1.numpy(), axis=0)
            centroid2 = np.mean(keypoints2.numpy(), axis=0)
            # Normalize the keypoints by subtracting the centroids
            keypoints1_norm = keypoints1.numpy() - centroid1
            keypoints2_norm = keypoints2.numpy() - centroid2
            # Compute the covariance matrix
            H = np.dot(keypoints1_norm.T, keypoints2_norm)
            # Perform SVD
            U, S, Vt = np.linalg.svd(H)
            R = np.dot(Vt.T, U.T)
            # Ensure the rotation matrix is proper (det(R) should be 1)
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = np.dot(Vt.T, U.T)
                return R
            else: 
                return None           
    
    def __calculate_translation_and_rotation(self, image1:np.ndarray, image2:np.ndarray) -> tuple[float,float]:
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
            size = (matches12['matches']).size(0)
            if matches12['matches'].size(0) > 12:
                points1, points2, _, scores = self.__get_three_best_matches(matches12['scores'], matches12['matches'], feats1["keypoints"], feats2["keypoints"])
                if (scores > self.__confidence_threshold).all():
                    R = self.__calculate_rotation_matrix(points1, points2)
                    if R is not None:
                        rotation_angle = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
                        world1x, world1y = self.__pixel_coord_to_world_coord(points1.numpy()[0], self.__map_msg.info.resolution,(self.__map_msg.info.origin.position.x, self.__map_msg.info.origin.position.y))
                        world2x, world2y = self.__pixel_coord_to_world_coord(points2.numpy()[0], self.__map_msg.info.resolution,(0,0))   
                        translation_x = world1x - world2x
                        translation_y = world1y - world2y
                        return (translation_x, translation_y, rotation_angle)
                    else:
                        self.get_logger().info(f'Rotation matrix is not properly calculated.')
                        return None
                else:
                    self.get_logger().info(f'Match confidences below threshold.')
                    return None
            else:
                self.get_logger().info(f'Not enough matches. Only {size} found.')
                return None
            
    def __transform_between_map_and_feuerplan(self, map_msg:OccupancyGrid, feuerplan_image:np.ndarray) -> tuple[float,float]:
        map_image = self.__ros_msg_to_image(map_msg)
        translation_and_rotation = self.__calculate_translation_and_rotation(map_image, feuerplan_image)
        if translation_and_rotation is not None:
            self.get_logger().info(f'New origin found: {translation_and_rotation[0], translation_and_rotation[1]}')
            self.get_logger().info(f'Image to be rotated at {translation_and_rotation[2]} degrees')
            (h, w) = feuerplan_image.shape[:2]
            center = (w / 2, h / 2)
            # Rotate the image around its center
            M = cv.getRotationMatrix2D(center, translation_and_rotation[2], 1.0)
            rotated_feuerplan_image = cv.warpAffine(feuerplan_image, M, (w, h))
            #self.__create_feuerplan_frame(origin[0], origin[1])
            self.__publish_occupancy_grid_from_image(rotated_feuerplan_image, translation_and_rotation[0], translation_and_rotation[1])

    def __publish_occupancy_grid_from_image(self, image:np.ndarray, translation_x, translation_y) -> None:
        #_, binary_image = cv.threshold(image, 127, 255, cv.THRESH_BINARY_INV)
        occupancy_grid = np.clip(image.astype(np.int16) - 128, -128, 127)
        data = occupancy_grid.flatten().tolist()
        ros_image_msg = OccupancyGrid()
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = 'map'
        ros_image_msg.info.map_load_time.sec = 0
        ros_image_msg.info.map_load_time.nanosec = 0
        ros_image_msg.info.resolution = 0.05
        ros_image_msg.info.width = occupancy_grid.shape[1]
        ros_image_msg.info.height = occupancy_grid.shape[0]
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
        self.__transform_between_map_and_feuerplan(self.__map_msg, feuerplan_image)

def main(args=None):
    rclpy.init(args=args)
    feuerplan_publisher = FeuerplanPublisher()
    rclpy.spin(feuerplan_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()