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

        self.__feuerplan_path = self.get_parameter('feuerplan_path').get_parameter_value().string_value 

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        map_qos.history = rclpy.qos.HistoryPolicy.KEEP_LAST

        self.__publisher = self.create_publisher(OccupancyGrid, 'global_costmap/feuerplan_image_topic', qos_profile=map_qos)
        self.__map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.__listener_callback_map, 10)
        self.__tf_broadcaster = TransformBroadcaster(self)

        self.__is_message_published_once = False
        self.__map_msg = None

    def __listener_callback_map(self, msg:OccupancyGrid) -> None:
        self.__map_msg = msg
        self.get_logger().info('Map recieved')
        self.__broadcast_frame_and_message()

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
    
    def __get_three_best_matches(self, confidence_values:torch.tensor, matches:torch.tensor, keypoints1:torch.tensor, keypoints2:torch.tensor) -> tuple[float,float]:
        top_values, top_indices = torch.topk(confidence_values, k=3)
        top_matches = matches[top_indices]
        points1, points2 = keypoints1[top_matches[..., 0]], keypoints2[top_matches[..., 1]]
        return points1, points2, matches, top_values
    

    def __pixel_coord_to_world_coord(self, pixel, resolution, origin) -> tuple[float,float]:
        world_x = pixel[0] * resolution + origin[0]
        world_y = pixel[1] * resolution + origin[1]
        return (world_x, world_y)
    
    def __create_transformation_matrix(self, image1:np.ndarray, image2:np.ndarray, map_resolution:float, map_origin_x:float,map_origin_y:float) -> tuple[float,float]:
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
            matches = matches12['matches']
            size = (matches12['matches']).size(0)
            self.get_logger().info(f'{matches}')
            self.get_logger().info(f'{size}')
            if matches12['matches'].size(0) > 3:
                points1, points2, matches, scores = self.__get_three_best_matches(matches12['scores'], matches12['matches'], feats1["keypoints"], feats2["keypoints"])
                if (scores > 0.5).all():
                    axes = viz2d.plot_images([image1, image2])
                    viz2d.plot_matches(points1, points2, color="lime", lw=0.2)
                    viz2d.save_plot("out.png")
                    world_points1 = [(x * map_resolution + map_origin_x, y * map_resolution + map_origin_y) for x, y in points1]
                    self.get_logger().info(f'{scores}')
                    A = np.array([
                        [points2[0][0], points2[0][1], 1, 0, 0, 0],
                        [0, 0, 0, points2[0][0], points2[0][1], 1],
                        [points2[1][0], points2[1][1], 1, 0, 0, 0],
                        [0, 0, 0, points2[1][0], points2[1][1], 1],
                        [points2[2][0], points2[2][1], 1, 0, 0, 0],
                        [0, 0, 0, points2[2][0], points2[2][1], 1]
                    ])

                    B = np.array([
                        world_points1[0][0], world_points1[0][1],
                        world_points1[1][0], world_points1[1][1],
                        world_points1[2][0], world_points1[2][1]
                    ])
                    affine_params, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
                    transformation_matrix = np.array([
                        [affine_params[0], affine_params[1]],
                        [affine_params[3], affine_params[4]]
                    ])
                    translation_vector = np.array([affine_params[2], affine_params[5]])
                    #point = np.float32([matched_keypoints1.numpy()[0][0],matched_keypoints1.numpy()[0][1],1])
                    #transformation_matrix = cv.getAffineTransform(points1.numpy(),points2.numpy())
                    #transformed = transformation_matrix @ point
                    #self.get_logger().info(f'{point}')
                    #self.get_logger().info(f'{transformation_matrix}')
                    #self.get_logger().info(f'{transformed}')
                    return {
                        "transformation_matrix": transformation_matrix,
                        "translation_vector": translation_vector
                    }
                else:
                    return None
            else:
                return None
    
    def __determine_origin_in_world(self,affine_transformation, feuerplan_origin):
        affine_matrix = affine_transformation['transformation_matrix']
        translation_vector = affine_transformation['translation_vector']

        # Apply the affine transformation to the image origin
        world_origin = affine_matrix @ feuerplan_origin + translation_vector

        return world_origin
    
    def __transform_between_map_and_feuerplan(self, map_msg:OccupancyGrid, feuerplan_image:np.ndarray) -> tuple[float,float]:
        map_image = self.__ros_msg_to_image(map_msg)
        cv.imwrite('output_image_opencv.png', map_image)
        if not self.__is_message_published_once:
            origin_feuerplan = (0.0,0.0)
            #self.__publish_occupancy_grid_from_image(feuerplan_image, origin_feuerplan[0], origin_feuerplan[1])
            self.__origin_feuerplan_prev = origin_feuerplan
            self.__is_message_published_once = True
        else:
            self.get_logger().info('Feuerplan published once already')
            transformation = self.__create_transformation_matrix(map_image, feuerplan_image, map_msg.info.resolution, map_msg.info.origin.position.x, map_msg.info.origin.position.y)    
            self.get_logger().info(f'{transformation}')
            if transformation is not None:
                origin_feuerplan_updated = self.__determine_origin_in_world(transformation, self.__origin_feuerplan_prev)
                self.__origin_feuerplan_prev = origin_feuerplan_updated

    def __publish_occupancy_grid_from_image(self, image:np.ndarray, translation_x, translation_y) -> None:
        data = image.flatten().tolist()
        ros_image_msg = OccupancyGrid()
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = 'feuerplan'
        ros_image_msg.info.map_load_time.sec = 0
        ros_image_msg.info.map_load_time.nanosec = 0
        ros_image_msg.info.resolution = 0.05
        ros_image_msg.info.width = image.shape[1]
        ros_image_msg.info.height = image.shape[0]
        ros_image_msg.info.origin.position.x = -19.4
        ros_image_msg.info.origin.position.y = -3.0
        ros_image_msg.info.origin.position.z = 0.0
        ros_image_msg.info.origin.orientation.x = 1.0
        ros_image_msg.info.origin.orientation.y = 0.0
        ros_image_msg.info.origin.orientation.z = 0.0
        ros_image_msg.info.origin.orientation.w = 0.0
        ros_image_msg.data = data

        self.get_logger().info('Feuerplan published')
        self.__publisher.publish(ros_image_msg)
        #self.__is_message_published = True

    def __create_feuerplan_frame(self, translation_x:float, translation_y:float) -> None:
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'feuerplan'

        transform.transform.translation.x = -20.2
        transform.transform.translation.y = -3.3
        transform.transform.translation.z = self.__map_msg.info.origin.position.z

        transform.transform.rotation.x = self.__map_msg.info.origin.orientation.x
        transform.transform.rotation.y = self.__map_msg.info.origin.orientation.y
        transform.transform.rotation.z = self.__map_msg.info.origin.orientation.z
        transform.transform.rotation.w = self.__map_msg.info.origin.orientation.w
        self.__tf_broadcaster.sendTransform(transform)

    def __broadcast_frame_and_message(self) -> None:
        feuerplan_image = cv.imread(self.__feuerplan_path, cv.IMREAD_GRAYSCALE)
        normalized_image = np.clip(feuerplan_image.astype(np.int16) - 128, -128, 127)
        #preprocessed_feuerplan = self.__preprocess_image(feuerplan_image)
        self.__transform_between_map_and_feuerplan(self.__map_msg, feuerplan_image)
        self.get_logger().info(f'{self.__origin_feuerplan_prev}')
        self.get_logger().info(f'{(self.__map_msg.info.origin.position.x,self.__map_msg.info.origin.position.y)}')
        self.__create_feuerplan_frame(self.__origin_feuerplan_prev[0],self.__origin_feuerplan_prev[1])
        self.__publish_occupancy_grid_from_image(normalized_image, self.__origin_feuerplan_prev[0], self.__origin_feuerplan_prev[1])

def main(args=None):
    rclpy.init(args=args)

    feuerplan_publisher = FeuerplanPublisher()

    rclpy.spin(feuerplan_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()