import rclpy
import cv2 as cv

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from feuerplan_publisher.utils import calculate_translation_and_rotation, create_feuerplan_frame, publish_occupancy_grid_from_image, rotate_image


class FeuerplanPublisher(Node):

    def __init__(self):
        super().__init__('feuerplan_publisher')

        self.declare_parameter('feuerplan_path', '')
        self.declare_parameter('confidence_threshold', 0.7)

        feuerplan_path = self.get_parameter('feuerplan_path').get_parameter_value().string_value 
        self.__feuerplan_image = cv.imread(feuerplan_path, cv.IMREAD_GRAYSCALE)
        self.__confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.__use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        map_qos.history = rclpy.qos.HistoryPolicy.KEEP_LAST

        self.__publisher = self.create_publisher(OccupancyGrid, 'global_costmap/feuerplan_image_topic', qos_profile=map_qos)
        self.__map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.__listener_callback_map, 10)
        self.get_logger().info('Waiting for map...')
        self.__tf_broadcaster = TransformBroadcaster(self)
        self.__previous_map_msg = None
        self.__map_msg = None
        self.__initial_match_obtained = False
        self.__throttled_timer = self.create_timer(10.0, self.__throttled_callback)  # 10 seconds
        self.__throttled_timer.cancel()
        self.__previous_translation_and_rotation = None
        self.__best_angle = None
        self.__rotated_feuerplan_image = None

    def __throttled_callback(self) -> None:
        if self.__map_msg.data != self.__previous_map_msg:
            self.get_logger().info('Map changed. Recalculating...')
            self.__broadcast_frame_and_message()

    def __listener_callback_map(self, msg:OccupancyGrid) -> None:
        self.__map_msg = msg 
        if not self.__initial_match_obtained:
            self.get_logger().info('Map recieved.')
            self.__broadcast_frame_and_message()
        else:
            self.get_logger().info('Previous matches would be used.')
            create_feuerplan_frame(self.__tf_broadcaster, self.__previous_translation_and_rotation, self.__map_msg, self.get_clock())
            publish_occupancy_grid_from_image(self.__publisher, self.__rotated_feuerplan_image, self.__previous_translation_and_rotation, self.get_logger(), self.get_clock())

    def __broadcast_frame_and_message(self) -> None:
        if self.__best_angle is not None:
            feuerplan_image_to_use = self.__rotated_feuerplan_image
            translation_and_rotation, _ = calculate_translation_and_rotation(self.__map_msg, feuerplan_image_to_use, self.__confidence_threshold, self.get_logger(), self.__best_angle)
        else:
            translation_and_rotation, best_angle = calculate_translation_and_rotation(self.__map_msg, self.__feuerplan_image, self.__confidence_threshold, self.get_logger())
            if best_angle is not None:
                self.__best_angle = best_angle
                self.__rotated_feuerplan_image = rotate_image(self.__feuerplan_image, self.__best_angle)

        if translation_and_rotation:
            self.__previous_translation_and_rotation = translation_and_rotation
            create_feuerplan_frame(self.__tf_broadcaster, self.__previous_translation_and_rotation, self.__map_msg, self.get_clock())
            publish_occupancy_grid_from_image(self.__publisher, self.__rotated_feuerplan_image, self.__previous_translation_and_rotation, self.get_logger(), self.get_clock())
            self.__initial_match_obtained = True
            self.__previous_map_msg = self.__map_msg.data
            self.__throttled_timer.reset()


def main(args=None):
    rclpy.init(args=args)
    feuerplan_publisher = FeuerplanPublisher()
    rclpy.spin(feuerplan_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()