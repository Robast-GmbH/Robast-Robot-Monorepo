import rclpy
import cv2 as cv

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from feuerplan_publisher.utils import calculate_translation_and_rotation, publish_occupancy_grid_from_image, create_feuerplan_frame


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
        self.__has_valid_transform = False
        self.__initial_match_obtained = False
        self.__throttled_timer = self.create_timer(10.0, self.__throttled_callback)  # 10 seconds interval
        self.__throttled_timer.cancel()
        self.__previous_translation_and_rotation = None

    def __throttled_callback(self):
        if self.__map_msg:
            self.get_logger().info('Map recieved')
            self.__broadcast_frame_and_message()

    def __listener_callback_map(self, msg:OccupancyGrid) -> None:
        self.__map_msg = msg 
        if not self.__initial_match_obtained:
            self.get_logger().info('Map recieved.')
            self.__broadcast_frame_and_message()
        else:
            self.get_logger().info('Previous matches would be used.')
            feuerplan_image = cv.imread(self.__feuerplan_path, cv.IMREAD_GRAYSCALE)
            create_feuerplan_frame(self.__tf_broadcaster, self.__previous_translation_and_rotation, self.__map_msg, self.get_clock())
            publish_occupancy_grid_from_image(self.__publisher, feuerplan_image, self.__previous_translation_and_rotation, self.get_logger(), self.get_clock())

    def __broadcast_frame_and_message(self) -> None:
        feuerplan_image = cv.imread(self.__feuerplan_path, cv.IMREAD_GRAYSCALE)
        translation_and_rotation = calculate_translation_and_rotation(self.__map_msg, feuerplan_image, self.__confidence_threshold, self.get_logger())
        if translation_and_rotation:
            self.__previous_translation_and_rotation = translation_and_rotation
            create_feuerplan_frame(self.__tf_broadcaster, self.__previous_translation_and_rotation, self.__map_msg, self.get_clock())
            publish_occupancy_grid_from_image(self.__publisher, feuerplan_image, self.__previous_translation_and_rotation, self.get_logger(), self.get_clock())
            self.__initial_match_obtained = True
            self.__throttled_timer.reset()

def main(args=None):
    rclpy.init(args=args)
    feuerplan_publisher = FeuerplanPublisher()
    rclpy.spin(feuerplan_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()