import rclpy
import math
import tf2_ros

from tf_transformations import euler_from_quaternion
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Twist, Point
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Duration

import os


class NavigateToPoseActionClient(Node):
    def __init__(self):
        super().__init__("navigate_to_pose_action_client")

        self.declare_parameter('sector_angle', math.pi / 4)
        self.declare_parameter('sector_radius', 0.7)
        self.declare_parameter('free_space_threshold', 50.0)
        self.declare_parameter('cmd_vel_topic', '')
        self.declare_parameter('robot_base_frame_param', '')

        self.__sector_angle = self.get_parameter('sector_angle').get_parameter_value().double_value
        self.__sector_radius = self.get_parameter('sector_radius').get_parameter_value().double_value
        self.__free_space_threshold = self.get_parameter('free_space_threshold').get_parameter_value().double_value
        self.__cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.__robot_base_frame = self.get_parameter('robot_base_frame_param').get_parameter_value().string_value

        timer_period_in_seconds = 1.0
        self.__timer = self.create_timer(timer_period_in_seconds, self.__timer_callback)
        self.__remaining_time = Duration()
        self.__goal_handle = None
        self.__local_costmap_data = None
        self.__check_sector_flag = False
        self.__is_reorientation_enabled = False
        self.__rotation_speed = 0.5
        self.__tf_buffer = tf2_ros.Buffer()
        self.__tf_listener = tf2_ros.TransformListener(self.__tf_buffer, self)

        self.__behavior_tree = "/workspace/src/navigation/nav_bringup/behavior_trees/humble/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml"
        if not os.path.isfile(self.__behavior_tree):
            raise FileNotFoundError(f"No such file: '{self.__behavior_tree}'")

        self.__action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        self.__goal_subscriber = self.create_subscription(Pose, "set_goal_pose", self.__send_goal, 10)
        self.__cancel_goal_subscriber = self.create_subscription(Bool, "cancel_goal", self.__cancel_goal, 10)
        self.__local_costmap_subscriber = self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.__local_costmap_callback, 10)
        self.__is_reorientation_enabled_subscriber = self.create_subscription(Bool, "is_reorientation_enabled", self.__set_reorientation_enabled, 10)
        self.__is_navigating_publisher = self.create_publisher(Bool, "is_navigating", 10)
        self.__remaining_time_publisher = self.create_publisher(Duration, "navigation_remaining_time", 10)
        self.__status_publisher = self.create_publisher( String, "goal_status", 10)
        self.__cmd_vel_publisher = self.create_publisher(Twist, self.__cmd_vel_topic, 1)
        self.__marker_for_visualisation_publisher = self.create_publisher(Marker, 'sector_marker', 10)

    def __timer_callback(self):
        self.__remaining_time_publisher.publish(self.__remaining_time)
    
    def __set_reorientation_enabled(self, msg: Bool):
        self.__is_reorientation_enabled = msg.data

    def __local_costmap_callback(self, msg: OccupancyGrid):
        self.__local_costmap_data = msg
        if self.__check_sector_flag:
            self.__check_free_space_and_publish_success()

    def __send_goal(self, goal_pose: Pose):
        self.get_logger().info("Goal received")
        goal_msg = NavigateToPose.Goal()
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.pose = goal_pose
        goal_pose_stamped.header.frame_id = "map"
        now = self.get_clock().now()
        goal_pose_stamped.header.stamp = now.to_msg()
        goal_msg.pose = goal_pose_stamped

        goal_msg.behavior_tree = self.__behavior_tree
        success = self.__action_client.wait_for_server(timeout_sec=2.0)
        if success:
            self._send_goal_future = self.__action_client.send_goal_async(
                goal_msg, feedback_callback=self.__feedback_callback
            )

            self._send_goal_future.add_done_callback(self.__goal_response_callback)
        else:
            self.get_logger().warn("Action server not available")

    def __cancel_goal(self, unused):
        if self.__goal_handle is None:
            return
        self.__action_client._cancel_goal_async(self.__goal_handle)
        self.get_logger().info("Action canceled")

    def __goal_response_callback(self, future):
        self.__goal_handle = future.result()
        if not self.__goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._get_result_future = self.__goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__get_result_callback)

    def __get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED and not self.__is_reorientation_enabled:
            self.__status_publisher.publish(String(data="SUCCEEDED"))
        elif status == GoalStatus.STATUS_SUCCEEDED and self.__is_reorientation_enabled:
            self.__check_sector_flag = True
        elif status == GoalStatus.STATUS_CANCELED:  
            self.get_logger().info("Goal was canceled")
            self.__status_publisher.publish(String(data="CANCELED"))
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Goal failed or was aborted")
            self.__status_publisher.publish(String(data="ABORTED"))

    def __feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            "Received feedback: {0}".format(feedback.estimated_time_remaining)
        )
        self.get_logger().debug(
            "Received feedback distance: {0}".format(feedback.distance_remaining)
        )
        self.__remaining_time = feedback.estimated_time_remaining

    def __check_free_space_and_publish_success(self):
        try:
            transform = self.__tf_buffer.lookup_transform('map', self.__robot_base_frame, rclpy.time.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            _, _, yaw = euler_from_quaternion(quaternion) 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Could not get robot pose from tf')
            return

        free_cells = 0
        total_cells_in_sector = 0

        resolution = self.__local_costmap_data.info.resolution
        width = self.__local_costmap_data.info.width
        height = self.__local_costmap_data.info.height

        # For Visualisation
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  
        marker.color.a = 1.0
        marker.color.r = 1.0  
        marker.lifetime = Duration(sec=1)  

        for i in range(height):
            for j in range(width):
                cell_x = self.__local_costmap_data.info.origin.position.x + j * resolution
                cell_y = self.__local_costmap_data.info.origin.position.y + i * resolution

                dx = cell_x - robot_x
                dy = cell_y - robot_y
                distance = math.sqrt(dx * dx + dy * dy)
                angle = math.atan2(dy, dx)
                relative_angle = angle - yaw

                if distance <= self.__sector_radius and abs(relative_angle) <= self.__sector_angle / 2:
                    total_cells_in_sector += 1
                    index = i * width + j
                    if self.__local_costmap_data.data[index] >= 0 and self.__local_costmap_data.data[index] <= 50:
                        free_cells += 1
                    # For Visualisation
                    point = Point()
                    point.x = cell_x
                    point.y = cell_y
                    point.z = 0.0
                    marker.points.append(point)

        self.__marker_for_visualisation_publisher.publish(marker)

        free_space_percentage = 0.0
        if total_cells_in_sector > 0:
            free_space_percentage = (free_cells / total_cells_in_sector) * 100.0

        self.get_logger().info(f"Free space in sector: {free_space_percentage:.2f}%")

        if free_space_percentage < self.__free_space_threshold:
            self.get_logger().info(f"Free space below threshold ({self.__free_space_threshold}%), rotating robot...")
            self.__rotate_robot()
        else:
            self.get_logger().info("Free space above threshold, goal succeeded!")
            self.__stop_robot()
            self.__status_publisher.publish(String(data="SUCCEEDED"))
            self.__check_sector_flag = False
    
    
    def __rotate_robot(self):
        twist = Twist()
        twist.angular.z = self.__rotation_speed
        self.__cmd_vel_publisher.publish(twist)

    def __stop_robot(self):
        twist = Twist()
        twist.angular.z = 0.0 
        self.__cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = NavigateToPoseActionClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
