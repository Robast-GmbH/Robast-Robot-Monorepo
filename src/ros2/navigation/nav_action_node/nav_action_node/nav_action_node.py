import rclpy
import math
import tf2_ros
from tf_transformations import euler_from_quaternion
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Twist
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Duration

import os


class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__("navigate_to_pose_action_client")

        self.declare_parameter('sector_angle', math.pi / 4)
        self.declare_parameter('sector_radius', 0.7)
        self.declare_parameter('free_space_threshold', 80.0)

        self.__behavior_tree = "/workspace/src/navigation/nav_bringup/behavior_trees/humble/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml"
        if not os.path.isfile(self.__behavior_tree):
            raise FileNotFoundError(f"No such file: '{self.__behavior_tree}'")

        self.__action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.__goal_subscriber = self.create_subscription(
            Pose, "set_goal_pose", self.__send_goal, 10
        )
        self.__cancel_goal_subscriber = self.create_subscription(
            Bool, "cancel_goal", self.__cancel_goal, 10
        )
        self.__remaining_time_publisher = self.create_publisher(
            Duration, "navigation_remaining_time", 10
        )
        self.__is_navigating_publisher = self.create_publisher(
            Bool, "is_navigating", 10
        )
        self.__status_publisher = self.create_publisher( 
            String, "goal_status", 10
        )
        self.__local_costmap_subscriber = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.__local_costmap_callback, 10
        )
        self.__cmd_vel_publisher = self.create_publisher(
            Twist, 'diff_drive_base_controller/cmd_vel_unstamped', 1
        )

        timer_period_in_seconds = 1.0

        self.__timer = self.create_timer(timer_period_in_seconds, self.__timer_callback)
        self.__remaining_time = Duration()
        self.__goal_handle = None
        self.__sector_angle = self.get_parameter('sector_angle').get_parameter_value().double_value
        self.__sector_radius = self.get_parameter('sector_radius').get_parameter_value().double_value
        self.__free_space_threshold = self.get_parameter('free_space_threshold').get_parameter_value().double_value 
        self.__rotation_speed = 0.5
        self.__tf_buffer = tf2_ros.Buffer()
        self.__tf_listener = tf2_ros.TransformListener(self.__tf_buffer, self)

        self.__local_costmap_data = None
        self.__is_rotating = False
        self.__rotation_timer = None  # To handle continuous checking during rotation

    def __timer_callback(self):
        self.__remaining_time_publisher.publish(self.__remaining_time)
    
    def __local_costmap_callback(self, msg: OccupancyGrid):
        self.__local_costmap_data = msg

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
        self.__is_navigating_publisher.publish(Bool(data=True))
        self._get_result_future = self.__goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.__get_result_callback)

    def __get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:  
            self.__status_publisher.publish(String(data="SUCCEEDED"))
            #self.__check_free_space_and_publish_success()
        elif status == GoalStatus.STATUS_CANCELED:  
            self.get_logger().info("Goal was canceled")
            self.__status_publisher.publish(String(data="CANCELED"))
        elif status == GoalStatus.STATUS_ABORTED:  
            self.get_logger().info("Goal failed or was aborted")
            self.__status_publisher.publish(String(data="ABORTED"))
    
        self.__is_navigating_publisher.publish(Bool(data=False))

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
        if self.__local_costmap_data is not None:
            transform = self.__tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.__robot_x = transform.transform.translation.x
            self.__robot_y = transform.transform.translation.y
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            _, _, self.__yaw = euler_from_quaternion(quaternion)
            success = self.__check_sector()
            if success:
                self.get_logger().info("Free space sufficient. Goal SUCCEEDED!")
                self.__stop_robot()
                self.__status_publisher.publish(String(data="SUCCEEDED"))
            else:
                self.get_logger().info("Rotating to find sufficient free space. . .")
                self.__rotate_robot()

    def __check_sector(self):
    
        free_cells = 0
        total_cells_in_sector = 0
        resolution = self.__local_costmap_data.info.resolution
        width = self.__local_costmap_data.info.width
        height = self.__local_costmap_data.info.height

        # Check each cell in the costmap
        for i in range(height):
            for j in range(width):
                cell_x = self.__local_costmap_data.info.origin.position.x + j * resolution
                cell_y = self.__local_costmap_data.info.origin.position.y + i * resolution
                dx = cell_x - self.__robot_x
                dy = cell_y -self.__robot_y
                distance = math.sqrt(dx * dx + dy * dy)
                angle = math.atan2(dy, dx)
                relative_angle = angle - self.__yaw

                if distance <= self.__sector_radius and abs(relative_angle) <= self.__sector_angle / 2:
                    total_cells_in_sector += 1
                    index = i * width + j
                    if self.__local_costmap_data.data[index] >= 0 and self.__local_costmap_data.data[index] <= 70:
                        free_cells += 1

        if total_cells_in_sector > 0:
            free_space_percentage = (free_cells / total_cells_in_sector) * 100.0
        else:
            free_space_percentage = 0.0

        return free_space_percentage >= self.__free_space_threshold
    
    def __rotate_robot(self):
        twist = Twist()
        twist.angular.z = self.__rotation_speed
        self.__cmd_vel_publisher.publish(twist)

        # Recheck free space every 1 second while rotating
        if self.__rotation_timer is None:
            self.__rotation_timer = self.create_timer(1.0, self.__check_free_space_during_rotation)

    def __check_free_space_during_rotation(self):
        """Recheck sector while robot is rotating."""
        if self.__check_sector():
            self.get_logger().info("Free space found during rotation. Stopping robot...")
            self.__stop_robot()
            self.__status_publisher.publish(String(data="SUCCEEDED"))
        else:
            self.get_logger().info("Still rotating to find sufficient free space...")

    def __stop_robot(self):
        twist = Twist()
        twist.angular.z = 0.0 
        self.__cmd_vel_publisher.publish(twist)

        # Cancel the timer if it's running
        if self.__rotation_timer is not None:
            self.__rotation_timer.cancel()
            self.__rotation_timer = None

        self.get_logger().info("Rotation stopped")

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