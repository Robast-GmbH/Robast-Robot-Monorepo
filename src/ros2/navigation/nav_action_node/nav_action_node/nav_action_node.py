import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Duration

import os


class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__("navigate_to_pose_action_client")

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

        timer_period_in_seconds = 1.0
        self.__timer = self.create_timer(timer_period_in_seconds, self.__timer_callback)
        self.__remaining_time = Duration()
        self.__goal_handle = None

    def __timer_callback(self):
        self.__remaining_time_publisher.publish(self.__remaining_time)

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
            self.get_logger().info("Goal succeeded!")
            self.__status_publisher.publish(String(data="SUCCEEDED"))
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