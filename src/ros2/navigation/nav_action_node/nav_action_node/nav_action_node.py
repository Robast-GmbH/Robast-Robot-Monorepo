import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration

import os


class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__("navigate_to_pose_action_client")

        self.behavior_tree = "/workspace/src/navigation/nav_bringup/behavior_trees/humble/navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml"
        if not os.path.isfile(self.behavior_tree):
            raise FileNotFoundError(f"No such file: '{self.behavior_tree}'")

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_subscriber = self.create_subscription(
            Pose, "set_goal_pose", self.send_goal, 10
        )
        self._cancel_goal_subscriber = self.create_subscription(
            Bool, "cancel_goal", self.cancel_goal, 10
        )
        self._remaining_time_publisher = self.create_publisher(
            Duration, "navigation_remaining_time", 10
        )
        self._is_navigating_publisher = self.create_publisher(Bool, "is_navigating", 10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.remaining_time = Duration()
        self.goal_handle = None

    def timer_callback(self):
        self._remaining_time_publisher.publish(self.remaining_time)

    def send_goal(self, goal_pose: Pose):
        self.get_logger().info("Goal received")
        goal_msg = NavigateToPose.Goal()
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.pose = goal_pose
        goal_pose_stamped.header.frame_id = "map"
        now = self.get_clock().now()
        goal_pose_stamped.header.stamp = now.to_msg()
        goal_msg.pose = goal_pose_stamped

        goal_msg.behavior_tree = self.behavior_tree
        success = self._action_client.wait_for_server(timeout_sec=2.0)
        if success:
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )

            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().warn("Action server not available")

    def cancel_goal(self, unused):
        if self.goal_handle is None:
            return
        self._action_client._cancel_goal_async(self.goal_handle)
        self.get_logger().info("Action canceled")

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._is_navigating_publisher.publish(Bool(data=True))
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result))
        self._is_navigating_publisher.publish(Bool(data=False))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            "Received feedback: {0}".format(feedback.estimated_time_remaining)
        )
        self.get_logger().debug(
            "Received feedback distance: {0}".format(feedback.distance_remaining)
        )
        self.remaining_time = feedback.estimated_time_remaining


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
