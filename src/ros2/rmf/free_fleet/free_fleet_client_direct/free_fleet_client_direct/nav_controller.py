from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from . import math_helper

class nav_controller:

    def __init__(self, ros_node:Node, odom_topic:str, frame_id:str, publish_status):
        self.nav = ActionClient(ros_node, NavigateToPose, 'navigate_to_pose')
        self.robot_x= 0
        self.robot_y=0
        self.robot_yaw=0
        self.active=False
        self.publish_status= publish_status
        self.frame_id=frame_id
        self.ros_node= ros_node

        self.subscriber_odom = ros_node.create_subscription(
            Odometry,
            odom_topic,
            self.get_robot_odom,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))

    def start_navigation(self, x, y, yaw):  
        print("start_nav")
        self.goal_pose =self.create_pose(x, y, yaw)
        print(self.goal_pose)

        self.nav.wait_for_server()

        self._send_goal_future = self.nav.send_goal_async(
                self.goal_pose,
                feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        print("start nav done")


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            self.publish_status( "canceld", "could not plan route to goal pose", True)
            return
        self.get_logger().info('Goal accepted')
        self.active= True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: NavigateToPose.Result):
        self.publish_status()
        self.active=False

    def feedback_callback(self, feedback_msg: NavigateToPose.Feedback):
        self.get_logger().debug('Received feedback')
        self.publish_status()
    
    def pause_navigation(self):
        self._send_goal_future.cancel()
        self.active=False

    def cancel_navigation(self):
        self.goal_pose = None
        self._send_goal_future.cancel()
        self.active=False
        
    def create_pose(self, pose_x, pose_y, pose_yaw) -> NavigateToPose.Goal:
        pose = NavigateToPose.Goal()
        waypoint=PoseStamped()
        waypoint.header.frame_id = self.frame_id
        waypoint.header.stamp = self.ros_node.get_clock().now().to_msg()
        waypoint.pose.position.x = pose_x
        waypoint.pose.position.y = pose_y
        waypoint.pose.position.z = 0.0
        qx, qy, qz, qw = math_helper.quaternion_from_euler(0, 0, pose_yaw)
        waypoint.pose.orientation.x = qx
        waypoint.pose.orientation.y = qy
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw
        pose.pose=waypoint
        pose.behavior_tree="navigate_to_pose_w_replanning_goal_patience_and_recovery"
        return pose

    def get_robot_odom(self, data:Odometry):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        q1 = data.pose.pose.orientation.x
        q2 = data.pose.pose.orientation.y
        q3 = data.pose.pose.orientation.z
        q4 = data.pose.pose.orientation.w
        q = (q1, q2, q3, q4)
        e = math_helper.euler_from_quaternion(q)
        th = 90
        yaw = math_helper.to_positive_angle(th)
        self.robot_x= float(x)
        self.robot_y=float(y)
        self.robot_yaw=float(yaw)