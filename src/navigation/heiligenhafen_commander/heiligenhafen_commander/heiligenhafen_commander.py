import rclpy
import numpy as np
import geometry_msgs

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_srvs.srv import SetBool


class HeiligenhafenCommander(Node):

    def __init__(self):
        super().__init__(node_name='heiligenhafen_commander')
        self.navigator = BasicNavigator()
        header_frame_id = 'map'
        initial_pose_x = 0.0
        initial_pose_y = 0.0
        initial_pose_yaw = 3.14

        self.set_initial_pose(header_frame_id, initial_pose_x, initial_pose_y, initial_pose_yaw)

        self.init_waypoints(header_frame_id)
        self.target_waypoint = 1

        self.waypoint_following_is_activated = True
        self.client = self.create_service(SetBool, 'activate_waypoint_following', self.activate_waypoint_following)

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

        self.handle_waypoint_follow_intervall = 1  # seconds
        self.timer = self.create_timer(self.handle_waypoint_follow_intervall, self.handle_waypoint_follow_callback)

        self.get_logger().info("The heiligenhafen commander is running")

    def set_initial_pose(self, header_frame_id, initial_pose_x, initial_pose_y, initial_pose_yaw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = header_frame_id
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = initial_pose_x
        initial_pose.pose.position.y = initial_pose_y
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, initial_pose_yaw)
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        self.navigator.setInitialPose(initial_pose)

    def handle_waypoint_follow_callback(self):
        if (self.waypoint_following_is_activated):
            if (self.navigator.isTaskComplete()):
                self.state_machine()
        else:
            self.navigator.cancelTask()

    def destroy_node(self):
        self.destroy_node()

    def init_waypoints(self, header_frame_id):
        header_stamp = self.navigator.get_clock().now().to_msg()

        goal_pose_1 = PoseStamped()
        goal_pose_1.header.frame_id = header_frame_id
        goal_pose_1.header.stamp = header_stamp  # TODO: We might want to update this right before sending the goal?
        goal_pose_1.pose.position.x = 0.0
        goal_pose_1.pose.position.y = -3.00
        yaw_1 = 3 * (3.14 / 4)
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, yaw_1)
        goal_pose_1.pose.orientation.x = qx
        goal_pose_1.pose.orientation.y = qy
        goal_pose_1.pose.orientation.z = qz
        goal_pose_1.pose.orientation.w = qw

        goal_pose_2 = PoseStamped()
        goal_pose_2.header.frame_id = header_frame_id
        goal_pose_2.header.stamp = header_stamp
        goal_pose_2.pose.position.x = -6.0
        goal_pose_2.pose.position.y = -3.00
        yaw_2 = 1 * (3.14 / 4)
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, yaw_2)
        goal_pose_2.pose.orientation.x = qx
        goal_pose_2.pose.orientation.y = qy
        goal_pose_2.pose.orientation.z = qz
        goal_pose_2.pose.orientation.w = qw

        goal_pose_3 = PoseStamped()
        goal_pose_3.header.frame_id = header_frame_id
        goal_pose_3.header.stamp = header_stamp
        goal_pose_3.pose.position.x = -6.0
        goal_pose_3.pose.position.y = 3.00
        yaw_3 = - 1 * (3.14 / 4)
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, yaw_3)
        goal_pose_3.pose.orientation.x = qx
        goal_pose_3.pose.orientation.y = qy
        goal_pose_3.pose.orientation.z = qz
        goal_pose_3.pose.orientation.w = qw

        goal_pose_4 = PoseStamped()
        goal_pose_4.header.frame_id = header_frame_id
        goal_pose_4.header.stamp = header_stamp
        goal_pose_4.pose.position.x = 0.0
        goal_pose_4.pose.position.y = 3.00
        yaw_4 = 3.14 - 1 * (3.14 / 4)
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, yaw_4)
        goal_pose_4.pose.orientation.x = qx
        goal_pose_4.pose.orientation.y = qy
        goal_pose_4.pose.orientation.z = qz
        goal_pose_4.pose.orientation.w = qw

        self.target_pose_by_waypoint_number = {
            1: goal_pose_1,
            2: goal_pose_2,
            3: goal_pose_3,
            4: goal_pose_4,
        }

    def state_machine(self):
        match self.target_waypoint:
            case 1:
                self.navigator.goToPose(self.target_pose_by_waypoint_number[self.target_waypoint])
                self.target_waypoint += 1
                return

            case 2:
                self.navigator.goToPose(self.target_pose_by_waypoint_number[self.target_waypoint])
                self.target_waypoint += 1
                return

            case 3:
                self.navigator.goToPose(self.target_pose_by_waypoint_number[self.target_waypoint])
                self.target_waypoint += 1
                return

            case 4:
                self.navigator.goToPose(self.target_pose_by_waypoint_number[self.target_waypoint])
                self.target_waypoint = 1
                return

            # If an exact match is not confirmed, this last case will be used if provided
            case _:
                self.target_waypoint = 1
                return

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    def activate_waypoint_following(self, request, response):
        if (request.data):
            self.get_logger().info('Incoming request to activate waypoint following!')
        else:
            self.get_logger().info('Incoming request to deactivate waypoint following!')

        self.waypoint_following_is_activated = request.data
        response.success = True
