from pickletools import uint8
from xmlrpc.client import Boolean
import rclpy
import requests
import time
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from . import HH_Nav_Statemachine
from std_msgs.msg import UInt8, UInt8MultiArray
from datetime import datetime
from typing import List


# from . import state_machine
from . import web_interface
from . import drawer_helper_module

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
try:
    from communication_interfaces.action import DrawerUserAccess
except ModuleNotFoundError as err:
    print(err)
from communication_interfaces.msg import DrawerAddress

from . import parameters_module as static_params


class SimpleFleetmanagement(Node):

    def __init__(self):
        super().__init__('simple_fleetmanagement')

        self.backend_polling_intervall = 5  # seconds
        self.navigation_update_interval = 0.5  # seconds
        self.goal_reach_epsilon = 0.4  # meter
        self.error_reset_time = 20  # seconds

        self.order_queue = []
        self.waypoint_queue = []
        self.last_feedback = datetime.now()

        self.initialize_ros_robot_status_communication()

        self.initialize_ros_robot_refill_status_subscription()
        self.initialize_web_interface()
        self.setup_navigator()
        self.initialize_statemachine()

        self.setup_drawer_interaction()
        self.get_logger().info("The simple fleetmanagement is running")
        self.start_web_interface()
        self.start_statemachine()

    def setup_drawer_interaction(self):
        self.drawer_gate_action_client = ActionClient(self, DrawerUserAccess, "control_drawer")

    def initialize_ros_robot_refill_status_subscription(self):
        self.drawer_refill_status_publisher = self.create_publisher(
            UInt8MultiArray,
            'drawer_refill_status',
            10)

    def initialize_ros_robot_status_communication(self):
        self.robot_status_publisher = self.create_publisher(
            UInt8,
            'robot_status',
            10)
        self.robot_status_subscription = self.create_subscription(
            UInt8,
            'robot_status',
            self.set_robot_status_callback,
            10)

    def initialize_web_interface(self):
        functions_for_web = {
            "get_drawer_open_ros_function": self.get_drawer_open_ros_function,
            "publish_robot_status": self.publish_robot_status,
            "publish_drawer_refill_status": self.publish_drawer_refill_status,
            "set_waypoint": self.set_waypoint
        }
        self._webInterface = web_interface.WebInterface("http://localhost:8000", functions_for_web)

    def start_web_interface(self):
        self._backend_polling_timer = self.create_timer(
            self.backend_polling_intervall, self._webInterface.backend_polling)

    def initialize_statemachine(self):
        # self.navigation_trigger = self.create_timer(
        #     self.navigation_update_interval, self.handle_waypoint_follow_callback)
        # self.state_machine_state = 1
        # pass
        functions_fo_statemachine = {
            "check_status": self.check_status,
            "is_any_drawer_open": self.is_any_drawer_open,
            "navigator_cancel_task": self.navigator_cancel_task,
            "get_waypoints_by_id": self.get_waypoints_by_id,
            "navigate_to_pose": self.navigate_to_pose,
            "open_drawer": self.open_drawer
        }
        self.HH_state_machine = HH_Nav_Statemachine.HHStateMachine(functions_by_functionname=functions_fo_statemachine)

    def start_statemachine(self):
        self.HH_state_machine.start()

    def set_robot_status_callback(self, msg):
        # self.get_logger().info("Received robot status: {1}", str(msg.data))
        if (static_params.RobotStatus(msg.data) == static_params.RobotStatus.is_homing):
            self.robot_status = static_params.RobotStatus.is_homing
            self.get_logger().info("Setting Robot status to homing!")
        else:
            if len(self.target_pose_by_waypoint_id) > 1:
                if(static_params.RobotStatus(msg.data) == static_params.RobotStatus.is_running):
                    self.robot_status = static_params.RobotStatus.is_running
                    self.get_logger().info("Activating the waypoint following. Number of current waypoints: " +
                                           str(len(self.target_pose_by_waypoint_id)))
                elif (static_params.RobotStatus(msg.data) == static_params.RobotStatus.is_pausing):
                    self.robot_status = static_params.RobotStatus.is_pausing
                    self.get_logger().info("Deactivating the waypoint following. Number of current waypoints: " +
                                           str(len(self.target_pose_by_waypoint_id)))
            else:
                self.robot_status = static_params.RobotStatus.is_pausing
                self.get_logger().info("Deactivating the waypoint following because the number of current waypoints is below 1. Number of current waypoints: " +
                                       str(len(self.target_pose_by_waypoint_id)))

    def handle_waypoint_follow_callback(self):
        if (static_params.RobotStatus(self.robot_status) == static_params.RobotStatus.is_running):
            if (self.navigator.isTaskComplete()):
                self.state_machine()
        else:
            self.get_logger().debug("Robot is Paused")
            self.navigator.cancelTask()

    def setup_navigator(self):
        self.navigator = BasicNavigator()
        self.header_frame_id = 'map'
        initial_pose_x = 0.0
        initial_pose_y = 0.0
        initial_pose_yaw = 3.14
        self.set_initial_pose(initial_pose_x, initial_pose_y, initial_pose_yaw)

        self.target_pose_by_waypoint_id = {}
        self._webInterface.set_navigator_waypoints_from_backend()

        # Waypoint following needs to be activated via the corresponding topic
        self.robot_status = static_params.RobotStatus.is_pausing

        self.waypoint_following_is_activated = True

    def navigate_to_pose(self, waypoint_id):
        self.navigator.goToPose(self.target_pose_by_waypoint_id[waypoint_id])

    def navigator_cancel_task(self):
        self.navigator.cancelTask()

    def check_navigator_status(self):
        return self.navigator.getResult()

    def open_drawer(self, number):
        pass

    def check_status(self):
        return self.robot_status

    def is_drawer_open(self, id):
        pass

    def is_any_drawer_open(self) -> Boolean:
        pass

    def create_pose(self, pose_x, pose_y, pose_yaw) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.header_frame_id
        # TODO: We might want to update the time stamp right before sending the goal?
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, pose_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def set_initial_pose(self, initial_pose_x, initial_pose_y, initial_pose_yaw):
        initial_pose = self.create_pose(initial_pose_x, initial_pose_y, initial_pose_yaw)
        self.navigator.setInitialPose(initial_pose)

    def set_waypoint(self, waypoint_id, waypoint_pose_x, waypoint_pose_y, waypoint_pose_yaw):
        goal_pose = self.create_pose(waypoint_pose_x, waypoint_pose_y, waypoint_pose_yaw)
        self.target_pose_by_waypoint_id[waypoint_id] = goal_pose
        self.get_logger().info(
            'New waypoint with waypoint_id {0}, x = {1}, y = {2}, yaw = {3} was added to target waypoints!'.format(waypoint_id, waypoint_pose_x, waypoint_pose_y, waypoint_pose_yaw))

    def get_waypoints_by_id(self):
        return self.target_pose_by_waypoint_id

    def get_drawer_open_ros_function(self, goal_msg):
        self.drawer_gate_action_client.wait_for_server()
        self.drawer_gate_action_client.send_goal_async(goal_msg)  # TODO: Do something with the result

    def publish_robot_status(self, status):
        msg = UInt8()
        msg.data = status
        self.robot_status_publisher.publish(msg)

    def publish_drawer_refill_status(self, list_of_drawers_to_be_refilled: List[int]):
        msg = UInt8MultiArray()
        msg.data = list_of_drawers_to_be_refilled
        self.drawer_refill_status_publisher.publish(msg)

    def create_waypoints_for_task(self, order_id):
        nav_goal_within_room = self.order_queue[0][1]
        nav_goal_door_bell = self.order_queue[0][2]

    def state_machine(self):
        match self.state_machine_state < len(self.target_pose_by_waypoint_id):
            case 1:
                self.get_logger().info("navigate to waypoint {1}", self.state_machine_state)
                self.navigator.goToPose(self.target_pose_by_waypoint_id[self.state_machine_state])
                self.state_machine_state += 1
                return
            # If an exact match is not confirmed, this last case will be used if provided
            case _:
                self.get_logger().info("Navigation loop finised. Will restart now")
                self.state_machine_state = 1
                return

    # def send_nav_goal(self):
    #     if len(self.order_queue) > 0:

    #         self.get_nav_goal(nav_goal_within_room)

    # def get_nav_goal(self, goal):
    #     goal_msg = NavigateToPose.Goal()
    #     goal_msg.pose.header.frame_id = "map"
    #     goal_msg.pose.pose.position.x = float(nav_goal_within_room["x"]) / 100
    #     goal_msg.pose.pose.position.y = float(nav_goal_within_room["y"]) / -100
    #     self.get_logger().info('Navigating to goal: ' + str(goal_msg.pose.pose.position.x) + ' ' +
    #                            str(goal_msg.pose.pose.position.y) + ' for order ' + str(order_id))

    #     def order_feedback_callback(feedback_msg): return self.feedback_callback(order_id, feedback_msg)
    #     self.send_goal_future = self.nav_to_pose_client.send_goal_async(
    #         goal_msg, feedback_callback=order_feedback_callback)

    #     # goal_response_callback is called before the task is finished (don't know why).
    #     # So better use the feedback topic abd check distance_remaining
    #     def order_goal_response_callback(a): return self.goal_response_callback(order_id, a)
    #     # self.send_goal_future.add_done_callback(order_goal_response_callback)

    def feedback_callback(self, order_id, feedback_msg):
        feedback = feedback_msg.feedback
        self.last_feedback = datetime.now()
        if (feedback.distance_remaining < self.goal_reach_epsilon):
            self.set_order_to_finished(order_id)
        # else:
        #    self.get_logger().debug('Received feedback: {0}'.format(feedback.distance_remaining))

    def goal_response_callback(self, order_id, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        def order_get_result_callback(a): return self.get_result_callback(order_id, a)
        self._get_result_future.add_done_callback(order_get_result_callback)

    # def get_result_callback(self, order_id, future):
    #     result = future.result().result
    #     self.get_logger().info('Result: {0}'.format(result))
    #     self.set_order_to_finished(order_id)

    # def set_order_to_finished(self, order_id):
    #     # Send update request with finished=true
    #     response = self.update_order(order_id, "finished", True)

    #     if response.status_code == 200:
    #         initial_size = len(self.order_queue)
    #         # Remove finished task from queue
    #         self.order_queue = list(filter(lambda order_item: order_item[0] != order_id, self.order_queue))

    #         if len(self.order_queue) < initial_size:
    #             self.get_logger().info('Order with order_id {0} is finished!'.format(order_id))
    #             # Send next nav_goal to robot
    #             self.send_nav_goal()
    #     else:
    #         self.get_logger().info('Server does not respond. Order ' + str(order_id) + ' could not be finished.')

    # def update_order(self, order_id, property, value):
    #     url = "http://backend:8000/orders/" + str(order_id)
    #     self.checkConnection(url)

    #     order_json = self.get_order(url)
    #     order_json[property] = value

    #     response = requests.put(url, data=json.dumps(order_json))
    #     return response

    # def does_queue_contains_order(self, order_id):
    #     for nav_goal_by_order_id in self.order_queue:
    #         if order_id == nav_goal_by_order_id[0]:
    #             return True
    #     return False

    def checkConnection(self, url):
        noConnection = True
        errorSent = False

        while noConnection:
            try:
                # self.get_logger().info("test")
                response = requests.get(url)
                noConnection = False
            except requests.exceptions.RequestException:
                if not errorSent:
                    self.get_logger().info("connection to " + url + " failed.")
                    errorSent = True
                time.sleep(1)  # if fixed in use wait until

        if errorSent:
            self.get_logger().info("connection established.")
        return url

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


def main(args=None):
    rclpy.init(args=args)
    fleetmanagement = SimpleFleetmanagement()
    rclpy.spin(fleetmanagement)
    fleetmanagement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
