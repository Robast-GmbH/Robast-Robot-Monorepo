from pickletools import uint8
import rclpy
import requests
import time
import json
import numpy as np
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import UInt8, UInt8MultiArray
from datetime import datetime

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from communication_interfaces.action import DrawerUserAccess
from communication_interfaces.msg import DrawerAddress


class RobotStatus(Enum):
    is_running = 1
    is_pausing = 2
    is_homing = 3


NUM_OF_DRAWERS = 5


class SimpleFleetmanagement(Node):

    def __init__(self):
        super().__init__('simple_fleetmanagement')

        # the interval in which the data gets pulled from backend interface.
        self.backend_polling_intervall = 5  # seconds
        # the radius in which the robot defines the goal reached.
        self.goal_reach_epsilon = 0.4  # meter
        # the time which is acceptable for no new feedback message to be sent after that the navigation will be reset.
        self.error_reset_time = 20  # seconds

        # order queue is a array that contains tuples with (order_id, nav_goal_within_room nav_goal_door_bell)
        self.order_queue = []
        # waypoint_queue contains the nav goals of the door_bell and the pose within the room for each order
        self.waypoint_queue = []
        self.last_feedback = datetime.now()

        self.robot_status_publisher = self.create_publisher(
            UInt8,
            'robot_status',
            10)
        self.robot_status_subscription = self.create_subscription(
            UInt8,
            'robot_status',
            self.set_robot_status_callback,
            10)

        self.drawer_refill_status_publisher = self.create_publisher(
            UInt8MultiArray,
            'drawer_refill_status',
            10)

        self.drawer_gate_action_client = ActionClient(self, DrawerUserAccess, "control_drawer")

        self.setup_navigator()

        # Repeted call for new tasks
        self.get_logger().info("The simple fleetmanagement is running")
        self.timer = self.create_timer(self.backend_polling_intervall, self.backend_polling_callback)

    def set_robot_status_callback(self, msg):
        if (msg.data == RobotStatus.is_homing):
            self.robot_status = RobotStatus.is_homing
            self.get_logger().info("Setting Robot status to homing!")
        else:
            if len(self.target_pose_by_waypoint_number) > 1:
                if(msg.data == RobotStatus.is_running):
                    self.robot_status = RobotStatus.is_running
                    self.get_logger().info("Activating the waypoint following. Number of current waypoints: " +
                                           str(len(self.target_pose_by_waypoint_number)))
                elif (msg.data == RobotStatus.is_pausing):
                    self.robot_status = RobotStatus.is_pausing
                    self.get_logger().info("Deactivating the waypoint following. Number of current waypoints: " +
                                           str(len(self.target_pose_by_waypoint_number)))
            else:
                self.robot_status = RobotStatus.is_pausing
                self.get_logger().info("Deactivating the waypoint following because the number of current waypoints is below 1. Number of current waypoints: " +
                                       str(len(self.target_pose_by_waypoint_number)))

    def setup_navigator(self):
        self.navigator = BasicNavigator()
        self.header_frame_id = 'map'
        initial_pose_x = 0.0
        initial_pose_y = 0.0
        initial_pose_yaw = 3.14
        self.set_initial_pose(initial_pose_x, initial_pose_y, initial_pose_yaw)

        self.target_pose_by_waypoint_number = {}
        self.get_map_positions()

        self.robot_status = RobotStatus.is_pausing  # Waypoint following needs to be activated via the corresponding topic

        self.waypoint_following_is_activated = False

    def set_initial_pose(self, initial_pose_x, initial_pose_y, initial_pose_yaw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = self.header_frame_id
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = initial_pose_x
        initial_pose.pose.position.y = initial_pose_y
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, initial_pose_yaw)
        initial_pose.pose.orientation.x = qx
        initial_pose.pose.orientation.y = qy
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        self.navigator.setInitialPose(initial_pose)

    def set_waypoint(self, waypoint_id, goal_pose_x, goal_pose_y, goal_pose_yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.header_frame_id
        # TODO: We might want to update the time stamp right before sending the goal?
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_pose_x
        goal_pose.pose.position.y = goal_pose_y
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, goal_pose_yaw)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        self.target_pose_by_waypoint_number[waypoint_id] = goal_pose

    def backend_polling_callback(self):
        self.get_robot_status()
        self.get_drawer_open_status()
        self.get_drawer_refilling_status()

    def get_drawer_open_status(self):
        api_url = "http://localhost:8000/drawer/open"
        self.checkConnection(api_url)
        response = requests.get(api_url)
        if (response.status_code == 200):
            drawer_controller_id = response.json()
            if (drawer_controller_id > 0):
                goal_msg = DrawerUserAccess.Goal()
                drawer_adress = DrawerAddress()
                drawer_adress.drawer_controller_id = drawer_controller_id
                drawer_adress.drawer_id = 1  # there can be either 1 or 2 drawers per module, for now its only 1 drawer
                goal_msg.drawer_address = drawer_adress
                goal_msg.state = 1
                self.drawer_gate_action_client.wait_for_server()
                self.drawer_gate_action_client.send_goal_async(goal_msg)  # TODO: Do something with the result
                response = requests.delete(api_url)
                if(response.status_code == 200):
                    self.get_logger().info(
                        "Opening of drawer with drawer_controller_id {0} was successfull!".format(drawer_controller_id))
                else:
                    self.get_logger().warning(
                        "Drawer_controller_id {0} could not be deleted from drawer/open/ request!".format(drawer_controller_id))
            elif (drawer_controller_id > NUM_OF_DRAWERS):
                self.get_logger().warning(
                    "Request for opening drawer with invalid drawer_controller_id: {0}".format(drawer_controller_id))

        else:
            self.get_logger().warning('Response code from api_url ' + str(api_url) + ' is ' + str(response.status_code))

    def get_robot_status(self):
        api_url = "http://localhost:8000/robot/status"
        self.checkConnection(api_url)
        response = requests.get(api_url)
        if(response.status_code == 200):
            robot_status = response.json()
            msg = UInt8()
            msg.data = robot_status
            self.robot_status_publisher.publish(msg)
        else:
            self.get_logger().warning('Response code from api_url ' + str(api_url) + ' is ' + str(response.status_code))

    def get_drawer_refilling_status(self):
        api_url = "http://localhost:8000/drawer/empty"
        self.checkConnection(api_url)
        response = requests.get(api_url)
        if(response.status_code == 200):
            drawer_controller_ids_to_be_refilled = []
            response_data = response.json()
            for item in response_data:
                drawer_controller_id = item["drawer_controller_id"]
                drawer_controller_ids_to_be_refilled.append(drawer_controller_id)
            msg = UInt8MultiArray()
            msg.data = drawer_controller_ids_to_be_refilled
            self.drawer_refill_status_publisher.publish(msg)
        else:
            self.get_logger().warning('Response code from api_url ' + str(api_url) + ' is ' + str(response.status_code))

    def get_map_positions(self):
        api_url = "http://localhost:8000/map_positions"
        self.checkConnection(api_url)
        response = requests.get(api_url)
        if(response.status_code == 200):
            response_data = response.json()
            for item in response_data:
                waypoint_id = item["id"]
                goal_pose_x = item["x"]
                goal_pose_y = item["y"]
                goal_pose_yaw = item["t"]
                self.set_waypoint(self, waypoint_id, goal_pose_x, goal_pose_y, goal_pose_yaw)
                self.get_logger().info(
                    'New waypoint with waypoint_id {0}, x = {1}, y = {2}, yaw = {3} was added to target waypoints!'.format(waypoint_id, goal_pose_x, goal_pose_y, goal_pose_yaw))
        else:
            self.get_logger().warning('Response code from api_url ' + str(api_url) + ' is ' + str(response.status_code))

    def create_waypoints_for_task(self, order_id):
        nav_goal_within_room = self.order_queue[0][1]
        nav_goal_door_bell = self.order_queue[0][2]

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
