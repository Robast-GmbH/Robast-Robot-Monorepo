# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rclpy
import rclpy.node
from rmf_fleet_msgs.msg._fleet_state import FleetState
from rmf_fleet_msgs.msg._mode_request import ModeRequest
from rmf_fleet_msgs.msg._destination_request import DestinationRequest
from rmf_fleet_msgs.msg._path_request import PathRequest
from rmf_fleet_msgs.msg._location import Location
from rmf_fleet_msgs.msg._robot_mode import RobotMode


from datetime import datetime, timedelta


class freeFleetAPI:
    def __init__(self, fleet_name: str,
                 prefix: str,
                 responce_interval: float,
                 node: rclpy.node.Node):

        self.prefix = prefix
        self.fleet_name = fleet_name
        self.responce_interval = responce_interval
        self.connected = False
        t = datetime.now() + timedelta(seconds=responce_interval*2)
        self.last_update = t
        self.ros_node = node
        self.robots = []

        self.fleet_state_subscription = self.ros_node.create_subscription(
            FleetState,
            '/fleet_states',
            self.update_robot_states_callback,
            10)
        self.fleet_state_subscription
        self.publisher_mode_request = self.ros_node.create_publisher(ModeRequest,
                                                                     '/robot_mode_requests',
                                                                     10)
        self.publisher_destination_request = self.ros_node\
                                                 .create_publisher(DestinationRequest,
                                                                   '/robot_destination_requests',
                                                                   10)

        self.publisher_Path_request = self.ros_node.create_publisher(PathRequest,
                                                                     '/robot_path_requests',
                                                                     10)

        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")

    def update_robot_states_callback(self, msg: FleetState):

        if(msg.name is not self.fleet_name):
            "log error"
            return
        self.last_update = datetime.now()
        self.robots = []
        for robot_state in msg.FleetState.robots:
            robot = {}
            robot.name = robot_state.name
            robot.model = robot_state.model
            robot.task_id = robot_state.task_id
            robot.battery_percent = robot_state.battery_percent
            robot.x = robot_state.location.x
            robot.y = robot_state.location.y
            robot.theta = robot_state.location.yaw
            self.robots.append(robot)

    def check_connection(self):
        """Return True if connection to the robot API server is successful."""
        time_since_last_update = self.last_update - datetime.now()
        return time_since_last_update.total_seconds() < self.responce_interval

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or \
        None if any errors are encountered."""
        if(not self.check_connection()):
            robot_state = self.find_robot_state(robot_name)
            if robot_state is not None:
                return [robot_state.x, robot_state.y, robot_state.theta]
        return None

    def navigate(self, robot_name: str, pose, map_name: str):
        """Request the robot to navigate to pose:[x,y,theta] where x, y and \
        and theta are in the robot's coordinate convention. This function \
        should return True if the robot has accepted the request, \
        else False."""
        if(not self.check_connection()):
            robot_state = self.find_robot_state(robot_name)
            if robot_state is not None:
                msg = DestinationRequest()
                msg.data.fleet_name = self.fleet_name
                msg.data.robot_name = robot_name
                location = Location()
                location.x = pose[0]
                location.y = pose[1]
                location.yaw = pose[2]
                msg.data.destination = location
                self.publisher_destination_request.publish(msg)
                return True
        return False

    def start_process(self, robot_name: str, process: str, map_name: str):
        """Request the robot to begin a process. This is specific to the robot\
        and the use case. For example, load/unload a cart for Deliverybot\
        or begin cleaning a zone for a cleaning robot.\
        Return True if the robot has accepted the request, else False."""
        if(not self.check_connection()):
            robot_state = self.find_robot_state(robot_name)
            if robot_state is not None:
                msg = ModeRequest()
                msg.fleet_name = self.fleet_name
                msg.robot_name = robot_name
                msg.mode = RobotMode.MODE_MOVING
                msg.task_id = robot_state.task_id
                return True
            return False

    def stop(self, robot_name: str):
        """Command the robot to stop.\
        Return True if robot has successfully stopped. Else False."""
        if(not self.check_connection()):
            robot_state = self.find_robot_state(robot_name)
            if robot_state is not None:
                msg = ModeRequest()
                msg.fleet_name = self.fleet_name
                msg.robot_name = robot_name
                msg.mode = RobotMode.MODE_PAUSED
                msg.task_id = robot_state.task_id
                return True
        return False

    def navigation_remaining_duration(self, robot_name: str):
        """Return the number of seconds remaining for the robot to reach its \
        destination."""
        # ------------------------ #
        # ToDo:to be defined later is not importend at the moment
        # ------------------------ #
        return 0.0

    def navigation_completed(self, robot_name: str):
        """Return True if the robot has successfully completed its previous\
        navigation request. Else False."""
        # ------------------------ #
        # ToDo:to be defined later is not importend at the moment
        # ------------------------ #
        return False

    def process_completed(self, robot_name: str):
        """Return True if the robot has successfully completed its previous\
        process request. Else False."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def battery_soc(self, robot_name: str):
        """Return the state of charge of the robot as a value between 0.0\
        and 1.0. Else return None if any errors are encountered."""
        robot_state = self.find_robot_state(robot_name)
        if robot_state is not None:
            return robot_state.battery_percent
        return None

    def find_robot_state(self, robot_name: str):
        robot_state = list(filter(lambda x: x.name == robot_name, self.robots))
        if len(robot_state) != 1:
            return None
        return robot_state[0]
