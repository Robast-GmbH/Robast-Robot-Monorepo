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
from rclpy.node import Node
from rmf_fleet_msgs.msg._fleet_state import FleetState
from rmf_fleet_msgs.msg._mode_request import ModeRequest
from rmf_fleet_msgs.msg._destination_request import DestinationRequest
from rmf_fleet_msgs.msg._path_request import PathRequest
from rmf_fleet_msgs.msg._location import Location
from rmf_fleet_msgs.msg._robot_mode import RobotMode
from rmf_fleet_msgs.msg._fleet_state import FleetState
from dataclasses import dataclass
import requests

from datetime import datetime, timedelta



class RobastFleetAPI(Node):
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, fleetname: str, fleet_responce_interval: int, server_url: str ):# fleet_responce_interval is in minutes
        super().__init__("robast_fleet_api")
        self.fleet_responce_interval=fleet_responce_interval
        self.connected = False
        self.Robotstates=dict()
        self.LastFleetUpdate=None
        self.fleetname="ROBAST"

        self.fleetstate_sub = self.create_subscription(
            FleetState,
            'robot_state',
            self.listen_to_fleet_state,
            10)
        self.fleetstate_sub 

        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")
    
    def listen_to_fleet_state(self, msg:FleetState):
        self.LastFleetUpdate= datetime.now()
        self.Robotstates[msg.robot_name]= msg


    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        time_to_last_update= self.LastFleetUpdate-datetime.now()
        return (time_to_last_update.total_seconds/60) < self.fleet_responce_interval

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        
        robotstate=self.Robotstates.get(robot_name)
        if(robotstate is not None):
            return [robotstate.location.x, robotstate.location.y, robotstate.location.yaw ]
        return None

    def navigate(self, robot_name: str, pose, map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        param = {'robot': {'fleet_name':self.fleetname, 'robot_name':robot_name}}
        requests.post(self.server_url+"/settings/navigation/pause", json = param)
        return False

    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return 0.0

    def navigation_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
       #ToDo Torben: replace after implemented on the robo 
       #return self.Robotstates[msg.robot_name].battery_percent/100
        return None