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
from fleet_interfaces.msg import FleetDataTaskState
from fleet_interfaces.msg import FleetDataRobotState

from dataclasses import dataclass
import requests
import json

from datetime import datetime, timedelta



class RobastFleetAPI(Node):
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, fleetname: str, fleet_responce_interval: int, server_url: str ):# fleet_responce_interval is in minutes
        super().__init__("robast_fleet_api")
        self.fleet_responce_interval=fleet_responce_interval
        self.connected = False
        self.server_url=server_url
        self.Robotstates=dict()
        self.Taskstates=dict()
        self.LastFleetUpdate=None
        self.fleetname="ROBAST"
        self.task_id=0
        self.task_prefix="rmf"

        

        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")
    
  

    def check_connection(self):
        ''' Return True if connection to the robot API server is successful'''
        responce = requests.get(self.server_url+"/robots")
        return responce.status_code==200

    def position(self, robot_name: str):
        ''' Return [x, y, theta] expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        responce = requests.get(self.server_url+"/robots/"+robot_name)
        if responce.status_code!=200:
            return None
        else:
            robot_data= responce.json()
            return [robot_data["x_pose"],robot_data["y_pose"], robot_data["yaw_pose"]]

    def navigate(self, robot_name: str, pose, map_name: str):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False'''
        self.task_id+=1
        pose = { "x":pose.x, "y": pose.y, "z": 0.0}
        target = { "pose": pose, "yaw": pose.theta} 
        navigation_msg={ "target":target, "owner_id":2 }
        response = requests.get(self.server_url+"/robots/"+robot_name+"/navigate", data= json.dumps(navigation_msg))
        return response.status_code== 200

    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        #ToDo Torben: implement laster    
        return False
     

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        response=requests.get(self.server_url+"robots/pause_resume?robot_name="+robot_name+"&fleet_name="+self.fleetname+"&pause="+"True")
        return response.status_code== 200

    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        responce = requests.get(self.server_url+"/robots/"+robot_name)
        if responce.status_code!=200:
            return None
        else:
            robot_data= responce.json()
            return robot_data["status"].spit(":")[1]

    def navigation_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            navigation request. Else False.'''
        return self.Robotstates[robot_name].task_id!="rmf"+self.task_id

    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed its previous
            process request. Else False.'''
        return self.Robotstates[robot_name].task_id!="rmf"+self.task_id

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
       #ToDo Torben: replace after implemented on the robo 
        responce = requests.get(self.server_url+"/robots/"+robot_name)
        if responce.status_code!=200:
            return None
        else:
            robot_data= responce.json()
            return robot_data["battery_level"]