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

        self.fleetstate_sub = self.create_subscription(
            FleetDataRobotState,
            'robot_state',
            self.listen_to_fleet_state,
            10)
        
        self.taskstate_sub = self.create_subscription(
            FleetDataTaskState,
            'task_state',
            self.listen_to_task_state,
            10)

        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query API server")
            self.connected = True
        else:
            print("Unable to query API server")
    
    def listen_to_fleet_state(self, msg:FleetDataRobotState):
        self.LastFleetUpdate= datetime.now()
        self.Robotstates[msg.robot_name]= msg
    
    def listen_to_task_state(self, msg:FleetDataTaskState):
        self.LastFleetUpdate= datetime.now()
        if(msg.task_idstartswith(self.task_prefix)):
            self.Taskstates[msg.task_id]= msg 
        self.Taskstates={key: value for key, value in self.Taskstates.items() if not( int(key.lstrip(self.task_prefix)) <= self.task_id-5)}


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
        self.task_id+=1
        action = {"step":1, "type":"NAVIGATION", "finished":False, "action":{ "pose": {"x":pose.x, "y":pose.y, "z":0.0 },"yaw": pose.yaw} }
        task_msg={"robot":{ "robot_name":robot_name, "fleet_name": self.fleet_name}, "task_id":self.task_prefix+self.task_id,"actions":[ action]}
        response = requests.get(self.server_url+"/task", data= json.dumps(task_msg))
        return response.status_code!= 200

    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        self.task_id+=1
        action = process
         
        task_msg={"robot":{ "robot_name":robot_name, "fleet_name": self.fleet_name}, "task_id":"rmf"+self.task_id,"actions":[ action]}
        response = requests.get(self.server_url+"/task", data= json.dumps(task_msg))
        return response.status_code!= 200
     

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        param = {'robot': {'fleet_name':self.fleetname, 'robot_name':robot_name}}
        requests.post(self.server_url+"/settings/navigation/pause", json = param)
        return False

    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''


        return 0.0

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
       #return self.Robotstates[msg.robot_name].battery_percent/100
        return None