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
"""
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
"""

import requests
import nudged


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(
        self,
        prefix: str,
        user: str,
        password: str,
    ):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False
        # Transforms
        rmf_coordinates = [
            [32.1081, -5.9120],
            [23.1892, -9.9892],
            [16.8186, -6.1668],
            [6.3197, -9.1738],
        ]
        robot_coordinates = [[8.2, 3.0], [-0.65, -0.65], [-6.8, 2.9], [-16.9, -0.25]]
        self.transforms = {
            "rmf_to_robot": nudged.estimate(rmf_coordinates, robot_coordinates),
            "robot_to_rmf": nudged.estimate(robot_coordinates, rmf_coordinates),
        }
        self.transforms["orientation_offset"] = self.transforms[
            "rmf_to_robot"
        ].get_rotation()
        self.map_offset = [466.0, -179.4]
        self.map_scale = 0.05

    def check_connection(self):
        """Return True if connection to the robot API server is successful"""
        response = requests.get(f"{self.prefix}/is_ros_bridge_connected").json()
        return response["is_connected"]

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered"""
        pose = requests.get(f"{self.prefix}/robot_pos").json()
        x, y = self.transforms["robot_to_rmf"].transform([pose["x"], pose["y"]])
        return [x, y, pose["z"]]

    def navigate(
        self, robot_name: str, cmd_id: int, pose, map_name: str, speed_limit=0.0
    ):
        """Request the robot to navigate to pose:[x,y,theta] where x, y and
        and theta are in the robot's coordinate convention. This function
        should return True if the robot has accepted the request,
        else False"""
        goal_pose = [
            (pose[0] - self.map_offset[0]) * self.map_scale,
            (pose[1] - self.map_offset[1]) * self.map_scale,
            pose[2],
        ]

        requests.post(
            f"{self.prefix}/goal_pose?x={goal_pose[0]}&y={goal_pose[1]}&z={goal_pose[2]}"
        )
        return True

    def requires_replan(self, robot_name: str):
        return False

    def stop(self, robot_name: str, cmd_id: int):
        """Command the robot to stop.
        Return True if robot has successfully stopped. Else False"""
        requests.post(f"{self.prefix}/cancel_goal")
        return True

    def navigation_remaining_duration(self, robot_name: str, cmd_id: int):
        """Return the number of seconds remaining for the robot to reach its
        destination"""
        response = requests.get(f"{self.prefix}/remaining_nav_time").json()
        return response["remaining_seconds"]

    def navigation_completed(self, robot_name: str, cmd_id: int):
        """Return True if the robot has successfully completed its previous
        navigation request. Else False."""
        response = requests.get(f"{self.prefix}/is_navigating").json()
        return not response["is_navigating"]

    def battery_soc(self, robot_name: str):
        """Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered"""
        return 1.0

    def start_process(self, robot_name: str, process: str, map_name: str):
        """Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if the robot has accepted the request, else False"""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True

    def process_completed(self, robot_name: str):
        """Return True if the robot has successfully completed its previous
        process request. Else False."""
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True
