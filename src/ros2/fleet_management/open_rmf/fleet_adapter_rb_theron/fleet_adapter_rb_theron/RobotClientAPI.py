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

        rmf_coordinates = [
            [10.77,-22.39],
            [10.63,-16.09],
            [11.42,-31.53],
            [7.38,-31.68],
        ]
        robot_coordinates = [
            [1.31,-6.45],
            [1.3,-0.04],
            [1.7,-15.6],
            [-1.82,-15.4],
        ]
        self.transforms = {
            "rmf_to_robot": nudged.estimate(rmf_coordinates, robot_coordinates),
            "robot_to_rmf": nudged.estimate(robot_coordinates, rmf_coordinates),
        }
        self.transforms["orientation_offset"] = self.transforms[
            "rmf_to_robot"
        ].get_rotation()

    def check_connection(self):
        """Return True if connection to the robot API server is successful"""
        response = requests.get(f"{self.prefix}/")
        return response.status_code == 200

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered"""
        pose = requests.get(f"{self.prefix}/robot_pos?robot_name={robot_name}").json()
        x, y = self.transforms["robot_to_rmf"].transform([pose["x"], pose["y"]])
        return [x, y, pose["z"]]

    def navigate(
        self, robot_name: str, cmd_id: int, pose, map_name: str, speed_limit=0.0
    ):
        """Request the robot to navigate to pose:[x,y,theta] where x, y and
        and theta are in the robot's coordinate convention. This function
        should return True if the robot has accepted the request,
        else False"""
        x, y = self.transforms["rmf_to_robot"].transform([pose[0], pose[1]])
        requests.post(
            f"{self.prefix}/goal_pose?robot_name={robot_name}&x={x}&y={y}&z={pose[2]}"
        )
        return True

    def requires_replan(self, robot_name: str):
        return False

    def stop(self, robot_name: str, cmd_id: int):
        """Command the robot to stop.
        Return True if robot has successfully stopped. Else False"""
        requests.post(f"{self.prefix}/cancel_goal?robot_name={robot_name}")
        return True

    def navigation_remaining_duration(self, robot_name: str, cmd_id: int):
        """Return the number of seconds remaining for the robot to reach its
        destination"""
        response = requests.get(f"{self.prefix}/remaining_nav_time?robot_name={robot_name}").json()
        return response["remaining_seconds"]

    def navigation_completed(self, robot_name: str, cmd_id: int):
        """Return True if the robot has successfully completed its previous
        navigation request. Else False."""
        response = requests.get(f"{self.prefix}/is_navigating?robot_name={robot_name}").json()
        return not response["is_navigating"]

    def battery_soc(self, robot_name: str):
        """Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered"""
        response = requests.get(f"{self.prefix}/battery_level?robot_name={robot_name}").json()
        return response["battery_level"]

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
        # TODO MPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return True
