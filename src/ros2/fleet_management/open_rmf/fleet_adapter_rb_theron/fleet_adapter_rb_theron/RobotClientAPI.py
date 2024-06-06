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
import math

TIMEOUT_DURATION_IN_SECONDS = 3


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, reference_coordinates):
        self.__prefix = prefix

        rmf_coordinates = reference_coordinates["rmf"]
        robot_coordinates = reference_coordinates["robot"]

        self.__transforms = {
            "rmf_to_robot": nudged.estimate(rmf_coordinates, robot_coordinates),
            "robot_to_rmf": nudged.estimate(robot_coordinates, rmf_coordinates),
        }
        self.__transforms["orientation_offset"] = self.__transforms[
            "rmf_to_robot"
        ].get_rotation()

    def check_connection(self):
        """Return True if connection to the robot API server is successful"""
        try:
            response = requests.get(
                f"{self.__prefix}/",
                timeout=TIMEOUT_DURATION_IN_SECONDS,
            )
            return response.status_code == 200
        except Exception as e:
            return False

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered"""
        try:
            pose = requests.get(
                f"{self.__prefix}/robot_pos?robot_name={robot_name}",
                timeout=TIMEOUT_DURATION_IN_SECONDS,
            ).json()
            x, y = self.__transforms["robot_to_rmf"].transform([pose["x"], pose["y"]])
            z = pose["z"] - self.__transforms["orientation_offset"]
            if z > math.pi:
                z -= 2 * math.pi
            return [x, y, z]
        # TODO(ane-robast): Add proper error handling -> RE-2187
        except Exception as e:
            return None

    def navigate(
        self, robot_name: str, cmd_id: int, pose, map_name: str, speed_limit=0.0
    ):
        """Request the robot to navigate to pose:[x,y,theta] where x, y and
        and theta are in the robot's coordinate convention. This function
        should return True if the robot has accepted the request,
        else False"""
        try:
            x, y = self.__transforms["rmf_to_robot"].transform([pose[0], pose[1]])
            z = pose[2] + self.__transforms["orientation_offset"]
            requests.post(
                f"{self.__prefix}/goal_pose?robot_name={robot_name}&x={x}&y={y}&z={z}",
                timeout=TIMEOUT_DURATION_IN_SECONDS,
            )
            return True
        # TODO(ane-robast): Add proper error handling -> RE-2187
        except Exception as e:
            return False

    def requires_replan(self, robot_name: str):
        return False

    def stop(self, robot_name: str, cmd_id: int):
        """Command the robot to stop.
        Return True if robot has successfully stopped. Else False"""
        try:
            requests.post(
                f"{self.__prefix}/cancel_goal?robot_name={robot_name}",
                timeout=TIMEOUT_DURATION_IN_SECONDS,
            )
            return True
        # TODO(ane-robast): Add proper error handling -> RE-2187
        except Exception as e:
            return False

    def navigation_remaining_duration(self, robot_name: str, cmd_id: int):
        """Return the number of seconds remaining for the robot to reach its
        destination"""
        try:
            response = requests.get(
                f"{self.__prefix}/remaining_nav_time?robot_name={robot_name}",
                timeout=TIMEOUT_DURATION_IN_SECONDS,
            ).json()
            return response["remaining_seconds"]
        # TODO(ane-robast): Add proper error handling -> RE-2187
        except Exception as e:
            return None

    def navigation_completed(self, robot_name: str, cmd_id: int):
        """Return True if the robot has successfully completed its previous
        navigation request. Else False."""
        try:
            response = requests.get(
                f"{self.__prefix}/is_navigating?robot_name={robot_name}",
                timeout=TIMEOUT_DURATION_IN_SECONDS,
            ).json()
            return not response["is_navigating"]
        # TODO(ane-robast): Add proper error handling -> RE-2187
        except Exception as e:
            return False

    def battery_soc(self, robot_name: str):
        """Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered"""
        try:
            response = requests.get(
                f"{self.__prefix}/battery_level?robot_name={robot_name}",
                timeout=TIMEOUT_DURATION_IN_SECONDS,
            ).json()
            return response["battery_level"]
        # TODO(ane-robast): Add proper error handling -> RE-2187
        except Exception as e:
            return None

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
