from ast import Try
from logging import error
from multiprocessing import get_logger
import requests
from . import parameters_module as params
from . import web_module
from . import drawer_helper_module
from typing import Dict

try:
    from communication_interfaces.action import DrawerUserAccess
    from communication_interfaces.msg import DrawerAddress
except ModuleNotFoundError as err:
    print(err)


class WebInterface:
    def __init__(self, base_url: str, functions):
        self.base_url = base_url
        self.functions = functions

    # checke nicht was das machen soll TODO refactor

    def get_drawer_open_status(self):
        api_url = self.base_url+"/drawer/open"
        response = web_module.getDataFromServer(api_url)
        if (response != None):
            drawer_controller_id = response.json()
            if (drawer_controller_id > 0):
                goal_msg = DrawerUserAccess.Goal()
                drawer_address = DrawerAddress()
                drawer_address.drawer_controller_id = drawer_controller_id
                drawer_address.drawer_id = 1  # there can be either 1 or 2 drawers per module, for now its only 1 drawer
                goal_msg.drawer_address = drawer_address
                goal_msg.state = 1
                try:
                    self.functions["get_drawer_open_ros_function"](goal_msg)
                except:
                    print("get_drawer_open_ros_function im web interface failed")
                response = requests.delete(api_url)
                if(response.status_code == 200):
                    get_logger().info(
                        "Opening of drawer with drawer_controller_id {0} was successfull!".format(drawer_controller_id))
                else:
                    get_logger().warning(
                        "Drawer_controller_id {0} could not be deleted from drawer/open/ request!".format(drawer_controller_id))
            elif (drawer_controller_id > params.NUM_OF_DRAWERS):
                get_logger().warning(
                    "Request for opening drawer with invalid drawer_controller_id: {0}".format(drawer_controller_id))

    def get_robot_status(self):
        response = web_module.getDataFromServer(self.base_url + "/robot/status")
        if(response != None):
            robot_status = response.json()
            try:
                self.functions["publish_robot_status"](robot_status)
            except:
                print("publish_robot_status im web interface failed")

    def get_drawer_refilling_status(self):
        response = web_module.getDataFromServer(self.base_url + "/drawer/empty")
        if(response != None):
            drawer_controller_ids_to_be_refilled = []
            response_data = response.json()
            drawer_controller_ids_to_be_refilled = drawer_helper_module.get_list_of_drawer_ids(response_data)
            try:
                self.functions["publish_drawer_refill_status"](drawer_controller_ids_to_be_refilled)
            except:
                print("publish_drawer_refill_status im web interface failed")

    def set_navigator_waypoints_from_backend(self):
        response = web_module.getDataFromServer(self.base_url + "/map_positions")
        if(response != None):
            for waypoint in response.json():
                try:
                    self.functions["add_waypoint"](waypoint["id"], waypoint["x"], waypoint["y"], waypoint["t"])
                except:
                    print("add_waypoint im web interface failed" + str(waypoint))

    def backend_polling(self):
        self.get_robot_status()
        self.get_drawer_open_status()
        self.get_drawer_refilling_status()
