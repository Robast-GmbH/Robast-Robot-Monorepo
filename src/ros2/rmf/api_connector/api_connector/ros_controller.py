
from rclpy.node import Node
from rclpy.action import ActionClient
from . import web_module
from logging import error
from multiprocessing import get_logger
import threading
import requests


class ros_controller(Node):

    def __init__(self, api_url, polling_interval):
        super().__init__('ros_controller')
        self.base_url = api_url
        self.polling_timer = self.create_timer(polling_interval, self.api_polling_callback)

    def api_polling_callback(self):
        self.get_robot_status()
        self.get_drawer_open_status()
    
    def get_robot_status(self):
        response = web_module.getDataFromServer(self.base_url + "/robot/status")
        if(response != None):
            robot_status = response.json()
            #ToDo Torben: update the status
    
    def get_drawer_open_status(self):
        api_url = self.base_url+"/drawer/open"
        response = web_module.getDataFromServer(api_url)
        if (response != None):
            drawer_controller_id = response.json()
            if (drawer_controller_id > 0):
                #request for opening the drawer
                self.delete_request(api_url)
            elif(drawer_controller_id != -1):
                get_logger().warning(
                    "Request for opening drawer with invalid drawer_controller_id: {0}".format(drawer_controller_id))

    def delete_request(self, api_url):
        response = requests.delete(api_url)
        if(response.status_code == 200):
            get_logger().info(
                "Deleting request for {0} was successfull!".format(api_url))
        else:
            get_logger().warning(
                "Deleting request for {0} was NOT successfull!".format(api_url))
