
from rclpy.node import Node
from rclpy.action import ActionClient
from . import web_module
from logging import error
from multiprocessing import get_logger
import threading
import requests
from communication_interfaces.msg import FFOpenDrawer


class ros_controller(Node):

    def __init__(self, api_url):
        super().__init__('ros_controller')
        self.base_url = api_url
        self.open_drawer_to_ff = self.create_publisher(FFOpenDrawer, 'ff_open_drawer', 10)
    
  
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
                #ToDO Torben: get all Values from the Frontend / restapi
                self.openDrawer(drawer_controller_id, 1, "ROBAST_1", "rb0")
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

    def openDrawer(self, drawer_id:int, module_id:int, fleet_name:str, robot_name:str):
        msg= FFOpenDrawer()
        msg.fleet_name = fleet_name
        msg.robot_name = robot_name
        msg.drawer_address.module_id = module_id
        msg.drawer_address.drawer_id = drawer_id
        self.open_drawer_to_ff.publish(msg)
       
      
        