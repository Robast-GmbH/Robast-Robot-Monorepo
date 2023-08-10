
from rclpy.node import Node
from . import web_module
from multiprocessing import get_logger
import requests
from communication_interfaces.msg import FFSlideDrawer


class ros_controller(Node):

    def __init__(self, api_url):

        super().__init__('ros_controller')
        self.base_url = api_url
        self.slide_drawer_to_ff = self.create_publisher(
            FFSlideDrawer,
            'ff_open_drawer',
            10)
        
        self.declare_parameter('dds', "cyclone_DDS")
        self.declare_parameter('dds_domain_id', 42)
        self.declare_parameter('dds_robot_state_topic', "robot_state")
        self.declare_parameter('dds_mode_request_topic', "mode_request")
        self.declare_parameter('dds_path_request_topic', "path_request")
        self.declare_parameter('dds_destination_request_topic', "destination_request")
        self.declare_parameter('dds_slide_drawer_request_topic', "slide_drawer_request")
        self.declare_parameter('dds_setting_request_topic', "setting_request")
        self.declare_parameter('dds_info_state_topic',"info_state")
        self.dds_config = {
                "dds": self.get_parameter('dds').get_parameter_value().string_value,
                "domain_id": self.get_parameter('dds_domain_id').get_parameter_value().integer_value,
                "robot_state_topic": self.get_parameter('dds_robot_state_topic')
                                         .get_parameter_value().string_value,
                "mode_request_topic": self.get_parameter('dds_mode_request_topic')
                                          .get_parameter_value().string_value,
                "path_request_topic": self.get_parameter('dds_path_request_topic')
                                          .get_parameter_value().string_value,
                "destination_request_topic": self.get_parameter('dds_destination_request_topic')
                                          .get_parameter_value().string_value,
                "slide_drawer_request_topic": self.get_parameter('dds_slide_drawer_request_topic')
                                          .get_parameter_value().string_value,
                "setting_request_topic": self.get_parameter('dds_setting_request_topic')
                                          .get_parameter_value().string_value,
                "info_state_topic": self.get_parameter('dds_info_state_topic')
                                         .get_parameter_value().string_value

            }

    def get_robot_status(self):
        response = web_module.getDataFromServer(self.base_url + "/robot/status")
        if response is not None:
            self.robot_status = response.json()

    def get_drawer_open_status(self):
        api_url = self.base_url+"/drawer/open"
        response = web_module.getDataFromServer(api_url)
        if (response is not None):
            drawer_controller_id = response.json()
            if (drawer_controller_id > 0):
                # ToDO Torben: get all Values from the Frontend / restapi
                self.openDrawer(drawer_controller_id, 1, "ROBAST_1", "rb0")
                self.delete_request(api_url)
            elif(drawer_controller_id != -1):
                get_logger().warning(
                    "Request for opening drawer with invalid drawer_controller_id: {0}"
                    .format(drawer_controller_id))

    def delete_request(self, api_url):
        response = requests.delete(api_url)
        if(response.status_code == 200):
            get_logger().info(
                "Deleting request for {0} was successfull!".format(api_url))
        else:
            get_logger().warning(
                "Deleting request for {0} was NOT successfull!".format(api_url))

    def open_drawer(self, drawer_id: int, module_id: int, fleet_name: str, robot_name: str):
        msg = FFSlideDrawer()
        msg.fleet_name = fleet_name
        msg.robot_name = robot_name
        msg.drawer_address.module_id = module_id
        msg.drawer_address.drawer_id = drawer_id
        msg.drawer_
        self.slide_drawer_to_ff.publish(msg)

    def get_dds_config(self):
        return self.dds_config
