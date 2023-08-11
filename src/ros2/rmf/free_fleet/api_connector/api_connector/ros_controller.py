
from rclpy.node import Node
from . import web_module
import json
from multiprocessing import get_logger
import requests
from communication_interfaces.msg import FFSlideDrawer
from ddsmessages.msg import FreeFleetDataModeRequest, FreeFleetDataPathRequest, FreeFleetDataLocation, FreeFleetDataDestinationRequest, FreeFleetDataSlideDrawerRequest, FreeFleetDataSettingRequest, FreeFleetDataRobotState, FreeFleetDataInfoState, FreeFleetDataDrawerState



class ros_controller(Node):

    def __init__(self, api_url):

        super().__init__('ros_controller')
        self.base_url = api_url

        self.robot_states = [] 
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
        self.declare_parameter('dds_drawer_state_topic',"drawer_state")
        self.declare_parameter('backend_address',"http://127.0.0.1:3001/")

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
                                         .get_parameter_value().string_value,
                "drawer_state_topic": self.get_parameter('dds_drawer_state_topic')
                                        .get_parameter_value().string_value,
                "backend_address": self.get_parameter('backend_address')
                                        .get_parameter_value().string_value
            }
        
        self.dds_robot_state= self.create_subscription(FreeFleetDataRobotState, self.dds_config["robot_state_topic"], self.send_robot_state ,10)
        self.dds_mode_request= self.create_publisher(FreeFleetDataModeRequest, self.dds_config["mode_request_topic"], 10)
        self.dds_path_request= self.create_publisher(FreeFleetDataPathRequest, self.dds_config["path_request_topic"], 10)
        self.dds_destination_request= self.create_publisher( FreeFleetDataDestinationRequest, self.dds_config["destination_request_topic"], 10)
        self.dds_slide_drawer_request= self.create_publisher( FreeFleetDataSlideDrawerRequest, self.dds_config["slide_drawer_request_topic"], 10)
        self.dds_settings_request= self.create_publisher( FreeFleetDataSettingRequest, self.dds_config["setting_request_topic"], 10)
        self.dds_info_state=  self.create_subscription(FreeFleetDataInfoState, self.dds_config["info_state_topic"], self.sent_info_state,10)
        self.dds_drawer_state= self.create_subscription( FreeFleetDataDrawerState, self.dds_config["drawer_state_topic"], self.sent_drawer_state, 10)

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
                self.openDrawer(drawer_controller_id, 1, "ROBAST", "RB0")
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
        self.slide_drawer_to_ff.publish(msg)

    def get_dds_config(self):
        return self.dds_config


    def get_robot_status(self):
        return self.robot_states

    # handle Mode request
    def handle_mode_request(self, fleet_name, robot_name, mode):
        state = self.get_robot_state(robot_name)
        mode_request = FreeFleetDataModeRequest
        mode_request.robot_name = robot_name
        mode_request.fleet_name = fleet_name
        mode_request.mode = mode
        mode_request.task_id = state.task_id
        mode_request.parameters = []
        self.dds_mode_request.publish(mode_request)

    # handle path request
    def handle_path_request(self, fleet_name, robot_name, path):
        state = self.get_robot_state(robot_name)
        path_request = FreeFleetDataPathRequest()
        path_request.robot_name = robot_name
        path_request.fleet_name = fleet_name
        path_request.task_id = state.task_id
        path_request.path = path
        self.dds_path_request.publish(path_request)
        

    # handle destination request
    def handle_destination_request(self, fleet_name, robot_name, destination):
        state = self.get_robot_state(robot_name)
        goal= FreeFleetDataLocation()
        goal.sec= 0
        goal.nanosec=0
        goal.x= float(destination.pose.x)
        goal.y= float(destination.pose.y)
        goal.yaw= float(destination.orientation)
        goal.level_name=""
        destination_request = FreeFleetDataDestinationRequest()
        destination_request.robot_name=robot_name
        destination_request.fleet_name=fleet_name
        destination_request.task_id=""
        destination_request.destination=goal
        self.dds_destination_request.publish(destination_request)

    # handle_drawer_request
    def handle_slide_drawer_request(self, fleet_name:str, robot_name: str, module_id: int, drawer_id: int, restricted:list[str] , e_drawer: bool, open: bool):
        slide_drawer_request = FreeFleetDataSlideDrawerRequest()
        slide_drawer_request.fleet_name= fleet_name
        slide_drawer_request.robot_name= robot_name
        slide_drawer_request.module_id=  module_id
        slide_drawer_request.drawer_id= drawer_id
        slide_drawer_request.restricted= restricted
        slide_drawer_request.e_drawer= e_drawer
        slide_drawer_request.open= open
        self.dds_slide_drawer_request.publish(slide_drawer_request)

    def handle_setting_request(self, command:str, name:str, value:str):
        setting_request = FreeFleetDataSettingRequest()
        setting_request.command=command
        setting_request.name= name
        setting_request.value=value
        self.dds_settings_request.publish(setting_request)

    def get_robot_states(self):
        return self.robot_states

    def get_robot_state(self, name):
        if len(self.robot_states)>0:
            return [state for state in self.robot_states if state.name == name][0]
        return

    def send_robot_state(self, msg):
            message={"robot_name": msg.name,"fleet_name":"ROBAST", "task_id":msg.task_id, "y_pose": msg.location.y ,"x_pose": msg.location.x ,"yaw_pose": msg.location.yaw}
            headers =  {"Content-Type":"application/json"}
            sender = requests.Session()
            sender.post(url= self.dds_config["backend_address"]+"/robots/status", json= json.dumps(message), headers= headers, verify=False)
 

    def sent_info_state(self, msg):
        if(msg.type== "new_user"):
                message={"nfc_code":msg.value }
                headers =  {"Content-Type":"application/json"}
                sender = requests.Session()
                sender.post(url= self.dds_config["backend_address"]+"/users/"+msg.name+"/nfc", json= json.dumps(message), headers= headers, verify=False)

    def sent_drawer_state(self, msg):
            status="closed"
            if(msg.open):
                status="opened"
            
            message={"robot_name": msg.robot_name, "id":msg.module_id, "drawer_id":msg.drawer_it, "status":status }
            headers =  {"Content-Type":"application/json"}
            sender = requests.Session()
            sender.post(url= self.dds_config["backend_address"]+"/robots/modules/status", json= json.dumps(message), headers= headers, verify=False)
        