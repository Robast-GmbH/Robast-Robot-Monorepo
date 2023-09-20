
from rclpy.node import Node
import json
from multiprocessing import get_logger
import requests
from requests.exceptions import ConnectTimeout
from communication_interfaces.msg import FFSlideDrawer
import fleet_interfaces.msg  as dds



class ros_controller(Node):

    def __init__(self):

        super().__init__('ros_controller')
        self.responce= None
        
        self.slide_drawer_to_ff = self.create_publisher(
            FFSlideDrawer,
            'ff_open_drawer',
            10)
        
       
        self.declare_parameter('dds_mode_request_topic', "mode_request")
        self.declare_parameter('dds_path_request_topic', "path_request")
        self.declare_parameter('dds_destination_request_topic', "destination_request")
        self.declare_parameter('dds_slide_drawer_request_topic', "drawer_request")
        self.declare_parameter('dds_setting_request_topic', "setting_request")
        self.declare_parameter('new_user_request_topic', "new_user_request")

        self.declare_parameter('dds_robot_state_topic', "robot_state")
        self.declare_parameter('dds_task_state_topic',"task_state")
        
        
        self.declare_parameter('backend_address',"http://127.0.0.1:3000")
        self.declare_parameter('fleet_server',3002)

        self.node_config = {
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
                "new_user_request_topic": self.get_parameter('new_user_request_topic')
                                              .get_parameter_value().string_value,
                "robot_state_topic": self.get_parameter('dds_robot_state_topic')
                                         .get_parameter_value().string_value,
                "task_state_topic":self.get_parameter('dds_task_state_topic')
                                       .get_parameter_value().string_value,
                "backend_address": self.get_parameter('backend_address')
                                       .get_parameter_value().string_value,
                "fleet_server": self.get_parameter('fleet_server')
                                       .get_parameter_value().integer_value
            }
        
        
        self.dds_mode_request= self.create_publisher(dds.FreeFleetDataModeRequest, self.node_config["mode_request_topic"], 10)
        self.dds_path_request= self.create_publisher(dds.FreeFleetDataPathRequest, self.node_config["path_request_topic"], 10)
        
        self.dds_destination_request= self.create_publisher( dds.FreeFleetDataDestinationRequest, self.node_config["destination_request_topic"], 10)
        self.dds_slide_drawer_request= self.create_publisher( dds.FreeFleetDataDrawerRequest, self.node_config["slide_drawer_request_topic"], 10)
        self.dds_new_user_request= self.create_publisher(dds.FreeFleetDataCreateNfcRequest, self.node_config["new_user_request_topic"],10)
        self.dds_settings_request= self.create_publisher( dds.FreeFleetDataSettingRequest, self.node_config["setting_request_topic"], 10)
       
        self.dds_robot_state= self.create_subscription(dds.FreeFleetDataRobotState, self.node_config["robot_state_topic"], self.update_robot_state ,10)
        self.dds_task_state=self.create_subscription(dds.FreeFleetDataTaskState, self.node_config["task_state_topic"], self.update_task_state,10)
   

    # handle Mode request
    def handle_mode_request(self, fleet_name, robot_name, mode):
        mode_request = dds.FreeFleetDataModeRequest
        mode_request.robot_name = robot_name
        mode_request.fleet_name = fleet_name
        mode_request.mode = mode
        mode_request.task_id =""
        mode_request.parameters = []
        self.dds_mode_request.publish(mode_request)

    # handle path request
    def handle_path_request(self, fleet_name, robot_name, path):
        path_request = dds.FreeFleetDataPathRequest()
        path_request.robot_name = robot_name
        path_request.fleet_name = fleet_name
        path_request.task_id = ""
        path_request.path = path
        self.dds_path_request.publish(path_request)


    # handle destination request
    def handle_destination_request(self, fleet_name:str,robot_name:str, task_id:int,step:str, x_pose:int, y_pose:int, yaw:int):
        goal= dds.FreeFleetDataLocation()
        goal.sec= 0
        goal.nanosec=0
        goal.x= x_pose
        goal.y= y_pose
        goal.yaw= yaw
        goal.level_name= ""
        destination_request = dds.FreeFleetDataDestinationRequest()
        destination_request.robot_name=robot_name
        destination_request.fleet_name=fleet_name
        destination_request.task_id=task_id+"#"+step
        destination_request.destination=goal
        self.dds_destination_request.publish(destination_request)

    # handle_drawer_request
    def handle_slide_drawer_request(self, fleet_name:str, robot_name: str, task_id:str, step:str, module_id: int, drawer_id: int, restricted:list[int] , e_drawer: bool, open: bool):
        slide_drawer_request = dds.FreeFleetDataDrawerRequest()
        slide_drawer_request.task_id= task_id+"#"+step
        slide_drawer_request.fleet_name= fleet_name
        slide_drawer_request.robot_name= robot_name
        slide_drawer_request.module_id=  module_id
        slide_drawer_request.drawer_id= drawer_id
        slide_drawer_request.authorized_user = restricted
        slide_drawer_request.e_drawer= e_drawer
        self.dds_slide_drawer_request.publish(slide_drawer_request)    

    def handle_new_user_request(self, fleet_name:str, robot_name:str, task_id:str, step:str, user_id:int):
        new_user_request=dds.FreeFleetDataCreateNfcRequest()
        new_user_request.fleet_name=fleet_name
        new_user_request.robot_name=robot_name
        new_user_request.task_id=task_id+"#"+step
        new_user_request.user_id=user_id
        self.dds_new_user_request.publish(new_user_request)

    def handle_setting_request(self, robot_name:str, fleet_name:str, command:str, value:[str]):
        setting_request = dds.FreeFleetDataSettingRequest()
        setting_request.command=command
        setting_request.value=value
        setting_request.robot_name= robot_name
        setting_request.fleet_name=fleet_name
        self.dds_settings_request.publish(setting_request)

    def set_responce(self, responce_object):
        self.responce= responce_object

    def update_robot_state(self, msg:dds.FreeFleetDataRobotState):
        if self.responce is not None:
            mode =msg.mode
            self.responce.handle_robot_update(robot_name= msg.name, task_id=msg.task_id, mode=mode, x_pose= msg.location.x ,y_pose= msg.location.y, yaw_pose= msg.location.yaw, battery_level= msg.battery_percent)
          

    def update_task_state(self, msg: dds.FreeFleetDataTaskState):
        self.get_logger().info(f"{msg}")
        if msg.status=="DrawerState":
            data= msg.status_message.split('#')
            self.get_logger().info(f"{data}")
            self.responce.handle_drawer_status_change(task_id=msg.task_id, module_id= data[0], drawer_id=data[1], status=data[2])
        elif msg.status=="Task":
            if msg.status_message=="Completed":
                self.responce.handle_requesting_next_task()            
            
    def divide_task_id(self,task_id):
        combined_ids= task_id.split('#')
        if len(combined_ids)!=2:
            return task_id, None
        else:
            return combined_ids[0], combined_ids[1] 
        
    def get_node_config(self):
        return self.node_config
