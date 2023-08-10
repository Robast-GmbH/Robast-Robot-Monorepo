import requests
import json
from .dds import messages
from .dds import dds_communicator as dds
import sched, time


class free_fleet_controller:
  

    def __init__(self, dds_config: dict):
        self.config = dds_config
        self.robot_states = []  # :list[messages.FreeFleetData_RobotState] =[]
        
        self.dds_robot_state= dds.DDS_communicator(self.config["domain_id"], self.config["robot_state_topic"], messages.FreeFleetData_RobotState)
        self.dds_mode_request= dds.DDS_communicator(self.config["domain_id"], self.config["mode_request_topic"], messages.FreeFleetData_ModeRequest)
        self.dds_path_request= dds.DDS_communicator(self.config["domain_id"], self.config["path_request_topic"], messages.FreeFleetData_PathRequest)
        self.dds_destination_request= dds.DDS_communicator(self.config["domain_id"], self.config["destination_request_topic"], messages.FreeFleetData_DestinationRequest)
        self.dds_slide_drawer_request= dds.DDS_communicator(self.config["domain_id"], self.config["slide_drawer_request_topic"], messages.FreeFleetData_SlideDrawerRequest)
        self.dds_settings_request=dds.DDS_communicator(self.config["domain_id"], self.config["setting_request_topic"], messages.FreeFleetData_SettingRequest)
        self.dds_info_state= dds.DDS_communicator(self.config["domain_id"], self.config["info_state_topic"], messages.FreeFleetData_InfoState )
        self.dds_drawer_state= dds.DDS_communicator(self.config["domain_id"], self.config["drawer_state_topic"], messages.FreeFleetData_DrawerState )
        my_scheduler = sched.scheduler(time.time, time.sleep)
        my_scheduler.enter(60, 1, self.start_receiving_info, (my_scheduler,))
        my_scheduler.run()
                

    def get_robot_status(self, robot_name):
        return list(filter(lambda robot: robot.name == robot_name, self.robot_states))

    # handle Mode request
    def handle_mode_request(self, fleet_name, robot_name, mode):
        state = self.get_robot_state(robot_name)
        mode_request = messages.FreeFleetData_ModeRequest()
        mode_request.robot_name = robot_name
        mode_request.fleet_name = fleet_name
        mode_request.mode = mode
        mode_request.task_id = state.task_id
        mode_request.parameters = []
        self.dds_mode_request.publish(mode_request)

    # handle path request
    def handle_path_request(self, fleet_name, robot_name, path):
        state = self.get_robot_state(robot_name)
        path_request = messages.FreeFleetData_PathRequest()
        path_request.robot_name = robot_name
        path_request.fleet_name = fleet_name
        path_request.task_id = state.task_id
        path_request.path = path
        self.dds_path_request.publish(path_request)
        

    # handle destination request
    def handle_destination_request(self, fleet_name, robot_name, destination):
        state = self.get_robot_state(robot_name)
        goal= messages.FreeFleetData_Location(sec=0, nanosec=0, x=destination.pose.x, y= destination.pose.y, yaw=destination.orientation,level_name="")
        destination_request = messages.FreeFleetData_DestinationRequest(robot_name=robot_name, fleet_name=fleet_name, task_id="", destination=goal)
        self.dds_destination_request.publish(destination_request)

    # handle_drawer_request
    def handle_slide_drawer_request(self, fleet_name:str, robot_name: str, module_id: int, drawer_id: int, restricted:list[str] , e_drawer: bool, open: bool):
        slide_drawer_request = messages.FreeFleetData_SlideDrawerRequest(
            fleet_name,
            robot_name,
            module_id,
            drawer_id,
            restricted,
            e_drawer,
            open)
        self.dds_slide_drawer_request.publish(slide_drawer_request)

    def handle_setting_request(self, command:str, name:str, value:str):
        setting_request = messages.FreeFleetData_SettingRequest( command=command, name= name, value=value)
        self.dds_settings_request.publish(setting_request)

    def get_robot_states(self):
        return self.robot_states

    def get_robot_state(self, name):
        return [state for state in self.robot_states if state.name == name][0]
        
    def start_receiving_info(self):
        info_state_msg=  self.dds_info_state.get_next() 
        if(info_state_msg is not None):
            if(info_state_msg.type== "new_user"):
                message={"nfc_code":info_state_msg.value }
                headers =  {"Content-Type":"application/json"}
                sender = requests.Session()
                answer  =sender.post(url= self.config["backend_address"]+"/users/"+info_state_msg.name+"/nfc", json= json.dumps(message), headers= headers, verify=False)

        robot_state_msg=self.dds_robot_state.get_next()
        if(robot_state_msg is not None):
            message={"robot_name": robot_state_msg.name,"fleet_name":"ROBAST", "task_id":robot_state_msg.task_id, "y_pose": robot_state_msg.location.y ,"x_pose": robot_state_msg.location.x ,"yaw_pose": robot_state_msg.location.yaw}
            headers =  {"Content-Type":"application/json"}
            sender = requests.Session()
            answer  =sender.post(url= self.config["backend_address"]+"/robots/status", json= json.dumps(message), headers= headers, verify=False)

            pass

        drawer_state_msg = self.dds_drawer_state.get_next()
        if(info_state_msg is not None):
            status="closed"
            if(drawer_state_msg.open):
                status="opened"
            
            message={"robot_name": drawer_state_msg.robot_name, "id":drawer_state_msg.module_id, "drawer_id":drawer_state_msg.drawer_it, "status":status }
            headers =  {"Content-Type":"application/json"}
            sender = requests.Session()
            answer  =sender.post(url= self.config["backend_address"]+"/robots/modules/status", json= json.dumps(message), headers= headers, verify=False)
        
        self.scheduler.enter(60, 1, self.start_receiving_info, (self.scheduler,))

    