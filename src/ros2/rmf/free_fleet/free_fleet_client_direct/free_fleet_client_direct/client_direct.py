from communication_interfaces.msg import DrawerAddress, DrawerStatus
from communication_interfaces.action import CreateUserNfcTag
from std_msgs.msg import Bool, String

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionClient

import datetime
from enum import Enum


from free_fleet_client_direct.nav_controller  import nav_controller 
import robast_dds_communicator.msg as dds 


# TODO@ Torben this aproach is rubbisch switch the logic to simple statemaschine  

class drawer():
    def __init__(self, module_id :int, drawer_id :int, e_drawer:bool, locked_for: dict):
        self.module_id=module_id
        self.drawer_id=drawer_id
        self.locked_for=locked_for
        self.e_drawer= e_drawer

class free_fleet_client_direct(Node):

    def __init__(self):
        super().__init__('free_fleet_direct_client')
        
        self.declare_parameter('fleet_name', 'ROBAST')
        self.declare_parameter('robot_name', 'RB0')
        self.declare_parameter('robot_model',"Robast_Theron")
        self.declare_parameter('robot_frame_id', 'map')
        self.declare_parameter('robot_odom', '/odometry/filtered')
 
        self.declare_parameter('heartbeat', 0.001)
        self.declare_parameter('statemaschine_open_drawer_topic', 'trigger_drawer_tree')
        self.declare_parameter('statemaschine_close_e_drawer_topic', 'close_drawer')
        self.declare_parameter('statemaschine_open_e_drawer_topic', 'trigger_electric_drawer_tree')
        self.declare_parameter('move_base_server_name', 'goal_pose')

        self.declare_parameter('dds_domain', 42)
        self.declare_parameter('dds_slide_drawer_topic', 'slide_drawer_request')
    

        self.fleet_name = self.get_parameter('fleet_name').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_model= self.get_parameter('robot_model').get_parameter_value().string_value
        self.frame_id = self.get_parameter("robot_frame_id").get_parameter_value().string_value
        self.robot_odom= self.get_parameter("robot_odom").get_parameter_value().string_value
        self.heartbeat= self.get_parameter('heartbeat').get_parameter_value().double_value
        self.ros_opendrawer_topic = self.get_parameter('statemaschine_open_drawer_topic').get_parameter_value().string_value
        self.ros_open_e_drawer_topic = self.get_parameter('statemaschine_open_e_drawer_topic').get_parameter_value().string_value
        self.ros_close_e_drawer_topic = self.get_parameter('statemaschine_close_e_drawer_topic').get_parameter_value().string_value
        self.ros_move_base_server_name=self.get_parameter('move_base_server_name').get_parameter_value().string_value
        self.dds_domain = self.get_parameter('dds_domain').get_parameter_value().integer_value
        self.dds_slide_drawer_topic=self.get_parameter('dds_slide_drawer_topic').get_parameter_value().string_value
       
        self.nav_controller= nav_controller(self, self.robot_odom, self.frame_id, self.publish_task_state)
      
        self.task_id= ""
        self.step=-1
        self.open_drawers:list[drawer] =[]
        self.locked_drawers:list[drawer]=[]

        self.drawer_requests = []
        self.destination_requests=[]
        self.new_user_requests=[]

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #controll drawer
        self.drawer_publisher = self.create_publisher(DrawerAddress, self.ros_opendrawer_topic, qos_profile=qos_profile)
        self.e_drawer_open_publisher = self.create_publisher(DrawerAddress, self.ros_open_e_drawer_topic, qos_profile=qos_profile)
        self.e_drawer_close_publisher = self.create_publisher(DrawerAddress, self.ros_close_e_drawer_topic, qos_profile=qos_profile)
        
      
        #controll nfc
        self.controll_nfc_publisher = self.create_publisher(Bool,"/nfc_switch", qos_profile)
        self.nfc_codes_subscriber= self.create_subscription(String, "/authenticated_user",self.receive_authentication_codes, qos_profile)
        self.create_nfc_action_client= ActionClient(self, CreateUserNfcTag,"/create_user")

        #task_requests
        self.setting_request_dds=  self.create_subscription(dds.FreeFleetDataSettingRequest, "setting_request", self.setting_callback, 10) 
        self.new_user_request_dds= self.create_subscription(dds.FreeFleetDataCreateNfcRequest, "new_user_request",self.new_user_callback,10)
        self.drawer_request_dds= self.create_subscription(dds.FreeFleetDataDrawerRequest, "slide_drawer_request",self.slide_drawer_callback ,10)
        self.destination_request_dds= self.create_subscription(dds.FreeFleetDataDestinationRequest,  "destination_request", self.destination_callback ,10)
        
        #publish_ info
        self.robot_state_dds = self.create_publisher(dds.FreeFleetDataRobotState, "robot_state",10)
        self.task_state_dds= self.create_publisher(dds.FreeFleetDataTaskState, "task_state",10)
        
        self.status_timer = self.create_timer(self.heartbeat,self.publish_fleet_state) 
        
    ##handle task
    #navigation request
    def destination_callback(self, msg:dds.FreeFleetDataDestinationRequest):
        print("recived")
        print(msg)
        step= self.received_new_action(msg)
        if str(step)==str(1):
            print("1")
            self.step=1
            self.start_navigation_task( msg.destination.x, msg.destination.y, msg.destination.yaw)
        else:
            self.destination_requests.append(msg)

    def start_navigation_task(self, x:float,y: float, yaw: float):
        print("nav")
        self.nav_controller.start_navigation(x, y, yaw)
    
    #drawer request
    def slide_drawer_callback(self, msg:dds.FreeFleetDataDrawerRequest):
        step= self.received_new_action(msg)
        if str(step)==str(1):
            self.step=1
            self.start_drawer_request(msg.fleet_name, msg.robot_name,msg.module_id,msg.drawer_id, msg.e_drawer, msg.restricted)
        else:
            self.drawer_requests.append(msg)

    def start_drawer_request(self,fleet_name:str, robot_name:str, module_id:int, drawer_id:int, e_drawer:bool, restriction:[String]):
        user_restriction= next((s_drawer.locked_for for s_drawer in self.locked_drawers if  s_drawer.module_id == module_id and s_drawer.drawer_id== drawer_id),None) 
        if user_restriction is not None:
            user_name=self.perform_NFC_reading( user_restriction)
            if  user_name is None:
                return
            else: 
                self.publish_task_state("DrawerAuthentification", str(module_id)+"#"+str(drawer_id)+user_name, False)
        self.set_drawer_lock(module_id, drawer_id, restriction)
        self.open_drawer( module_id=module_id, drawer_id= drawer_id, e_drawer= e_drawer)
        self.publish_task_state("DrawerState", str(module_id)+"#"+str(drawer_id)+"#Opened", False)

    def open_drawer(self, module_id:int, drawer_id:int, e_drawer:bool): 
        ros_msg = DrawerAddress()
        ros_msg.module_id = int(module_id)
        ros_msg.drawer_id = int(drawer_id)
        drawer_change= drawer(module_id, drawer_id,e_drawer,{})
        self.open_drawers.append(drawer_change)

        if(e_drawer):
            self.e_drawer_open_publisher.publish(ros_msg)
            print("edrawer")
        else:
            self.drawer_publisher.publish(ros_msg)
            print("drawer")
        
    def close_drawer(self,module_id, drawer_id, e_drawer):
        print("close")
        if(not e_drawer):
            return
        ros_msg = DrawerAddress()
        ros_msg.module_id = module_id
        ros_msg.drawer_id = drawer_id
        self.e_drawer_close_publisher.publish(ros_msg)
        self.publish_task_state("DrawerState", str(module_id)+"#"+str(drawer_id)+"#Closed", False)
    
    def end_drawer_task(self):
        self.publish_task_state("DrawerAction", "Finished", True)
        
    def set_drawer_lock(self, module_id:int, drawer_id:int, restriction):
        if(len(restriction)>0):
            user_dict={}
            for user in restriction:
                entry= user.split(':')
                user_dict.update({entry[0] : entry[2]})
            self.locked_drawers.append(drawer(module_id= module_id, drawer_id= drawer_id, locked_for=user_dict))

    def perform_NFC_reading(self, possible_nfc_codes:dict):         
        nfc_toggle_msg=Bool()
        nfc_toggle_msg.data= True
        self.controll_nfc_publisher.publish(nfc_toggle_msg)
        successful= True
        start = datetime.datetime.now()
        while( self.received_nfc_codes not in possible_nfc_codes ):
            duration=  datetime.datetime.now()-start
            if duration.total_seconds >120:
                successful=False
                break
        nfc_toggle_msg.data= False
        self.controll_nfc_publisher.publish(nfc_toggle_msg)
        if successful:
            return possible_nfc_codes[self.received_nfc_codes]
        return successful   

    def receive_authentication_codes(self, msg):
        pass

    # new_NFC request
    def new_user_callback(self, msg:dds.FreeFleetDataCreateNfcRequest):
        step= self.received_new_action(self, msg)
        if step==1:
            self.step=1
            self.start_new_user_request(msg.user_id)
        else:
            self.new_user_requests.append(msg)

    def start_new_user_request(self, user_id):
        self.publish_task_state("User","Started", False)
        goal_msg= CreateUserNfcTag.Goal()
        goal_msg.user_id= user_id
        goal_msg.first_name = ""
        goal_msg.last_name = ""
        self.create_nfc_send_goal_feature= self.create_nfc_action_client.send_goal_async(goal_msg)
        self.create_nfc_send_goal_feature.add_done_callback(self.create_nfc_accept)
    
    def create_nfc_accept(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('create nfc rejected')
            self.publish_task_state("User","Rejected",False)
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.store_nfc_code)

    def store_nfc_code(self,future):
        result = future.result()
        self.publish_task_state("User_NFC", result.nfc_code, True)
            
    # settings request
    def setting_callback(self, msg:dds.FreeFleetDataSettingRequest):
        if(not self.task_validation(msg)):
            return
        if msg.command == "move":
            if msg.value == "resume":
                self.nav_controller.start_navigation()
            elif self.nav_controller.active:
                if msg.value=="pause":
                    self.nav_controller.pause_navigation()
                elif msg.value == "resume":
                    self.nav_controller.cancel_navigation()
                    self.publish_fleet_state()

        elif msg.command =="drawer":
            if msg.value == "completed":
                #doing a drawertask
                if len(self.open_drawers)==0:
                    self.find_next_action()
                    
            else:
                selected_drawer= next((drawer for drawer in self.open_drawers if drawer.module_id== int(msg.value) and drawer.e_drawer), None)
                if selected_drawer is not None: 
                    self.get_logger().info('close_drawer')
                    self.close_drawer(selected_drawer.module_id,selected_drawer.drawer_id, selected_drawer.e_drawer)
                    print("close drawer")


    #publish states
    def publish_fleet_state(self):
        battery= 0.0 #todo @Torben read the battery sate from the robot
        sequence = [] #todo @Torben add a list of waypoint of the robot
        mode= dds.FreeFleetDataRobotMode()
        mode.mode=0 
        current_location= dds.FreeFleetDataLocation()
        current_location.sec= 0
        current_location.nanosec=0
        current_location.x= float(self.nav_controller.robot_x)
        current_location.y= float(self.nav_controller.robot_y)
        current_location.yaw= float(self.nav_controller.robot_yaw)
        current_location.level_name=""
        robot_state = dds.FreeFleetDataRobotState()
        robot_state.name=self.robot_name
        robot_state.model=self.robot_model
        task_id=""
        if(self.task_id != ""):
            task_id= str(self.task_id)+"#"+str(self.step)
        robot_state.task_id= task_id
        robot_state.mode=mode
        robot_state.battery_percent= battery
        robot_state.location= current_location
        robot_state.path = sequence
        self.robot_state_dds.publish(robot_state) 

    def publish_task_state(self, status, message, completed):
        task_state= dds.FreeFleetDataTaskState()
        
        task_state.task_id= str(self.task_id)+"#"+str(self.step)
        task_state.status=status
        task_state.status_message= message
        task_state.completed= completed 
        self.task_state_dds.publish(task_state)    

    #support task
    def divide_task_id(self,task_id):
        combined_ids= task_id.split('#')
        if len(combined_ids)!=2:
            return task_id, None
        else:
            return combined_ids[0], combined_ids[1] 
    
    def task_validation(self, task):
        return task.fleet_name == self.fleet_name and task.robot_name == self.robot_name 
    
    
    def swap_task(self, new_task_id:str):
        self.clear_task()
        self.task_id= new_task_id
        self.publish_fleet_state()

    def clear_task(self):
        self.task_id=None
        self.step=0
        self.drawer_requests = []
        self.destination_requests=[]
        self.new_user_requests=[]

    def find_next_action(self):
        self.step+= 1
        
        destination_task= self.search_for_task(self.destination_requests)
        if(destination_task is not None):
            self.start_navigation_task( destination_task.destination.x, destination_task.destination.y, destination_task.destination.yaw)
            return 
        
        drawer_task= self.search_for_task(self.drawer_requests)
        if(drawer_task is not None):
            self.start_drawer_request(drawer_task.fleet_name, drawer_task.robot_name, drawer_task.e_drawer, drawer_task.restricted)
            return
        
        new_user_task= self.search_for_task(self.new_user_requests)
        if(new_user_task is not None):
            self.start_new_user_request(new_user_task.user_id)
            return
        self.finish_task()

    def finish_task(self):
        self.publish_task_state("Task","Completed", True)
        self.clear_task()
        self.publish_fleet_state()

    def search_for_task(self, task_list:list):
        new_task= next((task for task in task_list if self.divide_task_id( task.task_id)[1]==self.step), None)
        return new_task 
    
    def received_new_action(self, msg):
        if not self.task_validation(msg):
            return
        (id, step)=self.divide_task_id(msg.task_id)
            
        if self.task_id != "" and self.task_id !=id:
            self.publish_task_state("Task","Postponed",False)
        
        self.swap_task(id)
        return step
    
