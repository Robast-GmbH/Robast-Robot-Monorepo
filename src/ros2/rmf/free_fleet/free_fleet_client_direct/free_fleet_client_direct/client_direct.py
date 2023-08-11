import datetime
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionClient


from rclpy.node import Node
# from .dds import messages
from std_msgs.msg import Bool, String

# from .dds import dds_communicator as dds
from . import math_helper
from enum import Enum

from communication_interfaces.msg import DrawerAddress, DrawerStatus
from communication_interfaces.action import CreateUserNfcTag
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ddsmessages.msg  import FreeFleetDataInfoState, FreeFleetDataSettingRequest, FreeFleetDataSlideDrawerRequest, FreeFleetDataDestinationRequest, FreeFleetDataRobotState, FreeFleetDataDrawerState, FreeFleetDataLocation, FreeFleetDataRobotMode 


# TODO@ Torben this aproach is rubbisch switch the logic to simple statemaschine  

class Robot_states(Enum):
    IDLE = 1
    DRAWERMODE = 2
    MOVEMENTMODE = 3

class drawer():
    module_id :int
    drawer_id :int
    locked_for: dict

class free_fleet_client_direct(Node):

    def __init__(self):
        super().__init__('free_fleet_direct_client')
        
        self.declare_parameter('fleet_name', 'ROBAST_1')
        self.declare_parameter('robot_name', 'RB0')
        self.declare_parameter('robot_model',"Robast_Theron")
        self.declare_parameter('robot_frame_id', 'map')
        self.declare_parameter('robot_odom', '/odom')
 
        self.declare_parameter('heartbeat', 0.01)
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
       
        self.navigator = BasicNavigator()
        self.state= Robot_states.IDLE
        self.task_id= None
        self.open_drawers:list[drawer] =[]
        self.locked_drawers:list[drawer]=[]

        self.drawer_requests = []
        self.destination_request=[]
        self.setting_requests=[]


        self.robot_x= 0
        self.robot_y=0
        self.robot_yaw=0



        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
     
        self.drawer_publisher = self.create_publisher(DrawerAddress, self.ros_opendrawer_topic, qos_profile=qos_profile)
        self.e_drawer_open_publisher = self.create_publisher(DrawerAddress, self.ros_open_e_drawer_topic, qos_profile=qos_profile)
        self.e_drawer_close_publisher = self.create_publisher(DrawerAddress, self.ros_close_e_drawer_topic, qos_profile=qos_profile)

        self.controll_nfc_publisher = self.create_publisher(Bool,"/nfc_switch", qos_profile)
        self.nfc_codes_subscriber= self.create_subscription(String, "/authenticated_user",self.receive_authentication_codes, qos_profile)
        self.create_nfc_action_client= ActionClient(self, CreateUserNfcTag,"/create_user")

        self.setting_request_dds=  self.create_subscription(FreeFleetDataSettingRequest, "/settings_request", self.setting_callback, 10) 
        self.drawer_request_dds= self.create_subscription( FreeFleetDataSlideDrawerRequest, "/slide_drawer_request",self.slide_drawer_callback ,10)
        self.destination_request_dds= self.create_subscription( FreeFleetDataDestinationRequest,  "/destination_request", self.destination_callback ,10)
        
        self.robot_state_dds = self.create_publisher(FreeFleetDataRobotState,  "/robot_state",10)
        self.drawer_states_dds = self.create_publisher(FreeFleetDataDrawerState, "/drawer_state", 10)
        self.info_state_dds= self.create_publisher(FreeFleetDataInfoState, "/info_state",10)
        
        self.action_timer = self.create_timer(self.heartbeat, self.start_robot_behavior)
        self.start_receiving_robot_info()
        self.status_timer = self.create_timer(self.heartbeat,self.start_sending_status) 
        
        

    def start_robot_behavior(self):
        self.check_settings()
        if(self.state== Robot_states.IDLE):
            #drawer_task = self.drawer_request_dds.get_next()
              
            if len(self.drawer_requests)>0:
                drawer_task= self.drawer_requests.pop()
                self.state= Robot_states.DRAWERMODE 
                self.task_id= None
                self.do_drawer_action(drawer_task)
                return
            
            if len(self.destination_request):
                move_task= self.destination_request.pop()
                self.state= Robot_states.MOVEMENTMODE
                self.task_id= None
                self.do_move_action(move_task)
                return
            
        elif(self.state== Robot_states.DRAWERMODE):
            if len(self.drawer_requests)>0:
                drawer_task= self.drawer_requests.pop()
                self.do_drawer_action(drawer_task)
                return                

        elif(self.state== Robot_states.MOVEMENTMODE ): 
               if self.navigator.isTaskComplete():
                   self.finish_movement()

    def check_settings(self):
        new_settings=True 
        while(new_settings):
            if len(self.setting_requests)>0:
                msg= self.setting_requests.pop()
                new_settings= False
                return
            if(msg.command=="move" and self.state==Robot_states.MOVEMENTMODE):
                if(msg.value=="pause"):
                    self.pause_navigation()
                elif(msg.value== "resume"):
                    self.start_navigation()
                elif(msg.value== "cancel"):
                    self.cancel_navigation()

            elif(msg.command=="new_user"):
                user_id=msg.value
                self.create_nfc_card(user_id)
        
        def create_nfc_card(self, user_id): 
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
                return
            
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.store_nfc_code)

        def store_nfc_code(self,future):
                result = future.result()
                nfc_key= result.nfc_code
                msg= FreeFleetDataInfoState("new_user", self.new_user_id, nfc_key)
                self.info_State_dds.publish(msg)

            
    def finish_movement(self):
        move_result= self.navigator.getResult()
        if move_result == TaskResult.SUCCEEDED:
            movement_status ="completetd"
        elif move_result == TaskResult.CANCELED:
            if(self.goal_pose== None):
                movement_status= "canceled"
            else:
                movement_status= "paused"
        elif move_result == TaskResult.FAILED:
             movement_status = "failed"
        self.state= Robot_states.IDLE         

    def start_receiving_robot_info(self):  
        self.subscriber_odom = self.create_subscription(
            Odometry,
            self.robot_odom,
            self.get_robot_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.subscriber_drawerStatus =self.create_subscription(
            DrawerStatus, "/drawer_is_open", self.update_drawer,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        

    def do_drawer_action(self, msg):
        if msg.fleet_name == self.fleet_name and msg.robot_name == self.name:
            if msg.module_id==-1 and msg.drawer_id==-1:#Drawer_task_ended
                if(self.open_drawers.__len__==0):
                    self.publish_fleet_states()
                    self.state= Robot_states.IDLE
            elif(msg.open):
                user_restriction= next((s_drawer.locked_for for s_drawer in self.locked_drawers if  s_drawer.module_id == msg.module_id and s_drawer.drawer_id== msg.drawer_id),None) 
                if user_restriction is not None:
                    if not self.perform_NFC_reading( user_restriction):
                        return
                self.set_drawer_lock(msg)
                self.open_drawer(msg)
            else:
                ros_msg=self.dds_Drawer_msg_to_ros(msg) 
                self.e_drawer_close_publisher.publish(ros_msg)

    def open_drawer(self, msg): 
        ros_msg=self.dds_Drawer_msg_to_ros(msg)  
        self.open_drawers.append(drawer(module_id=msg.module_id, drawer_id=msg.drawer_id))
        if(msg.e_drawer):
            self.e_drawer_open_publisher.publish(ros_msg)
        else:
            self.drawer_publisher.publish(ros_msg)
            
    def dds_Drawer_msg_to_ros(self,dds_msg):
        ros_msg = DrawerAddress()
        ros_msg.module_id = dds_msg.module_id
        ros_msg.drawer_id = dds_msg.drawer_id
        return ros_msg
    
    def set_drawer_lock(self,msg):
        if(len(msg.restricted)>0):
            user_dict={}
            for user in msg.restricted:
                entry= user.partition(':')
                user_dict.update({entry[0] : entry[2]})

            self.locked_drawers.append(drawer(module_id=msg.module_id, drawer_id=msg.drawer_id, locked_for=user_dict))
    
    def perform_NFC_reading(self, possible_nfc_codes:dict): 
            #read NFC        
            nfc_toggle_msg=Bool()
            nfc_toggle_msg.data= True
            self.controll_nfc_publisher.publish(nfc_toggle_msg)
            sucessfull= True
            start = datetime.datetime.now()
            while( self.received_nfc_codes not in possible_nfc_codes.values() ):
                duration=  datetime.datetime.now()-start
                if duration.total_seconds >120:
                    sucessfull=False
                    break

            nfc_toggle_msg.data= False
            self.controll_nfc_publisher.publish(nfc_toggle_msg)
            return sucessfull
    
    
    def do_move_action(self, msg):
        self.set_goal_pose(msg)
        self.start_navigation()

    
    def set_goal_pose(self, msg):
        self.goal_pose =self.create_pose(msg.destination.x, msg.destination.y, msg.destination.yaw)
        
    def start_navigation(self):    
        self.navigator.goToPose(self.goal_pose)
    
    def pause_navigation(self):
        self.navigator.cancelTask()

    def cancel_navigation(self):
        self.goal_pose = None
        self.navigator.cancelTask()
        self.state= Robot_states.IDLE
      
    def create_pose(self, pose_x, pose_y, pose_yaw) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        qx, qy, qz, qw = math_helper.quaternion_from_euler(0, 0, pose_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def get_robot_odom(self, data:Odometry):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        q1 = data.pose.pose.orientation.x
        q2 = data.pose.pose.orientation.y
        q3 = data.pose.pose.orientation.z
        q4 = data.pose.pose.orientation.w
        q = (q1, q2, q3, q4)
        e = math_helper.euler_from_quaternion(q)
        th = math_helper.degrees(e[2])
        yaw = math_helper.to_positive_angle(th)
        self.robot_x= x
        self.robot_y=y
        self.robot_yaw=yaw
       

    def update_drawer(self, data:DrawerStatus):
        drawer_id = data.drawer_address.drawer_id
        module_id = data.drawer_address.module_id
        drawer_open= data.drawer_is_open
        if(not drawer_open):
            self.open_drawers.remove(drawer(module_id= module_id, drawer_id= drawer_id))
            if self.open_drawers.__len__==0:
                self.start_wait_for_drawer =datetime.datetime.now()

        drawer_state= FreeFleetDataDrawerState(self.fleet_name, self.robot_name, module_id, drawer_id, drawer_open)
        self.drawer_states_dds.publish(drawer_state)


    def publish_fleet_states(self):
        battery= 0.0 #todo @Torben read the battery sate from the robot
        sequence = [] #todo @Torben add a list of waypoint of the robot
        current_location= FreeFleetDataLocation(0,0,self.robot_x,self.robot_y, self.robot_yaw,"")
        robot_state = FreeFleetDataRobotState(name=self.robot_name, model=self.robot_model,task_id="0",mode= FreeFleetDataRobotMode(0), battery_percent= battery, location= current_location, path = sequence )
        self.robot_state_dds.publish(robot_state) 
   
    def receive_authentication_codes(self, msg):
        self.received_nfc_codes =msg.data 
 
    def start_sending_status(self):
        self.publish_fleet_states()
    
    def setting_callback(self, msg):
        self.setting_requests.append(msg)

    def slide_drawer_callback(self, msg):
        self.drawer_requests.append(msg)

    def destination_callback(self,msg):
        self.destination_request.append(msg)