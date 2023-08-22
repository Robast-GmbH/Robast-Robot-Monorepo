from communication_interfaces.msg import DrawerAddress, DrawerStatus
from communication_interfaces.action import CreateUserNfcTag
from std_msgs.msg import Bool, String

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from tf2_ros import TransformException

from . import math_helper

import datetime
from enum import Enum

import robast_dds_communicator.msg as dds 


# TODO@ Torben this approach is rubbish switch the logic to simple statemaschine  

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
        self.declare_parameter('behavior_tree','/workspace/src/navigation/nav_bringup/behavior_trees/humble/navigate_to_pose_w_replanning_goal_patience_and_recovery.xml')
        
 
        self.declare_parameter('heartbeat', 2.0)
        self.declare_parameter('patrol_break_frequency', 0.0056) #3 minute
        self.declare_parameter('statemaschine_open_drawer_topic', 'trigger_drawer_tree')
        self.declare_parameter('statemaschine_close_e_drawer_topic', 'close_drawer')
        self.declare_parameter('statemaschine_open_e_drawer_topic', 'trigger_electric_drawer_tree')
        self.declare_parameter('statemaschine_reset_simple_tree_topic', 'reset_simple_tree')
        self.declare_parameter('move_base_server_name', 'goal_pose')
        self.declare_parameter('drawer_status_change_topic','/drawer_is_open')

        self.declare_parameter('dds_domain', 42)
        self.declare_parameter('dds_slide_drawer_topic', 'slide_drawer_request')
    

        self.fleet_name = self.get_parameter('fleet_name').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_model= self.get_parameter('robot_model').get_parameter_value().string_value
        self.frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value
        self.nav_behavior_tree = self.get_parameter('behavior_tree').get_parameter_value().string_value
        self.robot_odom= self.get_parameter('robot_odom').get_parameter_value().string_value
        self.heartbeat= self.get_parameter('heartbeat').get_parameter_value().double_value
        self.patrol_break_frequency=self.get_parameter('patrol_break_frequency').get_parameter_value().double_value
        self.ros_opendrawer_topic = self.get_parameter('statemaschine_open_drawer_topic').get_parameter_value().string_value
        self.ros_open_e_drawer_topic = self.get_parameter('statemaschine_open_e_drawer_topic').get_parameter_value().string_value
        self.ros_close_e_drawer_topic = self.get_parameter('statemaschine_close_e_drawer_topic').get_parameter_value().string_value
        self.ros_drawer_change_topic = self.get_parameter('drawer_status_change_topic').get_parameter_value().string_value
        self.ros_reset_simple_tree_topic = self.get_parameter('statemaschine_reset_simple_tree_topic').get_parameter_value().string_value
        self.ros_move_base_server_name=self.get_parameter('move_base_server_name').get_parameter_value().string_value
        self.dds_domain = self.get_parameter('dds_domain').get_parameter_value().integer_value
        self.dds_slide_drawer_topic=self.get_parameter('dds_slide_drawer_topic').get_parameter_value().string_value
       
       #nav
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.robot_x= 0
        self.robot_y=0
        self.robot_yaw=0
        self.active=False
        self.frame_id= self.frame_id
        self.goal_frame="map"
        self.start_frame="robot_base_footprint"

        # self.subscriber_odom = self.create_subscription(
        #     Odometry,
        #     self.robot_odom,
        #     self.get_robot_odom,
        #     QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #task
        self.task_id= ""
        self.step=-1
        self.battery=0.0
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
        self.drawer_status_subscriber = self.create_subscription(DrawerStatus, self.ros_drawer_change_topic, qos_profile=qos_profile, callback=self.drawer_change_callback)
        self.reset_simple_tree_publisher= self.create_publisher(Bool, self.ros_reset_simple_tree_topic, qos_profile=qos_profile)
      
        #controll nfc
        self.controll_nfc_publisher = self.create_publisher(Bool,"/nfc_switch", qos_profile)
        self.nfc_codes_subscriber= self.create_subscription(String, "/authenticated_user",self.receive_authentication_codes, qos_profile)
        self.create_nfc_action_client= ActionClient(self, CreateUserNfcTag,"/create_user")

        #task_requests
        self.setting_request_dds=  self.create_subscription(dds.FreeFleetDataSettingRequest, "setting_request", self.setting_callback, 10) 
        self.new_user_request_dds= self.create_subscription(dds.FreeFleetDataCreateNfcRequest, "new_user_request",self.new_user_callback,10)
        self.drawer_request_dds= self.create_subscription(dds.FreeFleetDataDrawerRequest, "slide_drawer_request",self.slide_drawer_callback ,10)
        self.destination_request_dds= self.create_subscription(dds.FreeFleetDataDestinationRequest,  "destination_request", self.destination_callback ,10)
        
        
        #info
        self.robot_state_dds = self.create_publisher(dds.FreeFleetDataRobotState, "robot_state",10)
        self.task_state_dds= self.create_publisher(dds.FreeFleetDataTaskState, "task_state",10)
        #self.battery_subscriber= self.subscriptions(BatteryState, "/robot/robotnik_base_hw/robotnik_battery_broadcaster/battery", self.publish_battery_data, 10)


        self.status_timer = self.create_timer(timer_period_sec=self.heartbeat,callback= self.publish_fleet_state) 
        
    ##handle task
    #navigation request
    def destination_callback(self, msg:dds.FreeFleetDataDestinationRequest):
        step= self.received_new_action(msg)
        if str(step)==str(1):
            self.step=1
            self.start_navigation_task( msg.destination.x, msg.destination.y, msg.destination.yaw)
            
        else:
            self.destination_requests.append(msg)

    def start_navigation_task(self, x:float,y: float, yaw: float):
        self.start_navigation(x, y, yaw)
    
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
            print("e-drawer")
        else:
            self.drawer_publisher.publish(ros_msg)
            print("drawer")
        
    def close_drawer(self,module_id, drawer_id, e_drawer):
        if(not e_drawer):
            return
        ros_msg = DrawerAddress()
        ros_msg.module_id = module_id
        ros_msg.drawer_id = drawer_id
        self.e_drawer_close_publisher.publish(ros_msg)
        self.publish_task_state("DrawerState", str(module_id)+"#"+str(drawer_id)+"#Closed", False)
    
    def end_drawer_task(self):
        self.publish_task_state("DrawerAction", "Finished", True)
    
    def reset_simple_tree(self):
        tree_reset_msg=Bool()
        tree_reset_msg.data=True
        self.reset_simple_tree_publisher.publish(tree_reset_msg)

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


    def drawer_change_callback(self, msg:DrawerStatus):
        if(not msg.drawer_is_open):
            self.publish_task_state("DrawerState", str(msg.drawer_address.module_id)+"#"+str(msg.drawer_address.drawer_id)+"#Closed", False)

    # new_NFC request
    def new_user_callback(self, msg:dds.FreeFleetDataCreateNfcRequest):
        step= self.received_new_action(self, msg)
        if step==1:
            self.step=1
            self.start_new_user_request(msg.user_id)
        else:
            self.new_user_requests.append(msg)

    def receive_authentication_codes(self, msg):
        pass

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
        if msg.command == "loop":
                if msg.value=="start":
                    self.loop = True
                elif msg.value=="stop":
                    self.loop = False

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
                if len(self.open_drawers)==0:
                    self.end_drawer_task()
            elif msg.value=="reset":
                self.reset_simple_tree()
            else:
                selected_drawer= next((drawer for drawer in self.open_drawers if drawer.module_id== int(msg.value) and drawer.e_drawer), None)
                if selected_drawer is not None: 
                    self.close_drawer(selected_drawer.module_id,selected_drawer.drawer_id, selected_drawer.e_drawer)


    #publish states
    def publish_fleet_state(self):
        self.get_robot_location()
        battery= self.battery #todo @Torben read the battery sate from the robot
        sequence = [] #todo @Torben add a list of waypoint of the robot
        mode= dds.FreeFleetDataRobotMode()
        mode.mode=0 
        current_location= dds.FreeFleetDataLocation()
        current_location.sec= 0
        current_location.nanosec=0
        current_location.x= float(self.robot_x)
        current_location.y= float(self.robot_y)
        current_location.yaw= float(self.robot_yaw)
        current_location.level_name=""
        robot_state = dds.FreeFleetDataRobotState()
        robot_state.name=self.robot_name
        robot_state.model=self.robot_model
        task_id= ""
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
        if completed:
            self.find_next_action()

    def publish_battery_data(self,msg):
        self.battery=msg.voltage

    #Tasks
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
        self.loop=False
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
        
        destination_task= self.lookup_next_action(self.destination_requests)
        if(destination_task is not None):
            self.start_navigation_task( destination_task.destination.x, destination_task.destination.y, destination_task.destination.yaw)
            return 
        
        drawer_task= self.lookup_next_action(self.drawer_requests)
        if(drawer_task is not None):
            self.start_drawer_request(drawer_task.fleet_name, drawer_task.robot_name, drawer_task.e_drawer, drawer_task.restricted)
            return
        
        new_user_task= self.lookup_next_action(self.new_user_requests)
        if(new_user_task is not None):
            self.start_new_user_request(new_user_task.user_id)
            return
        
        self.finish_task()

    def finish_task(self):
        #self.publish_task_state("Task","Completed", True)
        self.clear_task()
        self.publish_fleet_state()

    def lookup_next_action(self, task_list:list):
        new_task= next((task for task in task_list if self.divide_task_id( task.task_id)[1]==self.step), None)
        if new_task is None:
            task_list.remove(new_task)
        return new_task 

    #suppost
    def received_new_action(self, msg):
        if not self.task_validation(msg):
            return
        (id, step)=self.divide_task_id(msg.task_id)
            
        if self.task_id != "" and self.task_id !=id:
            self.publish_task_state("Task","Postponed",False)
        self.swap_task(id)
        return step
    
    #nav
    def start_navigation(self, x, y, yaw):  
        print("start_nav")
        self.goal_pose =self.create_pose(x, y, yaw)
        print(self.goal_pose)

        self.nav.wait_for_server()

        self._send_goal_future = self.nav.send_goal_async(
                self.goal_pose,
                feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')

            self.publish_task_state( "canceld", "could not plan route to goal pose", True)
            return
        self.get_logger().info('Goal accepted')
        self.active= True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: NavigateToPose.Result):
        
        if self.loop:
            rate =self.create_rate( self.patrol_break_frequency)
            rate.sleep()
            if self.loop:
                sorted_list =map( lambda task:(self.divide_task_id( task.task_id)[1],task),self.destination_requests)
                next_action=next(action for action in sorted_list if action[0]>self.step)
                self.step= next_action[0]
                self.start_navigation_task( next_action.destination.x, next_action.destination.y, next_action.destination.yaw)
                return
        else:
                self.active=False
                self.find_next_action()


    def feedback_callback(self, feedback_msg: NavigateToPose.Feedback):
        self.get_logger().debug('Received feedback')
        
    
    def pause_navigation(self):
        self._send_goal_future.cancel()
        self.active=False

    def cancel_navigation(self):
        self.goal_pose = None
        self._send_goal_future.cancel()
        self.active=False
        
    def create_pose(self, pose_x, pose_y, pose_yaw) -> NavigateToPose.Goal:
        pose = NavigateToPose.Goal()
        waypoint=PoseStamped()
        waypoint.header.frame_id = self.frame_id
        waypoint.header.stamp.nanosec = 0
        waypoint.header.stamp.sec = 0
        waypoint.pose.position.x = pose_x
        waypoint.pose.position.y = pose_y
        waypoint.pose.position.z = 0.0
        qx, qy, qz, qw = math_helper.quaternion_from_euler(0, 0, pose_yaw)
        waypoint.pose.orientation.x = qx
        waypoint.pose.orientation.y = qy
        waypoint.pose.orientation.z = qz
        waypoint.pose.orientation.w = qw
        pose.pose=waypoint
        pose.behavior_tree= self.nav_behavior_tree()
        return pose

    # def get_robot_odom(self, data:Odometry):
    #     x = data.pose.pose.position.x
    #     y = data.pose.pose.position.y
    #     q1 = data.pose.pose.orientation.x
    #     q2 = data.pose.pose.orientation.y
    #     q3 = data.pose.pose.orientation.z
    #     q4 = data.pose.pose.orientation.w
    #     q = (q1, q2, q3, q4)
    #     roll, pitch, yaw= math_helper.euler_from_quaternion(q)
    #     th = math_helper.euler_angle_to_degree(yaw)
    #     yaw = math_helper.to_positive_angle(th)
    #     self.robot_x= float(x)
    #     self.robot_y=float(y)
    #     self.robot_yaw=float(yaw)
    
    def get_robot_location(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.goal_frame,
                self.start_frame,
                rclpy.time.Time())
            
        except TransformException as ex:
                self.get_logger().info( f'Could not transform {self.goal_frame} to {self.start_frame}: {ex}')
                return
        (_, _, yaw) = math_helper.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w])
        th = math_helper.euler_angle_to_degree(yaw)
        yaw = math_helper.to_positive_angle(th)
        self.robot_yaw=yaw
        self.robot_x=t.transform.translation.x
        self.robot_y=t.transform.translation.y
            
       