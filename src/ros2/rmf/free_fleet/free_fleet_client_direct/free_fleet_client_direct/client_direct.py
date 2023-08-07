import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node

from rclpy.node import Node
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from  .dds import dds_communicator as dds
from . import math_helper
from . import messages

from enum import Enum

from .dds.dds_communicator import DDS_communicator 
from.dds import messages

from communication_interfaces.msg import DrawerAddress
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from threading import Thread

# TODO@ Torben this aproach is rubbisch switch the logic to simple statemaschine  

class Robot_states(Enum):
    IDLE = 1
    DRAWERACTION = 2
    MOVEMENTACTION = 3
    TASKACTION =4

class free_fleet_client_direct(Node):

    def __init__(self):
        super().__init__('free_fleet_direct_client')
        # test=DDS_communicator(42,"test",messages.FreeFleetData_SlideDrawerRequest)
        # while True:
        #     answer =test.get_next()
        #     if(answer is not None):
        #         print( answer.fleet_name)
        #     print("")


        self.declare_parameter('fleet_name', 'ROBAST_1')
        self.declare_parameter('robot_name', 'RB0')
        self.declare_parameter('robot_frame_id', 'map')
        self.declare_parameter('robot_odom', '/odom')
        self.declare_parameter('heartbeat', 0.5)
        self.declare_parameter('statemaschine_open_drawer_topic', 'trigger_drawer_tree')
        self.declare_parameter('statemaschine_close_e_drawer_topic', 'close_drawer')
        self.declare_parameter('statemaschine_open_e_drawer_topic', 'trigger_electric_drawer_tree')
        self.declare_parameter('move_base_server_name', 'goal_pose')

        self.declare_parameter('dds_domain', 42)
        self.declare_parameter('dds_slide_drawer_topic', '/slide_drawer_request')
        self.declare_parameter('dds_nav_goal_topic', '/move_to_request')

        self.fleet_name = self.get_parameter('fleet_name').get_parameter_value().string_value
        self.name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter("robot_frame_id").get_parameter_value().string_value
        self.heartbeat= self.get_parameter('heartbeat').get_parameter_value().double_value
        self.ros_opendrawer_topic = self.get_parameter('statemaschine_open_drawer_topic').get_parameter_value().string_value
        self.ros_open_e_drawer_topic = self.get_parameter('statemaschine_open_e_drawer_topic').get_parameter_value().string_value
        self.ros_close_e_drawer_topic = self.get_parameter('statemaschine_close_e_drawer_topic').get_parameter_value().string_value
        self.ros_move_base_server_name=self.get_parameter('move_base_server_name').get_parameter_value().string_value
        self.dds_domain = self.get_parameter('dds_domain').get_parameter_value().integer_value
        self.dds_slide_drawer_topic=self.get_parameter('dds_slide_drawer_topic').get_parameter_value().string_value
        self.dds_move_to_topic=self.get_parameter('dds_nav_goal_topic').get_parameter_value().string_value 
        
        self.navigator = BasicNavigator()
        self.status= Robot_states.IDLE

        qos_profile_drawer = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
     
        self.drawer_publisher = self.create_publisher(DrawerAddress, self.ros_opendrawer_topic, qos_profile=qos_profile_drawer)
        self.e_drawer_open_publisher = self.create_publisher(DrawerAddress, self.ros_open_e_drawer_topic, qos_profile=qos_profile_drawer)
        self.e_drawer_close_publisher = self.create_publisher(DrawerAddress, self.ros_close_e_drawer_topic, qos_profile=qos_profile_drawer)
        timer_period = self.heartbeat
        self.drawer_subscriber= dds.DDS_communicator(self.dds_domain, self.dds_slide_drawer_topic, messages.FreeFleetData_SlideDrawerRequest)
        self.movement_subscriber= dds.DDS_communicator(self.dds_domain, self.dds_move_to_topic, messages.FreeFleetData_DestinationRequest)
        #sub = self.create_subscription('/odom',Odometry, self.get_robot_odom)
        self.action_timer = self.create_timer(timer_period, self.start_robot_behavior)
        #self.info_timer = self.create_timer(timer_period, self.start_sending_robot_info)
        

    def start_robot_behavior(self):
        if(self.status== Robot_states.IDLE):
            drawer_task = self.drawer_subscriber.get_next()
              
            if drawer_task is not None:
                print("drawer")
                self.do_drawer_action(drawer_task)
                return
            
            move_task = self.movement_subscriber.get_next()
            if move_task is not None:
                self.do_move_action(move_task)
                return
            
            # elif(self.status== Robot_states.DRAWERACTION):
            #     pass

            # elif(self.status== Robot_states.MOVEMENTACTION ):
            #     pass
            
    def start_sending_robot_info(self):  
        #return self.navigator.isTaskComplete()
        #return self.navigator.getResult()
        pass
    
        
    def check_task_recipient(self,task):
        if task.fleet_name == self.fleet_name and task.robot_name == self.name:
            return task
        else:
            return None

    def do_drawer_action(self, msg):
        self.status= Robot_states.DRAWERACTION
        ros_msg=self.dds_Drawer_msg_to_ros(msg)
        if(msg.open):
            #ToDo @Torben: nfc
            if(msg.e_drawer):
                self.e_drawer_open_publisher.publish(ros_msg)
            else:
                self.drawer_publisher.publish(ros_msg)
        else:
                self.e_drawer_close_publisher.publish(ros_msg)

    def dds_Drawer_msg_to_ros(self,dds_msg):
        ros_msg = DrawerAddress()
        ros_msg.module_id = dds_msg.module_id
        ros_msg.drawer_id = dds_msg.drawer_id
        return ros_msg            

    def do_move_action(self, msg):
        self.status= Robot_states.MOVEMENTACTION
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
      
        
    def create_pose(self, pose_x, pose_y, pose_yaw) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        qx, qy, qz, qw = math_helper.get_quaternion_from_euler(0, 0, pose_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def get_robot_odom(self, data):
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

        
       

