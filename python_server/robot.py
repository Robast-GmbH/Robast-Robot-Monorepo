# System modules
from datetime import datetime

# 3rd party modules
from flask import make_response, abort
import sys
 
from api_ros_interfaces.srv import Multi
import rclpy
from rclpy.node import Node
 
class MinimalClientAsync(Node):
 
    def __init__(self):
        # Create a client    
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Multi, '/multi')
       
        # Check if the a service is available  
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Multi.Request()
 
    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

def call_a_Funktion(name, parameter ):
    rclpy.init(args=None)
    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        # See if the service has replied
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (minimal_client.req.a, minimal_client.req.b, response.sum))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()
 
 
def robots():
   call_a_Funktion()
 

def navToRoom(waypoints,lrobot):
    for wp in waypoints:
        print(wp)

def randomNavToRoom(waypoints):
    None

def addWaypoint(waypoints, lrobot):
   None

    
def addTask(task, lrobot, waypoints= None):
   None

def randomAddTask(task, waypoints= None):
    None

def getTask(lrobot):
    return "not implemented"

def getPrio(lrobot):
    return -1

def getTimeToIdle(lrobot):
    return -1

def getIdleBots():
    return []



    
        
