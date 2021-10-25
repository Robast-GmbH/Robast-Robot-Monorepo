import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from nav_msgs.msg import OccupancyGrid
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile, DurabilityPolicy
from std_srvs.srv import SetBool
import time

class MapBuffer(Node):

    def __init__(self):
        super().__init__('map_buffer')
        self.get_logger().info("Started Map Buffer")
        self.received_map = False

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.trigger_map_publish_service = self.create_service(
            SetBool,
            'trigger_robast_map_publishing',
            self.trigger_map_publish_callback)
        
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/robast_map',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        

    def handle_map_publish(self):
         if self.received_map:
            self.get_logger().info('Publishing robast map!')
            self.map_publisher.publish(self.robast_map)


    def map_callback(self, msg):        
        self.get_logger().info('Received new map!')
        self.robast_map = msg
        if self.received_map == False:
            # If a map was received for the first time, publish map once to make sure that there is
            # at least one map in the topic (DurabilityPolicy.TRANSIENT_LOCAL ensures that subscriber,
            # that subscribe at a later moment, receive the map)
            self.received_map = True
            self.map_publisher.publish(msg)
            self.get_logger().info('Published Robast map for the first time!')

    
    def publish_robast_map(self):
        # Check if we allready received a map at least once
        if self.received_map:
            self.map_publisher.publish(self.robast_map)
            self.get_logger().info('Published Robast map!')
            return True
        else:
            self.get_logger().info('The Map Buffer has not yet received a map, so the robast map cannot be published!')
            return False


    def trigger_map_publish_callback(self, request, response):
        self.get_logger().info('Publishing Robast map triggered!')
        response.success = self.publish_robast_map()
        return response

            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    robast_map_server = MapBuffer()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(robast_map_server)
    # Explicity destroy the node
    robast_map_server.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()