# Import AddTwoInts service type from the example_interfaces package
from api_ros_interfaces.srv import Add
from api_ros_interfaces.srv import Multi
 
# ROS 2 Client Library for Python
import rclpy
 
# Handles nodes
from rclpy.node import Node
 
class MinimalService(Node):
 
    def __init__(self):
        # Initialize the node via this constructor
        super().__init__('minimal_service')
      
        # Create a service
        self.srv = self.create_service(Add, 'add', self.add_two_ints_callback)
        self.srv = self.create_service(Multi, 'multi', self.multi_two_ints_callback)
 
    def add_two_ints_callback(self, request, response):
        # Receive the request data and sum it
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        response.sum = request.a + request.b

        # Return the sum as the reply
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def multi_two_ints_callback(self, request, response):
        # Receive the request data and sum it
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        response.product = request.a * request.b

        # Return the sum as the reply
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
 
def main(args=None):
 
    # Start ROS
    rclpy.init(args=args)
 
    # Create the service
    minimal_service = MinimalService()
 
    # Make the service available to the network
    rclpy.spin(minimal_service)
 
    # Shutdown ROS
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()