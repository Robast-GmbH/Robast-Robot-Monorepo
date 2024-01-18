import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log

class RosoutListener(Node):

    def __init__(self):
        super().__init__('rosout_listener')

        self.declare_parameter('trigger_message', '')
        self.trigger_message = self.get_parameter('trigger_message').get_parameter_value().string_value

        self.declare_parameter('node_name', '') # The name of the node we listen to
        self.node_name = self.get_parameter('node_name').get_parameter_value().string_value

        self.create_subscription(
            Log, '/rosout', self.log_callback, 10)

    def log_callback(self, msg):
        if self.node_name == msg.name:
            if self.trigger_message in msg.msg:
                self.get_logger().info("Trigger message found. Shutting down.")
                exit() # we need to call exit here so that we can use the event_handler OnProcessExit
                

def main(args=None):
    rclpy.init(args=args)
    rosout_listener = RosoutListener()
    rclpy.spin(rosout_listener)
    rosout_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
