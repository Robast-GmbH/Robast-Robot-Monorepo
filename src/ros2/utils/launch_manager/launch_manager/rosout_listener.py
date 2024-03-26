import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import sys


class RosoutListener(Node):
    def __init__(self):
        super().__init__("rosout_listener")

        self.declare_parameter("trigger_message", "")
        self.trigger_message = (
            self.get_parameter("trigger_message").get_parameter_value().string_value
        )

        self.declare_parameter("node_name", "")  # The name of the node we listen to
        self.node_name = (
            self.get_parameter("node_name").get_parameter_value().string_value
        )
        self.get_logger().info(
            "Listeing to the '/rosout' topic for the trigger message '"
            + self.trigger_message
            + "' published by the node name '"
            + self.node_name
            + "'."
        )

        self.create_subscription(Log, "/rosout", self.log_callback, 10)

    def log_callback(self, msg):
        if self.node_name == msg.name and self.trigger_message in msg.msg:
            self.get_logger().info("Trigger message found. Shutting down.")
            self.destroy_node()
            sys.exit()
            # TODO@SomeoneSmarter:
            # The purpose of the node is, that this should trigger a 'OnProcessExit' event
            # in a launch file. This works, if you exit the node.
            # Unfortunately, I have found no way to write a test for this behavior.
            # My test works, if I destroy and shutdown the node, but then the 'OnProcessExit'
            # event is not triggered.
            # Furthermore, I don't know if exiting a node like this has any bad side effects.
            # For now it works for my launch file, so I leave it like this.


def main(args=None):
    rclpy.init(args=args)
    rosout_listener = RosoutListener()

    try:
        rclpy.spin(rosout_listener)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
