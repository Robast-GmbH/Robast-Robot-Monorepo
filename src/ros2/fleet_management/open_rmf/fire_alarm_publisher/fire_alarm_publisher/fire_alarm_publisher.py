import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
import requests


class FireAlarmPublisher(Node):
    def __init__(self):
        super().__init__("fire_alarm_publisher")

        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
)   
  
        self.__publisher = self.create_publisher(Bool, "fire_alarm_trigger", qos_profile)
        self.__url = "http://10.10.13.7:8003/fire_alarm/fire_alarm_triggered?source=fire_alarm_publisher"
        self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("node started")

    def timer_callback(self):
        try:
            response = requests.get(self.__url, timeout=1)
            response.raise_for_status()
            data = response.json()
            message = Bool()
            message.data = data["fire_alarm_triggered"]
            self.__publisher.publish(message)

            self.get_logger().info(f"fire_alarm_triggered published: {message.data}")
        except requests.RequestException as e:
            self.get_logger().warn(f"HTTP-request failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FireAlarmPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node is shutting down")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
