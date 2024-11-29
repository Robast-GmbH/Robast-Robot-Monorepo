import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
import requests


class FireAlarmPublisher(Node):
    def __init__(self) -> None:
        super().__init__("fire_alarm_publisher")

        self.declare_parameter("middleware_address", "http://10.10.13.7:8003")
        middleware_address = self.get_parameter("middleware_address").value
        self.__url = f"{middleware_address}/fire_alarm/fire_alarm_triggered?source=fire_alarm_publisher"

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.__publisher = self.create_publisher(
            Bool, "fire_alarm_trigger", qos_profile
        )

        self.create_timer(1.0, self.__timer_callback)

    def __timer_callback(self) -> None:
        fire_alarm_triggered = self.__get_fire_alarm_triggered()
        if fire_alarm_triggered is not None:
            self.__publish_fire_alarm_triggered(fire_alarm_triggered)

    def __get_fire_alarm_triggered(self) -> bool | None:
        try:
            response = requests.get(self.__url, timeout=1)
            response.raise_for_status()
            data = response.json()
            return data["fire_alarm_triggered"]
        except requests.RequestException as e:
            self.get_logger().warn(f"HTTP-request failed: {e}")
            return None

    def __publish_fire_alarm_triggered(self, fire_alarm_triggered: bool) -> None:
        message = Bool()
        message.data = fire_alarm_triggered
        self.__publisher.publish(message)
        self.get_logger().debug(f"fire_alarm_triggered published: {message.data}")


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
