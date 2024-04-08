import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult


class ProcessMock(Node):

    def __init__(self):
        super().__init__("process_mock")
        self.dispenser_publisher_ = self.create_publisher(
            DispenserResult, "/dispenser_results", 10
        )
        self.dispenser_subscriber_ = self.create_subscription(
            DispenserRequest,
            "/dispenser_requests",
            self.listener_callback,
            10,
        )

        self.ingestor_publisher_ = self.create_publisher(
            IngestorResult, "/ingestor_results", 10
        )
        self.ingestor_subscriber_ = self.create_subscription(
            IngestorRequest, "/ingestor_requests", self.listener_callback, 10
        )
        self.timer_period = 5.0  # seconds
        self.request_guid = ""
        self.source_guid = ""
        self.timer = None

    def timer_callback(self, request_type):
        self.timer.cancel()
        if request_type == "DispenserRequest":
            msg = DispenserResult()
        elif request_type == "IngestorRequest":
            msg = IngestorResult()
        else:
            self.get_logger().error("Unknown request type")
            return
        msg.time = self.get_clock().now().to_msg()
        msg.status = DispenserResult.SUCCESS
        msg.request_guid = self.request_guid
        msg.source_guid = self.source_guid
        self.publish_result(msg)
        self.get_logger().info("timer callback")
        self.timer.cancel()

    def publish_result(self, msg):
        if msg.__class__.__name__ == "DispenserResult":
            self.dispenser_publisher_.publish(msg)
        else:
            self.ingestor_publisher_.publish(msg)

    def listener_callback(self, msg):
        self.request_guid = msg.request_guid
        self.source_guid = msg.target_guid
        if self.timer is None or self.timer.is_canceled():
            request_type = msg.__class__.__name__
            self.timer = self.create_timer(
                self.timer_period, lambda: self.timer_callback(request_type)
            )
            self.get_logger().info("listener callback")


def main(args=None):
    rclpy.init(args=args)

    process_mock = ProcessMock()

    rclpy.spin(process_mock)

    process_mock.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
