import rclpy
from rclpy.node import Node

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult
from dispenser_ingestor_mock.robot_api import RobotAPI

DRAWER_IDLE = 0
DRAWER_OPEN = 1

class DispenserIngestorMock(Node):

    def __init__(self):
        super().__init__("dispenser_ingestor_mock")

        self.dispenser_publisher_ = self.create_publisher(
            DispenserResult, "/dispenser_results", 10
        )
        self.dispenser_subscriber_ = self.create_subscription(
            DispenserRequest,
            "/dispenser_requests",
            self.on_request_callback,
            10,
        )

        self.ingestor_publisher_ = self.create_publisher(
            IngestorResult, "/ingestor_results", 10
        )
        self.ingestor_subscriber_ = self.create_subscription(
            IngestorRequest, "/ingestor_requests", self.on_request_callback, 10
        )
        self.timer_period = 1.0  # seconds
        self.process_finish_delay = 5  # seconds
        self.request_guid = ""
        self.source_guid = ""
        self.timer = None
        self.drawer_status = DRAWER_IDLE
        self.is_in_process = False
        self.robot_api = RobotAPI()

    def create_result_msg(self):
        if isinstance(self.current_request, DispenserRequest):
            msg = DispenserResult()
            msg.status = DispenserResult.SUCCESS
        elif isinstance(self.current_request, IngestorRequest):
            msg = IngestorResult()
            msg.status = IngestorResult.SUCCESS
        else:
            self.get_logger().error("Unknown request type")
            return
        msg.time = self.get_clock().now().to_msg()
        msg.request_guid = self.request_guid
        msg.source_guid = self.source_guid
        return msg


    def drawer_status_request(self):
        is_drawer_open = self.robot_api.is_drawer_open()
        if self.drawer_status == DRAWER_IDLE and is_drawer_open:
            self.drawer_status = DRAWER_OPEN
        elif self.drawer_status == DRAWER_OPEN and not is_drawer_open:
            self.drawer_status = DRAWER_IDLE
            self.timer.cancel()
            msg = self.create_result_msg()
            self.publish_result_with_delay(msg)


    def publish_result_with_delay(self, msg):
        if isinstance(self.current_request, DispenserRequest):
            self.dispenser_publisher_.publish(msg)
        else:
            self.ingestor_publisher_.publish(msg)
        self.timer = self.create_timer(
            self.process_finish_delay, lambda : self.finish_process_callback()
        )

    def finish_process_callback(self):
        self.is_in_process = False
        self.timer.cancel()

    def on_request_callback(self, msg):
        if not self.is_in_process:
            self.is_in_process = True
            self.request_guid = msg.request_guid
            self.source_guid = msg.target_guid
            self.robot_api.open_drawer()
            self.current_request = msg
            self.timer = self.create_timer(
                self.timer_period, lambda : self.drawer_status_request()
            )
            self.get_logger().info("started drawer request timer")


def main(args=None):
    rclpy.init(args=args)

    dispenser_ingestor_mock = DispenserIngestorMock()

    rclpy.spin(dispenser_ingestor_mock)

    dispenser_ingestor_mock.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
