import rclpy
from rclpy.node import Node

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult
from dispenser_ingestor_mock.robot_drawer_api import (
    RobotDrawerAPI,
    DRAWER_IDLE,
    DRAWER_OPEN,
)


class DispenserIngestorMock(Node):
    def __init__(self):
        super().__init__("dispenser_ingestor_mock")

        self.declare_parameter("api_url", "http://localhost:8003")
        api_url = self.get_parameter("api_url").value
        self.robot_drawer_api = RobotDrawerAPI(api_url=api_url)

        self._dispenser_publisher = self.create_publisher(
            DispenserResult, "/dispenser_results", 10
        )
        self._dispenser_subscriber = self.create_subscription(
            DispenserRequest, "/dispenser_requests", self.on_request_callback, 10
        )

        self._ingestor_publisher = self.create_publisher(
            IngestorResult, "/ingestor_results", 10
        )
        self._ingestor_subscriber = self.create_subscription(
            IngestorRequest, "/ingestor_requests", self.on_request_callback, 10
        )

        self.update_timer_period = 1.0  # seconds
        self.process_finish_delay = 5  # seconds
        self.request_guid = ""
        self.target_guid = ""
        self.timer = None
        self.drawer_status = DRAWER_IDLE
        self.is_in_process = False

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
        msg.source_guid = self.target_guid
        return msg

    def update_drawer_status(self):
        self.get_logger().info("Updating drawer status")
        is_drawer_open = self.robot_drawer_api.is_drawer_open()
        if is_drawer_open is None:
            return
        if self.drawer_status == DRAWER_IDLE and is_drawer_open:
            self.drawer_status = DRAWER_OPEN
        elif self.drawer_status == DRAWER_OPEN and not is_drawer_open:
            self.drawer_status = DRAWER_IDLE
            self.timer.cancel()
            msg = self.create_result_msg()
            self.publish_result_with_delay(msg)

    def publish_result_with_delay(self, msg):
        if isinstance(self.current_request, DispenserRequest):
            self._dispenser_publisher.publish(msg)
        else:
            self._ingestor_publisher.publish(msg)
        self.timer = self.create_timer(
            self.process_finish_delay, lambda: self.finish_process_callback()
        )

    def finish_process_callback(self):
        self.is_in_process = False
        self.get_logger().info("Finished pick-up/drop-off process")
        self.timer.cancel()

    def on_request_callback(self, msg):
        if not self.is_in_process:
            self.get_logger().info("Started pick-up/drop-off process")
            self.is_in_process = True
            self.request_guid = msg.request_guid
            self.target_guid = msg.target_guid
            # For now the drawer_address is derived from the entered item type
            # Example: 2_0_item_type for module 2 drawer 0
            self.robot_drawer_api.open_drawer(msg.items[0].type_guid)
            self.current_request = msg
            self.timer = self.create_timer(
                self.update_timer_period, lambda: self.update_drawer_status()
            )


def main(args=None):
    rclpy.init(args=args)

    dispenser_ingestor_mock = DispenserIngestorMock()

    rclpy.spin(dispenser_ingestor_mock)

    dispenser_ingestor_mock.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
