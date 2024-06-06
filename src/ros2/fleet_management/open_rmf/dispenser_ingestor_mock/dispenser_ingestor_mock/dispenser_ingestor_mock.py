import rclpy
from rclpy.node import Node

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult
from dispenser_ingestor_mock.robot_modules_api import RobotModulesAPI


class DispenserIngestorMock(Node):
    def __init__(self):
        super().__init__("dispenser_ingestor_mock")

        self.declare_parameter("middleware_url", "http://10.10.23.6:8003")
        middleware_url = self.get_parameter("middleware_url").value
        self.__robot_drawer_api = RobotModulesAPI(
            middleware_url == middleware_url,
            robot_name="rb_theron",
        )

        self.__dispenser_publisher = self.create_publisher(
            DispenserResult, "/dispenser_results", 10
        )
        self.__dispenser_subscriber = self.create_subscription(
            DispenserRequest, "/dispenser_requests", self.__on_request_callback, 10
        )

        self.__ingestor_publisher = self.create_publisher(
            IngestorResult, "/ingestor_results", 10
        )
        self.__ingestor_subscriber = self.create_subscription(
            IngestorRequest, "/ingestor_requests", self.__on_request_callback, 10
        )

        self.__update_timer_period_in_seconds = 1
        self.__process_finish_delay_in_seconds = 5
        self.__request_guid = ""
        self.__target_guid = ""
        self.__timer = None
        self.__is_in_process = False

    def __create_result_msg(self):
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
        msg.request_guid = self.__request_guid
        msg.source_guid = self.__target_guid
        return msg

    def __update_drawer_status(self):
        module_process_status = self.__robot_drawer_api.update_module_process_status()
        if module_process_status is not None and module_process_status == "finished":
            self.__timer.cancel()
            msg = self.__create_result_msg()
            self.__publish_result_with_delay(msg)

    def __publish_result_with_delay(self, msg):
        if isinstance(self.current_request, DispenserRequest):
            self.__dispenser_publisher.publish(msg)
        else:
            self.__ingestor_publisher.publish(msg)
        self.__timer = self.create_timer(
            self.__process_finish_delay_in_seconds,
            lambda: self.__finish_process_callback(),
        )

    def __finish_process_callback(self):
        self.__is_in_process = False
        self.get_logger().info("Finished pick-up/drop-off process")
        self.__timer.cancel()

    def __on_request_callback(self, msg):
        if not self.__is_in_process:
            self.get_logger().info("Started pick-up/drop-off process")
            self.__is_in_process = True
            self.__request_guid = msg.request_guid
            self.__target_guid = msg.target_guid
            process_name = (
                "pick_up" if isinstance(msg, DispenserRequest) else "drop_off"
            )
            # For now the drawer_address is derived from the entered item type
            # Example: 2_0_item_type for module 2 drawer 0
            self.__robot_drawer_api.start_module_process(
                msg.items[0].type_guid, process_type=process_name
            )
            self.current_request = msg
            self.__timer = self.create_timer(
                self.__update_timer_period_in_seconds,
                lambda: self.__update_drawer_status(),
            )


def main(args=None):
    rclpy.init(args=args)

    dispenser_ingestor_mock = DispenserIngestorMock()

    rclpy.spin(dispenser_ingestor_mock)

    dispenser_ingestor_mock.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
