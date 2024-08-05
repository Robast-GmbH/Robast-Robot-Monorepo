import rclpy
from rclpy.node import Node

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult
from dispenser_ingestor_mock.robot_modules_api import RobotModulesAPI
from dispenser_ingestor_mock.module_process import (
    ModuleProcess,
    PICK_UP_PROCESS,
    DROP_OFF_PROCESS,
)

UPDATE_TIMER_PERIOD_IN_S = 1


class DispenserIngestorMock(Node):
    def __init__(self) -> None:
        super().__init__("dispenser_ingestor_mock")

        self.declare_parameter("middleware_url", "http://10.10.23.6:8003")
        middleware_url = self.get_parameter("middleware_url").value
        self.__robot_module_api = RobotModulesAPI(
            middleware_url,
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

        self.__module_processes: dict[str, ModuleProcess] = {}
        self.__timer = self.create_timer(
            UPDATE_TIMER_PERIOD_IN_S,
            self.__update_active_module_processes_callback,
        )

    def __on_request_callback(self, msg: DispenserRequest | IngestorRequest) -> None:
        module_process_request = ModuleProcess.from_request_msg(msg)
        current_module_process = self.__module_processes.get(
            module_process_request.robot_name
        )
        if (
            current_module_process is None
            or current_module_process.request_guid
            != module_process_request.request_guid
        ):
            self.get_logger().info("Started pick-up/drop-off process")
            self.__module_processes[module_process_request.robot_name] = (
                module_process_request
            )
            self.__robot_module_api.start_module_process(
                module_process_request,
            )

    def __update_active_module_processes_callback(self) -> None:
        for module_process in self.__module_processes.values():
            if module_process.is_in_process:
                self.__update_module_process(module_process)

    def __update_module_process(self, module_process: ModuleProcess) -> None:
        module_process_status = self.__robot_module_api.update_module_process_status(
            robot_name=module_process.robot_name,
            module_id=module_process.module_id,
            drawer_id=module_process.drawer_id,
        )
        if module_process_status is not None and module_process_status == "idle":
            msg = self.__create_result_msg(module_process)
            self.__publish_result(msg)
            module_process.is_in_process = False

    def __create_result_msg(
        self, module_process: ModuleProcess
    ) -> DispenserResult | IngestorResult | None:
        if module_process.process_name == PICK_UP_PROCESS:
            msg = DispenserResult()
            msg.status = DispenserResult.SUCCESS
        elif module_process.process_name == DROP_OFF_PROCESS:
            msg = IngestorResult()
            msg.status = IngestorResult.SUCCESS
        else:
            self.get_logger().error("Unknown request type")
            raise ValueError("Unknown request type")
        msg.request_guid = module_process.request_guid
        msg.source_guid = module_process.target_guid
        msg.time = self.get_clock().now().to_msg()
        return msg

    def __publish_result(self, msg: DispenserResult | IngestorResult) -> None:
        if isinstance(msg, DispenserResult):
            self.__dispenser_publisher.publish(msg)
        else:
            self.__ingestor_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    dispenser_ingestor_mock = DispenserIngestorMock()

    rclpy.spin(dispenser_ingestor_mock)

    dispenser_ingestor_mock.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
