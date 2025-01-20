import rclpy
from rclpy.node import Node

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult
from dispenser_ingestor_mock.robot_modules_api import RobotModulesAPI
from dispenser_ingestor_mock.submodule_process import (
    SubmoduleProcess,
    PICK_UP_PROCESS,
    DROP_OFF_PROCESS,
)


class DispenserIngestorMock(Node):
    UPDATE_TIMER_PERIOD_IN_S = 1

    def __init__(self) -> None:
        super().__init__("dispenser_ingestor_mock")

        self.declare_parameter("middleware_address", "http://10.10.13.7:8003")
        middleware_address = self.get_parameter("middleware_address").value
        self.__robot_module_api = RobotModulesAPI(
            middleware_address,
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

        self.__submodule_processes: dict[str, SubmoduleProcess] = {}
        self.__timer = self.create_timer(
            self.UPDATE_TIMER_PERIOD_IN_S,
            self.__update_active_submodule_processes_callback,
        )

    def __on_request_callback(self, msg: DispenserRequest | IngestorRequest) -> None:
        submodule_process_request = SubmoduleProcess.from_request_msg(msg)
        current_submodule_process = self.__submodule_processes.get(
            submodule_process_request.robot_name
        )
        if (
            current_submodule_process is None
            or current_submodule_process.request_guid
            != submodule_process_request.request_guid
        ):
            self.get_logger().info("Started pick-up/drop-off process")
            self.__submodule_processes[submodule_process_request.robot_name] = (
                submodule_process_request
            )
            self.__robot_module_api.start_submodule_process(
                submodule_process_request,
            )

    def __update_active_submodule_processes_callback(self) -> None:
        for submodule_process in self.__submodule_processes.values():
            if submodule_process.is_in_process:
                self.__update_submodule_process(submodule_process)

    def __update_submodule_process(self, submodule_process: SubmoduleProcess) -> None:
        submodule_process_status = (
            self.__robot_module_api.update_submodule_process_status(
                robot_name=submodule_process.robot_name,
                module_id=submodule_process.module_id,
                submodule_id=submodule_process.submodule_id,
            )
        )
        if submodule_process_status is not None and submodule_process_status == "idle":
            msg = self.__create_result_msg(submodule_process)
            self.__publish_result(msg)
            submodule_process.is_in_process = False

    def __create_result_msg(
        self, submodule_process: SubmoduleProcess
    ) -> DispenserResult | IngestorResult | None:
        if submodule_process.process_name == PICK_UP_PROCESS:
            msg = DispenserResult()
            msg.status = DispenserResult.SUCCESS
        elif submodule_process.process_name == DROP_OFF_PROCESS:
            msg = IngestorResult()
            msg.status = IngestorResult.SUCCESS
        else:
            self.get_logger().error("Unknown request type")
            raise ValueError("Unknown request type")
        msg.request_guid = submodule_process.request_guid
        msg.source_guid = submodule_process.target_guid
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
