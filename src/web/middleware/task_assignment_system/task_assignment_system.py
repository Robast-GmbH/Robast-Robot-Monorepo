from task_assignment_system.models.robot import Robot
from task_assignment_system.models.nav_graph import NavGraph
from pydantic_models.delivery_request import DeliveryRequest
from task_assignment_system.models.fleet_management_api import FleetManagementAPI
from module_manager.module_manager import ModuleManager
from configs.url_config import (
    fleet_management_address,
    robot_name_to_ip,
)

from threading import Timer
from typing import Tuple
import json

TASK_ASSIGNMENT_TRIGGER_INTERVAL_IN_SECONDS = 10


class TaskAssignmentSystem:
    def __init__(
        self,
    ) -> None:
        path = "task_assignment_system/configs/nav_config.json"
        with open(path, "r") as file:
            data = json.load(file)["levels"][0]["nav_graphs"][0]
        self.nav_graph = NavGraph.from_json(data=data)
        self.available_drawer_types = set()
        self.fleet_management_api = FleetManagementAPI(fleet_management_address)
        self.module_manager = ModuleManager()
        self.robots = self.__init_robots(
            self.fleet_management_api,
            self.module_manager,
        )

        self.request_queue = []
        self.timer = None

    def receive_request(
        self,
        delivery_request: DeliveryRequest,
    ) -> Tuple[bool, str]:
        if self.timer is not None:
            self.timer.cancel()

        # TODO taskvalidation
        if delivery_request.target_id is None and delivery_request.start_id is None:
            return False, "Invalid request, target node not found."
        if not self.module_manager.is_module_size_available(
            "rb_theron", delivery_request.required_drawer_type
        ):
            return False, "Invalid request, drawer type not available."

        self.request_queue.append(delivery_request)
        self.__trigger_task_assignment()
        return True, "Request added to queue."

    def __trigger_task_assignment(self) -> None:
        if len(self.request_queue) > 0:
            delivery_request = self.request_queue.pop(0)

            assignee = self.__find_cheapest_assignment(delivery_request)

            if assignee is not None:
                assignee.accept_request(delivery_request)
            else:
                self.request_queue.append(delivery_request)

        self.timer = Timer(
            TASK_ASSIGNMENT_TRIGGER_INTERVAL_IN_SECONDS,
            self.__trigger_task_assignment,
        )
        self.timer.start()

    def __find_cheapest_assignment(
        self, delivery_request: DeliveryRequest
    ) -> Robot | None:
        min_cost = float("inf")
        min_cost_robot = None

        for robot in self.robots.values():
            cost = robot.get_request_cost(delivery_request)
            if cost < min_cost:
                min_cost = cost
                min_cost_robot = robot

        if min_cost != float("inf"):
            return min_cost_robot
        else:
            return None

    def __init_robots(
        self,
        fleet_management_api: FleetManagementAPI,
        module_manager: ModuleManager,
    ) -> dict[str, Robot]:
        robots = {}
        for robot_name in robot_name_to_ip.keys():
            robots[robot_name] = Robot(
                name=robot_name,
                fleet_management_api=fleet_management_api,
                module_manager=module_manager,
                # TODO: Find a clever way to dynamically initialize the robots at different nodes
                initial_node=self.nav_graph.nodes[15],
                nav_graph=self.nav_graph,
            )

        return robots
