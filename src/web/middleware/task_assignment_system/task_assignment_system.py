from task_assignment_system.models.robot import Robot
from task_assignment_system.models.nav_graph import NavGraph
from pydantic_models.task import Task
from task_assignment_system.models.fleet_management_api import FleetManagementAPI
from module_manager.module_manager import ModuleManager
from configs.url_config import FLEET_MANAGEMENT_ADDRESS, ROBOT_NAME_TO_IP

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
        self.fleet_management_api = FleetManagementAPI(FLEET_MANAGEMENT_ADDRESS)
        self.module_manager = ModuleManager()
        self.robots = self.__init_robots(
            self.fleet_management_api,
            self.module_manager,
        )

        self.request_queue = []
        self.timer = None

    def receive_task(
        self,
        task: Task,
    ) -> Tuple[bool, str]:
        if self.timer is not None:
            self.timer.cancel()

        if not self.__validate_subtask_targets(task):
            return False, "Invalid request, target node not found."
        if not self.__validate_task_requirements(task):
            return False, "Invalid request, drawer type not mounted in fleet."

        self.request_queue.append(task)
        self.__trigger_task_assignment()
        return True, "Request added to queue."

    def __validate_task_requirements(self, task: Task) -> bool:
        if "required_drawer_type" in task.requirements:
            for robot in self.robots.values():
                if not self.module_manager.is_module_size_mounted(
                    robot.name, task.requirements["required_drawer_type"]
                ):
                    return False
        return True

    def __validate_subtask_targets(self, task: Task) -> bool:
        for subtask in task.subtasks:
            try:
                self.nav_graph.get_node_by_id(subtask.target_id)
            except ValueError:
                return False
        return True

    def __trigger_task_assignment(self) -> None:
        if len(self.request_queue) > 0:
            delivery_request = self.request_queue.pop(0)

            assignee = self.__find_cheapest_assignment(delivery_request)

            if assignee is not None:
                is_accepted = assignee.accept_request(delivery_request)
            if assignee is None or not is_accepted:
                self.request_queue.append(delivery_request)

        self.timer = Timer(
            TASK_ASSIGNMENT_TRIGGER_INTERVAL_IN_SECONDS,
            self.__trigger_task_assignment,
        )
        self.timer.start()

    def __find_cheapest_assignment(self, task_request: Task) -> Robot | None:
        min_cost = float("inf")
        min_cost_robot = None

        for robot in self.robots.values():
            cost = robot.get_request_cost(task_request)
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
        for robot_name in ROBOT_NAME_TO_IP.keys():
            robots[robot_name] = Robot(
                name=robot_name,
                fleet_management_api=fleet_management_api,
                module_manager=module_manager,
                initial_node=self.nav_graph.nodes[15],
                nav_graph=self.nav_graph,
            )

        return robots
