from task_system.models.robot import Robot
from task_system.models.nav_graph import NavGraph
from pydantic_models.task import Task
from task_system.task_repository import TaskRepository
from configs.url_config import ROBOT_NAME_TO_IP

from threading import Timer
from typing import Tuple
import json

TASK_ASSIGNMENT_TRIGGER_INTERVAL_IN_SECONDS = 10


class TaskAssignmentSystem:
    def __init__(
        self,
    ) -> None:
        rmf_nav_config_path = "task_system/configs/nav_config.json"
        with open(rmf_nav_config_path, "r") as file:
            nav_graph_json = json.load(file)["levels"][0]["nav_graphs"][0]
        self.__nav_graph = NavGraph.from_json(data=nav_graph_json)
        self.__task_repository = TaskRepository()
        self.__robots = self.__init_robots()
        self.__task_assignment_trigger_timer = None
        self.__start_task_assignment_trigger_timer()

    def receive_task(
        self,
        task: Task,
    ) -> Tuple[bool, str]:
        if self.__task_assignment_trigger_timer is not None:
            self.__task_assignment_trigger_timer.cancel()

        if not self.__validate_subtask_targets(task):
            return False, "Invalid request, target node not found."
        if not self.__validate_task_requirements(task):
            return False, "Invalid request, drawer type not mounted in fleet."

        self.__task_repository.create_task(task)

        self.__trigger_task_assignment()
        return True, "Request added to queue."

    def __validate_task_requirements(self, task: Task) -> bool:
        if "required_drawer_type" in task.requirements:
            robots_with_required_drawer_type = [
                robot
                for robot in self.__robots.values()
                if robot.is_drawer_type_mounted(
                    task.requirements["required_drawer_type"]
                )
            ]
            if not robots_with_required_drawer_type:
                return False
        return True

    def __validate_subtask_targets(self, task: Task) -> bool:
        for subtask in task.subtasks:
            try:
                self.__nav_graph.get_node_by_id(subtask.target_id)
            except ValueError:
                return False
        return True

    def __start_task_assignment_trigger_timer(self) -> None:
        self.__task_assignment_trigger_timer = Timer(
            TASK_ASSIGNMENT_TRIGGER_INTERVAL_IN_SECONDS,
            self.__trigger_task_assignment,
        )
        self.__task_assignment_trigger_timer.start()

    def __trigger_task_assignment(self) -> None:
        unassigned_tasks = self.__task_repository.read_unassigned_tasks()
        if len(unassigned_tasks) > 0:
            unassigned_task = unassigned_tasks[0]

            assignee = self.__find_cheapest_assignment(unassigned_task)

            if assignee is not None:
                print(f"Assigning task {unassigned_task.id}")
                assignee.accept_task(unassigned_task)

        self.__start_task_assignment_trigger_timer()

    def __find_cheapest_assignment(self, task_request: Task) -> Robot | None:
        min_cost = float("inf")
        min_cost_robot = None

        for robot in self.__robots.values():
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
    ) -> dict[str, Robot]:
        robots = {}
        for robot_name in ROBOT_NAME_TO_IP.keys():
            robots[robot_name] = Robot(
                name=robot_name,
                initial_node=self.__nav_graph.nodes[15],
                nav_graph=self.__nav_graph,
            )

        return robots
