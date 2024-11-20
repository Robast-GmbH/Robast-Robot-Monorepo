from task_system.models.robot import Robot
from task_system.models.nav_graph import NavGraph
from pydantic_models.task_request import TaskRequest
from db_models.task import Task
from task_system.task_manager import TaskManager
from configs.url_config import ROBOT_NAME_TO_IP

from threading import Timer
from typing import Tuple, Any
import time
import json


class TaskAssignmentSystem:
    TASK_ASSIGNMENT_TRIGGER_INTERVAL_IN_S = 10

    def __init__(
        self,
    ) -> None:
        rmf_nav_config_path = "task_system/configs/nav_config.json"
        with open(rmf_nav_config_path, "r") as file:
            nav_graph_json = json.load(file)["levels"][0]["nav_graphs"][0]
        self.__nav_graph = NavGraph.from_json(data=nav_graph_json)
        self.__task_manager = TaskManager()
        self.__robots = self.__init_robots()
        self.__task_assignment_trigger_timer = None
        self.__start_task_assignment_trigger_timer()

    def get_robot_tasks(self, robot_name: str) -> dict[str, Any]:
        if robot_name not in self.__robots:
            return {"status": "failure", "message": "Robot not found."}
        return {"status": "success", "tasks": self.__robots[robot_name].get_subtasks()}

    def receive_task(
        self,
        task: TaskRequest,
    ) -> Tuple[bool, str]:
        if not self.__validate_subtask_targets(task):
            return False, "Invalid request, target node not found."
        if not self.__validate_task_requirements(task):
            return False, "Invalid request, submodule type not mounted in fleet."
        if task.assignee_name and task.assignee_name not in self.__robots:
            return False, "Invalid request, assignee not found."

        if self.__task_assignment_trigger_timer is not None:
            self.__task_assignment_trigger_timer.cancel()

        self.__task_manager.create_task(task.to_db_task())

        if task.assignee_name:
            robot = self.__robots[task.assignee_name]
            robot.accept_direct_task(task.to_db_task())

        self.__trigger_task_assignment()
        return True, "Request added to queue."

    def __validate_task_requirements(self, task: TaskRequest) -> bool:
        if "required_submodule_type" in task.requirements:
            robots_with_required_submodule_type = [
                robot
                for robot in self.__robots.values()
                if robot.is_submodule_type_mounted(
                    task.requirements["required_submodule_type"]
                )
            ]
            if not robots_with_required_submodule_type:
                return False
        return True

    def __validate_subtask_targets(self, task: TaskRequest) -> bool:
        for subtask in task.subtasks:
            try:
                self.__nav_graph.get_node_by_id(subtask.target_id)
            except ValueError:
                return False
        return True

    def __start_task_assignment_trigger_timer(self) -> None:
        self.__task_assignment_trigger_timer = Timer(
            self.TASK_ASSIGNMENT_TRIGGER_INTERVAL_IN_S,
            self.__trigger_task_assignment,
        )
        self.__task_assignment_trigger_timer.start()

    def __trigger_task_assignment(self) -> None:
        unassigned_tasks = self.__task_manager.read_unassigned_tasks()
        unassigned_tasks = [
            task
            for task in unassigned_tasks
            if task.earliest_start_time <= int(time.time())
        ]
        if len(unassigned_tasks) > 0:
            unassigned_task = unassigned_tasks[0]

            assignee = self.__find_cheapest_assignment(unassigned_task)
            if assignee and assignee.accept_assigned_task(unassigned_task):
                print(f"Assigned task {unassigned_task.id} to {assignee}")

        self.__start_task_assignment_trigger_timer()

    def __find_cheapest_assignment(self, task: Task) -> Robot | None:
        min_cost = float("inf")
        min_cost_robot = None

        for robot in self.__robots.values():
            cost = robot.get_request_cost(task)
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
