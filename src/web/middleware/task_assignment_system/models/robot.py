from pydantic_models.sub_task import SubTask
from pydantic_models.drawer_address import DrawerAddress
from task_assignment_system.models.node import Node
from task_assignment_system.models.fleet_management_api import FleetManagementAPI
from task_assignment_system.models.nav_graph import NavGraph
from pydantic_models.task import Task
from module_manager.module_manager import ModuleManager
from typing import Any
import threading

UPDATE_TIMER_INTERVAL_IN_SECONDS = 1


class Robot:
    def __init__(
        self,
        name: str,
        fleet_management_api: FleetManagementAPI,
        module_manager: ModuleManager,
        initial_node: Node,
        nav_graph: NavGraph,
    ) -> None:
        self.name = name
        self.fleet_management_api = fleet_management_api
        self.module_manager = module_manager
        self.current_task = None
        self.current_task_id = None
        self.task_queue: list[SubTask] = []
        self.current_node = initial_node
        self.nav_graph = nav_graph
        self.done_tasks_ids: list[str] = []
        self.task_queue_lock = threading.Lock()
        print(
            f"Robot {self.name} initialized at node {self.current_node.id} -> Starting task status update timer"
        )
        self.__start_task_status_update_timer()

    def get_request_cost(self, task: Task) -> float:
        # check if any drawer of required drawer_type is available
        if "required_drawer_type" in task.requirements:
            is_drawer_available = self.module_manager.is_module_size_available(
                self.name,
                int(task.requirements["required_drawer_type"]),
            )
            if not is_drawer_available:
                return float("inf")

        return float(len(self.task_queue))

    def accept_request(self, task: Task) -> bool:
        with self.task_queue_lock:
            if not self.__handle_requirements(task):
                return False
            task.assignee_name = self.name
            for subtask in task.subtasks:
                subtask.assignee_name = self.name
            self.task_queue.extend(task.subtasks)
            self.__optimize_task_queue()
            return True

    def get_robot_tasks(self) -> dict[str, Any]:
        return {
            "active": self.current_task.to_json() if self.current_task else None,
            "queued": [task.to_json() for task in self.task_queue],
        }

    def __check_subtask_for_drawer_process(self, subtask: SubTask) -> bool:
        action = subtask.action
        while action:
            if action.name == "drawer_process":
                return True
            action = action.subaction
        return False

    def __write_drawer_address_to_subtask(
        self, subtask: SubTask, drawer_address: DrawerAddress
    ) -> None:
        subtask.requirements["drawer_address"] = drawer_address.to_json()
        action = subtask.action
        while action:
            if action.name == "drawer_process":
                action.parameters["drawer_address"] = drawer_address.to_json()
            action = action.subaction

    def __handle_requirements(self, task: Task) -> bool:
        if "required_drawer_type" in task.requirements:
            drawer = self.module_manager.try_reserve_module_type(
                self.name, task.requirements["required_drawer_type"], task.id, [], []
            )
            if not drawer:
                return False
            subtasks_with_drawer_process = [
                subtask
                for subtask in task.subtasks
                if self.__check_subtask_for_drawer_process(subtask)
            ]
            for subtask in subtasks_with_drawer_process:
                self.__write_drawer_address_to_subtask(subtask, drawer.address)
            self.__handle_subtask_requirements(subtasks_with_drawer_process[0])
        return True

    def __handle_subtask_requirements(self, subtask: SubTask) -> bool:
        if subtask.requirements["drawer_address"]:
            was_successful = self.module_manager.reserve_module(
                drawer_address=DrawerAddress.from_json(
                    subtask.requirements["drawer_address"]
                ),
                task_id=subtask.parent_id,
                user_ids=subtask.requirements["required_user_ids"],
                user_groups=subtask.requirements["required_user_groups"],
            )
            if not was_successful:
                return False
        return True

    def __start_task_status_update_timer(self) -> None:
        self.timer = threading.Timer(
            UPDATE_TIMER_INTERVAL_IN_SECONDS, self.__task_status_update_callback
        )
        self.timer.start()

    def __task_status_update_callback(self) -> None:
        with self.task_queue_lock:
            if self.current_task is None and self.task_queue:
                self.__start_next_task()
            elif self.__is_task_completed():
                self.__finish_task()

        self.__start_task_status_update_timer()

    def __is_task_completed(self) -> bool:
        if self.current_task_id is None:
            return True
        data = self.fleet_management_api.request_task_status(self.current_task_id)
        if data is None:
            return False
        completed_phases = data["completed"]
        active_phase = data["active"]
        return active_phase in completed_phases

    def __finish_task(self) -> None:
        if self.current_task is None:
            return
        self.done_tasks_ids.append(self.current_task.id)
        self.current_task.status = "completed"
        related_drawer_tasks = [
            task
            for task in self.task_queue
            if self.current_task.parent_id == task.parent_id
            and self.__check_subtask_for_drawer_process(task)
        ]
        if related_drawer_tasks:
            self.__handle_subtask_requirements(related_drawer_tasks[0])
        elif self.__check_subtask_for_drawer_process(self.current_task) or not [
            task
            for task in self.task_queue
            if task.parent_id == self.current_task.parent_id
        ]:
            self.module_manager.free_module(
                DrawerAddress.from_json(
                    self.current_task.requirements["drawer_address"]
                )
            )
        self.current_task = None

    def __start_next_task(self) -> None:
        self.current_task = self.task_queue.pop(0)
        self.current_task.status = "active"
        result = self.fleet_management_api.dispatch_robot_task(
            self.current_task.to_robot_task_request()
        )
        if result is not None:
            print(result)
            assigned_id = result["state"]["booking"]["id"]
            self.current_task_id = assigned_id
        else:
            self.current_task.status = "pending"
            self.task_queue.insert(0, self.current_task)

    def __optimize_task_queue(self) -> None:
        eligible_tasks = [
            task
            for task in self.task_queue
            if task.requires_task_id is None
            or task.requires_task_id in self.done_tasks_ids
            or (
                self.current_task is not None
                and task.requires_task_id == self.current_task.id
            )
        ]
        non_eligible_tasks = [
            task for task in self.task_queue if task not in eligible_tasks
        ]
        self.task_queue.clear()

        if self.current_task is None:
            start = self.current_node
        else:
            start = self.nav_graph.get_node_by_id(self.current_task.target_id)

        while len(eligible_tasks) > 0:
            min_distance = float("inf")
            closest_tasks = []
            for task in eligible_tasks:
                task_target = self.nav_graph.get_node_by_id(task.target_id)
                distance = self.nav_graph.min_distances[start][task_target]

                if distance < min_distance:
                    min_distance = distance
                    closest_tasks = [task]
                elif distance == min_distance:
                    closest_tasks.append(task)

            eligible_tasks = [
                task for task in eligible_tasks if task not in closest_tasks
            ]

            self.task_queue.extend(closest_tasks)
            closest_tasks_ids = [task.id for task in closest_tasks]
            new_eligible_tasks = [
                task
                for task in non_eligible_tasks
                if task.requires_task_id in closest_tasks_ids
            ]

            for task in new_eligible_tasks:
                non_eligible_tasks.remove(task)
                eligible_tasks.append(task)

            start = self.nav_graph.get_node_by_id(closest_tasks[0].target_id)
