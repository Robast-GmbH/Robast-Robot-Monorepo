import time
from pydantic_models.sub_task import SubTask
from pydantic_models.submodule_address import SubmoduleAddress
from task_system.models.node import Node
from task_system.models.fleet_management_api import FleetManagementAPI
from task_system.models.nav_graph import NavGraph
from pydantic_models.task import Task
from module_manager.module_manager import ModuleManager
from task_system.task_manager import TaskManager
from configs.url_config import FLEET_MANAGEMENT_ADDRESS
from typing import Dict, Any
import threading


class Robot:
    UPDATE_TIMER_INTERVAL_IN_S = 1
    TASK_RECEIVE_WINDOW_IN_S = 5

    def __init__(
        self,
        name: str,
        initial_node: Node,
        nav_graph: NavGraph,
    ) -> None:
        self.__name = name
        self.__fleet_management_api = FleetManagementAPI(FLEET_MANAGEMENT_ADDRESS)
        self.__current_node = initial_node
        self.__nav_graph = nav_graph
        self.__submodule_manager = ModuleManager()
        self.__task_manager = TaskManager()
        self.__current_subtask_id = None
        self.__current_subtask_fm_id = None
        self.__subtask_queue: list[str] = []
        self.__done_subtasks_ids: list[str] = []
        self.__subtask_queue_lock = threading.Lock()

        print(
            f"Robot {self.__name} initialized at node {self.__current_node.id} -> Starting task status update timer"
        )
        self.__timer = None
        self.__start_task_status_update_timer()

    def get_subtasks(self) -> Dict[str, Any]:
        subtasks = {}
        if self.__current_subtask_id:
            subtasks["active"] = self.__task_manager.read_subtask(
                self.__current_subtask_id
            )
        else:
            subtasks["active"] = None
        subtasks["queue"] = self.__task_manager.read_subtasks_by_subtask_ids(
            self.__subtask_queue
        )
        return subtasks

    def get_request_cost(self, task: Task) -> float:
        if "required_submodule_type" in task.requirements:
            is_submodule_available = (
                self.__submodule_manager.is_submodule_size_available(
                    self.__name,
                    int(task.requirements["required_submodule_type"]),
                )
            )
            if not is_submodule_available:
                return float("inf")

        return float(len(self.__subtask_queue))

    def accept_assigned_task(self, task: Task) -> bool:
        self.__open_task_receive_window()
        if self.__handle_requirements(task):
            self.__task_manager.assign_task(task.id, self.__name)
            self.__enqueue_task(task)
            return True
        return False

    def accept_direct_task(self, task: Task) -> None:
        self.__open_task_receive_window()
        self.__enqueue_task(task)

    def is_submodule_type_mounted(self, submodule_type: int) -> bool:
        return self.__submodule_manager.is_submodule_type_mounted(
            self.__name, submodule_type
        )

    def __open_task_receive_window(self) -> None:
        if self.__timer:
            self.__timer.cancel()
            self.__timer = threading.Timer(
                self.TASK_RECEIVE_WINDOW_IN_S, self.__task_status_update_callback
            )
            self.__timer.start()

    def __handle_requirements(self, task: Task) -> bool:
        if "required_submodule_type" in task.requirements:
            submodule = self.__submodule_manager.try_reserve_submodule_type(
                self.__name,
                task.requirements["required_submodule_type"],
                task.id,
                [],
                [],
            )
            if not submodule:
                return False
            subtasks_with_submodule_process = [
                subtask
                for subtask in task.subtasks
                if subtask.contains_submodule_process_action()
            ]
            for subtask in subtasks_with_submodule_process:
                subtask.write_submodule_address(submodule.address)
                self.__task_manager.update_subtask(subtask)
        return True

    def __handle_subtask_requirements(self, subtask: SubTask) -> bool:
        if subtask.requirements["submodule_address"]:
            was_successful = self.__submodule_manager.reserve_submodule(
                submodule_address=SubmoduleAddress.from_json(
                    subtask.requirements["submodule_address"]
                ),
                task_id=subtask.parent_id,
                user_ids=subtask.requirements["required_user_ids"],
                user_groups=subtask.requirements["required_user_groups"],
            )
            if not was_successful:
                return False
        return True

    def __start_task_status_update_timer(self) -> None:
        self.__timer = threading.Timer(
            self.UPDATE_TIMER_INTERVAL_IN_S, self.__task_status_update_callback
        )
        self.__timer.start()

    def __task_status_update_callback(self) -> None:
        with self.__subtask_queue_lock:
            if self.__current_subtask_id is None and self.__subtask_queue:
                self.__start_next_task()
            elif self.__is_task_completed():
                self.__finish_task()

        self.__start_task_status_update_timer()

    def __is_task_completed(self) -> bool:
        if self.__current_subtask_fm_id is None:
            return False
        data = self.__fleet_management_api.request_task_status(
            self.__current_subtask_fm_id
        )
        if data is None:
            return False
        completed_phases = data["completed"]
        active_phase = data["active"]
        if completed_phases and active_phase:
            return active_phase in completed_phases
        return False

    def __finish_task(self) -> None:
        if self.__current_subtask_id is None:
            return
        current_task = self.__task_manager.read_subtask(self.__current_subtask_id)
        if not current_task:
            return
        self.__current_node = self.__nav_graph.get_node_by_id(current_task.target_id)
        self.__done_subtasks_ids.append(current_task.id)
        queued_subtasks = self.__task_manager.read_subtasks_by_subtask_ids(
            self.__subtask_queue
        )
        related_tasks = [
            task for task in queued_subtasks if task.parent_id == current_task.parent_id
        ]
        related_submodule_tasks = [
            task for task in related_tasks if task.contains_submodule_process_action()
        ]
        self.__task_manager.finish_subtask(current_task.parent_id, current_task.id)
        if related_submodule_tasks:
            self.__submodule_manager.reserve_submodule(
                SubmoduleAddress.from_json(
                    related_submodule_tasks[0].requirements["submodule_address"]
                ),
                current_task.parent_id,
                [],
                [],
            )
        elif current_task.contains_submodule_process_action():
            self.__submodule_manager.free_submodule(
                SubmoduleAddress.from_json(
                    current_task.requirements["submodule_address"]
                )
            )

        if not related_tasks:
            self.__task_manager.finish_task(current_task.parent_id)

        self.__current_subtask_id = None

    def __start_next_task(self) -> None:
        timeStamp = int(time.time())
        self.__optimize_task_queue()
        next_subtask = self.__task_manager.read_subtask(self.__subtask_queue[0])
        if not next_subtask:
            return
        if next_subtask.earliest_start_time > timeStamp:
            return
        if not self.__handle_subtask_requirements(next_subtask):
            return
        result = self.__fleet_management_api.dispatch_robot_task(
            next_subtask.to_robot_task_request()
        )
        if result is not None:
            print(result)
            assigned_id = result["state"]["booking"]["id"]
            self.__current_subtask_fm_id = assigned_id
            self.__current_subtask_id = self.__subtask_queue.pop(0)
            self.__task_manager.start_subtask(self.__current_subtask_id)

    def __enqueue_task(self, task: Task) -> None:
        with self.__subtask_queue_lock:
            self.__subtask_queue.extend([subtask.id for subtask in task.subtasks])
            self.__optimize_task_queue()

    def __optimize_task_queue(self) -> None:
        queued_subtasks = self.__task_manager.read_subtasks_by_subtask_ids(
            self.__subtask_queue
        )
        self.__subtask_queue.clear()

        startable_tasks, non_startable_tasks = self.__partition_tasks_by_startability(
            queued_subtasks
        )
        eligible_tasks, non_eligible_tasks = self.__partition_tasks_by_eligibility(
            startable_tasks
        )

        start = self.__determine_start_node()
        while len(eligible_tasks) > 0:
            closest_tasks = self.__find_closest_tasks(eligible_tasks, start)
            eligible_tasks = [
                task for task in eligible_tasks if task not in closest_tasks
            ]
            self.__subtask_queue.extend([subtask.id for subtask in closest_tasks])

            new_eligible_tasks = self.__find_new_eligible_tasks(
                closest_tasks, non_eligible_tasks
            )
            eligible_tasks.extend(new_eligible_tasks)

            start = self.__nav_graph.get_node_by_id(closest_tasks[0].target_id)

        self.__subtask_queue.extend([subtask.id for subtask in non_startable_tasks])

    def __partition_tasks_by_startability(
        self, queued_subtasks: list[SubTask]
    ) -> tuple[list[SubTask], list[SubTask]]:
        startable_tasks = [
            task
            for task in queued_subtasks
            if task.earliest_start_time <= int(time.time())
        ]
        non_startable_tasks = [
            task for task in queued_subtasks if task not in startable_tasks
        ]
        return startable_tasks, non_startable_tasks

    def __partition_tasks_by_eligibility(
        self, queued_subtasks: list[SubTask]
    ) -> tuple[list[SubTask], list[SubTask]]:

        eligible_tasks = [
            task
            for task in queued_subtasks
            if task.requires_task_id is None
            or task.requires_task_id in self.__done_subtasks_ids
            or (
                self.__current_subtask_id is not None
                and task.requires_task_id == self.__current_subtask_id
            )
        ]
        non_eligible_tasks = [
            task for task in queued_subtasks if task not in eligible_tasks
        ]
        return eligible_tasks, non_eligible_tasks

    def __determine_start_node(self) -> Node:
        if self.__current_subtask_id:
            current_task = self.__task_manager.read_subtask(self.__current_subtask_id)
            if current_task:
                return self.__nav_graph.get_node_by_id(current_task.target_id)
        return self.__current_node

    def __find_closest_tasks(
        self, eligible_tasks: list[SubTask], start_node: Node
    ) -> list[SubTask]:
        min_distance = float("inf")
        closest_tasks = []

        for task in eligible_tasks:
            task_target = self.__nav_graph.get_node_by_id(task.target_id)
            distance = self.__nav_graph.min_distances[start_node][task_target]

            if distance < min_distance:
                min_distance = distance
                closest_tasks = [task]
            elif distance == min_distance:
                closest_tasks.append(task)

        return closest_tasks

    def __find_new_eligible_tasks(
        self, closest_tasks: list[SubTask], non_eligible_tasks: list[SubTask]
    ) -> list[SubTask]:
        closest_tasks_ids = [task.id for task in closest_tasks]
        new_eligible_tasks = [
            task
            for task in non_eligible_tasks
            if task.requires_task_id in closest_tasks_ids
        ]

        for task in new_eligible_tasks:
            non_eligible_tasks.remove(task)

        return new_eligible_tasks
