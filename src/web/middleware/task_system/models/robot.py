from pydantic_models.sub_task import SubTask
from pydantic_models.drawer_address import DrawerAddress
from task_system.models.node import Node
from task_system.models.fleet_management_api import FleetManagementAPI
from task_system.models.nav_graph import NavGraph
from pydantic_models.task import Task
from module_manager.module_manager import ModuleManager
from task_system.task_manager import TaskManager
from configs.url_config import FLEET_MANAGEMENT_ADDRESS
import threading


class Robot:
    UPDATE_TIMER_INTERVAL_IN_S = 1

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
        self.__module_manager = ModuleManager()
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

    def get_request_cost(self, task: Task) -> float:
        if "required_drawer_type" in task.requirements:
            is_drawer_available = self.__module_manager.is_module_size_available(
                self.__name,
                int(task.requirements["required_drawer_type"]),
            )
            if not is_drawer_available:
                return float("inf")

        return float(len(self.__subtask_queue))

    def accept_task(self, task: Task) -> None:
        if not self.__handle_requirements(task):
            return

        self.__task_manager.assign_task(task.id, self.__name)

        with self.__subtask_queue_lock:
            self.__subtask_queue.extend([subtask.id for subtask in task.subtasks])
            self.__optimize_task_queue()

    def is_drawer_type_mounted(self, drawer_type: int) -> bool:
        return self.__module_manager.is_drawer_type_mounted(self.__name, drawer_type)

    def __handle_requirements(self, task: Task) -> bool:
        if "required_drawer_type" in task.requirements:
            drawer = self.__module_manager.try_reserve_module_type(
                self.__name, task.requirements["required_drawer_type"], task.id, [], []
            )
            if not drawer:
                return False
            subtasks_with_drawer_process = [
                subtask
                for subtask in task.subtasks
                if subtask.contains_drawer_process_action()
            ]
            for subtask in subtasks_with_drawer_process:
                subtask.write_drawer_address(drawer.address)
                self.__task_manager.update_subtask(subtask)
            self.__handle_subtask_requirements(subtasks_with_drawer_process[0])
        return True

    def __handle_subtask_requirements(self, subtask: SubTask) -> bool:
        if subtask.requirements["drawer_address"]:
            was_successful = self.__module_manager.reserve_module(
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
        self.__done_subtasks_ids.append(current_task.id)
        queued_subtasks = self.__task_manager.read_subtasks_by_subtask_ids(
            self.__subtask_queue
        )
        related_tasks = [
            task for task in queued_subtasks if task.parent_id == current_task.parent_id
        ]
        related_drawer_tasks = [
            task for task in related_tasks if task.contains_drawer_process_action()
        ]
        self.__task_manager.finish_subtask(current_task.parent_id, current_task.id)
        if related_drawer_tasks:
            self.__handle_subtask_requirements(related_drawer_tasks[0])
        elif current_task.contains_drawer_process_action():
            self.__module_manager.free_module(
                DrawerAddress.from_json(current_task.requirements["drawer_address"])
            )

        if not related_tasks:
            self.__task_manager.finish_task(current_task.parent_id)

        self.__current_subtask_id = None

    def __start_next_task(self) -> None:
        print(self.__subtask_queue)
        next_subtask = self.__task_manager.read_subtask(self.__subtask_queue[0])
        if not next_subtask:
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

    def __optimize_task_queue(self) -> None:
        queued_subtasks = self.__task_manager.read_subtasks_by_subtask_ids(
            self.__subtask_queue
        )
        self.__subtask_queue.clear()

        eligible_tasks, non_eligible_tasks = self.__filter_tasks(queued_subtasks)

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

    def __filter_tasks(
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
