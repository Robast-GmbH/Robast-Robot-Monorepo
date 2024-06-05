from task_assignment_system.models.node import Node
from task_assignment_system.models.task import Task
from task_assignment_system.models.nav_graph import NavGraph
import uuid
import threading
import requests

UPDATE_TIMER_INTERVAL_IN_SECONDS = 1


class Robot:
    def __init__(
        self, name, api_endpoint, modules, initial_node: Node, nav_graph: NavGraph
    ) -> None:
        self.name = name
        self.api_endpoint = api_endpoint
        self.modules = modules
        self.current_task = None
        self.current_task_id = None
        self.task_queue = []
        self.current_node = initial_node
        self.nav_graph = nav_graph
        self.done_tasks_ids = []
        self.task_queue_lock = threading.Lock()
        print(
            f"Robot {self.name} initialized at node {self.current_node.id} -> Starting task status update timer"
        )
        self.__start_task_status_update_timer()

    def get_request_cost(self, delivery_request):
        # check if any drawer of required drawer_type is available
        is_drawer_available = False
        for drawer in self.modules:
            if (
                drawer.size == delivery_request.required_drawer_type
                and drawer.is_available
            ):
                is_drawer_available = True
                break

        if not is_drawer_available:
            return float("inf")

        return len(self.task_queue)

    def accept_request(self, delivery_request):
        with self.task_queue_lock:
            unique_id = str(uuid.uuid4())
            drawer = self.__reserve_drawer(delivery_request.required_drawer_type)

            pick_up_task = Task(
                f"{unique_id}_pick_up",
                "pickup",
                None,
                drawer,
                delivery_request.start,
            )
            drop_off_task = Task(
                f"{unique_id}_drop_off",
                "dropoff",
                pick_up_task.id,
                drawer,
                delivery_request.target,
            )

            self.task_queue.append(pick_up_task)
            self.task_queue.append(drop_off_task)
            self.__optimize_task_queue()

    def get_robot_tasks(self):
        if self.current_task is not None:
            return {
                "active": str(self.current_task),
                "queued": [str(task) for task in self.task_queue],
            }
        return [str(task) for task in self.task_queue]

    def __start_task_status_update_timer(self):
        self.timer = threading.Timer(
            UPDATE_TIMER_INTERVAL_IN_SECONDS, self.__update_task_status_callback
        )
        self.timer.start()

    def __update_task_status_callback(self):
        with self.task_queue_lock:
            if self.current_task is None:
                self.__start_next_task()
            else:
                data = self.__request_task_status()
                if data is not None:
                    completed_phases = data["completed"]
                    active_phase = data["active"]
                    if active_phase in completed_phases:
                        self.done_tasks_ids.append(self.current_task.id)
                        if self.current_task.task_type == "dropoff":
                            self.current_task.drawer.is_available = True
                        self.current_task = None

        self.__start_task_status_update_timer()

    def __request_task_status(self):
        try:
            response = requests.get(
                f"{self.api_endpoint}/tasks/{self.current_task_id}/state"
            )
            response.raise_for_status()
        except:
            return None
        data = response.json()
        return data

    def __start_next_task(self):
        if len(self.task_queue) > 0:
            self.current_task = self.task_queue.pop(0)
            result = requests.post(
                f"{self.api_endpoint}/tasks/robot_task",
                json=self.current_task.to_json(),
            )
            print(result.json())
            assigned_id = result.json()["state"]["booking"]["id"]
            self.current_task_id = assigned_id

    def __optimize_task_queue(self):
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
            start = self.current_task.target

        while len(eligible_tasks) > 0:
            min_distance = float("inf")
            closest_tasks = []
            for task in eligible_tasks:
                distance = self.nav_graph.min_distances[start][task.target]

                if distance < min_distance:
                    min_distance = distance
                    closest_tasks = [task]
                elif distance == min_distance:
                    closest_tasks.append(task)

            eligible_tasks = [
                task for task in eligible_tasks if task not in closest_tasks
            ]

            def sort_drop_off_first(task):
                return task.task_type != "dropoff"

            closest_tasks.sort(key=sort_drop_off_first)
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

            start = closest_tasks[0].target

    def __reserve_drawer(self, drawer_type):
        for drawer in self.modules:
            if drawer.size == drawer_type and drawer.is_available:
                drawer.is_available = False
                return drawer
