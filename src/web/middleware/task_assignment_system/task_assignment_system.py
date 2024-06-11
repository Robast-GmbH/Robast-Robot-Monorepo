from task_assignment_system.models.robot import Robot
from task_assignment_system.models.drawer import Drawer
from task_assignment_system.models.nav_graph import NavGraph
from task_assignment_system.models.delivery_request import DeliveryRequest

from threading import Timer
import requests
import time
import json

TASK_ASSIGNMENT_TRIGGER_INTERVAL_IN_SECONDS = 10


class TaskAssignmentSystem:
    def __init__(
        self,
        fleet_ip_config: dict[str, str],
        robot_api_port: str,
        fleet_management_address: str,
    ) -> None:
        path = "task_assignment_system/configs/nav_config.json"
        with open(path, "r") as file:
            data = json.load(file)["levels"][0]["nav_graphs"][0]
        self.nav_graph = NavGraph.from_json(data=data)
        self.available_drawer_types = set()

        self.robots = self.__init_robots(
            fleet_ip_config, robot_api_port, fleet_management_address
        )

        self.request_queue = []
        self.timer = None

    def receive_request(
        self,
        required_drawer_type: int,
        start_id: str = None,
        target_id: str = None,
    ) -> str:
        if self.timer is not None:
            self.timer.cancel()
        start = self.nav_graph.get_node_by_id(start_id)
        target = self.nav_graph.get_node_by_id(target_id)

        if target is None:
            return "Invalid request, target node not found."
        if required_drawer_type not in self.available_drawer_types:
            return "Invalid request, drawer type not available."

        delivery_request = DeliveryRequest(
            required_drawer_type,
            start=start,
            target=target,
        )
        self.request_queue.append(delivery_request)
        self.__trigger_task_assignment()
        return "Request added to queue."

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
        fleet_ip_config: dict[str, str],
        robot_api_port: str,
        fleet_management_address: str,
    ) -> list[Robot]:
        robots = {}
        for robot_name in fleet_ip_config.keys():
            modules_json = None
            while modules_json is None:
                try:
                    modules_json = requests.get(
                        f"http://{fleet_ip_config[robot_name]}:{robot_api_port}/modules?robot_name={robot_name}"
                    ).json()
                except requests.exceptions.ConnectionError:
                    time.sleep(1)
                    print("Retrying to get modules")

            modules = [Drawer.from_dict(module_dict) for module_dict in modules_json]
            self.available_drawer_types.update([drawer.size for drawer in modules])

            robots[robot_name] = Robot(
                name=robot_name,
                api_endpoint=fleet_management_address,
                modules=modules,
                initial_node=self.nav_graph.nodes[15],
                nav_graph=self.nav_graph,
            )

        return robots
