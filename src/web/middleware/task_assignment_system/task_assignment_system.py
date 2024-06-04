from task_assignment_system.models.robot import Robot
from task_assignment_system.models.module import Module
from task_assignment_system.models.nav_graph import NavGraph
from task_assignment_system.models.delivery_request import DeliveryRequest

from threading import Timer
import requests
import time


class TaskAssignmentSystem:
    def __init__(self, fleet_ip_config, robot_api_port, fleet_management_address):
        self.nav_graph = NavGraph.from_json()

        self.robots = []
        for robot_name in fleet_ip_config.keys():
            modules_json = None
            while modules_json is None:
                try:
                    modules_json = requests.get(
                        f"http://{fleet_ip_config[robot_name]}:{robot_api_port}/modules?robot_name={robot_name}"
                    ).json()
                except:
                    time.sleep(1)
                    print("Retrying to get modules")

            modules = Module.module_setup_from_json(modules_json)

            self.robots.append(
                Robot(
                    name=robot_name,
                    api_endpoint=fleet_management_address,
                    modules=modules,
                    initial_node=self.nav_graph.nodes[15],
                    nav_graph=self.nav_graph,
                )
            )

        print("Robots initialized")

    def receive_task(
        self, required_drawer_type: int, start_id: str = None, target_id: str = None
    ):
        start = self.nav_graph.get_node_by_id(start_id)
        target = self.nav_graph.get_node_by_id(target_id)
        if start is None or target is None:
            return None

        delivery_request = DeliveryRequest(
            required_drawer_type,
            start=start,
            target=target,
        )

        costs = []
        for robot in self.robots:
            costs.append(robot.get_cost(delivery_request))
        min_cost = min(costs)

        if min_cost == float("inf"):
            Timer(
                10, self.receive_task, [required_drawer_type, start_id, target_id]
            ).start()
            return (
                "Task currently not feasible, retrying every 10 seconds until feasible."
            )

        self.robots[costs.index(min_cost)].take_task(delivery_request)

        return {
            "Task Start": start.index,
            "Task Target": target.index,
            "Required Drawer Type": required_drawer_type,
        }
